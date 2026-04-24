#include "camera_uart.hpp"

#include <cstdio>
#include <cstring>
#include <algorithm>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_dvp.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "hal/cam_ll.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_pins.h"
#include "soc/gpio_sig_map.h"

#include "app_config.h"
#include "bsp.h"
#include "gc032a_regs.hpp"

namespace {
constexpr const char *TAG = "camera_uart";
constexpr uart_port_t kUart = UART_NUM_2;
constexpr ledc_mode_t kMclkSpeedMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kMclkTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kMclkChannel = LEDC_CHANNEL_0;
constexpr uint8_t kFrameStart = 0xAB;
constexpr uint8_t kLineStart  = 0x80;
constexpr uint8_t kLineEnd    = 0x9D;
constexpr uint8_t kFrameEnd   = 0xB6;
constexpr uint32_t kBt656SyncPrefix = 0xFF000000UL;
constexpr uint32_t kFrameStartSync = kBt656SyncPrefix | kFrameStart;
constexpr uint32_t kLineStartSync  = kBt656SyncPrefix | kLineStart;
constexpr uint32_t kLineEndSync    = kBt656SyncPrefix | kLineEnd;
constexpr uint32_t kFrameEndSync   = kBt656SyncPrefix | kFrameEnd;
constexpr uint8_t kGc032aIdHighReg = 0xF0;
constexpr uint8_t kGc032aIdLowReg  = 0xF1;
constexpr uint16_t kGc032aExpectedId = 0x232A;

// 32 KB raw window = 32K PCLK edges. This captures a short slice of the
// GC032A's 640x480 2-bit stream for bring-up preview and sync diagnostics.
constexpr size_t kRawCapacityBytes = 4U * 1024U;
constexpr uint32_t kBurstPollBudget = 32U * 1000U;
constexpr uint64_t kCaptureMaxUs = 250U * 1000U;
constexpr uint64_t kFullFrameMaxUs = 12U * 1000U * 1000U;
constexpr uint64_t kRawFrameMaxUs = 4500U * 1000U;
constexpr uint32_t kRawFrameYieldEveryBursts = 16U;
constexpr TickType_t kCameraYieldTicks = 1;
constexpr size_t kDvpProbeRawBytes = 4U * 1024U;
constexpr size_t kDvpProbePackedBytes = kDvpProbeRawBytes / 4U;
constexpr uint32_t kDvpProbeWindowsPerLog = 64U;
constexpr uint32_t kSensorWidth = APP_CAMERA_SENSOR_WIDTH;
constexpr uint32_t kSensorHeight = APP_CAMERA_SENSOR_HEIGHT;
constexpr size_t kSensorRowBytes = kSensorWidth * 2U;
constexpr size_t kSensorFrameBytes = kSensorRowBytes * kSensorHeight;
constexpr uint32_t kPreviewWidth = APP_CAMERA_PREVIEW_WIDTH;
constexpr uint32_t kPreviewHeight = APP_CAMERA_PREVIEW_HEIGHT;
constexpr size_t kPreviewBytes = kPreviewWidth * kPreviewHeight * 2U;
constexpr size_t kPreviewRowBytes = kPreviewWidth * 2U;
static_assert(kPreviewWidth <= 0xFFFFU && kPreviewHeight <= 0xFFFFU,
              "GC032A SPI output dimensions must fit 16-bit registers");
static_assert(kSensorWidth % 2U == 0, "YVYU width must be even");

enum class FrameParseState {
    SeekFrameStart,
    SeekLineStart,
    Payload,
    ExpectLineEnd,
};

struct FrameAssembler {
    FrameParseState state = FrameParseState::SeekFrameStart;
    uint8_t *out = nullptr;
    size_t out_capacity = 0;
    size_t out_len = 0;
    size_t line_pos = 0;
    size_t sync_wait_bytes = 0;
    uint32_t sync_window = 0;
    uint32_t lines = 0;
    uint32_t frame_syncs = 0;
    uint32_t line_start_syncs = 0;
    uint32_t line_end_syncs = 0;
    uint32_t frame_end_syncs = 0;
    uint32_t restarts = 0;
    uint32_t bad_line_end = 0;
    uint32_t last_bad_line_end = 0;
    int32_t first_frame_start = -1;
    bool done = false;

    void reset_frame()
    {
        state = FrameParseState::SeekFrameStart;
        out_len = 0;
        line_pos = 0;
        sync_wait_bytes = 0;
        lines = 0;
    }

    void finish_line()
    {
        ++lines;
        if (lines >= kSensorHeight || (out_capacity > 0 && out_len >= out_capacity)) {
            done = true;
            return;
        }
        state = FrameParseState::SeekLineStart;
    }

    void feed(uint8_t b, size_t packed_offset)
    {
        if (done) return;
        sync_window = (sync_window << 8U) | b;
        if (sync_window == kFrameStartSync) ++frame_syncs;
        if (sync_window == kLineStartSync) ++line_start_syncs;
        if (sync_window == kLineEndSync) ++line_end_syncs;
        if (sync_window == kFrameEndSync) ++frame_end_syncs;

        switch (state) {
        case FrameParseState::SeekFrameStart:
            if (sync_window == kFrameStartSync) {
                if (first_frame_start < 0) {
                    first_frame_start = static_cast<int32_t>(packed_offset) - 3;
                }
                out_len = 0;
                line_pos = 0;
                lines = 0;
                state = FrameParseState::SeekLineStart;
            }
            break;

        case FrameParseState::SeekLineStart:
            if (sync_window == kLineStartSync) {
                line_pos = 0;
                state = FrameParseState::Payload;
            } else if (sync_window == kFrameStartSync && out_len == 0) {
                ++restarts;
                out_len = 0;
                line_pos = 0;
                lines = 0;
            } else if (sync_window == kFrameEndSync && lines > 0) {
                done = true;
            }
            break;

        case FrameParseState::Payload:
            if (out_len < out_capacity) {
                if (out) {
                    out[out_len] = b;
                }
                ++out_len;
            }
            if (++line_pos >= kSensorRowBytes) {
                sync_wait_bytes = 0;
                state = FrameParseState::ExpectLineEnd;
            }
            break;

        case FrameParseState::ExpectLineEnd:
            ++sync_wait_bytes;
            if (sync_window == kLineEndSync) {
                finish_line();
            } else if (sync_wait_bytes >= 4U) {
                ++bad_line_end;
                last_bad_line_end = sync_window;
                ++restarts;
                reset_frame();
                if (sync_window == kFrameStartSync) {
                    state = FrameParseState::SeekLineStart;
                }
            }
            break;
        }
    }
};

enum PacketType : uint8_t {
    PKT_STATUS      = 1,
    PKT_FRAME_START = 2,
    PKT_FRAME_DATA  = 3,
    PKT_FRAME_END   = 4,
    PKT_DEBUG_HEX   = 5,
};

struct __attribute__((packed)) PacketHeader {
    char     magic[4];
    uint8_t  type;
    uint8_t  flags;
    uint16_t header_len;
    uint32_t seq;
    uint32_t payload_len;
    uint32_t checksum;
};

struct DvpProbeDone {
    uint8_t *buffer = nullptr;
    size_t received = 0;
};

struct DvpProbeContext {
    uint8_t *buffers[2] = {};
    size_t buffer_len = 0;
    uint32_t next = 0;
    QueueHandle_t done_queue = nullptr;
};

uint32_t fnv1a(const void *data, size_t len)
{
    const auto *p = static_cast<const uint8_t *>(data);
    uint32_t h = 2166136261UL;
    for (size_t i = 0; i < len; ++i) {
        h ^= p[i];
        h *= 16777619UL;
    }
    return h;
}

bool IRAM_ATTR dvp_probe_get_new_trans(esp_cam_ctlr_handle_t,
                                       esp_cam_ctlr_trans_t *trans,
                                       void *user_data)
{
    auto *ctx = static_cast<DvpProbeContext *>(user_data);
    uint32_t idx = ctx->next++ & 0x01U;
    trans->buffer = ctx->buffers[idx];
    trans->buflen = ctx->buffer_len;
    return false;
}

bool IRAM_ATTR dvp_probe_trans_finished(esp_cam_ctlr_handle_t,
                                        esp_cam_ctlr_trans_t *trans,
                                        void *user_data)
{
    auto *ctx = static_cast<DvpProbeContext *>(user_data);
    DvpProbeDone done = {
        .buffer = static_cast<uint8_t *>(trans->buffer),
        .received = trans->received_size,
    };
    BaseType_t task_woken = pdFALSE;
    xQueueSendFromISR(ctx->done_queue, &done, &task_woken);
    return task_woken == pdTRUE;
}

struct SyncCounts {
    uint32_t fs = 0;
    uint32_t ls = 0;
    uint32_t le = 0;
    uint32_t fe = 0;
};

// GC032A 2-wire SPI output puts a full independent BT.656 byte stream on EACH
// of D0 and D1 (not bits of a single stream split across lanes). So `FF 00 00
// XX` on D1 means 8 PCLKs of D1=1 then 16 PCLKs of D1=0 then 8 bits of sync
// byte XX on D1, regardless of what D0 is doing. Scan each lane independently.
struct RawSyncCounts {
    // Counts per lane. `ls` counts a match for any of the four sync markers at
    // any MSB-first or LSB-first bit order.
    uint32_t d0_prefix = 0, d1_prefix = 0;
    uint32_t d0_fs = 0, d0_ls = 0, d0_le = 0, d0_fe = 0;
    uint32_t d1_fs = 0, d1_ls = 0, d1_le = 0, d1_fe = 0;
    int32_t  d0_first_fs = -1, d1_first_fs = -1;
    uint32_t d0_max_one_run = 0, d0_max_zero_run = 0;
    uint32_t d1_max_one_run = 0, d1_max_zero_run = 0;
};

static uint8_t reverse_byte(uint8_t b)
{
    b = static_cast<uint8_t>((b >> 4) | (b << 4));
    b = static_cast<uint8_t>(((b & 0xCCU) >> 2) | ((b & 0x33U) << 2));
    b = static_cast<uint8_t>(((b & 0xAAU) >> 1) | ((b & 0x55U) << 1));
    return b;
}

// Check if the 8 bits of `lane_stream` starting at bit offset `off` match any
// known sync byte (AB / 80 / 9D / B6) in either MSB-first or LSB-first order.
// `update` receives which sync matched (0=fs, 1=ls, 2=le, 3=fe) or -1.
static int classify_sync_tail(uint32_t bit_offset, const uint8_t *pairs, size_t n,
                              bool lane_d1)
{
    if (bit_offset + 8 > n) return -1;
    uint8_t v = 0;
    // Collect 8 consecutive bits from the chosen lane, MSB-first order (i.e.,
    // the bit sampled earliest ends up in bit 7 of v).
    for (uint32_t k = 0; k < 8; ++k) {
        uint8_t p = pairs[bit_offset + k];
        uint8_t lane_bit = lane_d1 ? ((p >> 1) & 1U) : (p & 1U);
        v = static_cast<uint8_t>((v << 1) | lane_bit);
    }
    uint8_t v_rev = reverse_byte(v);
    auto hit = [&](uint8_t expected) { return v == expected || v_rev == expected; };
    if (hit(kFrameStart)) return 0;
    if (hit(kLineStart))  return 1;
    if (hit(kLineEnd))    return 2;
    if (hit(kFrameEnd))   return 3;
    return -1;
}

// Extract a single lane (D0 if `d1`==false, D1 otherwise) from `pairs` and
// pack into MSB-first bytes. Returns how many bytes were written.
size_t extract_lane_bytes(const uint8_t *pairs, size_t n_pairs,
                          bool d1, uint8_t *out, size_t out_capacity)
{
    size_t out_len = 0;
    uint8_t acc = 0;
    uint8_t bit_count = 0;
    for (size_t i = 0; i < n_pairs && out_len < out_capacity; ++i) {
        uint8_t b = d1 ? ((pairs[i] >> 1) & 1U) : (pairs[i] & 1U);
        acc = static_cast<uint8_t>((acc << 1) | b);
        if (++bit_count == 8) {
            out[out_len++] = acc;
            acc = 0;
            bit_count = 0;
        }
    }
    return out_len;
}

RawSyncCounts scan_raw_pairs_for_sync(const uint8_t *pairs, size_t n)
{
    RawSyncCounts c;
    if (n < 32) return c;

    // Track consecutive runs per lane.
    uint32_t d0_zero = 0, d0_one = 0, d1_zero = 0, d1_one = 0;

    // Sliding window for 8-ones + 16-zeros prefix on each lane, using shift
    // registers of the last 24 bits. We compare against:
    //   d0_prefix_match: low 24 bits of d0_window == 0xFF0000
    //   same for d1.
    uint32_t d0_win = 0, d1_win = 0;
    constexpr uint32_t kPrefixMask = 0x00FFFFFFU;  // 24 bits
    constexpr uint32_t kPrefixVal  = 0x00FF0000U;  // 8 ones then 16 zeros

    for (size_t i = 0; i < n; ++i) {
        uint8_t p = pairs[i];
        uint8_t b0 = p & 1U;
        uint8_t b1 = (p >> 1) & 1U;

        // Runs
        if (b0) { if (++d0_one > c.d0_max_one_run) c.d0_max_one_run = d0_one; d0_zero = 0; }
        else    { if (++d0_zero > c.d0_max_zero_run) c.d0_max_zero_run = d0_zero; d0_one = 0; }
        if (b1) { if (++d1_one > c.d1_max_one_run) c.d1_max_one_run = d1_one; d1_zero = 0; }
        else    { if (++d1_zero > c.d1_max_zero_run) c.d1_max_zero_run = d1_zero; d1_one = 0; }

        // Sliding windows
        d0_win = ((d0_win << 1) | b0) & kPrefixMask;
        d1_win = ((d1_win << 1) | b1) & kPrefixMask;

        // On a new prefix match at this bit index, the prefix's last bit
        // landed at position i. The 8-bit sync tail starts at i+1 and spans
        // pairs[i+1 .. i+8]. Need at least 8 more pairs to classify.
        if (d0_win == kPrefixVal) {
            ++c.d0_prefix;
            int kind = classify_sync_tail(static_cast<uint32_t>(i + 1), pairs, n, /*lane_d1=*/false);
            switch (kind) {
            case 0: ++c.d0_fs; if (c.d0_first_fs < 0) c.d0_first_fs = static_cast<int32_t>(i - 23); break;
            case 1: ++c.d0_ls; break;
            case 2: ++c.d0_le; break;
            case 3: ++c.d0_fe; break;
            default: break;
            }
        }
        if (d1_win == kPrefixVal) {
            ++c.d1_prefix;
            int kind = classify_sync_tail(static_cast<uint32_t>(i + 1), pairs, n, /*lane_d1=*/true);
            switch (kind) {
            case 0: ++c.d1_fs; if (c.d1_first_fs < 0) c.d1_first_fs = static_cast<int32_t>(i - 23); break;
            case 1: ++c.d1_ls; break;
            case 2: ++c.d1_le; break;
            case 3: ++c.d1_fe; break;
            default: break;
            }
        }
    }
    return c;
}

SyncCounts count_bt656_syncs(const uint8_t *data, size_t len)
{
    SyncCounts counts;
    uint32_t window = 0;
    for (size_t i = 0; i < len; ++i) {
        window = (window << 8U) | data[i];
        if (window == kFrameStartSync) ++counts.fs;
        if (window == kLineStartSync) ++counts.ls;
        if (window == kLineEndSync) ++counts.le;
        if (window == kFrameEndSync) ++counts.fe;
    }
    return counts;
}

// IRAM-resident tight loop. Reads GPIO_IN_REG every iteration, latches D0/D1
// on each PCLK edge of the requested polarity, writes one pair (low 2 bits)
// per output byte. Bit layout: bit0 = D0 sample, bit1 = D1 sample.
//
// Stops when `pair_total` samples are collected or the short poll budget is
// exhausted. The caller repeats this in chunks and yields between chunks so a
// stuck or very slow PCLK cannot starve the idle task and trip the task WDT.
__attribute__((noinline, optimize("O3"), section(".iram1")))
static size_t spi2_cpu_burst(uint8_t *raw_pairs, size_t pair_total,
                             bool sample_falling, uint32_t poll_budget)
{
    volatile uint32_t * const gpio_in =
        reinterpret_cast<volatile uint32_t *>(GPIO_IN_REG);
    constexpr uint32_t pclk_mask = 1U << BSP_GC032A_SPI_CLK_GPIO; // GPIO12
    constexpr uint32_t d0_mask   = 1U << BSP_GC032A_DATA0_GPIO;   // GPIO6
    constexpr uint32_t d1_mask   = 1U << BSP_GC032A_DATA1_GPIO;   // GPIO7

    uint32_t prev_clk = *gpio_in & pclk_mask;
    size_t   idx      = 0;
    uint32_t budget   = poll_budget;

    while (idx < pair_total && budget--) {
        uint32_t v       = *gpio_in;
        uint32_t cur_clk = v & pclk_mask;
        bool edge        = sample_falling
                               ? (prev_clk != 0 && cur_clk == 0)
                               : (prev_clk == 0 && cur_clk != 0);
        prev_clk = cur_clk;
        if (edge) {
            uint8_t pair = static_cast<uint8_t>(((v & d0_mask) ? 0x01 : 0x00) |
                                                ((v & d1_mask) ? 0x02 : 0x00));
            raw_pairs[idx++] = pair;
        }
    }
    return idx;
}

// Pack `pair_count` raw 2-bit samples into bytes. `swap` swaps D0/D1 inside the
// pair; `lsb_first` controls whether the first PCLK sample lands in bits [1:0]
// of the packed byte (LSB) or bits [7:6] (MSB).
struct Spi2WirePacker {
    bool swap = false;
    bool lsb_first = false;
    uint8_t out_byte = 0;
    uint8_t phase = 0;

    void reset()
    {
        out_byte = 0;
        phase = 0;
    }

    uint8_t normalize_pair(uint8_t pair) const
    {
        pair &= 0x03U;
        if (swap) {
            pair = static_cast<uint8_t>(((pair & 0x01U) << 1) |
                                        ((pair & 0x02U) >> 1));
        }
        return pair;
    }

    size_t push(const uint8_t *raw_pairs, size_t pair_count,
                uint8_t *packed, size_t packed_capacity)
    {
        size_t out_len = 0;
        for (size_t i = 0; i < pair_count; ++i) {
            uint8_t pair = normalize_pair(raw_pairs[i]);
            uint32_t shift = lsb_first ? (phase * 2U) : ((3U - phase) * 2U);
            out_byte = static_cast<uint8_t>(out_byte | (pair << shift));
            if (++phase >= 4U) {
                if (out_len >= packed_capacity) {
                    reset();
                    return out_len;
                }
                packed[out_len++] = out_byte;
                reset();
            }
        }
        return out_len;
    }
};

size_t pack_pairs(const uint8_t *raw_pairs, size_t pair_count,
                  uint8_t *packed, bool swap, bool lsb_first)
{
    Spi2WirePacker packer = {
        .swap = swap,
        .lsb_first = lsb_first,
    };
    return packer.push(raw_pairs, pair_count, packed, pair_count / 4U);
}

struct DecodeStats {
    bool     swap;
    bool     lsb_first;
    uint32_t fs;          // 0xAB count
    uint32_t ls;          // 0x80
    uint32_t le;          // 0x9D
    uint32_t fe;          // 0xB6
    uint32_t fs_then_ls;  // adjacent (frame_start, line_start) hits
    int32_t  first_fs;    // byte offset of first 0xAB, -1 if none
    uint32_t score;
};

DecodeStats score_packed(const uint8_t *packed, size_t byte_count, bool swap, bool lsb)
{
    DecodeStats s = {};
    s.swap = swap;
    s.lsb_first = lsb;
    s.first_fs = -1;
    uint8_t prev = 0;
    for (size_t i = 0; i < byte_count; ++i) {
        uint8_t b = packed[i];
        if (b == kFrameStart) {
            ++s.fs;
            if (s.first_fs < 0) s.first_fs = static_cast<int32_t>(i);
        }
        if (b == kLineStart) ++s.ls;
        if (b == kLineEnd)   ++s.le;
        if (b == kFrameEnd)  ++s.fe;
        if (prev == kFrameStart && b == kLineStart) ++s.fs_then_ls;
        prev = b;
    }
    s.score = s.fs * 32U + s.fs_then_ls * 64U + s.ls * 4U + s.le * 4U + s.fe * 16U;
    return s;
}

} // namespace

esp_err_t CameraUartStreamer::init()
{
    if (initialized_) return ESP_OK;

    ESP_RETURN_ON_ERROR(start_mclk(), TAG, "mclk");

    uart_config_t cfg = {
        .baud_rate = APP_CAMERA_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(kUart, 2048, 0, 0, nullptr, 0), TAG, "uart driver");
    ESP_RETURN_ON_ERROR(uart_param_config(kUart, &cfg), TAG, "uart config");
    ESP_RETURN_ON_ERROR(uart_set_pin(kUart, BSP_UART2_TX_GPIO, BSP_UART2_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart pins");

    ESP_RETURN_ON_ERROR(power_on_camera(), TAG, "camera power");
    initialized_ = true;
    return ESP_OK;
}

esp_err_t CameraUartStreamer::start()
{
    ESP_RETURN_ON_ERROR(init(), TAG, "init");
    BaseType_t ok = xTaskCreatePinnedToCore(task_entry, "cam_uart",
                                            APP_CAMERA_TASK_STACK_BYTES, this,
                                            APP_CAMERA_TASK_PRIORITY, nullptr,
                                            APP_CAMERA_TASK_CORE);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void CameraUartStreamer::task_entry(void *arg)
{
    static_cast<CameraUartStreamer *>(arg)->task();
}

void CameraUartStreamer::task()
{
    write_status("camera uart stream: protocol=CAMU, sensor=GC032A, source=cpu-spi2-burst");
    probe_i2c();

    esp_err_t init_err = init_gc032a_sensor();
    if (init_err != ESP_OK) {
        char line[96];
        std::snprintf(line, sizeof(line), "gc032a: init register table failed: %s",
                      esp_err_to_name(init_err));
        write_status(line);
        vTaskDelete(nullptr);
    }
    if (alloc_sample_buffer() != ESP_OK) {
        write_status("gc032a: failed to allocate sample buffers");
        vTaskDelete(nullptr);
    }

    char banner[128];
    std::snprintf(banner, sizeof(banner),
                  "gc032a cpu-burst: capacity=%u pairs (~%u packed bytes), output=%ux%u YVYU%s",
                  static_cast<unsigned>(raw_capacity_),
                  static_cast<unsigned>(raw_capacity_ / 4U),
                  static_cast<unsigned>(kSensorWidth),
                  static_cast<unsigned>(kSensorHeight),
#if APP_CAMERA_FULL_FRAME_ENABLE
                  " full-frame"
#else
                  " preview"
#endif
                  );
    write_status(banner);

#if APP_CAMERA_DVP_DMA_PROBE_ENABLE
    dvp_dma_probe_loop();
#else
    uint32_t seq = 0;
    const TickType_t period = pdMS_TO_TICKS(1000);
    while (true) {
        capture_and_send_frame(seq++);
        vTaskDelay(period);
    }
#endif
}

esp_err_t CameraUartStreamer::start_mclk()
{
    ledc_timer_config_t timer = {
        .speed_mode = kMclkSpeedMode,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .timer_num = kMclkTimer,
        .freq_hz = APP_GC032A_MCLK_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "mclk timer");

    ledc_channel_config_t ch = {
        .gpio_num = BSP_GC032A_MCLK_GPIO,
        .speed_mode = kMclkSpeedMode,
        .channel = kMclkChannel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = kMclkTimer,
        .duty = 1,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {},
    };
    return ledc_channel_config(&ch);
}

esp_err_t CameraUartStreamer::power_on_camera()
{
    // Drive the adapter's 2.8 V LDO enable high. The schematic puts PWR_EN on
    // CON6 pin 3 == GPIO9. Left floating it defaults to off on most boards,
    // which silently denies power to the sensor and makes I2C NAK.
    gpio_config_t pwr_en = {
        .pin_bit_mask = 1ULL << BSP_CAMERA_PWR_EN_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&pwr_en), TAG, "pwr_en gpio");
    gpio_set_level(BSP_CAMERA_PWR_EN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_config_t pwdn = {
        .pin_bit_mask = 1ULL << BSP_GC032A_PWDN_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&pwdn), TAG, "pwdn gpio");
    // Pulse PWDN high briefly to force a clean reset, then release.
    gpio_set_level(BSP_GC032A_PWDN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(BSP_GC032A_PWDN_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    gpio_config_t in = {
        .pin_bit_mask = (1ULL << BSP_GC032A_SPI_CLK_GPIO) |
                        (1ULL << BSP_GC032A_DATA0_GPIO) |
                        (1ULL << BSP_GC032A_DATA1_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&in);
}

esp_err_t CameraUartStreamer::attach_gc032a()
{
    if (gc032a_) return ESP_OK;
    ESP_RETURN_ON_ERROR(select_gc032a_address(), TAG, "select gc032a addr");

    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) {
        ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "camera i2c init");
        bus = bsp_i2c_bus();
    }
    if (!bus) return ESP_ERR_INVALID_STATE;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = gc032a_addr_,
        .scl_speed_hz = BSP_I2C0_FREQ_HZ,
        .scl_wait_us = 0,
        .flags = {},
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &gc032a_);
    if (err != ESP_OK) {
        gc032a_ = nullptr;
    }
    return err;
}

esp_err_t CameraUartStreamer::select_gc032a_address()
{
    if (gc032a_addr_ != 0) return ESP_OK;

    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) {
        ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "camera i2c init");
        bus = bsp_i2c_bus();
    }
    if (!bus) return ESP_ERR_INVALID_STATE;

    constexpr uint8_t candidates[] = {
        APP_GC032A_I2C_ADDR,
        0x21, 0x31, 0x3C, 0x42, 0x5C, 0x6E,
    };

    for (int attempt = 0; attempt < 3; ++attempt) {
        for (uint8_t addr : candidates) {
            if (addr == 0 || addr >= 0x78) continue;
            esp_err_t pe = i2c_master_probe(bus, addr, 50);
            if (pe != ESP_OK) {
                char line[96];
                std::snprintf(line, sizeof(line),
                              "gc032a probe try=%d addr=0x%02X: no ack (%s)",
                              attempt, addr, esp_err_to_name(pe));
                write_status(line);
                continue;
            }

            i2c_master_dev_handle_t dev = nullptr;
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                .scl_speed_hz = BSP_I2C0_FREQ_HZ,
                .scl_wait_us = 0,
                .flags = {},
            };
            if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK) continue;

            uint8_t idh = 0, idl = 0;
            uint8_t reg = kGc032aIdHighReg;
            esp_err_t e0 = i2c_master_transmit_receive(dev, &reg, 1, &idh, 1, 100);
            reg = kGc032aIdLowReg;
            esp_err_t e1 = i2c_master_transmit_receive(dev, &reg, 1, &idl, 1, 100);
            i2c_master_bus_rm_device(dev);

            uint16_t id = static_cast<uint16_t>((idh << 8) | idl);
            char line[96];
            std::snprintf(line, sizeof(line),
                          "gc032a try=%d addr=0x%02X id=0x%04X (e0=%s e1=%s)",
                          attempt, addr, id, esp_err_to_name(e0), esp_err_to_name(e1));
            write_status(line);

            if (e0 == ESP_OK && e1 == ESP_OK && id == kGc032aExpectedId) {
                gc032a_addr_ = addr;
                return ESP_OK;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t CameraUartStreamer::gc032a_read_reg(uint8_t reg, uint8_t *val)
{
    if (!gc032a_) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(gc032a_, &reg, 1, val, 1, 100);
}

esp_err_t CameraUartStreamer::gc032a_write_reg(uint8_t reg, uint8_t val)
{
    if (!gc032a_) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(gc032a_, buf, sizeof(buf), 100);
}

void CameraUartStreamer::probe_gc032a()
{
    if (attach_gc032a() != ESP_OK) {
        write_status("gc032a: i2c attach failed");
        return;
    }
    uint8_t idh = 0, idl = 0;
    esp_err_t e0 = gc032a_read_reg(kGc032aIdHighReg, &idh);
    esp_err_t e1 = gc032a_read_reg(kGc032aIdLowReg, &idl);
    char line[128];
    if (e0 == ESP_OK && e1 == ESP_OK) {
        std::snprintf(line, sizeof(line),
                      "gc032a: i2c=0x%02X id=0x%02X%02X mclk=%u pwdn=low",
                      gc032a_addr_, idh, idl, APP_GC032A_MCLK_HZ);
    } else {
        std::snprintf(line, sizeof(line), "gc032a: probe failed i2c=0x%02X",
                      gc032a_addr_);
    }
    write_status(line);
}

esp_err_t CameraUartStreamer::init_gc032a_sensor()
{
    ESP_RETURN_ON_ERROR(attach_gc032a(), TAG, "gc032a attach");
    uint8_t idh = 0, idl = 0;
    ESP_RETURN_ON_ERROR(gc032a_read_reg(kGc032aIdHighReg, &idh), TAG, "read id high");
    ESP_RETURN_ON_ERROR(gc032a_read_reg(kGc032aIdLowReg, &idl), TAG, "read id low");
    if (((idh << 8) | idl) != kGc032aExpectedId) {
        return ESP_ERR_NOT_FOUND;
    }
    for (const auto &rv : kGc032aInitRegs) {
        ESP_RETURN_ON_ERROR(gc032a_write_reg(rv.reg, rv.val), TAG, "write init reg");
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    char line[96];
    std::snprintf(line, sizeof(line),
                  "gc032a: wrote %u init regs, source stream=640x480, preview=%ux%u, expecting sync AB/80/9D/B6",
                  static_cast<unsigned>(sizeof(kGc032aInitRegs) /
                                        sizeof(kGc032aInitRegs[0])),
                  static_cast<unsigned>(kPreviewWidth),
                  static_cast<unsigned>(kPreviewHeight));
    write_status(line);
    dump_gc032a_spi_regs();
    return ESP_OK;
}

void CameraUartStreamer::dump_gc032a_spi_regs()
{
    static constexpr uint8_t regs[] = {
        0x51, 0x52, 0x53, 0x54, 0x55, 0x59, 0x5a, 0x5b, 0x5c,
        0x5d, 0x5e, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67,
    };

    if (gc032a_write_reg(0xfe, 0x03) != ESP_OK) {
        write_status("gc032a: failed to select spi reg page for dump");
        return;
    }

    char line[192];
    int w = std::snprintf(line, sizeof(line), "gc032a spi regs:");
    for (uint8_t reg : regs) {
        uint8_t val = 0;
        if (gc032a_read_reg(reg, &val) == ESP_OK) {
            w += std::snprintf(line + w, sizeof(line) - w, " %02X=%02X", reg, val);
        } else {
            w += std::snprintf(line + w, sizeof(line) - w, " %02X=??", reg);
        }
        if (w >= static_cast<int>(sizeof(line)) - 8) {
            break;
        }
    }
    write_status(line);
    gc032a_write_reg(0xfe, 0x00);
}

esp_err_t CameraUartStreamer::alloc_sample_buffer()
{
    if (raw_pair_buf_ && packed_buf_) return ESP_OK;
    raw_capacity_ = kRawCapacityBytes;
    raw_pair_buf_ = static_cast<uint8_t *>(
        heap_caps_malloc(raw_capacity_, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    packed_buf_ = static_cast<uint8_t *>(
        heap_caps_malloc(raw_capacity_ / 4U, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
#if APP_CAMERA_FULL_FRAME_ENABLE
    frame_capacity_ = kSensorFrameBytes;
    frame_buf_ = static_cast<uint8_t *>(
        heap_caps_malloc(frame_capacity_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
#endif
    if (!raw_pair_buf_ || !packed_buf_
#if APP_CAMERA_FULL_FRAME_ENABLE
        || !frame_buf_
#endif
        ) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void CameraUartStreamer::probe_i2c()
{
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) {
        write_status("camera i2c: bus unavailable");
        return;
    }
    int found = 0;
    char line[96];
    for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
        if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
            std::snprintf(line, sizeof(line), "camera i2c: ack addr=0x%02X", addr);
            write_status(line);
            ++found;
        }
    }
    std::snprintf(line, sizeof(line), "camera i2c: scan complete, devices=%d", found);
    write_status(line);
}

void CameraUartStreamer::dvp_dma_probe_loop()
{
    write_status("gc032a dvp-dma probe: LCD_CAM samples D0/D1 on PCLK; packing low 2 bits in software");

    esp_cam_ctlr_dvp_pin_config_t pin_cfg = {};
    pin_cfg.data_width = CAM_CTLR_DATA_WIDTH_8;
    for (int i = 0; i < CAM_DVP_DATA_SIG_NUM; ++i) {
        pin_cfg.data_io[i] = GPIO_NUM_NC;
    }
    pin_cfg.data_io[0] = BSP_GC032A_DATA0_GPIO;
    pin_cfg.data_io[1] = BSP_GC032A_DATA1_GPIO;
    pin_cfg.vsync_io = GPIO_NUM_NC;
    pin_cfg.de_io = GPIO_NUM_NC;
    pin_cfg.pclk_io = BSP_GC032A_SPI_CLK_GPIO;
    pin_cfg.xclk_io = GPIO_NUM_NC;

    esp_cam_ctlr_dvp_config_t dvp_cfg = {
        .ctlr_id = 0,
        .clk_src = CAM_CLK_SRC_DEFAULT,
        .h_res = 256,
        .v_res = static_cast<uint32_t>(kDvpProbeRawBytes / 256U),
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .cam_data_width = 8,
        .bit_swap_en = false,
        .byte_swap_en = false,
        .bk_buffer_dis = true,
        .pin_dont_init = false,
        .pic_format_jpeg = false,
        .external_xtal = true,
        .dma_burst_size = 64,
        .xclk_freq = 0,
        .pin = &pin_cfg,
    };

    esp_cam_ctlr_handle_t cam = nullptr;
    esp_err_t err = esp_cam_new_dvp_ctlr(&dvp_cfg, &cam);
    if (err != ESP_OK) {
        char line[128];
        std::snprintf(line, sizeof(line), "dvp-dma probe: controller init failed: %s",
                      esp_err_to_name(err));
        write_status(line);
        vTaskDelete(nullptr);
    }

    DvpProbeContext ctx = {};
    ctx.buffer_len = kDvpProbeRawBytes;
    ctx.done_queue = xQueueCreate(4, sizeof(DvpProbeDone));
    uint32_t caps =
#if CONFIG_SPIRAM
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
#else
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
#endif
    ctx.buffers[0] = static_cast<uint8_t *>(
        esp_cam_ctlr_alloc_buffer(cam, ctx.buffer_len, caps));
    ctx.buffers[1] = static_cast<uint8_t *>(
        esp_cam_ctlr_alloc_buffer(cam, ctx.buffer_len, caps));
    uint8_t *probe_packed = static_cast<uint8_t *>(
        heap_caps_malloc(kDvpProbePackedBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!ctx.done_queue || !ctx.buffers[0] || !ctx.buffers[1] || !probe_packed) {
        write_status("dvp-dma probe: failed to allocate DMA/probe buffers");
        vTaskDelete(nullptr);
    }

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = dvp_probe_get_new_trans,
        .on_trans_finished = dvp_probe_trans_finished,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(cam, &cbs, &ctx));
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam));

    auto *cam_hw = CAM_LL_GET_HW(0);
    cam_ll_stop(cam_hw);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, CAM_V_SYNC_IDX, false);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, CAM_H_ENABLE_IDX, false);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, CAM_H_SYNC_IDX, false);
    cam_ll_set_vh_de_mode(cam_hw, false);
    cam_ll_enable_vsync_generate_eof(cam_hw, false);
    cam_ll_set_recv_data_bytelen(cam_hw, static_cast<uint32_t>(kDvpProbeRawBytes - 1U));
    cam_ll_enable_invert_pclk(cam_hw, false);
    cam_ll_fifo_reset(cam_hw);

    ESP_ERROR_CHECK(esp_cam_ctlr_start(cam));
    cam_ll_start(cam_hw);

    uint32_t windows = 0;
    uint32_t best_fs = 0, best_ls = 0, best_le = 0, best_fe = 0;
    char best_mode[16] = "none";
    while (true) {
        DvpProbeDone done;
        if (xQueueReceive(ctx.done_queue, &done, pdMS_TO_TICKS(1000)) != pdTRUE) {
            char line[192];
            std::snprintf(line, sizeof(line),
                          "dvp-dma probe: timeout size-eof ctrl=0x%08lX ctrl1=0x%08lX raw=0x%08lX st=0x%08lX",
                          static_cast<unsigned long>(cam_hw->cam_ctrl.val),
                          static_cast<unsigned long>(cam_hw->cam_ctrl1.val),
                          static_cast<unsigned long>(cam_hw->lc_dma_int_raw.val),
                          static_cast<unsigned long>(cam_hw->lc_dma_int_st.val));
            write_status(line);
            continue;
        }

        size_t raw_len = std::min(done.received, ctx.buffer_len);
        const char *mode_name = "none";
        SyncCounts mode_counts = {};
        for (int mode = 0; mode < 4; ++mode) {
            Spi2WirePacker packer = {
                .swap = (mode & 0x01) != 0,
                .lsb_first = (mode & 0x02) != 0,
            };
            size_t packed_len = packer.push(done.buffer, raw_len,
                                            probe_packed, kDvpProbePackedBytes);
            SyncCounts counts = count_bt656_syncs(probe_packed, packed_len);
            uint32_t score = counts.fs * 16U + counts.ls * 4U + counts.le * 4U + counts.fe;
            uint32_t best_score = mode_counts.fs * 16U + mode_counts.ls * 4U +
                                  mode_counts.le * 4U + mode_counts.fe;
            if (score > best_score) {
                mode_counts = counts;
                mode_name = mode == 0 ? "msb" :
                            mode == 1 ? "msb-swap" :
                            mode == 2 ? "lsb" : "lsb-swap";
            }
        }

        best_fs += mode_counts.fs;
        best_ls += mode_counts.ls;
        best_le += mode_counts.le;
        best_fe += mode_counts.fe;
        std::snprintf(best_mode, sizeof(best_mode), "%s", mode_name);
        if (++windows >= kDvpProbeWindowsPerLog ||
            mode_counts.fs || mode_counts.ls || mode_counts.le || mode_counts.fe) {
            char line[192];
            std::snprintf(line, sizeof(line),
                          "dvp-dma probe windows=%lu raw=%u packed=%u mode=%s fs=%lu ls=%lu le=%lu fe=%lu totals=%lu/%lu/%lu/%lu",
                          static_cast<unsigned long>(windows),
                          static_cast<unsigned>(raw_len),
                          static_cast<unsigned>(kDvpProbePackedBytes),
                          best_mode,
                          static_cast<unsigned long>(mode_counts.fs),
                          static_cast<unsigned long>(mode_counts.ls),
                          static_cast<unsigned long>(mode_counts.le),
                          static_cast<unsigned long>(mode_counts.fe),
                          static_cast<unsigned long>(best_fs),
                          static_cast<unsigned long>(best_ls),
                          static_cast<unsigned long>(best_le),
                          static_cast<unsigned long>(best_fe));
            write_status(line);
            windows = 0;
        }
    }
}

void CameraUartStreamer::capture_and_send_frame(uint32_t seq)
{
#if APP_CAMERA_FULL_FRAME_ENABLE
    uint64_t t0 = esp_timer_get_time();
#if APP_CAMERA_RAW_ACCUM_ENABLE
    Spi2WirePacker packer = {
        .swap = true,
        .lsb_first = false,
    };
    size_t frame_len = 0;
    size_t total_pairs = 0;
    uint32_t bursts_since_delay = 0;
    while (frame_len < frame_capacity_ &&
           esp_timer_get_time() - t0 < kRawFrameMaxUs) {
        size_t got = spi2_cpu_burst(raw_pair_buf_, raw_capacity_,
                                    /*sample_falling=*/false, kBurstPollBudget);
        if (got < 16) {
            vTaskDelay(kCameraYieldTicks);
            continue;
        }
        total_pairs += got;
        frame_len += packer.push(raw_pair_buf_, got,
                                 frame_buf_ + frame_len,
                                 frame_capacity_ - frame_len);
        if (++bursts_since_delay >= kRawFrameYieldEveryBursts) {
            bursts_since_delay = 0;
            vTaskDelay(kCameraYieldTicks);
        }
    }

    uint64_t dt_us = esp_timer_get_time() - t0;
    char line[192];
    if (frame_len < frame_capacity_) {
        std::snprintf(line, sizeof(line),
                      "raw-frame seq=%lu timeout bytes=%u/%u pairs=%u took=%llu us",
                      static_cast<unsigned long>(seq),
                      static_cast<unsigned>(frame_len),
                      static_cast<unsigned>(frame_capacity_),
                      static_cast<unsigned>(total_pairs),
                      static_cast<unsigned long long>(dt_us));
        write_status(line);
        return;
    }

    std::snprintf(line, sizeof(line),
                  "raw-frame seq=%lu bytes=%u pairs=%u took=%llu us",
                  static_cast<unsigned long>(seq),
                  static_cast<unsigned>(frame_len),
                  static_cast<unsigned>(total_pairs),
                  static_cast<unsigned long long>(dt_us));
    write_status(line);
    emit_frame(seq, frame_buf_, frame_len, kSensorWidth, kSensorHeight);
    return;
#else
    FrameAssembler msb = {
        .out = frame_buf_,
        .out_capacity = frame_capacity_,
    };
    Spi2WirePacker packer = {
        .swap = false,
        .lsb_first = false,
    };

    size_t total_pairs = 0;
    size_t total_packed = 0;
    while (!msb.done && esp_timer_get_time() - t0 < kFullFrameMaxUs) {
        size_t got = spi2_cpu_burst(raw_pair_buf_, raw_capacity_,
                                    /*sample_falling=*/false, kBurstPollBudget);
        if (got < 16) {
            vTaskDelay(kCameraYieldTicks);
            continue;
        }
        total_pairs += got;

        size_t packed_len = packer.push(raw_pair_buf_, got, packed_buf_,
                                        raw_capacity_ / 4U);
        for (size_t i = 0; i < packed_len; ++i) {
            msb.feed(packed_buf_[i], total_packed + i);
        }
        total_packed += packed_len;
        vTaskDelay(kCameraYieldTicks);
    }

    uint64_t dt_us = esp_timer_get_time() - t0;
    FrameAssembler *winner = nullptr;
    const char *mode = "none";
    if (msb.done) {
        winner = &msb;
        mode = "msb";
    }

    char line[192];
    if (!winner) {
        std::snprintf(line, sizeof(line),
                      "full-frame seq=%lu timeout pairs=%u packed=%u took=%llu us lines=%lu bad_end=%lu last_bad=0x%08lX rst=%lu fs=%lu ls=%lu le=%lu fe=%lu sync=ff0000xx bitorder=msb-swap",
                      static_cast<unsigned long>(seq),
                      static_cast<unsigned>(total_pairs),
                      static_cast<unsigned>(total_packed),
                      static_cast<unsigned long long>(dt_us),
                      static_cast<unsigned long>(msb.lines),
                      static_cast<unsigned long>(msb.bad_line_end),
                      static_cast<unsigned long>(msb.last_bad_line_end),
                      static_cast<unsigned long>(msb.restarts),
                      static_cast<unsigned long>(msb.frame_syncs),
                      static_cast<unsigned long>(msb.line_start_syncs),
                      static_cast<unsigned long>(msb.line_end_syncs),
                      static_cast<unsigned long>(msb.frame_end_syncs));
        write_status(line);

        // Grab one more burst and run the 4-mode decode + hex dump so we can
        // see whether any (swap, lsb_first) combination finds BT.656 syncs.
        size_t probe_got = spi2_cpu_burst(raw_pair_buf_, raw_capacity_,
                                          /*sample_falling=*/false,
                                          kBurstPollBudget);
        if (probe_got >= 64) {
            analyze_and_dump(raw_pair_buf_, probe_got, /*sample_falling=*/false, seq);

            // Pair-granular per-lane sync scan: each of D0/D1 carries its own
            // BT.656 byte stream.
            RawSyncCounts rc = scan_raw_pairs_for_sync(raw_pair_buf_, probe_got);
            char rl[224];
            std::snprintf(rl, sizeof(rl),
                          "raw-scan pairs=%u D0 pfx=%lu fs=%lu ls=%lu le=%lu fe=%lu 1run=%lu 0run=%lu first=%ld | D1 pfx=%lu fs=%lu ls=%lu le=%lu fe=%lu 1run=%lu 0run=%lu first=%ld",
                          static_cast<unsigned>(probe_got),
                          static_cast<unsigned long>(rc.d0_prefix),
                          static_cast<unsigned long>(rc.d0_fs),
                          static_cast<unsigned long>(rc.d0_ls),
                          static_cast<unsigned long>(rc.d0_le),
                          static_cast<unsigned long>(rc.d0_fe),
                          static_cast<unsigned long>(rc.d0_max_one_run),
                          static_cast<unsigned long>(rc.d0_max_zero_run),
                          static_cast<long>(rc.d0_first_fs),
                          static_cast<unsigned long>(rc.d1_prefix),
                          static_cast<unsigned long>(rc.d1_fs),
                          static_cast<unsigned long>(rc.d1_ls),
                          static_cast<unsigned long>(rc.d1_le),
                          static_cast<unsigned long>(rc.d1_fe),
                          static_cast<unsigned long>(rc.d1_max_one_run),
                          static_cast<unsigned long>(rc.d1_max_zero_run),
                          static_cast<long>(rc.d1_first_fs));
            write_status(rl);

            // Dump the first 64 bytes of each lane's bit stream so we can
            // eyeball whether BT.656-like structure is on a single lane.
            uint8_t lane_buf[64];
            size_t n0 = extract_lane_bytes(raw_pair_buf_, probe_got, /*d1=*/false,
                                           lane_buf, sizeof(lane_buf));
            write_hex_dump("D0 lane", lane_buf, n0);
            size_t n1 = extract_lane_bytes(raw_pair_buf_, probe_got, /*d1=*/true,
                                           lane_buf, sizeof(lane_buf));
            write_hex_dump("D1 lane", lane_buf, n1);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        return;
    }

    std::snprintf(line, sizeof(line),
                  "full-frame seq=%lu mode=%s first_sync=%ld lines=%lu bad_end=%lu bytes=%u pairs=%u took=%llu us sync=ff0000xx bitorder=msb-swap",
                  static_cast<unsigned long>(seq),
                  mode,
                  static_cast<long>(winner->first_frame_start),
                  static_cast<unsigned long>(winner->lines),
                  static_cast<unsigned long>(winner->bad_line_end),
                  static_cast<unsigned>(winner->out_len),
                  static_cast<unsigned>(total_pairs),
                  static_cast<unsigned long long>(dt_us));
    write_status(line);
    emit_frame(seq, winner->out, winner->out_len, kSensorWidth, kSensorHeight);
    return;
#endif
#else
    uint64_t t0 = esp_timer_get_time();
    size_t got_total = 0;
    size_t packed_len = 0;
    Spi2WirePacker packer = {
        .swap = false,
        .lsb_first = false,
    };
    while (packed_len < raw_capacity_ / 4U) {
        size_t got = spi2_cpu_burst(raw_pair_buf_, raw_capacity_,
                                    /*sample_falling=*/false, kBurstPollBudget);
        got_total += got;
        packed_len += packer.push(raw_pair_buf_, got,
                                  packed_buf_ + packed_len,
                                  raw_capacity_ / 4U - packed_len);
        if (esp_timer_get_time() - t0 >= kCaptureMaxUs) {
            break;
        }
        vTaskDelay(kCameraYieldTicks);
    }
    uint64_t dt_us = esp_timer_get_time() - t0;

    char line[160];
    std::snprintf(line, sizeof(line),
                  "spi-frame seq=%lu edge=fall pairs=%u/%u took=%llu us (%.2f MHz est.)",
                  static_cast<unsigned long>(seq),
                  static_cast<unsigned>(got_total),
                  static_cast<unsigned>(raw_capacity_),
                  static_cast<unsigned long long>(dt_us),
                  dt_us > 0 ? (static_cast<double>(got_total) / static_cast<double>(dt_us))
                            : 0.0);
    write_status(line);

    if (got_total < 16 || packed_len == 0) {
        write_status("spi-burst: too few samples — PCLK not toggling?");
        return;
    }

    // Current hardware captures the cleanest diagnostic sync stream on falling
    // PCLK with D0/D1 in natural order and first sample in the low bits.
    size_t image_len = 0;
    bool in_frame = false;
    bool in_line = false;
    uint32_t line_count = 0;
    int32_t first_frame_start = -1;

    for (size_t i = 0; i < packed_len; ++i) {
        uint8_t b = packed_buf_[i];
        if (b == kFrameStart) {
            if (!in_frame) {
                first_frame_start = static_cast<int32_t>(i);
            }
            in_frame = true;
            in_line = false;
            continue;
        }
        if (!in_frame) {
            continue;
        }
        if (b == kLineStart) {
            in_line = true;
            continue;
        }
        if (b == kLineEnd) {
            in_line = false;
            ++line_count;
            continue;
        }
        if (b == kFrameEnd) {
            break;
        }
        if (in_line && image_len < kPreviewBytes) {
            packed_buf_[image_len++] = b;
        }
    }

    if (image_len == 0) {
        write_status("spi-frame: no line payload found in burst");
        return;
    }
    size_t emit_len = std::min(image_len, kPreviewBytes);
    uint32_t emit_height = static_cast<uint32_t>(emit_len / kPreviewRowBytes);
    emit_len = emit_height * kPreviewRowBytes;
    if (emit_height == 0) {
        write_status("spi-frame: payload shorter than one preview row");
        return;
    }

    std::snprintf(line, sizeof(line),
                  "spi-frame seq=%lu first_AB=%ld lines=%lu payload=%u/%u emit=%ux%u",
                  static_cast<unsigned long>(seq),
                  static_cast<long>(first_frame_start),
                  static_cast<unsigned long>(line_count),
                  static_cast<unsigned>(image_len),
                  static_cast<unsigned>(kPreviewBytes),
                  static_cast<unsigned>(kPreviewWidth),
                  static_cast<unsigned>(emit_height));
    write_status(line);
    emit_frame(seq, packed_buf_, emit_len, kPreviewWidth, emit_height);
#endif
}

void CameraUartStreamer::emit_frame(uint32_t seq, const uint8_t *data, size_t len,
                                    uint32_t width, uint32_t height)
{
    char meta[96];
    std::snprintf(meta, sizeof(meta),
                  "frame=%lu;width=%lu;height=%lu;format=YVYU",
                  static_cast<unsigned long>(seq),
                  static_cast<unsigned long>(width),
                  static_cast<unsigned long>(height));
    write_packet(PKT_FRAME_START, seq, meta, std::strlen(meta));

    size_t off = 0;
    while (off < len) {
        size_t n = std::min<size_t>(APP_CAMERA_UART_CHUNK_BYTES, len - off);
        write_packet(PKT_FRAME_DATA, seq, data + off, static_cast<uint32_t>(n));
        off += n;
    }
    write_packet(PKT_FRAME_END, seq, nullptr, 0);
}

void CameraUartStreamer::analyze_and_dump(const uint8_t *raw_pairs, size_t pair_count,
                                          bool sample_falling, uint32_t seq)
{
    DecodeStats best = {};
    best.first_fs = -1;
    bool best_swap = false, best_lsb = false;
    for (int mode = 0; mode < 4; ++mode) {
        bool swap = (mode & 0x01) != 0;
        bool lsb  = (mode & 0x02) != 0;
        size_t bc = pack_pairs(raw_pairs, pair_count, packed_buf_, swap, lsb);
        DecodeStats s = score_packed(packed_buf_, bc, swap, lsb);

        char line[192];
        std::snprintf(line, sizeof(line),
                      "spi-burst decode edge=%s swap=%u lsb=%u: AB=%lu 80=%lu 9D=%lu B6=%lu AB->80=%lu first_AB=%ld score=%lu",
                      sample_falling ? "fall" : "rise",
                      swap ? 1U : 0U, lsb ? 1U : 0U,
                      static_cast<unsigned long>(s.fs),
                      static_cast<unsigned long>(s.ls),
                      static_cast<unsigned long>(s.le),
                      static_cast<unsigned long>(s.fe),
                      static_cast<unsigned long>(s.fs_then_ls),
                      static_cast<long>(s.first_fs),
                      static_cast<unsigned long>(s.score));
        write_status(line);

        if (s.score > best.score) {
            best = s;
            best_swap = swap;
            best_lsb  = lsb;
        }
    }

    // Dump the best decode so the host side can verify the byte stream by eye.
    size_t bc = pack_pairs(raw_pairs, pair_count, packed_buf_, best_swap, best_lsb);

    // Dump 64 bytes from the start of the packed stream
    {
        char tag[64];
        std::snprintf(tag, sizeof(tag), "best edge=%s swap=%u lsb=%u head",
                      sample_falling ? "fall" : "rise",
                      best_swap ? 1U : 0U, best_lsb ? 1U : 0U);
        write_hex_dump(tag, packed_buf_, std::min<size_t>(bc, 64U));
    }

    // Dump the 64 bytes around the first frame_start sync, if any
    if (best.first_fs >= 0) {
        size_t start = static_cast<size_t>(best.first_fs);
        size_t avail = bc - start;
        char tag[64];
        std::snprintf(tag, sizeof(tag), "best edge=%s @first_AB=%zu",
                      sample_falling ? "fall" : "rise", start);
        write_hex_dump(tag, packed_buf_ + start, std::min<size_t>(avail, 96U));
    }

    // Send the raw packed stream as one binary debug packet for offline tooling.
    write_packet(PKT_DEBUG_HEX, seq, packed_buf_, static_cast<uint32_t>(bc));
}

void CameraUartStreamer::write_packet(uint8_t type, uint32_t seq,
                                      const void *payload, uint32_t len)
{
    PacketHeader hdr = {
        .magic = {'C', 'A', 'M', 'U'},
        .type = type,
        .flags = 0,
        .header_len = sizeof(PacketHeader),
        .seq = seq,
        .payload_len = len,
        .checksum = fnv1a(payload, len),
    };
    uart_write_bytes(kUart, &hdr, sizeof(hdr));
    if (payload && len) {
        uart_write_bytes(kUart, payload, len);
    }
}

void CameraUartStreamer::write_status(const char *msg)
{
    ESP_LOGI(TAG, "%s", msg);
    write_packet(PKT_STATUS, 0, msg, std::strlen(msg));
}

void CameraUartStreamer::write_hex_dump(const char *tag, const uint8_t *data, size_t len)
{
    // 16 bytes per line: "<tag> [offset]: HH HH HH ..."
    char line[256];
    for (size_t off = 0; off < len; off += 16) {
        size_t n = std::min<size_t>(16, len - off);
        int w = std::snprintf(line, sizeof(line), "hex %s [%4u]:", tag,
                              static_cast<unsigned>(off));
        for (size_t i = 0; i < n && w < static_cast<int>(sizeof(line)) - 4; ++i) {
            w += std::snprintf(line + w, sizeof(line) - w, " %02X", data[off + i]);
        }
        write_status(line);
    }
}
