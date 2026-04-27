// ============================================================
// camera_uart.cpp  —  LCD_CAM sampler for GC032A 2-bit SPI stream
//
// Changes vs original:
//   1. Removed spi2_cpu_burst / capture_and_send_frame /
//      dump_probe_burst / analyze_and_dump (CPU-polling paths).
//   2. dvp_dma_probe_loop() split into two phases:
//        Phase A – calibration: collect kCalibWindows DMA
//          buffers, use the measured GC032A 2-wire bit order,
//          then pick the best byte phase.
//        Phase B – streaming: decode the raw D0/D1 samples as a
//          logic-analyzer-style dual-SPI stream. D0 is the low bit,
//          D1 is the high bit, and bytes are MSB-first.
//   3. PCLK polarity fixed (falling edge) – no mid-run toggle.
//   4. task() simplified: no APP_CAMERA_DVP_DMA_PROBE_ENABLE
//      guard; always takes the DVP path.
// ============================================================

#include "camera_uart.hpp"

#include <cinttypes>
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
#include "soc/lcd_cam_struct.h"   // lcd_cam_dev_t 定义在这里
#include "hal/cam_ll.h"           // CAM_LL_GET_HW、cam_ll_stop 等
#include "app_config.h"
#include "bsp.h"
#include "gc032a_regs.hpp"

namespace {

constexpr const char *TAG = "camera_uart";
constexpr uart_port_t kUart = UART_NUM_2;

// ── LEDC MCLK ────────────────────────────────────────────────
constexpr ledc_mode_t        kMclkSpeedMode      = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t       kMclkTimer          = LEDC_TIMER_0;
constexpr ledc_channel_t     kMclkChannel        = LEDC_CHANNEL_0;
constexpr ledc_timer_bit_t   kMclkDutyResolution = LEDC_TIMER_1_BIT;
constexpr uint32_t           kMclkDuty50Percent  = 1U;

// ── BT.656 sync codes emitted by GC032A SPI mode ──────────────
// XYZ byte: bit7=1, bit6=F, bit5=V, bit4=H, bit[3:0]=protection.
// F=0,V=0,H=0 -> 0x80 active-video SAV
// F=0,V=0,H=1 -> 0x9D active-video EAV
// F=0,V=1,H=0 -> 0xAB blanking SAV
// F=0,V=1,H=1 -> 0xB6 blanking EAV
constexpr uint8_t  kBt656BlankingSav = 0xAB;
constexpr uint8_t  kBt656ActiveSav   = 0x80;
constexpr uint8_t  kBt656ActiveEav   = 0x9D;
constexpr uint8_t  kBt656BlankingEav = 0xB6;
constexpr uint32_t kBt656SyncPrefix = 0xFF000000UL;
constexpr uint32_t kBlankingSavSync = kBt656SyncPrefix | kBt656BlankingSav;
constexpr uint32_t kActiveSavSync   = kBt656SyncPrefix | kBt656ActiveSav;
constexpr uint32_t kActiveEavSync   = kBt656SyncPrefix | kBt656ActiveEav;
constexpr uint32_t kBlankingEavSync = kBt656SyncPrefix | kBt656BlankingEav;

// ── GC032A I2C / ID ──────────────────────────────────────────
constexpr uint8_t  kGc032aIdHighReg  = 0xF0;
constexpr uint8_t  kGc032aIdLowReg   = 0xF1;
constexpr uint16_t kGc032aExpectedId = 0x232A;

// ── Sensor / preview geometry ─────────────────────────────────
constexpr uint32_t kSensorWidth    = APP_CAMERA_SENSOR_WIDTH;
constexpr uint32_t kSensorHeight   = APP_CAMERA_SENSOR_HEIGHT;
constexpr size_t   kSensorRowBytes = kSensorWidth * 2U;   // YVYU: 2 bytes/px
constexpr size_t   kSensorFrameBytes = kSensorRowBytes * kSensorHeight;
constexpr uint32_t kBringupEmitLines =
    (APP_CAMERA_PREVIEW_HEIGHT < 8U) ? APP_CAMERA_PREVIEW_HEIGHT : 8U;

// ── LCD_CAM raw sampler buffer sizing ─────────────────────────
// Each DMA transfer captures this many raw bytes from LCD_CAM.
// LCD_CAM stores one D0/D1 sample per byte (bits [1:0] used).
// 4 samples pack into 1 output byte, so packed ≈ raw/4.
// 8 KiB raw -> 2 KiB packed, enough to hold a 640px YVYU row plus sync.
constexpr size_t kDvpRawBytes = 8U * 1024U;
// Real GC032A active line = 1280 YVYU bytes between SAV and EAV. Noise hits
// in blanking show up at ~6–9 byte spacing — reject anything well below the
// real line size so we only commit genuine pixel rows.
constexpr size_t kMinLinePayloadBytes = 1200U;

// Empirically, each committed line starts with [2 trailing pixels of previous
// line] + [4 EAV bytes FF 00 00 9D] + [2 byte trailer]. Skip those 8 bytes at
// the front of each row and keep the next 1280 as real YVYU pixels.
constexpr size_t kLineHeaderSkipBytes = 8U;

// Logic analyzer QSPI decode shows GC032A emits D0 as the low bit and D1 as
// the high bit, with bytes reconstructed MSB-first, e.g. FF 00 00 80.
constexpr bool kGc032aWireSwap = false;
constexpr bool kGc032aWireLsbFirst = false;

// How many DMA windows to collect during calibration before picking the best
// 2-bit phase. The wire bit order is fixed by the measured GC032A output.
constexpr uint32_t kCalibWindows = 16U;
constexpr uint32_t kCalibMaxAttempts = 32U;

// ── Packet types ──────────────────────────────────────────────
enum PacketType : uint8_t {
    PKT_STATUS      = 1,
    PKT_FRAME_START = 2,
    PKT_FRAME_DATA  = 3,
    PKT_FRAME_END   = 4,
};

struct __attribute__((packed)) PacketHeader {
    char     magic[4];      // "CAMU"
    uint8_t  type;
    uint8_t  flags;
    uint16_t header_len;
    uint32_t seq;
    uint32_t payload_len;
    uint32_t checksum;
};

// ── DvpProbeContext (ISR ↔ task queue) ───────────────────────
struct DvpProbeDone {
    uint8_t *buffer   = nullptr;
    size_t   received = 0;
};

struct DvpProbeContext {
    uint8_t      *buffers[2] = {};
    size_t        buffer_len = 0;
    uint32_t      next       = 0;
    QueueHandle_t done_queue = nullptr;
};

// ── Utilities ─────────────────────────────────────────────────
static uint32_t fnv1a(const void *data, size_t len)
{
    const auto *p = static_cast<const uint8_t *>(data);
    uint32_t h = 2166136261UL;
    for (size_t i = 0; i < len; ++i) {
        h ^= p[i];
        h *= 16777619UL;
    }
    return h;
}

// ── ISR callbacks ─────────────────────────────────────────────
static bool IRAM_ATTR dvp_get_new_trans(esp_cam_ctlr_handle_t,
                                        esp_cam_ctlr_trans_t *trans,
                                        void *user_data)
{
    auto *ctx = static_cast<DvpProbeContext *>(user_data);
    uint32_t idx = ctx->next++ & 0x01U;
    trans->buffer = ctx->buffers[idx];
    trans->buflen = ctx->buffer_len;
    return false;
}

static bool IRAM_ATTR dvp_trans_finished(esp_cam_ctlr_handle_t,
                                         esp_cam_ctlr_trans_t *trans,
                                         void *user_data)
{
    auto *ctx = static_cast<DvpProbeContext *>(user_data);
    DvpProbeDone done = {
        .buffer   = static_cast<uint8_t *>(trans->buffer),
        .received = trans->received_size,
    };
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(ctx->done_queue, &done, &woken);
    return woken == pdTRUE;
}

// ─────────────────────────────────────────────────────────────
// Spi2WirePacker
//
// Packs raw 2-bit samples (one per byte, bits[1:0]) into bytes.
// Maintains phase across calls — do NOT reset between DMA
// buffers in the streaming phase.
// ─────────────────────────────────────────────────────────────
struct Spi2WirePacker {
    bool    swap      = false;  // swap D0↔D1 inside each pair
    bool    lsb_first = false;  // first sample → bits[1:0] (LSB) vs bits[7:6] (MSB)
    uint8_t out_byte  = 0;
    uint8_t phase     = 0;      // 0..3: samples accumulated so far in current byte

    void reset_state()
    {
        out_byte = 0;
        phase    = 0;
    }

    // Pack `pair_count` raw samples into `packed`.
    // Returns number of bytes written.
    size_t push(const uint8_t *raw, size_t pair_count,
                uint8_t *packed, size_t packed_cap)
    {
        size_t out_len = 0;
        for (size_t i = 0; i < pair_count; ++i) {
            uint8_t pair = raw[i] & 0x03U;
            if (swap) {
                pair = static_cast<uint8_t>(((pair & 0x01U) << 1) |
                                             ((pair & 0x02U) >> 1));
            }
            uint32_t shift = lsb_first ? (phase * 2U) : ((3U - phase) * 2U);
            out_byte = static_cast<uint8_t>(out_byte | (pair << shift));
            if (++phase >= 4U) {
                if (out_len >= packed_cap) {
                    // buffer full — caller must handle
                    phase    = 0;
                    out_byte = 0;
                    return out_len;
                }
                packed[out_len++] = out_byte;
                out_byte = 0;
                phase    = 0;
            }
        }
        return out_len;
    }
};

// ─────────────────────────────────────────────────────────────
// FrameAssembler
//
// Feed packed bytes one at a time. Parses BT.656 active video:
//   FF 00 00 80  = active SAV, start accepting image payload
//   FF 00 00 9D  = active EAV, finish current image line
//   FF 00 00 AB/B6 are blanking SAV/EAV and are ignored.
// ─────────────────────────────────────────────────────────────
enum class FPS { SeekActiveSav, Payload };

struct FrameAssembler {
    FPS      state    = FPS::SeekActiveSav;
    uint8_t *out      = nullptr;
    size_t   out_cap  = 0;
    size_t   out_len  = 0;
    size_t   line_start_len = 0;
    size_t   line_pos = 0;
    uint32_t lines    = 0;
    uint32_t sync_win = 0;
    // diagnostics
    uint32_t frame_syncs = 0, line_start_syncs = 0;
    uint32_t line_end_syncs = 0, frame_end_syncs = 0;
    uint32_t bad_line_ends = 0, restarts = 0;
    bool     done     = false;

    void reset_frame()
    {
        state    = FPS::SeekActiveSav;
        out_len  = 0;
        line_start_len = 0;
        line_pos = 0;
        lines    = 0;
    }

    void drop_sync_from_payload()
    {
        out_len = (out_len >= 4U) ? (out_len - 4U) : 0U;
        line_pos = (line_pos >= 4U) ? (line_pos - 4U) : 0U;
    }

    // Trim the just-finished line: drop the leading kLineHeaderSkipBytes
    // (trailing pixels of the previous row + EAV + a 2-byte trailer that the
    // sensor inserts after EAV) by shifting payload left, then cap to exactly
    // kSensorRowBytes so we land on a clean 1280-byte YVYU row.
    void trim_line_to_row()
    {
        if (!out) { line_pos = 0; return; }
        size_t collected = (out_len > line_start_len)
                               ? (out_len - line_start_len) : 0U;
        if (collected > kLineHeaderSkipBytes) {
            size_t shift = kLineHeaderSkipBytes;
            size_t kept  = collected - shift;
            std::memmove(out + line_start_len,
                         out + line_start_len + shift,
                         kept);
            out_len = line_start_len + kept;
        } else {
            out_len = line_start_len;
        }
        size_t want_end = line_start_len + kSensorRowBytes;
        if (out_len > want_end) {
            out_len = want_end;
        } else if (out_len < want_end) {
            std::memset(out + out_len, 0, want_end - out_len);
            out_len = want_end;
        }
        line_pos = kSensorRowBytes;
    }

    void rollback_line()
    {
        out_len = line_start_len;
        line_pos = 0;
    }

    void feed(uint8_t b)
    {
        if (done) return;

        sync_win = (sync_win << 8U) | b;

        if (sync_win == kBlankingSavSync) ++frame_syncs;
        if (sync_win == kActiveSavSync)   ++line_start_syncs;
        if (sync_win == kActiveEavSync)   ++line_end_syncs;
        if (sync_win == kBlankingEavSync) ++frame_end_syncs;

        switch (state) {
        case FPS::SeekActiveSav:
            if (sync_win == kActiveSavSync) {
                line_start_len = out_len;
                line_pos       = 0;
                state          = FPS::Payload;
            }
            break;

        case FPS::Payload:
            if (out_len < out_cap && out) {
                out[out_len] = b;
            }
            ++out_len;
            ++line_pos;
            if (sync_win == kActiveEavSync) {
                if (line_pos >= kMinLinePayloadBytes) {
                    drop_sync_from_payload();
                    trim_line_to_row();
                    ++lines;
                    if (lines >= kBringupEmitLines || out_len >= out_cap) {
                        done = true;
                    } else {
                        line_start_len = out_len;
                        line_pos = 0;
                        state = FPS::SeekActiveSav;
                    }
                } else {
                    // Noise hit while still inside the line — ignore, keep
                    // collecting until either a real EAV (line_pos>=min) or
                    // the line-length watchdog below trips.
                    ++bad_line_ends;
                }
            } else if (sync_win == kActiveSavSync) {
                if (line_pos >= kMinLinePayloadBytes) {
                    // Real next-line SAV without seeing EAV — accept current
                    // line and start the next one immediately.
                    drop_sync_from_payload();
                    trim_line_to_row();
                    ++lines;
                    if (lines >= kBringupEmitLines || out_len >= out_cap) {
                        done = true;
                    } else {
                        line_start_len = out_len;
                        line_pos = 0;
                        state = FPS::Payload;
                    }
                } else {
                    ++bad_line_ends;  // noise hit — keep collecting
                }
            } else if (sync_win == kBlankingSavSync || sync_win == kBlankingEavSync) {
                if (line_pos >= kMinLinePayloadBytes) {
                    drop_sync_from_payload();
                    trim_line_to_row();
                    ++lines;
                    if (lines >= kBringupEmitLines || out_len >= out_cap) {
                        done = true;
                    } else {
                        line_start_len = out_len;
                        line_pos = 0;
                        state = FPS::SeekActiveSav;
                    }
                } else {
                    ++bad_line_ends;  // noise blanking sync — ignore
                }
            } else if (line_pos > (kSensorRowBytes + 64U)) {
                ++bad_line_ends;
                ++restarts;
                rollback_line();
                state = FPS::SeekActiveSav;
            }
            break;
        }
    }
};

// ─────────────────────────────────────────────────────────────
// scan_packed_syncs  —  score one packed buffer
// ─────────────────────────────────────────────────────────────
struct SyncCounts {
    uint32_t prefix = 0; // FF 00 00 xx
    uint32_t fs = 0, ls = 0, le = 0, fe = 0;
    uint32_t score  = 0;
    int first_prefix = -1;
    int first_tail = -1;

    uint32_t exact() const { return fs + ls + le + fe; }
};

static SyncCounts scan_packed_syncs(const uint8_t *packed, size_t len)
{
    SyncCounts c = {};
    uint32_t win = 0;
    for (size_t i = 0; i < len; ++i) {
        win = (win << 8U) | packed[i];
        if (i < 3) continue;
        if ((win & 0xFFFFFF00UL) == kBt656SyncPrefix) {
            ++c.prefix;
            if (c.first_prefix < 0) c.first_prefix = static_cast<int>(i - 3);
            if (c.first_tail < 0) c.first_tail = static_cast<int>(win & 0xFFU);
        }
        if (win == kBlankingSavSync) ++c.fs;
        if (win == kActiveSavSync)   ++c.ls;
        if (win == kActiveEavSync)   ++c.le;
        if (win == kBlankingEavSync) ++c.fe;
    }
    c.score = c.prefix * 8U + c.fs * 256U + c.ls * 64U +
              c.le * 64U + c.fe * 256U;
    return c;
}

// ─────────────────────────────────────────────────────────────
// pack_raw  —  single-shot packing (for calibration only)
// Does NOT maintain state between calls.
// ─────────────────────────────────────────────────────────────
static size_t pack_raw(const uint8_t *raw, size_t raw_len,
                       uint8_t *packed,
                       bool swap, bool lsb, uint8_t pair_phase)
{
    Spi2WirePacker p = { .swap = swap, .lsb_first = lsb };
    // skip `pair_phase` samples to align to byte boundary
    if (pair_phase >= raw_len) return 0;
    return p.push(raw + pair_phase, raw_len - pair_phase,
                  packed, (raw_len - pair_phase) / 4U);
}

// ─────────────────────────────────────────────────────────────
// restart_cam_hw  —  stop → reset FIFO → re-arm → start
// ─────────────────────────────────────────────────────────────
static void restart_cam_hw(lcd_cam_dev_t *hw, bool falling)
{
    cam_ll_stop(hw);
    cam_ll_fifo_reset(hw);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT,
                                   CAM_V_SYNC_IDX, false);
    cam_ll_enable_invert_pclk(hw, falling);
    cam_ll_start(hw);
    // pulse VSYNC high so the controller begins accepting pixels
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT,
                                   CAM_V_SYNC_IDX, false);
}

} // namespace

// =============================================================
// CameraUartStreamer — public / lifecycle
// =============================================================

esp_err_t CameraUartStreamer::init()
{
    if (initialized_) return ESP_OK;
    ESP_RETURN_ON_ERROR(start_mclk(), TAG, "mclk");
    uart_config_t cfg = {
        .baud_rate  = APP_CAMERA_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(kUart, 2048, 0, 0, nullptr, 0),
                        TAG, "uart driver");
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
    BaseType_t ok = xTaskCreatePinnedToCore(
        task_entry, "cam_uart",
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
    write_status("gc032a bring-up: MCLK → PWDN → I2C init → DVP DMA stream");
    probe_i2c();

    if (init_gc032a_sensor() != ESP_OK) {
        write_status("gc032a: sensor init failed");
        vTaskDelete(nullptr);
    }

    // Frame buffer in PSRAM
    frame_buf_ = static_cast<uint8_t *>(
        heap_caps_malloc(kSensorFrameBytes,
                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    // Scratch packed buffer (raw/4 bytes)
    packed_buf_ = static_cast<uint8_t *>(
        heap_caps_malloc(kDvpRawBytes / 4U,
                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!frame_buf_ || !packed_buf_) {
        write_status("gc032a: frame/packed buf alloc failed");
        vTaskDelete(nullptr);
    }

    dvp_stream_loop();
}

// =============================================================
// dvp_stream_loop  —  calibrate then stream
// =============================================================
void CameraUartStreamer::dvp_stream_loop()
{
    // ── Build and start the LCD_CAM DVP controller ───────────
    esp_cam_ctlr_dvp_pin_config_t pin_cfg = {};
    pin_cfg.data_width = CAM_CTLR_DATA_WIDTH_8;
    for (int i = 0; i < CAM_DVP_DATA_SIG_NUM; ++i) {
        pin_cfg.data_io[i] = GPIO_NUM_NC;
    }
    pin_cfg.data_io[0] = BSP_GC032A_DATA0_GPIO;
    pin_cfg.data_io[1] = BSP_GC032A_DATA1_GPIO;
    pin_cfg.vsync_io   = GPIO_NUM_NC;
    pin_cfg.de_io      = GPIO_NUM_NC;
    pin_cfg.pclk_io    = BSP_GC032A_SPI_CLK_GPIO;
    pin_cfg.xclk_io    = GPIO_NUM_NC;

    esp_cam_ctlr_dvp_config_t dvp_cfg = {
        .ctlr_id               = 0,
        .clk_src               = CAM_CLK_SRC_DEFAULT,
        .h_res                 = 256,
        .v_res                 = static_cast<uint32_t>(kDvpRawBytes / 256U),
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .cam_data_width        = 8,
        .bit_swap_en           = false,
        .byte_swap_en          = false,
        .bk_buffer_dis         = true,
        .pin_dont_init         = false,
        .pic_format_jpeg       = false,
        .external_xtal         = true,
        .dma_burst_size        = 64,
        .xclk_freq             = 0,
        .pin                   = &pin_cfg,
    };

    esp_cam_ctlr_handle_t cam = nullptr;
    if (esp_cam_new_dvp_ctlr(&dvp_cfg, &cam) != ESP_OK) {
        write_status("dvp: controller init failed");
        vTaskDelete(nullptr);
    }

    DvpProbeContext ctx = {};
    ctx.buffer_len  = kDvpRawBytes;
    ctx.done_queue  = xQueueCreate(4, sizeof(DvpProbeDone));
    uint32_t buf_caps =
#if CONFIG_SPIRAM
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
#else
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
#endif
    ctx.buffers[0] = static_cast<uint8_t *>(
        esp_cam_ctlr_alloc_buffer(cam, ctx.buffer_len, buf_caps));
    ctx.buffers[1] = static_cast<uint8_t *>(
        esp_cam_ctlr_alloc_buffer(cam, ctx.buffer_len, buf_caps));
    if (!ctx.done_queue || !ctx.buffers[0] || !ctx.buffers[1]) {
        write_status("dvp: DMA buffer alloc failed");
        vTaskDelete(nullptr);
    }

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans  = dvp_get_new_trans,
        .on_trans_finished = dvp_trans_finished,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(cam, &cbs, &ctx));
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam));
    ESP_ERROR_CHECK(esp_cam_ctlr_start(cam));

    auto *hw = CAM_LL_GET_HW(0);
    bool pclk_invert = false;

    // Tie off unused sync inputs so LCD_CAM keeps running
    cam_ll_stop(hw);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, CAM_V_SYNC_IDX, false);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT,  CAM_H_ENABLE_IDX, false);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT,  CAM_H_SYNC_IDX,   false);
    cam_ll_set_vh_de_mode(hw, false);
    cam_ll_enable_vsync_generate_eof(hw, false);
    cam_ll_set_recv_data_bytelen(hw, static_cast<uint32_t>(kDvpRawBytes - 1U));
    cam_ll_enable_invert_pclk(hw, pclk_invert);
    cam_ll_fifo_reset(hw);
    cam_ll_start(hw);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, CAM_V_SYNC_IDX, false);

    // ── Phase A: calibration ──────────────────────────────────
    // Collect kCalibWindows DMA buffers, score all byte phases, and pick the
    // one with the best cumulative sync score. D0/D1 order is fixed from the
    // logic analyzer capture: D0=bit0, D1=bit1, MSB-first.
    write_status("dvp calib: fixed D0=low D1=high MSB-first; finding phase");

    struct ComboScore {
        bool     swap;
        bool     lsb;
        bool     pclk_invert;
        const char *profile;
        uint8_t  spi53;
        uint8_t  spi55;
        uint8_t  spi64;
        uint8_t  phase;
        uint32_t score;
        uint32_t prefix;
        uint32_t fs;
        uint32_t ls;
        uint32_t le;
        uint32_t fe;
        uint32_t windows;
        int      first_tail;
    };

    auto exact_syncs = [](const ComboScore &c) -> uint32_t {
        return c.fs + c.ls + c.le + c.fe;
    };
    auto better_combo = [&](const ComboScore &a, const ComboScore &b) -> bool {
        uint32_t ae = exact_syncs(a);
        uint32_t be = exact_syncs(b);
        if (ae != be) return ae > be;
        if (a.score != b.score) return a.score > b.score;
        return a.prefix > b.prefix;
    };

    auto calibrate_edge = [&](bool edge_invert) -> ComboScore {
        ComboScore combos[4] = {};
        for (int i = 0; i < 4; ++i) {
            combos[i] = { .swap  = kGc032aWireSwap,
                          .lsb   = kGc032aWireLsbFirst,
                          .pclk_invert = edge_invert,
                          .profile = nullptr,
                          .spi53 = 0,
                          .spi55 = 0,
                          .spi64 = 0,
                          .phase = static_cast<uint8_t>(i),
                          .score = 0,
                          .prefix = 0,
                          .fs = 0,
                          .ls = 0,
                          .le = 0,
                          .fe = 0,
                          .windows = 0,
                          .first_tail = -1 };
        }

        xQueueReset(ctx.done_queue);
        restart_cam_hw(hw, edge_invert);

        uint32_t calib_collected = 0;
        for (uint32_t attempt = 0;
             calib_collected < kCalibWindows && attempt < kCalibMaxAttempts;
             ++attempt) {
            DvpProbeDone done;
            if (xQueueReceive(ctx.done_queue, &done, pdMS_TO_TICKS(500)) != pdTRUE) {
                restart_cam_hw(hw, edge_invert);
                vTaskDelay(1);
                continue;
            }
            size_t raw_len = std::min(done.received, ctx.buffer_len);
            for (auto &c : combos) {
                size_t plen = pack_raw(done.buffer, raw_len, packed_buf_,
                                       c.swap, c.lsb, c.phase);
                SyncCounts sc = scan_packed_syncs(packed_buf_, plen);
                c.score += sc.score;
                c.prefix += sc.prefix;
                c.fs += sc.fs;
                c.ls += sc.ls;
                c.le += sc.le;
                c.fe += sc.fe;
                c.windows = calib_collected + 1U;
                if (c.first_tail < 0 && sc.first_tail >= 0) {
                    c.first_tail = sc.first_tail;
                }
            }
            ++calib_collected;
            vTaskDelay(1);
        }

        const ComboScore *best = &combos[0];
        for (const auto &c : combos) {
            if (better_combo(c, *best)) best = &c;
        }

        char edge_line[176];
        std::snprintf(edge_line, sizeof(edge_line),
                      "dvp calib edge invert=%u: windows=%lu swap=%u lsb=%u phase=%u score=%lu prefix=%lu first_tail=%02x bsav/asav/aeav/beav=%lu/%lu/%lu/%lu",
                      edge_invert ? 1U : 0U,
                      static_cast<unsigned long>(calib_collected),
                      best->swap ? 1U : 0U,
                      best->lsb ? 1U : 0U,
                      static_cast<unsigned>(best->phase),
                      static_cast<unsigned long>(best->score),
                      static_cast<unsigned long>(best->prefix),
                      static_cast<unsigned>(best->first_tail < 0 ? 0xff : best->first_tail),
                      static_cast<unsigned long>(best->fs),
                      static_cast<unsigned long>(best->ls),
                      static_cast<unsigned long>(best->le),
                      static_cast<unsigned long>(best->fe));
        write_status(edge_line);
        return *best;
    };

    static ComboScore fallback = { .swap = kGc032aWireSwap,
                                   .lsb = kGc032aWireLsbFirst,
                                   .pclk_invert = false,
                                   .profile = "factory_24m",
                                   .spi53 = 0x25,
                                   .spi55 = 0x00,
                                   .spi64 = 0x0c,
                                   .phase = 0, .score = 0,
                                   .prefix = 0, .fs = 0, .ls = 0,
                                   .le = 0, .fe = 0, .windows = 0,
                                   .first_tail = -1 };
    write_status("dvp calib profile=factory_24m 53=25 55=00 64=0c");
    ComboScore best = calibrate_edge(true);
    best.profile = "factory_24m";
    best.spi53 = 0x25;
    best.spi55 = 0x00;
    best.spi64 = 0x0c;
    ComboScore other = calibrate_edge(false);
    other.profile = "factory_24m";
    other.spi53 = 0x25;
    other.spi55 = 0x00;
    other.spi64 = 0x0c;
    if (better_combo(other, best)) {
        best = other;
    }
    if (best.score == 0) {
        best = fallback;
    }
    pclk_invert = best.pclk_invert;
    xQueueReset(ctx.done_queue);
    restart_cam_hw(hw, pclk_invert);

    char calib_line[192];
    std::snprintf(calib_line, sizeof(calib_line),
                  "dvp calib done: profile=%s 53=%02x 55=%02x 64=%02x pclk_invert=%u windows=%lu swap=%u lsb=%u phase=%u score=%lu",
                  best.profile ? best.profile : "?",
                  best.spi53,
                  best.spi55,
                  best.spi64,
                  best.pclk_invert ? 1U : 0U,
                  static_cast<unsigned long>(best.windows),
                  best.swap ? 1U : 0U,
                  best.lsb  ? 1U : 0U,
                  static_cast<unsigned>(best.phase),
                  static_cast<unsigned long>(best.score));
    write_status(calib_line);
    std::snprintf(calib_line, sizeof(calib_line),
                  "dvp calib syncs: prefix=%lu bsav/asav/aeav/beav=%lu/%lu/%lu/%lu",
                  static_cast<unsigned long>(best.prefix),
                  static_cast<unsigned long>(best.fs),
                  static_cast<unsigned long>(best.ls),
                  static_cast<unsigned long>(best.le),
                  static_cast<unsigned long>(best.fe));
    write_status(calib_line);
    if ((best.fs + best.ls + best.le + best.fe) < 4U) {
        write_status("dvp calib: weak sync lock; expect unstable frames");
    }

    // ── Phase B: streaming ────────────────────────────────────
    // ESP32-S3 has no PARLIO RX peripheral, and the standard SPI slave path
    // needs CS/transactions. Use LCD_CAM as a passive sampler and decode it
    // like the logic analyzer: D0=low bit, D1=high bit, MSB-first.
    Spi2WirePacker packer;
    packer.swap      = best.swap;
    packer.lsb_first = best.lsb;
    packer.reset_state();

    // Discard `best->phase` raw bytes to align to byte boundary.
    // We do this by feeding `phase` dummy samples at the start.
    // Simplest: just mark phase_skip and skip first raw bytes.
    const uint8_t phase_skip = best.phase;
    bool first_buf = true;

    FrameAssembler assembler;
    assembler.out     = frame_buf_;
    assembler.out_cap = kSensorFrameBytes;

    uint32_t frame_seq = 0;
    uint32_t diag_windows = 0;
    uint32_t zero_diag_periods = 0;
    SyncCounts diag_counts = {};
    uint32_t raw_dump_count = 0;
    constexpr uint32_t kRawDumpMax = 64U;
    uint32_t raw_dump_period = 0;
    constexpr uint32_t kRawDumpPeriod = 32U;  // print every Nth window
    uint32_t sync_span_dump_count = 0;
    constexpr uint32_t kSyncSpanDumpMax = 8U;
    uint32_t sync_search_windows = 0;
    constexpr uint32_t kSyncSearchMaxWindows = 4U;
    // Per-window rescan picks the wrong phase when a DMA window only contains
    // noise (short-distance bogus FF 00 00 hits) — the noise count beats the
    // real phase that has zero hits in that window. Lock to the calibrated
    // phase and let Spi2WirePacker maintain state across windows.
    const bool rescan_each_window = false;
    // Bring-up: bypass FrameAssembler (no VBLANK lock yet) and push packed
    // bytes straight into frame_buf_. Once a full sensor-sized buffer fills,
    // emit it as a YVYU frame so camu_uart_viewer.py can render whatever
    // bytes the wire is actually carrying.
    constexpr bool kRawPassthrough = false;
    constexpr bool kEmitRawDmaBuffer = false;
    size_t raw_accum = 0;
    write_status("qspi stream: LCD_CAM sampler, D0=low D1=high MSB-first");
    if (kRawPassthrough) {
        write_status("raw-passthrough: dumping packed bytes as 640x480 YVYU frames");
    }

    while (true) {
        DvpProbeDone done;
        if (xQueueReceive(ctx.done_queue, &done, pdMS_TO_TICKS(2000)) != pdTRUE) {
            write_status("dvp stream: timeout — restarting hardware");
            restart_cam_hw(hw, pclk_invert);
            // Reset packer and assembler so we re-sync cleanly
            packer.reset_state();
            assembler = FrameAssembler{};
            assembler.out     = frame_buf_;
            assembler.out_cap = kSensorFrameBytes;
            first_buf = true;
            vTaskDelay(1);
            continue;
        }

        size_t raw_len = std::min(done.received, ctx.buffer_len);
        const uint8_t *raw = done.buffer;

        // On the very first buffer of a streaming session,
        // skip `phase_skip` raw bytes so we start aligned.
        if (!rescan_each_window && first_buf && phase_skip > 0) {
            if (phase_skip >= raw_len) {
                vTaskDelay(1);
                continue;
            }
            raw     += phase_skip;
            raw_len -= phase_skip;
            first_buf = false;
        } else {
            first_buf = false;
        }

        size_t packed_len = 0;
        SyncCounts sc = {};
        if (rescan_each_window) {
            // Wire order is fixed by the logic analyzer. Only search byte
            // phase because each LCD_CAM DMA window can start at any 2-bit
            // symbol boundary.
            uint8_t best_phase = best.phase;
            for (uint8_t phase = 0; phase < 4; ++phase) {
                size_t plen = pack_raw(raw, raw_len, packed_buf_,
                                       kGc032aWireSwap,
                                       kGc032aWireLsbFirst,
                                       phase);
                SyncCounts candidate = scan_packed_syncs(packed_buf_, plen);
                if (candidate.exact() > sc.exact() ||
                    (candidate.exact() == sc.exact() &&
                     (candidate.score > sc.score ||
                      (candidate.score == sc.score &&
                       candidate.prefix > sc.prefix)))) {
                    sc = candidate;
                    best_phase = phase;
                }
            }
            packed_len = pack_raw(raw, raw_len, packed_buf_,
                                  kGc032aWireSwap,
                                  kGc032aWireLsbFirst,
                                  best_phase);
        } else {
            packed_len = packer.push(raw, raw_len, packed_buf_,
                                     kDvpRawBytes / 4U);
            sc = scan_packed_syncs(packed_buf_, packed_len);
        }
        diag_counts.fs += sc.fs;
        diag_counts.ls += sc.ls;
        diag_counts.le += sc.le;
        diag_counts.fe += sc.fe;

        if (sync_span_dump_count < kSyncSpanDumpMax && packed_len > 0) {
            char span[192];
            size_t off = std::snprintf(
                span, sizeof(span), "sync_span window=%lu:",
                static_cast<unsigned long>(sync_span_dump_count));
            uint32_t win = 0;
            int prev = -1;
            uint32_t shown = 0;
            for (size_t i = 0; i < packed_len && shown < 12U; ++i) {
                win = (win << 8U) | packed_buf_[i];
                if (i < 3U) continue;
                const char *name = nullptr;
                if (win == kBlankingSavSync) name = "BSAV";
                else if (win == kActiveSavSync) name = "ASAV";
                else if (win == kActiveEavSync) name = "AEAV";
                else if (win == kBlankingEavSync) name = "BEAV";
                if (!name) continue;
                int pos = static_cast<int>(i - 3U);
                int delta = (prev < 0) ? -1 : (pos - prev);
                off += std::snprintf(span + off, sizeof(span) - off,
                                     " %s@%d(+%d)", name, pos, delta);
                prev = pos;
                ++shown;
            }
            write_status(span);
            ++sync_span_dump_count;
        }

        if (++diag_windows >= 8U || sc.fs || sc.ls || sc.le || sc.fe) {
            char line[64];
            std::snprintf(line, sizeof(line), "bsav/asav/aeav/beav=%lu/%lu/%lu/%lu",
                          static_cast<unsigned long>(diag_counts.fs),
                          static_cast<unsigned long>(diag_counts.ls),
                          static_cast<unsigned long>(diag_counts.le),
                          static_cast<unsigned long>(diag_counts.fe));
            write_status(line);
            bool all_zero = diag_counts.fs == 0 && diag_counts.ls == 0 &&
                            diag_counts.le == 0 && diag_counts.fe == 0;
            diag_windows = 0;
            diag_counts = {};
            if (all_zero) {
                ++zero_diag_periods;
            } else {
                zero_diag_periods = 0;
            }
            if (zero_diag_periods >= 8U) {
                pclk_invert = !pclk_invert;
                std::snprintf(line, sizeof(line), "dvp pclk invert=%u",
                              pclk_invert ? 1U : 0U);
                write_status(line);
                restart_cam_hw(hw, pclk_invert);
                packer.reset_state();
                assembler = FrameAssembler{};
                assembler.out = frame_buf_;
                assembler.out_cap = kSensorFrameBytes;
                first_buf = true;
                zero_diag_periods = 0;
            }
        }

        // Raw DMA stats so we can see whether D0/D1 are actually
        // toggling and where in the buffer real data sits.
        bool want_raw_dump = (raw_dump_count < kRawDumpMax) &&
                             (raw_dump_period++ % kRawDumpPeriod == 0U);
        if (want_raw_dump && raw_len >= 64U) {
            uint32_t hist[4] = {0, 0, 0, 0};
            uint32_t bit0 = 0, bit1 = 0, top6 = 0;
            size_t   first_nz = SIZE_MAX, last_nz = 0;
            for (size_t i = 0; i < raw_len; ++i) {
                uint8_t v = done.buffer[i];
                ++hist[v & 0x03U];
                if (v & 0x01U) ++bit0;
                if (v & 0x02U) ++bit1;
                if (v & 0xFCU) ++top6;
                if (v != 0) {
                    if (first_nz == SIZE_MAX) first_nz = i;
                    last_nz = i;
                }
            }
            ESP_LOGI(TAG,
                     "raw stats len=%u  00=%lu 01=%lu 02=%lu 03=%lu  "
                     "d0=%lu d1=%lu top6=%lu  nz=[%u..%u]",
                     static_cast<unsigned>(raw_len),
                     static_cast<unsigned long>(hist[0]),
                     static_cast<unsigned long>(hist[1]),
                     static_cast<unsigned long>(hist[2]),
                     static_cast<unsigned long>(hist[3]),
                     static_cast<unsigned long>(bit0),
                     static_cast<unsigned long>(bit1),
                     static_cast<unsigned long>(top6),
                     static_cast<unsigned>(first_nz == SIZE_MAX ? 0 : first_nz),
                     static_cast<unsigned>(last_nz));
            // Dump 64 bytes starting at the first non-zero region (or 0)
            size_t start = (first_nz == SIZE_MAX) ? 0 : first_nz;
            if (start + 64 > raw_len) start = raw_len - 64;
            char dump[3 * 64 + 8];
            size_t off = 0;
            for (size_t i = 0; i < 64U; ++i) {
                off += std::snprintf(dump + off, sizeof(dump) - off,
                                     "%02x ", done.buffer[start + i]);
            }
            ESP_LOGI(TAG, "raw[%u..]: %s", static_cast<unsigned>(start), dump);
            ++raw_dump_count;
        }

        // Brute-force search for FF 00 00 in 8 different bit-packing
        // interpretations of the raw buffer.  Wait for a buffer with
        // enough D0/D1 activity before searching — sync codes won't
        // appear in idle/blanking-only windows.
        uint32_t d_active = 0;
        if (sync_search_windows < kSyncSearchMaxWindows && raw_len >= 1024U) {
            for (size_t i = 0; i < raw_len; ++i) {
                if (done.buffer[i] & 0x03U) ++d_active;
            }
        }
        if (sync_search_windows < kSyncSearchMaxWindows &&
            raw_len >= 1024U && d_active >= 800U) {
            struct PackMode { const char *name; bool one_lane; bool use_d0;
                              bool msb_first; bool invert; };
            static const PackMode modes[] = {
                {"d1_msb",     true,  false, true,  false},
                {"d1_lsb",     true,  false, false, false},
                {"d0_msb",     true,  true,  true,  false},
                {"d0_lsb",     true,  true,  false, false},
                {"2b_d0low_d1high_m", false, false, true,  false},
                {"2b_d0low_d1high_l", false, false, false, false},
                {"2b_d0high_d1low_m", false, true,  true,  false},
                {"2b_d0high_d1low_l", false, true,  false, false},
                {"d1_msb_inv", true,  false, true,  true},
                {"d0_msb_inv", true,  true,  true,  true},
            };
            uint8_t *buf = packed_buf_;
            ESP_LOGI(TAG, "sync_search window=%lu d_active=%lu raw_len=%u",
                     static_cast<unsigned long>(sync_search_windows),
                     static_cast<unsigned long>(d_active),
                     static_cast<unsigned>(raw_len));
            for (const auto &m : modes) {
                size_t out_len = 0;
                uint8_t cur = 0;
                uint8_t phase = 0;
                size_t step_bits = m.one_lane ? 1U : 2U;
                for (size_t i = 0; i < raw_len; ++i) {
                    uint8_t s = done.buffer[i];
                    if (m.invert) s = ~s;
                    uint8_t v;
                    if (m.one_lane) {
                        v = m.use_d0 ? (s & 0x01U) : ((s >> 1) & 0x01U);
                    } else {
                        // 2-bit: D0 low / D1 high vs swapped.
                        v = m.use_d0
                              ? static_cast<uint8_t>(((s & 0x01U) << 1) | ((s >> 1) & 0x01U))
                              : static_cast<uint8_t>(s & 0x03U);
                    }
                    if (m.msb_first) {
                        cur = static_cast<uint8_t>(cur << step_bits) |
                              static_cast<uint8_t>(v);
                    } else {
                        cur = static_cast<uint8_t>(cur >> step_bits) |
                              static_cast<uint8_t>(v << (8U - step_bits));
                    }
                    phase = static_cast<uint8_t>(phase + step_bits);
                    if (phase >= 8U) {
                        if (out_len < kDvpRawBytes / 4U) buf[out_len++] = cur;
                        cur = 0;
                        phase = 0;
                    }
                }
                // Search for FF 00 00 anywhere in `buf`
                int first_match = -1;
                uint32_t prefix_count = 0;
                uint32_t fs = 0, ls = 0, le = 0, fe = 0;
                uint8_t tails[8] = {};
                uint32_t tail_count = 0;
                uint32_t win = 0;
                for (size_t i = 0; i < out_len; ++i) {
                    win = ((win << 8) | buf[i]) & 0xFFFFFFFFu;
                    if ((win & 0xFFFFFF00u) == 0xFF000000u) {
                        ++prefix_count;
                        if (first_match < 0) first_match = static_cast<int>(i - 3);
                        if (tail_count < sizeof(tails)) {
                            tails[tail_count++] = static_cast<uint8_t>(win & 0xFFU);
                        }
                    }
                    if (win == kBlankingSavSync) ++fs;
                    if (win == kActiveSavSync)   ++ls;
                    if (win == kActiveEavSync)   ++le;
                    if (win == kBlankingEavSync) ++fe;
                }
                char tail_line[3 * sizeof(tails) + 1] = {};
                size_t tail_off = 0;
                for (uint32_t t = 0; t < tail_count; ++t) {
                    tail_off += std::snprintf(tail_line + tail_off,
                                              sizeof(tail_line) - tail_off,
                                              "%s%02x", t ? "," : "", tails[t]);
                }
                ESP_LOGI(TAG,
                         "sync_search %s: out=%u prefix=FF0000xx count=%lu first=%d tails=%s bsav/asav/aeav/beav=%lu/%lu/%lu/%lu",
                         m.name, static_cast<unsigned>(out_len),
                         static_cast<unsigned long>(prefix_count), first_match,
                         tail_count ? tail_line : "-",
                         static_cast<unsigned long>(fs),
                         static_cast<unsigned long>(ls),
                         static_cast<unsigned long>(le),
                         static_cast<unsigned long>(fe));
            }
            ++sync_search_windows;
        }
        if (kRawPassthrough && (packed_len > 0 || kEmitRawDmaBuffer)) {
            size_t remaining = kSensorFrameBytes - raw_accum;
            const uint8_t *src = kEmitRawDmaBuffer ? raw : packed_buf_;
            size_t        src_len = kEmitRawDmaBuffer ? raw_len : packed_len;
            size_t copy = std::min(src_len, remaining);
            std::memcpy(frame_buf_ + raw_accum, src, copy);
            raw_accum += copy;
            if (raw_accum >= kSensorFrameBytes) {
                char line[96];
                std::snprintf(line, sizeof(line),
                              "raw-passthrough emit seq=%lu bytes=%u",
                              static_cast<unsigned long>(frame_seq),
                              static_cast<unsigned>(kSensorFrameBytes));
                write_status(line);
                emit_frame(frame_seq++, frame_buf_, kSensorFrameBytes,
                           kSensorWidth, kSensorHeight);
                raw_accum = 0;
            }
            vTaskDelay(1);
            continue;
        }

        for (size_t i = 0; i < packed_len; ++i) {
            assembler.feed(packed_buf_[i]);
        }

        if (assembler.done) {
            // Log diagnostics
            char line[160];
            std::snprintf(line, sizeof(line),
                          "frame seq=%lu lines=%lu bytes=%u "
                          "bad_end=%lu restarts=%lu "
                          "bsav/asav/aeav/beav=%lu/%lu/%lu/%lu",
                          static_cast<unsigned long>(frame_seq),
                          static_cast<unsigned long>(assembler.lines),
                          static_cast<unsigned>(
                              std::min(assembler.out_len, assembler.out_cap)),
                          static_cast<unsigned long>(assembler.bad_line_ends),
                          static_cast<unsigned long>(assembler.restarts),
                          static_cast<unsigned long>(assembler.frame_syncs),
                          static_cast<unsigned long>(assembler.line_start_syncs),
                          static_cast<unsigned long>(assembler.line_end_syncs),
                          static_cast<unsigned long>(assembler.frame_end_syncs));
            write_status(line);

            size_t emit = std::min(assembler.out_len, assembler.out_cap);
            if (emit > 0 && assembler.lines > 0) {
                emit_frame(frame_seq, frame_buf_, emit,
                           kSensorWidth, assembler.lines);
            }
            ++frame_seq;

            // Reset assembler for next frame.
            // Do NOT reset packer — it must keep its phase.
            assembler = FrameAssembler{};
            assembler.out     = frame_buf_;
            assembler.out_cap = kSensorFrameBytes;
        }

        vTaskDelay(1);
    }
}

// =============================================================
// Hardware init helpers (unchanged logic, cleaned up)
// =============================================================

esp_err_t CameraUartStreamer::start_mclk()
{
    if (mclk_started_) return ESP_OK;
    ledc_timer_config_t timer = {
        .speed_mode      = kMclkSpeedMode,
        .duty_resolution = kMclkDutyResolution,
        .timer_num       = kMclkTimer,
        .freq_hz         = APP_GC032A_MCLK_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
        .deconfigure     = false,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "mclk timer");
    ledc_channel_config_t ch = {
        .gpio_num   = BSP_GC032A_MCLK_GPIO,
        .speed_mode = kMclkSpeedMode,
        .channel    = kMclkChannel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = kMclkTimer,
        .duty       = kMclkDuty50Percent,
        .hpoint     = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch), TAG, "mclk channel");
    mclk_started_ = true;
    return ESP_OK;
}

esp_err_t CameraUartStreamer::power_on_camera()
{
    gpio_config_t pwr = {
        .pin_bit_mask = 1ULL << BSP_CAMERA_PWR_EN_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&pwr), TAG, "pwr_en");
    gpio_set_level(BSP_CAMERA_PWR_EN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_config_t pwdn = {
        .pin_bit_mask = 1ULL << BSP_GC032A_PWDN_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&pwdn), TAG, "pwdn");
    gpio_set_level(BSP_GC032A_PWDN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(BSP_GC032A_PWDN_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    gpio_config_t in = {
        .pin_bit_mask = (1ULL << BSP_GC032A_SPI_CLK_GPIO) |
                        (1ULL << BSP_GC032A_DATA0_GPIO)    |
                        (1ULL << BSP_GC032A_DATA1_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    return gpio_config(&in);
}

esp_err_t CameraUartStreamer::attach_gc032a()
{
    if (gc032a_) return ESP_OK;
    ESP_RETURN_ON_ERROR(select_gc032a_address(), TAG, "select addr");
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) { ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c init"); bus = bsp_i2c_bus(); }
    if (!bus) return ESP_ERR_INVALID_STATE;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = gc032a_addr_,
        .scl_speed_hz    = BSP_I2C0_FREQ_HZ,
    };
    esp_err_t e = i2c_master_bus_add_device(bus, &dev_cfg, &gc032a_);
    if (e != ESP_OK) gc032a_ = nullptr;
    return e;
}

esp_err_t CameraUartStreamer::select_gc032a_address()
{
    if (gc032a_addr_ != 0) return ESP_OK;
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) { ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c init"); bus = bsp_i2c_bus(); }
    if (!bus) return ESP_ERR_INVALID_STATE;

    constexpr uint8_t candidates[] = {
        APP_GC032A_I2C_ADDR, 0x21, 0x31, 0x3C, 0x42, 0x5C, 0x6E,
    };
    for (int attempt = 0; attempt < 3; ++attempt) {
        for (uint8_t addr : candidates) {
            if (addr == 0 || addr >= 0x78) continue;
            if (i2c_master_probe(bus, addr, 50) != ESP_OK) continue;

            i2c_master_dev_handle_t dev = nullptr;
            i2c_device_config_t cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address  = addr,
                .scl_speed_hz    = BSP_I2C0_FREQ_HZ,
            };
            if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK) continue;
            uint8_t idh = 0, idl = 0, reg = kGc032aIdHighReg;
            esp_err_t e0 = i2c_master_transmit_receive(dev, &reg, 1, &idh, 1, 100);
            reg = kGc032aIdLowReg;
            esp_err_t e1 = i2c_master_transmit_receive(dev, &reg, 1, &idl, 1, 100);
            i2c_master_bus_rm_device(dev);
            if (e0 == ESP_OK && e1 == ESP_OK &&
                (uint16_t)((idh << 8) | idl) == kGc032aExpectedId) {
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
    return i2c_master_transmit(gc032a_, buf, 2, 100);
}

esp_err_t CameraUartStreamer::init_gc032a_sensor()
{
    ESP_RETURN_ON_ERROR(attach_gc032a(), TAG, "attach");
    uint8_t idh = 0, idl = 0;
    ESP_RETURN_ON_ERROR(gc032a_read_reg(kGc032aIdHighReg, &idh), TAG, "id high");
    ESP_RETURN_ON_ERROR(gc032a_read_reg(kGc032aIdLowReg,  &idl), TAG, "id low");
    if ((uint16_t)((idh << 8) | idl) != kGc032aExpectedId) return ESP_ERR_NOT_FOUND;

    for (const auto &rv : kGc032aInitRegs) {
        ESP_RETURN_ON_ERROR(gc032a_write_reg(rv.reg, rv.val), TAG, "init reg");
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    char line[96];
    std::snprintf(line, sizeof(line),
                  "gc032a: sensor ready, %u regs written",
                  static_cast<unsigned>(sizeof(kGc032aInitRegs) /
                                        sizeof(kGc032aInitRegs[0])));
    write_status(line);
    dump_gc032a_spi_regs();
    return ESP_OK;
}

void CameraUartStreamer::dump_gc032a_spi_regs()
{
    if (gc032a_write_reg(0xfe, 0x03) != ESP_OK) {
        write_status("gc032a spi regs: select page failed");
        return;
    }
    constexpr uint8_t regs[] = {
        0x53, 0x55, 0x5a, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67
    };
    uint8_t vals[sizeof(regs)] = {};
    for (size_t i = 0; i < sizeof(regs); ++i) {
        if (gc032a_read_reg(regs[i], &vals[i]) != ESP_OK) {
            write_status("gc032a spi regs: read failed");
            gc032a_write_reg(0xfe, 0x00);
            return;
        }
    }
    char line[176];
    std::snprintf(line, sizeof(line),
                  "gc032a spi regs: 53=%02x 55=%02x 5a=%02x 60=%02x 61=%02x 62=%02x 63=%02x 64=%02x 65=%02x 66=%02x 67=%02x",
                  vals[0], vals[1], vals[2], vals[3], vals[4],
                  vals[5], vals[6], vals[7], vals[8], vals[9], vals[10]);
    write_status(line);
    gc032a_write_reg(0xfe, 0x00);
}

void CameraUartStreamer::probe_i2c()
{
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) { write_status("i2c: bus unavailable"); return; }
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
        if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
            char line[64];
            std::snprintf(line, sizeof(line), "i2c ack: 0x%02X", addr);
            write_status(line);
            ++found;
        }
    }
    char line[64];
    std::snprintf(line, sizeof(line), "i2c scan: %d device(s)", found);
    write_status(line);
}

// =============================================================
// UART packet output
// =============================================================

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

void CameraUartStreamer::write_packet(uint8_t type, uint32_t seq,
                                      const void *payload, uint32_t len)
{
    PacketHeader hdr = {
        .magic      = {'C', 'A', 'M', 'U'},
        .type       = type,
        .flags      = 0,
        .header_len = sizeof(PacketHeader),
        .seq        = seq,
        .payload_len = len,
        .checksum   = fnv1a(payload, len),
    };
    uart_write_bytes(kUart, &hdr, sizeof(hdr));
    if (payload && len) uart_write_bytes(kUart, payload, len);
}

void CameraUartStreamer::write_status(const char *msg)
{
    ESP_LOGI(TAG, "%s", msg);
    write_packet(PKT_STATUS, 0, msg, std::strlen(msg));
}
