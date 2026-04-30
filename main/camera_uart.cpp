// ============================================================
// camera_uart.cpp - GC032A packet stream capture over LCD_CAM
//
// 中文说明：
// GC032A 当前输出 packet 数据流。1SDR 模式只采 D0，每 8 个 sample 拼 1 byte；
// 2-bit 模式采 D0/D1，每 4 个 sample 拼 1 byte。
// 因为 DMA 起点不一定落在 GC032A 字节边界，采样过程中会同时尝试
// phase。只有找到 FF FF FF 01 帧头后才固定 phase 并开始保存，
// 保存的第一个字节序列就是 FF FF FF 01，最后通过 UART2 输出。
// ============================================================

#include "camera_uart.hpp"

#include <algorithm>
#include <cinttypes>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_dvp.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "hal/cam_ll.h"
#include "soc/gpio_pins.h"
#include "soc/gpio_sig_map.h"
#include "soc/lcd_cam_struct.h"
#include "app_config.h"
#include "bsp.h"
#include "gc032a_regs.hpp"

namespace {

constexpr const char *TAG = "camera_uart";
constexpr uart_port_t kUart = UART_NUM_2;

constexpr ledc_mode_t      kMclkSpeedMode      = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t     kMclkTimer          = LEDC_TIMER_0;
constexpr ledc_channel_t   kMclkChannel        = LEDC_CHANNEL_0;
constexpr ledc_timer_bit_t kMclkDutyResolution = LEDC_TIMER_1_BIT;
constexpr ledc_clk_cfg_t   kMclkClockSource    = LEDC_USE_APB_CLK;
constexpr uint32_t         kMclkDuty50Percent  = 1U;

constexpr uint8_t  kGc032aIdHighReg = 0xF0;
constexpr uint8_t  kGc032aIdLowReg  = 0xF1;
constexpr uint16_t kGc032aExpectedId = 0x232A;

// LCD_CAM DMA 每个 byte 只用低位数据：1SDR 用 bit0=D0，2-bit 用 bit0=D0/bit1=D1。
// DMA buffer 必须在 internal SRAM；一帧 packet 数据另存到 PSRAM。
constexpr size_t kDvpRawBytes = 64U * 1024U;
constexpr size_t kDvpBufferCount = 4U;
constexpr bool kUse1Sdr = APP_GC032A_SPI_1SDR_ENABLE != 0;
constexpr uint8_t kSampleBits = kUse1Sdr ? 1U : 2U;
constexpr uint8_t kSamplesPerByte = static_cast<uint8_t>(8U / kSampleBits);
constexpr uint8_t kPhaseCount = kSamplesPerByte;
constexpr uint8_t kSampleMask = static_cast<uint8_t>((1U << kSampleBits) - 1U);
constexpr uint8_t kPreferredPhase = kUse1Sdr ? 0U : 3U;
constexpr uint32_t kSyncSearchWindowsPerPhase = 128U;
constexpr uint32_t kIdleDelayEveryWindows = 16U;
constexpr uint8_t kSyncFrameStart = 0x01U;
constexpr uint8_t kSyncFrameEnd = 0x00U;
constexpr uint32_t kSyncPrefix = 0xffffff00UL;
constexpr uint32_t kFrameStartWord = kSyncPrefix | kSyncFrameStart;
constexpr uint32_t kFrameEndWord = kSyncPrefix | kSyncFrameEnd;

struct DvpProbeDone {
    uint8_t *buffer = nullptr;
    size_t received = 0;
};

struct DvpProbeContext {
    uint8_t *buffers[kDvpBufferCount] = {};
    size_t buffer_len = 0;
    uint32_t next = 0;
    uint32_t queue_drops = 0;
    portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
    QueueHandle_t done_queue = nullptr;
};

static bool IRAM_ATTR dvp_get_new_trans(esp_cam_ctlr_handle_t,
                                        esp_cam_ctlr_trans_t *trans,
                                        void *user_data)
{
    auto *ctx = static_cast<DvpProbeContext *>(user_data);
    uint32_t idx = ctx->next++ % kDvpBufferCount;
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
        .buffer = static_cast<uint8_t *>(trans->buffer),
        .received = trans->received_size,
    };
    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(ctx->done_queue, &done, &woken) != pdTRUE) {
        portENTER_CRITICAL_ISR(&ctx->lock);
        ++ctx->queue_drops;
        portEXIT_CRITICAL_ISR(&ctx->lock);
    }
    return woken == pdTRUE;
}

static void restart_cam_hw(lcd_cam_dev_t *hw, bool pclk_invert)
{
    // LCD_CAM 偶尔收不到 DMA 完成事件时，复位 FIFO 并重新打开采样。
    cam_ll_stop(hw);
    cam_ll_fifo_reset(hw);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT,
                                   CAM_V_SYNC_IDX, false);
    cam_ll_enable_invert_pclk(hw, pclk_invert);
    cam_ll_start(hw);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT,
                                   CAM_V_SYNC_IDX, false);
}

struct PhaseSyncCandidate {
    uint8_t skip_samples = 0;
    uint8_t byte = 0;
    uint8_t phase = 0;
    uint32_t sync_word = 0;

    void reset(uint8_t initial_skip)
    {
        skip_samples = initial_skip;
        byte = 0;
        phase = 0;
        sync_word = 0;
    }

    bool push_sample(uint8_t sample, uint8_t *out)
    {
        if (skip_samples > 0U) {
            --skip_samples;
            return false;
        }
        // GC032A 当前配置是 LSB-first：第 1 个 sample 放在 byte 的低位。
        byte = static_cast<uint8_t>(byte | ((sample & kSampleMask) << (phase * kSampleBits)));
        if (++phase == kSamplesPerByte) {
            *out = byte;
            sync_word = (sync_word << 8U) | byte;
            byte = 0;
            phase = 0;
            return true;
        }
        return false;
    }
};

static inline uint8_t pack_lsb_samples(const uint8_t *raw)
{
    uint8_t out = 0;
    for (uint8_t i = 0; i < kSamplesPerByte; ++i) {
        out = static_cast<uint8_t>(out | ((raw[i] & kSampleMask) << (i * kSampleBits)));
    }
    return out;
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
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(kUart, 8192, 0, 0, nullptr, 0),
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
    ESP_LOGI(TAG, "gc032a bring-up: MCLK, PWDN, I2C init, UART image/raw output");
    probe_i2c();

    if (init_gc032a_sensor() != ESP_OK) {
        ESP_LOGE(TAG, "gc032a: sensor init failed");
        vTaskDelete(nullptr);
    }

    dvp_stream_loop();
}

void CameraUartStreamer::dvp_stream_loop()
{
    // 使用 LCD_CAM 的 DVP 输入采 GC032A 的 PCLK/D0/D1。
    // 1SDR 只接 D0；2-bit 模式接 D0/D1。LCD_CAM 按 RAW8 收，有效数据在低位。
    esp_cam_ctlr_dvp_pin_config_t pin_cfg = {};
    pin_cfg.data_width = CAM_CTLR_DATA_WIDTH_8;
    for (int i = 0; i < CAM_DVP_DATA_SIG_NUM; ++i) {
        pin_cfg.data_io[i] = GPIO_NUM_NC;
    }
    pin_cfg.data_io[0] = BSP_GC032A_DATA0_GPIO;
#if !APP_GC032A_SPI_1SDR_ENABLE
    pin_cfg.data_io[1] = BSP_GC032A_DATA1_GPIO;
#endif
    pin_cfg.vsync_io = GPIO_NUM_NC;
    pin_cfg.de_io = GPIO_NUM_NC;
    pin_cfg.pclk_io = BSP_GC032A_SPI_CLK_GPIO;
    pin_cfg.xclk_io = GPIO_NUM_NC;

    esp_cam_ctlr_dvp_config_t dvp_cfg = {
        .ctlr_id = 0,
        .clk_src = CAM_CLK_SRC_PLL240M,
        .h_res = 256,
        .v_res = static_cast<uint32_t>(kDvpRawBytes / 256U),
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
    if (esp_cam_new_dvp_ctlr(&dvp_cfg, &cam) != ESP_OK) {
        ESP_LOGE(TAG, "dvp: controller init failed");
        vTaskDelete(nullptr);
    }

    size_t capture_cap = APP_CAMERA_PACKED_CAPTURE_BYTES;
    size_t capture_len = 0;
    uint8_t *capture = static_cast<uint8_t *>(
        heap_caps_malloc(capture_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!capture) {
        ESP_LOGE(TAG, "packed capture PSRAM alloc failed: %u bytes, largest_psram=%u",
                 static_cast<unsigned>(capture_cap),
                 static_cast<unsigned>(
                     heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM |
                                                      MALLOC_CAP_8BIT)));
        gpio_set_level(BSP_GC032A_PWDN_GPIO, 1);
        vTaskDelete(nullptr);
    }
    ESP_LOGI(TAG, "packed one-frame PSRAM capture buffer=%u bytes",
             static_cast<unsigned>(capture_cap));

    DvpProbeContext ctx = {};
    ctx.buffer_len = kDvpRawBytes;
    ctx.done_queue = xQueueCreate(kDvpBufferCount, sizeof(DvpProbeDone));
    // DMA buffer 必须在 internal SRAM，不能放 PSRAM。
    uint32_t buf_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
    for (size_t i = 0; i < kDvpBufferCount; ++i) {
        ctx.buffers[i] = static_cast<uint8_t *>(
            esp_cam_ctlr_alloc_buffer(cam, ctx.buffer_len, buf_caps));
    }
    bool dma_alloc_ok = ctx.done_queue != nullptr;
    for (size_t i = 0; i < kDvpBufferCount; ++i) {
        dma_alloc_ok = dma_alloc_ok && ctx.buffers[i] != nullptr;
    }
    if (!dma_alloc_ok) {
        ESP_LOGE(TAG, "dvp: DMA buffer alloc failed, window=%u count=%u largest_dma=%u",
                 static_cast<unsigned>(ctx.buffer_len),
                 static_cast<unsigned>(kDvpBufferCount),
                 static_cast<unsigned>(
                     heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL |
                                                      MALLOC_CAP_DMA |
                                                      MALLOC_CAP_8BIT)));
        heap_caps_free(capture);
        vTaskDelete(nullptr);
    }

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = dvp_get_new_trans,
        .on_trans_finished = dvp_trans_finished,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(cam, &cbs, &ctx));
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam));
    ESP_ERROR_CHECK(esp_cam_ctlr_start(cam));

    auto *hw = CAM_LL_GET_HW(0);
    bool pclk_invert = APP_CAMERA_DVP_PCLK_INVERT != 0;

    cam_ll_stop(hw);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, CAM_V_SYNC_IDX, false);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, CAM_H_ENABLE_IDX, false);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, CAM_H_SYNC_IDX, false);
    cam_ll_set_vh_de_mode(hw, false);
    cam_ll_enable_vsync_generate_eof(hw, false);
    cam_ll_set_recv_data_bytelen(hw, static_cast<uint32_t>(kDvpRawBytes - 1U));
    cam_ll_enable_invert_pclk(hw, pclk_invert);
    cam_ll_fifo_reset(hw);
    cam_ll_start(hw);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, CAM_V_SYNC_IDX, false);

    ESP_LOGI(TAG, "packed binary stream: UART%d %u baud, mode=%s",
             static_cast<int>(kUart), static_cast<unsigned>(APP_CAMERA_UART_BAUD),
             kUse1Sdr ? "1SDR D0-only" : "2-bit D0/D1");

    uint8_t spi52 = 0xff;
    if (gc032a_write_reg(0xfe, 0x03) == ESP_OK &&
        gc032a_read_reg(0x52, &spi52) == ESP_OK) {
        ESP_LOGI(TAG, "gc032a spi reg 52=%02x, default pack order=%s",
                 spi52, (spi52 & 0x80U) ? "MSB-first" : "LSB-first");
    } else {
        ESP_LOGW(TAG, "gc032a spi reg 52 read failed, defaulting MSB-first");
    }
    gc032a_write_reg(0xfe, 0x00);

    PhaseSyncCandidate phases[kPhaseCount];
    for (uint8_t i = 0; i < kPhaseCount; ++i) {
        phases[i].reset(i);
    }
    uint32_t windows = 0;
    uint32_t timeouts = 0;
    size_t samples_seen = 0;
    bool frame_locked = false;
    bool frame_done = false;
    uint8_t locked_phase = 0;
    uint8_t search_phase = kPreferredPhase;
    ESP_LOGI(TAG, "%s online sync: search phase=%u/%u, lock on FF FF FF 01, LSB-first",
             kUse1Sdr ? "1SDR" : "2-bit",
             static_cast<unsigned>(search_phase),
             static_cast<unsigned>(kPhaseCount));

    while (!frame_done) {
        DvpProbeDone done;
        if (xQueueReceive(ctx.done_queue, &done, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "dvp stream timeout, restarting LCD_CAM");
            if (frame_locked) {
                ESP_LOGW(TAG, "timeout after frame start, output partial frame");
                frame_done = true;
                continue;
            }
            restart_cam_hw(hw, pclk_invert);
            if (!frame_locked) {
                for (uint8_t i = 0; i < kPhaseCount; ++i) {
                    phases[i].reset(i);
                }
                search_phase = static_cast<uint8_t>((search_phase + 1U) % kPhaseCount);
                ESP_LOGW(TAG, "sync not found before timeout, try phase=%u",
                         static_cast<unsigned>(search_phase));
            }
            ++timeouts;
            continue;
        }

        size_t raw_len = std::min(done.received, ctx.buffer_len);
        const uint8_t *raw = done.buffer;
        for (size_t i = 0; i < raw_len && !frame_done;) {
            // 每个 DMA byte 的 bit[1:0] 是一次 PCLK 上采到的 D1/D0。
            uint8_t out = 0;

            if (frame_locked) {
                if (phases[locked_phase].phase == 0U &&
                    phases[locked_phase].skip_samples == 0U &&
                    i + kSamplesPerByte <= raw_len) {
                    // 锁定 phase 后直接 pack，减少逐 sample 状态机开销。
                    out = pack_lsb_samples(raw + i);
                    phases[locked_phase].sync_word =
                        (phases[locked_phase].sync_word << 8U) | out;
                    i += kSamplesPerByte;
                } else {
                    if (!phases[locked_phase].push_sample(raw[i] & kSampleMask, &out)) {
                        ++i;
                        continue;
                    }
                    ++i;
                }

                if (capture_len < capture_cap) {
                    capture[capture_len++] = out;
                    if (phases[locked_phase].sync_word == kFrameEndWord &&
                        capture_len > 4U) {
                        frame_done = true;
                    }
                } else {
                    if (!frame_done) {
                        ESP_LOGW(TAG, "frame buffer full before FF FF FF 00");
                        frame_done = true;
                    }
                }
                continue;
            }

            // 未同步前不存储数据。为了跟上 DMA，只检查当前 phase；
            // 如果 2 秒内没有找到帧头，timeout 分支会切到下一个 phase。
            if (phases[search_phase].push_sample(raw[i] & kSampleMask, &out) &&
                phases[search_phase].sync_word == kFrameStartWord) {
                frame_locked = true;
                locked_phase = search_phase;
                capture[0] = 0xffU;
                capture[1] = 0xffU;
                capture[2] = 0xffU;
                capture[3] = kSyncFrameStart;
                capture_len = 4U;
                ESP_LOGI(TAG, "frame start locked: phase=%u samples_seen=%u",
                         static_cast<unsigned>(locked_phase),
                         static_cast<unsigned>(samples_seen + i + 1U));
            }
            ++i;
        }
        samples_seen += raw_len;

        if ((++windows % 128U) == 0U) {
            ESP_LOGI(TAG, "packed windows=%" PRIu32 " timeouts=%" PRIu32
                     " queue_drops=%" PRIu32 " locked=%u bytes=%u",
                     windows, timeouts, ctx.queue_drops,
                     frame_locked ? 1U : 0U,
                     static_cast<unsigned>(capture_len));
        }
        if (!frame_locked && (windows % kSyncSearchWindowsPerPhase) == 0U) {
            search_phase = static_cast<uint8_t>((search_phase + 1U) % kPhaseCount);
            phases[search_phase].reset(search_phase);
            ESP_LOGI(TAG, "sync search rotate phase=%u",
                     static_cast<unsigned>(search_phase));
        }
        if ((windows % kIdleDelayEveryWindows) == 0U) {
            vTaskDelay(1);
        } else {
            taskYIELD();
        }
    }

    cam_ll_stop(hw);
    gpio_set_level(BSP_GC032A_PWDN_GPIO, 1);
    ESP_LOGI(TAG, "frame capture complete: phase=%u samples=%u bytes=%u end=%u queue_drops=%u",
             static_cast<unsigned>(locked_phase),
             static_cast<unsigned>(samples_seen),
             static_cast<unsigned>(capture_len),
             phases[locked_phase].sync_word == kFrameEndWord ? 1U : 0U,
             static_cast<unsigned>(ctx.queue_drops));
    if (capture_len >= 16U) {
        ESP_LOGI(TAG,
                 "UART dump first16: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                 capture[0], capture[1], capture[2], capture[3],
                 capture[4], capture[5], capture[6], capture[7],
                 capture[8], capture[9], capture[10], capture[11],
                 capture[12], capture[13], capture[14], capture[15]);
    }

    for (size_t off = 0; off < capture_len;) {
        size_t chunk = std::min(static_cast<size_t>(APP_CAMERA_UART_CHUNK_BYTES),
                                capture_len - off);
        int written = uart_write_bytes(kUart, capture + off, chunk);
        if (written < 0 || static_cast<size_t>(written) != chunk) {
            ESP_LOGE(TAG, "UART write failed: off=%u requested=%u written=%d",
                     static_cast<unsigned>(off),
                     static_cast<unsigned>(chunk),
                     written);
            break;
        }
        off += chunk;
        vTaskDelay(1);
    }
    esp_err_t tx_done = uart_wait_tx_done(kUart, pdMS_TO_TICKS(30000));
    if (tx_done != ESP_OK) {
        ESP_LOGE(TAG, "UART wait tx done failed: %s", esp_err_to_name(tx_done));
    }
    heap_caps_free(capture);
    ESP_LOGI(TAG, "LCD_CAM stopped, GC032A PWDN asserted, UART output complete");
    vTaskDelete(nullptr);
}

esp_err_t CameraUartStreamer::start_mclk()
{
    if (mclk_started_) return ESP_OK;

    ledc_timer_config_t timer = {
        .speed_mode = kMclkSpeedMode,
        .duty_resolution = kMclkDutyResolution,
        .timer_num = kMclkTimer,
        .freq_hz = APP_GC032A_MCLK_HZ,
        .clk_cfg = kMclkClockSource,
        .deconfigure = false,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "mclk timer");

    ledc_channel_config_t ch = {
        .gpio_num = BSP_GC032A_MCLK_GPIO,
        .speed_mode = kMclkSpeedMode,
        .channel = kMclkChannel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = kMclkTimer,
        .duty = kMclkDuty50Percent,
        .hpoint = 0,
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
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&pwr), TAG, "pwr_en");
    gpio_set_level(BSP_CAMERA_PWR_EN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_config_t pwdn = {
        .pin_bit_mask = 1ULL << BSP_GC032A_PWDN_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&pwdn), TAG, "pwdn");
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
    ESP_RETURN_ON_ERROR(select_gc032a_address(), TAG, "select addr");

    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) {
        ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c init");
        bus = bsp_i2c_bus();
    }
    if (!bus) return ESP_ERR_INVALID_STATE;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = gc032a_addr_,
        .scl_speed_hz = BSP_I2C0_FREQ_HZ,
    };
    esp_err_t e = i2c_master_bus_add_device(bus, &dev_cfg, &gc032a_);
    if (e != ESP_OK) gc032a_ = nullptr;
    return e;
}

esp_err_t CameraUartStreamer::select_gc032a_address()
{
    if (gc032a_addr_ != 0) return ESP_OK;

    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) {
        ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c init");
        bus = bsp_i2c_bus();
    }
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
                .device_address = addr,
                .scl_speed_hz = BSP_I2C0_FREQ_HZ,
            };
            if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK) continue;

            uint8_t idh = 0, idl = 0, reg = kGc032aIdHighReg;
            esp_err_t e0 = i2c_master_transmit_receive(dev, &reg, 1, &idh, 1, 100);
            reg = kGc032aIdLowReg;
            esp_err_t e1 = i2c_master_transmit_receive(dev, &reg, 1, &idl, 1, 100);
            i2c_master_bus_rm_device(dev);
            if (e0 == ESP_OK && e1 == ESP_OK &&
                static_cast<uint16_t>((idh << 8) | idl) == kGc032aExpectedId) {
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
    for (const auto &rv : kGc032aInitRegs) {
        ESP_RETURN_ON_ERROR(gc032a_write_reg(rv.reg, rv.val), TAG, "init reg");
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    dump_gc032a_init_regs();
    dump_gc032a_spi_regs();
    return ESP_OK;
}

void CameraUartStreamer::dump_gc032a_init_regs()
{
    uint8_t page = 0;
    uint32_t checked = 0;
    uint32_t mismatches = 0;

    ESP_LOGI(TAG, "gc032a init regs readback begin");
    for (const auto &rv : kGc032aInitRegs) {
        if (rv.reg == 0xfe) {
            page = rv.val;
            if (gc032a_write_reg(0xfe, page) != ESP_OK) {
                ESP_LOGW(TAG, "gc032a readback: select page %02x failed", page);
                return;
            }
            ESP_LOGI(TAG, "gc032a readback: page=%02x", page);
            continue;
        }

        uint8_t actual = 0;
        esp_err_t err = gc032a_read_reg(rv.reg, &actual);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "gc032a readback: p%02x r%02x failed: %s",
                     page, rv.reg, esp_err_to_name(err));
            continue;
        }

        ++checked;
        if (actual != rv.val) ++mismatches;
        ESP_LOGI(TAG, "gc032a readback: p%02x r%02x=%02x expected=%02x%s",
                 page, rv.reg, actual, rv.val,
                 actual == rv.val ? "" : " mismatch");
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    gc032a_write_reg(0xfe, 0x00);
    ESP_LOGI(TAG, "gc032a init regs readback done: checked=%" PRIu32
                  " mismatches=%" PRIu32,
             checked, mismatches);
}

void CameraUartStreamer::dump_gc032a_spi_regs()
{
    if (gc032a_write_reg(0xfe, 0x03) != ESP_OK) {
        ESP_LOGW(TAG, "gc032a spi regs: select page failed");
        return;
    }

    constexpr uint8_t regs[] = {
        0x53, 0x55, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e,
        0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67,
    };
    uint8_t vals[sizeof(regs)] = {};
    for (size_t i = 0; i < sizeof(regs); ++i) {
        if (gc032a_read_reg(regs[i], &vals[i]) != ESP_OK) {
            ESP_LOGW(TAG, "gc032a spi regs: read failed");
            gc032a_write_reg(0xfe, 0x00);
            return;
        }
    }

    ESP_LOGI(TAG,
             "gc032a spi regs: 53=%02x 55=%02x 5a=%02x "
             "5b=%02x 5c=%02x 5d=%02x 5e=%02x "
             "60=%02x 61=%02x 62=%02x 63=%02x 64=%02x 65=%02x 66=%02x 67=%02x",
             vals[0], vals[1], vals[2],
             vals[3], vals[4], vals[5], vals[6],
             vals[7], vals[8], vals[9], vals[10],
             vals[11], vals[12], vals[13], vals[14]);
    gc032a_write_reg(0xfe, 0x00);
}

void CameraUartStreamer::probe_i2c()
{
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) {
        ESP_LOGW(TAG, "i2c: bus unavailable");
        return;
    }

    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
        if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "i2c ack: 0x%02X", addr);
            ++found;
        }
    }
    ESP_LOGI(TAG, "i2c scan: %d device(s)", found);
}

void CameraUartStreamer::write_status(const char *msg)
{
    ESP_LOGI(TAG, "%s", msg);
}
