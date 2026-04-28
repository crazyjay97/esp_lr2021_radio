// ============================================================
// camera_uart.cpp - LCD_CAM raw sampler for GC032A PCLK + D0/D1
//
// UART2 output is packed binary: every four LCD_CAM samples are packed into
// one 2-bit SPI byte so host-side hex views can see FF 00 00 xx sync codes.
// ============================================================

#include "camera_uart.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstdio>

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
constexpr uint32_t         kMclkDuty50Percent  = 1U;

constexpr uint8_t  kGc032aIdHighReg = 0xF0;
constexpr uint8_t  kGc032aIdLowReg  = 0xF1;
constexpr uint16_t kGc032aExpectedId = 0x232A;

// One DMA window. LCD_CAM stores one PCLK sample per byte, with D0/D1 in
// bits [1:0].
constexpr size_t kDvpRawBytes = 8U * 1024U;
constexpr size_t kPackedBytesPerWindow = kDvpRawBytes / 4U;
constexpr uint32_t kGc032aSyncPrefix = 0xFFFFFF00UL;
constexpr uint32_t kFrameStartSync = kGc032aSyncPrefix | 0x01U;
constexpr uint32_t kLineStartSync = kGc032aSyncPrefix | 0x02U;
constexpr uint32_t kLineEndSync = kGc032aSyncPrefix | 0x40U;
constexpr uint32_t kFrameEndSync = kGc032aSyncPrefix | 0x80U;

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
        .buffer = static_cast<uint8_t *>(trans->buffer),
        .received = trans->received_size,
    };
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(ctx->done_queue, &done, &woken);
    return woken == pdTRUE;
}

static void restart_cam_hw(lcd_cam_dev_t *hw, bool pclk_invert)
{
    cam_ll_stop(hw);
    cam_ll_fifo_reset(hw);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT,
                                   CAM_V_SYNC_IDX, false);
    cam_ll_enable_invert_pclk(hw, pclk_invert);
    cam_ll_start(hw);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT,
                                   CAM_V_SYNC_IDX, false);
}

struct Spi2BitPacker {
    bool swap = false;
    bool lsb_first = false;
    uint8_t byte = 0;
    uint8_t phase = 0;

    void reset()
    {
        byte = 0;
        phase = 0;
    }

    size_t push(const uint8_t *raw, size_t raw_len, uint8_t *out, size_t out_cap)
    {
        size_t out_len = 0;
        for (size_t i = 0; i < raw_len; ++i) {
            uint8_t pair = raw[i] & 0x03U; // D0=bit0, D1=bit1
            if (swap) {
                pair = static_cast<uint8_t>(((pair & 0x01U) << 1U) |
                                            ((pair & 0x02U) >> 1U));
            }
            uint8_t shift = lsb_first ? static_cast<uint8_t>(phase * 2U)
                                      : static_cast<uint8_t>((3U - phase) * 2U);
            byte = static_cast<uint8_t>(byte | (pair << shift));
            if (++phase == 4U) {
                if (out_len >= out_cap) {
                    reset();
                    return out_len;
                }
                out[out_len++] = byte;
                byte = 0;
                phase = 0;
            }
        }
        return out_len;
    }
};

struct PackedSyncScan {
    uint32_t prefix = 0;
    uint32_t exact = 0;
    uint32_t frame_start = 0;
    uint32_t tail_hist[256] = {};
    int first_prefix = -1;
    int first_exact = -1;
    int first_frame_start = -1;
};

static PackedSyncScan scan_packed_syncs(const uint8_t *data, size_t len)
{
    uint32_t win = 0;
    PackedSyncScan scan = {};
    for (size_t i = 0; i < len; ++i) {
        win = (win << 8U) | data[i];
        if (i < 3U) continue;
        if ((win & 0xFFFFFF00UL) == kGc032aSyncPrefix) {
            if (scan.first_prefix < 0) {
                scan.first_prefix = static_cast<int>(i - 3U);
            }
            ++scan.tail_hist[win & 0xffU];
            ++scan.prefix;
        }
        if (win == kFrameStartSync || win == kLineStartSync ||
            win == kLineEndSync || win == kFrameEndSync) {
            if (scan.first_exact < 0) {
                scan.first_exact = static_cast<int>(i - 3U);
            }
            ++scan.exact;
        }
        if (win == kFrameStartSync) {
            if (scan.first_frame_start < 0) {
                scan.first_frame_start = static_cast<int>(i - 3U);
            }
            ++scan.frame_start;
        }
    }
    return scan;
}

static void log_sync_tail_summary(const char *label, const PackedSyncScan &scan)
{
    char line[192];
    size_t off = std::snprintf(line, sizeof(line),
                               "%s prefix=%" PRIu32 " exact=%" PRIu32
                               " fs=%" PRIu32 " tails:",
                               label, scan.prefix, scan.exact, scan.frame_start);
    uint32_t shown = 0;
    for (uint32_t tail = 0; tail < 256U && shown < 12U; ++tail) {
        uint32_t count = scan.tail_hist[tail];
        if (count == 0U) continue;
        off += std::snprintf(line + off, sizeof(line) - off,
                             " %02x=%" PRIu32, static_cast<unsigned>(tail), count);
        ++shown;
    }
    ESP_LOGI(TAG, "%s", line);
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
    ESP_LOGI(TAG, "gc032a raw bring-up: MCLK, PWDN, I2C init, LCD_CAM raw hex UART");
    probe_i2c();

    if (init_gc032a_sensor() != ESP_OK) {
        ESP_LOGE(TAG, "gc032a: sensor init failed");
        vTaskDelete(nullptr);
    }

    dvp_stream_loop();
}

void CameraUartStreamer::dvp_stream_loop()
{
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

    DvpProbeContext ctx = {};
    ctx.buffer_len = kDvpRawBytes;
    ctx.done_queue = xQueueCreate(4, sizeof(DvpProbeDone));
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
        ESP_LOGE(TAG, "dvp: DMA buffer alloc failed");
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
    bool pclk_invert = false;

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

    ESP_LOGI(TAG, "packed binary stream: UART%d %u baud, D0=low D1=high",
             static_cast<int>(kUart), static_cast<unsigned>(APP_CAMERA_UART_BAUD));

    uint8_t spi52 = 0xff;
    if (gc032a_write_reg(0xfe, 0x03) == ESP_OK &&
        gc032a_read_reg(0x52, &spi52) == ESP_OK) {
        ESP_LOGI(TAG, "gc032a spi reg 52=%02x, default pack order=%s",
                 spi52, (spi52 & 0x80U) ? "MSB-first" : "LSB-first");
    } else {
        ESP_LOGW(TAG, "gc032a spi reg 52 read failed, defaulting MSB-first");
    }
    gc032a_write_reg(0xfe, 0x00);

    uint8_t packed[kPackedBytesPerWindow];
    Spi2BitPacker packer;
    uint32_t windows = 0;
    uint32_t timeouts = 0;
    size_t samples_sent = 0;
    bool phase_locked = false;
    uint32_t lock_attempts = 0;
    bool default_lsb_first = (spi52 != 0xff) && ((spi52 & 0x80U) == 0U);

    while (true) {
        DvpProbeDone done;
        if (xQueueReceive(ctx.done_queue, &done, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "dvp stream timeout, restarting LCD_CAM");
            restart_cam_hw(hw, pclk_invert);
            phase_locked = false;
            packer.reset();
            ++timeouts;
            continue;
        }

        size_t raw_len = std::min(done.received, ctx.buffer_len);
        const uint8_t *raw = done.buffer;
        if (!phase_locked) {
            PackedSyncScan best_scan = {};
            uint8_t best_phase = 0;
            bool best_swap = false;
            bool best_lsb = default_lsb_first;
            int best_first_sync = -1;
            struct PackMode {
                bool swap;
                bool lsb;
            };
            PackMode modes[] = {
                {false, default_lsb_first},
                {true,  default_lsb_first},
                {false, !default_lsb_first},
                {true,  !default_lsb_first},
            };
            for (uint8_t phase = 0; phase < 4U; ++phase) {
                if (phase >= raw_len) break;
                for (const auto &mode : modes) {
                    Spi2BitPacker trial;
                    trial.swap = mode.swap;
                    trial.lsb_first = mode.lsb;
                    size_t packed_len = trial.push(raw + phase, raw_len - phase,
                                                   packed, sizeof(packed));
                    PackedSyncScan scan = scan_packed_syncs(packed, packed_len);
                    if (lock_attempts == 0U) {
                        char label[64];
                        std::snprintf(label, sizeof(label),
                                      "syncdiag phase=%u swap=%u %s",
                                      static_cast<unsigned>(phase),
                                      mode.swap ? 1U : 0U,
                                      mode.lsb ? "LSB" : "MSB");
                        log_sync_tail_summary(label, scan);
                    }
                    if (scan.frame_start > best_scan.frame_start ||
                        (scan.frame_start == best_scan.frame_start &&
                         (scan.exact > best_scan.exact ||
                          (scan.exact == best_scan.exact && scan.prefix > best_scan.prefix)))) {
                        best_scan = scan;
                        best_phase = phase;
                        best_swap = mode.swap;
                        best_lsb = mode.lsb;
                        best_first_sync = (scan.first_frame_start >= 0)
                                              ? scan.first_frame_start
                                              : ((scan.first_exact >= 0)
                                                     ? scan.first_exact : scan.first_prefix);
                    }
                }
            }
            if (best_scan.frame_start == 0U) {
                if ((++lock_attempts % 16U) == 0U) {
                    ESP_LOGW(TAG, "packed stream waiting for frame_start, attempts=%" PRIu32
                                  " exact=%" PRIu32 " prefix=%" PRIu32,
                             lock_attempts, best_scan.exact, best_scan.prefix);
                }
                vTaskDelay(1);
                continue;
            }

            size_t raw_skip = static_cast<size_t>(best_phase) +
                              static_cast<size_t>(best_first_sync) * 4U;
            if (raw_skip >= raw_len) {
                vTaskDelay(1);
                continue;
            }
            phase_locked = true;
            lock_attempts = 0;
            packer.swap = best_swap;
            packer.lsb_first = best_lsb;
            packer.reset();
            ESP_LOGI(TAG, "packed stream locked at frame_start: phase=%u swap=%u %s "
                          "first_sync_byte=%d raw_skip=%u frame_start=%" PRIu32
                          " exact=%" PRIu32 " prefix=%" PRIu32,
                     static_cast<unsigned>(best_phase), best_swap ? 1U : 0U,
                     best_lsb ? "LSB-first" : "MSB-first", best_first_sync,
                     static_cast<unsigned>(raw_skip), best_scan.frame_start,
                     best_scan.exact, best_scan.prefix);
            raw += raw_skip;
            raw_len -= raw_skip;
        }

#if APP_CAMERA_RAW_ONE_SHOT_ENABLE
        size_t remaining = APP_CAMERA_RAW_ONE_SHOT_SAMPLES - samples_sent;
        raw_len = std::min(raw_len, remaining);
#endif
        size_t packed_len = packer.push(raw, raw_len, packed, sizeof(packed));
        if (packed_len > 0U) {
            uart_write_bytes(kUart, packed, packed_len);
        }
        samples_sent += raw_len;

        if ((++windows % 128U) == 0U) {
            ESP_LOGI(TAG, "packed windows=%" PRIu32 " timeouts=%" PRIu32,
                     windows, timeouts);
        }

#if APP_CAMERA_RAW_ONE_SHOT_ENABLE
        if (samples_sent >= APP_CAMERA_RAW_ONE_SHOT_SAMPLES) {
            ESP_LOGI(TAG, "packed one-shot complete: samples=%u packed_bytes=%u",
                     static_cast<unsigned>(samples_sent),
                     static_cast<unsigned>(samples_sent / 4U));
            uart_wait_tx_done(kUart, pdMS_TO_TICKS(30000));
            cam_ll_stop(hw);
            gpio_set_level(BSP_GC032A_PWDN_GPIO, 1);
            ESP_LOGI(TAG, "LCD_CAM stopped, GC032A PWDN asserted");
            vTaskDelete(nullptr);
        }
#endif
        vTaskDelay(1);
    }
}

esp_err_t CameraUartStreamer::start_mclk()
{
    if (mclk_started_) return ESP_OK;

    ledc_timer_config_t timer = {
        .speed_mode = kMclkSpeedMode,
        .duty_resolution = kMclkDutyResolution,
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

    uint8_t idh = 0, idl = 0;
    ESP_RETURN_ON_ERROR(gc032a_read_reg(kGc032aIdHighReg, &idh), TAG, "id high");
    ESP_RETURN_ON_ERROR(gc032a_read_reg(kGc032aIdLowReg, &idl), TAG, "id low");
    if (static_cast<uint16_t>((idh << 8) | idl) != kGc032aExpectedId) {
        return ESP_ERR_NOT_FOUND;
    }

    for (const auto &rv : kGc032aInitRegs) {
        ESP_RETURN_ON_ERROR(gc032a_write_reg(rv.reg, rv.val), TAG, "init reg");
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGI(TAG, "gc032a: sensor ready, %u regs written",
             static_cast<unsigned>(sizeof(kGc032aInitRegs) /
                                   sizeof(kGc032aInitRegs[0])));
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
        0x53, 0x55, 0x5a, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67,
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
             "gc032a spi regs: 53=%02x 55=%02x 5a=%02x 60=%02x 61=%02x "
             "62=%02x 63=%02x 64=%02x 65=%02x 66=%02x 67=%02x",
             vals[0], vals[1], vals[2], vals[3], vals[4],
             vals[5], vals[6], vals[7], vals[8], vals[9], vals[10]);
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
