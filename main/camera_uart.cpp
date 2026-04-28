// ============================================================
// camera_uart.cpp - LCD_CAM raw sampler for GC032A PCLK + D0/D1
//
// UART2 output is ASCII hex only: each LCD_CAM sample byte is emitted as
// two lowercase hex chars. Only bits [1:0] carry D0/D1.
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
// bits [1:0]. UART hex output is 2x this size.
constexpr size_t kDvpRawBytes = 8U * 1024U;
constexpr size_t kHexChunkSamples = 256U;

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

    ESP_LOGI(TAG, "raw hex stream: UART%d %u baud, each sample byte masked to 0x03",
             static_cast<int>(kUart), static_cast<unsigned>(APP_CAMERA_UART_BAUD));

    static constexpr char hex[] = "0123456789abcdef";
    char line[kHexChunkSamples * 2U + 1U];
    uint32_t windows = 0;
    uint32_t timeouts = 0;
    size_t samples_sent = 0;

    while (true) {
        DvpProbeDone done;
        if (xQueueReceive(ctx.done_queue, &done, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "dvp stream timeout, restarting LCD_CAM");
            restart_cam_hw(hw, pclk_invert);
            ++timeouts;
            continue;
        }

        size_t raw_len = std::min(done.received, ctx.buffer_len);
#if APP_CAMERA_RAW_ONE_SHOT_ENABLE
        size_t remaining = APP_CAMERA_RAW_ONE_SHOT_SAMPLES - samples_sent;
        raw_len = std::min(raw_len, remaining);
#endif
        const uint8_t *raw = done.buffer;
        size_t off = 0;
        while (off < raw_len) {
            size_t n = std::min(kHexChunkSamples, raw_len - off);
            for (size_t i = 0; i < n; ++i) {
                uint8_t v = raw[off + i] & 0x03U;
                line[i * 2U] = hex[v >> 4U];
                line[i * 2U + 1U] = hex[v & 0x0FU];
            }
            line[n * 2U] = '\n';
            uart_write_bytes(kUart, line, n * 2U + 1U);
            off += n;
            samples_sent += n;
        }

        if ((++windows % 128U) == 0U) {
            ESP_LOGI(TAG, "raw hex windows=%" PRIu32 " timeouts=%" PRIu32,
                     windows, timeouts);
        }

#if APP_CAMERA_RAW_ONE_SHOT_ENABLE
        if (samples_sent >= APP_CAMERA_RAW_ONE_SHOT_SAMPLES) {
            ESP_LOGI(TAG, "raw one-shot complete: samples=%u hex_bytes=%u",
                     static_cast<unsigned>(samples_sent),
                     static_cast<unsigned>(samples_sent * 2U));
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
    dump_gc032a_spi_regs();
    return ESP_OK;
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
