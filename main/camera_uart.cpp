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
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/spi_slave.h"
#include "driver/uart.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_dvp.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "hal/cam_ll.h"
#include "hal/spi_ll.h"
#include "soc/gpio_pins.h"
#include "soc/gpio_sig_map.h"
#include "soc/lcd_cam_struct.h"
#include "soc/spi_periph.h"
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
constexpr spi_host_device_t kCameraSpiHost = SPI3_HOST;

// One DMA window. LCD_CAM stores one PCLK sample per byte, with D0/D1 in
// bits [1:0].
constexpr size_t kDvpRawBytes = 32U * 1024U;
constexpr size_t kPackedBytesPerWindow =
#if APP_GC032A_SPI_1SDR_ENABLE
    kDvpRawBytes / 8U;
#else
    kDvpRawBytes / 4U;
#endif
constexpr size_t kPackedRawPhaseOffset = 0U;
constexpr size_t kSpiSlaveDmaBytes = 32U * 1024U;
constexpr TickType_t kSpiSlaveWindowTicks = pdMS_TO_TICKS(10);
constexpr size_t kSyncSize = 4U;
constexpr uint8_t kSyncFrameStart = 0x01U;
constexpr uint8_t kSyncLineStart = 0x02U;
constexpr uint8_t kSyncLineEnd = 0x40U;

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

static void force_spi_slave_cs_level(spi_host_device_t host, bool active)
{
    gpio_set_level(BSP_GC032A_SPI_CS_GPIO, active ? 0 : 1);
    esp_rom_gpio_connect_in_signal(BSP_GC032A_SPI_CS_GPIO,
                                   spi_periph_signal[host].spics_in, false);
}

static void force_camera_data_input_only()
{
    gpio_set_direction(BSP_GC032A_DATA0_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(BSP_GC032A_DATA1_GPIO, GPIO_MODE_INPUT);
}

static void IRAM_ATTR camera_spi_slave_post_setup(spi_slave_transaction_t *)
{
#if !APP_GC032A_SPI_1SDR_ENABLE
    spi_dev_t *hw = SPI_LL_GET_HW(kCameraSpiHost);
    hw->user.doutdin = 0;
    hw->user.fwrite_dual = 1;
    hw->ctrl.fread_dual = 1;
#endif
    force_spi_slave_cs_level(kCameraSpiHost, true);
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

struct Spi1BitPacker {
    bool lsb_first = true;
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
            uint8_t bit = raw[i] & 0x01U; // D0 only
            uint8_t shift = lsb_first ? phase : static_cast<uint8_t>(7U - phase);
            byte = static_cast<uint8_t>(byte | (bit << shift));
            if (++phase == 8U) {
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

static bool sync_at(const uint8_t *data, size_t len, size_t pos, uint8_t tag)
{
    return pos + kSyncSize <= len &&
           data[pos] == 0xffU &&
           data[pos + 1U] == 0xffU &&
           data[pos + 2U] == 0xffU &&
           data[pos + 3U] == tag;
}

static size_t find_sync(const uint8_t *data, size_t len, size_t start, uint8_t tag)
{
    if (len < kSyncSize || start > len - kSyncSize) {
        return SIZE_MAX;
    }
    for (size_t i = start; i <= len - kSyncSize; ++i) {
        if (sync_at(data, len, i, tag)) {
            return i;
        }
    }
    return SIZE_MAX;
}

static uint8_t clamp_u8(int v)
{
    if (v < 0) return 0;
    if (v > 255) return 255;
    return static_cast<uint8_t>(v);
}

static void yuv_to_rgb(uint8_t y, uint8_t u, uint8_t v, uint8_t *rgb)
{
    int c = static_cast<int>(y) - 16;
    int d = static_cast<int>(u) - 128;
    int e = static_cast<int>(v) - 128;
    if (c < 0) c = 0;

    rgb[0] = clamp_u8((298 * c + 409 * e + 128) >> 8);
    rgb[1] = clamp_u8((298 * c - 100 * d - 208 * e + 128) >> 8);
    rgb[2] = clamp_u8((298 * c + 516 * d + 128) >> 8);
}

static bool parse_gc032a_frame_header(const uint8_t *data,
                                      size_t len,
                                      size_t frame_start,
                                      uint16_t *width,
                                      uint16_t *height,
                                      size_t *payload_pos)
{
    size_t pos = frame_start + kSyncSize;
    if (pos + 5U > len) {
        return false;
    }

    uint16_t w = static_cast<uint16_t>((static_cast<uint16_t>(data[pos + 2U]) << 8U) |
                                       data[pos + 1U]);
    uint16_t h = static_cast<uint16_t>((static_cast<uint16_t>(data[pos + 4U]) << 8U) |
                                       data[pos + 3U]);
    if (w == 0U || h == 0U ||
        w > APP_CAMERA_IMAGE_MAX_WIDTH ||
        h > APP_CAMERA_IMAGE_MAX_HEIGHT) {
        return false;
    }

    *width = w;
    *height = h;
    *payload_pos = pos + 5U;
    return true;
}

static esp_err_t output_ppm_from_gc032a_stream(const uint8_t *data, size_t len)
{
    size_t frame_start = find_sync(data, len, 0, kSyncFrameStart);
    if (frame_start == SIZE_MAX) {
        ESP_LOGE(TAG, "PPM output: frame_start FF FF FF 01 not found");
        return ESP_ERR_NOT_FOUND;
    }

    uint16_t width = APP_CAMERA_SENSOR_WIDTH;
    uint16_t height = APP_CAMERA_SENSOR_HEIGHT;
    size_t pos = frame_start + kSyncSize;
    if (parse_gc032a_frame_header(data, len, frame_start, &width, &height, &pos)) {
        ESP_LOGI(TAG, "PPM output: frame header width=%u height=%u",
                 static_cast<unsigned>(width), static_cast<unsigned>(height));
    } else {
        width = std::min<uint16_t>(APP_CAMERA_SENSOR_WIDTH,
                                   APP_CAMERA_IMAGE_MAX_WIDTH);
        height = std::min<uint16_t>(APP_CAMERA_SENSOR_HEIGHT,
                                    APP_CAMERA_IMAGE_MAX_HEIGHT);
        ESP_LOGW(TAG, "PPM output: invalid frame header, fallback width=%u height=%u",
                 static_cast<unsigned>(width), static_cast<unsigned>(height));
    }

    const size_t rgb_len = static_cast<size_t>(width) * height * 3U;
    uint8_t *rgb = static_cast<uint8_t *>(
        heap_caps_malloc(rgb_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!rgb) {
        rgb = static_cast<uint8_t *>(
            heap_caps_malloc(rgb_len, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    if (!rgb) {
        ESP_LOGE(TAG, "PPM output: RGB frame alloc failed: %u bytes",
                 static_cast<unsigned>(rgb_len));
        return ESP_ERR_NO_MEM;
    }
    std::memset(rgb, 0, rgb_len);

    uint32_t lines_found = 0;
    uint32_t lines_used = 0;
    uint32_t malformed = 0;
    const size_t expected_line_bytes = static_cast<size_t>(width) * 2U;
    const size_t max_line_scan_extra = 64U;

    while (pos < len) {
        size_t line_start = find_sync(data, len, pos, kSyncLineStart);
        if (line_start == SIZE_MAX || line_start + kSyncSize + 2U > len) {
            break;
        }
        ++lines_found;

        uint16_t line_no = static_cast<uint16_t>(
            data[line_start + kSyncSize] |
            (static_cast<uint16_t>(data[line_start + kSyncSize + 1U]) << 8U));
        size_t payload_start = line_start + kSyncSize + 2U;
        size_t scan_end = std::min(len, payload_start + expected_line_bytes + max_line_scan_extra);
        size_t line_end = find_sync(data, scan_end, payload_start, kSyncLineEnd);
        if (line_end == SIZE_MAX) {
            ++malformed;
            pos = payload_start;
            continue;
        }

        size_t payload_len = line_end - payload_start;
        size_t usable = std::min(payload_len, expected_line_bytes);
        usable &= ~static_cast<size_t>(0x03U);

        if (line_no < height && usable >= 4U) {
            uint8_t *dst = rgb + (static_cast<size_t>(line_no) * width * 3U);
            size_t x = 0;
            for (size_t i = 0; i + 3U < usable && x + 1U < width; i += 4U, x += 2U) {
                uint8_t y0 = data[payload_start + i];
                uint8_t v = data[payload_start + i + 1U];
                uint8_t y1 = data[payload_start + i + 2U];
                uint8_t u = data[payload_start + i + 3U];
                yuv_to_rgb(y0, u, v, dst + x * 3U);
                yuv_to_rgb(y1, u, v, dst + (x + 1U) * 3U);
            }
            ++lines_used;
        } else {
            ++malformed;
        }

        pos = line_end + kSyncSize;
    }

    char header[48];
    int header_len = std::snprintf(header, sizeof(header),
                                   "P6\n%u %u\n255\n",
                                   static_cast<unsigned>(width),
                                   static_cast<unsigned>(height));
    if (header_len <= 0 || static_cast<size_t>(header_len) >= sizeof(header)) {
        heap_caps_free(rgb);
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "PPM output: frame_start_off=%u lines_found=%u lines_used=%u malformed=%u rgb_bytes=%u",
             static_cast<unsigned>(frame_start),
             static_cast<unsigned>(lines_found),
             static_cast<unsigned>(lines_used),
             static_cast<unsigned>(malformed),
             static_cast<unsigned>(rgb_len));

    uart_write_bytes(kUart, header, header_len);
    for (size_t off = 0; off < rgb_len;) {
        size_t chunk = std::min(static_cast<size_t>(APP_CAMERA_UART_CHUNK_BYTES),
                                rgb_len - off);
        uart_write_bytes(kUart, rgb + off, chunk);
        off += chunk;
        vTaskDelay(1);
    }
    uart_wait_tx_done(kUart, pdMS_TO_TICKS(30000));
    heap_caps_free(rgb);
    return ESP_OK;
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

#if APP_CAMERA_CAPTURE_USE_SPI_SLAVE
    spi_slave_stream_loop();
#else
    dvp_stream_loop();
#endif
}

void CameraUartStreamer::spi_slave_stream_loop()
{
    uint8_t spi52 = 0xff;
    if (gc032a_write_reg(0xfe, 0x03) == ESP_OK &&
        gc032a_read_reg(0x52, &spi52) == ESP_OK) {
    ESP_LOGI(TAG, "gc032a spi reg 52=%02x, expected order=%s",
                 spi52, (spi52 & 0x80U) ? "MSB-first" : "LSB-first");
    } else {
        ESP_LOGW(TAG, "gc032a spi reg 52 read failed");
    }
    gc032a_write_reg(0xfe, 0x00);

    gpio_config_t cs_cfg = {
        .pin_bit_mask = 1ULL << BSP_GC032A_SPI_CS_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cs_cfg));
    gpio_set_level(BSP_GC032A_SPI_CS_GPIO, 1);

    const size_t capture_cap = APP_CAMERA_PACKED_CAPTURE_BYTES;
    uint8_t *capture = static_cast<uint8_t *>(
        heap_caps_malloc(capture_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!capture) {
        ESP_LOGW(TAG, "spi slave PSRAM capture alloc failed, trying internal");
        capture = static_cast<uint8_t *>(
            heap_caps_malloc(capture_cap, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    if (!capture) {
        ESP_LOGE(TAG, "spi slave capture buffer alloc failed: %u bytes",
                 static_cast<unsigned>(capture_cap));
        gpio_set_level(BSP_GC032A_PWDN_GPIO, 1);
        vTaskDelete(nullptr);
    }
    std::memset(capture, 0, capture_cap);

    uint8_t *dma_buf = static_cast<uint8_t *>(
        heap_caps_aligned_alloc(64, kSpiSlaveDmaBytes,
                                MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
    if (!dma_buf) {
        ESP_LOGE(TAG, "spi slave DMA buffer alloc failed: %u bytes",
                 static_cast<unsigned>(kSpiSlaveDmaBytes));
        heap_caps_free(capture);
        gpio_set_level(BSP_GC032A_PWDN_GPIO, 1);
        vTaskDelete(nullptr);
    }

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = BSP_GC032A_DATA0_GPIO;
#if APP_GC032A_SPI_1SDR_ENABLE
    bus_cfg.miso_io_num = -1;
#else
    bus_cfg.miso_io_num = BSP_GC032A_DATA1_GPIO;
#endif
    bus_cfg.sclk_io_num = BSP_GC032A_SPI_CLK_GPIO;
    bus_cfg.data2_io_num = -1;
    bus_cfg.data3_io_num = -1;
    bus_cfg.data4_io_num = -1;
    bus_cfg.data5_io_num = -1;
    bus_cfg.data6_io_num = -1;
    bus_cfg.data7_io_num = -1;
    bus_cfg.max_transfer_sz = static_cast<int>(kSpiSlaveDmaBytes);
    bus_cfg.flags = SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MOSI |
#if !APP_GC032A_SPI_1SDR_ENABLE
                    SPICOMMON_BUSFLAG_DUAL |
#endif
                    SPICOMMON_BUSFLAG_GPIO_PINS;

    spi_slave_interface_config_t slave_cfg = {};
    slave_cfg.spics_io_num = BSP_GC032A_SPI_CS_GPIO;
    slave_cfg.flags = SPI_SLAVE_RXBIT_LSBFIRST;
    slave_cfg.queue_size = 1;
    slave_cfg.mode = 0;
    slave_cfg.post_setup_cb = camera_spi_slave_post_setup;

    esp_err_t err = spi_slave_initialize(kCameraSpiHost, &bus_cfg, &slave_cfg,
                                         SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi slave init failed: %s", esp_err_to_name(err));
        heap_caps_free(dma_buf);
        heap_caps_free(capture);
        gpio_set_level(BSP_GC032A_PWDN_GPIO, 1);
        vTaskDelete(nullptr);
    }
    ESP_ERROR_CHECK(gpio_config(&cs_cfg));
    gpio_set_level(BSP_GC032A_SPI_CS_GPIO, 1);
    force_camera_data_input_only();
    ESP_LOGI(TAG, "spi slave data pins forced input-only after bus init");

    force_spi_slave_cs_level(kCameraSpiHost, false);

    ESP_LOGI(TAG, "spi slave %s capture: host=SPI3 PCLK=GPIO%d MOSI/IO0=GPIO%d IO1=GPIO%d CS=GPIO%d loopback %u ms bytes=%u dma_window=%u",
#if APP_GC032A_SPI_1SDR_ENABLE
             "1-line SDR",
#else
             "DIO",
#endif
             BSP_GC032A_SPI_CLK_GPIO, BSP_GC032A_DATA0_GPIO,
             BSP_GC032A_DATA1_GPIO, BSP_GC032A_SPI_CS_GPIO,
             static_cast<unsigned>(APP_CAMERA_SPI_SLAVE_CAPTURE_MS),
             static_cast<unsigned>(capture_cap),
             static_cast<unsigned>(kSpiSlaveDmaBytes));

    size_t got = 0;
    uint32_t windows = 0;
    TickType_t deadline = xTaskGetTickCount() +
                          pdMS_TO_TICKS(APP_CAMERA_SPI_SLAVE_CAPTURE_MS);
    while (got < capture_cap && xTaskGetTickCount() < deadline) {
        spi_slave_transaction_t trans = {};
        trans.length = kSpiSlaveDmaBytes * 8U;
        trans.rx_buffer = dma_buf;
        std::memset(dma_buf, 0, kSpiSlaveDmaBytes);

        err = spi_slave_queue_trans(kCameraSpiHost, &trans, pdMS_TO_TICKS(1000));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi slave queue failed: %s", esp_err_to_name(err));
            break;
        }

        force_spi_slave_cs_level(kCameraSpiHost, true);
        vTaskDelay(kSpiSlaveWindowTicks);
        force_spi_slave_cs_level(kCameraSpiHost, false);

        spi_slave_transaction_t *ret = nullptr;
        err = spi_slave_get_trans_result(kCameraSpiHost, &ret, pdMS_TO_TICKS(1000));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi slave result failed: %s", esp_err_to_name(err));
            break;
        }

        size_t rx_len = 0;
        if (ret) {
            rx_len = std::min(kSpiSlaveDmaBytes, (ret->trans_len + 7U) / 8U);
        }
        size_t copy_len = std::min(rx_len, capture_cap - got);
        size_t nonzero = 0;
        for (size_t i = 0; i < copy_len; ++i) {
            nonzero += dma_buf[i] != 0;
        }
        std::memcpy(capture + got, dma_buf, copy_len);
        got += copy_len;
        ++windows;
        if (windows <= 4U || (windows % 32U) == 0U) {
            ESP_LOGI(TAG, "spi slave window=%u trans_bits=%u rx_bytes=%u nonzero=%u total=%u",
                     static_cast<unsigned>(windows),
                     ret ? static_cast<unsigned>(ret->trans_len) : 0U,
                     static_cast<unsigned>(copy_len),
                     static_cast<unsigned>(nonzero),
                     static_cast<unsigned>(got));
        }
    }

    gpio_set_level(BSP_GC032A_PWDN_GPIO, 1);
    ESP_LOGI(TAG, "spi slave capture complete: windows=%u dump_bytes=%u",
             static_cast<unsigned>(windows),
             static_cast<unsigned>(got));

#if APP_CAMERA_OUTPUT_PPM_ENABLE
    if (got > 0U && output_ppm_from_gc032a_stream(capture, got) == ESP_OK) {
        ESP_LOGI(TAG, "SPI slave PPM output complete");
    } else
#endif
    {
        for (size_t off = 0; off < got;) {
            size_t chunk = std::min(static_cast<size_t>(APP_CAMERA_UART_CHUNK_BYTES),
                                    got - off);
            uart_write_bytes(kUart, capture + off, chunk);
            off += chunk;
            vTaskDelay(1);
        }
        uart_wait_tx_done(kUart, pdMS_TO_TICKS(30000));
    }

    spi_slave_free(kCameraSpiHost);
    heap_caps_free(dma_buf);
    heap_caps_free(capture);
    ESP_LOGI(TAG, "SPI slave UART dump complete");
    vTaskDelete(nullptr);
}

void CameraUartStreamer::dvp_stream_loop()
{
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
        .clk_src = CAM_CLK_SRC_XTAL,
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
    uint32_t buf_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
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

    uint8_t *capture = nullptr;
    size_t capture_cap = 0;
    size_t capture_len = 0;
#if APP_CAMERA_RAW_ONE_SHOT_ENABLE
    capture_cap = APP_CAMERA_PACKED_CAPTURE_BYTES;
    capture = static_cast<uint8_t *>(
        heap_caps_malloc(capture_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!capture) {
        capture = static_cast<uint8_t *>(
            heap_caps_malloc(capture_cap, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    if (!capture) {
        ESP_LOGE(TAG, "packed capture buffer alloc failed: %u bytes",
                 static_cast<unsigned>(capture_cap));
        cam_ll_stop(hw);
        gpio_set_level(BSP_GC032A_PWDN_GPIO, 1);
        vTaskDelete(nullptr);
    }
    ESP_LOGI(TAG, "packed one-shot capture buffer=%u bytes",
             static_cast<unsigned>(capture_cap));
#endif

    uint8_t packed[kPackedBytesPerWindow];
#if APP_GC032A_SPI_1SDR_ENABLE
    Spi1BitPacker packer;
    packer.lsb_first = true;
#else
    Spi2BitPacker packer;
    packer.swap = false;
    packer.lsb_first = true;
#endif
    packer.reset();
    uint32_t windows = 0;
    uint32_t timeouts = 0;
    size_t samples_sent = 0;
#if APP_GC032A_SPI_1SDR_ENABLE
    ESP_LOGI(TAG, "1-bit map fixed: IO0/D0 -> bits 0..7, LSB-first");
#else
    ESP_LOGI(TAG, "2-bit map fixed: IO0/D0 -> bits 0/2/4/6, IO1/D1 -> bits 1/3/5/7, LSB-first");
#endif

    while (true) {
        DvpProbeDone done;
        if (xQueueReceive(ctx.done_queue, &done, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "dvp stream timeout, restarting LCD_CAM");
            restart_cam_hw(hw, pclk_invert);
            packer.reset();
            ++timeouts;
            continue;
        }

        size_t raw_len = std::min(done.received, ctx.buffer_len);
        const uint8_t *raw = done.buffer;
        if (raw_len <= kPackedRawPhaseOffset) {
            continue;
        }
        raw += kPackedRawPhaseOffset;
        raw_len -= kPackedRawPhaseOffset;

        size_t packed_len = packer.push(raw, raw_len, packed, sizeof(packed));
#if APP_CAMERA_RAW_ONE_SHOT_ENABLE
        if (packed_len > 0U) {
            size_t room = capture_cap - capture_len;
            size_t copy_len = std::min(packed_len, room);
            if (copy_len > 0U) {
                std::memcpy(capture + capture_len, packed, copy_len);
                capture_len += copy_len;
            }
            if (copy_len < packed_len) {
                ESP_LOGW(TAG, "packed capture buffer full, dropping %u bytes",
                         static_cast<unsigned>(packed_len - copy_len));
            }
        }
#else
        if (packed_len > 0U) {
            uart_write_bytes(kUart, packed, packed_len);
        }
#endif
        samples_sent += raw_len;

        if ((++windows % 128U) == 0U) {
            ESP_LOGI(TAG, "packed windows=%" PRIu32 " timeouts=%" PRIu32,
                     windows, timeouts);
        }

#if APP_CAMERA_RAW_ONE_SHOT_ENABLE
        if (capture_len >= capture_cap) {
            cam_ll_stop(hw);
            gpio_set_level(BSP_GC032A_PWDN_GPIO, 1);
            ESP_LOGI(TAG, "packed capture complete: samples=%u packed_bytes=%u",
                     static_cast<unsigned>(samples_sent),
                     static_cast<unsigned>(capture_len));
#if APP_CAMERA_OUTPUT_PPM_ENABLE
            esp_err_t ppm_err = output_ppm_from_gc032a_stream(capture, capture_len);
            if (ppm_err != ESP_OK) {
                ESP_LOGE(TAG, "PPM output failed: %s, dumping raw packed stream",
                         esp_err_to_name(ppm_err));
#endif
            for (size_t off = 0; off < capture_len;) {
                size_t chunk = std::min(static_cast<size_t>(APP_CAMERA_UART_CHUNK_BYTES),
                                        capture_len - off);
                uart_write_bytes(kUart, capture + off, chunk);
                off += chunk;
                vTaskDelay(1);
            }
            uart_wait_tx_done(kUart, pdMS_TO_TICKS(30000));
#if APP_CAMERA_OUTPUT_PPM_ENABLE
            } else {
                ESP_LOGI(TAG, "PPM UART output complete");
            }
#endif
            heap_caps_free(capture);
            ESP_LOGI(TAG, "LCD_CAM stopped, GC032A PWDN asserted, UART output complete");
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

#if APP_GC032A_TIMING_PATCH_ENABLE
    ESP_RETURN_ON_ERROR(gc032a_write_reg(0xfe, 0x00), TAG, "timing page");
    ESP_RETURN_ON_ERROR(gc032a_write_reg(0x05, APP_GC032A_PATCH_HB_HI), TAG, "timing hb hi");
    ESP_RETURN_ON_ERROR(gc032a_write_reg(0x06, APP_GC032A_PATCH_HB_LO), TAG, "timing hb lo");
    ESP_RETURN_ON_ERROR(gc032a_write_reg(0x07, APP_GC032A_PATCH_VB_HI), TAG, "timing vb hi");
    ESP_RETURN_ON_ERROR(gc032a_write_reg(0x08, APP_GC032A_PATCH_VB_LO), TAG, "timing vb lo");
    ESP_LOGI(TAG, "gc032a timing patch: HB=%02x%02x VB=%02x%02x",
             APP_GC032A_PATCH_HB_HI, APP_GC032A_PATCH_HB_LO,
             APP_GC032A_PATCH_VB_HI, APP_GC032A_PATCH_VB_LO);
#endif

#if APP_GC032A_SPI_1SDR_ENABLE
    ESP_RETURN_ON_ERROR(gc032a_write_reg(0xfe, 0x03), TAG, "1sdr page");
    ESP_RETURN_ON_ERROR(gc032a_write_reg(0x52, 0x58), TAG, "1sdr mode");
    ESP_RETURN_ON_ERROR(gc032a_write_reg(0x53, 0x21), TAG, "1sdr line num");
    ESP_RETURN_ON_ERROR(gc032a_write_reg(0x5a, 0x01), TAG, "1sdr output");
    ESP_RETURN_ON_ERROR(gc032a_write_reg(0x64, 0x0c), TAG, "1sdr sck");
    ESP_RETURN_ON_ERROR(gc032a_write_reg(0xfe, 0x00), TAG, "1sdr page0");
    ESP_LOGI(TAG, "gc032a 1-line SDR patch applied: MCLK=%uHz p03[52]=58 p03[53]=21 p03[5a]=01 p03[64]=0c",
             static_cast<unsigned>(APP_GC032A_MCLK_HZ));
#endif

#if APP_GC032A_PCLK_DELAY_ENABLE
    ESP_RETURN_ON_ERROR(gc032a_write_reg(0xfe, 0x00), TAG, "pclk delay page0");
    ESP_RETURN_ON_ERROR(gc032a_write_reg(APP_GC032A_PCLK_DELAY_REG,
                                         APP_GC032A_PCLK_DELAY_VAL),
                        TAG, "pclk delay");
    ESP_LOGI(TAG, "gc032a pclk delay patch: p00 r%02x=%02x",
             APP_GC032A_PCLK_DELAY_REG, APP_GC032A_PCLK_DELAY_VAL);
#endif

#if APP_GC032A_PCLK_POLARITY_ENABLE
    ESP_RETURN_ON_ERROR(gc032a_write_reg(0xfe, 0x00), TAG, "pclk polarity page0");
    ESP_RETURN_ON_ERROR(gc032a_write_reg(0x46, APP_GC032A_PCLK_POLARITY_VAL),
                        TAG, "pclk polarity");
    ESP_LOGI(TAG, "gc032a pclk polarity patch: p00 r46=%02x, lcd_cam_pclk_invert=%u",
             APP_GC032A_PCLK_POLARITY_VAL,
             static_cast<unsigned>(APP_CAMERA_DVP_PCLK_INVERT));
#endif

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
