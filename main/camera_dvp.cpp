#include "camera_uart.hpp"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "app_config.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_dvp.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "bsp.h"

#include "sp0a39_regs.h"

namespace {

constexpr const char *TAG = "camera_dvp";
constexpr TickType_t kPwdnSettleTicks = pdMS_TO_TICKS(100);
constexpr TickType_t kResetSettleTicks = pdMS_TO_TICKS(120);
constexpr uint32_t kFourccGrey = 0x59455247; // 'GREY'
constexpr uint32_t kFourccUyvy = 0x59565955; // 'UYVY'
constexpr uint32_t kFourccYuyv = 0x56595559; // 'YUYV'
constexpr uint32_t kFourccVyuy = 0x59555956; // 'VYUY'
constexpr uint32_t kFrameBytes = APP_CAMERA_FRAME_BYTES;
constexpr size_t kCaptureDmaBufferCount = 4;

#if APP_CAMERA_COLOR_ENABLE
constexpr uint32_t kDvpCaptureWidth = APP_CAMERA_SENSOR_WIDTH;
constexpr cam_ctlr_color_t kDvpInputColor = CAM_CTLR_COLOR_YUV422;
constexpr uint32_t kOutputPixelformat = kFourccVyuy;
#else
constexpr uint32_t kDvpCaptureWidth = APP_CAMERA_SENSOR_WIDTH;
constexpr cam_ctlr_color_t kDvpInputColor = CAM_CTLR_COLOR_GRAY8;
constexpr uint32_t kOutputPixelformat = kFourccGrey;
#endif

struct dvp_cb_ctx {
    uint8_t *buffers[kCaptureDmaBufferCount];
    size_t buflen;
    size_t received;
    int frame_count;
    size_t next_buffer;
    size_t last_received;
    uint8_t *captured_buffer;
    SemaphoreHandle_t done_sem;
};

static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle,
                                       esp_cam_ctlr_trans_t *trans, void *user_data)
{
    dvp_cb_ctx *ctx = static_cast<dvp_cb_ctx *>(user_data);
    if (ctx->captured_buffer && ctx->buffers[ctx->next_buffer] == ctx->captured_buffer) {
        ctx->next_buffer = (ctx->next_buffer + 1) % kCaptureDmaBufferCount;
    }
    trans->buffer = ctx->buffers[ctx->next_buffer];
    trans->buflen = ctx->buflen;
    ctx->next_buffer = (ctx->next_buffer + 1) % kCaptureDmaBufferCount;
    return trans->buffer != nullptr;
}

static bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t handle,
                                        esp_cam_ctlr_trans_t *trans, void *user_data)
{
    dvp_cb_ctx *ctx = static_cast<dvp_cb_ctx *>(user_data);
    ctx->frame_count++;
    ctx->last_received = trans->received_size;
    if (ctx->frame_count >= 2) {
        ctx->received = trans->received_size;
        ctx->captured_buffer = static_cast<uint8_t *>(trans->buffer);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(ctx->done_sem, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }
    return false;
}

i2c_master_dev_handle_t s_sensor_dev = nullptr;

esp_err_t sensor_i2c_attach()
{
    if (s_sensor_dev) return ESP_OK;
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) return ESP_ERR_INVALID_STATE;

    i2c_device_config_t cfg = {};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address = APP_SP0A39_I2C_ADDR;
    cfg.scl_speed_hz = BSP_I2C0_FREQ_HZ;
    return i2c_master_bus_add_device(bus, &cfg, &s_sensor_dev);
}

void sensor_i2c_detach()
{
    if (s_sensor_dev) {
        i2c_master_bus_rm_device(s_sensor_dev);
        s_sensor_dev = nullptr;
    }
}

esp_err_t sensor_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_sensor_dev, buf, 2, 50);
}

esp_err_t sensor_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_sensor_dev, &reg, 1, val, 1, 50);
}

esp_err_t sensor_write_regs()
{
    const size_t count = sizeof(s_sp0a39_regs) / sizeof(s_sp0a39_regs[0]);
    for (size_t i = 0; i < count; i++) {
        esp_err_t ret = sensor_write_reg(s_sp0a39_regs[i][0], s_sp0a39_regs[i][1]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "reg write failed at [%u] 0x%02x=0x%02x: %s",
                     (unsigned)i, s_sp0a39_regs[i][0], s_sp0a39_regs[i][1],
                     esp_err_to_name(ret));
            return ret;
        }
    }
    ESP_LOGI(TAG, "SP0A39 register init done (%u regs)", (unsigned)count);
    return ESP_OK;
}

esp_err_t sensor_read_id()
{
    ESP_RETURN_ON_ERROR(sensor_write_reg(0xfd, 0x00), TAG, "page select");
    uint8_t id_h = 0, id_l = 0;
    ESP_RETURN_ON_ERROR(sensor_read_reg(0x00, &id_h), TAG, "read id_h");
    ESP_RETURN_ON_ERROR(sensor_read_reg(0x01, &id_l), TAG, "read id_l");
    ESP_LOGI(TAG, "SP0A39 chip ID: 0x%02X%02X", id_h, id_l);
    return ESP_OK;
}

void log_sensor_output_regs()
{
    uint8_t p0_1c = 0;
    uint8_t p0_30 = 0;
    uint8_t p0_31 = 0;
    uint8_t p1_32 = 0;
    uint8_t p1_34 = 0;
    uint8_t p1_35 = 0;
    uint8_t p1_36 = 0;
    if (sensor_write_reg(0xfd, 0x00) == ESP_OK) {
        sensor_read_reg(0x1c, &p0_1c);
        sensor_read_reg(0x30, &p0_30);
        sensor_read_reg(0x31, &p0_31);
    }
    if (sensor_write_reg(0xfd, 0x01) == ESP_OK) {
        sensor_read_reg(0x32, &p1_32);
        sensor_read_reg(0x34, &p1_34);
        sensor_read_reg(0x35, &p1_35);
        sensor_read_reg(0x36, &p1_36);
    }
    sensor_write_reg(0xfd, 0x00);
    ESP_LOGI(TAG, "SP0A39 output regs: P0:1c=0x%02x P0:30=0x%02x P0:31=0x%02x P1:32=0x%02x P1:34=0x%02x P1:35=0x%02x P1:36=0x%02x",
             p0_1c, p0_30, p0_31, p1_32, p1_34, p1_35, p1_36);
}

void log_gpio_diagnostics()
{
    int vsync_changes = 0;
    int hsync_changes = 0;
    int pclk_changes = 0;
    int last_vsync = gpio_get_level(BSP_SP0A39_VSYNC_GPIO);
    int last_hsync = gpio_get_level(BSP_SP0A39_HSYNC_GPIO);
    int last_pclk = gpio_get_level(BSP_SP0A39_PCLK_GPIO);
    for (int i = 0; i < 100000; i++) {
        int vsync = gpio_get_level(BSP_SP0A39_VSYNC_GPIO);
        int hsync = gpio_get_level(BSP_SP0A39_HSYNC_GPIO);
        int pclk = gpio_get_level(BSP_SP0A39_PCLK_GPIO);
        if (vsync != last_vsync) {
            vsync_changes++;
            last_vsync = vsync;
        }
        if (hsync != last_hsync) {
            hsync_changes++;
            last_hsync = hsync;
        }
        if (pclk != last_pclk) {
            pclk_changes++;
            last_pclk = pclk;
        }
    }
    ESP_LOGI(TAG, "GPIO diagnostics: VSYNC changes=%d HSYNC changes=%d PCLK changes=%d HSYNC level=%d",
             vsync_changes, hsync_changes, pclk_changes, gpio_get_level(BSP_SP0A39_HSYNC_GPIO));
}

} // namespace

esp_err_t CameraUartStreamer::init()
{
    if (initialized_) return ESP_OK;
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c init");

    // Start MCLK output immediately - SP0A39 needs clock to respond to I2C
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_1;
    ledc_timer.duty_resolution = LEDC_TIMER_1_BIT;
    ledc_timer.freq_hz = APP_SP0A39_MCLK_HZ;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_RETURN_ON_ERROR(ledc_timer_config(&ledc_timer), TAG, "ledc timer");

    ledc_channel_config_t ledc_ch = {};
    ledc_ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_ch.channel = LEDC_CHANNEL_1;
    ledc_ch.timer_sel = LEDC_TIMER_1;
    ledc_ch.intr_type = LEDC_INTR_DISABLE;
    ledc_ch.gpio_num = BSP_SP0A39_MCLK_GPIO;
    ledc_ch.duty = 1;
    ledc_ch.hpoint = 0;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ledc_ch), TAG, "ledc channel");
    ESP_LOGI(TAG, "MCLK running on GPIO%d at %lu Hz", BSP_SP0A39_MCLK_GPIO, APP_SP0A39_MCLK_HZ);

    // Pre-init sensor at boot so first capture can use the fast path
    set_pwdn(false);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_err_t ret = reset_sensor();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "sensor pre-init reset failed: %s", esp_err_to_name(ret));
        set_pwdn(true);
        initialized_ = true;
        return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    ret = sensor_i2c_attach();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "sensor pre-init i2c failed: %s", esp_err_to_name(ret));
        set_pwdn(true);
        initialized_ = true;
        return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    ret = sensor_read_id();
    if (ret == ESP_OK) {
        ret = sensor_write_regs();
    }
    if (ret == ESP_OK) {
        sensor_configured_ = true;
        ESP_LOGI(TAG, "sensor pre-initialized at boot");
    } else {
        ESP_LOGW(TAG, "sensor pre-init regs failed: %s", esp_err_to_name(ret));
    }

    set_pwdn(true);
    initialized_ = true;
    return ESP_OK;
}

esp_err_t CameraUartStreamer::start()
{
    ESP_RETURN_ON_ERROR(init(), TAG, "init");
    return ESP_OK;
}

esp_err_t CameraUartStreamer::set_pwdn(bool asserted)
{
    return bsp_ioexp_set_pin(BSP_SP0A39_PWDN_IOEXP_PIN, asserted);
}

esp_err_t CameraUartStreamer::reset_sensor()
{
    gpio_config_t reset = {};
    reset.pin_bit_mask = 1ULL << BSP_SP0A39_RESET_GPIO;
    reset.mode = GPIO_MODE_OUTPUT;
    reset.pull_up_en = GPIO_PULLUP_DISABLE;
    reset.pull_down_en = GPIO_PULLDOWN_DISABLE;
    reset.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&reset), TAG, "camera reset gpio");
    gpio_set_level(BSP_SP0A39_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(BSP_SP0A39_RESET_GPIO, 1);
    vTaskDelay(kResetSettleTicks);
    return ESP_OK;
}

esp_err_t CameraUartStreamer::configure_camera_pins()
{
    uint64_t mask = 0;
    mask |= 1ULL << BSP_SP0A39_PCLK_GPIO;
    mask |= 1ULL << BSP_SP0A39_VSYNC_GPIO;
    mask |= 1ULL << BSP_SP0A39_HSYNC_GPIO;
    mask |= 1ULL << BSP_SP0A39_D0_GPIO;
    mask |= 1ULL << BSP_SP0A39_D1_GPIO;
    mask |= 1ULL << BSP_SP0A39_D2_GPIO;
    mask |= 1ULL << BSP_SP0A39_D3_GPIO;
    mask |= 1ULL << BSP_SP0A39_D4_GPIO;
    mask |= 1ULL << BSP_SP0A39_D5_GPIO;
    mask |= 1ULL << BSP_SP0A39_D6_GPIO;
    mask |= 1ULL << BSP_SP0A39_D7_GPIO;
    gpio_config_t input_conf = {};
    input_conf.pin_bit_mask = mask;
    input_conf.mode = GPIO_MODE_INPUT;
    input_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    input_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    input_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&input_conf), TAG, "gpio input config");
    ESP_LOGI(TAG, "camera DVP pins configured as inputs");
    return ESP_OK;
}

esp_err_t CameraUartStreamer::power_down()
{
    set_pwdn(true);
    gpio_reset_pin(BSP_SP0A39_RESET_GPIO);
    sensor_i2c_detach();
    sensor_configured_ = false;
    sensor_awake_ = false;
    return ESP_OK;
}

esp_err_t CameraUartStreamer::soft_power_down()
{
    set_pwdn(true);
    return ESP_OK;
}

esp_err_t CameraUartStreamer::capture_frame(uint8_t **out_data,
                                            size_t *out_len,
                                            uint32_t *out_width,
                                            uint32_t *out_height,
                                            uint32_t *out_pixelformat)
{
    ESP_RETURN_ON_FALSE(out_data && out_len && out_width && out_height && out_pixelformat,
                        ESP_ERR_INVALID_ARG, TAG, "invalid args");
    *out_data = nullptr;
    *out_len = 0;
    *out_width = 0;
    *out_height = 0;
    *out_pixelformat = 0;

    ESP_RETURN_ON_ERROR(init(), TAG, "init");
    ESP_RETURN_ON_ERROR(configure_camera_pins(), TAG, "camera pins");

    if (!sensor_configured_) {
        // First capture: full cold init with MCLK already running from init()
        ESP_LOGI(TAG, "releasing SP0A39 PWDN: P%d low", BSP_SP0A39_PWDN_IOEXP_PIN);
        set_pwdn(false);
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_err_t init_ret = reset_sensor();
        if (init_ret != ESP_OK) { power_down(); return init_ret; }
        vTaskDelay(pdMS_TO_TICKS(100));

        init_ret = sensor_i2c_attach();
        if (init_ret != ESP_OK) { power_down(); return init_ret; }
        vTaskDelay(pdMS_TO_TICKS(500));

        init_ret = sensor_read_id();
        if (init_ret != ESP_OK) { power_down(); return init_ret; }
        init_ret = sensor_write_regs();
        if (init_ret != ESP_OK) { power_down(); return init_ret; }
        log_sensor_output_regs();
        sensor_configured_ = true;
        sensor_awake_ = true;
    } else if (!sensor_awake_) {
        // Sensor was powered down after a failure, do fast reinit
        gpio_reset_pin(BSP_SP0A39_MCLK_GPIO);
        ledc_timer_resume(LEDC_LOW_SPEED_MODE, LEDC_TIMER_1);
        ledc_channel_config_t ledc_ch = {};
        ledc_ch.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_ch.channel = LEDC_CHANNEL_1;
        ledc_ch.timer_sel = LEDC_TIMER_1;
        ledc_ch.intr_type = LEDC_INTR_DISABLE;
        ledc_ch.gpio_num = BSP_SP0A39_MCLK_GPIO;
        ledc_ch.duty = 1;
        ledc_ch.hpoint = 0;
        ledc_channel_config(&ledc_ch);

        ESP_LOGI(TAG, "releasing SP0A39 PWDN: P%d low (fast)", BSP_SP0A39_PWDN_IOEXP_PIN);
        set_pwdn(false);
        vTaskDelay(pdMS_TO_TICKS(10));

        sensor_i2c_attach();
        esp_err_t init_ret = sensor_write_regs();
        if (init_ret != ESP_OK) { power_down(); return init_ret; }
        vTaskDelay(pdMS_TO_TICKS(100));
        sensor_awake_ = true;
    } else {
        // Sensor still powered and configured — nothing to do
        ESP_LOGI(TAG, "sensor already awake, skipping reinit");
    }

    // Release LEDC from GPIO3, then immediately create DVP (restores XCLK)
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_timer_pause(LEDC_LOW_SPEED_MODE, LEDC_TIMER_1);
    gpio_reset_pin(BSP_SP0A39_MCLK_GPIO);

    esp_cam_ctlr_dvp_pin_config_t pins = {};
    pins.data_width = CAM_CTLR_DATA_WIDTH_8;
    pins.data_io[0] = BSP_SP0A39_D0_GPIO;
    pins.data_io[1] = BSP_SP0A39_D1_GPIO;
    pins.data_io[2] = BSP_SP0A39_D2_GPIO;
    pins.data_io[3] = BSP_SP0A39_D3_GPIO;
    pins.data_io[4] = BSP_SP0A39_D4_GPIO;
    pins.data_io[5] = BSP_SP0A39_D5_GPIO;
    pins.data_io[6] = BSP_SP0A39_D6_GPIO;
    pins.data_io[7] = BSP_SP0A39_D7_GPIO;
    pins.vsync_io = BSP_SP0A39_VSYNC_GPIO;
    pins.de_io = BSP_SP0A39_HSYNC_GPIO;
    pins.pclk_io = BSP_SP0A39_PCLK_GPIO;
    pins.xclk_io = BSP_SP0A39_MCLK_GPIO;

    esp_cam_ctlr_dvp_config_t dvp_cfg = {};
    dvp_cfg.ctlr_id = 0;
    dvp_cfg.clk_src = CAM_CLK_SRC_DEFAULT;
    dvp_cfg.h_res = kDvpCaptureWidth;
    dvp_cfg.v_res = APP_CAMERA_SENSOR_HEIGHT;
    dvp_cfg.input_data_color_type = kDvpInputColor;
    dvp_cfg.pin = &pins;
    dvp_cfg.xclk_freq = APP_SP0A39_MCLK_HZ;
    dvp_cfg.dma_burst_size = 64;
    dvp_cfg.bk_buffer_dis = 1;
    ESP_LOGI(TAG, "DVP capture format: %s, capture=%lux%lu, logical=%ux%u, buffer=%lu bytes",
             APP_CAMERA_COLOR_ENABLE ? "YUV422 VYUY" : "GRAY8",
             (unsigned long)kDvpCaptureWidth,
             (unsigned long)APP_CAMERA_SENSOR_HEIGHT,
             APP_CAMERA_SENSOR_WIDTH,
             APP_CAMERA_SENSOR_HEIGHT,
             (unsigned long)kFrameBytes);

    esp_cam_ctlr_handle_t cam_handle = nullptr;
    esp_err_t ret = esp_cam_new_dvp_ctlr(&dvp_cfg, &cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_cam_new_dvp_ctlr failed: %s", esp_err_to_name(ret));
        power_down();
        return ret;
    }

    // DVP now outputs XCLK on GPIO3 - wait for sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t *frame_bufs[kCaptureDmaBufferCount] = {};
    for (size_t i = 0; i < kCaptureDmaBufferCount; ++i) {
        frame_bufs[i] = static_cast<uint8_t *>(
            esp_cam_ctlr_alloc_buffer(cam_handle, kFrameBytes,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA));
        if (!frame_bufs[i]) {
            ESP_LOGE(TAG, "frame buffer alloc failed: %u x %lu bytes",
                     (unsigned)kCaptureDmaBufferCount, (unsigned long)kFrameBytes);
            for (size_t j = 0; j < kCaptureDmaBufferCount; ++j) {
                if (frame_bufs[j]) heap_caps_free(frame_bufs[j]);
            }
            esp_cam_ctlr_del(cam_handle);
            power_down();
            return ESP_ERR_NO_MEM;
        }
    }
    uint8_t *stable_frame = nullptr;

    dvp_cb_ctx ctx = {};
    for (size_t i = 0; i < kCaptureDmaBufferCount; ++i) {
        ctx.buffers[i] = frame_bufs[i];
    }
    ctx.buflen = kFrameBytes;
    ctx.received = 0;
    ctx.frame_count = 0;
    ctx.next_buffer = 0;
    ctx.last_received = 0;
    ctx.captured_buffer = nullptr;
    ctx.done_sem = xSemaphoreCreateBinary();
    if (!ctx.done_sem) {
        ESP_LOGE(TAG, "capture semaphore alloc failed");
        for (size_t i = 0; i < kCaptureDmaBufferCount; ++i) {
            heap_caps_free(frame_bufs[i]);
        }
        esp_cam_ctlr_del(cam_handle);
        power_down();
        return ESP_ERR_NO_MEM;
    }

    esp_cam_ctlr_evt_cbs_t cbs = {};
    cbs.on_get_new_trans = on_get_new_trans;
    cbs.on_trans_finished = on_trans_finished;
    ret = esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, &ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "register cbs failed: %s", esp_err_to_name(ret));
        vSemaphoreDelete(ctx.done_sem);
        for (size_t i = 0; i < kCaptureDmaBufferCount; ++i) {
            heap_caps_free(frame_bufs[i]);
        }
        esp_cam_ctlr_del(cam_handle);
        power_down();
        return ret;
    }

    ret = esp_cam_ctlr_enable(cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_cam_ctlr_enable failed: %s", esp_err_to_name(ret));
        vSemaphoreDelete(ctx.done_sem);
        for (size_t i = 0; i < kCaptureDmaBufferCount; ++i) {
            heap_caps_free(frame_bufs[i]);
        }
        esp_cam_ctlr_del(cam_handle);
        power_down();
        return ret;
    }

    bool got_frame = false;
    bool cam_running = false;
    ret = esp_cam_ctlr_start(cam_handle);
    if (ret != ESP_OK) goto cleanup;
    cam_running = true;

    log_gpio_diagnostics();

    got_frame = xSemaphoreTake(ctx.done_sem, pdMS_TO_TICKS(5000)) == pdTRUE;
    esp_cam_ctlr_stop(cam_handle);
    esp_cam_ctlr_disable(cam_handle);
    cam_running = false;

    if (got_frame && ctx.received >= kFrameBytes) {
        ESP_LOGI(TAG, "captured frame: %u bytes (skipped %d), copying stable PSRAM frame",
                 (unsigned)ctx.received, ctx.frame_count - 1);
        stable_frame = static_cast<uint8_t *>(
            heap_caps_malloc(kFrameBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (!stable_frame) {
            stable_frame = static_cast<uint8_t *>(
                heap_caps_malloc(kFrameBytes, MALLOC_CAP_8BIT));
        }
        if (stable_frame) {
            memcpy(stable_frame, ctx.captured_buffer, kFrameBytes);
            *out_data = stable_frame;
            *out_len = kFrameBytes;
            *out_width = APP_CAMERA_SENSOR_WIDTH;
            *out_height = APP_CAMERA_SENSOR_HEIGHT;
            *out_pixelformat = kOutputPixelformat;
            stable_frame = nullptr;
        } else {
            ESP_LOGE(TAG, "stable frame alloc failed: %lu bytes",
                     (unsigned long)kFrameBytes);
            ret = ESP_ERR_NO_MEM;
        }
    } else {
        log_gpio_diagnostics();
        ESP_LOGE(TAG, "capture failed: got=%d frames=%d last=%u received=%u expected=%u",
                 got_frame ? 1 : 0, ctx.frame_count, (unsigned)ctx.last_received,
                 (unsigned)ctx.received, (unsigned)kFrameBytes);
        ret = got_frame ? ESP_ERR_INVALID_SIZE : ESP_ERR_TIMEOUT;
    }

cleanup:
    if (cam_running) {
        esp_cam_ctlr_stop(cam_handle);
        esp_cam_ctlr_disable(cam_handle);
    }
    vSemaphoreDelete(ctx.done_sem);
    if (stable_frame) heap_caps_free(stable_frame);
    for (size_t i = 0; i < kCaptureDmaBufferCount; ++i) {
        if (frame_bufs[i]) heap_caps_free(frame_bufs[i]);
    }
    esp_cam_ctlr_del(cam_handle);
    if (ret == ESP_OK) {
        // Keep sensor powered, restore LEDC MCLK (DVP released the GPIO)
        gpio_reset_pin(BSP_SP0A39_MCLK_GPIO);
        ledc_timer_resume(LEDC_LOW_SPEED_MODE, LEDC_TIMER_1);
        ledc_channel_config_t ledc_ch = {};
        ledc_ch.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_ch.channel = LEDC_CHANNEL_1;
        ledc_ch.timer_sel = LEDC_TIMER_1;
        ledc_ch.intr_type = LEDC_INTR_DISABLE;
        ledc_ch.gpio_num = BSP_SP0A39_MCLK_GPIO;
        ledc_ch.duty = 1;
        ledc_ch.hpoint = 0;
        ledc_channel_config(&ledc_ch);
    } else {
        power_down();
    }
    return ret;
}
