#include "audio_diagnostics.hpp"
#include "camera_uart.hpp"
#include "radio_ping.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "app_config.h"
#include "bsp.h"

namespace {
constexpr const char *TAG = "app";

AudioDiagnostics g_audio;
CameraUartStreamer g_camera_uart;
RadioPing g_radio;
volatile bool g_capture_busy = false;

void camera_capture_task(void *arg)
{
    (void)arg;
    uint8_t *frame = nullptr;
    size_t len = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pixfmt = 0;

    bsp_lcd_set_camera_status("Preparing camera...");
    esp_err_t e = bsp_lcd_release_for_camera();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "release lcd for camera: %s", esp_err_to_name(e));
        bsp_lcd_reinit_after_camera();
        bsp_lcd_set_camera_status("LCD release failed");
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }

    e = g_camera_uart.capture_frame(&frame, &len, &width, &height, &pixfmt);
    ESP_LOGI(TAG, "capture result=%s frame=%p len=%u %lux%lu fourcc=0x%08lx",
             esp_err_to_name(e), frame, static_cast<unsigned>(len),
             static_cast<unsigned long>(width),
             static_cast<unsigned long>(height),
             static_cast<unsigned long>(pixfmt));

    esp_err_t lcd_e = bsp_lcd_reinit_after_camera();
    if (lcd_e != ESP_OK) {
        ESP_LOGE(TAG, "lcd reinit after camera: %s", esp_err_to_name(lcd_e));
    }

    if (e == ESP_OK && pixfmt == 0x59455247) { // 'GREY'
        if (bsp_lcd_show_gray_photo(frame, width, height) == ESP_OK) {
            bsp_lcd_set_camera_status("Captured. Touch capture to retake");
        } else {
            bsp_lcd_set_camera_status("Display photo failed");
        }
    } else if (e == ESP_OK) {
        bsp_lcd_set_camera_status("Unsupported camera pixel format");
    } else {
        bsp_lcd_set_camera_status("Capture failed");
    }

    heap_caps_free(frame);
    g_capture_busy = false;
    vTaskDelete(nullptr);
}

void on_lcd_capture(void *user)
{
    (void)user;
    if (g_capture_busy) {
        bsp_lcd_set_camera_status("Capture already running");
        return;
    }
    g_capture_busy = true;
    BaseType_t ok = xTaskCreatePinnedToCore(camera_capture_task,
                                            "touch_capture",
                                            APP_CAMERA_TASK_STACK_BYTES,
                                            nullptr,
                                            APP_CAMERA_TASK_PRIORITY + 3,
                                            nullptr,
                                            APP_CAMERA_TASK_CORE);
    if (ok != pdPASS) {
        g_capture_busy = false;
        bsp_lcd_set_camera_status("Capture task start failed");
    }
}

void on_button(bsp_btn_id_t id, bool pressed, void *user)
{
    (void)user;

    if (id == APP_PTT_BUTTON) {
        g_radio.handle_button(id, pressed);
        return;
    }

    if (id == BSP_BTN_BOOT && pressed) {
        ESP_LOGI(TAG, "boot pressed");
        return;
    }

    g_audio.handle_button(id, pressed);
}
} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Lierda L-LRMAM36-FANN4-DK01 booting");
    esp_log_level_set("RALF_LR20XX", ESP_LOG_WARN);

    esp_err_t e;
    ESP_ERROR_CHECK(bsp_i2c_init());

    printf("PSRAM free: %d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    printf("PSRAM total: %d\n", heap_caps_get_total_size(MALLOC_CAP_SPIRAM));

#if APP_CAMERA_LCD_BRINGUP
    bsp_i2c_scan();
    if ((e = g_camera_uart.init()) != ESP_OK) {
        ESP_LOGE(TAG, "camera init: %s", esp_err_to_name(e));
        return;
    }
    if ((e = bsp_lcd_init()) != ESP_OK) {
        ESP_LOGE(TAG, "lcd init: %s", esp_err_to_name(e));
        return;
    }
    if ((e = bsp_lcd_show_test_pattern()) != ESP_OK) {
        ESP_LOGE(TAG, "lcd test pattern: %s", esp_err_to_name(e));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(800));
    if ((e = bsp_lcd_start_camera_ui(on_lcd_capture, nullptr)) != ESP_OK) {
        ESP_LOGE(TAG, "camera ui start: %s", esp_err_to_name(e));
        return;
    }
    ESP_LOGI(TAG, "V02 camera/LCD validation UI ready: ST7789V3 %ux%u, SP0A39 DVP %ux%u",
             APP_LCD_H_RES, APP_LCD_V_RES,
             APP_CAMERA_SENSOR_WIDTH, APP_CAMERA_SENSOR_HEIGHT);
    return;
#endif

#if APP_CAMERA_ONLY_BRINGUP
    ESP_LOGW(TAG, "camera-only bring-up: skipping CON6 detect, LED, audio, LR2021 radio, buttons, LCD, chime");
#if APP_CAMERA_UART_ENABLE
    if ((e = g_camera_uart.start()) != ESP_OK) {
        ESP_LOGE(TAG, "camera uart start: %s", esp_err_to_name(e));
    }
    ESP_LOGI(TAG, "camera-only SP0A39 one-frame capture: MCLK GPIO%d, PWDN IOEXP P%d",
             BSP_SP0A39_MCLK_GPIO, BSP_SP0A39_PWDN_IOEXP_PIN);
#else
    ESP_LOGW(TAG, "APP_CAMERA_UART_ENABLE is disabled");
#endif
    return;
#endif

    bsp_i2c_scan();

    bsp_con6_peripheral_t con6 = BSP_CON6_PERIPHERAL_LCD_ST7789;
    if ((e = bsp_con6_detect(&con6)) != ESP_OK) {
        ESP_LOGE(TAG, "CON6 detect: %s", esp_err_to_name(e));
    }
#if APP_CON6_FORCE_CAMERA
    con6 = BSP_CON6_PERIPHERAL_CAMERA;
    ESP_LOGW(TAG, "CON6 force camera mode enabled; skipping LCD path");
#endif

    if ((e = bsp_led_init()) != ESP_OK) {
        ESP_LOGE(TAG, "led init: %s", esp_err_to_name(e));
    }
    if ((e = bsp_audio_init(APP_AUDIO_SAMPLE_RATE_HZ)) != ESP_OK) {
        ESP_LOGE(TAG, "audio init: %s", esp_err_to_name(e));
    }
    if ((e = g_audio.init()) != ESP_OK) {
        ESP_LOGE(TAG, "audio diagnostics init: %s", esp_err_to_name(e));
    }
    if ((e = g_radio.init()) != ESP_OK) {
        ESP_LOGE(TAG, "radio init: %s", esp_err_to_name(e));
    }
    if ((e = g_radio.start()) != ESP_OK) {
        ESP_LOGE(TAG, "radio task start: %s", esp_err_to_name(e));
    }
    if (con6 == BSP_CON6_PERIPHERAL_CAMERA) {
#if APP_CAMERA_UART_ENABLE
        if ((e = g_camera_uart.start()) != ESP_OK) {
            ESP_LOGE(TAG, "camera uart start: %s", esp_err_to_name(e));
        }
#else
        ESP_LOGW(TAG, "SP0A39 detected but APP_CAMERA_UART_ENABLE is disabled");
#endif
    } else {
        if ((e = bsp_lcd_init()) != ESP_OK) {
            ESP_LOGE(TAG, "lcd init: %s", esp_err_to_name(e));
        } else if ((e = bsp_lcd_start_lvgl_demo()) != ESP_OK) {
            ESP_LOGE(TAG, "lcd lvgl demo: %s", esp_err_to_name(e));
        }
    }
    if ((e = bsp_button_init(on_button, nullptr)) != ESP_OK) {
        ESP_LOGE(TAG, "btn init: %s", esp_err_to_name(e));
    }

    g_audio.play_startup_chime();

    ESP_LOGI(TAG, "voice config: Opus %u Hz, %u ms, %d bps CBR; FLRC %lu Hz, %lu bps",
             APP_AUDIO_SAMPLE_RATE_HZ, APP_AUDIO_FRAME_MS, APP_OPUS_BITRATE_BPS,
             APP_FLRC_FREQUENCY_HZ, APP_FLRC_BITRATE_BPS);
    if (con6 == BSP_CON6_PERIPHERAL_CAMERA) {
        ESP_LOGI(TAG, "camera UART2: %d baud on GPIO%d TX / GPIO%d RX; SP0A39 SPI_1BIT grayscale PGM output",
                 APP_CAMERA_UART_BAUD, BSP_UART2_TX_GPIO, BSP_UART2_RX_GPIO);
    } else {
        ESP_LOGI(TAG, "CON6 display: ST7789T3 %ux%u SPI 4W with LVGL touch",
                 APP_LCD_H_RES, APP_LCD_V_RES);
    }
    ESP_LOGI(TAG, "K5/PTT: hold to send Opus voice over FLRC. K3: local record/play diagnostic. K4=vol-, K6=vol+");
}
