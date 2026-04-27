#include "audio_diagnostics.hpp"
#include "camera_uart.hpp"
#include "radio_ping.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "app_config.h"
#include "bsp.h"

namespace {
constexpr const char *TAG = "app";

AudioDiagnostics g_audio;
CameraUartStreamer g_camera_uart;
RadioPing g_radio;

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
    bsp_i2c_scan();

    bsp_con6_peripheral_t con6 = BSP_CON6_PERIPHERAL_LCD_ST7789;
    if ((e = bsp_con6_detect(&con6)) != ESP_OK) {
        ESP_LOGE(TAG, "CON6 detect: %s", esp_err_to_name(e));
    }

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
    if (con6 == BSP_CON6_PERIPHERAL_CAMERA_GC032A) {
#if APP_CAMERA_UART_ENABLE
        if ((e = g_camera_uart.start()) != ESP_OK) {
            ESP_LOGE(TAG, "camera uart start: %s", esp_err_to_name(e));
        }
#else
        ESP_LOGW(TAG, "GC032A detected but APP_CAMERA_UART_ENABLE is disabled");
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
    if (con6 == BSP_CON6_PERIPHERAL_CAMERA_GC032A) {
        ESP_LOGI(TAG, "camera UART2: %d baud on GPIO%d TX / GPIO%d RX; framed GC032A YVYU stream",
                 APP_CAMERA_UART_BAUD, BSP_UART2_TX_GPIO, BSP_UART2_RX_GPIO);
    } else {
        ESP_LOGI(TAG, "CON6 display: ST7789T3 %ux%u SPI 4W with LVGL touch",
                 APP_LCD_H_RES, APP_LCD_V_RES);
    }
    ESP_LOGI(TAG, "K5/PTT: hold to send Opus voice over FLRC. K3: local record/play diagnostic. K4=vol-, K6=vol+");
}
