#include "audio_diagnostics.hpp"
#include "radio_ping.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "app_config.h"
#include "bsp.h"

namespace {
constexpr const char *TAG = "app";

AudioDiagnostics g_audio;
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

    ESP_ERROR_CHECK(bsp_i2c_init());
    bsp_i2c_scan();

    esp_err_t e;
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
    if ((e = bsp_button_init(on_button, nullptr)) != ESP_OK) {
        ESP_LOGE(TAG, "btn init: %s", esp_err_to_name(e));
    }

    g_audio.play_startup_chime();

    ESP_LOGI(TAG, "voice config: Opus %u Hz, %u ms, %d bps CBR; FLRC %lu Hz, %lu bps",
             APP_AUDIO_SAMPLE_RATE_HZ, APP_AUDIO_FRAME_MS, APP_OPUS_BITRATE_BPS,
             APP_FLRC_FREQUENCY_HZ, APP_FLRC_BITRATE_BPS);
    ESP_LOGI(TAG, "K5/PTT: hold to send FLRC ping. K3: local record/play diagnostic. K4=vol-, K6=vol+");
}
