#include "audio_diagnostics.hpp"
#include "camera_uart.hpp"
#include "radio_ping.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <stdio.h>

#include "app_config.h"
#include "bsp.h"

namespace {
constexpr const char *TAG = "app";
constexpr const char *kNvsNs = "app";
constexpr const char *kModeKey = "mode";

enum class AppMode : uint8_t {
    camera = 0,
    radio = 1,
};

AudioDiagnostics g_audio;
CameraUartStreamer g_camera_uart;
RadioPing g_radio;
volatile bool g_capture_busy = false;
AppMode g_app_mode = AppMode::camera;
bool g_radio_active = false;

const char *mode_name(AppMode mode)
{
    return mode == AppMode::radio ? "radio" : "camera";
}

const char *short_error_name(esp_err_t err)
{
    switch (err) {
    case ESP_ERR_NO_MEM:
        return "NO_MEM";
    case ESP_ERR_TIMEOUT:
        return "TIMEOUT";
    case ESP_ERR_INVALID_SIZE:
        return "BAD_SIZE";
    case ESP_ERR_NOT_SUPPORTED:
        return "NOT_SUP";
    default:
        return nullptr;
    }
}

void init_nvs()
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
}

AppMode load_app_mode()
{
    nvs_handle_t nvs;
    uint8_t value = static_cast<uint8_t>(AppMode::camera);
    if (nvs_open(kNvsNs, NVS_READONLY, &nvs) == ESP_OK) {
        (void)nvs_get_u8(nvs, kModeKey, &value);
        nvs_close(nvs);
    }
    return value == static_cast<uint8_t>(AppMode::radio) ? AppMode::radio : AppMode::camera;
}

void save_app_mode(AppMode mode)
{
    nvs_handle_t nvs;
    esp_err_t e = nvs_open(kNvsNs, NVS_READWRITE, &nvs);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "open mode nvs: %s", esp_err_to_name(e));
        return;
    }
    e = nvs_set_u8(nvs, kModeKey, static_cast<uint8_t>(mode));
    if (e == ESP_OK) e = nvs_commit(nvs);
    nvs_close(nvs);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "save mode nvs: %s", esp_err_to_name(e));
    }
}

void switch_mode_and_restart()
{
    AppMode next = g_app_mode == AppMode::camera ? AppMode::radio : AppMode::camera;
    ESP_LOGW(TAG, "K5 mode switch: %s -> %s, restarting",
             mode_name(g_app_mode), mode_name(next));
    save_app_mode(next);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

void camera_capture_task(void *arg)
{
    (void)arg;
    uint8_t *frame = nullptr;
    size_t len = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pixfmt = 0;

#if APP_AUDIO_FEATURES_ENABLE
#if APP_RADIO_FEATURES_ENABLE
    if (g_radio_active) {
        g_radio.suspend();
    }
#endif
    esp_err_t audio_e = bsp_audio_suspend();
    if (audio_e != ESP_OK) {
        ESP_LOGW(TAG, "audio suspend before camera: %s", esp_err_to_name(audio_e));
    }
#endif
    bsp_lcd_set_camera_status("Preparing camera...");
    esp_err_t e = bsp_lcd_release_for_camera();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "release lcd for camera: %s", esp_err_to_name(e));
        bsp_lcd_reinit_after_camera();
        bsp_lcd_set_camera_status("LCD release failed");
#if APP_AUDIO_FEATURES_ENABLE
        if ((e = bsp_audio_resume()) != ESP_OK) {
            ESP_LOGW(TAG, "audio resume after LCD release failure: %s", esp_err_to_name(e));
        }
#if APP_RADIO_FEATURES_ENABLE
        if (g_radio_active) {
            g_radio.resume();
        }
#endif
#endif
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }

    esp_err_t capture_e = g_camera_uart.capture_frame(&frame, &len, &width, &height, &pixfmt);
    ESP_LOGI(TAG, "capture result=%s frame=%p len=%u %lux%lu fourcc=0x%08lx",
             esp_err_to_name(capture_e), frame, static_cast<unsigned>(len),
             static_cast<unsigned long>(width),
             static_cast<unsigned long>(height),
             static_cast<unsigned long>(pixfmt));

    esp_err_t lcd_e = bsp_lcd_reinit_after_camera();
    if (lcd_e != ESP_OK) {
        ESP_LOGE(TAG, "lcd reinit after camera: %s", esp_err_to_name(lcd_e));
    }
#if APP_AUDIO_FEATURES_ENABLE
    if ((e = bsp_audio_resume()) != ESP_OK) {
        ESP_LOGW(TAG, "audio resume after camera: %s", esp_err_to_name(e));
    }
#if APP_RADIO_FEATURES_ENABLE
    if (g_radio_active) {
        g_radio.resume();
    }
#endif
#endif

    if (capture_e == ESP_OK && (pixfmt == 0x56595559 || pixfmt == 0x59565955 ||
                                pixfmt == 0x55595659 || pixfmt == 0x59555956)) {
        if (bsp_lcd_show_yuv422_photo(frame, width, height, pixfmt) == ESP_OK) {
            bsp_lcd_set_camera_status("Captured. Touch capture to retake");
        } else {
            bsp_lcd_set_camera_status("Display photo failed");
        }
    } else if (capture_e == ESP_OK && pixfmt == 0x59455247) { // 'GREY'
        if (bsp_lcd_show_gray_photo(frame, width, height) == ESP_OK) {
            bsp_lcd_set_camera_status("Captured. Touch capture to retake");
        } else {
            bsp_lcd_set_camera_status("Display photo failed");
        }
    } else if (capture_e == ESP_OK) {
        char status[64];
        snprintf(status, sizeof(status), "Unsupported pixel 0x%08lx",
                 static_cast<unsigned long>(pixfmt));
        bsp_lcd_set_camera_status(status);
    } else {
        bsp_lcd_clear_camera_photo();
        char status[64];
        const char *short_name = short_error_name(capture_e);
        if (short_name) {
            snprintf(status, sizeof(status), "Fail:%s", short_name);
        } else {
            snprintf(status, sizeof(status), "Fail:0x%lx",
                     static_cast<unsigned long>(capture_e));
        }
        bsp_lcd_set_camera_status(status);
    }

    heap_caps_free(frame);
    g_capture_busy = false;
    vTaskDelete(nullptr);
}

void on_lcd_capture(void *user)
{
    (void)user;
    if (g_app_mode == AppMode::radio) {
        bsp_lcd_set_camera_status("Radio mode. Press K5 for camera mode");
        return;
    }
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

    if (id == BSP_BTN_USER1) {
        if (pressed) {
            switch_mode_and_restart();
        }
        return;
    }

    if (id == APP_PTT_BUTTON) {
#if APP_RADIO_FEATURES_ENABLE && APP_RADIO_TASKS_ENABLE
        if (!g_radio_active) {
            if (pressed) ESP_LOGW(TAG, "PTT ignored in camera mode");
            return;
        }
        g_radio.handle_button(id, pressed);
#else
        if (pressed) {
            ESP_LOGW(TAG, "PTT ignored: radio tasks disabled for camera/radio isolation");
        }
#endif
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
    init_nvs();
    g_app_mode = load_app_mode();
    ESP_LOGI(TAG, "app mode: %s", mode_name(g_app_mode));

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

#if APP_AUDIO_FEATURES_ENABLE
    if ((e = bsp_led_init()) != ESP_OK) {
        ESP_LOGE(TAG, "led init: %s", esp_err_to_name(e));
    }
    if ((e = bsp_audio_init(APP_AUDIO_SAMPLE_RATE_HZ)) != ESP_OK) {
        ESP_LOGE(TAG, "audio init: %s", esp_err_to_name(e));
    }
    if ((e = g_audio.init()) != ESP_OK) {
        ESP_LOGE(TAG, "audio diagnostics init: %s", esp_err_to_name(e));
    }
#if APP_RADIO_FEATURES_ENABLE
    if (g_app_mode == AppMode::radio) {
        bool radio_ok = true;
        if ((e = g_radio.init()) != ESP_OK) {
            ESP_LOGE(TAG, "radio init: %s", esp_err_to_name(e));
            radio_ok = false;
        }
#if APP_RADIO_TASKS_ENABLE
        if (radio_ok) {
            if ((e = g_radio.start()) != ESP_OK) {
                ESP_LOGE(TAG, "radio task start: %s", esp_err_to_name(e));
            } else {
                g_radio_active = true;
            }
        }
#else
        ESP_LOGW(TAG, "radio initialized but tasks/RX disabled for camera isolation");
#endif
    } else {
        ESP_LOGW(TAG, "camera mode: radio is not initialized");
    }
#else
    ESP_LOGW(TAG, "radio feature disabled for camera/audio isolation");
#endif
#endif
    if ((e = g_camera_uart.init()) != ESP_OK) {
        ESP_LOGE(TAG, "camera init: %s", esp_err_to_name(e));
    }
#if APP_CAMERA_UART_ENABLE
    if ((e = g_camera_uart.start()) != ESP_OK) {
        ESP_LOGE(TAG, "camera uart start: %s", esp_err_to_name(e));
    }
#endif
    if ((e = bsp_lcd_init()) != ESP_OK) {
        ESP_LOGE(TAG, "lcd init: %s", esp_err_to_name(e));
    } else if ((e = bsp_lcd_start_camera_ui(on_lcd_capture, nullptr)) != ESP_OK) {
        ESP_LOGE(TAG, "camera ui start: %s", esp_err_to_name(e));
    }
#if APP_AUDIO_FEATURES_ENABLE
    if ((e = bsp_button_init(on_button, nullptr)) != ESP_OK) {
        ESP_LOGE(TAG, "btn init: %s", esp_err_to_name(e));
    }

    g_audio.play_startup_chime();

    ESP_LOGI(TAG, "audio config: %u Hz local record/playback",
             APP_AUDIO_SAMPLE_RATE_HZ);
#if APP_RADIO_FEATURES_ENABLE
#if APP_RADIO_TASKS_ENABLE
    ESP_LOGI(TAG, "voice config: Opus %u Hz, %u ms, %d bps CBR; FLRC %lu Hz, %lu bps",
             APP_AUDIO_SAMPLE_RATE_HZ, APP_AUDIO_FRAME_MS, APP_OPUS_BITRATE_BPS,
             APP_FLRC_FREQUENCY_HZ, APP_FLRC_BITRATE_BPS);
#else
    ESP_LOGW(TAG, "FLRC radio init only; RX/TX tasks disabled in this build");
#endif
#else
    ESP_LOGW(TAG, "FLRC voice disabled in this build");
#endif
#else
    ESP_LOGW(TAG, "audio/radio/button features disabled");
#endif
    ESP_LOGI(TAG, "display: ST7789T3 %ux%u camera capture UI",
             APP_LCD_H_RES, APP_LCD_V_RES);
    ESP_LOGI(TAG, "K5: switch camera/radio mode. K6/PTT: FLRC voice in radio mode. K4=vol+, K3=vol-");
}
