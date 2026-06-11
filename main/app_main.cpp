#include "audio_diagnostics.hpp"
#include "camera_uart.hpp"
#include "image_transfer.hpp"
#include "radio_ping.hpp"
#include "ui_gateway.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_jpeg_common.h"

#include <stdio.h>
#include <new>

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

// K6 short/long press state
int64_t g_ptt_press_time_us = 0;
bool g_ptt_held_long = false;
esp_timer_handle_t g_ptt_timer = nullptr;
constexpr int64_t kShortPressMaxUs = 300000; // 300ms

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

// PTT long-press timer callback
void ptt_long_press_cb(void *arg)
{
    (void)arg;
    g_ptt_held_long = true;
    // In camera mode, long K6 activates voice PTT
    // In radio mode, long K6 triggers mode switch (handled in on_button release)
#if APP_RADIO_FEATURES_ENABLE && APP_RADIO_TASKS_ENABLE
    if (g_app_mode == AppMode::camera && g_radio_active) {
        g_radio.handle_button(APP_PTT_BUTTON, true);
    }
#endif
}

// Image capture task: runs on device A (camera mode) when ImageCmd received
struct ImageCaptureCtx {
    uint16_t session_id;
};

void image_capture_task(void *arg)
{
    auto *ctx = static_cast<ImageCaptureCtx *>(arg);
    uint16_t session_id = ctx->session_id;
    delete ctx;

    uint8_t *frame = nullptr;
    size_t len = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pixfmt = 0;

    bsp_lcd_set_camera_status("Remote capture...");

#if APP_AUDIO_FEATURES_ENABLE
    esp_err_t audio_e = bsp_audio_suspend();
    if (audio_e != ESP_OK) {
        ESP_LOGW(TAG, "audio suspend for image capture: %s", esp_err_to_name(audio_e));
    }
#endif

    esp_err_t e = bsp_lcd_release_for_camera();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "release lcd for image capture: %s", esp_err_to_name(e));
        bsp_lcd_reinit_after_camera();
        bsp_lcd_set_camera_status("LCD release failed");
#if APP_AUDIO_FEATURES_ENABLE
        bsp_audio_resume();
#endif
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }

    esp_err_t capture_e = g_camera_uart.capture_frame(&frame, &len, &width, &height, &pixfmt);
    ESP_LOGI(TAG, "image capture: %s %lux%lu fourcc=0x%08lx len=%u",
             esp_err_to_name(capture_e),
             static_cast<unsigned long>(width),
             static_cast<unsigned long>(height),
             static_cast<unsigned long>(pixfmt),
             static_cast<unsigned>(len));

    esp_err_t lcd_e = bsp_lcd_reinit_after_camera();
    if (lcd_e != ESP_OK) {
        ESP_LOGE(TAG, "lcd reinit after image capture: %s", esp_err_to_name(lcd_e));
    }
#if APP_AUDIO_FEATURES_ENABLE
    bsp_audio_resume();
#endif

    if (capture_e != ESP_OK || !frame) {
        bsp_lcd_set_camera_status("Capture failed");
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }

    // JPEG encode
    uint8_t *jpeg = nullptr;
    size_t jpeg_len = 0;
    e = g_radio.image_xfer().encode_frame(frame, len, width, height, pixfmt, &jpeg, &jpeg_len);
    heap_caps_free(frame);

    if (e != ESP_OK || !jpeg) {
        bsp_lcd_set_camera_status("JPEG encode failed");
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }

    char status[48];
    uint16_t total_frags = static_cast<uint16_t>(
        (jpeg_len + APP_IMAGE_FRAGMENT_DATA_SIZE - 1) / APP_IMAGE_FRAGMENT_DATA_SIZE);
    snprintf(status, sizeof(status), "Sending %u pkts...", total_frags);
    bsp_lcd_set_camera_status(status);

    // Send via radio (blocks until done or timeout)
    g_radio.send_image(jpeg, jpeg_len, session_id);

    // Wait for image_tx_task to finish transmitting before freeing jpeg
    uint32_t wait_start = xTaskGetTickCount();
    while (xTaskGetTickCount() - wait_start < pdMS_TO_TICKS(30000)) {
        if (!g_radio.image_tx_busy()) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    heap_caps_free(jpeg);
    bsp_lcd_set_camera_status("Image sent");
    g_capture_busy = false;
    vTaskDelete(nullptr);
}

// Callback: device A receives ImageCmd from B
void on_image_capture_request(uint16_t session_id)
{
    if (g_app_mode != AppMode::camera) {
        ESP_LOGW(TAG, "ImageCmd received but not in camera mode");
        return;
    }
    if (g_capture_busy) {
        ESP_LOGW(TAG, "ImageCmd ignored: capture already busy");
        return;
    }
    g_capture_busy = true;

    auto *ctx = new (std::nothrow) ImageCaptureCtx{ session_id };
    if (!ctx) {
        g_capture_busy = false;
        return;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(image_capture_task, "img_cap",
                                            APP_IMAGE_TASK_STACK_BYTES, ctx,
                                            APP_IMAGE_TASK_PRIORITY, nullptr,
                                            APP_IMAGE_TASK_CORE);
    if (ok != pdPASS) {
        delete ctx;
        g_capture_busy = false;
        ESP_LOGE(TAG, "image capture task create failed");
    }
}

// Callback: device B receives complete image
void on_image_rx_complete(ImageTransfer *xfer)
{
    ESP_LOGI(TAG, "on_image_rx_complete: xfer=%p complete=%d",
             xfer, xfer ? xfer->rx_complete() : -1);
    if (!xfer || !xfer->rx_complete()) return;

    uint8_t *rgb565 = nullptr;
    uint32_t w = 0, h = 0;
    esp_err_t e = xfer->decode_to_rgb565(&rgb565, &w, &h);
    ESP_LOGI(TAG, "decode_to_rgb565: e=%d rgb565=%p w=%lu h=%lu",
             e, rgb565, (unsigned long)w, (unsigned long)h);
    if (e == ESP_OK && rgb565) {
        uint32_t elapsed_ms = 0; // TODO: track from rx_begin
        uint32_t jpeg_size = 0;
        // Approximate jpeg_size from fragment count
        jpeg_size = xfer->rx_total_count() * APP_IMAGE_FRAGMENT_DATA_SIZE;

        ESP_LOGI(TAG, "showing image: mode=%d jpeg_size=%lu",
                 (int)g_app_mode, (unsigned long)jpeg_size);
        if (g_app_mode == AppMode::radio) {
            ui_gw_rx_complete(reinterpret_cast<const uint16_t *>(rgb565), w, h,
                              jpeg_size, elapsed_ms);
        } else {
            bsp_lcd_show_rgb565_photo(reinterpret_cast<const uint16_t *>(rgb565), w, h);
            bsp_lcd_set_camera_status("Photo received");
        }
        jpeg_free_align(rgb565);
    } else {
        ESP_LOGE(TAG, "decode failed: e=%d", e);
        if (g_app_mode == AppMode::radio) {
            ui_gw_rx_failed("Decode failed");
        } else {
            bsp_lcd_set_camera_status("Decode failed");
        }
    }

    xfer->rx_reset();
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

// Radio mode: progress callback for UI update during image RX
void on_image_rx_progress(uint16_t received, uint16_t total, int16_t rssi)
{
    if (g_app_mode == AppMode::radio) {
        if (received <= 1) {
            ui_gw_rx_begin(0, total);
        }
        ui_gw_rx_progress(received, total, rssi);
    }
}

// Gateway UI capture callback — triggers remote photo via radio
void on_gw_capture(void)
{
    ESP_LOGI(TAG, "UI capture: trigger remote photo");
    g_radio.trigger_image_capture();
}

void on_button(bsp_btn_id_t id, bool pressed, void *user)
{
    (void)user;

    // In radio mode, route all keys to the gateway UI
    if (g_app_mode == AppMode::radio) {
        // K6 long press (>1.5s) → switch mode (keep as escape hatch)
        if (id == BSP_BTN_PTT) {
            if (pressed) {
                g_ptt_press_time_us = esp_timer_get_time();
                g_ptt_held_long = false;
                if (g_ptt_timer) {
                    esp_timer_start_once(g_ptt_timer, 1500000); // 1.5s for mode switch
                }
            } else {
                if (g_ptt_timer) {
                    esp_timer_stop(g_ptt_timer);
                }
                if (g_ptt_held_long) {
                    switch_mode_and_restart();
                } else {
                    ui_gw_key_event(id, true);
                }
                g_ptt_held_long = false;
            }
            return;
        }
        ui_gw_key_event(id, pressed);
        return;
    }

    // Camera mode: keep legacy behavior
    if (id == BSP_BTN_USER1) {
        if (pressed) {
            switch_mode_and_restart();
        }
        return;
    }

    if (id == BSP_BTN_PTT) {
        if (pressed) {
            g_ptt_press_time_us = esp_timer_get_time();
            g_ptt_held_long = false;
            if (g_ptt_timer) {
                esp_timer_start_once(g_ptt_timer, kShortPressMaxUs);
            }
        } else {
            if (g_ptt_timer) {
                esp_timer_stop(g_ptt_timer);
            }
            if (g_ptt_held_long) {
#if APP_RADIO_FEATURES_ENABLE && APP_RADIO_TASKS_ENABLE
                if (g_radio_active) {
                    g_radio.handle_button(id, false);
                }
#endif
            }
            g_ptt_held_long = false;
        }
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
    {
        bool radio_ok = true;
        if ((e = g_radio.init()) != ESP_OK) {
            ESP_LOGE(TAG, "radio init: %s", esp_err_to_name(e));
            radio_ok = false;
        }
        if (radio_ok) {
            // Register image transfer callbacks on both modes
            g_radio.set_image_capture_cb(on_image_capture_request);
            g_radio.set_image_rx_complete_cb(on_image_rx_complete);
            g_radio.set_image_rx_progress_cb(on_image_rx_progress);
        }
        if (g_app_mode == AppMode::radio && radio_ok) {
#if APP_RADIO_TASKS_ENABLE
            if ((e = g_radio.start()) != ESP_OK) {
                ESP_LOGE(TAG, "radio task start: %s", esp_err_to_name(e));
            } else {
                g_radio_active = true;
            }
#else
            ESP_LOGW(TAG, "radio initialized but tasks/RX disabled for camera isolation");
#endif
        } else if (g_app_mode == AppMode::camera && radio_ok) {
            // Camera mode: start all radio tasks (voice tasks idle, image TX active)
            if ((e = g_radio.start()) != ESP_OK) {
                ESP_LOGE(TAG, "radio task start (camera mode): %s", esp_err_to_name(e));
            }
            ESP_LOGI(TAG, "camera mode: radio initialized for image transfer");
        }
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
    } else if (g_app_mode == AppMode::radio) {
        if ((e = bsp_lcd_start_gateway_ui()) != ESP_OK) {
            ESP_LOGE(TAG, "gateway ui start: %s", esp_err_to_name(e));
        } else {
            ui_gw_set_capture_cb(on_gw_capture);
        }
    } else if ((e = bsp_lcd_start_camera_ui(on_lcd_capture, nullptr)) != ESP_OK) {
        ESP_LOGE(TAG, "camera ui start: %s", esp_err_to_name(e));
    }
#if APP_AUDIO_FEATURES_ENABLE
    if ((e = bsp_button_init(on_button, nullptr)) != ESP_OK) {
        ESP_LOGE(TAG, "btn init: %s", esp_err_to_name(e));
    }

    // Create PTT long-press timer for K6 short/long press detection
    const esp_timer_create_args_t ptt_timer_args = {
        .callback = ptt_long_press_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ptt_long",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&ptt_timer_args, &g_ptt_timer);

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
