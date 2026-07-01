#include "audio_diagnostics.hpp"
#include "camera_uart.hpp"
#include "image_transfer.hpp"
#include "radio_ping.hpp"
#include "opus_codec.hpp"
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
#include "esp_jpeg_dec.h"

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

// Auto-capture timer state
esp_timer_handle_t g_auto_capture_timer = nullptr;
esp_timer_handle_t g_countdown_timer = nullptr;
uint32_t g_capture_interval_sec = APP_AUTO_CAPTURE_DEFAULT_SEC;
bool g_audio_clip_enabled = APP_AUDIO_CLIP_DEFAULT_ENABLE;
uint16_t g_auto_session_id = 0x8000;
int64_t g_last_capture_time_us = 0;

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

uint32_t load_capture_interval()
{
    nvs_handle_t nvs;
    uint32_t val = APP_AUTO_CAPTURE_DEFAULT_SEC;
    if (nvs_open(kNvsNs, NVS_READONLY, &nvs) == ESP_OK) {
        (void)nvs_get_u32(nvs, "interval", &val);
        nvs_close(nvs);
    }
    return val;
}

void save_capture_interval(uint32_t sec)
{
    nvs_handle_t nvs;
    esp_err_t e = nvs_open(kNvsNs, NVS_READWRITE, &nvs);
    if (e != ESP_OK) return;
    e = nvs_set_u32(nvs, "interval", sec);
    if (e == ESP_OK) e = nvs_commit(nvs);
    nvs_close(nvs);
}

void on_image_capture_request(uint16_t session_id);

void auto_capture_timer_cb(void *arg)
{
    (void)arg;
    if (g_app_mode != AppMode::camera) return;
    if (g_capture_busy) {
        ESP_LOGW(TAG, "auto-capture skipped: busy");
        return;
    }
    g_last_capture_time_us = esp_timer_get_time();
    uint16_t sid = g_auto_session_id++;
    if (g_auto_session_id == 0) g_auto_session_id = 0x8000;
    ESP_LOGI(TAG, "auto-capture trigger: session=%u interval=%lus",
             sid, static_cast<unsigned long>(g_capture_interval_sec));
    on_image_capture_request(sid);
}

void start_auto_capture_timer()
{
    if (g_auto_capture_timer) {
        esp_timer_stop(g_auto_capture_timer);
    }
    if (g_capture_interval_sec == 0) {
        ESP_LOGI(TAG, "auto-capture disabled");
        return;
    }
    uint64_t period_us = static_cast<uint64_t>(g_capture_interval_sec) * 1000000ULL;
    if (!g_auto_capture_timer) {
        const esp_timer_create_args_t args = {
            .callback = auto_capture_timer_cb,
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "auto_cap",
            .skip_unhandled_events = true,
        };
        esp_timer_create(&args, &g_auto_capture_timer);
    }
    esp_timer_start_periodic(g_auto_capture_timer, period_us);
    ESP_LOGI(TAG, "auto-capture timer started: %lus", static_cast<unsigned long>(g_capture_interval_sec));
}

void countdown_timer_cb(void *arg)
{
    (void)arg;
    if (g_app_mode != AppMode::camera) return;
    if (g_capture_busy) return;

    char buf[48];
    if (g_capture_interval_sec == 0) {
        snprintf(buf, sizeof(buf), "Auto: Off");
    } else {
        int64_t elapsed_us = esp_timer_get_time() - g_last_capture_time_us;
        int32_t remaining = static_cast<int32_t>(g_capture_interval_sec) -
                            static_cast<int32_t>(elapsed_us / 1000000LL);
        if (remaining < 0) remaining = 0;

        const char *unit;
        uint32_t val;
        if (g_capture_interval_sec >= 3600) {
            unit = "h"; val = g_capture_interval_sec / 3600;
        } else if (g_capture_interval_sec >= 60) {
            unit = "min"; val = g_capture_interval_sec / 60;
        } else {
            unit = "s"; val = g_capture_interval_sec;
        }
        snprintf(buf, sizeof(buf), "%lu%s | %lds", static_cast<unsigned long>(val), unit,
                 static_cast<long>(remaining));
    }
    if (g_audio_clip_enabled) {
        strncat(buf, " radio", sizeof(buf) - strlen(buf) - 1);
    }
    bsp_lcd_set_camera_status(buf);
}

void update_camera_timer_status()
{
    g_last_capture_time_us = esp_timer_get_time();
    countdown_timer_cb(nullptr);
}

void start_countdown_timer()
{
    if (g_countdown_timer) return;
    const esp_timer_create_args_t args = {
        .callback = countdown_timer_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "countdown",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&args, &g_countdown_timer);
    esp_timer_start_periodic(g_countdown_timer, 1000000ULL);
}

void on_config_received(uint8_t key, uint32_t value)
{
    if (key == APP_CFG_KEY_INTERVAL) {
        if (value != 0 && value < APP_AUTO_CAPTURE_MIN_SEC) {
            value = APP_AUTO_CAPTURE_MIN_SEC;
        }
        g_capture_interval_sec = value;
        save_capture_interval(value);
        start_auto_capture_timer();
        update_camera_timer_status();
        ESP_LOGI(TAG, "config: interval=%lus", static_cast<unsigned long>(value));
    } else if (key == APP_CFG_KEY_INTER_PACKET) {
        g_radio.set_inter_packet_us(value);
        ESP_LOGI(TAG, "config: inter_packet=%luus", static_cast<unsigned long>(value));
        char buf[16];
        snprintf(buf, sizeof(buf), "%luus", static_cast<unsigned long>(value));
        bsp_lcd_set_camera_status(buf);
    } else if (key == APP_CFG_KEY_AUDIO_CLIP) {
        g_audio_clip_enabled = (value != 0);
        update_camera_timer_status();
        ESP_LOGI(TAG, "config: audio_clip=%s", g_audio_clip_enabled ? "on" : "off");
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

    uint32_t t_cmd = static_cast<uint32_t>(esp_log_timestamp());
    ESP_LOGI(TAG, "[TIMING] cmd received t=0ms");

    bsp_lcd_set_camera_status("Remote capture...");

    // Snapshot pre-encoded Opus ring buffer before pausing audio
    uint8_t *opus_buf = nullptr;
    size_t opus_len = 0;
    if (g_audio_clip_enabled) {
        opus_buf = static_cast<uint8_t *>(
            heap_caps_malloc(APP_AUDIO_CLIP_MAX_OPUS_BYTES, MALLOC_CAP_SPIRAM));
        if (opus_buf) {
            opus_len = g_radio.snapshot_opus(opus_buf, APP_AUDIO_CLIP_MAX_OPUS_BYTES);
            ESP_LOGI(TAG, "opus snapshot: %u bytes", static_cast<unsigned>(opus_len));
        }
    }

#if APP_AUDIO_FEATURES_ENABLE
    g_radio.pause_audio_capture();
    vTaskDelay(pdMS_TO_TICKS(APP_AUDIO_FRAME_MS + 5));
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
        g_radio.resume_audio_capture();
#endif
        heap_caps_free(opus_buf);
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }

    uint32_t t_cam_start = static_cast<uint32_t>(esp_log_timestamp());
    ESP_LOGI(TAG, "[TIMING] camera init start +%lums", static_cast<unsigned long>(t_cam_start - t_cmd));

    esp_err_t capture_e = g_camera_uart.capture_frame(&frame, &len, &width, &height, &pixfmt);

    uint32_t t_cam_done = static_cast<uint32_t>(esp_log_timestamp());
    ESP_LOGI(TAG, "[TIMING] capture done +%lums (camera=%lums) %lux%lu %u bytes",
             static_cast<unsigned long>(t_cam_done - t_cmd),
             static_cast<unsigned long>(t_cam_done - t_cam_start),
             static_cast<unsigned long>(width),
             static_cast<unsigned long>(height),
             static_cast<unsigned>(len));

    // LCD reinit and audio resume deferred until after transmission

    if (capture_e != ESP_OK || !frame) {
        bsp_lcd_reinit_after_camera();
#if APP_AUDIO_FEATURES_ENABLE
        bsp_audio_resume();
#endif
        bsp_lcd_set_camera_status("Capture failed");
        heap_caps_free(opus_buf);
        g_radio.resume_audio_capture();
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }

    // JPEG encode
    uint8_t *jpeg = nullptr;
    size_t jpeg_len = 0;
    uint32_t t_jpeg_start = static_cast<uint32_t>(esp_log_timestamp());
    e = g_radio.image_xfer().encode_frame(frame, len, width, height, pixfmt, &jpeg, &jpeg_len);
    heap_caps_free(frame);
    uint32_t t_jpeg_done = static_cast<uint32_t>(esp_log_timestamp());
    ESP_LOGI(TAG, "[TIMING] JPEG done +%lums (jpeg=%lums) %u bytes",
             static_cast<unsigned long>(t_jpeg_done - t_cmd),
             static_cast<unsigned long>(t_jpeg_done - t_jpeg_start),
             static_cast<unsigned>(jpeg_len));

    if (e != ESP_OK || !jpeg) {
        bsp_lcd_reinit_after_camera();
#if APP_AUDIO_FEATURES_ENABLE
        bsp_audio_resume();
#endif
        bsp_lcd_set_camera_status("JPEG encode failed");
        heap_caps_free(opus_buf);
        g_radio.resume_audio_capture();
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }

    // --- Step 1: Send JPEG (no audio flag) ---
    uint16_t total_frags = static_cast<uint16_t>(
        (jpeg_len + APP_IMAGE_FRAGMENT_DATA_SIZE - 1) / APP_IMAGE_FRAGMENT_DATA_SIZE);
    ESP_LOGI(TAG, "sending JPEG: %u pkts, %u bytes", total_frags, static_cast<unsigned>(jpeg_len));

    g_radio.send_image(jpeg, jpeg_len, session_id);

    uint32_t wait_start = xTaskGetTickCount();
    while (xTaskGetTickCount() - wait_start < pdMS_TO_TICKS(30000)) {
        if (!g_radio.image_tx_busy()) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    heap_caps_free(jpeg);

    uint32_t t_jpeg_tx_done = static_cast<uint32_t>(esp_log_timestamp());
    ESP_LOGI(TAG, "[TIMING] JPEG TX done +%lums",
             static_cast<unsigned long>(t_jpeg_tx_done - t_cmd));

    // --- Step 2: Send pre-encoded Opus (already snapshot at start) ---
    ESP_LOGI(TAG, "[TIMING] Opus pre-encoded, %u bytes ready",
             static_cast<unsigned>(opus_len));

    // --- Step 3: Send Opus as second transfer (with audio flag) ---
    if (opus_buf && opus_len > 0) {
        uint16_t audio_session = session_id | APP_AUDIO_SESSION_FLAG;
        uint16_t audio_frags = static_cast<uint16_t>(
            (opus_len + APP_IMAGE_FRAGMENT_DATA_SIZE - 1) / APP_IMAGE_FRAGMENT_DATA_SIZE);
        ESP_LOGI(TAG, "sending audio: %u pkts, %u bytes", audio_frags, static_cast<unsigned>(opus_len));

        g_radio.send_image(opus_buf, opus_len, audio_session);

        wait_start = xTaskGetTickCount();
        while (xTaskGetTickCount() - wait_start < pdMS_TO_TICKS(30000)) {
            if (!g_radio.image_tx_busy()) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    heap_caps_free(opus_buf);

    uint32_t t_all_done = static_cast<uint32_t>(esp_log_timestamp());
    ESP_LOGI(TAG, "[TIMING] total +%lums | img_prep=%lu img_tx=%lu audio_tx=%lums",
             static_cast<unsigned long>(t_all_done - t_cmd),
             static_cast<unsigned long>(t_jpeg_done - t_cmd),
             static_cast<unsigned long>(t_jpeg_tx_done - t_jpeg_done),
             static_cast<unsigned long>(t_all_done - t_jpeg_tx_done));

    // Reinit LCD now that transmission is done
    esp_err_t lcd_e = bsp_lcd_reinit_after_camera();
    if (lcd_e != ESP_OK) {
        ESP_LOGE(TAG, "lcd reinit after tx: %s", esp_err_to_name(lcd_e));
    }
#if APP_AUDIO_FEATURES_ENABLE
    bsp_audio_resume();
#endif
    char status[48];
    snprintf(status, sizeof(status), "Done %u pkts", total_frags);
    bsp_lcd_set_camera_status(status);

    g_radio.resume_audio_capture();
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
void play_audio_clip(const uint8_t *opus_packed, size_t total_len)
{
    OpusCodec clip_codec;
    if (clip_codec.init() != ESP_OK) {
        ESP_LOGE(TAG, "play_audio_clip: codec init failed");
        return;
    }
    bsp_audio_pa_enable(true);

    size_t pos = 0;
    int16_t pcm[APP_AUDIO_FRAME_SAMPLES];
    int16_t stereo[APP_AUDIO_FRAME_SAMPLES * 2];
    uint32_t frames_played = 0;

    while (pos < total_len) {
        uint8_t flen = opus_packed[pos++];
        if (flen == 0 || pos + flen > total_len) break;
        int decoded = clip_codec.decode(&opus_packed[pos], flen, pcm, APP_AUDIO_FRAME_SAMPLES);
        pos += flen;
        if (decoded <= 0) continue;
        for (int i = 0; i < decoded; i++) {
            stereo[2 * i] = pcm[i];
            stereo[2 * i + 1] = pcm[i];
        }
        size_t written = 0;
        bsp_audio_write(stereo, decoded * 2 * sizeof(int16_t), &written);
        frames_played++;
    }
    ESP_LOGI(TAG, "audio clip played: %lu frames", static_cast<unsigned long>(frames_played));
    bsp_audio_pa_enable(false);
}

void on_image_rx_complete(ImageTransfer *xfer)
{
    ESP_LOGI(TAG, "on_image_rx_complete: xfer=%p complete=%d",
             xfer, xfer ? xfer->rx_complete() : -1);
    if (!xfer || !xfer->rx_complete()) return;

    uint16_t sid = xfer->rx_session_id();
    bool is_audio = (sid & APP_AUDIO_SESSION_FLAG) != 0;

    // Reassemble the received data
    uint8_t *raw = nullptr;
    size_t raw_len = 0;
    esp_err_t e = xfer->rx_reassemble(&raw, &raw_len);
    if (e != ESP_OK || !raw) {
        ESP_LOGE(TAG, "rx_reassemble failed: %d", e);
        xfer->rx_reset();
        return;
    }

    if (is_audio) {
        // Pure Opus audio transfer
        ESP_LOGI(TAG, "audio transfer received: %u bytes", static_cast<unsigned>(raw_len));
        play_audio_clip(raw, raw_len);
    } else {
        // Pure JPEG image transfer
        ESP_LOGI(TAG, "image transfer received: %u bytes", static_cast<unsigned>(raw_len));

        jpeg_dec_config_t dec_cfg = DEFAULT_JPEG_DEC_CONFIG();
        dec_cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_LE;

        jpeg_dec_handle_t decoder = nullptr;
        jpeg_error_t jerr = jpeg_dec_open(&dec_cfg, &decoder);
        if (jerr == JPEG_ERR_OK && decoder) {
            jpeg_dec_io_t io = {};
            io.inbuf = const_cast<unsigned char *>(raw);
            io.inbuf_len = static_cast<int>(raw_len);

            jpeg_dec_header_info_t header = {};
            jerr = jpeg_dec_parse_header(decoder, &io, &header);
            if (jerr == JPEG_ERR_OK) {
                int outbuf_len = header.width * header.height * 2;
                uint8_t *rgb565 = static_cast<uint8_t *>(jpeg_calloc_align(outbuf_len, 16));
                if (rgb565) {
                    io.outbuf = rgb565;
                    jerr = jpeg_dec_process(decoder, &io);
                    if (jerr == JPEG_ERR_OK) {
                        uint32_t w = header.width;
                        uint32_t h = header.height;
                        if (g_app_mode == AppMode::radio) {
                            ui_gw_rx_complete(reinterpret_cast<const uint16_t *>(rgb565), w, h,
                                              static_cast<uint32_t>(raw_len), g_radio.last_transfer_ms());
                        } else {
                            bsp_lcd_show_rgb565_photo(reinterpret_cast<const uint16_t *>(rgb565), w, h);
                            bsp_lcd_set_camera_status("Photo received");
                        }
                    }
                    jpeg_free_align(rgb565);
                }
            }
            jpeg_dec_close(decoder);
        }
    }

    heap_caps_free(raw);
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
        if (received == 0) {
            ui_gw_rx_begin(0, total);
            return;
        }
        ui_gw_rx_progress(received, total, rssi);
    }
}

void on_image_rx_eot_nack(uint16_t missing_count, bool is_first_eot)
{
    if (g_app_mode == AppMode::radio) {
        ui_gw_rx_eot_nack(missing_count, is_first_eot);
    }
}

// Gateway UI capture callback — triggers remote photo via radio
void on_gw_capture(void)
{
    ESP_LOGI(TAG, "UI capture: trigger remote photo");
    g_radio.trigger_image_capture();
}

// Gateway UI interval change callback — sends config to camera node
bool on_gw_interval_change(uint32_t interval_sec)
{
    ESP_LOGI(TAG, "UI interval change: %lus", static_cast<unsigned long>(interval_sec));
    return g_radio.send_config(APP_CFG_KEY_INTERVAL, interval_sec);
}

bool on_gw_pkt_delay_change(uint32_t us)
{
    ESP_LOGI(TAG, "UI pkt delay change: %luus", static_cast<unsigned long>(us));
    return g_radio.send_config(APP_CFG_KEY_INTER_PACKET, us);
}

bool on_gw_audio_clip_change(uint32_t enable)
{
    ESP_LOGI(TAG, "UI audio clip: %s", enable ? "on" : "off");
    return g_radio.send_config(APP_CFG_KEY_AUDIO_CLIP, enable);
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
            g_radio.set_image_rx_eot_cb(on_image_rx_eot_nack);
            g_radio.set_config_received_cb(on_config_received);
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
            // Start auto-capture timer
            g_capture_interval_sec = load_capture_interval();
            start_auto_capture_timer();
            update_camera_timer_status();
            start_countdown_timer();
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
            ui_gw_set_interval_cb(on_gw_interval_change);
            ui_gw_set_pkt_delay_cb(on_gw_pkt_delay_change);
            ui_gw_set_audio_clip_cb(on_gw_audio_clip_change);
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
    bsp_audio_set_volume(100);

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
