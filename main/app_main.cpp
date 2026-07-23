#include "audio_diagnostics.hpp"
#include "camera_uart.hpp"
#include "image_transfer.hpp"
#include "radio_ping.hpp"
#include "opus_codec.hpp"
#include "ui_gateway.h"
#include "wifi_manager.h"
#include "image_store.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_jpeg_common.h"
#include "esp_jpeg_dec.h"

#include <stdio.h>
#include <new>
#include <cmath>

#include "app_config.h"
#include "bsp.h"

#if APP_VOICE_ALARM_ENABLE
#include "warning_voice_opus.h"
#endif

volatile bool g_low_power_enabled = false;

namespace {
constexpr const char *TAG = "app";
constexpr const char *kNvsNs = "app";
constexpr const char *kModeKey = "mode";

uint32_t s_last_gw_capture_ms = 0;

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
volatile bool g_voice_alarm_enabled = false;
uint16_t g_auto_session_id = 0x8000;
int64_t g_last_capture_time_us = 0;

// K6 short/long press state
int64_t g_ptt_press_time_us = 0;
bool g_ptt_held_long = false;
esp_timer_handle_t g_cooldown_retry_timer = nullptr;
esp_timer_handle_t g_ptt_timer = nullptr;

// Continuous video stream: one-shot timer that fires the next capture request a
// short delay after a frame finishes, so the re-trigger runs off the radio task
// (not re-entrant inside the rx-complete callback).
esp_timer_handle_t g_stream_next_timer = nullptr;

// Gateway AV sync queue. Each received frame (image + audio) is paired into an
// AVFrame struct by the rx-complete callback (non-blocking). gateway_audio_task
// dequeues these pairs and displays the image immediately before playing its
// corresponding audio segment, naturally syncing video to the I2S clock-driven
// audio timeline. There is NO prebuffer: playback starts as soon as a frame is
// available (lowest latency). To stop latency from accumulating when frames
// pile up faster than they drain, a frame that has waited longer than
// APP_GW_AV_MAX_LATENCY_MS in the queue is dropped (image + audio together, so
// sync is preserved) whenever a newer frame is already queued behind it.
struct AVFrame {
    uint16_t *rgb565;       // decoded image in SPIRAM, null if no image
    uint32_t  width;
    uint32_t  height;
    uint8_t  *opus_packed;  // opus blob in SPIRAM, null if no audio
    size_t    opus_len;
    uint32_t  jpeg_size;    // original compressed size, for diagnostics
    uint32_t  transfer_ms;  // RF transfer time, for diagnostics
    uint32_t  enqueue_ms;   // time the frame was pushed, for the catch-up test
};
QueueHandle_t g_av_queue = nullptr;
TaskHandle_t g_audio_task = nullptr;
bool g_audio_playing_state = false;
#define APP_GW_AV_QUEUE_SIZE        20      /* ~ a few seconds of AV pairs */
#define APP_GW_AV_MAX_LATENCY_MS    600U    /* drop-oldest catch-up threshold */

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

void save_config_u8(const char *key, uint8_t val)
{
    nvs_handle_t nvs;
    if (nvs_open(kNvsNs, NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_u8(nvs, key, val);
    nvs_commit(nvs);
    nvs_close(nvs);
}

uint8_t load_config_u8(const char *key, uint8_t def)
{
    nvs_handle_t nvs;
    uint8_t val = def;
    if (nvs_open(kNvsNs, NVS_READONLY, &nvs) == ESP_OK) {
        (void)nvs_get_u8(nvs, key, &val);
        nvs_close(nvs);
    }
    return val;
}

bool on_image_capture_request(uint16_t session_id);
void stream_next_frame_cb(void *arg);

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
    (void)on_image_capture_request(sid);
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

// 15s one-shot timer that re-arms the PIR trigger after a detection.
// MUST be esp_timer, NOT a FreeRTOS xTimer: with CONFIG_PM_ENABLE off, the
// FreeRTOS tick is FROZEN during esp_light_sleep_start(), so a tick-based timer
// barely advances across the node's 500ms CAD light-sleep cycles and would take
// many minutes (or effectively never) to expire — which is exactly why the 2nd
// PIR never re-armed until a gateway wakeup kept the CPU awake long enough to
// accumulate ticks. esp_timer runs off the hardware timer, which keeps counting
// through light sleep, so 15s of wall-clock time expires correctly.
static esp_timer_handle_t g_pir_rearm_timer = nullptr;

// PIR uses GPIO_INTR_HIGH_LEVEL: the sensor holds GPIO12 HIGH for several
// seconds after motion, and level-mode is also the ONLY mode the ESP32-S3
// light-sleep GPIO wake supports. A level trigger left enabled would fire
// continuously while the pin is high — an interrupt storm that trips the
// interrupt watchdog and RESTARTS the node. So the ISR disables the trigger
// source the instant it fires, and a 15s one-shot timer re-arms it. That 15s
// re-arm also serves as the capture cooldown: continuous motion just keeps the
// pin high, and when the timer re-enables the level interrupt on a still-high
// pin it fires again immediately, disables again, and restarts the 15s timer.
static void IRAM_ATTR pir_isr_handler(void *arg)
{
    // Kill the level trigger NOW so it can't re-fire while GPIO12 stays high.
    gpio_intr_disable(APP_PIR_GPIO);
    static_cast<RadioPing *>(arg)->set_pir_armed(false);
    static_cast<RadioPing *>(arg)->pir_trigger();
    if (g_pir_rearm_timer != nullptr) {
        // esp_timer_start_once is ISR-safe. Guard against a double-start (the
        // timer could still be pending if a stray edge slipped in) by stopping
        // first is NOT ISR-safe, so we rely on the trigger being disabled above
        // to prevent re-entry until the timer fires and re-arms.
        esp_timer_start_once(g_pir_rearm_timer,
                             (uint64_t)APP_TRIGGER_COOLDOWN_SEC * 1000000ULL);
    }
}

// Re-arm the PIR level trigger 15s after the last detection.
static void pir_rearm_timer_cb(void *arg)
{
    gpio_intr_disable(APP_PIR_GPIO);
    gpio_set_intr_type(APP_PIR_GPIO, GPIO_INTR_HIGH_LEVEL);
    static_cast<RadioPing *>(arg)->set_pir_armed(true);
    gpio_intr_enable(APP_PIR_GPIO);
    ESP_LOGI(TAG, "PIR: GPIO%d high-level re-armed", APP_PIR_GPIO);
}

void pir_arm_timer_cb(void *arg)
{
    if (g_pir_rearm_timer == nullptr) {
        const esp_timer_create_args_t rearm_args = {
            .callback = pir_rearm_timer_cb,
            .arg = arg,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "pir_rearm",
            .skip_unhandled_events = true,
        };
        esp_timer_create(&rearm_args, &g_pir_rearm_timer);
    }
    gpio_intr_disable(APP_PIR_GPIO);
    gpio_isr_handler_add(APP_PIR_GPIO, pir_isr_handler, arg);
    gpio_set_intr_type(APP_PIR_GPIO, GPIO_INTR_HIGH_LEVEL);
    static_cast<RadioPing *>(arg)->set_pir_armed(true);
    gpio_intr_enable(APP_PIR_GPIO);
    ESP_LOGI(TAG, "PIR: GPIO%d high-level armed", APP_PIR_GPIO);
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
        save_config_u8("audio_clip", value ? 1 : 0);
        update_camera_timer_status();
        ESP_LOGI(TAG, "config: audio_clip=%s", g_audio_clip_enabled ? "on" : "off");
    } else if (key == APP_CFG_KEY_SOUND_TRIGGER) {
        g_radio.set_sound_trigger_level(value);
        save_config_u8("snd_trig", (uint8_t)value);
        ESP_LOGI(TAG, "config: sound_trigger=%lu", static_cast<unsigned long>(value));
    } else if (key == APP_CFG_KEY_PIR_TRIGGER) {
        g_radio.set_pir_enabled(value != 0);
        save_config_u8("pir", value ? 1 : 0);
        ESP_LOGI(TAG, "config: pir_trigger=%s", value ? "on" : "off");
    } else if (key == APP_CFG_KEY_VOICE_ALARM) {
        g_voice_alarm_enabled = (value != 0);
        save_config_u8("alarm", value ? 1 : 0);
        ESP_LOGI(TAG, "config: voice_alarm=%s", value ? "on" : "off");
    } else if (key == APP_CFG_KEY_LOW_POWER) {
        g_low_power_enabled = (value != 0);
        save_config_u8("lowpwr", value ? 1 : 0);
        ESP_LOGI(TAG, "config: low_power=%s", value ? "on" : "off");
        if (value != 0) {
            // Low power sleeps the CPU during CAD standby, so sound trigger and
            // audio clip can't work. Zero them (and persist) so they stay off
            // even after low power is turned off, matching the gateway UI.
            g_audio_clip_enabled = false;
            save_config_u8("audio_clip", 0);
            g_radio.set_sound_trigger_level(0);
            save_config_u8("snd_trig", 0);
            update_camera_timer_status();
            ESP_LOGI(TAG, "low power: sound trigger + audio clip disabled");
        }
    }
}

// Node low power: called by the radio when entering CAD sleep standby. Release
// the camera to save power; capture_frame() rebuilds it on the next capture.
void on_low_power_standby(bool entering)
{
    if (!entering) return;
    if (g_capture_busy) return;  // never release mid-capture
    g_camera_uart.low_power_standby();
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
    // Only gateway mode arms this timer now (K6 long-press >1.5s -> mode switch,
    // handled in on_button on release). Camera mode no longer binds K6/PTT, so
    // there is no camera voice activation here anymore.
    g_ptt_held_long = true;
}

void play_audio_clip(const uint8_t *opus_packed, size_t total_len);

// Image capture task: runs on device A (camera mode) when ImageCmd received
struct ImageCaptureCtx {
    uint16_t session_id;
};

// --- Frame prefetch (pipelining) --------------------------------------------
// The gateway drives the stream serially: it only requests frame N+1 after it
// has fully received frame N. On the node the per-frame work is
// capture+downsample+JPEG (~100ms, CPU) followed by the radio push (~140ms,
// SPI + airtime) — done back to back that is ~240ms/frame. But the two use
// independent hardware (LCD_CAM/DMA vs the radio SPI) and, since the TX wait now
// blocks (layer 2), the CPU is idle during most of the push. So while frame N is
// being transmitted we pre-capture+encode frame N+1 into this cache. When the
// next ImageCmd arrives we send the cached blob immediately, moving the ~100ms
// prep off the per-frame critical path. Video-only: an opus clip must be
// snapshotted at the capture instant, so prefetch is skipped when audio clips
// are enabled (default off) and that path stays fully serial.
static SemaphoreHandle_t g_prefetch_mutex = nullptr;
static uint8_t *g_prefetch_jpeg = nullptr;   // heap_caps blob, owned by cache
static size_t g_prefetch_jpeg_len = 0;
static uint32_t g_prefetch_ready_ms = 0;     // esp_log_timestamp() when produced
// A prefetched frame older than this is stale (stream stopped / manual capture
// gap): discard it and capture fresh so we never send a second-old image.
#define APP_PREFETCH_MAX_AGE_MS 800U

static void prefetch_cache_init()
{
    if (!g_prefetch_mutex) {
        g_prefetch_mutex = xSemaphoreCreateMutex();
    }
}

// Drop any cached prefetch blob. Caller must NOT hold g_prefetch_mutex.
static void prefetch_discard()
{
    if (!g_prefetch_mutex) return;
    xSemaphoreTake(g_prefetch_mutex, portMAX_DELAY);
    if (g_prefetch_jpeg) {
        heap_caps_free(g_prefetch_jpeg);
        g_prefetch_jpeg = nullptr;
        g_prefetch_jpeg_len = 0;
        g_prefetch_ready_ms = 0;
    }
    xSemaphoreGive(g_prefetch_mutex);
}

// Capture one frame and JPEG-encode it into a freshly heap_caps-allocated blob.
// Returns the blob (caller owns / frees) or nullptr on any failure. Assumes the
// camera is powered and the LCD is already released (true inside the capture
// task; the camera node has no LCD anyway). Video-only — no opus. Used by both
// the cold path (no prefetch available) and the prefetch producer.
static uint8_t *prepare_jpeg_blob(size_t *out_len)
{
    *out_len = 0;
    uint8_t *frame = nullptr;
    size_t len = 0;
    uint32_t width = 0, height = 0, pixfmt = 0;
    esp_err_t capture_e = g_camera_uart.capture_frame(&frame, &len, &width, &height, &pixfmt);
    if (capture_e != ESP_OK || !frame) {
        return nullptr;
    }
    uint8_t *jpeg = nullptr;
    size_t jpeg_len = 0;
    esp_err_t e = g_radio.image_xfer().encode_frame(frame, len, width, height, pixfmt,
                                                    &jpeg, &jpeg_len);
    heap_caps_free(frame);
    if (e != ESP_OK || !jpeg) {
        return nullptr;
    }
    *out_len = jpeg_len;
    return jpeg;
}

// Producer: called from the TX-wait loop while the radio is pushing the current
// frame. Captures+encodes the next frame and stashes it in the cache. Runs at
// most once per capture task (guarded by the caller). Skipped if a prefetch is
// already cached.
static void prefetch_produce()
{
    if (!g_prefetch_mutex) return;
    // Already have one cached? Don't stack.
    xSemaphoreTake(g_prefetch_mutex, portMAX_DELAY);
    bool have = (g_prefetch_jpeg != nullptr);
    xSemaphoreGive(g_prefetch_mutex);
    if (have) return;

    size_t jpeg_len = 0;
    uint8_t *jpeg = prepare_jpeg_blob(&jpeg_len);
    if (!jpeg) return;

    xSemaphoreTake(g_prefetch_mutex, portMAX_DELAY);
    if (g_prefetch_jpeg) {
        // Raced with another producer (shouldn't happen — single capture task) —
        // keep the existing one, drop ours.
        heap_caps_free(jpeg);
    } else {
        g_prefetch_jpeg = jpeg;
        g_prefetch_jpeg_len = jpeg_len;
        g_prefetch_ready_ms = static_cast<uint32_t>(esp_log_timestamp());
    }
    xSemaphoreGive(g_prefetch_mutex);
}

// Consumer: try to take a fresh cached prefetch blob. Returns the blob (caller
// owns / frees) or nullptr if none / stale. On success *out_len is set.
static uint8_t *prefetch_take(size_t *out_len)
{
    *out_len = 0;
    if (!g_prefetch_mutex) return nullptr;
    xSemaphoreTake(g_prefetch_mutex, portMAX_DELAY);
    uint8_t *blob = nullptr;
    if (g_prefetch_jpeg) {
        uint32_t age = static_cast<uint32_t>(esp_log_timestamp()) - g_prefetch_ready_ms;
        if (age <= APP_PREFETCH_MAX_AGE_MS) {
            blob = g_prefetch_jpeg;
            *out_len = g_prefetch_jpeg_len;
        } else {
            heap_caps_free(g_prefetch_jpeg);  // stale, drop
        }
        g_prefetch_jpeg = nullptr;
        g_prefetch_jpeg_len = 0;
        g_prefetch_ready_ms = 0;
    }
    xSemaphoreGive(g_prefetch_mutex);
    return blob;
}

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
    // Per-frame capture step timings are DEBUG (they fire mid-capture, on the hot
    // path); only the post-send "[TIMING] total" summary stays at INFO.
    ESP_LOGD(TAG, "[TIMING] cmd received t=0ms");

    bsp_lcd_set_camera_status("Remote capture...");

#if APP_AUDIO_FEATURES_ENABLE
    g_radio.pause_audio_capture();
    vTaskDelay(pdMS_TO_TICKS(APP_AUDIO_FRAME_MS + 5));
#endif

    // Opus audio is drained fresh from the ring right before the blob is built
    // (see below), NOT snapshotted here. The ring keeps filling during this whole
    // capture+TX (tx_task no longer pauses on image_tx_active_), so draining at
    // send time yields a continuous, gap-free stream across frames.
    uint8_t *opus_buf = nullptr;
    size_t opus_len = 0;

#if APP_CAMERA_NODE_LCD_ENABLE && APP_AUDIO_FEATURES_ENABLE
    esp_err_t audio_e = bsp_audio_suspend();
    if (audio_e != ESP_OK) {
        ESP_LOGW(TAG, "audio suspend for image capture: %s", esp_err_to_name(audio_e));
    }
#endif

    esp_err_t e = ESP_OK;
    (void)e;
#if APP_CAMERA_NODE_LCD_ENABLE
    e = bsp_lcd_release_for_camera();
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
#endif

    // JPEG blob for this frame. Fast path: a prefetched frame produced during
    // the previous frame's TX is ready — use it and skip capture+encode entirely,
    // moving that ~100ms off the per-frame critical path. Cold path (first frame
    // or stale prefetch): capture + encode now. Prefetch is no longer gated on
    // audio: opus is drained separately at send time, so JPEG being slightly old
    // no longer breaks audio continuity.
    uint8_t *jpeg = nullptr;
    size_t jpeg_len = 0;
    uint32_t t_jpeg_done;
    jpeg = prefetch_take(&jpeg_len);
    if (jpeg) {
        t_jpeg_done = static_cast<uint32_t>(esp_log_timestamp());
        ESP_LOGD(TAG, "[TIMING] prefetch hit +%lums (jpeg=%u bytes)",
                 static_cast<unsigned long>(t_jpeg_done - t_cmd),
                 static_cast<unsigned>(jpeg_len));
    } else {
        uint32_t t_cam_start = static_cast<uint32_t>(esp_log_timestamp());
        ESP_LOGD(TAG, "[TIMING] camera init start +%lums", static_cast<unsigned long>(t_cam_start - t_cmd));

        esp_err_t capture_e = g_camera_uart.capture_frame(&frame, &len, &width, &height, &pixfmt);

        uint32_t t_cam_done = static_cast<uint32_t>(esp_log_timestamp());
        ESP_LOGD(TAG, "[TIMING] capture done +%lums (camera=%lums) %lux%lu %u bytes",
                 static_cast<unsigned long>(t_cam_done - t_cmd),
                 static_cast<unsigned long>(t_cam_done - t_cam_start),
                 static_cast<unsigned long>(width),
                 static_cast<unsigned long>(height),
                 static_cast<unsigned>(len));

        // LCD reinit and audio resume deferred until after transmission
        if (capture_e != ESP_OK || !frame) {
#if APP_CAMERA_NODE_LCD_ENABLE
            bsp_lcd_reinit_after_camera();
#endif
#if APP_CAMERA_NODE_LCD_ENABLE && APP_AUDIO_FEATURES_ENABLE
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
        uint32_t t_jpeg_start = static_cast<uint32_t>(esp_log_timestamp());
        e = g_radio.image_xfer().encode_frame(frame, len, width, height, pixfmt, &jpeg, &jpeg_len);
        heap_caps_free(frame);
        t_jpeg_done = static_cast<uint32_t>(esp_log_timestamp());
        ESP_LOGD(TAG, "[TIMING] JPEG done +%lums (jpeg=%lums) %u bytes",
                 static_cast<unsigned long>(t_jpeg_done - t_cmd),
                 static_cast<unsigned long>(t_jpeg_done - t_jpeg_start),
                 static_cast<unsigned>(jpeg_len));

        if (e != ESP_OK || !jpeg) {
#if APP_CAMERA_NODE_LCD_ENABLE
            bsp_lcd_reinit_after_camera();
#endif
#if APP_CAMERA_NODE_LCD_ENABLE && APP_AUDIO_FEATURES_ENABLE
            bsp_audio_resume();
#endif
            bsp_lcd_set_camera_status("JPEG encode failed");
            heap_caps_free(opus_buf);
            g_radio.resume_audio_capture();
            g_capture_busy = false;
            vTaskDelete(nullptr);
            return;
        }
    }

    // --- Drain fresh Opus audio accumulated since the previous send ---
    // Done here (just before the blob is built) so the segment is as fresh as
    // possible. drain_opus() advances the ring's read cursor, so successive
    // frames carry a continuous, non-overlapping audio timeline. Capped at
    // APP_STREAM_OPUS_MAX_BYTES so a slow retransmit round can't bloat the blob.
    opus_buf = static_cast<uint8_t *>(
        heap_caps_malloc(APP_STREAM_OPUS_MAX_BYTES, MALLOC_CAP_SPIRAM));
    if (opus_buf) {
        opus_len = g_radio.drain_opus(opus_buf, APP_STREAM_OPUS_MAX_BYTES);
        if (opus_len == 0) {
            heap_caps_free(opus_buf);
            opus_buf = nullptr;
        }
    }

    // --- Build blob: [4-byte jpeg_len][jpeg][opus] and send once ---
    bool has_opus = (opus_buf && opus_len > 0);
    uint8_t *blob;
    size_t blob_len;

    if (has_opus) {
        blob_len = 4 + jpeg_len + opus_len;
        blob = static_cast<uint8_t *>(
            heap_caps_malloc(blob_len, MALLOC_CAP_SPIRAM));
        if (!blob) {
            ESP_LOGE(TAG, "blob alloc failed: %u bytes", static_cast<unsigned>(blob_len));
            heap_caps_free(jpeg);
            heap_caps_free(opus_buf);
#if APP_CAMERA_NODE_LCD_ENABLE
            bsp_lcd_reinit_after_camera();
#endif
#if APP_CAMERA_NODE_LCD_ENABLE && APP_AUDIO_FEATURES_ENABLE
            bsp_audio_resume();
#endif
            bsp_lcd_set_camera_status("Alloc failed");
            g_radio.resume_audio_capture();
            g_capture_busy = false;
            vTaskDelete(nullptr);
            return;
        }
        blob[0] = static_cast<uint8_t>(jpeg_len);
        blob[1] = static_cast<uint8_t>(jpeg_len >> 8);
        blob[2] = static_cast<uint8_t>(jpeg_len >> 16);
        blob[3] = static_cast<uint8_t>(jpeg_len >> 24);
        memcpy(&blob[4], jpeg, jpeg_len);
        memcpy(&blob[4 + jpeg_len], opus_buf, opus_len);
    } else {
        blob_len = jpeg_len;
        blob = jpeg;
        jpeg = nullptr;
    }
    heap_caps_free(jpeg);
    heap_caps_free(opus_buf);

    uint16_t total_frags = static_cast<uint16_t>(
        (blob_len + APP_IMAGE_FRAGMENT_DATA_SIZE - 1) / APP_IMAGE_FRAGMENT_DATA_SIZE);
    uint16_t tx_session = has_opus ? (session_id | APP_AUDIO_SESSION_FLAG) : session_id;
    ESP_LOGD(TAG, "sending blob: %u pkts, %u bytes (jpeg=%u opus=%u)",
             total_frags, static_cast<unsigned>(blob_len),
             static_cast<unsigned>(jpeg_len), static_cast<unsigned>(opus_len));

    g_radio.send_image(blob, blob_len, tx_session);

    // Pipeline: the radio is now pushing this frame on the img_tx task (~140ms,
    // SPI + airtime) with the CPU otherwise idle (layer-2 blocking TX wait). Use
    // that window to pre-capture+encode the NEXT frame into the prefetch cache so
    // the next ImageCmd can send it immediately. Camera (LCD_CAM/DMA) and radio
    // (SPI) are independent buses, and img_tx runs one priority above this task,
    // so a TX_DONE wakeup preempts the encode and the burst never stalls. Always
    // runs now: opus is drained separately at send time (not tied to the capture
    // instant), so prefetch no longer needs to be gated on audio.
    // A prefetch that no next-frame request consumes is dropped by the staleness
    // guard in prefetch_take, so a single/auto capture only costs one idle encode.
    prefetch_produce();

    // Ownership of `blob` has been handed to send_image / image_tx_task, which
    // frees it when the transfer finishes or aborts. Do NOT free it here: the
    // tx task may still be reading it (its own handshake budget is up to 30s,
    // which used to race this wait and cause a use-after-free). We wait only to
    // sequence the post-tx cleanup after TX, not to govern the buffer lifetime.
    // Poll at a fine granularity: this wait sits in the per-frame critical path,
    // so a coarse poll adds dead time between the real TX finishing and this task
    // clearing g_capture_busy (which also widens the window where the next
    // ImageCmd lands while still "busy"). 5ms keeps the [TIMING] tx= number
    // honest and lets the next frame start almost immediately after TX.
    bool tx_done = false;
    uint32_t wait_start = xTaskGetTickCount();
    while (xTaskGetTickCount() - wait_start < pdMS_TO_TICKS(30000)) {
        if (!g_radio.image_tx_busy()) {
            tx_done = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    uint32_t t_tx_done = static_cast<uint32_t>(esp_log_timestamp());
    ESP_LOGI(TAG, "[TIMING] total +%lums | img_prep=%lu tx=%lums",
             static_cast<unsigned long>(t_tx_done - t_cmd),
             static_cast<unsigned long>(t_jpeg_done - t_cmd),
             static_cast<unsigned long>(t_tx_done - t_jpeg_done));

    if (!tx_done) {
        // TX outran our 30s wait and is still running. Do NOT run the post-tx
        // block: resume_audio_capture() clears image_tx_active_ (which the tx
        // task still depends on) and the voice alarm plays a clip — both would
        // race the still-active tx task on the radio from this thread. The tx
        // task cleans up after itself when it finishes (clears image_tx_active_,
        // ends the wake window, resumes audio capture). The camera node has no
        // LCD, so there is nothing else to restore here. Just release the
        // capture guard and exit.
        ESP_LOGW(TAG, "image tx still active after 30s; skipping post-tx cleanup (tx task finishes on its own)");
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }

    // Reinit LCD now that transmission is done
#if APP_CAMERA_NODE_LCD_ENABLE
    esp_err_t lcd_e = bsp_lcd_reinit_after_camera();
    if (lcd_e != ESP_OK) {
        ESP_LOGE(TAG, "lcd reinit after tx: %s", esp_err_to_name(lcd_e));
    }
#endif
#if APP_CAMERA_NODE_LCD_ENABLE && APP_AUDIO_FEATURES_ENABLE
    bsp_audio_resume();
#endif
    char status[48];
    snprintf(status, sizeof(status), "Done %u pkts", total_frags);
    bsp_lcd_set_camera_status(status);

    g_radio.resume_audio_capture();

#if APP_VOICE_ALARM_ENABLE
    if (g_voice_alarm_enabled && session_id >= 0xC000) {
        ESP_LOGI(TAG, "trigger capture done, playing voice alarm");
        // Low power: image_tx_task already cleared the PIR keep-awake guard, so
        // without this the radio task would drop into CAD light sleep and halt
        // both cores mid-clip. Hold an untimed keep-awake for exactly the clip,
        // then release so the node re-sleeps on the next idle pass.
        g_radio.audio_playback_begin();
        uint8_t prev_vol = APP_VOICE_ALARM_VOLUME_PERCENT;
        bsp_audio_set_volume(prev_vol);
        play_audio_clip(warning_voice_opus, warning_voice_opus_len);
        bsp_audio_set_volume(100);
        g_radio.audio_playback_end();
    }
#endif

    g_capture_busy = false;
    vTaskDelete(nullptr);
}

// Callback: device A receives ImageCmd from B
// Returns true if the request was accepted (or is a same-session retransmit we
// already dispatched — re-ack it), false if dropped (not camera / busy /
// cooldown / task-create failed). The radio layer only sends ImageCmdAck when
// this returns true, so a dropped request leaves the gateway flooding until the
// node is free — see image_capture_cb_t in radio_ping.hpp.
bool on_image_capture_request(uint16_t session_id)
{
    if (g_app_mode != AppMode::camera) {
        ESP_LOGW(TAG, "ImageCmd received but not in camera mode");
        g_radio.notify_capture_dropped();
        return false;
    }
    if (g_capture_busy) {
        ESP_LOGW(TAG, "ImageCmd ignored: capture already busy");
        g_radio.notify_capture_dropped();
        return false;
    }
    // Dedup gateway ImageCmd retransmits of the same session. The gateway resends
    // ImageCmd until it gets an ack; if an ack is lost for the whole capture, a
    // late retransmit can arrive after g_capture_busy cleared. handle_image_cmd
    // still re-acks it (good), but we must not launch a second capture for a
    // session we already handled. New requests always carry a new session_id.
    static uint16_t s_last_dispatched_session = 0;
    static bool s_have_dispatched = false;
    if (s_have_dispatched && session_id == s_last_dispatched_session) {
        // Retransmit of a session we already handled: re-ack it (return true) so
        // the gateway's lost-ack self-heal still works, but do not launch a
        // second capture.
        ESP_LOGW(TAG, "ImageCmd ignored: session %u already dispatched", session_id);
        return true;
    }
    if (g_audio_clip_enabled) {
        static uint32_t s_last_node_capture_ms = 0;
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (s_last_node_capture_ms != 0 &&
            (now - s_last_node_capture_ms) < APP_AUDIO_CAPTURE_COOLDOWN_MS) {
            ESP_LOGW(TAG, "ImageCmd ignored: audio cooldown");
            g_radio.notify_capture_dropped();
            return false;
        }
        s_last_node_capture_ms = now;
    }
    g_capture_busy = true;
    s_last_dispatched_session = session_id;
    s_have_dispatched = true;

    // Low power: hold the node awake through the capture + push. Without this,
    // poll_once re-enters CAD light sleep (500ms halt) immediately after the
    // timer fires, starving the camera/JPEG task so capture never finishes. The
    // keep-awake guard is cleared on TX completion or 8s timeout, whichever first.
    g_radio.notify_capture_starting();

    auto *ctx = new (std::nothrow) ImageCaptureCtx{ session_id };
    if (!ctx) {
        g_capture_busy = false;
        return false;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(image_capture_task, "img_cap",
                                            APP_IMAGE_TASK_STACK_BYTES, ctx,
                                            APP_IMAGE_TASK_PRIORITY, nullptr,
                                            APP_IMAGE_TASK_CORE);
    if (ok != pdPASS) {
        delete ctx;
        g_capture_busy = false;
        ESP_LOGE(TAG, "image capture task create failed");
        return false;
    }
    return true;
}

// Free both buffers an AVFrame owns. Safe on null fields.
static void av_frame_free(AVFrame *f)
{
    if (!f) return;
    if (f->rgb565)      heap_caps_free(f->rgb565);
    if (f->opus_packed) heap_caps_free(f->opus_packed);
    f->rgb565 = nullptr;
    f->opus_packed = nullptr;
}

// Push one paired image+audio frame into the AV sync queue. Non-blocking so the
// rx-complete path / pull loop is never stalled. On a full queue the OLDEST
// pair is dropped (and freed) to make room, keeping playback close to real time
// rather than backing up unbounded latency. Takes ownership of both buffers.
static void av_queue_push(AVFrame *frame)
{
    if (!g_av_queue || !frame) { av_frame_free(frame); return; }

    frame->enqueue_ms = static_cast<uint32_t>(esp_log_timestamp());

    if (xQueueSend(g_av_queue, frame, 0) != pdTRUE) {
        AVFrame old;
        if (xQueueReceive(g_av_queue, &old, 0) == pdTRUE) {
            av_frame_free(&old);
        }
        if (xQueueSend(g_av_queue, frame, 0) != pdTRUE) {
            av_frame_free(frame);   // still no room; drop ours
        }
    }
}

// --- Howling (acoustic feedback) suppression --------------------------------
// The gateway speaker sitting close to the node mic can form an acoustic
// feedback loop (howling). Field data showed howling has two clean signatures
// in the decoded playback PCM that normal audio never hits: heavy sample
// saturation (clip%) and very high energy (rms). Autocorrelation / ZCR variance
// were unreliable in practice, so detection uses only clip% + rms.
//
// State machine (per 10ms frame, 160 samples @ 16kHz):
//   ACTIVE  -> a frame with clip >= ENTER_CLIP AND rms >= ENTER_RMS held for
//              ENTER_FRAMES starts muting.
//   MUTED   -> output is zeroed; when clip <= EXIT_CLIP AND rms <= EXIT_RMS for
//              EXIT_FRAMES (one full echo round-trip) playback resumes.
// Thresholds are intentionally loose: only unmistakable howling is muted, so
// loud but legitimate audio is never cut. Muting the gateway speaker breaks the
// loop, so the mic stops re-capturing the tone and levels fall back to normal.
#define HOWL_CLIP_SAMPLE    29490   /* 0.9 * full scale: a "clipped" sample     */
#define HOWL_ENTER_CLIP     5       /* % clipped samples to call it howling     */
#define HOWL_ENTER_RMS      6000    /* energy floor to call it howling          */
#define HOWL_ENTER_FRAMES   3       /* ~30ms sustained before muting            */
#define HOWL_EXIT_CLIP      2       /* % clipped to consider the loop broken    */
#define HOWL_EXIT_RMS       4500    /* energy ceiling to consider it gone       */
#define HOWL_EXIT_FRAMES    50      /* ~500ms clear (> one echo round-trip)     */

struct HowlSuppressor {
    bool     muted;
    uint16_t enter_run;   // consecutive frames meeting the enter condition
    uint16_t exit_run;    // consecutive frames meeting the exit condition
};

// Analyze one decoded frame and update mute state. Returns true if this frame
// should be silenced (zeroed) before playback.
static bool howl_process(HowlSuppressor *h, const int16_t *pcm, int n)
{
    if (n <= 0) return h->muted;

    int64_t sum_sq = 0;
    int      clipped = 0;
    for (int i = 0; i < n; i++) {
        int32_t s = pcm[i];
        sum_sq += (int64_t)s * s;
        int32_t a = s < 0 ? -s : s;
        if (a > HOWL_CLIP_SAMPLE) clipped++;
    }
    uint32_t rms = (uint32_t)sqrtf((float)(sum_sq / n));
    uint32_t clip_pct = (uint32_t)(clipped * 100 / n);

    if (!h->muted) {
        bool howl = (clip_pct >= HOWL_ENTER_CLIP && rms >= HOWL_ENTER_RMS);
        h->enter_run = howl ? (uint16_t)(h->enter_run + 1) : 0;
        if (h->enter_run >= HOWL_ENTER_FRAMES) {
            h->muted = true;
            h->exit_run = 0;
            ESP_LOGW(TAG, "howling detected (clip=%lu%% rms=%lu), muting",
                     (unsigned long)clip_pct, (unsigned long)rms);
        }
    } else {
        bool clear = (clip_pct <= HOWL_EXIT_CLIP && rms <= HOWL_EXIT_RMS);
        h->exit_run = clear ? (uint16_t)(h->exit_run + 1) : 0;
        if (h->exit_run >= HOWL_EXIT_FRAMES) {
            h->muted = false;
            h->enter_run = 0;
            ESP_LOGI(TAG, "howling cleared, resuming playback");
        }
    }

    return h->muted;
}

// Gateway AV playback task. Dequeues image+audio pairs, shows the image, then
// decodes/writes the opus segment to I2S. No prebuffer: it plays as soon as a
// frame exists (lowest latency). Catch-up: if the frame pulled off the queue
// has already waited > APP_GW_AV_MAX_LATENCY_MS AND a newer frame is queued
// behind it, the stale frame (image + audio together) is dropped so latency
// cannot keep growing when frames arrive faster than they drain.
void gateway_audio_task(void *arg)
{
    (void)arg;

    OpusCodec codec;
    if (codec.init_decoder_only() != ESP_OK) {
        ESP_LOGE(TAG, "gateway_audio_task: codec init failed");
        vTaskDelete(nullptr);
        return;
    }
    // PA is NOT left on continuously: a class-D amp switching next to the LR2021
    // front-end raises the packet error rate. It is enabled just before the
    // first write and disabled when the queue runs dry, so the radio sees a
    // quiet front-end whenever no sound is being produced.
    ESP_LOGI(TAG, "gateway audio task started (PA gated on playback)");

    int16_t pcm[APP_AUDIO_FRAME_SAMPLES];
    int16_t stereo[APP_AUDIO_FRAME_SAMPLES * 2];
    HowlSuppressor howl = {};   // acoustic-feedback (howling) mute state

    for (;;) {
        // Block until a frame is available. No prebuffer -> play immediately.
        AVFrame frame;
        if (xQueueReceive(g_av_queue, &frame, pdMS_TO_TICKS(300)) != pdTRUE) {
            if (g_audio_playing_state) {
                g_audio_playing_state = false;   // queue dry -> silence the amp
                bsp_audio_pa_enable(false);
            }
            continue;
        }

        // Catch-up: drop this frame if it has been waiting too long AND a newer
        // frame is already queued. Image + audio are dropped together so they
        // stay in sync; we only ever advance to a frame that is fresh enough or
        // is the last one in the queue (never leaving nothing to play).
        while ((static_cast<uint32_t>(esp_log_timestamp()) - frame.enqueue_ms)
                   > APP_GW_AV_MAX_LATENCY_MS
               && uxQueueMessagesWaiting(g_av_queue) > 0) {
            av_frame_free(&frame);
            xQueueReceive(g_av_queue, &frame, 0);
        }

        if (!g_audio_playing_state) {
            g_audio_playing_state = true;
            bsp_audio_pa_enable(true);   // enable just before the first write
            ESP_LOGI(TAG, "audio playback started, PA on");
        }

        // AV sync: display THIS frame's image right before playing its audio.
        // Because we only reach here at the audio drain rate (I2S clock paced),
        // the image cadence tracks the audio timeline instead of racing ahead
        // at RF-arrival speed. The rotate+invalidate is a few ms; the LVGL flush
        // then overlaps the ~220ms audio playback below on the LVGL task.
        if (frame.rgb565) {
            ui_gw_rx_complete(frame.rgb565, frame.width, frame.height,
                              frame.jpeg_size, frame.transfer_ms);
            heap_caps_free(frame.rgb565);
            frame.rgb565 = nullptr;
        }

        size_t pos = 0;
        while (frame.opus_packed && pos < frame.opus_len) {
            uint8_t flen = frame.opus_packed[pos++];
            if (flen == 0 || pos + flen > frame.opus_len) break;
            int n = codec.decode(&frame.opus_packed[pos], flen, pcm, APP_AUDIO_FRAME_SAMPLES);
            pos += flen;
            if (n <= 0) continue;
            // Break the acoustic feedback loop: zero this frame while howling.
            bool mute = howl_process(&howl, pcm, n);
            for (int i = 0; i < n; i++) {
                int16_t s = mute ? 0 : pcm[i];
                stereo[2 * i] = s;
                stereo[2 * i + 1] = s;
            }
            size_t written = 0;
            bsp_audio_write(stereo, n * 2 * sizeof(int16_t), &written);
        }
        av_frame_free(&frame);
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
    if (!xfer || !xfer->rx_complete()) return;

    // Per-frame consumption timing (gateway side). The node cannot push faster
    // than the gateway can decode + rotate + flush to the LCD + persist; this
    // breakdown measures that ceiling so the stream cadence (and any future node
    // self-push flow control) can be sized against it. period = wall-clock gap
    // between consecutive completed frames = the real end-to-end frame rate.
    uint32_t t_rx = static_cast<uint32_t>(esp_log_timestamp());
    uint32_t t_decode_ms = 0, t_display_ms = 0, t_save_ms = 0;
    static uint32_t s_last_frame_ms = 0;

    // AV sync staging: when a frame carries audio, the decoded image is copied
    // here (SPIRAM) and later paired with its opus segment into g_av_queue so
    // gateway_audio_task displays it in lockstep with playback. Null when the
    // image was already shown inline (no audio, or SPIRAM copy failed).
    uint16_t *av_pending_rgb565 = nullptr;
    uint32_t  av_pending_w = 0, av_pending_h = 0;

    if (g_audio_clip_enabled) {
        s_last_gw_capture_ms = (uint32_t)(esp_timer_get_time() / 1000);
    }

    uint16_t sid = xfer->rx_session_id();
    bool has_audio = (sid & APP_AUDIO_SESSION_FLAG) != 0;

    // Reassemble the received data
    uint8_t *raw = nullptr;
    size_t raw_len = 0;
    esp_err_t e = xfer->rx_reassemble(&raw, &raw_len);
    if (e != ESP_OK || !raw) {
        ESP_LOGE(TAG, "rx_reassemble failed: %d", e);
        xfer->rx_reset();
        return;
    }

    const uint8_t *jpeg_data = raw;
    size_t jpeg_len = raw_len;
    const uint8_t *opus_data = nullptr;
    size_t opus_len = 0;

    if (has_audio && raw_len > 4) {
        uint32_t jlen = static_cast<uint32_t>(raw[0])
                      | (static_cast<uint32_t>(raw[1]) << 8)
                      | (static_cast<uint32_t>(raw[2]) << 16)
                      | (static_cast<uint32_t>(raw[3]) << 24);
        if (jlen <= raw_len - 4) {
            jpeg_data = &raw[4];
            jpeg_len = jlen;
            opus_data = &raw[4 + jlen];
            opus_len = raw_len - 4 - jlen;
        }
    }
    // Decode and display JPEG
    jpeg_dec_config_t dec_cfg = DEFAULT_JPEG_DEC_CONFIG();
    dec_cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_LE;

    jpeg_dec_handle_t decoder = nullptr;
    jpeg_error_t jerr = jpeg_dec_open(&dec_cfg, &decoder);
    if (jerr == JPEG_ERR_OK && decoder) {
        jpeg_dec_io_t io = {};
        io.inbuf = const_cast<unsigned char *>(jpeg_data);
        io.inbuf_len = static_cast<int>(jpeg_len);

        jpeg_dec_header_info_t header = {};
        jerr = jpeg_dec_parse_header(decoder, &io, &header);
        if (jerr == JPEG_ERR_OK) {
            int outbuf_len = header.width * header.height * 2;
            uint8_t *rgb565 = static_cast<uint8_t *>(jpeg_calloc_align(outbuf_len, 16));
            if (rgb565) {
                io.outbuf = rgb565;
                jerr = jpeg_dec_process(decoder, &io);
                t_decode_ms = static_cast<uint32_t>(esp_log_timestamp()) - t_rx;
                if (jerr == JPEG_ERR_OK) {
                    uint32_t w = header.width;
                    uint32_t h = header.height;
                    uint32_t t_disp0 = static_cast<uint32_t>(esp_log_timestamp());
                    if (g_app_mode == AppMode::radio) {
                        if (has_audio && opus_len > 0) {
                            // AV sync path: hand the image to gateway_audio_task
                            // paired with its audio, so display is paced by the
                            // I2S clock instead of racing ahead at RF speed. Copy
                            // rgb565 into SPIRAM (decode buffer is freed here).
                            av_pending_rgb565 = static_cast<uint16_t *>(
                                heap_caps_malloc((size_t)w * h * 2, MALLOC_CAP_SPIRAM));
                            if (av_pending_rgb565) {
                                memcpy(av_pending_rgb565, rgb565, (size_t)w * h * 2);
                                av_pending_w = w;
                                av_pending_h = h;
                            } else {
                                // SPIRAM full: fall back to immediate display so
                                // the frame is not lost (audio will be ahead).
                                ui_gw_rx_complete(reinterpret_cast<const uint16_t *>(rgb565),
                                                  w, h, static_cast<uint32_t>(raw_len),
                                                  g_radio.last_transfer_ms());
                            }
                        } else {
                            // No audio: display immediately for lowest latency.
                            // Synchronous cost only (rotate + mark dirty); the LCD
                            // flush happens async on the LVGL task (core 1).
                            ui_gw_rx_complete(reinterpret_cast<const uint16_t *>(rgb565), w, h,
                                              static_cast<uint32_t>(raw_len), g_radio.last_transfer_ms());
                        }
                    } else {
                        bsp_lcd_show_rgb565_photo(reinterpret_cast<const uint16_t *>(rgb565), w, h);
                        bsp_lcd_set_camera_status("Photo received");
                    }
                    t_display_ms = static_cast<uint32_t>(esp_log_timestamp()) - t_disp0;
                }
                jpeg_free_align(rgb565);
            }
        }
        jpeg_dec_close(decoder);
    }

    // Save to image store for HTTP gallery
    uint32_t t_save0 = static_cast<uint32_t>(esp_log_timestamp());
    image_store_save(jpeg_data, jpeg_len, opus_data, opus_len, sid);
    t_save_ms = static_cast<uint32_t>(esp_log_timestamp()) - t_save0;

    // Enqueue the AV pair for synced playback. Non-blocking: the old blocking
    // play_audio_clip() ran the whole segment inline on the rx path and stalled
    // the pull loop, capping frame rate. gateway_audio_task now displays the
    // image + decodes/plays audio off this thread, paced by the I2S clock so
    // video stays in sync with audio. If the image was already shown inline
    // (SPIRAM copy failed) av_pending_rgb565 is null and only audio is queued.
    if (av_pending_rgb565 || (opus_data && opus_len > 0)) {
        AVFrame frame = {};
        frame.rgb565      = av_pending_rgb565;
        frame.width       = av_pending_w;
        frame.height      = av_pending_h;
        frame.jpeg_size   = static_cast<uint32_t>(raw_len);
        frame.transfer_ms = g_radio.last_transfer_ms();
        if (opus_data && opus_len > 0) {
            frame.opus_packed = static_cast<uint8_t *>(
                heap_caps_malloc(opus_len, MALLOC_CAP_SPIRAM));
            if (frame.opus_packed) {
                memcpy(frame.opus_packed, opus_data, opus_len);
                frame.opus_len = opus_len;
            }
        }
        av_queue_push(&frame);   // takes ownership of both buffers
    }

    heap_caps_free(raw);
    xfer->rx_reset();

    // One line per frame: radio transfer (airtime + handshake, from the node's
    // ImageStart to last fragment) vs gateway consume (decode + display + save),
    // and the wall-clock period between frames = the real achieved fps.
    uint32_t transfer_ms = g_radio.last_transfer_ms();
    uint32_t now_ms = static_cast<uint32_t>(esp_log_timestamp());
    uint32_t period_ms = (s_last_frame_ms != 0) ? (now_ms - s_last_frame_ms) : 0;
    s_last_frame_ms = now_ms;
    uint32_t consume_ms = t_decode_ms + t_display_ms + t_save_ms;
    if (period_ms > 0) {
        ESP_LOGI(TAG, "[FRAME] period=%lums (%lu.%lu fps) | transfer=%lums consume=%lums "
                      "(decode=%lu display=%lu save=%lu)",
                 (unsigned long)period_ms,
                 (unsigned long)(1000 / period_ms), (unsigned long)((10000 / period_ms) % 10),
                 (unsigned long)transfer_ms, (unsigned long)consume_ms,
                 (unsigned long)t_decode_ms, (unsigned long)t_display_ms, (unsigned long)t_save_ms);
    }

    // Continuous video stream: if the gateway UI is still streaming, auto-request
    // the next frame after a short delay. Scheduled on a one-shot timer so the
    // new request runs off the radio task instead of re-entering it here.
    if (g_app_mode == AppMode::radio && ui_gw_stream_active()) {
        if (!g_stream_next_timer) {
            const esp_timer_create_args_t args = {
                .callback = stream_next_frame_cb,
                .arg = nullptr,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "stream_next",
                .skip_unhandled_events = true,
            };
            esp_timer_create(&args, &g_stream_next_timer);
        }
        esp_timer_stop(g_stream_next_timer);
        esp_timer_start_once(g_stream_next_timer,
                             (uint64_t)APP_STREAM_NEXT_FRAME_DELAY_MS * 1000);
    }
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

// Radio mode (gateway): a node reported its battery voltage -> push to UI.
void on_vbat_received(uint16_t vbat_mv)
{
    if (g_app_mode == AppMode::radio) {
        ui_gw_update_vbat(vbat_mv);
    }
}

void on_image_rx_eot_nack(uint16_t missing_count, bool is_first_eot)
{
    if (g_app_mode == AppMode::radio) {
        ui_gw_rx_eot_nack(missing_count, is_first_eot);
    }
}

// Gateway UI capture callback — triggers remote photo via radio

void cooldown_retry_cb(void *arg)
{
    ESP_LOGI(TAG, "Cooldown expired, auto-triggering capture");
    s_last_gw_capture_ms = (uint32_t)(esp_timer_get_time() / 1000);
    image_store_abort_transfer();
    g_radio.trigger_image_capture();
}

// Continuous video stream: request the next frame. Runs in the esp_timer task,
// scheduled by on_image_rx_complete. Re-checks ui_gw_stream_active() so a stream
// the user just stopped (left the image page) does not fire one extra request.
void stream_next_frame_cb(void *arg)
{
    (void)arg;
    if (g_app_mode != AppMode::radio) return;
    if (!ui_gw_stream_active()) return;
    ESP_LOGI(TAG, "stream: request next frame");
    image_store_abort_transfer();
    g_radio.trigger_image_capture();
}

bool on_gw_capture(void)
{
    if (g_audio_clip_enabled) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (s_last_gw_capture_ms != 0 &&
            (now - s_last_gw_capture_ms) < APP_AUDIO_CAPTURE_COOLDOWN_MS) {
            uint32_t remaining_ms = APP_AUDIO_CAPTURE_COOLDOWN_MS - (now - s_last_gw_capture_ms);
            ESP_LOGW(TAG, "UI capture: audio cooldown, auto-retry in %lums",
                     (unsigned long)remaining_ms);
            if (!g_cooldown_retry_timer) {
                const esp_timer_create_args_t args = {
                    .callback = cooldown_retry_cb,
                    .arg = nullptr,
                    .dispatch_method = ESP_TIMER_TASK,
                    .name = "cooldown_retry",
                };
                esp_timer_create(&args, &g_cooldown_retry_timer);
            }
            esp_timer_stop(g_cooldown_retry_timer);
            esp_timer_start_once(g_cooldown_retry_timer, (uint64_t)remaining_ms * 1000);
            return false;
        }
        s_last_gw_capture_ms = now;
    }
    if (g_cooldown_retry_timer) {
        esp_timer_stop(g_cooldown_retry_timer);
    }
    ESP_LOGI(TAG, "UI capture: trigger remote photo");
    image_store_abort_transfer();
    g_radio.trigger_image_capture();
    return true;
}

// Gateway UI: user left the transfer page — abort the in-progress RX so the
// gateway stops requesting/receiving this image (the node's TX self-aborts once
// its ACKs stop). Also drop any partial store-side transfer.
void on_gw_rx_abort(void)
{
    ESP_LOGI(TAG, "UI: left transfer page, aborting image RX");
    g_radio.abort_image_rx();
    image_store_abort_transfer();
}

// Gateway UI interval change callback — sends config to camera node
bool on_gw_interval_change(uint32_t interval_sec)
{
    ESP_LOGI(TAG, "UI interval change: %lus", static_cast<unsigned long>(interval_sec));
    return g_radio.send_config(APP_CFG_KEY_INTERVAL, interval_sec);
}

bool on_gw_audio_clip_change(uint32_t enable)
{
    ESP_LOGI(TAG, "UI audio clip: %s", enable ? "on" : "off");
    // Commit local state only on success so the UI switch stays the single
    // source of truth (mismatch would desync the capture cooldown logic).
    bool ok = g_radio.send_config(APP_CFG_KEY_AUDIO_CLIP, enable);
    if (ok) {
        g_audio_clip_enabled = (enable != 0);
    }
    return ok;
}

bool on_gw_sound_trigger_change(uint32_t level)
{
    ESP_LOGI(TAG, "UI sound trigger: %lu", static_cast<unsigned long>(level));
    return g_radio.send_config(APP_CFG_KEY_SOUND_TRIGGER, level);
}

bool on_gw_pir_trigger_change(uint32_t enable)
{
    ESP_LOGI(TAG, "UI PIR trigger: %s", enable ? "on" : "off");
    return g_radio.send_config(APP_CFG_KEY_PIR_TRIGGER, enable);
}

bool on_gw_voice_alarm_change(uint32_t enable)
{
    ESP_LOGI(TAG, "UI voice alarm: %s", enable ? "on" : "off");
    return g_radio.send_config(APP_CFG_KEY_VOICE_ALARM, enable);
}

bool on_gw_low_power_change(uint32_t enable)
{
    ESP_LOGI(TAG, "UI low power: %s", enable ? "on" : "off");
    // Send first (reads current g_low_power_enabled to decide whether to send
    // the wakeup preamble), then commit local state only on success so the UI
    // switch stays the single source of truth.
    bool ok = g_radio.send_config(APP_CFG_KEY_LOW_POWER, enable);
    if (ok) {
        g_low_power_enabled = (enable != 0);
    }
    return ok;
}

void on_wifi_prov_request(void)
{
    if (wifi_mgr_get_state() == WIFI_MGR_CONNECTED ||
        wifi_mgr_get_state() == WIFI_MGR_PROVISIONING) {
        ESP_LOGW(TAG, "WiFi already active, ignoring prov request");
        return;
    }
    ESP_LOGI(TAG, "WiFi provisioning requested");
    esp_err_t err = wifi_mgr_start_provisioning();
    if (err == ESP_OK) {
        httpd_handle_t h = wifi_mgr_get_httpd();
        if (h) {
            image_store_register_httpd(h);
        }
        const char *name = wifi_mgr_get_service_name();
        char payload[200];
        snprintf(payload, sizeof(payload),
            "{\"ver\":\"v1\",\"name\":\"%s\",\"pop\":\"%s\",\"transport\":\"softap\"}",
            name, wifi_mgr_get_ap_password());
        ui_gw_show_qr(payload);
    } else {
        ESP_LOGE(TAG, "start provisioning failed: %s", esp_err_to_name(err));
    }
}

void on_wifi_disconnect_request(void)
{
    ESP_LOGI(TAG, "WiFi disconnect requested");
    wifi_mgr_disconnect();
}

void on_wifi_state_change(wifi_mgr_state_t state)
{
    const char *str = "Disconnected";
    switch (state) {
    case WIFI_MGR_CONNECTING:    str = "Connecting..."; break;
    case WIFI_MGR_CONNECTED:     str = "Connected"; break;
    case WIFI_MGR_PROVISIONING:  str = "Provisioning..."; break;
    default: break;
    }
    ui_gw_wifi_update(str, wifi_mgr_get_ssid(), wifi_mgr_get_rssi());

    if (state == WIFI_MGR_CONNECTED) {
        wifi_mgr_ensure_httpd();
        httpd_handle_t h = wifi_mgr_get_httpd();
        if (h) {
            image_store_register_httpd(h);
        }
        image_store_start_sntp();
    }
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

    // Camera mode: only K5/USER1 is bound (mode switch). K6/PTT voice, K2/BOOT
    // and K3/K4 volume are intentionally unbound here. The PTT voice-chat path
    // (RadioPing::handle_button, voice queues, encode/play tasks) is kept intact
    // in the codebase, just no longer driven by a key in camera mode.
    if (id == BSP_BTN_USER1) {
        if (pressed) {
            switch_mode_and_restart();
        }
        return;
    }
}
} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Lierda L-LRMAM36-FANN4-DK01 booting");
    prefetch_cache_init();
    esp_log_level_set("RALF_LR20XX", ESP_LOG_WARN);
    // Suppress noisy but harmless HTTP server warnings: /favicon.ico 404s
    // (browser auto-requests it) and recv errno 104 (client closed connection).
    esp_log_level_set("httpd_uri", ESP_LOG_ERROR);
    esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);

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
    if (g_app_mode == AppMode::radio) {
        if ((e = bsp_audio_init_playback_only(APP_AUDIO_SAMPLE_RATE_HZ)) != ESP_OK) {
            ESP_LOGE(TAG, "audio init (playback): %s", esp_err_to_name(e));
        }
    } else {
        if ((e = bsp_audio_init(APP_AUDIO_SAMPLE_RATE_HZ)) != ESP_OK) {
            ESP_LOGE(TAG, "audio init: %s", esp_err_to_name(e));
        }
    }
    if ((e = g_audio.init()) != ESP_OK) {
        ESP_LOGE(TAG, "audio diagnostics init: %s", esp_err_to_name(e));
    }
#if APP_RADIO_FEATURES_ENABLE
    {
        bool radio_ok = true;
        if (g_app_mode == AppMode::radio) {
            if ((e = g_radio.init_gateway()) != ESP_OK) {
                ESP_LOGE(TAG, "radio init (gateway): %s", esp_err_to_name(e));
                radio_ok = false;
            }
        } else {
            if ((e = g_radio.init()) != ESP_OK) {
                ESP_LOGE(TAG, "radio init: %s", esp_err_to_name(e));
                radio_ok = false;
            }
        }
        if (radio_ok) {
            g_radio.set_image_capture_cb(on_image_capture_request);
            g_radio.set_image_rx_complete_cb(on_image_rx_complete);
            g_radio.set_image_rx_progress_cb(on_image_rx_progress);
            g_radio.set_vbat_received_cb(on_vbat_received);
            g_radio.set_image_rx_eot_cb(on_image_rx_eot_nack);
            g_radio.set_config_received_cb(on_config_received);
            g_radio.set_low_power_standby_cb(on_low_power_standby);
        }
        if (g_app_mode == AppMode::radio && radio_ok) {
#if APP_RADIO_TASKS_ENABLE
            if ((e = g_radio.start_gateway()) != ESP_OK) {
                ESP_LOGE(TAG, "radio task start (gateway): %s", esp_err_to_name(e));
            } else {
                g_radio_active = true;
                nvs_handle_t gw_nvs;
                if (nvs_open("ui_gw", NVS_READONLY, &gw_nvs) == ESP_OK) {
                    uint8_t lp = 0;
                    nvs_get_u8(gw_nvs, "lowpwr", &lp);
                    g_low_power_enabled = (lp != 0);
                    nvs_close(gw_nvs);
                }

                // AV sync playback (gateway only): create the paired image+audio
                // queue and start the player task. Pinned to core 1 (with LVGL)
                // at a priority below the UI so display refresh stays smooth.
                g_av_queue = xQueueCreate(APP_GW_AV_QUEUE_SIZE, sizeof(AVFrame));
                if (!g_av_queue) {
                    ESP_LOGE(TAG, "gateway AV queue create failed");
                } else {
                    // 8KB stack: the Opus decoder uses several KB internally,
                    // plus the pcm/stereo frame buffers are on-stack locals.
                    // 4KB overflows and corrupts adjacent memory (crashes at the
                    // next tick, not at the write, so it looks like a scheduler
                    // fault). play_audio_clip never hit this because it ran on a
                    // larger task stack.
                    xTaskCreatePinnedToCore(gateway_audio_task, "gw_audio", 8192,
                                            nullptr, 4, &g_audio_task, 1);
                    ESP_LOGI(TAG, "gateway audio task created");
                }
            }
#else
            ESP_LOGW(TAG, "radio initialized but tasks/RX disabled for camera isolation");
#endif
        } else if (g_app_mode == AppMode::camera && radio_ok) {
            if ((e = g_radio.start()) != ESP_OK) {
                ESP_LOGE(TAG, "radio task start (camera mode): %s", esp_err_to_name(e));
            }
            g_radio.enable_opus_preenc(true);
            ESP_LOGI(TAG, "camera mode: radio initialized for image transfer");
            g_capture_interval_sec = load_capture_interval();
            g_audio_clip_enabled = load_config_u8("audio_clip", 0) != 0;
            g_radio.set_sound_trigger_level(load_config_u8("snd_trig", 0));
            g_radio.set_pir_enabled(load_config_u8("pir", 0) != 0);
            g_voice_alarm_enabled = load_config_u8("alarm", 0) != 0;
            g_low_power_enabled = load_config_u8("lowpwr", 0) != 0;
            ESP_LOGI(TAG, "NVS: audio=%d snd=%d pir=%d alarm=%d lowpwr=%d",
                     g_audio_clip_enabled, load_config_u8("snd_trig", 0),
                     load_config_u8("pir", 0), g_voice_alarm_enabled, g_low_power_enabled);
            start_auto_capture_timer();
            update_camera_timer_status();
            start_countdown_timer();

            // PIR sensor on GPIO12: delay 5s then arm high-level trigger
            // (allow residual touch IC signals to settle after power-on)
            gpio_config_t pir_cfg = {
                .pin_bit_mask = 1ULL << APP_PIR_GPIO,
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_ENABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            gpio_config(&pir_cfg);
            gpio_install_isr_service(0); // OK if already installed
            ESP_LOGI(TAG, "PIR: GPIO%d configured, arming in 5s...", APP_PIR_GPIO);
            const esp_timer_create_args_t pir_arm_args = {
                .callback = pir_arm_timer_cb,
                .arg = &g_radio,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "pir_arm",
                .skip_unhandled_events = true,
            };
            esp_timer_handle_t pir_arm_timer = nullptr;
            esp_timer_create(&pir_arm_args, &pir_arm_timer);
            esp_timer_start_once(pir_arm_timer, 5000000ULL); // 5s
        }
    }
#else
    ESP_LOGW(TAG, "radio feature disabled for camera/audio isolation");
#endif
#endif
    if (g_app_mode == AppMode::camera) {
        if ((e = g_camera_uart.init()) != ESP_OK) {
            ESP_LOGE(TAG, "camera init: %s", esp_err_to_name(e));
        }
#if APP_CAMERA_UART_ENABLE
        if ((e = g_camera_uart.start()) != ESP_OK) {
            ESP_LOGE(TAG, "camera uart start: %s", esp_err_to_name(e));
        }
#endif
    }
    if (g_app_mode != AppMode::camera) {
        if ((e = bsp_lcd_init()) != ESP_OK) {
            ESP_LOGE(TAG, "lcd init: %s", esp_err_to_name(e));
        } else if (g_app_mode == AppMode::radio) {
            if ((e = bsp_lcd_start_gateway_ui()) != ESP_OK) {
                ESP_LOGE(TAG, "gateway ui start: %s", esp_err_to_name(e));
            } else {
                ui_gw_set_capture_cb(on_gw_capture);
                ui_gw_set_interval_cb(on_gw_interval_change);
                ui_gw_set_audio_clip_cb(on_gw_audio_clip_change);
                ui_gw_set_sound_trigger_cb(on_gw_sound_trigger_change);
                ui_gw_set_pir_trigger_cb(on_gw_pir_trigger_change);
                ui_gw_set_voice_alarm_cb(on_gw_voice_alarm_change);
                ui_gw_set_low_power_cb(on_gw_low_power_change);
                ui_gw_set_wifi_prov_cb(on_wifi_prov_request);
                ui_gw_set_wifi_disconnect_cb(on_wifi_disconnect_request);
                ui_gw_set_rx_abort_cb(on_gw_rx_abort);

                wifi_mgr_set_state_cb(on_wifi_state_change);
                wifi_mgr_init();
                image_store_init();
                image_store_restore_time();
                {
                    httpd_handle_t h = wifi_mgr_get_httpd();
                    if (h) image_store_register_httpd(h);
                }

                // Sync audio clip state from gateway UI NVS
                {
                    nvs_handle_t h;
                    uint8_t val = 0;
                    if (nvs_open("ui_gw", NVS_READONLY, &h) == ESP_OK) {
                        nvs_get_u8(h, "audio", &val);
                        nvs_close(h);
                    }
                    g_audio_clip_enabled = (val != 0);
                }
            }
        }
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

    // Battery / external-supply voltage monitor: read VBAT_ADC (GPIO11) every
    // 15 s and cache the result. Radio path grabs the cached value to embed in
    // ImageStart and periodic broadcast packets.
    if ((e = bsp_vbat_monitor_start(0)) != ESP_OK) {
        ESP_LOGW(TAG, "vbat monitor start: %s", esp_err_to_name(e));
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
