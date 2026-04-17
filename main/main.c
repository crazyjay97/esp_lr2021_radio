#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "bsp.h"
#include "LiotLr2021.h"

static const char *TAG = "app";

#define SAMPLE_RATE_HZ      16000
#define REC_SECONDS         3
#define REC_BUF_BYTES       (SAMPLE_RATE_HZ * 2 * REC_SECONDS)   /* mono 16-bit */
#define IO_CHUNK_BYTES      2048

/* Beep cue parameters. */
#define BEEP_FREQ_HZ        1800
#define BEEP_ON_MS          140
#define BEEP_GAP_MS         90
#define BEEP_TAIL_MS        80
#define BEEP_AMP            12000
#define PA_SETTLE_MS        40
#define DIAG_TONE_HZ        1000
#define DIAG_TONE_MS        5000

typedef enum {
    AS_IDLE = 0,
    AS_RECORDING,
    AS_PLAYING,
} audio_state_t;

static volatile uint8_t       s_volume = 70;
static volatile bool          s_muted  = false;
static volatile audio_state_t s_state  = AS_IDLE;
static SemaphoreHandle_t      s_audio_sem;

static uint8_t *s_rec_buf;
static size_t   s_rec_cap;
static size_t   s_rec_len;
static int16_t  s_rec_left_min;
static int16_t  s_rec_left_max;
static int16_t  s_rec_right_min;
static int16_t  s_rec_right_max;
static uint32_t s_rec_left_sum_abs;
static uint32_t s_rec_right_sum_abs;
static size_t   s_rec_frame_count;

static void pa_enable_for_cue(void);

static void reset_record_stats(void)
{
    s_rec_left_min = 0;
    s_rec_left_max = 0;
    s_rec_right_min = 0;
    s_rec_right_max = 0;
    s_rec_left_sum_abs = 0;
    s_rec_right_sum_abs = 0;
    s_rec_frame_count = 0;
}

static void apply_audio_state(void)
{
    bsp_audio_set_volume(s_volume);
    bsp_audio_pa_enable(s_state == AS_PLAYING && !s_muted);

    bool rec  = (s_state == AS_RECORDING);
    bool play = (s_state == AS_PLAYING);
    bool r = s_muted && s_state == AS_IDLE;
    bool g = !s_muted && s_state == AS_IDLE;
    bool b = rec || play;
    bsp_led_set(r, g, b);
}

static void on_button(bsp_btn_id_t id, bool pressed, void *user)
{
    (void)user;
    switch (id) {
    case BSP_BTN_USER1:
        if (pressed && s_state == AS_IDLE) {
            s_rec_len = 0;
            reset_record_stats();
            s_state   = AS_RECORDING;
            ESP_LOGI(TAG, "user1 down -> record");
            apply_audio_state();
            xSemaphoreGive(s_audio_sem);
        } else if (!pressed && s_state == AS_RECORDING) {
            s_state = AS_PLAYING;
            ESP_LOGI(TAG, "user1 up -> play (%u bytes)", (unsigned)s_rec_len);
            apply_audio_state();
            xSemaphoreGive(s_audio_sem);
        }
        break;

    case BSP_BTN_VOL_UP:
        if (!pressed) break;
        s_volume = (s_volume >= 90) ? 100 : s_volume + 10;
        ESP_LOGI(TAG, "vol+ -> %u%%", s_volume);
        apply_audio_state();
        break;

    case BSP_BTN_VOL_DN:
        if (!pressed) break;
        s_volume = (s_volume <= 10) ? 0 : s_volume - 10;
        ESP_LOGI(TAG, "vol- -> %u%%", s_volume);
        apply_audio_state();
        break;

    case BSP_BTN_MUTE:
        if (!pressed) break;
        s_muted = !s_muted;
        ESP_LOGI(TAG, "mute -> %s", s_muted ? "ON" : "OFF");
        apply_audio_state();
        break;

    case BSP_BTN_BOOT:
        if (!pressed) break;
        ESP_LOGI(TAG, "boot pressed");
        break;

    default:
        break;
    }
}

/* ----- Beep generator: square wave, no math lib, stereo frames ----------- */

static void play_tone(int freq_hz, int duration_ms)
{
    const int period = SAMPLE_RATE_HZ / freq_hz;
    const int half   = period / 2;
    const int total  = SAMPLE_RATE_HZ * duration_ms / 1000;
    int16_t buf[128 * 2];   /* up to 128 stereo frames */
    int frame = 0;
    size_t total_written = 0;
    ESP_LOGI(TAG, "tone start: freq=%dHz duration=%dms frames=%d amp=%d",
             freq_hz, duration_ms, total, BEEP_AMP);
    while (frame < total) {
        int batch = total - frame;
        if (batch > 128) batch = 128;
        for (int i = 0; i < batch; i++) {
            int pos = (frame + i) % period;
            int16_t s = (pos < half) ? BEEP_AMP : -BEEP_AMP;
            buf[2 * i]     = s;
            buf[2 * i + 1] = s;
        }
        size_t w = 0;
        esp_err_t err = bsp_audio_write(buf, batch * 4, &w);
        if (err != ESP_OK || w != batch * 4) {
            ESP_LOGW(TAG, "tone write failed: %s, wrote %u/%u",
                     esp_err_to_name(err), (unsigned)w, (unsigned)(batch * 4));
            break;
        }
        total_written += w;
        frame += batch;
    }
    ESP_LOGI(TAG, "tone done: wrote=%u bytes", (unsigned)total_written);
}

static void play_silence(int duration_ms)
{
    static const int16_t zeros[128 * 2] = { 0 };
    const int total = SAMPLE_RATE_HZ * duration_ms / 1000;
    int frame = 0;
    while (frame < total) {
        int batch = total - frame;
        if (batch > 128) batch = 128;
        size_t w = 0;
        esp_err_t err = bsp_audio_write(zeros, batch * 4, &w);
        if (err != ESP_OK || w != batch * 4) {
            ESP_LOGW(TAG, "silence write failed: %s, wrote %u/%u",
                     esp_err_to_name(err), (unsigned)w, (unsigned)(batch * 4));
            break;
        }
        frame += batch;
    }
}

static void beep_beep(void)
{
    ESP_LOGI(TAG, "beep-beep start");
    play_tone(BEEP_FREQ_HZ, BEEP_ON_MS);
    play_silence(BEEP_GAP_MS);
    play_tone(BEEP_FREQ_HZ, BEEP_ON_MS);
    play_silence(BEEP_TAIL_MS);
    ESP_LOGI(TAG, "beep-beep done");
}

static void startup_audio_diag(void)
{
    ESP_LOGI(TAG, "startup audio diag: PA on, %d ms tone @ %d Hz",
             DIAG_TONE_MS, DIAG_TONE_HZ);
    pa_enable_for_cue();
    play_tone(DIAG_TONE_HZ, DIAG_TONE_MS);
    play_silence(100);
    bsp_audio_pa_enable(false);
    ESP_LOGI(TAG, "startup audio diag done");
}

static int32_t abs16(int16_t v)
{
    return v < 0 ? -(int32_t)v : v;
}

static size_t record_mono_chunk(size_t room)
{
    int16_t stereo[IO_CHUNK_BYTES / sizeof(int16_t)];
    size_t got = 0;
    esp_err_t err = bsp_audio_read(stereo, sizeof(stereo), &got);
    if (err != ESP_OK || got < 4) {
        ESP_LOGW(TAG, "record read failed: %s, got=%u",
                 esp_err_to_name(err), (unsigned)got);
        return 0;
    }

    int16_t *dst = (int16_t *)(s_rec_buf + s_rec_len);
    size_t frames = got / 4;
    size_t max_frames = room / sizeof(int16_t);
    if (frames > max_frames) frames = max_frames;

    for (size_t i = 0; i < frames; i++) {
        int16_t left = stereo[2 * i];
        int16_t right = stereo[2 * i + 1];
        if (left < s_rec_left_min) s_rec_left_min = left;
        if (left > s_rec_left_max) s_rec_left_max = left;
        if (right < s_rec_right_min) s_rec_right_min = right;
        if (right > s_rec_right_max) s_rec_right_max = right;
        s_rec_left_sum_abs += (uint32_t)abs16(left);
        s_rec_right_sum_abs += (uint32_t)abs16(right);
        dst[i] = (abs16(left) >= abs16(right)) ? left : right;
    }
    s_rec_frame_count += frames;

    return frames * sizeof(int16_t);
}

static void play_mono_buffer(void)
{
    int16_t stereo[256 * 2];
    const int16_t *src = (const int16_t *)s_rec_buf;
    size_t samples = s_rec_len / sizeof(int16_t);
    size_t pos = 0;
    size_t total_written = 0;

    ESP_LOGI(TAG, "play mono start: samples=%u bytes=%u",
             (unsigned)samples, (unsigned)s_rec_len);

    while (pos < samples) {
        size_t batch = samples - pos;
        if (batch > 256) batch = 256;

        for (size_t i = 0; i < batch; i++) {
            int16_t s = src[pos + i];
            stereo[2 * i] = s;
            stereo[2 * i + 1] = s;
        }

        size_t sent = 0;
        esp_err_t err = bsp_audio_write(stereo, batch * 4, &sent);
        if (err != ESP_OK || sent == 0) {
            ESP_LOGW(TAG, "playback write failed: %s, pos=%u",
                     esp_err_to_name(err), (unsigned)pos);
            break;
        }
        total_written += sent;
        pos += sent / 4;
    }
    ESP_LOGI(TAG, "play mono done: wrote=%u bytes", (unsigned)total_written);
}

static void pa_enable_for_cue(void)
{
    esp_err_t err = bsp_audio_pa_enable(true);
    ESP_LOGI(TAG, "PA enable for cue -> %s", esp_err_to_name(err));
    vTaskDelay(pdMS_TO_TICKS(PA_SETTLE_MS));
}

static void log_record_stats(void)
{
    const int16_t *samples = (const int16_t *)s_rec_buf;
    size_t count = s_rec_len / sizeof(int16_t);
    int16_t min = 0;
    int16_t max = 0;
    uint32_t sum_abs = 0;

    for (size_t i = 0; i < count; i++) {
        int16_t s = samples[i];
        if (s < min) min = s;
        if (s > max) max = s;
        sum_abs += (uint32_t)abs16(s);
    }

    uint32_t avg_abs = count ? sum_abs / count : 0;
    ESP_LOGI(TAG, "record stats: samples=%u min=%d max=%d avg_abs=%u",
             (unsigned)count, min, max, (unsigned)avg_abs);
    ESP_LOGI(TAG, "record raw channels: frames=%u L[min=%d max=%d avg_abs=%u] R[min=%d max=%d avg_abs=%u]",
             (unsigned)s_rec_frame_count,
             s_rec_left_min, s_rec_left_max,
             (unsigned)(s_rec_frame_count ? s_rec_left_sum_abs / s_rec_frame_count : 0),
             s_rec_right_min, s_rec_right_max,
             (unsigned)(s_rec_frame_count ? s_rec_right_sum_abs / s_rec_frame_count : 0));
}

static void drain_rx(int frames)
{
    uint8_t scratch[IO_CHUNK_BYTES];
    int left = frames * 4;
    while (left > 0) {
        size_t got = 0;
        int want = left < (int)sizeof(scratch) ? left : (int)sizeof(scratch);
        if (bsp_audio_read(scratch, want, &got) != ESP_OK || got == 0) break;
        left -= got;
    }
}

static void audio_task(void *arg)
{
    (void)arg;

    while (1) {
        xSemaphoreTake(s_audio_sem, portMAX_DELAY);

        /* Pre-record cue: PA on for the beeps, off for the actual capture. */
        if (s_state == AS_RECORDING) {
            ESP_LOGI(TAG, "record cue: PA on, then beep");
            pa_enable_for_cue();
            beep_beep();
            ESP_LOGI(TAG, "record cue done: PA off, start capture");
            bsp_audio_pa_enable(false);
            /* Flush DMA'd samples and ~50 ms of acoustic tail. */
            drain_rx(SAMPLE_RATE_HZ / 20);

            while (s_state == AS_RECORDING) {
                size_t room = s_rec_cap - s_rec_len;
                if (room == 0) {
                    ESP_LOGW(TAG, "record buffer full");
                    s_state = AS_PLAYING;
                    apply_audio_state();
                    break;
                }
                size_t got = record_mono_chunk(room);
                if (got == 0) break;
                s_rec_len += got;
            }
            log_record_stats();
        }

        /* Pre-playback cue: PA is already on from apply_audio_state. */
        if (s_state == AS_PLAYING) {
            ESP_LOGI(TAG, "playback cue: PA on, then beep");
            pa_enable_for_cue();
            beep_beep();
            play_mono_buffer();
            play_silence(80);
            ESP_LOGI(TAG, "playback done");
            s_state = AS_IDLE;
            apply_audio_state();
        }
    }
}

static esp_err_t record_buf_alloc(void)
{
    s_rec_cap = REC_BUF_BYTES;
    s_rec_buf = heap_caps_malloc(s_rec_cap, MALLOC_CAP_SPIRAM);
    if (!s_rec_buf) s_rec_buf = heap_caps_malloc(s_rec_cap, MALLOC_CAP_8BIT);
    if (!s_rec_buf) {
        ESP_LOGE(TAG, "record buffer alloc %u failed", (unsigned)s_rec_cap);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "record buffer %u bytes @%p", (unsigned)s_rec_cap, s_rec_buf);
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Lierda L-LRMAM36-FANN4-DK01 booting");

    ESP_ERROR_CHECK(bsp_i2c_init());
    bsp_i2c_scan();

    esp_err_t e;
    if ((e = bsp_led_init())                    != ESP_OK) ESP_LOGE(TAG, "led init: %s",   esp_err_to_name(e));
    if ((e = bsp_audio_init(SAMPLE_RATE_HZ))    != ESP_OK) ESP_LOGE(TAG, "audio init: %s", esp_err_to_name(e));
    if ((e = bsp_button_init(on_button, NULL))  != ESP_OK) ESP_LOGE(TAG, "btn init: %s",   esp_err_to_name(e));

    ESP_ERROR_CHECK(record_buf_alloc());
    s_audio_sem = xSemaphoreCreateBinary();

    apply_audio_state();
    xTaskCreate(audio_task, "audio", 4096, NULL, 6, NULL);
    startup_audio_diag();
    ESP_LOGI(TAG, "hold USER1 (K3): beep-beep then record; release: beep-beep then play. "
                  "K4=vol-, K5=mute, K6=vol+");

    smtc_rac_init();
    while (1) {
        smtc_rac_run_engine();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
