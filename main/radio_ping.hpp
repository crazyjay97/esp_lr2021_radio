#pragma once

#include <cstdint>
#include <cstddef>

#include "esp_err.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "app_config.h"
#include "bsp.h"
#include "LiotLr2021.h"
#include "opus_codec.hpp"
#include "audio_processor.hpp"
#include "image_transfer.hpp"
#include "audio_ringbuf.hpp"
#include "opus_ringbuf.hpp"

typedef void (*image_capture_cb_t)(uint16_t session_id);
typedef void (*image_rx_complete_cb_t)(ImageTransfer *xfer);
typedef void (*image_rx_progress_cb_t)(uint16_t received, uint16_t total, int16_t rssi);
typedef void (*image_rx_eot_cb_t)(uint16_t missing_count, bool is_first_eot);
typedef void (*config_received_cb_t)(uint8_t key, uint32_t value);
// Low power: called when the node enters (true) / leaves (false) CAD sleep
// standby, so the app can release/restore power-hungry peripherals (camera, I2S).
typedef void (*low_power_standby_cb_t)(bool entering);

class RadioPing {
public:
    esp_err_t init();
    esp_err_t init_gateway();
    esp_err_t start();
    esp_err_t start_gateway();
    void handle_button(bsp_btn_id_t id, bool pressed);
    void suspend();
    void resume();

    // Image transfer: B triggers A to capture
    void trigger_image_capture();
    // Gateway: abort the in-progress image RX (and any pending ImageCmd retry).
    // Called from the UI thread when the user leaves the transfer page; only
    // sets a request flag so the actual radio-state teardown happens in the
    // radio task (check_image_rx_abort), keeping all radio access serialized.
    void abort_image_rx() { image_rx_abort_req_ = true; }
    // Image transfer: A sends JPEG fragments to B.
    // Takes ownership of `jpeg` (must be a heap_caps allocation): the tx task
    // frees it when the transfer completes/aborts. Callers must not free it.
    void send_image(const uint8_t *jpeg, size_t jpeg_len, uint16_t session_id);
    // Register callbacks
    void set_image_capture_cb(image_capture_cb_t cb) { image_capture_cb_ = cb; }
    void set_image_rx_complete_cb(image_rx_complete_cb_t cb) { image_rx_complete_cb_ = cb; }
    void set_image_rx_progress_cb(image_rx_progress_cb_t cb) { image_rx_progress_cb_ = cb; }
    void set_image_rx_eot_cb(image_rx_eot_cb_t cb) { image_rx_eot_cb_ = cb; }
    void set_config_received_cb(config_received_cb_t cb) { config_received_cb_ = cb; }
    void set_low_power_standby_cb(low_power_standby_cb_t cb) { low_power_standby_cb_ = cb; }
    void set_inter_packet_us(uint32_t us) { image_tx_inter_packet_us_ = us; }

    bool send_config(uint8_t key, uint32_t value);

    ImageTransfer &image_xfer() { return image_xfer_; }
    uint32_t last_transfer_ms() const { return image_rx_transfer_ms_; }
    bool image_tx_busy() const { return image_tx_active_; }
    void pause_audio_capture() { image_tx_active_ = true; }
    void resume_audio_capture() { image_tx_active_ = false; }
    void enable_opus_preenc(bool en) { opus_preenc_enabled_ = en; }
    void set_sound_trigger_level(uint32_t level) { sound_trigger_level_ = level; }
    void set_pir_enabled(bool en) { pir_enabled_ = en; }
    void IRAM_ATTR pir_trigger() { pir_triggered_ = true; }
    // Tracks whether the PIR level trigger is currently armed. Cleared by the
    // ISR on detection (trigger disabled), set again by the 15s re-arm timer.
    // low_power_sleep only arms the light-sleep GPIO wake when this is true, so
    // during the 15s cooldown the node won't wake on the still-high PIR pin.
    void IRAM_ATTR set_pir_armed(bool armed) { pir_armed_ = armed; }
    bool pir_armed() const { return pir_armed_; }

    // Called when a PIR-triggered capture is dropped before it can push (not in
    // camera mode, capture busy, audio cooldown). Ends the PIR keep-awake guard
    // immediately so the node returns to CAD sleep on the next idle pass instead
    // of staying deaf to the gateway for the full 8s safety timeout.
    void notify_capture_dropped() { pir_push_wake_ = false; }

    // Called when a capture is actually dispatched (auto-capture timer, gateway
    // request, or PIR). In low power the node must stay awake-but-idle through
    // the capture + image push, exactly like the PIR path — otherwise poll_once
    // re-enters CAD light sleep and halts the whole chip for 500ms at a time,
    // starving the camera/JPEG task so the capture never completes. This reuses
    // the PIR keep-awake guard; image_tx_task clears it on completion and the 8s
    // poll_once timeout is only a fallback if the capture never reaches TX.
    void notify_capture_starting();

    // Untimed keep-awake for synchronous audio playback (e.g. the post-capture
    // voice alarm). Unlike pir_push_wake_ (8s safety timeout), this has NO
    // timeout: light sleep halts both cores and would starve/garble I2S output
    // and re-sleep the node mid-clip. The caller MUST pair begin/end. While set,
    // poll_once never enters CAD light sleep, so the node stays awake exactly for
    // the clip and re-sleeps on the next idle pass after audio_playback_end().
    void audio_playback_begin() { audio_playing_ = true; }
    void audio_playback_end() { audio_playing_ = false; }

    // Audio ring buffer for pre-capture retrospective recording
    size_t snapshot_audio(int16_t *out, size_t max_samples);
    size_t snapshot_opus(uint8_t *out, size_t max_bytes);

private:
    enum class Mode {
        idle,
        rx_pending,
        tx_pending,
        cad_pending,
    };

    static void task_trampoline(void *arg);
    static void tx_task_trampoline(void *arg);
    static void play_task_trampoline(void *arg);
    static void image_tx_task_trampoline(void *arg);
    static void irq_callback(void *context);

    void task();
    void tx_task();
    void play_task();
    void poll_once();
    void handle_irq(ral_irq_t irq);
    void schedule_rx();
    void schedule_tx();
    bool configure_flrc();
    bool build_voice_packet(uint16_t *tx_size);
    void capture_voice_packet();
    void handle_rx_packet();
    void queue_voice_packet(uint16_t len, int16_t rssi);
    void log_rx(uint16_t seq, uint16_t len, int16_t rssi);
    void wait_for_jitter_buffer();
    void conceal_missing_frames(uint16_t seq);
    bool read_mono_frame(int16_t *mono, size_t samples);
    void play_mono_frame(const int16_t *mono, size_t samples);
    void set_playback_pa(bool on);
    void update_playback_timeout();

    // Image transfer methods
    void handle_image_cmd();
    void handle_image_cmd_ack();
    // len = received packet length, needed to validate the trailing CRC32.
    // Send one ImageCmd packet for image_req_session_ (does LoRa wakeup +
    // FLRC reconfig first in low power). Shared by trigger_image_capture and
    // the retry poll.
    void send_image_cmd_once();
    // Called from the radio task loop: resend ImageCmd when the retry interval
    // elapses and the node hasn't acked yet. Runs in the SAME task as poll_once
    // so radio access stays serialized (no IRQ/mode races with an esp_timer).
    void check_image_req_retry();
    void stop_image_req_retry();
    void handle_image_start(uint16_t len);
    void handle_image_data();
    void handle_image_eot();
    void handle_image_nack();
    void handle_image_done();
    void image_tx_task();
    bool send_single_packet(const uint8_t *data, uint16_t len);
    bool wait_for_tx_done(uint32_t timeout_ms);
    void check_image_rx_timeout();
    void check_image_rx_abort();
    void send_config_ack(uint8_t key, uint32_t value);
    bool configure_lora_cad();
    void enter_low_power_cad();
    // Light-sleep the ESP32 for up to `ms`, waking on the timer or (if PIR is
    // enabled) the PIR GPIO. Returns true if woken by PIR. Used during CAD
    // standby to save whole-chip power.
    bool low_power_sleep(uint32_t ms);
    void handle_cad_irq(ral_irq_t irq);
    bool send_lora_wakeup();

    struct VoicePacket {
        uint16_t seq;
        uint16_t len;
        int16_t rssi;
        uint8_t payload[APP_OPUS_MAX_PACKET_BYTES];
    };

    struct TxFrame {
        uint16_t seq;
        uint16_t len;
        uint8_t payload[APP_OPUS_MAX_PACKET_BYTES];
    };

    struct ImageTxRequest {
        const uint8_t *jpeg;
        size_t jpeg_len;
        uint16_t session_id;
    };

    static RadioPing *instance_;

    TaskHandle_t task_handle_ = nullptr;
    ralf_t radio_ = RALF_LR20XX_INSTANTIATE(nullptr);
    OpusCodec codec_;
    AudioProcessor audio_proc_;
    ImageTransfer image_xfer_;
    AudioRingBuf audio_ringbuf_;
    OpusRingBuf opus_ringbuf_;
    QueueHandle_t voice_queue_ = nullptr;
    QueueHandle_t tx_queue_ = nullptr;
    QueueHandle_t image_tx_queue_ = nullptr;
    Mode mode_ = Mode::idle;
    volatile bool ptt_active_ = false;
    volatile bool suspended_ = false;
    bool tx_burst_active_ = false;
    bool tx_flush_pending_ = false;
    volatile bool irq_pending_ = false;

    uint8_t tx_buf_[APP_FLRC_MAX_PAYLOAD_BYTES] = {};
    uint8_t rx_buf_[APP_FLRC_MAX_PAYLOAD_BYTES] = {};
    int16_t tx_pcm_[APP_AUDIO_FRAME_SAMPLES] = {};
    int16_t rx_pcm_[APP_AUDIO_FRAME_SAMPLES] = {};

    uint16_t tx_seq_ = 0;
    uint16_t expected_rx_seq_ = 0;
    bool have_expected_rx_seq_ = false;
    uint32_t rx_packets_ = 0;
    uint32_t rx_lost_ = 0;
    uint32_t rx_crc_errors_ = 0;
    uint32_t rx_unknown_packets_ = 0;
    uint32_t rx_queue_drops_ = 0;
    uint32_t tx_queue_drops_ = 0;
    uint32_t last_rx_audio_ms_ = 0;
    uint16_t expected_play_seq_ = 0;
    bool have_expected_play_seq_ = false;
    bool playback_pa_on_ = false;
    bool playback_active_ = false;

    // Image transfer state
    image_capture_cb_t image_capture_cb_ = nullptr;
    image_rx_complete_cb_t image_rx_complete_cb_ = nullptr;
    image_rx_progress_cb_t image_rx_progress_cb_ = nullptr;
    image_rx_eot_cb_t image_rx_eot_cb_ = nullptr;
    config_received_cb_t config_received_cb_ = nullptr;
    low_power_standby_cb_t low_power_standby_cb_ = nullptr;
    uint16_t image_session_id_ = 1;
    volatile bool image_tx_active_ = false;
    bool opus_preenc_enabled_ = false;
    uint32_t image_tx_inter_packet_us_ = APP_IMAGE_TX_INTER_PACKET_US;
    uint32_t image_rx_last_frag_ms_ = 0;
    uint32_t image_rx_last_progress_ms_ = 0;
    uint32_t image_rx_expected_crc32_ = 0;
    bool image_rx_pending_ = false;
    volatile bool image_rx_abort_req_ = false;
    uint16_t image_rx_nack_sent_ = 0;
    uint16_t image_rx_eot_count_ = 0;
    int16_t image_rx_last_rssi_ = 0;
    uint16_t image_rx_done_session_ = 0;
    // Gateway ImageCmd request-retry state. image_req_session_ is fixed for the
    // whole request (NOT re-generated per retry), so retries never spawn a new
    // capture. image_req_active_ is cleared when the node replies (ImageCmdAck)
    // OR when an ImageStart arrives (backstop if the ack was lost). The retry is
    // driven from the radio task loop (check_image_req_retry), NOT an esp_timer,
    // so all radio ops stay in one task. image_req_next_ms_ = next resend time.
    bool image_req_active_ = false;
    uint16_t image_req_session_ = 0;
    uint32_t image_req_next_ms_ = 0;
    uint32_t image_cmd_sent_ms_ = 0;
    uint32_t image_rx_start_ms_ = 0;
    uint32_t image_rx_transfer_ms_ = 0;
    uint32_t image_rx_done_ms_ = 0;

    // NACK receive state for TX side (A)
    volatile bool image_nack_received_ = false;
    volatile bool image_done_received_ = false;
    uint16_t nack_indices_[APP_IMAGE_NACK_MAX_INDICES] = {};
    uint16_t nack_count_ = 0;

    // Config ACK state
    volatile bool config_ack_received_ = false;

    // Low power CAD state
    bool low_power_cad_active_ = false;
    bool is_gateway_ = false;
    uint32_t cad_wakeup_ms_ = 0;
    // Timestamp (ms) when the radio entered cad_pending. A missed CAD_DONE IRQ
    // would otherwise leave mode_ stuck in cad_pending forever (poll_once only
    // acts on idle), and even "exit low power" can't recover it (that path also
    // needs idle). The task() watchdog resets to idle after a timeout so the
    // node self-heals instead of requiring a power cycle. 0 = not waiting.
    uint32_t cad_pending_ms_ = 0;
    // PIR self-push wake: node woke on PIR to CAPTURE+PUSH an image (it is the
    // transmitter, not a receiver). Keeps the loop awake (no RX, no CAD) until
    // image_tx_task takes over the radio. Distinct from cad_wakeup_ms_, which is
    // the gateway-request RX window (node received a LoRa wakeup, must RX cmd).
    volatile bool pir_push_wake_ = false;
    uint32_t pir_push_wake_ms_ = 0;
    // Untimed keep-awake while a synchronous audio clip (post-capture voice
    // alarm) is playing. Blocks CAD light sleep so I2S output isn't halted and
    // the node doesn't re-sleep mid-clip. Set/cleared via audio_playback_*().
    volatile bool audio_playing_ = false;

    // Sound/PIR trigger state (shared cooldown)
    uint32_t sound_trigger_level_ = 0;
    bool pir_enabled_ = false;
    int64_t last_trigger_us_ = 0;
    uint16_t sound_trigger_session_id_ = 0xC000;
    volatile bool pir_triggered_ = false;
    volatile bool pir_armed_ = false;
};
