#pragma once

#include <cstdint>
#include <cstddef>

#include "esp_err.h"
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
    // Image transfer: A sends JPEG fragments to B
    void send_image(const uint8_t *jpeg, size_t jpeg_len, uint16_t session_id);
    // Register callbacks
    void set_image_capture_cb(image_capture_cb_t cb) { image_capture_cb_ = cb; }
    void set_image_rx_complete_cb(image_rx_complete_cb_t cb) { image_rx_complete_cb_ = cb; }
    void set_image_rx_progress_cb(image_rx_progress_cb_t cb) { image_rx_progress_cb_ = cb; }
    void set_image_rx_eot_cb(image_rx_eot_cb_t cb) { image_rx_eot_cb_ = cb; }
    void set_config_received_cb(config_received_cb_t cb) { config_received_cb_ = cb; }
    void set_inter_packet_us(uint32_t us) { image_tx_inter_packet_us_ = us; }

    bool send_config(uint8_t key, uint32_t value);

    ImageTransfer &image_xfer() { return image_xfer_; }
    uint32_t last_transfer_ms() const { return image_rx_transfer_ms_; }
    bool image_tx_busy() const { return image_tx_active_; }
    void pause_audio_capture() { image_tx_active_ = true; }
    void resume_audio_capture() { image_tx_active_ = false; }
    void enable_opus_preenc(bool en) { opus_preenc_enabled_ = en; }

    // Audio ring buffer for pre-capture retrospective recording
    size_t snapshot_audio(int16_t *out, size_t max_samples);
    size_t snapshot_opus(uint8_t *out, size_t max_bytes);

private:
    enum class Mode {
        idle,
        rx_pending,
        tx_pending,
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
    void handle_image_start();
    void handle_image_data();
    void handle_image_eot();
    void handle_image_nack();
    void handle_image_done();
    void image_tx_task();
    bool send_single_packet(const uint8_t *data, uint16_t len);
    bool wait_for_tx_done(uint32_t timeout_ms);
    void check_image_rx_timeout();
    void send_config_ack(uint8_t key, uint32_t value);

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
    uint16_t image_session_id_ = 1;
    volatile bool image_tx_active_ = false;
    bool opus_preenc_enabled_ = false;
    uint32_t image_tx_inter_packet_us_ = APP_IMAGE_TX_INTER_PACKET_US;
    uint32_t image_rx_last_frag_ms_ = 0;
    uint32_t image_rx_last_progress_ms_ = 0;
    uint32_t image_rx_expected_crc32_ = 0;
    bool image_rx_pending_ = false;
    uint16_t image_rx_nack_sent_ = 0;
    uint16_t image_rx_eot_count_ = 0;
    int16_t image_rx_last_rssi_ = 0;
    uint16_t image_rx_done_session_ = 0;
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
};
