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

class RadioPing {
public:
    esp_err_t init();
    esp_err_t start();
    void handle_button(bsp_btn_id_t id, bool pressed);

private:
    enum class Mode {
        idle,
        rx_pending,
        tx_pending,
    };

    static void task_trampoline(void *arg);
    static void play_task_trampoline(void *arg);
    static void post_callback(rp_status_t status);

    void task();
    void play_task();
    void poll_once();
    void on_done(rp_status_t status);
    void schedule_rx();
    void schedule_tx();
    void configure_common(smtc_rac_context_t *ctx, bool is_tx, uint16_t tx_size);
    bool build_voice_packet(uint16_t *tx_size);
    void handle_rx_packet();
    void queue_voice_packet(uint16_t len, int16_t rssi);
    void log_rx(uint16_t seq, uint16_t len, int16_t rssi);
    bool read_mono_frame(int16_t *mono, size_t samples);
    void play_mono_frame(const int16_t *mono, size_t samples);
    void update_playback_timeout();

    struct VoicePacket {
        uint16_t seq;
        uint16_t len;
        int16_t rssi;
        uint8_t payload[APP_OPUS_MAX_PACKET_BYTES];
    };

    static RadioPing *instance_;

    uint8_t radio_id_ = RAC_INVALID_RADIO_ID;
    OpusCodec codec_;
    QueueHandle_t voice_queue_ = nullptr;
    Mode mode_ = Mode::idle;
    bool ptt_active_ = false;
    volatile bool done_ = false;
    volatile rp_status_t done_status_ = RP_STATUS_TASK_INIT;

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
    uint32_t rx_queue_drops_ = 0;
    uint32_t last_rx_audio_ms_ = 0;
    bool playback_active_ = false;
};
