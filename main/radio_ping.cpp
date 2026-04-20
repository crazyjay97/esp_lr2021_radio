#include "radio_ping.hpp"

#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "app_config.h"

namespace {
constexpr const char *TAG = "radio_ping";
constexpr uint8_t kSyncWord[4] = {
    APP_FLRC_SYNC_WORD_0,
    APP_FLRC_SYNC_WORD_1,
    APP_FLRC_SYNC_WORD_2,
    APP_FLRC_SYNC_WORD_3,
};
constexpr uint8_t kMagic[4] = { 'L', 'R', 'P', '1' };
constexpr uint8_t kPacketTypePing = 1;
constexpr uint8_t kPacketTypeVoice = 2;
constexpr uint16_t kHeaderSize = 14;

int32_t abs16(int16_t v)
{
    return v < 0 ? -static_cast<int32_t>(v) : v;
}

const char *status_to_string(rp_status_t status)
{
    switch (status) {
    case RP_STATUS_RX_CRC_ERROR: return "RX_CRC_ERROR";
    case RP_STATUS_TX_DONE: return "TX_DONE";
    case RP_STATUS_RX_PACKET: return "RX_PACKET";
    case RP_STATUS_RX_TIMEOUT: return "RX_TIMEOUT";
    case RP_STATUS_TASK_ABORTED: return "TASK_ABORTED";
    default: return "OTHER";
    }
}

void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

void put_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

uint16_t get_u16_le(const uint8_t *p)
{
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

TickType_t ms_to_ticks_min_1(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    return ticks == 0 ? 1 : ticks;
}

} // namespace

RadioPing *RadioPing::instance_ = nullptr;

esp_err_t RadioPing::init()
{
    instance_ = this;

    smtc_rac_init();
    esp_err_t err = codec_.init();
    if (err != ESP_OK) {
        return err;
    }

    voice_queue_ = xQueueCreate(APP_VOICE_RX_QUEUE_LEN, sizeof(VoicePacket));
    if (voice_queue_ == nullptr) {
        ESP_LOGE(TAG, "voice queue alloc failed");
        return ESP_ERR_NO_MEM;
    }
    tx_queue_ = xQueueCreate(APP_VOICE_TX_QUEUE_LEN, sizeof(TxPacket));
    if (tx_queue_ == nullptr) {
        ESP_LOGE(TAG, "tx queue alloc failed");
        return ESP_ERR_NO_MEM;
    }

    radio_id_ = smtc_rac_open_radio(RAC_LOW_PRIORITY);
    if (radio_id_ == RAC_INVALID_RADIO_ID) {
        ESP_LOGE(TAG, "smtc_rac_open_radio failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LR2021 RAC initialized: radio_id=%u FLRC rf=%lu Hz br=%lu bps bw=%lu Hz",
             radio_id_, APP_FLRC_FREQUENCY_HZ, APP_FLRC_BITRATE_BPS,
             APP_FLRC_BANDWIDTH_HZ);
    schedule_rx();
    return ESP_OK;
}

esp_err_t RadioPing::start()
{
    BaseType_t ok = xTaskCreate(task_trampoline, "radio_ping",
                                APP_RADIO_TASK_STACK_BYTES, this,
                                APP_RADIO_TASK_PRIORITY, nullptr);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreate(tx_task_trampoline, "voice_tx",
                     APP_VOICE_TX_TASK_STACK_BYTES, this,
                     APP_VOICE_TX_TASK_PRIORITY, nullptr);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreate(play_task_trampoline, "voice_play",
                     APP_VOICE_PLAY_TASK_STACK_BYTES, this,
                     APP_VOICE_PLAY_TASK_PRIORITY, nullptr);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void RadioPing::handle_button(bsp_btn_id_t id, bool pressed)
{
    if (id != APP_PTT_BUTTON) return;

    ptt_active_ = pressed;
    ESP_LOGI(TAG, "PTT %s -> FLRC voice %s", pressed ? "down" : "up",
             pressed ? "TX" : "RX");

    if (pressed && mode_ == Mode::rx_pending) {
        set_playback_pa(false);
        playback_active_ = false;
        have_expected_play_seq_ = false;
        codec_.reset_encoder();
        if (voice_queue_ != nullptr) {
            xQueueReset(voice_queue_);
        }
        if (tx_queue_ != nullptr) {
            xQueueReset(tx_queue_);
        }
        smtc_rac_abort_radio_submit(radio_id_);
    } else if (!pressed) {
        if (tx_queue_ != nullptr) {
            xQueueReset(tx_queue_);
        }
        if (mode_ == Mode::idle) {
            schedule_rx();
        }
    }
}

void RadioPing::task_trampoline(void *arg)
{
    static_cast<RadioPing *>(arg)->task();
}

void RadioPing::tx_task_trampoline(void *arg)
{
    static_cast<RadioPing *>(arg)->tx_task();
}

void RadioPing::play_task_trampoline(void *arg)
{
    static_cast<RadioPing *>(arg)->play_task();
}

void RadioPing::task()
{
    while (true) {
        poll_once();
        update_playback_timeout();
        vTaskDelay(ms_to_ticks_min_1(APP_RADIO_TASK_POLL_MS));
    }
}

void RadioPing::tx_task()
{
    while (true) {
        if (!ptt_active_) {
            vTaskDelay(ms_to_ticks_min_1(APP_AUDIO_FRAME_MS));
            continue;
        }
        capture_voice_packet();
    }
}

void RadioPing::play_task()
{
    VoicePacket packet;

    while (true) {
        if (xQueueReceive(voice_queue_, &packet, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!playback_active_) {
            codec_.reset_decoder();
        }
        wait_for_jitter_buffer();
        conceal_missing_frames(packet.seq);

        int decoded = codec_.decode(packet.payload, packet.len, rx_pcm_, APP_AUDIO_FRAME_SAMPLES);
        if (decoded <= 0) {
            ESP_LOGW(TAG, "Opus decode failed: %d", decoded);
            continue;
        }

        play_mono_frame(rx_pcm_, static_cast<size_t>(decoded));
        last_rx_audio_ms_ = smtc_modem_hal_get_time_in_ms();
        playback_active_ = true;
    }
}

void RadioPing::poll_once()
{
    smtc_rac_run_engine();
    taskYIELD();

    if (done_) {
        done_ = false;
        on_done(done_status_);
    }

    if (ptt_active_ && mode_ == Mode::idle) {
        schedule_tx();
    } else if (!ptt_active_ && mode_ == Mode::idle) {
        schedule_rx();
    }
    taskYIELD();
}

void RadioPing::post_callback(rp_status_t status)
{
    if (instance_ == nullptr) return;
    instance_->done_status_ = status;
    instance_->done_ = true;
}

void RadioPing::on_done(rp_status_t status)
{
    Mode completed_mode = mode_;
    mode_ = Mode::idle;

    if (completed_mode == Mode::rx_pending) {
        if (status == RP_STATUS_RX_PACKET) {
            handle_rx_packet();
        } else if (status == RP_STATUS_RX_CRC_ERROR) {
            rx_crc_errors_++;
            if ((rx_crc_errors_ % 10) == 1) {
                ESP_LOGW(TAG, "RX CRC errors=%lu", static_cast<unsigned long>(rx_crc_errors_));
            }
        } else if (status != RP_STATUS_RX_TIMEOUT && status != RP_STATUS_TASK_ABORTED) {
            ESP_LOGW(TAG, "RX done: %s", status_to_string(status));
        }
    } else if (completed_mode == Mode::tx_pending) {
        if (status != RP_STATUS_TX_DONE) {
            ESP_LOGW(TAG, "TX done: %s", status_to_string(status));
        }
        if (APP_FLRC_VOICE_TX_GAP_MS > 0) {
            vTaskDelay(pdMS_TO_TICKS(APP_FLRC_VOICE_TX_GAP_MS));
        }
    }
}

void RadioPing::schedule_rx()
{
    if (radio_id_ == RAC_INVALID_RADIO_ID || mode_ != Mode::idle) return;

    smtc_rac_context_t *ctx = smtc_rac_get_context(radio_id_);
    std::memset(ctx, 0, sizeof(*ctx));
    configure_common(ctx, false, 0);
    ctx->smtc_rac_data_buffer_setup.rx_payload_buffer = rx_buf_;
    ctx->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = sizeof(rx_buf_);

    smtc_rac_return_code_t rc = smtc_rac_submit_radio_transaction(radio_id_);
    if (rc == SMTC_RAC_SUCCESS) {
        mode_ = Mode::rx_pending;
    } else {
        ESP_LOGW(TAG, "schedule RX failed: %d", rc);
    }
}

void RadioPing::schedule_tx()
{
    if (radio_id_ == RAC_INVALID_RADIO_ID || mode_ != Mode::idle) return;

    uint16_t tx_size = 0;
    if (!build_voice_packet(&tx_size)) {
        return;
    }

    smtc_rac_context_t *ctx = smtc_rac_get_context(radio_id_);
    std::memset(ctx, 0, sizeof(*ctx));
    configure_common(ctx, true, tx_size);
    ctx->smtc_rac_data_buffer_setup.tx_payload_buffer = tx_buf_;
    ctx->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = tx_size;

    smtc_rac_return_code_t rc = smtc_rac_submit_radio_transaction(radio_id_);
    if (rc == SMTC_RAC_SUCCESS) {
        mode_ = Mode::tx_pending;
    } else {
        ESP_LOGW(TAG, "schedule TX failed: %d", rc);
    }
}

void RadioPing::configure_common(smtc_rac_context_t *ctx, bool is_tx, uint16_t tx_size)
{
    ctx->modulation_type = SMTC_RAC_MODULATION_FLRC;
    ctx->radio_params.flrc.is_tx = is_tx;
    ctx->radio_params.flrc.tx_size = tx_size;
    ctx->radio_params.flrc.max_rx_size = APP_FLRC_MAX_PAYLOAD_BYTES;
    ctx->radio_params.flrc.frequency_in_hz = APP_FLRC_FREQUENCY_HZ;
    ctx->radio_params.flrc.tx_power_in_dbm = APP_FLRC_TX_POWER_DBM;
    ctx->radio_params.flrc.br_in_bps = APP_FLRC_BITRATE_BPS;
    ctx->radio_params.flrc.bw_dsb_in_hz = APP_FLRC_BANDWIDTH_HZ;
    ctx->radio_params.flrc.cr = APP_FLRC_CODING_RATE;
    ctx->radio_params.flrc.pulse_shape = APP_FLRC_PULSE_SHAPE;
    ctx->radio_params.flrc.preamble_len_in_bits = 32;
    ctx->radio_params.flrc.sync_word_len = RAL_FLRC_SYNCWORD_LENGTH_4_BYTES;
    ctx->radio_params.flrc.tx_syncword = RAL_FLRC_TX_SYNCWORD_1;
    ctx->radio_params.flrc.match_sync_word = RAL_FLRC_RX_MATCH_SYNCWORD_1;
    ctx->radio_params.flrc.pld_is_fix = false;
    ctx->radio_params.flrc.crc_type = RAL_FLRC_CRC_2_BYTES;
    ctx->radio_params.flrc.sync_word = kSyncWord;
    ctx->radio_params.flrc.crc_seed = 0xFFFFFFFFUL;
    ctx->radio_params.flrc.crc_polynomial = 0x04C11DB7UL;
    ctx->radio_params.flrc.rx_timeout_ms = APP_FLRC_RX_TIMEOUT_MS;

    ctx->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms();
    ctx->scheduler_config.scheduling = SMTC_RAC_ASAP_TRANSACTION;
    ctx->scheduler_config.callback_pre_radio_transaction = nullptr;
    ctx->scheduler_config.callback_post_radio_transaction = &RadioPing::post_callback;
}

bool RadioPing::build_voice_packet(uint16_t *tx_size)
{
    TxPacket packet;
    if (tx_queue_ == nullptr || xQueueReceive(tx_queue_, &packet, 0) != pdTRUE) {
        return false;
    }

    std::memcpy(tx_buf_, packet.payload, packet.len);
    *tx_size = packet.len;
    return true;
}

void RadioPing::capture_voice_packet()
{
    if (tx_queue_ == nullptr || !read_mono_frame(tx_pcm_, APP_AUDIO_FRAME_SAMPLES)) {
        return;
    }
    if (!ptt_active_) {
        return;
    }

    TxPacket packet = {
        .len = 0,
        .payload = {},
    };

    std::memcpy(packet.payload, kMagic, sizeof(kMagic));
    packet.payload[4] = kPacketTypeVoice;
    packet.payload[5] = 1;
    put_u16_le(&packet.payload[6], tx_seq_++);
    put_u32_le(&packet.payload[8], smtc_modem_hal_get_time_in_ms());
    packet.payload[12] = 0;
    packet.payload[13] = 0;

    int encoded = codec_.encode(tx_pcm_, APP_AUDIO_FRAME_SAMPLES,
                                &packet.payload[kHeaderSize],
                                APP_FLRC_MAX_PAYLOAD_BYTES - kHeaderSize);
    if (encoded <= 0) {
        ESP_LOGW(TAG, "Opus encode failed: %d", encoded);
        return;
    }
    if (encoded > 255) {
        ESP_LOGW(TAG, "Opus packet too large: %d", encoded);
        return;
    }

    packet.payload[12] = static_cast<uint8_t>(encoded);
    packet.len = static_cast<uint16_t>(kHeaderSize + encoded);

    if (xQueueSend(tx_queue_, &packet, 0) != pdTRUE) {
        TxPacket dropped;
        (void)xQueueReceive(tx_queue_, &dropped, 0);
        tx_queue_drops_++;
        if (xQueueSend(tx_queue_, &packet, 0) == pdTRUE) {
            ESP_LOGW(TAG, "voice TX queue full, dropped oldest drops=%lu",
                     static_cast<unsigned long>(tx_queue_drops_));
        } else {
            ESP_LOGW(TAG, "voice TX queue full drops=%lu",
                     static_cast<unsigned long>(tx_queue_drops_));
        }
    }
}

void RadioPing::handle_rx_packet()
{
    smtc_rac_context_t *ctx = smtc_rac_get_context(radio_id_);
    uint16_t len = ctx->smtc_rac_data_result.rx_size;
    int16_t rssi = ctx->smtc_rac_data_result.rssi_result;

    if (len < kHeaderSize || std::memcmp(rx_buf_, kMagic, sizeof(kMagic)) != 0) {
        ESP_LOGW(TAG, "RX unknown packet len=%u rssi=%d", len, rssi);
        return;
    }

    if (rx_buf_[4] == kPacketTypeVoice) {
        queue_voice_packet(len, rssi);
    } else if (rx_buf_[4] == kPacketTypePing) {
        uint16_t seq = get_u16_le(&rx_buf_[6]);
        log_rx(seq, len, rssi);
    } else {
        ESP_LOGW(TAG, "RX unsupported packet type=%u len=%u rssi=%d", rx_buf_[4], len, rssi);
    }
}

void RadioPing::queue_voice_packet(uint16_t len, int16_t rssi)
{
    uint8_t opus_len = rx_buf_[12];
    if (opus_len == 0 || opus_len > APP_OPUS_MAX_PACKET_BYTES || kHeaderSize + opus_len > len) {
        ESP_LOGW(TAG, "RX bad voice packet len=%u opus_len=%u", len, opus_len);
        return;
    }

    uint16_t seq = get_u16_le(&rx_buf_[6]);
    log_rx(seq, len, rssi);

    VoicePacket packet = {
        .seq = seq,
        .len = opus_len,
        .rssi = rssi,
        .payload = {},
    };
    std::memcpy(packet.payload, &rx_buf_[kHeaderSize], opus_len);

    if (xQueueSend(voice_queue_, &packet, 0) != pdTRUE) {
        VoicePacket dropped;
        (void)xQueueReceive(voice_queue_, &dropped, 0);
        rx_queue_drops_++;
        if (xQueueSend(voice_queue_, &packet, 0) == pdTRUE) {
            ESP_LOGW(TAG, "voice queue full, dropped oldest seq=%u drops=%lu",
                     dropped.seq, static_cast<unsigned long>(rx_queue_drops_));
        } else {
            ESP_LOGW(TAG, "voice queue full drops=%lu seq=%u",
                     static_cast<unsigned long>(rx_queue_drops_), seq);
        }
    }
}

void RadioPing::log_rx(uint16_t seq, uint16_t len, int16_t rssi)
{
    if (!have_expected_rx_seq_) {
        expected_rx_seq_ = static_cast<uint16_t>(seq + 1);
        have_expected_rx_seq_ = true;
    } else if (seq != expected_rx_seq_) {
        uint16_t gap = static_cast<uint16_t>(seq - expected_rx_seq_);
        if (gap < 0x8000) {
            rx_lost_ += gap;
            ESP_LOGW(TAG, "FLRC loss gap=%u expected=%u got=%u total_lost=%lu rssi=%d dBm",
                     gap, expected_rx_seq_, seq, static_cast<unsigned long>(rx_lost_), rssi);
        }
        expected_rx_seq_ = static_cast<uint16_t>(seq + 1);
    } else {
        expected_rx_seq_ = static_cast<uint16_t>(expected_rx_seq_ + 1);
    }

    rx_packets_++;
}

void RadioPing::wait_for_jitter_buffer()
{
    if (playback_active_ || voice_queue_ == nullptr || APP_RX_JITTER_FRAMES <= 1U) {
        return;
    }

    const UBaseType_t target_waiting = static_cast<UBaseType_t>(APP_RX_JITTER_FRAMES - 1U);
    const uint32_t start_ms = smtc_modem_hal_get_time_in_ms();
    while (uxQueueMessagesWaiting(voice_queue_) < target_waiting) {
        if (smtc_modem_hal_get_time_in_ms() - start_ms >= APP_RX_JITTER_BUFFER_MS) {
            break;
        }
        vTaskDelay(ms_to_ticks_min_1(1));
    }
}

void RadioPing::conceal_missing_frames(uint16_t seq)
{
    if (!have_expected_play_seq_) {
        expected_play_seq_ = seq;
        have_expected_play_seq_ = true;
    }

    uint16_t gap = static_cast<uint16_t>(seq - expected_play_seq_);
    if (gap > 0 && gap <= APP_RX_MAX_PLC_FRAMES) {
        for (uint16_t i = 0; i < gap; i++) {
            int decoded = codec_.decode_lost(rx_pcm_, APP_AUDIO_FRAME_SAMPLES);
            if (decoded <= 0) {
                ESP_LOGW(TAG, "Opus PLC failed: %d", decoded);
                break;
            }
            play_mono_frame(rx_pcm_, static_cast<size_t>(decoded));
            last_rx_audio_ms_ = smtc_modem_hal_get_time_in_ms();
            playback_active_ = true;
        }
    }

    expected_play_seq_ = static_cast<uint16_t>(seq + 1);
}

bool RadioPing::read_mono_frame(int16_t *mono, size_t samples)
{
    int16_t stereo[APP_AUDIO_FRAME_SAMPLES * 2];
    size_t got_total = 0;
    const size_t target = samples * 2 * sizeof(int16_t);

    while (got_total < target) {
        size_t got = 0;
        esp_err_t err = bsp_audio_read(reinterpret_cast<uint8_t *>(stereo) + got_total,
                                       target - got_total, &got);
        if (err != ESP_OK || got == 0) {
            ESP_LOGW(TAG, "audio read failed: %s got=%u",
                     esp_err_to_name(err), static_cast<unsigned>(got));
            return false;
        }
        got_total += got;
    }

    for (size_t i = 0; i < samples; i++) {
        int16_t left = stereo[2 * i];
        int16_t right = stereo[2 * i + 1];
        mono[i] = (abs16(left) >= abs16(right)) ? left : right;
    }
    return true;
}

void RadioPing::play_mono_frame(const int16_t *mono, size_t samples)
{
    int16_t stereo[APP_AUDIO_FRAME_SAMPLES * 2];
    if (samples > APP_AUDIO_FRAME_SAMPLES) {
        samples = APP_AUDIO_FRAME_SAMPLES;
    }

    for (size_t i = 0; i < samples; i++) {
        stereo[2 * i] = mono[i];
        stereo[2 * i + 1] = mono[i];
    }

    set_playback_pa(true);
    size_t written = 0;
    esp_err_t err = bsp_audio_write(stereo, samples * 2 * sizeof(int16_t), &written);
    if (err != ESP_OK || written == 0) {
        ESP_LOGW(TAG, "audio write failed: %s written=%u",
                 esp_err_to_name(err), static_cast<unsigned>(written));
    }
}

void RadioPing::set_playback_pa(bool on)
{
    if (playback_pa_on_ == on) {
        return;
    }
    esp_err_t err = bsp_audio_pa_enable(on);
    if (err == ESP_OK) {
        playback_pa_on_ = on;
    } else {
        ESP_LOGW(TAG, "PA %s failed: %s", on ? "enable" : "disable", esp_err_to_name(err));
    }
}

void RadioPing::update_playback_timeout()
{
    if (!playback_active_) return;
    uint32_t now = smtc_modem_hal_get_time_in_ms();
    if (now - last_rx_audio_ms_ > APP_RX_AUDIO_TIMEOUT_MS) {
        set_playback_pa(false);
        playback_active_ = false;
        have_expected_play_seq_ = false;
    }
}
