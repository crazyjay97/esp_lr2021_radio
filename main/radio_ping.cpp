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

    esp_err_t err = codec_.init();
    if (err != ESP_OK) {
        return err;
    }

    voice_queue_ = xQueueCreate(APP_VOICE_RX_QUEUE_LEN, sizeof(VoicePacket));
    if (voice_queue_ == nullptr) {
        ESP_LOGE(TAG, "voice queue alloc failed");
        return ESP_ERR_NO_MEM;
    }
    tx_queue_ = xQueueCreate(APP_VOICE_TX_QUEUE_LEN, sizeof(TxFrame));
    if (tx_queue_ == nullptr) {
        ESP_LOGE(TAG, "tx queue alloc failed");
        return ESP_ERR_NO_MEM;
    }

    smtc_modem_hal_protect_api_call();
    ral_status_t status = ral_reset(&radio_.ral);
    if (status == RAL_STATUS_OK) status = ral_init(&radio_.ral);
    if (status == RAL_STATUS_OK) {
        status = ral_set_rx_tx_fallback_mode(&radio_.ral, RAL_FALLBACK_STDBY_XOSC);
    }
    if (status == RAL_STATUS_OK) status = ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
    if (status == RAL_STATUS_OK && !configure_flrc()) status = RAL_STATUS_ERROR;
    if (status == RAL_STATUS_OK) {
        status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    }
    smtc_modem_hal_irq_config_radio_irq(&RadioPing::irq_callback, this);
    smtc_modem_hal_unprotect_api_call();

    if (status != RAL_STATUS_OK) {
        ESP_LOGE(TAG, "direct RAL init failed: %d", status);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LR2021 direct RAL initialized: FLRC rf=%lu Hz br=%lu bps bw=%lu Hz",
             APP_FLRC_FREQUENCY_HZ, APP_FLRC_BITRATE_BPS, APP_FLRC_BANDWIDTH_HZ);
    schedule_rx();
    return ESP_OK;
}

esp_err_t RadioPing::start()
{
    BaseType_t ok = xTaskCreatePinnedToCore(task_trampoline, "radio_ping",
                                            APP_RADIO_TASK_STACK_BYTES, this,
                                            APP_RADIO_TASK_PRIORITY, nullptr,
                                            APP_RADIO_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(tx_task_trampoline, "voice_tx",
                                 APP_VOICE_TX_TASK_STACK_BYTES, this,
                                 APP_VOICE_TX_TASK_PRIORITY, nullptr,
                                 APP_VOICE_TX_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(play_task_trampoline, "voice_play",
                                 APP_VOICE_PLAY_TASK_STACK_BYTES, this,
                                 APP_VOICE_PLAY_TASK_PRIORITY, nullptr,
                                 APP_VOICE_PLAY_TASK_CORE);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void RadioPing::handle_button(bsp_btn_id_t id, bool pressed)
{
    if (id != APP_PTT_BUTTON) return;

    ptt_active_ = pressed;
    ESP_LOGI(TAG, "PTT %s -> FLRC voice %s", pressed ? "down" : "up",
             pressed ? "TX" : "RX");

    if (pressed) {
        bool new_burst = !tx_burst_active_;
        tx_burst_active_ = true;
        tx_flush_pending_ = false;

        if (new_burst) {
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
        }

        if (mode_ == Mode::rx_pending) {
            smtc_modem_hal_protect_api_call();
            (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            smtc_modem_hal_unprotect_api_call();
            mode_ = Mode::idle;
        }
    } else {
        tx_flush_pending_ = tx_burst_active_;
        if (mode_ == Mode::idle) {
            schedule_tx();
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
    if (irq_pending_) {
        irq_pending_ = false;
        ral_irq_t irq = RAL_IRQ_NONE;
        smtc_modem_hal_protect_api_call();
        ral_status_t status = ral_get_and_clear_irq_status(&radio_.ral, &irq);
        smtc_modem_hal_unprotect_api_call();
        if (status == RAL_STATUS_OK && irq != RAL_IRQ_NONE) {
            handle_irq(irq);
        }
    }

    if (mode_ == Mode::idle) {
        if (tx_burst_active_) {
            schedule_tx();
            if (!tx_burst_active_ && !ptt_active_ && mode_ == Mode::idle) {
                schedule_rx();
            }
        } else if (!ptt_active_) {
            schedule_rx();
        }
    }
    taskYIELD();
}

void RadioPing::irq_callback(void *context)
{
    auto *self = static_cast<RadioPing *>(context);
    if (self != nullptr) {
        self->irq_pending_ = true;
    }
}

void RadioPing::handle_irq(ral_irq_t irq)
{
    Mode completed_mode = mode_;
    mode_ = Mode::idle;

    if (completed_mode == Mode::rx_pending) {
        if ((irq & RAL_IRQ_RX_DONE) != 0) {
            handle_rx_packet();
        } else if ((irq & RAL_IRQ_RX_CRC_ERROR) != 0) {
            rx_crc_errors_++;
            if ((rx_crc_errors_ % 10) == 1) {
                ESP_LOGW(TAG, "RX CRC errors=%lu", static_cast<unsigned long>(rx_crc_errors_));
            }
        } else if ((irq & RAL_IRQ_RX_HDR_ERROR) != 0) {
            ESP_LOGW(TAG, "RX header error");
        } else if ((irq & RAL_IRQ_RX_TIMEOUT) == 0) {
            ESP_LOGW(TAG, "RX irq=0x%08lx", static_cast<unsigned long>(irq));
        }
    } else if (completed_mode == Mode::tx_pending) {
        if ((irq & RAL_IRQ_TX_DONE) == 0) {
            ESP_LOGW(TAG, "TX irq=0x%08lx", static_cast<unsigned long>(irq));
        }
        if (APP_FLRC_VOICE_TX_GAP_MS > 0) {
            vTaskDelay(ms_to_ticks_min_1(APP_FLRC_VOICE_TX_GAP_MS));
        }
    }
}

void RadioPing::schedule_rx()
{
    if (mode_ != Mode::idle) return;

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(false);
    ral_status_t status = ral_set_dio_irq_params(&radio_.ral,
                                                 RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT |
                                                 RAL_IRQ_RX_HDR_ERROR | RAL_IRQ_RX_CRC_ERROR);
    if (status == RAL_STATUS_OK) status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    if (status == RAL_STATUS_OK) status = ral_set_rx(&radio_.ral, APP_FLRC_RX_TIMEOUT_MS);
    smtc_modem_hal_unprotect_api_call();

    if (status == RAL_STATUS_OK) {
        mode_ = Mode::rx_pending;
    } else {
        ESP_LOGW(TAG, "schedule RX failed: %d", status);
    }
}

void RadioPing::schedule_tx()
{
    if (mode_ != Mode::idle) return;
    if (tx_queue_ == nullptr) return;

    UBaseType_t queued = uxQueueMessagesWaiting(tx_queue_);
    if (queued == 0) {
        if (!ptt_active_) {
            tx_flush_pending_ = false;
            tx_burst_active_ = false;
        }
        return;
    }
    if (ptt_active_ && queued < APP_FLRC_OPUS_FRAMES_PER_PACKET) {
        return;
    }

    uint16_t tx_size = 0;
    if (!build_voice_packet(&tx_size)) {
        if (!ptt_active_ && uxQueueMessagesWaiting(tx_queue_) == 0) {
            tx_flush_pending_ = false;
            tx_burst_active_ = false;
        }
        return;
    }

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);
    ral_status_t status = ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    if (status == RAL_STATUS_OK) status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    if (status == RAL_STATUS_OK) status = ral_set_pkt_payload(&radio_.ral, tx_buf_, tx_size);
    if (status == RAL_STATUS_OK) status = ral_set_tx(&radio_.ral);
    smtc_modem_hal_unprotect_api_call();

    if (status == RAL_STATUS_OK) {
        mode_ = Mode::tx_pending;
        if (!ptt_active_ && uxQueueMessagesWaiting(tx_queue_) == 0) {
            tx_flush_pending_ = false;
        }
    } else {
        ESP_LOGW(TAG, "schedule TX failed: %d", status);
    }
}

bool RadioPing::configure_flrc()
{
    ralf_params_flrc_t params = {};
    params.rf_freq_in_hz = APP_FLRC_FREQUENCY_HZ;
    params.output_pwr_in_dbm = APP_FLRC_TX_POWER_DBM;
    params.mod_params.br_in_bps = APP_FLRC_BITRATE_BPS;
    params.mod_params.bw_dsb_in_hz = APP_FLRC_BANDWIDTH_HZ;
    params.mod_params.cr = APP_FLRC_CODING_RATE;
    params.mod_params.pulse_shape = APP_FLRC_PULSE_SHAPE;
    params.pkt_params.preamble_len_in_bits = 32;
    params.pkt_params.sync_word_len = RAL_FLRC_SYNCWORD_LENGTH_4_BYTES;
    params.pkt_params.tx_syncword = RAL_FLRC_TX_SYNCWORD_1;
    params.pkt_params.match_sync_word = RAL_FLRC_RX_MATCH_SYNCWORD_1;
    params.pkt_params.pld_is_fix = false;
    params.pkt_params.pld_len_in_bytes = APP_FLRC_MAX_PAYLOAD_BYTES;
    params.pkt_params.crc_type = RAL_FLRC_CRC_2_BYTES;
    params.sync_word = kSyncWord;
    params.crc_seed = 0xFFFFFFFFUL;
    params.crc_polynomial = 0x04C11DB7UL;
    return ralf_setup_flrc(&radio_, &params) == RAL_STATUS_OK;
}

bool RadioPing::build_voice_packet(uint16_t *tx_size)
{
    TxFrame frame;
    if (tx_queue_ == nullptr || xQueueReceive(tx_queue_, &frame, 0) != pdTRUE) {
        return false;
    }

    std::memcpy(tx_buf_, kMagic, sizeof(kMagic));
    tx_buf_[4] = kPacketTypeVoice;
    tx_buf_[5] = 2;
    put_u16_le(&tx_buf_[6], frame.seq);
    put_u32_le(&tx_buf_[8], smtc_modem_hal_get_time_in_ms());
    tx_buf_[12] = 0;
    tx_buf_[13] = 0;

    uint16_t offset = kHeaderSize;
    uint8_t frame_count = 0;
    while (frame_count < APP_FLRC_OPUS_FRAMES_PER_PACKET) {
        if (frame.len == 0 || frame.len > APP_OPUS_MAX_PACKET_BYTES ||
            offset + 1U + frame.len > APP_FLRC_MAX_PAYLOAD_BYTES) {
            break;
        }

        tx_buf_[offset++] = static_cast<uint8_t>(frame.len);
        std::memcpy(&tx_buf_[offset], frame.payload, frame.len);
        offset = static_cast<uint16_t>(offset + frame.len);
        frame_count++;

        if (frame_count >= APP_FLRC_OPUS_FRAMES_PER_PACKET ||
            xQueueReceive(tx_queue_, &frame, 0) != pdTRUE) {
            break;
        }
    }

    if (frame_count == 0) {
        return false;
    }

    tx_buf_[12] = frame_count;
    *tx_size = offset;
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

    TxFrame frame = {
        .seq = tx_seq_++,
        .len = 0,
        .payload = {},
    };

    int encoded = codec_.encode(tx_pcm_, APP_AUDIO_FRAME_SAMPLES,
                                frame.payload,
                                APP_OPUS_MAX_PACKET_BYTES);
    if (encoded <= 0) {
        ESP_LOGW(TAG, "Opus encode failed: %d", encoded);
        return;
    }
    if (encoded > 255) {
        ESP_LOGW(TAG, "Opus packet too large: %d", encoded);
        return;
    }

    frame.len = static_cast<uint16_t>(encoded);

    if (xQueueSend(tx_queue_, &frame, 0) != pdTRUE) {
        TxFrame dropped;
        (void)xQueueReceive(tx_queue_, &dropped, 0);
        tx_queue_drops_++;
        if (xQueueSend(tx_queue_, &frame, 0) == pdTRUE) {
            if ((tx_queue_drops_ % APP_TX_DROP_LOG_EVERY_N) == 1) {
                ESP_LOGW(TAG, "voice TX queue full, dropped oldest seq=%u drops=%lu",
                         dropped.seq, static_cast<unsigned long>(tx_queue_drops_));
            }
        } else {
            if ((tx_queue_drops_ % APP_TX_DROP_LOG_EVERY_N) == 1) {
                ESP_LOGW(TAG, "voice TX queue full drops=%lu",
                         static_cast<unsigned long>(tx_queue_drops_));
            }
        }
    }
}

void RadioPing::handle_rx_packet()
{
    uint16_t len = 0;
    ral_flrc_rx_pkt_status_t pkt_status = {};

    smtc_modem_hal_protect_api_call();
    ral_status_t status = ral_get_pkt_payload(&radio_.ral, sizeof(rx_buf_), rx_buf_, &len);
    if (status == RAL_STATUS_OK) {
        status = ral_get_flrc_rx_pkt_status(&radio_.ral, &pkt_status);
    }
    smtc_modem_hal_unprotect_api_call();

    if (status != RAL_STATUS_OK) {
        ESP_LOGW(TAG, "RX read failed: %d", status);
        return;
    }

    int16_t rssi = pkt_status.rssi_sync_in_dbm;

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
    uint8_t frame_count = rx_buf_[12];
    if (frame_count == 0 || frame_count > APP_FLRC_OPUS_FRAMES_PER_PACKET) {
        ESP_LOGW(TAG, "RX bad voice packet len=%u frames=%u", len, frame_count);
        return;
    }

    uint16_t seq = get_u16_le(&rx_buf_[6]);
    uint16_t offset = kHeaderSize;
    for (uint8_t i = 0; i < frame_count; i++) {
        if (offset >= len) {
            ESP_LOGW(TAG, "RX truncated voice packet len=%u frames=%u", len, frame_count);
            return;
        }

        uint8_t opus_len = rx_buf_[offset++];
        if (opus_len == 0 || opus_len > APP_OPUS_MAX_PACKET_BYTES || offset + opus_len > len) {
            ESP_LOGW(TAG, "RX bad voice frame len=%u opus_len=%u", len, opus_len);
            return;
        }

        uint16_t frame_seq = static_cast<uint16_t>(seq + i);
        log_rx(frame_seq, len, rssi);

        VoicePacket packet = {
            .seq = frame_seq,
            .len = opus_len,
            .rssi = rssi,
            .payload = {},
        };
        std::memcpy(packet.payload, &rx_buf_[offset], opus_len);
        offset = static_cast<uint16_t>(offset + opus_len);

        if (xQueueSend(voice_queue_, &packet, 0) != pdTRUE) {
            VoicePacket dropped;
            (void)xQueueReceive(voice_queue_, &dropped, 0);
            rx_queue_drops_++;
            if (xQueueSend(voice_queue_, &packet, 0) == pdTRUE) {
                ESP_LOGW(TAG, "voice queue full, dropped oldest seq=%u drops=%lu",
                         dropped.seq, static_cast<unsigned long>(rx_queue_drops_));
            } else {
                ESP_LOGW(TAG, "voice queue full drops=%lu seq=%u",
                         static_cast<unsigned long>(rx_queue_drops_), frame_seq);
            }
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
