#include "radio_ping.hpp"

#include <cstring>

#include "esp_log.h"
#include "esp_rom_sys.h"
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
constexpr uint8_t kPacketTypeImageCmd = 3;
constexpr uint8_t kPacketTypeImageData = 4;
constexpr uint8_t kPacketTypeImageNack = 5;
constexpr uint8_t kPacketTypeImageDone = 6;
constexpr uint8_t kPacketTypeImageEOT = 7;
constexpr uint8_t kPacketTypeImageStart = 8;
constexpr uint8_t kPacketTypeConfig = 9;
constexpr uint8_t kPacketTypeConfigAck = 10;
constexpr uint16_t kHeaderSize = 14;
constexpr uint32_t kConfigAckTimeoutMs = 500;

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

uint32_t get_u32_le(const uint8_t *p)
{
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

TickType_t ms_to_ticks_min_1(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    return ticks == 0 ? 1 : ticks;
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

uint32_t crc32_ieee(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1U) ? (crc >> 1) ^ 0xEDB88320U : (crc >> 1);
        }
    }
    return ~crc;
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
    image_tx_queue_ = xQueueCreate(1, sizeof(ImageTxRequest));
    if (image_tx_queue_ == nullptr) {
        ESP_LOGE(TAG, "image tx queue alloc failed");
        return ESP_ERR_NO_MEM;
    }

    if (!audio_ringbuf_.init(APP_AUDIO_RINGBUF_SAMPLES)) {
        ESP_LOGW(TAG, "audio ring buffer alloc failed (PSRAM?)");
    }

#if !APP_RADIO_HW_INIT_ENABLE
    ESP_LOGW(TAG, "LR2021 hardware init disabled for camera isolation");
    return ESP_OK;
#endif

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
#if APP_RADIO_AUTO_RX_ENABLE
    schedule_rx();
#else
    ESP_LOGW(TAG, "LR2021 auto RX disabled for camera isolation");
#endif
    return ESP_OK;
}

esp_err_t RadioPing::start()
{
    BaseType_t ok = xTaskCreatePinnedToCore(task_trampoline, "radio_ping",
                                            APP_RADIO_TASK_STACK_BYTES, this,
                                            APP_RADIO_TASK_PRIORITY, &task_handle_,
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
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(image_tx_task_trampoline, "img_tx",
                                 APP_IMAGE_TASK_STACK_BYTES, this,
                                 APP_IMAGE_TASK_PRIORITY, nullptr,
                                 APP_IMAGE_TASK_CORE);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void RadioPing::handle_button(bsp_btn_id_t id, bool pressed)
{
    if (id != APP_PTT_BUTTON) return;
    if (suspended_) return;

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
            audio_proc_.reset();
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

void RadioPing::suspend()
{
    suspended_ = true;
    ptt_active_ = false;
    tx_burst_active_ = false;
    tx_flush_pending_ = false;
    irq_pending_ = false;

    if (tx_queue_ != nullptr) {
        xQueueReset(tx_queue_);
    }
    if (voice_queue_ != nullptr) {
        xQueueReset(voice_queue_);
    }

    set_playback_pa(false);
    playback_active_ = false;
    have_expected_play_seq_ = false;

    smtc_modem_hal_protect_api_call();
    (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
    (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    smtc_modem_hal_unprotect_api_call();

    mode_ = Mode::idle;
    ESP_LOGI(TAG, "radio suspended");
}

void RadioPing::resume()
{
    smtc_modem_hal_protect_api_call();
    (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    smtc_modem_hal_unprotect_api_call();

    mode_ = Mode::idle;
    suspended_ = false;
    ESP_LOGI(TAG, "radio resumed");
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
        if (!suspended_) {
            poll_once();
            update_playback_timeout();
            check_image_rx_timeout();
        }
        ulTaskNotifyTake(pdTRUE, ms_to_ticks_min_1(APP_RADIO_TASK_POLL_MS));
    }
}

void RadioPing::tx_task()
{
    while (true) {
        if (suspended_ || image_tx_active_) {
            vTaskDelay(ms_to_ticks_min_1(APP_AUDIO_FRAME_MS));
            continue;
        }

        if (!read_mono_frame(tx_pcm_, APP_AUDIO_FRAME_SAMPLES)) {
            vTaskDelay(ms_to_ticks_min_1(APP_AUDIO_FRAME_MS));
            continue;
        }

        audio_ringbuf_.write(tx_pcm_, APP_AUDIO_FRAME_SAMPLES);
    }
}

void RadioPing::play_task()
{
    VoicePacket packet;

    while (true) {
        if (xQueueReceive(voice_queue_, &packet, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (suspended_) {
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

        audio_proc_.process_rx_frame(rx_pcm_, static_cast<size_t>(decoded));
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
        if (self->task_handle_ != nullptr) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(self->task_handle_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void RadioPing::handle_irq(ral_irq_t irq)
{
    Mode completed_mode = mode_;

    if (completed_mode == Mode::rx_pending) {
        if ((irq & RAL_IRQ_RX_DONE) != 0) {
            mode_ = Mode::idle;
            handle_rx_packet();
            if (mode_ == Mode::idle && !ptt_active_ && !tx_burst_active_) {
                schedule_rx();
            }
        } else if ((irq & RAL_IRQ_RX_CRC_ERROR) != 0) {
            mode_ = Mode::idle;
            rx_crc_errors_++;
            if ((rx_crc_errors_ % 10) == 1) {
                ESP_LOGW(TAG, "RX CRC errors=%lu", static_cast<unsigned long>(rx_crc_errors_));
            }
            schedule_rx();
        } else if ((irq & RAL_IRQ_RX_HDR_ERROR) != 0) {
            mode_ = Mode::idle;
            ESP_LOGW(TAG, "RX header error");
            schedule_rx();
        } else if ((irq & RAL_IRQ_RX_TIMEOUT) != 0) {
            mode_ = Mode::idle;
        } else {
            // Non-terminal IRQ (e.g. FIFO_LEVEL, PREAMBLE_DETECTED)
            // Do NOT reset mode or re-arm — packet still being received
        }
    } else if (completed_mode == Mode::tx_pending) {
        mode_ = Mode::idle;
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

    uint32_t rx_timeout = APP_FLRC_RX_TIMEOUT_MS;
    if (image_rx_pending_) {
        rx_timeout = RAL_RX_TIMEOUT_CONTINUOUS_MODE;
    }

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(false);
    ral_status_t status = ral_set_dio_irq_params(&radio_.ral,
                                                 RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT |
                                                 RAL_IRQ_RX_HDR_ERROR | RAL_IRQ_RX_CRC_ERROR);
    if (status == RAL_STATUS_OK) status = ral_set_rx(&radio_.ral, rx_timeout);
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
    if (tx_queue_ == nullptr) {
        return;
    }

    audio_proc_.process_tx_frame(tx_pcm_, APP_AUDIO_FRAME_SAMPLES);

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
        rx_unknown_packets_++;
        if ((rx_unknown_packets_ % 50U) == 1U) {
            ESP_LOGW(TAG, "RX unknown packets=%lu len=%u rssi=%d hdr=%02x%02x%02x%02x",
                     static_cast<unsigned long>(rx_unknown_packets_), len, rssi,
                     rx_buf_[0], rx_buf_[1], rx_buf_[2], rx_buf_[3]);
        }
        return;
    }

    if (rx_buf_[4] == kPacketTypeVoice) {
        queue_voice_packet(len, rssi);
    } else if (rx_buf_[4] == kPacketTypePing) {
        uint16_t seq = get_u16_le(&rx_buf_[6]);
        log_rx(seq, len, rssi);
    } else if (rx_buf_[4] == kPacketTypeImageCmd) {
        handle_image_cmd();
    } else if (rx_buf_[4] == kPacketTypeImageData) {
        image_rx_last_rssi_ = rssi;
        schedule_rx();
        handle_image_data();
    } else if (rx_buf_[4] == kPacketTypeImageNack) {
        handle_image_nack();
    } else if (rx_buf_[4] == kPacketTypeImageDone) {
        handle_image_done();
    } else if (rx_buf_[4] == kPacketTypeImageEOT) {
        handle_image_eot();
    } else if (rx_buf_[4] == kPacketTypeImageStart) {
        handle_image_start();
    } else if (rx_buf_[4] == kPacketTypeConfig) {
        uint8_t key = rx_buf_[8];
        uint32_t value = get_u32_le(&rx_buf_[9]);
        ESP_LOGI(TAG, "RX Config: key=%u value=%lu", key, static_cast<unsigned long>(value));
        if (config_received_cb_) {
            config_received_cb_(key, value);
        }
        send_config_ack(key, value);
    } else if (rx_buf_[4] == kPacketTypeConfigAck) {
        ESP_LOGI(TAG, "RX ConfigAck");
        config_ack_received_ = true;
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

// --- Image transfer implementation ---

void RadioPing::image_tx_task_trampoline(void *arg)
{
    static_cast<RadioPing *>(arg)->image_tx_task();
}

void RadioPing::trigger_image_capture()
{
    if (image_tx_active_) {
        ESP_LOGW(TAG, "image TX already active, ignoring trigger");
        return;
    }

    uint16_t session_id = image_session_id_++;
    if (image_session_id_ == 0) {
        image_session_id_ = 1;
    }

    // Build ImageCmd packet
    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeImageCmd;
    pkt[5] = 1;
    put_u16_le(&pkt[6], session_id);
    put_u32_le(&pkt[8], smtc_modem_hal_get_time_in_ms());
    pkt[12] = 0;
    pkt[13] = 0;

    // ESP_LOGI(TAG, "trigger_image_capture: sending ImageCmd session=%u", session_id);
    image_cmd_sent_ms_ = smtc_modem_hal_get_time_in_ms();

    // Stop RX, send cmd, return to RX
    smtc_modem_hal_protect_api_call();
    if (mode_ == Mode::rx_pending) {
        (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        mode_ = Mode::idle;
    }
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);
    ral_status_t status = ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    if (status == RAL_STATUS_OK) status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    if (status == RAL_STATUS_OK) status = ral_set_pkt_payload(&radio_.ral, pkt, kHeaderSize);
    if (status == RAL_STATUS_OK) status = ral_set_tx(&radio_.ral);
    smtc_modem_hal_unprotect_api_call();

    if (status == RAL_STATUS_OK) {
        mode_ = Mode::tx_pending;
    } else {
        ESP_LOGW(TAG, "trigger_image_capture TX failed: %d", status);
    }
}

void RadioPing::send_image(const uint8_t *jpeg, size_t jpeg_len, uint16_t session_id)
{
    if (!image_tx_queue_) {
        ESP_LOGE(TAG, "image_tx_queue not initialized");
        return;
    }
    ImageTxRequest req = { .jpeg = jpeg, .jpeg_len = jpeg_len, .session_id = session_id };
    if (xQueueSend(image_tx_queue_, &req, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "image_tx_queue full");
    }
}

/*
 * Image transfer protocol (T=transmitter/camera, R=receiver/controller):
 *
 * 1. T sends all ImageData fragments continuously, R does not reply
 * 2. T sends ImageEOT, R must reply; if T gets no reply, T resends EOT
 * 3. R replies with ImageACK containing list of missing fragment indices
 * 4. If missing list is empty → transfer complete
 * 5. If missing list has entries → T resends those fragments
 * 6. T sends missing fragments continuously, R does not reply
 * 7. T sends ImageEOT again, R must reply
 * 8. Repeat until complete or retry limit reached
 */
void RadioPing::image_tx_task()
{
    ImageTxRequest req;
    while (true) {
        if (xQueueReceive(image_tx_queue_, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        image_tx_active_ = true;
        image_done_received_ = false;
        image_nack_received_ = false;
        suspended_ = true;

        vTaskDelay(pdMS_TO_TICKS(APP_RADIO_TASK_POLL_MS * 2));

        smtc_modem_hal_protect_api_call();
        if (!configure_flrc()) {
            ESP_LOGE(TAG, "image TX: configure_flrc failed");
        }
        smtc_modem_hal_unprotect_api_call();

        uint16_t total_fragments = static_cast<uint16_t>(
            (req.jpeg_len + APP_IMAGE_FRAGMENT_DATA_SIZE - 1) / APP_IMAGE_FRAGMENT_DATA_SIZE);
        uint32_t jpeg_crc32 = crc32_ieee(req.jpeg, req.jpeg_len);

        // ESP_LOGI(TAG, "image TX start: session=%u jpeg=%u bytes frags=%u crc32=0x%08lx",
        //          req.session_id, static_cast<unsigned>(req.jpeg_len), total_fragments,
        //          static_cast<unsigned long>(jpeg_crc32));

        bool was_ptt = ptt_active_;
        ptt_active_ = false;
        tx_burst_active_ = false;
        if (mode_ == Mode::rx_pending) {
            smtc_modem_hal_protect_api_call();
            (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            smtc_modem_hal_unprotect_api_call();
            mode_ = Mode::idle;
        }

        // Step 0: Send ImageStart and wait for R to confirm ready
        bool r_ready = false;
        for (uint16_t start_try = 0; start_try < APP_IMAGE_EOT_RETRY_COUNT && !r_ready; start_try++) {
            uint8_t start_pkt[kHeaderSize];
            std::memcpy(start_pkt, kMagic, sizeof(kMagic));
            start_pkt[4] = kPacketTypeImageStart;
            start_pkt[5] = 1;
            put_u16_le(&start_pkt[6], req.session_id);
            put_u16_le(&start_pkt[8], total_fragments);
            put_u32_le(&start_pkt[10], jpeg_crc32);
            send_single_packet(start_pkt, kHeaderSize);

            image_nack_received_ = false;
            image_done_received_ = false;
            schedule_rx();

            uint32_t wait_start = smtc_modem_hal_get_time_in_ms();
            while (!image_nack_received_ && !image_done_received_) {
                if (smtc_modem_hal_get_time_in_ms() - wait_start > APP_IMAGE_EOT_RETRY_INTERVAL_MS) {
                    break;
                }
                if (irq_pending_) {
                    irq_pending_ = false;
                    ral_irq_t irq = RAL_IRQ_NONE;
                    smtc_modem_hal_protect_api_call();
                    ral_status_t s = ral_get_and_clear_irq_status(&radio_.ral, &irq);
                    smtc_modem_hal_unprotect_api_call();
                    if (s == RAL_STATUS_OK && irq != RAL_IRQ_NONE) {
                        handle_irq(irq);
                    }
                }
                taskYIELD();
            }

            if (image_nack_received_ || image_done_received_) {
                r_ready = true;
                // ESP_LOGI(TAG, "image TX: R ready, starting data burst");
            } else {
            // ESP_LOGW(TAG, "image TX: ImageStart no response, retry %u/%u",
            //          start_try + 1, APP_IMAGE_EOT_RETRY_COUNT);
            }
        }

        if (!r_ready) {
            // ESP_LOGW(TAG, "image TX: R not ready, aborting");
            image_tx_active_ = false;
            suspended_ = false;
            ptt_active_ = was_ptt;
            if (!ptt_active_) schedule_rx();
            continue;
        }

        // Step 1: Blast all fragments
        for (uint16_t i = 0; i < total_fragments; i++) {
            size_t offset = static_cast<size_t>(i) * APP_IMAGE_FRAGMENT_DATA_SIZE;
            uint16_t frag_len = static_cast<uint16_t>(
                ((offset + APP_IMAGE_FRAGMENT_DATA_SIZE) <= req.jpeg_len)
                    ? APP_IMAGE_FRAGMENT_DATA_SIZE
                    : (req.jpeg_len - offset));

            uint8_t pkt[APP_FLRC_MAX_PAYLOAD_BYTES];
            std::memcpy(pkt, kMagic, sizeof(kMagic));
            pkt[4] = kPacketTypeImageData;
            pkt[5] = 1;
            put_u16_le(&pkt[6], req.session_id);
            put_u16_le(&pkt[8], i);
            put_u16_le(&pkt[10], total_fragments);
            put_u16_le(&pkt[12], frag_len);
            std::memcpy(&pkt[kHeaderSize], req.jpeg + offset, frag_len);
            uint16_t crc = crc16_ccitt(&pkt[4], kHeaderSize - 4 + frag_len);
            put_u16_le(&pkt[kHeaderSize + frag_len], crc);

            if (!send_single_packet(pkt, static_cast<uint16_t>(kHeaderSize + frag_len + 2))) {
            // ESP_LOGW(TAG, "image TX frag %u/%u failed", i, total_fragments);
            }
            if (image_tx_inter_packet_us_ > 0) {
                esp_rom_delay_us(image_tx_inter_packet_us_);
            }
        }

        // ESP_LOGI(TAG, "image TX: initial burst done (%u frags)", total_fragments);

        // Step 2-8: EOT + wait ACK + retransmit loop
        bool transfer_done = false;
        for (uint16_t round = 0; round < APP_IMAGE_NACK_MAX_RETRIES && !transfer_done; round++) {
            // Give R time to process last packets before sending EOT
            vTaskDelay(pdMS_TO_TICKS(30));

            // Send EOT, retry if no response
            bool got_response = false;
            for (uint16_t eot_try = 0; eot_try < APP_IMAGE_EOT_RETRY_COUNT; eot_try++) {
                uint8_t eot[kHeaderSize];
                std::memcpy(eot, kMagic, sizeof(kMagic));
                eot[4] = kPacketTypeImageEOT;
                eot[5] = 1;
                put_u16_le(&eot[6], req.session_id);
                put_u16_le(&eot[8], total_fragments);
                eot[10] = 0; eot[11] = 0; eot[12] = 0; eot[13] = 0;
                send_single_packet(eot, kHeaderSize);

                // Wait for ACK
                image_nack_received_ = false;
                image_done_received_ = false;
                schedule_rx();

                uint32_t wait_start = smtc_modem_hal_get_time_in_ms();
                while (!image_nack_received_ && !image_done_received_) {
                    if (smtc_modem_hal_get_time_in_ms() - wait_start > APP_IMAGE_EOT_RETRY_INTERVAL_MS) {
                        break;
                    }
                    if (irq_pending_) {
                        irq_pending_ = false;
                        ral_irq_t irq = RAL_IRQ_NONE;
                        smtc_modem_hal_protect_api_call();
                        ral_status_t s = ral_get_and_clear_irq_status(&radio_.ral, &irq);
                        smtc_modem_hal_unprotect_api_call();
                        if (s == RAL_STATUS_OK && irq != RAL_IRQ_NONE) {
                            handle_irq(irq);
                        }
                    }
                    taskYIELD();
                }

                if (image_nack_received_ || image_done_received_) {
                    got_response = true;
                    break;
                }
                // ESP_LOGW(TAG, "image TX: EOT no response, retry %u/%u",
                //          eot_try + 1, APP_IMAGE_EOT_RETRY_COUNT);
            }

            if (!got_response) {
                // ESP_LOGW(TAG, "image TX: no ACK after %u EOT retries, resending all frags",
                //          APP_IMAGE_EOT_RETRY_COUNT);
                // No ACK — resend all fragments (blind retransmit)
                for (uint16_t i = 0; i < total_fragments; i++) {
                    size_t offset = static_cast<size_t>(i) * APP_IMAGE_FRAGMENT_DATA_SIZE;
                    uint16_t frag_len = static_cast<uint16_t>(
                        ((offset + APP_IMAGE_FRAGMENT_DATA_SIZE) <= req.jpeg_len)
                            ? APP_IMAGE_FRAGMENT_DATA_SIZE
                            : (req.jpeg_len - offset));

                    uint8_t pkt[APP_FLRC_MAX_PAYLOAD_BYTES];
                    std::memcpy(pkt, kMagic, sizeof(kMagic));
                    pkt[4] = kPacketTypeImageData;
                    pkt[5] = 1;
                    put_u16_le(&pkt[6], req.session_id);
                    put_u16_le(&pkt[8], i);
                    put_u16_le(&pkt[10], total_fragments);
                    put_u16_le(&pkt[12], frag_len);
                    std::memcpy(&pkt[kHeaderSize], req.jpeg + offset, frag_len);
                    uint16_t crc = crc16_ccitt(&pkt[4], kHeaderSize - 4 + frag_len);
                    put_u16_le(&pkt[kHeaderSize + frag_len], crc);

                    send_single_packet(pkt, static_cast<uint16_t>(kHeaderSize + frag_len + 2));
                    if (image_tx_inter_packet_us_ > 0) {
                        esp_rom_delay_us(image_tx_inter_packet_us_);
                    }
                }
                continue;
            }

            if (image_done_received_ || nack_count_ == 0) {
                // ESP_LOGI(TAG, "image TX complete: all received");
                transfer_done = true;
                break;
            }

            // Retransmit missing fragments
            // ESP_LOGI(TAG, "image TX round %u: resending %u missing frags",
            //          round + 1, nack_count_);
            for (uint16_t n = 0; n < nack_count_; n++) {
                uint16_t i = nack_indices_[n];
                if (i >= total_fragments) continue;

                size_t offset = static_cast<size_t>(i) * APP_IMAGE_FRAGMENT_DATA_SIZE;
                uint16_t frag_len = static_cast<uint16_t>(
                    ((offset + APP_IMAGE_FRAGMENT_DATA_SIZE) <= req.jpeg_len)
                        ? APP_IMAGE_FRAGMENT_DATA_SIZE
                        : (req.jpeg_len - offset));

                uint8_t pkt[APP_FLRC_MAX_PAYLOAD_BYTES];
                std::memcpy(pkt, kMagic, sizeof(kMagic));
                pkt[4] = kPacketTypeImageData;
                pkt[5] = 1;
                put_u16_le(&pkt[6], req.session_id);
                put_u16_le(&pkt[8], i);
                put_u16_le(&pkt[10], total_fragments);
                put_u16_le(&pkt[12], frag_len);
                std::memcpy(&pkt[kHeaderSize], req.jpeg + offset, frag_len);
                uint16_t crc = crc16_ccitt(&pkt[4], kHeaderSize - 4 + frag_len);
                put_u16_le(&pkt[kHeaderSize + frag_len], crc);

                if (!send_single_packet(pkt, static_cast<uint16_t>(kHeaderSize + frag_len + 2))) {
            // ESP_LOGW(TAG, "image TX retransmit frag %u failed", i);
                }
                if (image_tx_inter_packet_us_ > 0) {
                    esp_rom_delay_us(image_tx_inter_packet_us_);
                }
            }
        }

        image_tx_active_ = false;
        suspended_ = false;
        ptt_active_ = was_ptt;
        // ESP_LOGI(TAG, "image TX finished: session=%u done=%d",
        //          req.session_id, transfer_done ? 1 : 0);

        if (mode_ != Mode::rx_pending && !ptt_active_) {
            schedule_rx();
        }
    }
}

bool RadioPing::send_single_packet(const uint8_t *data, uint16_t len)
{
    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);
    ral_status_t status = ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    if (status == RAL_STATUS_OK) status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    if (status == RAL_STATUS_OK) status = ral_set_pkt_payload(&radio_.ral, data, len);
    if (status == RAL_STATUS_OK) status = ral_set_tx(&radio_.ral);
    smtc_modem_hal_unprotect_api_call();

    if (status != RAL_STATUS_OK) {
        return false;
    }

    mode_ = Mode::tx_pending;
    return wait_for_tx_done(50);
}

bool RadioPing::wait_for_tx_done(uint32_t timeout_ms)
{
    uint32_t start = smtc_modem_hal_get_time_in_ms();
    while (true) {
        if (irq_pending_) {
            irq_pending_ = false;
            ral_irq_t irq = RAL_IRQ_NONE;
            smtc_modem_hal_protect_api_call();
            ral_status_t s = ral_get_and_clear_irq_status(&radio_.ral, &irq);
            smtc_modem_hal_unprotect_api_call();
            if (s == RAL_STATUS_OK && (irq & RAL_IRQ_TX_DONE)) {
                mode_ = Mode::idle;
                return true;
            }
        }
        if (smtc_modem_hal_get_time_in_ms() - start > timeout_ms) {
            mode_ = Mode::idle;
            return false;
        }
        taskYIELD();
    }
}

void RadioPing::handle_image_cmd()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    // ESP_LOGI(TAG, "RX ImageCmd: session=%u", session_id);

    if (image_capture_cb_) {
        image_capture_cb_(session_id);
    }
}

void RadioPing::handle_image_start()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    uint16_t total_frags = get_u16_le(&rx_buf_[8]);
    uint32_t expected_crc32 = get_u32_le(&rx_buf_[10]);

    // ESP_LOGI(TAG, "RX ImageStart: session=%u total=%u crc32=0x%08lx",
    //          session_id, total_frags, static_cast<unsigned long>(expected_crc32));
    image_rx_start_ms_ = smtc_modem_hal_get_time_in_ms();

    // Prepare RX buffer
    image_xfer_.rx_begin(session_id, total_frags);
    image_rx_pending_ = true;
    image_rx_nack_sent_ = 0;
    image_rx_eot_count_ = 0;
    image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
    image_rx_last_progress_ms_ = smtc_modem_hal_get_time_in_ms();
    image_rx_expected_crc32_ = expected_crc32;

    // Update UI first (before sending ACK), so LVGL work finishes
    // before node starts blasting data
    if (image_rx_progress_cb_) {
        image_rx_progress_cb_(0, total_frags, 0);
    }

    // Send ACK (ready) — missing_count=0 means "ready"
    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeImageNack;
    pkt[5] = 3;
    put_u16_le(&pkt[6], session_id);
    put_u16_le(&pkt[8], 0);  // missing_count = 0 (ready signal)
    put_u16_le(&pkt[10], 0); // total_received = 0
    pkt[12] = 0; pkt[13] = 0;
    send_single_packet(pkt, kHeaderSize);

    // Enter RX for incoming data
    schedule_rx();
}

void RadioPing::handle_image_data()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    uint16_t frag_index = get_u16_le(&rx_buf_[8]);
    uint16_t total_frags = get_u16_le(&rx_buf_[10]);
    uint16_t frag_len = get_u16_le(&rx_buf_[12]);

    if (!image_rx_pending_ || session_id != image_xfer_.rx_session_id() ||
        total_frags != image_xfer_.rx_total_count()) {
        return;
    }

    if (frag_len > APP_IMAGE_FRAGMENT_DATA_SIZE) {
        ESP_LOGW(TAG, "RX ImageData: bad frag_len=%u", frag_len);
        return;
    }

    // Verify CRC16 appended after payload
    uint16_t rx_crc = get_u16_le(&rx_buf_[kHeaderSize + frag_len]);
    uint16_t calc_crc = crc16_ccitt(&rx_buf_[4], kHeaderSize - 4 + frag_len);
    if (rx_crc != calc_crc) {
        return;
    }

    image_xfer_.rx_fragment(session_id, frag_index, total_frags,
                            &rx_buf_[kHeaderSize], frag_len);
    image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
    image_rx_pending_ = true;
    if (frag_index == 0) {
        image_rx_nack_sent_ = 0;
    }
}

void RadioPing::handle_image_nack()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    uint16_t missing_count = get_u16_le(&rx_buf_[8]);

    if (missing_count > APP_IMAGE_NACK_MAX_INDICES) {
        missing_count = APP_IMAGE_NACK_MAX_INDICES;
    }

    nack_count_ = missing_count;
    for (uint16_t i = 0; i < missing_count; i++) {
        nack_indices_[i] = get_u16_le(&rx_buf_[kHeaderSize + i * 2]);
    }

    uint16_t total_received = get_u16_le(&rx_buf_[10]);
    // ESP_LOGI(TAG, "RX ImageACK: session=%u missing=%u received=%u",
    //          session_id, missing_count, total_received);
    image_nack_received_ = true;
    if (missing_count == 0) {
        image_done_received_ = true;
    }
}

void RadioPing::handle_image_done()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    // ESP_LOGI(TAG, "RX ImageDone: session=%u", session_id);
    image_done_received_ = true;
}

void RadioPing::handle_image_eot()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    uint16_t total_frags = get_u16_le(&rx_buf_[8]);

    // ESP_LOGI(TAG, "RX ImageEOT: session=%u received=%u/%u",
    //          session_id, image_xfer_.rx_received_count(), total_frags);

    if (!image_rx_pending_) {
        // Already completed — still send ACK so T stops retrying
        if (session_id == image_rx_done_session_) {
            uint8_t pkt[kHeaderSize];
            std::memcpy(pkt, kMagic, sizeof(kMagic));
            pkt[4] = kPacketTypeImageNack;
            pkt[5] = 3;
            put_u16_le(&pkt[6], session_id);
            put_u16_le(&pkt[8], 0);
            put_u16_le(&pkt[10], 0);
            pkt[12] = 0; pkt[13] = 0;
            send_single_packet(pkt, kHeaderSize);
            schedule_rx();
        }
        return;
    }

    // Exit continuous RX to send response
    if (mode_ == Mode::rx_pending) {
        smtc_modem_hal_protect_api_call();
        (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        smtc_modem_hal_unprotect_api_call();
        mode_ = Mode::idle;
    }

    // Update UI before sending NACK — node won't retransmit until it
    // receives our reply, so LVGL work here won't cause packet loss
    if (image_rx_progress_cb_) {
        uint16_t total = image_xfer_.rx_total_count();
        image_rx_progress_cb_(image_xfer_.rx_received_count(), total, image_rx_last_rssi_);
    }

    // Build ACK with missing indices
    uint8_t pkt[APP_FLRC_MAX_PAYLOAD_BYTES];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeImageNack;
    pkt[5] = 3;
    put_u16_le(&pkt[6], session_id);

    uint16_t missing_indices[APP_IMAGE_NACK_MAX_INDICES];
    uint16_t missing_count = image_xfer_.rx_get_missing(missing_indices, APP_IMAGE_NACK_MAX_INDICES);
    if (missing_count == 0 && image_rx_expected_crc32_ != 0) {
        uint32_t actual_crc32 = image_xfer_.rx_crc32();
        if (actual_crc32 != image_rx_expected_crc32_) {
            // ESP_LOGW(TAG, "image RX crc32 mismatch: expected=0x%08lx actual=0x%08lx, requesting full resend",
            //          static_cast<unsigned long>(image_rx_expected_crc32_),
            //          static_cast<unsigned long>(actual_crc32));
            image_xfer_.rx_begin(session_id, total_frags);
            image_rx_pending_ = true;
            image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
            image_rx_last_progress_ms_ = 0;
            missing_count = image_xfer_.rx_get_missing(missing_indices, APP_IMAGE_NACK_MAX_INDICES);
        } else {
            // ESP_LOGI(TAG, "image RX crc32 ok: 0x%08lx",
            //          static_cast<unsigned long>(actual_crc32));
        }
    }

    put_u16_le(&pkt[8], missing_count);
    put_u16_le(&pkt[10], image_xfer_.rx_received_count());
    pkt[12] = 0; pkt[13] = 0;

    for (uint16_t i = 0; i < missing_count; i++) {
        put_u16_le(&pkt[kHeaderSize + i * 2], missing_indices[i]);
    }

    uint16_t pkt_len = static_cast<uint16_t>(kHeaderSize + missing_count * 2);
    send_single_packet(pkt, pkt_len);
    schedule_rx();

    if (image_rx_eot_cb_) {
        bool is_first = (image_rx_eot_count_ == 0);
        image_rx_eot_count_++;
        image_rx_eot_cb_(missing_count, is_first);
    }

    if (missing_count == 0) {
        image_rx_pending_ = false;
        image_rx_done_session_ = session_id;
        uint32_t now_ms = smtc_modem_hal_get_time_in_ms();
        uint32_t prepare_ms = image_rx_start_ms_ - image_cmd_sent_ms_;
        uint32_t transfer_ms = now_ms - image_rx_start_ms_;
        uint32_t total_ms = now_ms - image_cmd_sent_ms_;
        image_rx_transfer_ms_ = transfer_ms;
        ESP_LOGI(TAG, "image RX complete: session=%u | prepare=%lums transfer=%lums total=%lums",
                 session_id,
                 static_cast<unsigned long>(prepare_ms),
                 static_cast<unsigned long>(transfer_ms),
                 static_cast<unsigned long>(total_ms));
        if (image_rx_complete_cb_) {
            image_rx_complete_cb_(&image_xfer_);
        }
    } else {
        ESP_LOGW(TAG, "image RX: sent ACK with %u missing, waiting for retransmit", missing_count);
        if (image_rx_eot_count_ == 1) {
            for (uint16_t i = 0; i < missing_count; i += 16) {
                char line[128];
                int pos = 0;
                for (uint16_t j = i; j < missing_count && j < i + 16; j++) {
                    pos += snprintf(line + pos, sizeof(line) - pos, "%u ", missing_indices[j]);
                }
                ESP_LOGW(TAG, "  missing: %s", line);
            }
        }
        image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
    }
}

void RadioPing::check_image_rx_timeout()
{
    if (!image_rx_pending_) return;
    uint32_t now = smtc_modem_hal_get_time_in_ms();
    if (image_xfer_.rx_complete() &&
        now - image_rx_last_frag_ms_ < APP_IMAGE_RX_TIMEOUT_MS) {
        return;
    }
    if (image_xfer_.rx_complete()) {
        if (image_rx_expected_crc32_ != 0) {
            uint32_t actual_crc32 = image_xfer_.rx_crc32();
            if (actual_crc32 != image_rx_expected_crc32_) {
                // ESP_LOGW(TAG, "image RX complete timeout crc32 mismatch: expected=0x%08lx actual=0x%08lx",
                //          static_cast<unsigned long>(image_rx_expected_crc32_),
                //          static_cast<unsigned long>(actual_crc32));
                image_xfer_.rx_begin(image_xfer_.rx_session_id(), image_xfer_.rx_total_count());
                image_rx_last_frag_ms_ = now;
                image_rx_last_progress_ms_ = 0;
                return;
            }
        }
        image_rx_pending_ = false;
        image_rx_done_session_ = image_xfer_.rx_session_id();
        // ESP_LOGI(TAG, "image RX complete (no EOT seen before timeout)");
        if (image_rx_complete_cb_) {
            image_rx_complete_cb_(&image_xfer_);
        }
        return;
    }

    if (now - image_rx_last_frag_ms_ < 10000U) {
        return;
    }

    // ESP_LOGW(TAG, "image RX: timeout (10s no activity), giving up. %u/%u received",
    //          image_xfer_.rx_received_count(), image_xfer_.rx_total_count());
    image_rx_pending_ = false;
    image_xfer_.rx_reset();
    image_rx_nack_sent_ = 0;
}

bool RadioPing::send_config(uint8_t key, uint32_t value)
{
    ESP_LOGI(TAG, "send_config: key=%u value=%lu", key, static_cast<unsigned long>(value));

    suspended_ = true;

    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeConfig;
    pkt[5] = 1;
    put_u16_le(&pkt[6], 0);
    pkt[8] = key;
    put_u32_le(&pkt[9], value);
    pkt[13] = 0;

    for (int attempt = 0; attempt < 3; attempt++) {
        smtc_modem_hal_protect_api_call();
        if (mode_ == Mode::rx_pending) {
            (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            mode_ = Mode::idle;
        }
        smtc_modem_hal_unprotect_api_call();

        send_single_packet(pkt, kHeaderSize);

        config_ack_received_ = false;
        schedule_rx();

        uint32_t wait_start = smtc_modem_hal_get_time_in_ms();
        while (!config_ack_received_) {
            if (smtc_modem_hal_get_time_in_ms() - wait_start > kConfigAckTimeoutMs) {
                break;
            }
            if (irq_pending_) {
                irq_pending_ = false;
                ral_irq_t irq = RAL_IRQ_NONE;
                smtc_modem_hal_protect_api_call();
                ral_status_t s = ral_get_and_clear_irq_status(&radio_.ral, &irq);
                smtc_modem_hal_unprotect_api_call();
                if (s == RAL_STATUS_OK && irq != RAL_IRQ_NONE) {
                    handle_irq(irq);
                }
            }
            taskYIELD();
        }

        if (config_ack_received_) {
            ESP_LOGI(TAG, "send_config: ACK received on attempt %d", attempt + 1);
            suspended_ = false;
            if (!ptt_active_) schedule_rx();
            return true;
        }
        ESP_LOGW(TAG, "send_config: no ACK, attempt %d/3", attempt + 1);
    }

    ESP_LOGW(TAG, "send_config: failed after 3 attempts");
    suspended_ = false;
    if (!ptt_active_) schedule_rx();
    return false;
}

void RadioPing::send_config_ack(uint8_t key, uint32_t value)
{
    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeConfigAck;
    pkt[5] = 1;
    put_u16_le(&pkt[6], 0);
    pkt[8] = key;
    put_u32_le(&pkt[9], value);
    pkt[13] = 0;

    smtc_modem_hal_protect_api_call();
    if (mode_ == Mode::rx_pending) {
        (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        mode_ = Mode::idle;
    }
    smtc_modem_hal_unprotect_api_call();

    send_single_packet(pkt, kHeaderSize);

    if (!ptt_active_) {
        schedule_rx();
    }
}

size_t RadioPing::snapshot_audio(int16_t *out, size_t max_samples)
{
    return audio_ringbuf_.snapshot(out, max_samples);
}
