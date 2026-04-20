#include "radio_ping.hpp"

#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
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
constexpr uint16_t kPingPacketSize = 12;

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
} // namespace

RadioPing *RadioPing::instance_ = nullptr;

esp_err_t RadioPing::init()
{
    instance_ = this;

    smtc_rac_init();
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
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void RadioPing::handle_button(bsp_btn_id_t id, bool pressed)
{
    if (id != APP_PTT_BUTTON) return;

    ptt_active_ = pressed;
    ESP_LOGI(TAG, "PTT %s -> FLRC ping %s", pressed ? "down" : "up",
             pressed ? "TX" : "RX");

    if (pressed && mode_ == Mode::rx_pending) {
        smtc_rac_abort_radio_submit(radio_id_);
    } else if (!pressed && mode_ == Mode::idle) {
        schedule_rx();
    }
}

void RadioPing::task_trampoline(void *arg)
{
    static_cast<RadioPing *>(arg)->task();
}

void RadioPing::task()
{
    while (true) {
        poll_once();
        vTaskDelay(pdMS_TO_TICKS(APP_RADIO_TASK_POLL_MS));
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
        } else if (status != RP_STATUS_RX_TIMEOUT && status != RP_STATUS_TASK_ABORTED) {
            ESP_LOGW(TAG, "RX done: %s", status_to_string(status));
        }
    } else if (completed_mode == Mode::tx_pending) {
        if (status == RP_STATUS_TX_DONE) {
            ESP_LOGI(TAG, "FLRC TX ping seq=%u", static_cast<unsigned>(tx_seq_ - 1));
        } else {
            ESP_LOGW(TAG, "TX done: %s", status_to_string(status));
        }
        vTaskDelay(pdMS_TO_TICKS(APP_FLRC_PING_INTERVAL_MS));
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

    build_ping_packet();
    smtc_rac_context_t *ctx = smtc_rac_get_context(radio_id_);
    std::memset(ctx, 0, sizeof(*ctx));
    configure_common(ctx, true, kPingPacketSize);
    ctx->smtc_rac_data_buffer_setup.tx_payload_buffer = tx_buf_;
    ctx->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = kPingPacketSize;

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

void RadioPing::build_ping_packet()
{
    std::memcpy(tx_buf_, kMagic, sizeof(kMagic));
    tx_buf_[4] = kPacketTypePing;
    tx_buf_[5] = 1;
    put_u16_le(&tx_buf_[6], tx_seq_++);
    put_u32_le(&tx_buf_[8], smtc_modem_hal_get_time_in_ms());
}

void RadioPing::handle_rx_packet()
{
    smtc_rac_context_t *ctx = smtc_rac_get_context(radio_id_);
    uint16_t len = ctx->smtc_rac_data_result.rx_size;
    int16_t rssi = ctx->smtc_rac_data_result.rssi_result;

    if (len < kPingPacketSize || std::memcmp(rx_buf_, kMagic, sizeof(kMagic)) != 0 ||
        rx_buf_[4] != kPacketTypePing) {
        ESP_LOGW(TAG, "RX unknown packet len=%u rssi=%d", len, rssi);
        return;
    }

    uint16_t seq = get_u16_le(&rx_buf_[6]);
    log_rx(seq, len, rssi);
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
        }
        expected_rx_seq_ = static_cast<uint16_t>(seq + 1);
    } else {
        expected_rx_seq_ = static_cast<uint16_t>(expected_rx_seq_ + 1);
    }

    rx_packets_++;
    uint32_t denom = rx_packets_ + rx_lost_;
    uint32_t loss_x100 = denom ? (rx_lost_ * 10000UL) / denom : 0;

    ESP_LOGI(TAG, "FLRC RX seq=%u rssi=%d dBm len=%u lost=%lu/%lu loss=%lu.%02lu%%",
             seq, rssi, len,
             static_cast<unsigned long>(rx_lost_),
             static_cast<unsigned long>(denom),
             static_cast<unsigned long>(loss_x100 / 100),
             static_cast<unsigned long>(loss_x100 % 100));
}
