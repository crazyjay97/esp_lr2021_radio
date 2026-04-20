#pragma once

#include <cstdint>

#include "esp_err.h"
#include "app_config.h"
#include "bsp.h"
#include "LiotLr2021.h"

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
    static void post_callback(rp_status_t status);

    void task();
    void poll_once();
    void on_done(rp_status_t status);
    void schedule_rx();
    void schedule_tx();
    void configure_common(smtc_rac_context_t *ctx, bool is_tx, uint16_t tx_size);
    void build_ping_packet();
    void handle_rx_packet();
    void log_rx(uint16_t seq, uint16_t len, int16_t rssi);

    static RadioPing *instance_;

    uint8_t radio_id_ = RAC_INVALID_RADIO_ID;
    Mode mode_ = Mode::idle;
    bool ptt_active_ = false;
    volatile bool done_ = false;
    volatile rp_status_t done_status_ = RP_STATUS_TASK_INIT;

    uint8_t tx_buf_[32] = {};
    uint8_t rx_buf_[APP_FLRC_MAX_PAYLOAD_BYTES] = {};

    uint16_t tx_seq_ = 0;
    uint16_t expected_rx_seq_ = 0;
    bool have_expected_rx_seq_ = false;
    uint32_t rx_packets_ = 0;
    uint32_t rx_lost_ = 0;
};
