#include "radio_ping.hpp"

#include <cstring>
#include <cmath>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "app_config.h"
#include "bsp.h"

#include "lr20xx_radio_lora.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_system.h"

extern volatile bool g_low_power_enabled;

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
constexpr uint8_t kPacketTypeImageCmdAck = 11;
constexpr uint8_t kPacketTypeVbat = 12;
constexpr uint16_t kHeaderSize = 14;
constexpr uint8_t kVoiceFlagMaster = 0x01;
constexpr uint8_t kVoiceFlagNodeReply = 0x02;
constexpr uint8_t kVoiceFlagStart = 0x04;
constexpr uint8_t kVoiceFlagStop = 0x08;
constexpr uint8_t kVoiceFlagStopAck = 0x10;
constexpr uint8_t kVoiceFlagMask = kVoiceFlagMaster | kVoiceFlagNodeReply |
    kVoiceFlagStart | kVoiceFlagStop | kVoiceFlagStopAck;

// Continuous-stream fast path. The gateway places the next session in bytes
// 12..13 of the previous frame's final empty-missing ACK. The node holds that
// session until the old capture task has released its busy state.
std::atomic<bool> s_image_stream_active{false};
std::atomic<uint16_t> s_chained_capture_session{0};
std::atomic<uint16_t> s_image_tx_session{0};
uint16_t s_image_rx_done_next_session = 0;

/* Low-power node battery voltage maintenance cadence and broadcast interval. */
constexpr uint32_t kVbatLowPowerSampleIntervalMs = 60000;     /* 60 s */
constexpr uint32_t kVbatBroadcastIntervalMs = 300000;         /* 5 min */
constexpr uint32_t kConfigAckTimeoutMs = 500;

int32_t abs16(int16_t v)
{
    return v < 0 ? -static_cast<int32_t>(v) : v;
}

void apply_intercom_playback_gain(int16_t *pcm, size_t samples)
{
    const int32_t gain_percent =
        static_cast<int32_t>(APP_INTERCOM_PLAYBACK_PERCENT);
    for (size_t i = 0; i < samples; ++i) {
        const int32_t scaled = static_cast<int32_t>(pcm[i]) * gain_percent;
        pcm[i] = static_cast<int16_t>(scaled / 100);
    }
}

void log_intercom_heap(const char *stage)
{
    constexpr uint32_t kInternalCaps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    constexpr uint32_t kDmaCaps =
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
    ESP_LOGI(TAG,
             "intercom heap %s: internal free=%u largest=%u | "
             "dma free=%u largest=%u",
             stage,
             static_cast<unsigned>(heap_caps_get_free_size(kInternalCaps)),
             static_cast<unsigned>(
                 heap_caps_get_largest_free_block(kInternalCaps)),
             static_cast<unsigned>(heap_caps_get_free_size(kDmaCaps)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(kDmaCaps)));
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

esp_err_t RadioPing::init_gateway()
{
    instance_ = this;
    is_gateway_ = true;

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

#if !APP_RADIO_HW_INIT_ENABLE
    ESP_LOGW(TAG, "LR2021 hardware init disabled");
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

    ESP_LOGI(TAG, "LR2021 gateway mode (RX-only): FLRC rf=%lu Hz br=%lu bps bw=%lu Hz",
             APP_FLRC_FREQUENCY_HZ, APP_FLRC_BITRATE_BPS, APP_FLRC_BANDWIDTH_HZ);
#if APP_RADIO_AUTO_RX_ENABLE
    schedule_rx();
#endif
    return ESP_OK;
}

esp_err_t RadioPing::start_gateway()
{
    BaseType_t ok = xTaskCreatePinnedToCore(task_trampoline, "radio_ping",
                                            APP_RADIO_TASK_STACK_BYTES, this,
                                            APP_RADIO_TASK_PRIORITY, &task_handle_,
                                            APP_RADIO_TASK_CORE);
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

    ok = xTaskCreatePinnedToCore(tx_task_trampoline, "voice_tx",
                                 APP_VOICE_TX_TASK_STACK_BYTES, this,
                                 APP_VOICE_TX_TASK_PRIORITY, nullptr,
                                 APP_VOICE_TX_TASK_CORE);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void RadioPing::start_intercom_local(uint16_t session)
{
    intercom_session_ = session;
    intercom_prepared_session_ = session;
    intercom_start_confirmed_ = false;
    intercom_stop_confirmed_ = false;
    intercom_stop_requested_ = false;
    intercom_stop_reply_ = false;
    intercom_reply_pending_ = false;
    intercom_last_sync_ms_ = smtc_modem_hal_get_time_in_ms();
    intercom_tx_slots_ = 0;
    intercom_rx_slots_ = 0;
    intercom_missed_slots_ = 0;
    intercom_mic_frames_ = 0;
    intercom_play_frames_ = 0;
    intercom_aec_us_total_ = 0;
    intercom_aec_us_max_ = 0;
    intercom_input_clip_samples_ = 0;
    intercom_next_slot_us_ = esp_timer_get_time() + 20000;
    ptt_active_ = false;
    tx_burst_active_ = false;
    tx_flush_pending_ = false;
    sound_trigger_pending_ = false;
    pir_triggered_ = false;
    if (tx_queue_) xQueueReset(tx_queue_);
    if (voice_queue_) xQueueReset(voice_queue_);
    codec_.reset_encoder();
    codec_.reset_decoder();
    audio_proc_.reset();
    echo_canceller_.reset();
    have_expected_rx_seq_ = false;
    have_expected_play_seq_ = false;
    playback_active_ = false;
    intercom_active_ = true;
    if (intercom_state_cb_) intercom_state_cb_(true);
    cad_wakeup_ms_ = 0;
    pir_push_wake_ = false;
    if (task_handle_) xTaskNotifyGive(task_handle_);
    ESP_LOGI(TAG, "intercom local start session=%u role=%s", session,
             is_gateway_ ? "gateway" : "node");
    ESP_LOGI(TAG,
             "intercom diagnostics: aec=esp-sr-direct-voip-high-perf ready=%d "
             "low_power=%d cad_active=%d mode=%u tx_core=%u tx_prio=%u "
             "play_core=%u play_prio=%u tx_gain=%u play_gain=%u%%",
             echo_canceller_.ready() ? 1 : 0, g_low_power_enabled ? 1 : 0,
             low_power_cad_active_ ? 1 : 0, static_cast<unsigned>(mode_),
             static_cast<unsigned>(APP_VOICE_TX_TASK_CORE),
             static_cast<unsigned>(APP_VOICE_TX_TASK_PRIORITY),
             static_cast<unsigned>(APP_VOICE_PLAY_TASK_CORE),
             static_cast<unsigned>(APP_VOICE_PLAY_TASK_PRIORITY),
             static_cast<unsigned>(APP_INTERCOM_INPUT_GAIN),
             static_cast<unsigned>(APP_INTERCOM_PLAYBACK_PERCENT));
}

void RadioPing::stop_intercom_local()
{
    intercom_last_stopped_session_ = intercom_session_;
    intercom_active_ = false;
    if (intercom_state_cb_) intercom_state_cb_(false);
    intercom_reply_pending_ = false;
    intercom_stop_requested_ = false;
    intercom_stop_reply_ = false;
    if (tx_queue_) xQueueReset(tx_queue_);
    if (voice_queue_) xQueueReset(voice_queue_);
    set_playback_pa(false);
    playback_active_ = false;
    echo_canceller_.reset();
    ESP_LOGI(TAG, "intercom local stop session=%u", intercom_session_);
}

bool RadioPing::set_intercom(bool enable)
{
    if (!is_gateway_) return false;
    ESP_LOGI(TAG,
             "intercom request: enable=%d active=%d started=%d stop_req=%d "
             "image_req=%d image_rx=%d suspended=%d mode=%u",
             enable ? 1 : 0, intercom_active_ ? 1 : 0,
             intercom_start_confirmed_ ? 1 : 0, intercom_stop_requested_ ? 1 : 0,
             image_req_active_ ? 1 : 0, image_rx_pending_ ? 1 : 0,
             suspended_ ? 1 : 0, static_cast<unsigned>(mode_));
    if (enable == intercom_active_) {
        ESP_LOGI(TAG, "intercom request already in local state enable=%d", enable ? 1 : 0);
        return true;
    }

    if (enable) {
        abort_image_rx();
        const uint32_t abort_start = smtc_modem_hal_get_time_in_ms();
        while (image_rx_abort_req_.load(std::memory_order_acquire) &&
               smtc_modem_hal_get_time_in_ms() - abort_start < 100U) {
            vTaskDelay(ms_to_ticks_min_1(2));
        }
        if (image_rx_abort_req_.load(std::memory_order_acquire)) {
            ESP_LOGW(TAG, "image abort not consumed before intercom CONFIG");
        }
#if APP_INTERCOM_AEC_ENABLE
        log_intercom_heap("before AEC");
        if (!echo_canceller_.ready() &&
            !echo_canceller_.init()) {
            ESP_LOGE(TAG,
                     "intercom start rejected: gateway ESP-SR direct AEC init failed");
            return false;
        }
        log_intercom_heap("after AEC");
#endif
        uint16_t session = static_cast<uint16_t>(intercom_session_ + 1U);
        if (session == 0) session = 1;
        if (!send_config(APP_CFG_KEY_INTERCOM, session)) {
            echo_canceller_.reset();
            return false;
        }
        start_intercom_local(session);

        const uint32_t start = smtc_modem_hal_get_time_in_ms();
        while (!intercom_start_confirmed_ &&
               smtc_modem_hal_get_time_in_ms() - start < APP_INTERCOM_START_TIMEOUT_MS) {
            vTaskDelay(ms_to_ticks_min_1(2));
        }
        if (!intercom_start_confirmed_) {
            ESP_LOGW(TAG, "intercom start handshake timeout session=%u", session);
            intercom_stop_requested_ = true;
            if (task_handle_) xTaskNotifyGive(task_handle_);
            const uint32_t stop_start = smtc_modem_hal_get_time_in_ms();
            while (!intercom_stop_confirmed_ &&
                   smtc_modem_hal_get_time_in_ms() - stop_start <
                       APP_INTERCOM_STOP_TIMEOUT_MS) {
                vTaskDelay(ms_to_ticks_min_1(2));
            }
            stop_intercom_local();
            schedule_rx();
            return false;
        }
        ESP_LOGI(TAG, "intercom START confirmed session=%u in %lums", session,
                 static_cast<unsigned long>(smtc_modem_hal_get_time_in_ms() - start));
        return true;
    }

    intercom_stop_confirmed_ = false;
    intercom_stop_requested_ = true;
    if (task_handle_) xTaskNotifyGive(task_handle_);
    const uint32_t start = smtc_modem_hal_get_time_in_ms();
    while (!intercom_stop_confirmed_ &&
           smtc_modem_hal_get_time_in_ms() - start < APP_INTERCOM_STOP_TIMEOUT_MS) {
        vTaskDelay(ms_to_ticks_min_1(2));
    }
    const bool acknowledged = intercom_stop_confirmed_;
    stop_intercom_local();
    schedule_rx();
    if (!acknowledged) ESP_LOGW(TAG, "intercom stop ACK timeout");
    ESP_LOGI(TAG, "intercom local close complete remote_ack=%d", acknowledged ? 1 : 0);
    // The UI represents the local owner state. Even if the final radio ACK was
    // lost, local cleanup is complete and the node has its link-loss failsafe;
    // returning false here incorrectly forces the switch back to ON.
    return true;
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
                                 APP_IMAGE_TX_TASK_PRIORITY, nullptr,
                                 APP_IMAGE_TX_TASK_CORE);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void RadioPing::handle_button(bsp_btn_id_t id, bool pressed)
{
    if (id != APP_PTT_BUTTON) return;
    if (suspended_ || intercom_active_) return;

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
            // Consume control-plane aborts before intercom takes the packet plane.
            check_image_rx_abort();
            if (intercom_active_) {
                service_intercom();
                ulTaskNotifyTake(pdTRUE, ms_to_ticks_min_1(APP_RADIO_TASK_POLL_MS));
                continue;
            }
            check_image_rx_timeout();
            check_image_capture_request();

            uint16_t chained_session =
                s_chained_capture_session.load(std::memory_order_acquire);
            if (chained_session != 0 && !image_tx_active_ && image_capture_cb_ &&
                image_capture_cb_(chained_session)) {
                uint16_t expected = chained_session;
                (void)s_chained_capture_session.compare_exchange_strong(
                    expected, 0, std::memory_order_acq_rel);
                ESP_LOGD(TAG, "chained capture accepted: session=%u", chained_session);
            }

            check_image_req_retry();
        }
        ulTaskNotifyTake(pdTRUE, ms_to_ticks_min_1(APP_RADIO_TASK_POLL_MS));
    }
}

void RadioPing::tx_task()
{
    if (is_gateway_) {
        ESP_LOGI(TAG, "gateway mic task parked until intercom starts");
    }
    while (true) {
        // Sound detection remains a local trigger input. It never enters the
        // image payload; dispatch is gated while an image transfer is active.
        if (suspended_) {
            vTaskDelay(ms_to_ticks_min_1(APP_AUDIO_FRAME_MS));
            continue;
        }

        // The gateway has no local sound-trigger or pre-encode consumer. Before
        // intercom starts, avoid waking the CPU0 capture task and competing with
        // the higher-priority radio service for I2S data that no consumer needs.
        if (is_gateway_ && !intercom_active_) {
            vTaskDelay(ms_to_ticks_min_1(APP_AUDIO_FRAME_MS));
            continue;
        }

        // Low power (node): the CPU sleeps during CAD standby, so we don't
        // sample the mic at all — no voice prep, no Opus pre-encoding, no sound
        // trigger. PIR is a hardware GPIO wake source, so its trigger is still
        // handled here after wakeup.
        if (g_low_power_enabled && !is_gateway_ && !intercom_active_) {
            if (pir_triggered_) {
                pir_triggered_ = false;
                bool dispatched = false;
                if (pir_enabled_) {
                    int64_t now = esp_timer_get_time();
                    if ((now - last_trigger_us_) >= (int64_t)APP_TRIGGER_COOLDOWN_SEC * 1000000LL) {
                        last_trigger_us_ = now;
                        ESP_LOGI(TAG, "PIR trigger! (low power)");
                        if (image_capture_cb_) {
                            image_capture_cb_(sound_trigger_session_id_++);
                            dispatched = true;
                        }
                    }
                }
                // Capture not dispatched (PIR disabled, still in 15s cooldown, or
                // no callback): end the keep-awake guard now so we don't sit deaf
                // to the gateway for the full 8s safety timeout. on_image_capture_
                // request handles the dropped-after-dispatch cases.
                if (!dispatched) {
                    pir_push_wake_ = false;
                }
            }
            vTaskDelay(ms_to_ticks_min_1(APP_AUDIO_FRAME_MS));
            continue;
        }

        if (!read_mono_frame(tx_pcm_, APP_AUDIO_FRAME_SAMPLES)) {
            vTaskDelay(ms_to_ticks_min_1(APP_AUDIO_FRAME_MS));
            continue;
        }

        if (intercom_active_) {
            uint32_t aec_us = 0;
#if APP_INTERCOM_AEC_ENABLE
            const int64_t aec_start_us = esp_timer_get_time();
            echo_canceller_.process_capture(tx_pcm_, APP_AUDIO_FRAME_SAMPLES);
            aec_us = static_cast<uint32_t>(esp_timer_get_time() - aec_start_us);
#endif
            uint32_t sum_abs = 0;
            int32_t peak = 0;
            for (size_t i = 0; i < APP_AUDIO_FRAME_SAMPLES; ++i) {
                int32_t level = abs16(tx_pcm_[i]);
                sum_abs += static_cast<uint32_t>(level);
                if (level > peak) peak = level;
                int32_t scaled = static_cast<int32_t>(tx_pcm_[i]) *
                    APP_INTERCOM_INPUT_GAIN;
                if (scaled > 32767) {
                    scaled = 32767;
                    intercom_input_clip_samples_++;
                }
                if (scaled < -32768) {
                    scaled = -32768;
                    intercom_input_clip_samples_++;
                }
                tx_pcm_[i] = static_cast<int16_t>(scaled);
            }
            intercom_mic_frames_++;
            intercom_aec_us_total_ += aec_us;
            if (aec_us > intercom_aec_us_max_) intercom_aec_us_max_ = aec_us;
            if (intercom_mic_frames_ == 100U ||
                (intercom_mic_frames_ % 500U) == 0U) {
                ESP_LOGI(TAG,
                         "intercom mic: frames=%lu avg_abs=%lu peak=%ld "
                         "clip_total=%lu aec_avg=%luus aec_max=%luus",
                         static_cast<unsigned long>(intercom_mic_frames_),
                         static_cast<unsigned long>(sum_abs / APP_AUDIO_FRAME_SAMPLES),
                         static_cast<long>(peak),
                         static_cast<unsigned long>(intercom_input_clip_samples_),
                         static_cast<unsigned long>(intercom_aec_us_total_ /
                                                    intercom_mic_frames_),
                         static_cast<unsigned long>(intercom_aec_us_max_));
            }
            uint8_t encoded[APP_OPUS_MAX_PACKET_BYTES];
            const int encoded_len = codec_.encode(tx_pcm_, APP_AUDIO_FRAME_SAMPLES,
                                                   encoded,
                                                   APP_OPUS_MAX_PACKET_BYTES);
            if (encoded_len > 0 && encoded_len <= 255) {
                enqueue_voice_frame(encoded, static_cast<uint16_t>(encoded_len));
            }

            // Direct VOIP_HIGH_PERF is synchronous and can otherwise keep its
            // pinned core continuously ready. Block for one tick so the idle
            // task can service TWDT; the higher-priority radio task can still
            // preempt AEC/Opus whenever its 2 ms poll wakes.
            vTaskDelay(ms_to_ticks_min_1(1));
            continue;
        }

        // Trigger DETECTION + DISPATCH is gated on !image_tx_active_: firing a new
        // capture while one is already being captured/transmitted is pointless (the
        // capture path is single-flight) and would race the in-flight transfer.
        if (!image_tx_active_) {
            if (sound_trigger_level_ > 0) {
                int64_t sum_sq = 0;
                for (size_t i = 0; i < APP_AUDIO_FRAME_SAMPLES; i++) {
                    int32_t s = tx_pcm_[i];
                    sum_sq += s * s;
                }
                uint16_t rms = (uint16_t)sqrtf((float)sum_sq / APP_AUDIO_FRAME_SAMPLES);
                uint16_t thresh = (sound_trigger_level_ == 1) ? APP_SOUND_TRIGGER_THRESH_LOW :
                                  (sound_trigger_level_ == 2) ? APP_SOUND_TRIGGER_THRESH_MED :
                                                               APP_SOUND_TRIGGER_THRESH_HIGH;
                if (rms >= thresh) {
                    int64_t now = esp_timer_get_time();
                    // Arm a delayed trigger (don't dispatch yet). last_trigger_us_ is
                    // stamped now so the cooldown covers the detection instant, and
                    // sound_trigger_pending_ blocks re-arming while one is in flight.
                    if (!sound_trigger_pending_ &&
                        (now - last_trigger_us_) >= (int64_t)APP_TRIGGER_COOLDOWN_SEC * 1000000LL) {
                        last_trigger_us_ = now;
                        sound_trigger_fire_us_ = now + (int64_t)APP_SOUND_TRIGGER_DELAY_MS * 1000LL;
                        sound_trigger_pending_session_ = sound_trigger_session_id_++;
                        sound_trigger_pending_ = true;
                        ESP_LOGI(TAG, "sound trigger detected! rms=%u thresh=%u, fire in %ums",
                                 rms, thresh, (unsigned)APP_SOUND_TRIGGER_DELAY_MS);
                    }
                }
            }

            if (pir_triggered_) {
                pir_triggered_ = false;
                if (pir_enabled_) {
                    int64_t now = esp_timer_get_time();
                    if ((now - last_trigger_us_) >= (int64_t)APP_TRIGGER_COOLDOWN_SEC * 1000000LL) {
                        last_trigger_us_ = now;
                        ESP_LOGI(TAG, "PIR trigger!");
                        if (image_capture_cb_) {
                            image_capture_cb_(sound_trigger_session_id_++);
                        }
                    }
                }
            }

            // Fire a pending sound trigger once the short debounce delay has
            // elapsed.
            if (sound_trigger_pending_ && esp_timer_get_time() >= sound_trigger_fire_us_) {
                sound_trigger_pending_ = false;
                ESP_LOGI(TAG, "sound trigger firing (delayed %ums)",
                         (unsigned)APP_SOUND_TRIGGER_DELAY_MS);
                if (image_capture_cb_) {
                    image_capture_cb_(sound_trigger_pending_session_);
                }
            }
        }
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

        if (!intercom_active_) {
            audio_proc_.process_rx_frame(rx_pcm_, static_cast<size_t>(decoded));
        } else {
            apply_intercom_playback_gain(rx_pcm_, static_cast<size_t>(decoded));
        }
#if APP_INTERCOM_AEC_ENABLE
        if (intercom_active_) {
            echo_canceller_.push_reference(rx_pcm_, static_cast<size_t>(decoded));
        }
#endif
        if (intercom_active_) {
            uint32_t sum_abs = 0;
            int32_t peak = 0;
            for (int i = 0; i < decoded; ++i) {
                int32_t level = abs16(rx_pcm_[i]);
                sum_abs += static_cast<uint32_t>(level);
                if (level > peak) peak = level;
            }
            intercom_play_frames_++;
            if (intercom_play_frames_ == 100U ||
                (intercom_play_frames_ % 500U) == 0U) {
                ESP_LOGI(TAG, "intercom playback: frames=%lu avg_abs=%lu peak=%ld",
                         static_cast<unsigned long>(intercom_play_frames_),
                         static_cast<unsigned long>(sum_abs / static_cast<uint32_t>(decoded)),
                         static_cast<long>(peak));
            }
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

    // CAD watchdog: a CAD_DONE IRQ normally arrives within a few ms. If it is
    // ever lost, mode_ would stay cad_pending forever and the node would be
    // unrecoverable without a reboot (poll_once and "exit low power" both need
    // idle). After 2s with no IRQ, force the radio back to standby+idle so the
    // next pass re-arms CAD (or FLRC RX if low power was turned off meanwhile).
    if (mode_ == Mode::cad_pending && cad_pending_ms_ != 0 &&
        (smtc_modem_hal_get_time_in_ms() - cad_pending_ms_) >= 2000) {
        ESP_LOGW(TAG, "CAD watchdog: no CAD_DONE in 2s, resetting to idle");
        smtc_modem_hal_protect_api_call();
        ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        smtc_modem_hal_unprotect_api_call();
        cad_pending_ms_ = 0;
        mode_ = Mode::idle;
    }

    if (mode_ == Mode::idle) {
        if (g_low_power_enabled && !is_gateway_ && !intercom_active_) {
            // Low-power CAD sleep is a NODE-only behavior. The gateway never
            // sleeps: it must stay in continuous FLRC RX so it can receive a
            // node's self-initiated (PIR-triggered) image push, which arrives
            // with no prior request from the gateway.
            if (audio_playing_) {
                // A voice alarm clip is playing (synchronous, in image_capture_
                // task). Stay awake-but-idle: light sleep would halt both cores
                // and starve/garble I2S. audio_playback_end() releases this and
                // the node re-sleeps on the next idle pass. No timeout — the clip
                // is short and bounded by its own length.
            } else if (pir_push_wake_) {
                // PIR push in progress: node is (about to be) the transmitter.
                // Stay awake-but-idle — do NOT open RX and do NOT sleep to CAD.
                // image_tx_task will grab the radio (suspended_=true) once the
                // capture completes. 8s safety timeout in case the capture never
                // fires (e.g. blocked by the 15s cooldown) so we don't stay awake
                // forever burning power — fall back to CAD sleep.
                if (smtc_modem_hal_get_time_in_ms() - pir_push_wake_ms_ >= 8000) {
                    pir_push_wake_ = false;
                    enter_low_power_cad();
                }
            } else if (cad_wakeup_ms_ != 0 &&
                (smtc_modem_hal_get_time_in_ms() - cad_wakeup_ms_) < APP_LP_WAKE_WINDOW_MS) {
                // Window is refreshed on every RX packet (handle_rx_packet), so
                // this stays open as long as there is traffic; it closes only
                // after APP_LP_WAKE_WINDOW_MS of total silence.
                schedule_rx();
            } else {
                cad_wakeup_ms_ = 0;
                // Node is awake and idle here; do voltage sampling / broadcast
                // now (before dropping into CAD light sleep) so we never spin up
                // the chip just for VBAT. Broadcast goes out on this awake window.
                vbat_maintenance_tick();
                enter_low_power_cad();
            }
        } else if (tx_burst_active_) {
            schedule_tx();
            if (!tx_burst_active_ && !ptt_active_ && mode_ == Mode::idle) {
                schedule_rx();
            }
        } else if (!ptt_active_) {
            if (low_power_cad_active_) {
                low_power_cad_active_ = false;
                configure_flrc();
                ESP_LOGI(TAG, "low power off, back to FLRC RX");
            }
            // Non-low-power node: periodic 5-min voltage broadcast. Guarded
            // internally by timestamp so this is cheap to call every idle pass.
            vbat_maintenance_tick();
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
        // If a task is block-waiting for TX_DONE inside send_single_packet, wake
        // that task directly; otherwise wake the main radio task (RX / poll loop).
        TaskHandle_t target = self->tx_done_waiter_;
        if (target == nullptr) {
            target = self->task_handle_;
        }
        if (target != nullptr) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(target, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void RadioPing::handle_irq(ral_irq_t irq)
{
    Mode completed_mode = mode_;

    if (completed_mode == Mode::rx_pending) {
        if ((irq & RAL_IRQ_RX_DONE) != 0) {
            // During an image burst the radio stays in continuous RX. Re-arming
            // after each fragment would clear bytes of the next fragment that
            // are already entering the 1024-byte FIFO.
            bool image_stream = image_rx_pending_;
            mode_ = Mode::idle;
            handle_rx_packet();
            if (image_stream) {
                if (mode_ == Mode::idle) {
                    mode_ = Mode::rx_pending;
                }
            } else if (mode_ == Mode::idle && !ptt_active_ && !tx_burst_active_) {
                schedule_rx();
            }
        } else if ((irq & RAL_IRQ_RX_CRC_ERROR) != 0) {
            mode_ = Mode::idle;
            rx_crc_errors_++;
            if ((rx_crc_errors_ % 10) == 1) {
                ESP_LOGW(TAG, "RX CRC errors=%lu", static_cast<unsigned long>(rx_crc_errors_));
            }
            // A bad packet has unknown usable length. Re-arm once to clear and
            // realign the FIFO; the existing EOT/NACK loop requests it again.
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
    } else if (completed_mode == Mode::cad_pending) {
        handle_cad_irq(irq);
    }
}

void RadioPing::schedule_rx()
{
    if (mode_ != Mode::idle) return;

    // The gateway is RX-only and must listen with NO gaps: a node's
    // self-initiated (PIR) image push arrives unannounced, and the 100ms
    // timeout+re-arm cycle leaves a set_standby/set_rx blind spot the push can
    // fall into. So the gateway always uses continuous RX. It still transmits
    // fine because every gateway TX path (send_lora_wakeup, trigger_image_
    // capture) breaks out of RX via ral_set_standby first. Nodes keep the short
    // timeout so they can fall back into CAD sleep between windows.
    uint32_t rx_timeout = APP_FLRC_RX_TIMEOUT_MS;
    if (image_rx_pending_ || is_gateway_ || intercom_active_) {
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

bool RadioPing::leave_rx_for_tx()
{
    if (mode_ == Mode::idle) return true;
    if (mode_ != Mode::rx_pending && mode_ != Mode::cad_pending) return false;

    smtc_modem_hal_protect_api_call();
    ral_status_t status = ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
    if (status == RAL_STATUS_OK) {
        status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    }
    smtc_modem_hal_unprotect_api_call();
    irq_pending_ = false;
    cad_pending_ms_ = 0;
    mode_ = Mode::idle;
    return status == RAL_STATUS_OK;
}

void RadioPing::service_intercom()
{
    if (!intercom_active_) return;

    // A low-power node may still own the radio in LoRa CAD when CONFIG arrives.
    // Intercom requires FLRC before the first START/master slot can be received.
    if (low_power_cad_active_) {
        if (!leave_rx_for_tx()) {
            ESP_LOGW(TAG, "intercom CAD exit deferred: mode=%u",
                     static_cast<unsigned>(mode_));
            return;
        }
        if (!configure_flrc()) {
            intercom_missed_slots_++;
            ESP_LOGW(TAG, "intercom FLRC restore failed");
            return;
        }
        low_power_cad_active_ = false;
        cad_wakeup_ms_ = 0;
        schedule_rx();
        ESP_LOGI(TAG, "intercom restored FLRC from low-power CAD");
    }
    const int64_t now_us = esp_timer_get_time();

    if (is_gateway_) {
        if (now_us < intercom_next_slot_us_) return;
        const int64_t period_us =
            static_cast<int64_t>(APP_INTERCOM_SLOT_PERIOD_MS) * 1000LL;
        intercom_next_slot_us_ += period_us;
        if (intercom_next_slot_us_ <= now_us) {
            intercom_next_slot_us_ = now_us + period_us;
            intercom_missed_slots_++;
        }
        uint8_t flags = kVoiceFlagMaster;
        if (!intercom_start_confirmed_) flags |= kVoiceFlagStart;
        if (intercom_stop_requested_) flags |= kVoiceFlagStop;
        (void)send_intercom_slot(flags);
        return;
    }

    if (intercom_start_confirmed_ &&
        smtc_modem_hal_get_time_in_ms() - intercom_last_sync_ms_ >
            APP_INTERCOM_LINK_TIMEOUT_MS) {
        ESP_LOGW(TAG, "intercom master sync lost; leaving session");
        stop_intercom_local();
        schedule_rx();
        return;
    }
    if (!intercom_start_confirmed_ &&
        smtc_modem_hal_get_time_in_ms() - intercom_last_sync_ms_ >
            APP_INTERCOM_START_TIMEOUT_MS) {
        ESP_LOGW(TAG, "intercom prepared session timed out");
        stop_intercom_local();
        schedule_rx();
        return;
    }
    if (!intercom_reply_pending_ || now_us < intercom_reply_due_us_) return;
    intercom_reply_pending_ = false;
    if (smtc_modem_hal_get_time_in_ms() - intercom_last_sync_ms_ >
        APP_INTERCOM_SYNC_TIMEOUT_MS) {
        intercom_missed_slots_++;
        return;
    }
    uint8_t flags = kVoiceFlagNodeReply;
    if (intercom_stop_reply_) flags |= kVoiceFlagStopAck;
    const bool stop_after_reply = intercom_stop_reply_;
    (void)send_intercom_slot(flags);
    if (stop_after_reply) {
        stop_intercom_local();
        schedule_rx();
    }
}

bool RadioPing::send_intercom_slot(uint8_t flags)
{
    if (!leave_rx_for_tx()) {
        intercom_missed_slots_++;
        return false;
    }
    uint16_t tx_size = 0;
    if (!build_voice_packet(&tx_size, flags)) {
        intercom_missed_slots_++;
        schedule_rx();
        return false;
    }
    const bool ok = send_single_packet(tx_buf_, tx_size,
                                       APP_INTERCOM_TX_TIMEOUT_MS);
    if (ok) intercom_tx_slots_++;
    else {
        intercom_missed_slots_++;
        if (intercom_missed_slots_ == 1U ||
            (intercom_missed_slots_ % 50U) == 0U) {
            ESP_LOGW(TAG, "intercom TX failed role=%s flags=0x%02x mode=%u missed=%lu",
                     is_gateway_ ? "gateway" : "node", flags,
                     static_cast<unsigned>(mode_),
                     static_cast<unsigned long>(intercom_missed_slots_));
        }
    }
    schedule_rx();
    if (((intercom_tx_slots_ + intercom_missed_slots_) % 100U) == 1U) {
        ESP_LOGI(TAG, "intercom slots tx=%lu rx=%lu missed=%lu queued=%u",
                 static_cast<unsigned long>(intercom_tx_slots_),
                 static_cast<unsigned long>(intercom_rx_slots_),
                 static_cast<unsigned long>(intercom_missed_slots_),
                 tx_queue_ ? static_cast<unsigned>(uxQueueMessagesWaiting(tx_queue_)) : 0U);
    }
    return ok;
}

bool RadioPing::configure_flrc()
{
    ralf_params_flrc_t params = {};
    params.rf_freq_in_hz = APP_FLRC_FREQUENCY_HZ;
    params.output_pwr_in_dbm = APP_FLRC_TX_POWER_DBM;
    params.mod_params.raw_bit_rate = APP_FLRC_RAW_BIT_RATE;
    params.mod_params.cr = APP_FLRC_CODING_RATE;
    params.mod_params.pulse_shape = APP_FLRC_PULSE_SHAPE;
    params.pkt_params.preamble_len = APP_FLRC_PREAMBLE_LEN;
    params.pkt_params.sync_word_len = RAL_FLRC_SYNCWORD_LENGTH_4_BYTES;
    params.pkt_params.tx_syncword = RAL_FLRC_TX_SYNCWORD_1;
    params.pkt_params.match_sync_word = RAL_FLRC_RX_MATCH_SYNCWORD_1;
    params.pkt_params.pld_is_fix = false;
    params.pkt_params.pld_len_in_bytes = APP_FLRC_MAX_PAYLOAD_BYTES;
    params.pkt_params.crc_type = RAL_FLRC_CRC_2_BYTES;
    params.sync_word[0] = kSyncWord;
    params.sync_word[1] = nullptr;
    params.sync_word[2] = nullptr;
    params.is_tx = true;
    params.crc_seed = 0xFFFFFFFFUL;
    params.crc_polynomial = 0x04C11DB7UL;
    if (ralf_setup_flrc(&radio_, &params) != RAL_STATUS_OK) {
        return false;
    }

    const void *ctx = radio_.ral.context;
    if (lr20xx_radio_fifo_configure_1024_byte_tx_fifo(ctx) != LR20XX_STATUS_OK) {
        ESP_LOGE(TAG, "configure_flrc: 1024 TX FIFO failed");
        return false;
    }
    if (lr20xx_radio_fifo_configure_1024_byte_rx_fifo(ctx) != LR20XX_STATUS_OK) {
        ESP_LOGE(TAG, "configure_flrc: 1024 RX FIFO failed");
        return false;
    }
    return true;
}

bool RadioPing::build_voice_packet(uint16_t *tx_size, uint8_t flags)
{
    if (tx_size == nullptr || tx_queue_ == nullptr) return false;
    TxFrame frame = {};
    bool have_frame = xQueueReceive(tx_queue_, &frame, 0) == pdTRUE;
    if (!have_frame && flags == 0) return false;

    std::memcpy(tx_buf_, kMagic, sizeof(kMagic));
    tx_buf_[4] = kPacketTypeVoice;
    tx_buf_[5] = 2;
    put_u16_le(&tx_buf_[6], have_frame ? frame.seq : tx_seq_);
    put_u32_le(&tx_buf_[8], smtc_modem_hal_get_time_in_ms());
    tx_buf_[12] = 0;
    tx_buf_[13] = flags & kVoiceFlagMask;

    uint16_t offset = kHeaderSize;
    uint8_t frame_count = 0;
    const uint8_t frame_limit = intercom_active_ ? APP_INTERCOM_FRAMES_PER_PACKET :
                                                   APP_FLRC_OPUS_FRAMES_PER_PACKET;
    while (have_frame && frame_count < frame_limit) {
        if (frame.len == 0 || frame.len > APP_OPUS_MAX_PACKET_BYTES ||
            offset + 1U + frame.len > APP_FLRC_VOICE_MAX_PAYLOAD_BYTES) {
            break;
        }

        tx_buf_[offset++] = static_cast<uint8_t>(frame.len);
        std::memcpy(&tx_buf_[offset], frame.payload, frame.len);
        offset = static_cast<uint16_t>(offset + frame.len);
        frame_count++;

        have_frame = frame_count < frame_limit &&
            xQueueReceive(tx_queue_, &frame, 0) == pdTRUE;
    }

    tx_buf_[12] = frame_count;
    if (intercom_active_ || flags != 0) {
        put_u16_le(&tx_buf_[8], intercom_session_);
    }
    *tx_size = offset;
    return frame_count > 0 || flags != 0;
}

void RadioPing::enqueue_voice_frame(const uint8_t *payload, uint16_t len)
{
    if (!tx_queue_ || !payload || len == 0 || len > APP_OPUS_MAX_PACKET_BYTES) return;
    if (intercom_active_) {
        TxFrame stale = {};
        while (uxQueueMessagesWaiting(tx_queue_) >= APP_INTERCOM_TX_QUEUE_FRAMES &&
               xQueueReceive(tx_queue_, &stale, 0) == pdTRUE) {
            tx_queue_drops_++;
        }
    }
    TxFrame frame = {.seq = tx_seq_++, .len = len, .payload = {}};
    std::memcpy(frame.payload, payload, len);
    if (xQueueSend(tx_queue_, &frame, 0) != pdTRUE) {
        TxFrame dropped = {};
        (void)xQueueReceive(tx_queue_, &dropped, 0);
        tx_queue_drops_++;
        (void)xQueueSend(tx_queue_, &frame, 0);
    }
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
    // Drain the RX FIFO fully on each RX_DONE. This is essential for the FLRC
    // burst image stream: at 2.6 Mbps a 511B fragment lands every ~2ms, faster
    // than the poll loop can service one interrupt, so multiple fragments pile
    // up in the 1024B RX FIFO before we get here. irq_pending_ is a bool, so N
    // back-to-back RX_DONEs collapse into a single poll_once pass; reading by
    // FIFO level until it drains keeps the read pointer aligned on boundaries.
    for (int drained = 0; ; drained++) {
        uint16_t level = 0;
        smtc_modem_hal_protect_api_call();
        ral_status_t level_status = static_cast<ral_status_t>(
            lr20xx_radio_fifo_get_rx_level(radio_.ral.context, &level));
        smtc_modem_hal_unprotect_api_call();

        if (level_status != RAL_STATUS_OK || level == 0) {
            break;
        }
        // Later iterations with a sub-fragment level mean the next fragment is
        // still arriving. The first iteration may legitimately read a short
        // control packet (or the remaining bytes reported by the FIFO).
        if (drained > 0 && level < APP_FLRC_MAX_PAYLOAD_BYTES) {
            break;
        }
        uint16_t take = (level >= APP_FLRC_MAX_PAYLOAD_BYTES)
                            ? APP_FLRC_MAX_PAYLOAD_BYTES : level;
        ral_flrc_rx_pkt_status_t pkt_status = {};

        smtc_modem_hal_protect_api_call();
        ral_status_t status = static_cast<ral_status_t>(
            lr20xx_radio_fifo_read_rx(radio_.ral.context, rx_buf_, take));
        if (status == RAL_STATUS_OK) {
            status = ral_get_flrc_rx_pkt_status(&radio_.ral, &pkt_status);
        }
        smtc_modem_hal_unprotect_api_call();

        if (status != RAL_STATUS_OK) {
            ESP_LOGW(TAG, "RX read failed: %d", status);
            break;
        }

        dispatch_rx_packet(take, pkt_status.rssi_sync_in_dbm);
    }
}

void RadioPing::dispatch_rx_packet(uint16_t len, int16_t rssi)
{
    if (len < kHeaderSize || std::memcmp(rx_buf_, kMagic, sizeof(kMagic)) != 0) {
        rx_unknown_packets_++;
        if ((rx_unknown_packets_ % 50U) == 1U) {
            // On the RX hot path; kept at DEBUG so it never floods the critical
            // receive loop (raise the log level to see it when debugging EMI).
            ESP_LOGD(TAG, "RX unknown packets=%lu len=%u rssi=%d hdr=%02x%02x%02x%02x",
                     static_cast<unsigned long>(rx_unknown_packets_), len, rssi,
                     rx_buf_[0], rx_buf_[1], rx_buf_[2], rx_buf_[3]);
        }
        return;
    }

    // Low power (node): any valid packet is "activity" — refresh the wake window
    // so the node stays in FLRC RX as long as the gateway keeps talking to it.
    // Only extends an already-open window (cad_wakeup_ms_ != 0); it never starts
    // one during CAD sleep. The window closes after APP_LP_WAKE_WINDOW_MS of
    // total silence (see the idle branch in poll_once).
    if (g_low_power_enabled && !is_gateway_ && cad_wakeup_ms_ != 0) {
        cad_wakeup_ms_ = smtc_modem_hal_get_time_in_ms();
    }

    if (intercom_active_ && rx_buf_[4] == kPacketTypeConfig && !is_gateway_ &&
        rx_buf_[8] == APP_CFG_KEY_INTERCOM &&
        static_cast<uint16_t>(get_u32_le(&rx_buf_[9])) == intercom_session_) {
        intercom_last_sync_ms_ = smtc_modem_hal_get_time_in_ms();
        send_config_ack(APP_CFG_KEY_INTERCOM, intercom_session_);
        return;
    }
    if (intercom_active_ && rx_buf_[4] != kPacketTypeVoice) {
        ESP_LOGD(TAG, "intercom ignored packet type=%u", rx_buf_[4]);
        return;
    }

    if (rx_buf_[4] == kPacketTypeVoice) {
        const uint16_t session = get_u16_le(&rx_buf_[8]);
        const uint8_t flags = rx_buf_[13] & kVoiceFlagMask;
        if (!intercom_active_ && flags == 0) {
            (void)queue_voice_packet(len, rssi);
            return;
        }
        if (!intercom_active_ && !is_gateway_ &&
            (flags & kVoiceFlagStop) != 0 &&
            session == intercom_last_stopped_session_) {
            intercom_session_ = session;
            (void)send_intercom_slot(kVoiceFlagNodeReply | kVoiceFlagStopAck);
            return;
        }
        if (!intercom_active_ || session != intercom_session_) {
            ESP_LOGD(TAG, "voice session mismatch rx=%u local=%u", session,
                     intercom_session_);
            return;
        }
        const bool valid = queue_voice_packet(len, rssi);
        if (valid && !is_gateway_ && (flags & kVoiceFlagMaster) != 0) {
            if ((flags & kVoiceFlagStart) != 0) {
                if (!intercom_start_confirmed_) {
                    ESP_LOGI(TAG, "intercom START received session=%u rssi=%d", session, rssi);
                }
                intercom_start_confirmed_ = true;
            }
            intercom_last_sync_ms_ = smtc_modem_hal_get_time_in_ms();
            intercom_reply_due_us_ = esp_timer_get_time() +
                static_cast<int64_t>(APP_INTERCOM_NODE_GUARD_US);
            intercom_reply_pending_ = true;
            intercom_stop_reply_ = (flags & kVoiceFlagStop) != 0;
            intercom_rx_slots_++;
        } else if (valid && is_gateway_ && (flags & kVoiceFlagNodeReply) != 0) {
            intercom_rx_slots_++;
            if (!intercom_start_confirmed_) {
                ESP_LOGI(TAG, "intercom first node reply session=%u rssi=%d", session, rssi);
            }
            intercom_start_confirmed_ = true;
            if ((flags & kVoiceFlagStopAck) != 0) {
                ESP_LOGI(TAG, "intercom STOP_ACK received session=%u", session);
                intercom_stop_confirmed_ = true;
            }
        }
    } else if (rx_buf_[4] == kPacketTypePing) {
        uint16_t seq = get_u16_le(&rx_buf_[6]);
        log_rx(seq, len, rssi);
    } else if (rx_buf_[4] == kPacketTypeImageCmd) {
        handle_image_cmd();
    } else if (rx_buf_[4] == kPacketTypeImageData) {
        image_rx_last_rssi_ = rssi;
        handle_image_data(len);
    } else if (rx_buf_[4] == kPacketTypeImageNack) {
        handle_image_nack();
    } else if (rx_buf_[4] == kPacketTypeImageDone) {
        handle_image_done();
    } else if (rx_buf_[4] == kPacketTypeImageEOT) {
        handle_image_eot();
    } else if (rx_buf_[4] == kPacketTypeImageStart) {
        handle_image_start(len);
    } else if (rx_buf_[4] == kPacketTypeImageCmdAck) {
        handle_image_cmd_ack();
    } else if (rx_buf_[4] == kPacketTypeConfig) {
        uint8_t key = rx_buf_[8];
        uint32_t value = get_u32_le(&rx_buf_[9]);
        ESP_LOGI(TAG, "RX Config: key=%u value=%lu", key, static_cast<unsigned long>(value));
#if APP_INTERCOM_AEC_ENABLE
        if (key == APP_CFG_KEY_INTERCOM && value != 0 && !is_gateway_ &&
            !echo_canceller_.ready()) {
            log_intercom_heap("node before AEC");
            if (!echo_canceller_.init()) {
                ESP_LOGE(TAG,
                         "intercom CONFIG rejected: node ESP-SR direct AEC init failed");
                return;
            }
            log_intercom_heap("node after AEC");
        }
#endif
        if (config_received_cb_) {
            config_received_cb_(key, value);
        }
        send_config_ack(key, value);
        if (key == APP_CFG_KEY_INTERCOM && value != 0 && !is_gateway_) {
            start_intercom_local(static_cast<uint16_t>(value));
        }
        // Low power: config applied + ACK sent, work done. End the wake window
        // so the main loop returns to CAD sleep on the next idle pass.
        if (g_low_power_enabled && !is_gateway_ && cad_wakeup_ms_ != 0) {
            cad_wakeup_ms_ = 0;
            ESP_LOGI(TAG, "config ACK sent, ending wake window -> CAD sleep");
        }
    } else if (rx_buf_[4] == kPacketTypeConfigAck) {
        ESP_LOGI(TAG, "RX ConfigAck");
        config_ack_received_ = true;
    } else if (rx_buf_[4] == kPacketTypeVbat) {
        // Battery voltage broadcast: [14..15] vbat_mv, [16..19] CRC32 over [0..15].
        if (len >= kHeaderSize + 6) {
            uint32_t hdr_crc = crc32_ieee(rx_buf_, 16);
            uint32_t rx_crc = get_u32_le(&rx_buf_[16]);
            if (hdr_crc == rx_crc) {
                uint16_t vbat_mv = get_u16_le(&rx_buf_[14]);
                ESP_LOGI(TAG, "RX Vbat: %u mV (%u.%02u V)", vbat_mv, vbat_mv / 1000, (vbat_mv % 1000) / 10);
                if (vbat_mv > 0 && vbat_received_cb_) vbat_received_cb_(vbat_mv);
            } else {
                ESP_LOGW(TAG, "RX Vbat CRC mismatch, dropping");
            }
        }
    } else {
        ESP_LOGW(TAG, "RX unsupported packet type=%u len=%u rssi=%d", rx_buf_[4], len, rssi);
    }
}

bool RadioPing::queue_voice_packet(uint16_t len, int16_t rssi)
{
    if (!voice_queue_ || len < kHeaderSize) return false;
    uint8_t frame_count = rx_buf_[12];
    const uint8_t flags = rx_buf_[13] & kVoiceFlagMask;
    if (frame_count == 0) {
        return flags != 0 && len == kHeaderSize;
    }
    if (frame_count > APP_FLRC_OPUS_FRAMES_PER_PACKET) {
        ESP_LOGW(TAG, "RX bad voice packet len=%u frames=%u", len, frame_count);
        return false;
    }

    uint16_t seq = get_u16_le(&rx_buf_[6]);
    uint16_t offset = kHeaderSize;
    for (uint8_t i = 0; i < frame_count; i++) {
        if (offset >= len) {
            ESP_LOGW(TAG, "RX truncated voice packet len=%u frames=%u", len, frame_count);
            return false;
        }

        uint8_t opus_len = rx_buf_[offset++];
        if (opus_len == 0 || opus_len > APP_OPUS_MAX_PACKET_BYTES || offset + opus_len > len) {
            ESP_LOGW(TAG, "RX bad voice frame len=%u opus_len=%u", len, opus_len);
            return false;
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
    return true;
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
            if (intercom_active_) {
                apply_intercom_playback_gain(rx_pcm_, static_cast<size_t>(decoded));
#if APP_INTERCOM_AEC_ENABLE
                // PLC audio is also written to I2S. Keep it in the far-end
                // reference timeline or every loss permanently shifts AEC.
                echo_canceller_.push_reference(rx_pcm_, static_cast<size_t>(decoded));
#endif
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
            if (!image_tx_active_) {
                //ESP_LOGW(TAG, "audio read failed: %s got=%u",
                         //esp_err_to_name(err), static_cast<unsigned>(got));
            }
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
    if (intercom_active_) {
        ESP_LOGW(TAG, "image capture blocked: intercom active=%d started=%d session=%u",
                 intercom_active_ ? 1 : 0, intercom_start_confirmed_ ? 1 : 0,
                 intercom_session_);
        return;
    }
    // UI and esp_timer callbacks must never touch the radio state directly.
    // Collapse duplicate triggers into one pending request and wake the radio
    // task; it owns all session teardown, mode changes and ImageCmd TX.
    s_image_stream_active.store(true, std::memory_order_release);
    image_capture_req_.store(true, std::memory_order_release);
    ESP_LOGI(TAG,
             "image capture queued: suspended=%d mode=%u req_active=%d rx_pending=%d "
             "abort_pending=%d",
             suspended_ ? 1 : 0, static_cast<unsigned>(mode_),
             image_req_active_ ? 1 : 0, image_rx_pending_ ? 1 : 0,
             image_rx_abort_req_.load(std::memory_order_acquire) ? 1 : 0);
    TaskHandle_t task = task_handle_;
    if (task != nullptr) {
        xTaskNotifyGive(task);
    }
}

void RadioPing::abort_image_rx()
{
    // Leaving the page cancels a queued next-frame request as well as the
    // current RX. The radio task performs the actual teardown.
    s_image_stream_active.store(false, std::memory_order_release);
    image_capture_req_.store(false, std::memory_order_release);
    image_rx_abort_req_.store(true, std::memory_order_release);
    TaskHandle_t task = task_handle_;
    if (task != nullptr) {
        xTaskNotifyGive(task);
    }
}

void RadioPing::check_image_capture_request()
{
    if (!image_capture_req_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }
    start_image_capture_request();
}

void RadioPing::start_image_capture_request()
{
    if (image_tx_active_) {
        ESP_LOGW(TAG, "image TX already active, ignoring trigger");
        return;
    }

    // A repeated UI press while a request/transfer is active must not replace its
    // session. The first camera wake + capture can take well over one second; a
    // second press during the following burst used to reset the reassembly state,
    // discard every fragment already on air, then report the old EOT as session/0.
    // Keep the current request alive. Its existing retry and 10s RX timeout paths
    // remain responsible for recovery if the peer really stops responding.
    if (image_rx_pending_ || image_req_active_) {
        uint16_t active_session = image_xfer_.rx_active()
                                      ? image_xfer_.rx_session_id()
                                      : image_req_session_;
        ESP_LOGI(TAG, "capture trigger ignored: request/RX active (session=%u received=%u/%u)",
                 active_session, image_xfer_.rx_received_count(),
                 image_xfer_.rx_total_count());
        return;
    }
    // Always discard any completed/stale reassembly metadata before assigning a
    // new request session. A late EOT from the previous session must not be able
    // to complete against the new request's pending state.
    image_xfer_.rx_reset();
    image_rx_expected_crc32_ = 0;
    image_rx_request_ms_ = 0;

    // Pick the session ONCE for this whole request. Retries reuse it (they do
    // NOT ++), so a resend can never spawn a second capture / a different JPEG.
    image_req_session_ = image_session_id_++;
    if (image_session_id_ == 0) {
        image_session_id_ = 1;
    }
    // Start the first request round. In low power this sends the LoRa wakeup +
    // FLRC reconfig, then floods ImageCmd every 30ms for the ~8s round to fill
    // the node's wake window. In non-low-power it just sends the first ImageCmd
    // (no wakeup) and floods continuously. check_image_req_retry (radio task
    // loop) drives the flood and starts fresh rounds until the node replies.
    image_req_active_ = true;
    image_req_debug_last_ms_ = smtc_modem_hal_get_time_in_ms();
    ESP_LOGI(TAG, "image request start: session=%u low_power=%d mode=%u",
             image_req_session_, g_low_power_enabled ? 1 : 0,
             static_cast<unsigned>(mode_));
    start_image_req_round();

    image_rx_pending_ = true;
    image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
    schedule_rx();
}

// Low power: begin a new request round. One LoRa wakeup preamble (~520ms) trips
// the node's CAD scan and opens its 8s FLRC wake window; then we flood ImageCmd
// every 30ms (check_image_req_retry) for the rest of the round instead of the
// old "one ImageCmd per second" that wasted most of the node's window. In
// non-low-power there is no CAD sleep, so no wakeup — just send the first
// ImageCmd and let the flood run continuously (round_end unused there).
void RadioPing::start_image_req_round()
{
    uint32_t now = smtc_modem_hal_get_time_in_ms();
    if (g_low_power_enabled) {
        if (!send_lora_wakeup()) {
            ESP_LOGE(TAG, "LoRa wakeup failed (round retry)");
            // Back off before the next wakeup attempt rather than spinning. Push
            // round_end out too so the round-timeout check in check_image_req_retry
            // doesn't re-fire immediately and defeat the backoff.
            uint32_t backoff = now + APP_IMAGE_REQ_RETRY_INTERVAL_LP_MS;
            image_req_round_end_ms_ = backoff;
            image_req_next_ms_ = backoff;
            return;
        }
        configure_flrc();
        image_req_round_end_ms_ = now + APP_IMAGE_REQ_ROUND_MS;
    }
    send_image_cmd_once();
    if (mode_ != Mode::rx_pending) {
        schedule_rx();
    }
    image_req_next_ms_ = smtc_modem_hal_get_time_in_ms() + APP_IMAGE_REQ_RETRY_INTERVAL_MS;
}

// Send one ImageCmd for image_req_session_ (build + TX only). The LoRa wakeup /
// FLRC reconfig is handled once per round by start_image_req_round.
void RadioPing::send_image_cmd_once()
{
    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeImageCmd;
    pkt[5] = 1;
    put_u16_le(&pkt[6], image_req_session_);
    put_u32_le(&pkt[8], smtc_modem_hal_get_time_in_ms());
    pkt[12] = 0;
    pkt[13] = 0;

    image_cmd_sent_ms_ = smtc_modem_hal_get_time_in_ms();

    // Stop RX before TX
    smtc_modem_hal_protect_api_call();
    if (mode_ == Mode::rx_pending) {
        (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        mode_ = Mode::idle;
    }
    smtc_modem_hal_unprotect_api_call();

    send_single_packet(pkt, kHeaderSize);
    // Keep the RX watchdog fresh so a long capture doesn't trip the 10s giveup.
    image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
}

// Runs in the radio task (next to poll_once), so send_image_cmd_once here shares
// the task with RX handling — no IRQ/mode races. Resends ImageCmd once per
// interval until the node acks or an ImageStart backstop clears image_req_active_.
void RadioPing::check_image_req_retry()
{
    if (!image_req_active_) return;
    // A transfer already started (ImageStart / data) — stop requesting.
    if (image_tx_active_) {
        image_req_active_ = false;
        return;
    }
    uint32_t now = smtc_modem_hal_get_time_in_ms();

    if (now - image_req_debug_last_ms_ >= 1000U) {
        image_req_debug_last_ms_ = now;
        ESP_LOGI(TAG,
                 "image request waiting: session=%u mode=%u suspended=%d rx_pending=%d "
                 "received=%u/%u",
                 image_req_session_, static_cast<unsigned>(mode_), suspended_ ? 1 : 0,
                 image_rx_pending_ ? 1 : 0, image_xfer_.rx_received_count(),
                 image_xfer_.rx_total_count());
    }

    // Low power: if the current round's flood window has elapsed with no
    // ImageStart, start a fresh round (new LoRa wakeup to re-open the node's
    // wake window). Rounds repeat with no cap until the node replies or the user
    // leaves the transfer page. Non-low-power has no rounds (round_end stays 0),
    // so this never fires and the flood is continuous.
    if (g_low_power_enabled && (int32_t)(now - image_req_round_end_ms_) >= 0) {
        ESP_LOGI(TAG, "ImageCmd round timed out, new wakeup round (session=%u)",
                 image_req_session_);
        start_image_req_round();
        return;
    }

    if ((int32_t)(now - image_req_next_ms_) < 0) return;

    // Flood cadence: one ImageCmd, then a short RX window to catch the node's
    // ImageStart. 30ms in both modes (non-harmonic with the node's 50ms
    // ImageStart retry so they interleave).
    send_image_cmd_once();
    if (mode_ != Mode::rx_pending) {
        schedule_rx();
    }
    image_req_next_ms_ = smtc_modem_hal_get_time_in_ms() + APP_IMAGE_REQ_RETRY_INTERVAL_MS;
}

void RadioPing::stop_image_req_retry()
{
    image_req_active_ = false;
}

// Takes ownership of `jpeg`: image_tx_task frees it (heap_caps_free) once the
// transfer finishes or aborts. The caller must NOT free it — the previous
// scheme (caller frees after a fixed 30s wait) raced the tx task's own 30s
// handshake budget and could free the buffer while a data burst was still
// reading from it (use-after-free). If the queue is full the request never
// reaches the task, so we free here to avoid leaking. `jpeg` must be a
// heap_caps allocation.
void RadioPing::send_image(const uint8_t *jpeg, size_t jpeg_len, uint16_t session_id)
{
    if (intercom_active_) {
        ESP_LOGW(TAG, "drop image while intercom is active");
        heap_caps_free(const_cast<uint8_t *>(jpeg));
        return;
    }
    if (!image_tx_queue_) {
        ESP_LOGE(TAG, "image_tx_queue not initialized");
        heap_caps_free(const_cast<uint8_t *>(jpeg));
        return;
    }
    ImageTxRequest req = { .jpeg = jpeg, .jpeg_len = jpeg_len, .session_id = session_id };
    if (xQueueSend(image_tx_queue_, &req, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "image_tx_queue full");
        heap_caps_free(const_cast<uint8_t *>(jpeg));
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
        s_image_tx_session.store(req.session_id, std::memory_order_release);
        image_done_received_ = false;
        image_nack_received_ = false;
        suspended_ = true;

        // We are about to reconfigure the radio to FLRC for the transfer. The
        // chip is leaving LoRa CAD, so drop the CAD-active flag now: otherwise
        // enter_low_power_cad() thinks CAD is still configured after the push
        // and skips the camera DVP power-down (low_power_standby_cb_), leaving
        // the camera powered — the node looks like it never re-sleeps.
        low_power_cad_active_ = false;

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

        // Step 0: Send ImageStart and wait for R to confirm ready. Persistent
        // (100 x 50ms = 5s) so it survives the gateway's ImageCmd flood: the
        // 50ms interval is non-harmonic with the gateway's 30ms ImageCmd retry,
        // so their phases drift and an ImageStart eventually lands in a gap. As
        // soon as one ImageStart reaches the gateway it stops sending ImageCmd
        // (backstop in handle_image_start), the channel clears, and the ready-ACK
        // gets through within a couple of tries.
        bool r_ready = false;
        for (uint16_t start_try = 0; start_try < APP_IMAGE_START_RETRY_COUNT && !r_ready; start_try++) {
            // ImageStart = 14-byte header + 2-byte vbat + 4-byte software CRC32
            // covering [0..15]. The gateway rejects any ImageStart whose CRC32
            // doesn't match, so a corrupted total_frags or vbat can't sneak past
            // the weak 2-byte FLRC CRC and poison the transfer (see handle_image_start).
            uint8_t start_pkt[kHeaderSize + 6];
            std::memcpy(start_pkt, kMagic, sizeof(kMagic));
            start_pkt[4] = kPacketTypeImageStart;
            start_pkt[5] = 1;
            put_u16_le(&start_pkt[6], req.session_id);
            put_u16_le(&start_pkt[8], total_fragments);
            put_u32_le(&start_pkt[10], jpeg_crc32);
            put_u16_le(&start_pkt[14], bsp_vbat_get_cached());  // battery voltage mV
            put_u32_le(&start_pkt[16], crc32_ieee(start_pkt, 16));
            send_single_packet(start_pkt, kHeaderSize + 6);

            image_nack_received_ = false;
            image_done_received_ = false;
            schedule_rx();

            uint32_t wait_start = smtc_modem_hal_get_time_in_ms();
            while (!image_nack_received_ && !image_done_received_) {
                if (smtc_modem_hal_get_time_in_ms() - wait_start > APP_IMAGE_START_RETRY_INTERVAL_MS) {
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
            //          start_try + 1, APP_IMAGE_START_RETRY_COUNT);
            }
        }

        if (!r_ready) {
            // ESP_LOGW(TAG, "image TX: R not ready, aborting");
            image_tx_active_ = false;
            s_image_tx_session.store(0, std::memory_order_release);
            suspended_ = false;
            ptt_active_ = was_ptt;
            if (g_low_power_enabled && !is_gateway_ && (cad_wakeup_ms_ != 0 || pir_push_wake_)) {
                cad_wakeup_ms_ = 0;
                pir_push_wake_ = false;
                ESP_LOGI(TAG, "image TX aborted, ending wake window -> CAD sleep");
            }
            if (!ptt_active_) schedule_rx();
            // We own req.jpeg (see send_image) — free before looping for the
            // next request, even on the abort path.
            heap_caps_free(const_cast<uint8_t *>(req.jpeg));
            req.jpeg = nullptr;
            continue;
        }

        // Step 1: Blast all fragments
        burst_send_fragments(req, total_fragments, nullptr, total_fragments);

        // ESP_LOGI(TAG, "image TX: initial burst done (%u frags)", total_fragments);

        // Step 2-8: EOT + wait ACK + retransmit loop
        bool transfer_done = false;
        // No-interaction abort: if we go APP_LP_WAKE_WINDOW_MS with no ACK/NACK
        // at all, the link is dead — stop hammering EOT and let the node fall
        // back to CAD sleep (in low power) / idle RX. Runs alongside the 400-round
        // cap; whichever trips first ends the transfer. Reset on every response.
        uint32_t last_interaction_ms = smtc_modem_hal_get_time_in_ms();
        for (uint16_t round = 0; round < APP_IMAGE_NACK_MAX_RETRIES && !transfer_done; round++) {
            if (smtc_modem_hal_get_time_in_ms() - last_interaction_ms > APP_LP_WAKE_WINDOW_MS) {
                ESP_LOGW(TAG, "image TX: no ACK/NACK for %ums, aborting transfer",
                         static_cast<unsigned>(APP_LP_WAKE_WINDOW_MS));
                break;
            }
            // Give R time to process last packets before sending EOT
            vTaskDelay(pdMS_TO_TICKS(10));

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
                // No ACK this round. Do NOT blindly resend the whole image —
                // that floods the channel and makes a bad RF environment worse.
                // Just advance to the next round, which re-sends EOT and waits
                // again. The initial burst already sent every fragment once;
                // actual retransmission only happens on an explicit NACK
                // missing-list below. The no-interaction abort at the top of the
                // loop ends things if this persists for the whole window.
                continue;
            }

            // Got a response — the link is alive, reset the no-interaction timer.
            last_interaction_ms = smtc_modem_hal_get_time_in_ms();

            if (image_done_received_ || nack_count_ == 0) {
                // ESP_LOGI(TAG, "image TX complete: all received");
                transfer_done = true;
                break;
            }

            // Retransmit missing fragments
            burst_send_fragments(req, total_fragments, nack_indices_, nack_count_);
        }

        image_tx_active_ = false;
        s_image_tx_session.store(0, std::memory_order_release);
        suspended_ = false;
        ptt_active_ = was_ptt;
        // ESP_LOGI(TAG, "image TX finished: session=%u done=%d",
        //          req.session_id, transfer_done ? 1 : 0);

        // Low power: work done, end the wake window(s) so the main loop returns
        // to CAD sleep on the next idle pass. cad_wakeup_ms_ = gateway-request RX
        // window; pir_push_wake_ = PIR self-push keep-awake guard. Clear whichever
        // brought us here.
        if (g_low_power_enabled && !is_gateway_ && (cad_wakeup_ms_ != 0 || pir_push_wake_)) {
            cad_wakeup_ms_ = 0;
            pir_push_wake_ = false;
            ESP_LOGI(TAG, "image TX done, ending wake window -> CAD sleep");
        }

        if (mode_ != Mode::rx_pending && !ptt_active_) {
            schedule_rx();
        }

        // Transfer complete (or gave up after the retransmit rounds). We own
        // req.jpeg (see send_image) — free it here so the buffer lives exactly
        // as long as the tx task needs it, with no dependency on the producer's
        // wait loop.
        heap_caps_free(const_cast<uint8_t *>(req.jpeg));
        req.jpeg = nullptr;
    }
}

uint16_t RadioPing::build_image_fragment(uint8_t *pkt, const ImageTxRequest &req,
                                         uint16_t frag_index, uint16_t total_fragments)
{
    size_t offset = static_cast<size_t>(frag_index) * APP_IMAGE_FRAGMENT_DATA_SIZE;
    uint16_t frag_len = static_cast<uint16_t>(
        ((offset + APP_IMAGE_FRAGMENT_DATA_SIZE) <= req.jpeg_len)
            ? APP_IMAGE_FRAGMENT_DATA_SIZE
            : (req.jpeg_len - offset));

    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeImageData;
    pkt[5] = 1;
    put_u16_le(&pkt[6], req.session_id);
    put_u16_le(&pkt[8], frag_index);
    put_u16_le(&pkt[10], total_fragments);
    put_u16_le(&pkt[12], frag_len);
    std::memcpy(&pkt[kHeaderSize], req.jpeg + offset, frag_len);
    uint16_t crc = crc16_ccitt(&pkt[4], kHeaderSize - 4 + frag_len);
    put_u16_le(&pkt[kHeaderSize + frag_len], crc);

    return static_cast<uint16_t>(kHeaderSize + frag_len + 2);
}

void RadioPing::burst_send_fragments(const ImageTxRequest &req, uint16_t total_fragments,
                                     const uint16_t *indices, uint16_t count)
{
    if (count == 0) {
        return;
    }

    // Preserve the existing retransmit guard: malformed NACK indices must not
    // address beyond the current JPEG buffer. Compact valid indices locally so
    // the FIFO prefill/refill loop still sees a dense sequence.
    uint16_t valid_indices[APP_IMAGE_NACK_MAX_INDICES];
    if (indices != nullptr) {
        uint16_t valid_count = 0;
        for (uint16_t i = 0; i < count; i++) {
            if (indices[i] < total_fragments) {
                valid_indices[valid_count++] = indices[i];
            }
        }
        if (valid_count == 0) {
            return;
        }
        indices = valid_indices;
        count = valid_count;
    }

    const void *ctx = radio_.ral.context;
    auto frag_at = [&](uint16_t pos) -> uint16_t {
        return indices ? indices[pos] : pos;
    };

    // Keep the target project's task-notification TX_DONE path: the image task
    // owns every IRQ for the complete burst while the main radio task is paused.
    tx_done_waiter_ = xTaskGetCurrentTaskHandle();
    xTaskNotifyStateClear(nullptr);

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);
    (void)ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    (void)lr20xx_radio_common_set_rx_tx_fallback_mode(ctx, LR20XX_RADIO_FALLBACK_FS);
    (void)lr20xx_radio_fifo_clear_tx(ctx);
    smtc_modem_hal_unprotect_api_call();

    uint8_t pkt[APP_FLRC_MAX_PAYLOAD_BYTES];
    uint16_t next_write = 0;
    auto build_padded = [&](uint16_t pos) {
        uint16_t len = build_image_fragment(pkt, req, frag_at(pos), total_fragments);
        if (len < APP_FLRC_BURST_PAYLOAD_LEN) {
            std::memset(pkt + len, 0, APP_FLRC_BURST_PAYLOAD_LEN - len);
        }
    };

    // A 1024-byte FIFO holds one packet in flight and one queued packet.
    for (int prefill = 0; prefill < 2 && next_write < count; prefill++) {
        build_padded(next_write);
        smtc_modem_hal_protect_api_call();
        (void)lr20xx_radio_fifo_write_tx(ctx, pkt, APP_FLRC_BURST_PAYLOAD_LEN);
        smtc_modem_hal_unprotect_api_call();
        next_write++;
    }

    mode_ = Mode::tx_pending;
    smtc_modem_hal_protect_api_call();
    (void)ral_set_tx(&radio_.ral);
    smtc_modem_hal_unprotect_api_call();

    for (uint16_t pos = 1; pos < count; pos++) {
        (void)wait_for_tx_done(50);

        mode_ = Mode::tx_pending;
        smtc_modem_hal_protect_api_call();
        (void)ral_set_tx(&radio_.ral);
        if (next_write < count) {
            build_padded(next_write);
            (void)lr20xx_radio_fifo_write_tx(ctx, pkt, APP_FLRC_BURST_PAYLOAD_LEN);
            next_write++;
        }
        smtc_modem_hal_unprotect_api_call();
    }

    (void)wait_for_tx_done(50);

    smtc_modem_hal_protect_api_call();
    (void)lr20xx_radio_common_set_rx_tx_fallback_mode(
        ctx, LR20XX_RADIO_FALLBACK_STDBY_XOSC);
    smtc_modem_hal_unprotect_api_call();
    mode_ = Mode::idle;
    tx_done_waiter_ = nullptr;
}

bool RadioPing::send_single_packet(const uint8_t *data, uint16_t len,
                                   uint32_t timeout_ms)
{
    // Register this task as the TX_DONE notify target and drop any stale
    // notification BEFORE arming TX, so a fast TX_DONE (which can fire the
    // instant set_tx runs) still lands as a pending notification wait_for_tx_done
    // will consume — no lost-wakeup window.
    tx_done_waiter_ = xTaskGetCurrentTaskHandle();
    xTaskNotifyStateClear(nullptr);

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);
    ral_status_t status = ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    if (status == RAL_STATUS_OK) status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    if (status == RAL_STATUS_OK) status = ral_set_pkt_payload(&radio_.ral, data, len);
    if (status == RAL_STATUS_OK) status = ral_set_tx(&radio_.ral);
    smtc_modem_hal_unprotect_api_call();

    if (status != RAL_STATUS_OK) {
        tx_done_waiter_ = nullptr;
        return false;
    }

    mode_ = Mode::tx_pending;
    bool ok = wait_for_tx_done(timeout_ms);
    tx_done_waiter_ = nullptr;
    return ok;
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
        // Block instead of spinning: the TX_DONE ISR notifies this task
        // (tx_done_waiter_), so the CPU is freed for the ~1.7ms of packet
        // airtime instead of burning it in a taskYIELD loop. The short poll
        // timeout is a safety net — if a notification is ever missed we still
        // re-check irq_pending_ / the hardware IRQ status and the overall
        // timeout_ms guard, so we can never dead-wait here.
        ulTaskNotifyTake(pdTRUE, ms_to_ticks_min_1(APP_RADIO_TASK_POLL_MS));
    }
}

void RadioPing::handle_image_cmd()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    // ESP_LOGI(TAG, "RX ImageCmd: session=%u", session_id);

    // Ack immediately (before the slow JPEG capture) so the gateway stops
    // resending ImageCmd. This is sent on EVERY ImageCmd — including gateway
    // retransmits of the same session (their ack was lost) — so a lost ack
    // self-heals. The capture-side dedup (image_capture_cb_) drops the duplicate
    // dispatch, so re-acking never triggers a second capture.
    // Ask the app whether it accepts this request BEFORE acking. The callback
    // only spawns the capture task (non-blocking, does not touch the radio here;
    // the task reconfigures FLRC ~110ms later), so running it while still in
    // rx_pending is safe. We ack only on acceptance: if the node is busy (e.g. a
    // stream's next-frame request landing in the brief post-TX busy tail) it
    // returns false and we stay silent, so the gateway's ImageCmd flood keeps
    // going and the node accepts as soon as it is free — no deadlock. A
    // same-session retransmit whose earlier ack was lost returns true again, so
    // lost acks still self-heal without launching a second capture.
    bool accepted = true;
    if (image_capture_cb_) {
        accepted = image_capture_cb_(session_id);
    }

    if (session_id != image_cmd_debug_session_) {
        image_cmd_debug_session_ = session_id;
        ESP_LOGI(TAG,
                 "RX ImageCmd: session=%u accepted=%d intercom=%d image_tx=%d mode=%u",
                 session_id, accepted ? 1 : 0, intercom_active_ ? 1 : 0,
                 image_tx_active_ ? 1 : 0, static_cast<unsigned>(mode_));
    }

    if (accepted) {
        uint8_t ack[kHeaderSize];
        std::memcpy(ack, kMagic, sizeof(kMagic));
        ack[4] = kPacketTypeImageCmdAck;
        ack[5] = 1;
        put_u16_le(&ack[6], session_id);
        ack[8] = 0; ack[9] = 0; ack[10] = 0; ack[11] = 0; ack[12] = 0; ack[13] = 0;

        if (mode_ == Mode::rx_pending) {
            smtc_modem_hal_protect_api_call();
            (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            smtc_modem_hal_unprotect_api_call();
            mode_ = Mode::idle;
        }
        send_single_packet(ack, kHeaderSize);
    }
    schedule_rx();
}

// Gateway side: node acknowledged the ImageCmd. Stop the request-retry timer.
void RadioPing::handle_image_cmd_ack()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    if (session_id != image_req_session_) {
        return;
    }
    ESP_LOGI(TAG, "RX ImageCmdAck: session=%u request_age=%lums", session_id,
             static_cast<unsigned long>(smtc_modem_hal_get_time_in_ms() -
                                        image_cmd_sent_ms_));
    stop_image_req_retry();
    schedule_rx();
}

void RadioPing::handle_image_start(uint16_t len)
{
    // ImageStart carries the fragment count that sizes the whole transfer, so a
    // single corrupted total_frags wrecks it (e.g. a bogus 0/4778). The FLRC
    // hardware CRC is only 2 bytes and, in variable-length mode, does not cover
    // the length byte — a rare bad packet can slip through. So ImageStart adds
    // its own CRC32 over the header + vbat ([0..15]), appended as 4 bytes.
    // Recompute and compare; on mismatch drop the packet entirely (touch no
    // state) — the node keeps resending ImageStart, so we just wait for a
    // clean one, same as any lost packet.
    //
    // Backward compat: old nodes send 18 bytes (no vbat, CRC over [0..13]);
    // new nodes send 20 bytes (vbat at [14..15], CRC over [0..15]).
    bool has_vbat = (len >= kHeaderSize + 6);
    uint16_t crc_len = has_vbat ? 16 : kHeaderSize;
    uint16_t crc_offset = has_vbat ? 16 : kHeaderSize;

    if (len < crc_offset + 4) {
        ESP_LOGW(TAG, "ImageStart too short (len=%u), dropping", len);
        return;
    }
    uint32_t hdr_crc = crc32_ieee(rx_buf_, crc_len);
    uint32_t rx_hdr_crc = get_u32_le(&rx_buf_[crc_offset]);
    if (hdr_crc != rx_hdr_crc) {
        ESP_LOGW(TAG, "ImageStart header CRC32 mismatch (calc=0x%08lx rx=0x%08lx), dropping",
                 static_cast<unsigned long>(hdr_crc), static_cast<unsigned long>(rx_hdr_crc));
        return;
    }

    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    uint16_t total_frags = get_u16_le(&rx_buf_[8]);
    uint32_t expected_crc32 = get_u32_le(&rx_buf_[10]);
    uint16_t vbat_mv = has_vbat ? get_u16_le(&rx_buf_[14]) : 0;

    const bool first_start_for_session =
        !image_xfer_.rx_active() || image_xfer_.rx_session_id() != session_id;
    if (first_start_for_session) {
        ESP_LOGI(TAG,
                 "RX ImageStart: session=%u total=%u req_session=%u req_active=%d "
                 "rx_pending=%d mode=%u",
                 session_id, total_frags, image_req_session_, image_req_active_ ? 1 : 0,
                 image_rx_pending_ ? 1 : 0, static_cast<unsigned>(mode_));
    }

    // While a gateway request is waiting for ImageStart, only that request's
    // session may start the transfer. Once reassembly is active, only a repeat
    // ImageStart for the same RX session is valid. This keeps a delayed
    // unsolicited/previous-session ImageStart from stopping the new ImageCmd
    // retry and hijacking the first frame.
    if (image_rx_pending_) {
        uint16_t expected_session = image_req_active_
                                        ? image_req_session_
                                        : (image_xfer_.rx_active()
                                               ? image_xfer_.rx_session_id()
                                               : image_req_session_);
        if (expected_session != 0 && session_id != expected_session) {
            ESP_LOGW(TAG, "ImageStart ignored: session=%u expected=%u",
                     session_id, expected_session);
            return;
        }
    }
    bool gateway_requested = image_rx_pending_ && session_id == image_req_session_;

    if (has_vbat) {
        // Start of a frame's RX — DEBUG so the per-frame transfer stays quiet;
        // the completion summary ("RX done") carries the useful outcome.
        ESP_LOGD(TAG, "RX ImageStart: session=%u total=%u crc32=0x%08lx vbat=%u mV (%u.%02u V)",
                 session_id, total_frags, static_cast<unsigned long>(expected_crc32),
                 vbat_mv, vbat_mv / 1000, (vbat_mv % 1000) / 10);
        // NOTE: deliberately NOT calling vbat_received_cb_ here. This is the
        // per-frame image RX hot path and the callback grabs the LVGL render
        // lock (s_lock) with portMAX_DELAY; if the LVGL task is mid-flush on the
        // other core it can block the radio task ~100ms, delaying the ready-ACK
        // and inflating prepare_ms. Battery voltage is instead delivered by the
        // low-power node inside this same ImageStart packet only for logging,
        // and by the non-low-power node via the periodic kPacketTypeVbat
        // broadcast (see line ~2476), which updates the UI off the hot path.
    } else {
        ESP_LOGD(TAG, "RX ImageStart: session=%u total=%u crc32=0x%08lx",
                 session_id, total_frags, static_cast<unsigned long>(expected_crc32));
    }

    // Backstop: an ImageStart proves the node got our request, so stop resending
    // ImageCmd even if its ImageCmdAck was lost.
    stop_image_req_retry();

    // Reentry guard against progress rollback: if we're already receiving this
    // same session and have taken in at least one fragment, this is a stale or
    // duplicate ImageStart (the node only sends it during its pre-data handshake).
    // Re-running rx_begin here would wipe received fragments and snap the UI back
    // to 0. Drop it. (During the handshake, received==0, so re-begin is harmless
    // and we fall through to re-send the ready-ACK so the node proceeds.)
    if (image_rx_pending_ &&
        session_id == image_xfer_.rx_session_id() &&
        image_xfer_.rx_received_count() > 0) {
        return;
    }

    image_rx_start_ms_ = smtc_modem_hal_get_time_in_ms();
    // Bind the timing origin to this RX session. Unsolicited node pushes have
    // no ImageCmd preparation phase, so their request origin is ImageStart.
    image_rx_request_ms_ = gateway_requested ? image_cmd_sent_ms_ : image_rx_start_ms_;

    // Prepare RX buffer
    image_xfer_.rx_begin(session_id, total_frags);
    image_rx_pending_ = true;
    image_rx_nack_sent_ = 0;
    image_rx_eot_count_ = 0;
    image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
    image_rx_last_progress_ms_ = smtc_modem_hal_get_time_in_ms();
    image_rx_expected_crc32_ = expected_crc32;

    // Send the ready-ACK FIRST (missing_count=0 means "ready"). The node starts
    // its data burst the instant it sees this, so prepare_ms is measured from
    // here — nothing that can block (LVGL lock, etc.) may run before this send.
    // The old code updated the UI first "so LVGL work finishes before the node
    // blasts data", but ui_gw_rx_begin takes the LVGL render lock with
    // portMAX_DELAY; when the LVGL task was mid-flush on the other core it
    // stalled the radio task ~100ms and inflated prepare_ms (same failure mode
    // as the vbat hot-path note above). Audio UI raised the flush rate and made
    // it frequent again. Fix: ACK first, then re-arm RX, then best-effort UI.
    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeImageNack;
    pkt[5] = 3;
    put_u16_le(&pkt[6], session_id);
    put_u16_le(&pkt[8], 0);  // missing_count = 0 (ready signal)
    put_u16_le(&pkt[10], 0); // total_received = 0
    pkt[12] = 0; pkt[13] = 0;
    send_single_packet(pkt, kHeaderSize);

    // Enter RX for incoming data immediately after the ACK, before any UI work,
    // so we are listening before the node's first fragment can arrive.
    schedule_rx();

    // Update UI last, off the critical path. The callback acquires the LVGL lock
    // non-blocking (see ui_gw_rx_begin / ui_gw_rx_progress); if LVGL is mid-flush
    // the update is skipped and the next per-fragment progress refresh catches up.
    if (image_rx_progress_cb_) {
        image_rx_progress_cb_(0, total_frags, 0);
    }
}

void RadioPing::handle_image_data(uint16_t len)
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

    if (len < static_cast<uint16_t>(kHeaderSize + frag_len + 2)) {
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
    uint16_t active_session = s_image_tx_session.load(std::memory_order_acquire);
    if (active_session == 0 || session_id != active_session) {
        return;
    }

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
        uint16_t next_session = get_u16_le(&rx_buf_[12]);
        if (next_session != 0 && next_session != session_id) {
            s_chained_capture_session.store(next_session, std::memory_order_release);
            ESP_LOGD(TAG, "chained capture queued: current=%u next=%u",
                     session_id, next_session);
        }
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

    // The final ACK may be lost after the gateway has already armed the chained
    // next session. Re-ACK the completed session with the SAME next-session ID;
    // otherwise the node would remain in its EOT retry loop while the gateway
    // waits for the next ImageStart.
    if (session_id == image_rx_done_session_ &&
        (!image_rx_pending_ || !image_xfer_.rx_active() ||
         session_id != image_xfer_.rx_session_id())) {
        uint8_t pkt[kHeaderSize];
        std::memcpy(pkt, kMagic, sizeof(kMagic));
        pkt[4] = kPacketTypeImageNack;
        pkt[5] = 3;
        put_u16_le(&pkt[6], session_id);
        put_u16_le(&pkt[8], 0);
        put_u16_le(&pkt[10], 0);
        uint16_t repeated_next =
            s_image_stream_active.load(std::memory_order_acquire)
                ? s_image_rx_done_next_session
                : 0;
        put_u16_le(&pkt[12], repeated_next);
        send_single_packet(pkt, kHeaderSize);
        schedule_rx();
        return;
    }

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

    // A late EOT must never operate on the pending state or reassembly buffer
    // of another session. In particular, while a new request is waiting for its
    // ImageStart the buffer is inactive and all old EOT packets are ignored.
    if (!image_xfer_.rx_active() ||
        session_id != image_xfer_.rx_session_id() ||
        total_frags != image_xfer_.rx_total_count()) {
        ESP_LOGW(TAG, "ImageEOT ignored: session=%u/%u total=%u/%u",
                 session_id, image_xfer_.rx_session_id(),
                 total_frags, image_xfer_.rx_total_count());
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
            ESP_LOGW(TAG, "image RX crc32 mismatch: expected=0x%08lx actual=0x%08lx, requesting full resend",
                     static_cast<unsigned long>(image_rx_expected_crc32_),
                     static_cast<unsigned long>(actual_crc32));
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

    uint16_t next_session = 0;
    if (missing_count == 0 &&
        s_image_stream_active.load(std::memory_order_acquire)) {
        next_session = image_session_id_++;
        if (image_session_id_ == 0) {
            image_session_id_ = 1;
        }
    }

    put_u16_le(&pkt[8], missing_count);
    put_u16_le(&pkt[10], image_xfer_.rx_received_count());
    put_u16_le(&pkt[12], next_session);

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
        s_image_rx_done_next_session = next_session;
        uint32_t now_ms = smtc_modem_hal_get_time_in_ms();
        uint32_t transfer_ms = now_ms - image_rx_start_ms_;
        int32_t prepare_delta =
            static_cast<int32_t>(image_rx_start_ms_ - image_rx_request_ms_);
        uint32_t prepare_ms =
            prepare_delta >= 0 ? static_cast<uint32_t>(prepare_delta) : 0U;
        uint32_t total_ms = prepare_ms + transfer_ms;
        image_rx_transfer_ms_ = transfer_ms;
        image_rx_done_ms_ = now_ms;
        ESP_LOGI(TAG, "RX done | prepare=%lums transfer=%lums total=%lums",
                 static_cast<unsigned long>(prepare_ms),
                 static_cast<unsigned long>(transfer_ms),
                 static_cast<unsigned long>(total_ms));
        if (image_rx_complete_cb_) {
            image_rx_complete_cb_(&image_xfer_);
        }

        // Arm the gateway for the piggybacked session without transmitting an
        // ImageCmd. If no ImageStart arrives, the existing request retry sends
        // the same session after one normal 30 ms interval.
        if (next_session != 0 &&
            s_image_stream_active.load(std::memory_order_acquire)) {
            image_req_session_ = next_session;
            image_req_active_ = true;
            image_req_next_ms_ = now_ms + APP_IMAGE_REQ_RETRY_INTERVAL_MS;
            image_req_round_end_ms_ = now_ms + APP_IMAGE_REQ_ROUND_MS;
            image_cmd_sent_ms_ = now_ms;
            image_rx_request_ms_ = now_ms;
            image_rx_pending_ = true;
            image_rx_last_frag_ms_ = now_ms;
            ESP_LOGD(TAG, "stream chained: completed=%u next=%u",
                     session_id, next_session);
        }
    } else {
        // On the RX/NACK hot path — this fires once per retransmit round mid
        // transfer. Kept at DEBUG so it does not stall reception; the per-frame
        // outcome is still visible in the "RX done" line after completion.
        ESP_LOGI(TAG, "image RX: sent ACK with %u missing (first=%u), waiting for retransmit",
                 missing_count, missing_indices[0]);
        if (image_rx_eot_count_ == 1) {
            for (uint16_t i = 0; i < missing_count; i += 16) {
                char line[128];
                int pos = 0;
                for (uint16_t j = i; j < missing_count && j < i + 16; j++) {
                    pos += snprintf(line + pos, sizeof(line) - pos, "%u ", missing_indices[j]);
                }
                ESP_LOGD(TAG, "  missing: %s", line);
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
                ESP_LOGW(TAG, "image RX complete timeout crc32 mismatch: expected=0x%08lx actual=0x%08lx",
                         static_cast<unsigned long>(image_rx_expected_crc32_),
                         static_cast<unsigned long>(actual_crc32));
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

    ESP_LOGW(TAG,
             "image RX timeout: session=%u received=%u/%u req_active=%d mode=%u "
             "intercom=%d suspended=%d",
             image_xfer_.rx_session_id(), image_xfer_.rx_received_count(),
             image_xfer_.rx_total_count(), image_req_active_ ? 1 : 0,
             static_cast<unsigned>(mode_), intercom_active_ ? 1 : 0,
             suspended_ ? 1 : 0);
    image_rx_pending_ = false;
    image_xfer_.rx_reset();
    image_rx_nack_sent_ = 0;
}

// Runs in the radio task. Tears down an in-progress image RX when the UI thread
// requested an abort (user left the transfer page). Clears both the RX state
// and the ImageCmd request-retry so the gateway stops sending/expecting anything
// for this session; the node's TX side self-aborts once its ACKs stop arriving.
void RadioPing::check_image_rx_abort()
{
    if (!image_rx_abort_req_.exchange(false, std::memory_order_acq_rel)) return;

    image_capture_req_.store(false, std::memory_order_release);
    ESP_LOGI(TAG,
             "image RX abort consumed: req_active=%d rx_pending=%d session=%u "
             "received=%u/%u mode=%u intercom=%d",
             image_req_active_ ? 1 : 0, image_rx_pending_ ? 1 : 0,
             image_xfer_.rx_session_id(), image_xfer_.rx_received_count(),
             image_xfer_.rx_total_count(), static_cast<unsigned>(mode_),
             intercom_active_ ? 1 : 0);
    image_req_active_ = false;
    image_rx_pending_ = false;
    image_xfer_.rx_reset();
    image_rx_nack_sent_ = 0;
    schedule_rx();
}

bool RadioPing::send_config(uint8_t key, uint32_t value)
{
    if (intercom_active_ && key != APP_CFG_KEY_INTERCOM) {
        ESP_LOGW(TAG, "config key=%u rejected while intercom is active", key);
        return false;
    }
    ESP_LOGI(TAG, "send_config: key=%u value=%lu", key, static_cast<unsigned long>(value));

    suspended_ = true;
    const uint32_t ack_timeout_ms =
        (key == APP_CFG_KEY_INTERCOM && value != 0)
            ? APP_INTERCOM_START_TIMEOUT_MS
            : kConfigAckTimeoutMs;

    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeConfig;
    pkt[5] = 1;
    put_u16_le(&pkt[6], 0);
    pkt[8] = key;
    put_u32_le(&pkt[9], value);
    pkt[13] = 0;

    // Wake the node with a long LoRa preamble before sending, since it sleeps
    // in CAD standby. Always do this for the LOW_POWER key itself (regardless of
    // the gateway's current belief about node state) so the on/off command is
    // guaranteed to reach the node and the two sides can't get stuck desynced.
    if (g_low_power_enabled || key == APP_CFG_KEY_LOW_POWER) {
        uint32_t t0 = smtc_modem_hal_get_time_in_ms();
        if (!send_lora_wakeup()) {
            ESP_LOGE(TAG, "LoRa wakeup failed for config");
            suspended_ = false;
            if (!ptt_active_) schedule_rx();
            return false;
        }
        uint32_t elapsed = smtc_modem_hal_get_time_in_ms() - t0;
        ESP_LOGI(TAG, "LoRa wakeup preamble TX took %lu ms (config)", (unsigned long)elapsed);
        configure_flrc();
    }

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
            if (smtc_modem_hal_get_time_in_ms() - wait_start > ack_timeout_ms) {
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

bool RadioPing::configure_lora_cad()
{
    const void *ctx = radio_.ral.context;

    lr20xx_radio_common_set_pkt_type(ctx, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);
    lr20xx_radio_common_set_rf_freq(ctx, APP_FLRC_FREQUENCY_HZ);

    lr20xx_radio_lora_mod_params_t mod = {};
    mod.sf = LR20XX_RADIO_LORA_SF7;
    mod.bw = LR20XX_RADIO_LORA_BW_125;
    mod.cr = LR20XX_RADIO_LORA_CR_4_5;
    mod.ppm = LR20XX_RADIO_LORA_NO_PPM;
    if (lr20xx_radio_lora_set_modulation_params(ctx, &mod) != LR20XX_STATUS_OK) {
        ESP_LOGE(TAG, "lora mod params failed");
        return false;
    }

    lr20xx_radio_lora_cad_params_t cad = {};
    cad.cad_symb_nb = 2;
    cad.pnr_delta = 0;
    cad.cad_exit_mode = LR20XX_RADIO_LORA_CAD_EXIT_MODE_STANDBYRC;
    cad.cad_timeout_in_pll_step = 0;
    cad.cad_detect_peak = 56;
    if (lr20xx_radio_lora_configure_cad_params(ctx, &cad) != LR20XX_STATUS_OK) {
        ESP_LOGE(TAG, "lora cad params failed");
        return false;
    }

    return true;
}

bool RadioPing::low_power_sleep(uint32_t ms)
{
    // Flush the console so the last log line isn't truncated when clocks stop.
    // Note: the USB Serial/JTAG console does not survive light sleep, so serial
    // output stops during CAD standby — this is expected. Run on battery/adapter.
    fflush(stdout);

    esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000ULL);

    // ESP32-S3 light-sleep GPIO wake ONLY supports level mode; edge mode
    // (GPIO_INTR_POSEDGE) is rejected at runtime with:
    //   "GPIO wakeup only supports level mode, but edge mode set"
    // so we arm GPIO_INTR_HIGH_LEVEL — the same trigger type the PIR ISR uses.
    // We only arm it when pir_armed_ is set: after a detection the ISR disables
    // the trigger and clears pir_armed_ for the 15s cooldown, during which the
    // still-high PIR pin must NOT wake us. The GPIO ISR (also HIGH_LEVEL) is
    // already installed, so on wake it fires, disables the source, sets
    // pir_triggered_, and starts the re-arm timer — we don't touch the trigger
    // config here, we only supply the light-sleep wake source.
    bool pir_wake = pir_enabled_ && pir_armed_;
    if (pir_wake) {
        gpio_wakeup_enable(APP_PIR_GPIO, GPIO_INTR_HIGH_LEVEL);
        esp_sleep_enable_gpio_wakeup();
    }

    esp_light_sleep_start();

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    bool woken_by_pir = false;

    if (pir_wake) {
        gpio_wakeup_disable(APP_PIR_GPIO);
        if (cause == ESP_SLEEP_WAKEUP_GPIO) {
            // The HIGH_LEVEL GPIO ISR fires on resume and handles the trigger
            // (disable source, set pir_triggered_, start 15s re-arm). We just
            // report the PIR wake so the caller keeps the node awake to push.
            woken_by_pir = true;
            ESP_LOGI(TAG, "light sleep: woken by PIR");
        }
    }
    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        ESP_LOGD(TAG, "light sleep: timer wake");
    }

    return woken_by_pir;
}

void RadioPing::notify_capture_starting()
{
    // Only meaningful for a low-power node (the gateway never CAD-sleeps). Set
    // the keep-awake guard so the next poll_once idle pass does NOT drop into
    // CAD light sleep while the capture + push runs.
    if (!g_low_power_enabled || is_gateway_) return;
    pir_push_wake_ = true;
    pir_push_wake_ms_ = smtc_modem_hal_get_time_in_ms();
}

void RadioPing::enter_low_power_cad()
{
    if (mode_ != Mode::idle) return;

    const void *ctx = radio_.ral.context;

    if (!low_power_cad_active_) {
        smtc_modem_hal_protect_api_call();
        if (mode_ == Mode::rx_pending) {
            ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            mode_ = Mode::idle;
        }
        smtc_modem_hal_unprotect_api_call();
        low_power_cad_active_ = true;
        ESP_LOGI(TAG, "entering low power CAD mode");
        // Release power-hungry peripherals (camera DVP). capture_frame() rebuilds
        // the camera on demand, so no explicit restore is needed on wake.
        if (low_power_standby_cb_) {
            low_power_standby_cb_(true);
        }
    }

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();

    if (!configure_lora_cad()) {
        smtc_modem_hal_unprotect_api_call();
        ESP_LOGE(TAG, "CAD config failed, fallback to FLRC RX");
        low_power_cad_active_ = false;
        configure_flrc();
        schedule_rx();
        return;
    }

    ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_CAD_DONE | RAL_IRQ_CAD_OK);
    lr20xx_radio_lora_set_cad(ctx);
    smtc_modem_hal_unprotect_api_call();

    cad_pending_ms_ = smtc_modem_hal_get_time_in_ms();
    mode_ = Mode::cad_pending;
}

void RadioPing::handle_cad_irq(ral_irq_t irq)
{
    mode_ = Mode::idle;
    cad_pending_ms_ = 0;

    if ((irq & RAL_IRQ_CAD_DONE) == 0) {
        ESP_LOGW(TAG, "CAD unexpected irq=0x%08lx", static_cast<unsigned long>(irq));
        return;
    }

    ESP_LOGI(TAG, "CAD done: %s", (irq & RAL_IRQ_CAD_OK) ? "activity detected" : "channel clear");

    if ((irq & RAL_IRQ_CAD_OK) != 0) {
        ESP_LOGI(TAG, "CAD detected activity, switching to FLRC RX (activity-refreshed window)");
        low_power_cad_active_ = false;
        cad_wakeup_ms_ = smtc_modem_hal_get_time_in_ms();
        smtc_modem_hal_protect_api_call();
        configure_flrc();
        smtc_modem_hal_unprotect_api_call();
        schedule_rx();
    } else {
        const void *ctx = radio_.ral.context;
        lr20xx_system_sleep_cfg_t sleep_cfg = {};
        sleep_cfg.is_clk_32k_enabled = 1;
        sleep_cfg.is_ram_retention_enabled = 1;
        smtc_modem_hal_protect_api_call();
        lr20xx_system_set_sleep_mode(ctx, &sleep_cfg, 0);
        smtc_modem_hal_unprotect_api_call();

        // LR2021 is now asleep and SPI is idle, so light-sleep the ESP32 too
        // for the 500ms CAD off-period. Wakes on timer (next CAD) or PIR.
        bool woken_by_pir = low_power_sleep(500);

        if (woken_by_pir) {
            // PIR is a self-initiated PUSH: the node is the transmitter, exactly
            // like the non-low-power PIR path in tx_task, which touches the radio
            // ZERO times — it only calls image_capture_cb_. image_tx_task then
            // takes over the radio entirely (its own configure_flrc + ImageStart
            // + schedule_rx for the ACK). So here we must NOT touch the radio: no
            // TCXO, no configure_flrc, and definitely no schedule_rx (we are not
            // waiting for anyone). The ONE thing we need is to stop the loop from
            // dropping back into CAD sleep before tx_task fires the capture. The
            // pir_push_wake_ guard keeps the loop awake-but-idle for that; the
            // radio stays asleep (RADIO_SLEEP) until image_tx_task's first SPI
            // access, which robustly wakes it via the HAL's improved sleep-branch
            // retry (NSS glitch every 10ms until BUSY drops).
            pir_push_wake_ = true;
            pir_push_wake_ms_ = smtc_modem_hal_get_time_in_ms();
            ESP_LOGI(TAG, "PIR wake: staying awake, capture will push image");
        }
    }
}

bool RadioPing::send_lora_wakeup()
{
    const void *ctx = radio_.ral.context;

    ESP_LOGI(TAG, "sending LoRa wakeup (508 symbol preamble)");

    smtc_modem_hal_protect_api_call();
    if (mode_ == Mode::rx_pending || mode_ == Mode::cad_pending) {
        ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        cad_pending_ms_ = 0;
        mode_ = Mode::idle;
    }
    smtc_modem_hal_unprotect_api_call();

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);

    lr20xx_radio_common_set_pkt_type(ctx, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);
    lr20xx_radio_common_set_rf_freq(ctx, APP_FLRC_FREQUENCY_HZ);

    lr20xx_radio_lora_mod_params_t mod = {};
    mod.sf = LR20XX_RADIO_LORA_SF7;
    mod.bw = LR20XX_RADIO_LORA_BW_125;
    mod.cr = LR20XX_RADIO_LORA_CR_4_5;
    mod.ppm = LR20XX_RADIO_LORA_NO_PPM;
    lr20xx_radio_lora_set_modulation_params(ctx, &mod);

    lr20xx_radio_lora_pkt_params_t pkt = {};
    pkt.preamble_len_in_symb = 508;
    pkt.pkt_mode = LR20XX_RADIO_LORA_PKT_EXPLICIT;
    pkt.pld_len_in_bytes = 4;
    pkt.crc = LR20XX_RADIO_LORA_CRC_ENABLED;
    pkt.iq = LR20XX_RADIO_LORA_IQ_STANDARD;
    lr20xx_radio_lora_set_packet_params(ctx, &pkt);

    uint8_t dummy[4] = {0xCA, 0xFE, 0x00, 0x01};
    lr20xx_radio_fifo_write_tx(ctx, dummy, 4);

    ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    lr20xx_radio_common_set_tx(ctx, 2000);
    smtc_modem_hal_unprotect_api_call();

    mode_ = Mode::tx_pending;
    bool ok = wait_for_tx_done(1500);
    if (!ok) {
        ESP_LOGE(TAG, "LoRa wakeup TX timeout");
    } else {
        ESP_LOGI(TAG, "LoRa wakeup sent");
    }
    return ok;
}

void RadioPing::send_vbat_broadcast()
{
    // Battery voltage broadcast: minimal FLRC packet with current cached voltage.
    // Layout: [0..13] 14-byte header (magic + type=12, rest zeroed)
    //         [14..15] vbat_mv (u16 LE)
    //         [16..19] CRC32 covering [0..15]
    uint8_t pkt[kHeaderSize + 6];
    std::memset(pkt, 0, sizeof(pkt));
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeVbat;
    put_u16_le(&pkt[14], bsp_vbat_get_cached());
    put_u32_le(&pkt[16], crc32_ieee(pkt, 16));

    send_single_packet(pkt, kHeaderSize + 6);
    ESP_LOGI(TAG, "vbat broadcast sent: %u mV", bsp_vbat_get_cached());
}

void RadioPing::vbat_maintenance_tick()
{
    // Gateway never broadcasts its voltage, only receives from nodes.
    if (is_gateway_) return;

    uint32_t now = smtc_modem_hal_get_time_in_ms();

    if (g_low_power_enabled) {
        // Low-power node: only refresh the cached voltage every 60s (搭 CAD 唤醒
        // 的车，不额外唤醒). No periodic broadcast — the voltage is carried back
        // to the gateway inside the ImageStart first packet when the node is
        // woken to push a photo. This also avoids having to switch the radio from
        // LoRa CAD to FLRC just for a broadcast.
        if ((int32_t)(now - vbat_last_sample_ms_) >= (int32_t)kVbatLowPowerSampleIntervalMs) {
            int mv = bsp_vbat_read_mv();
            if (mv >= 0) {
                ESP_LOGD(TAG, "vbat sample: %d mV", mv);
            }
            vbat_last_sample_ms_ = now;
        }
        return;
    }

    // Non-low-power node: sampling is handled by the bsp_vbat background task
    // (15s). Broadcast the cached voltage every 5 minutes (radio is already in
    // FLRC RX here, so send_single_packet is safe).
    if ((int32_t)(now - vbat_last_broadcast_ms_) >= (int32_t)kVbatBroadcastIntervalMs) {
        send_vbat_broadcast();
        vbat_last_broadcast_ms_ = now;
    }
}
