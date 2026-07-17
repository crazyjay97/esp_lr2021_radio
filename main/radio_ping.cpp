#include "radio_ping.hpp"

#include <cstring>
#include <cmath>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "app_config.h"

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

    size_t opus_ring_frames = APP_AUDIO_RINGBUF_SECONDS * 1000 / APP_AUDIO_FRAME_MS;
    if (!opus_ringbuf_.init(opus_ring_frames)) {
        ESP_LOGW(TAG, "opus ring buffer alloc failed (PSRAM?)");
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

    esp_err_t err = codec_.init_decoder_only();
    if (err != ESP_OK) {
        return err;
    }

    voice_queue_ = xQueueCreate(APP_VOICE_RX_QUEUE_LEN, sizeof(VoicePacket));
    if (voice_queue_ == nullptr) {
        ESP_LOGE(TAG, "voice queue alloc failed");
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
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
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
            check_image_req_retry();
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

        // Low power (node): the CPU sleeps during CAD standby, so we don't
        // sample the mic at all — no voice prep, no Opus pre-encoding, no sound
        // trigger. PIR is a hardware GPIO wake source, so its trigger is still
        // handled here after wakeup.
        if (g_low_power_enabled && !is_gateway_) {
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

        audio_ringbuf_.write(tx_pcm_, APP_AUDIO_FRAME_SAMPLES);

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
                if ((now - last_trigger_us_) >= (int64_t)APP_TRIGGER_COOLDOWN_SEC * 1000000LL) {
                    last_trigger_us_ = now;
                    ESP_LOGI(TAG, "sound trigger! rms=%u thresh=%u", rms, thresh);
                    if (image_capture_cb_) {
                        image_capture_cb_(sound_trigger_session_id_++);
                    }
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

        if (opus_preenc_enabled_) {
            uint8_t enc_buf[APP_OPUS_MAX_PACKET_BYTES];
            int enc_len = codec_.encode(tx_pcm_, APP_AUDIO_FRAME_SAMPLES,
                                        enc_buf, APP_OPUS_MAX_PACKET_BYTES);
            if (enc_len > 0 && enc_len <= 255) {
                opus_ringbuf_.write(enc_buf, static_cast<uint8_t>(enc_len));
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
        if (g_low_power_enabled && !is_gateway_) {
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
                (smtc_modem_hal_get_time_in_ms() - cad_wakeup_ms_) < 8000) {
                schedule_rx();
            } else {
                cad_wakeup_ms_ = 0;
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
    if (image_rx_pending_ || is_gateway_) {
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
    } else if (rx_buf_[4] == kPacketTypeImageCmdAck) {
        handle_image_cmd_ack();
    } else if (rx_buf_[4] == kPacketTypeConfig) {
        uint8_t key = rx_buf_[8];
        uint32_t value = get_u32_le(&rx_buf_[9]);
        ESP_LOGI(TAG, "RX Config: key=%u value=%lu", key, static_cast<unsigned long>(value));
        if (config_received_cb_) {
            config_received_cb_(key, value);
        }
        send_config_ack(key, value);
        // Low power: config applied + ACK sent, work done. End the wake window
        // so the main loop returns to CAD sleep on the next idle pass.
        if (g_low_power_enabled && !is_gateway_ && cad_wakeup_ms_ != 0) {
            cad_wakeup_ms_ = 0;
            ESP_LOGI(TAG, "config ACK sent, ending wake window -> CAD sleep");
        }
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
    if (image_tx_active_) {
        ESP_LOGW(TAG, "image TX already active, ignoring trigger");
        return;
    }

    // Pick the session ONCE for this whole request. Retries reuse it (they do
    // NOT ++), so a resend can never spawn a second capture / a different JPEG.
    image_req_session_ = image_session_id_++;
    if (image_session_id_ == 0) {
        image_session_id_ = 1;
    }
    // First send. send_image_cmd_once does the low-power LoRa wakeup + FLRC
    // reconfig itself; the radio-task retry poll repeats it (the node may have
    // gone back to CAD sleep between attempts).
    send_image_cmd_once();

    image_rx_pending_ = true;
    image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
    schedule_rx();

    // Mark the request active. check_image_req_retry (radio task loop) resends
    // ImageCmd once per interval until the node acks (or ImageStart backstop).
    uint32_t retry_ms = g_low_power_enabled ? APP_IMAGE_REQ_RETRY_INTERVAL_LP_MS
                                            : APP_IMAGE_REQ_RETRY_INTERVAL_MS;
    image_req_active_ = true;
    image_req_next_ms_ = smtc_modem_hal_get_time_in_ms() + retry_ms;
}

// Send one ImageCmd for image_req_session_. In low power, redo the LoRa wakeup
// preamble + FLRC reconfig first (the node may have returned to CAD sleep).
void RadioPing::send_image_cmd_once()
{
    if (g_low_power_enabled) {
        if (!send_lora_wakeup()) {
            ESP_LOGE(TAG, "LoRa wakeup failed (retry)");
            return;
        }
        configure_flrc();
    }

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
    if ((int32_t)(now - image_req_next_ms_) < 0) return;

    ESP_LOGI(TAG, "ImageCmd no ack yet, resending (session=%u)", image_req_session_);
    send_image_cmd_once();
    if (mode_ != Mode::rx_pending) {
        schedule_rx();
    }
    uint32_t retry_ms = g_low_power_enabled ? APP_IMAGE_REQ_RETRY_INTERVAL_LP_MS
                                            : APP_IMAGE_REQ_RETRY_INTERVAL_MS;
    image_req_next_ms_ = smtc_modem_hal_get_time_in_ms() + retry_ms;
}

void RadioPing::stop_image_req_retry()
{
    image_req_active_ = false;
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
            suspended_ = false;
            ptt_active_ = was_ptt;
            if (g_low_power_enabled && !is_gateway_ && (cad_wakeup_ms_ != 0 || pir_push_wake_)) {
                cad_wakeup_ms_ = 0;
                pir_push_wake_ = false;
                ESP_LOGI(TAG, "image TX aborted, ending wake window -> CAD sleep");
            }
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
                // No ACK this round. Do NOT blindly resend the whole image —
                // that floods the channel and makes a bad RF environment worse.
                // Just advance to the next round, which re-sends EOT and waits
                // again. The initial burst already sent every fragment once;
                // actual retransmission only happens on an explicit NACK
                // missing-list below.
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

    // Ack immediately (before the slow JPEG capture) so the gateway stops
    // resending ImageCmd. This is sent on EVERY ImageCmd — including gateway
    // retransmits of the same session (their ack was lost) — so a lost ack
    // self-heals. The capture-side dedup (image_capture_cb_) drops the duplicate
    // dispatch, so re-acking never triggers a second capture.
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
    schedule_rx();

    if (image_capture_cb_) {
        image_capture_cb_(session_id);
    }
}

// Gateway side: node acknowledged the ImageCmd. Stop the request-retry timer.
void RadioPing::handle_image_cmd_ack()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    if (session_id != image_req_session_) {
        return;
    }
    // ESP_LOGI(TAG, "RX ImageCmdAck: session=%u", session_id);
    stop_image_req_retry();
    schedule_rx();
}

void RadioPing::handle_image_start()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    uint16_t total_frags = get_u16_le(&rx_buf_[8]);
    uint32_t expected_crc32 = get_u32_le(&rx_buf_[10]);

    // ESP_LOGI(TAG, "RX ImageStart: session=%u total=%u crc32=0x%08lx",
    //          session_id, total_frags, static_cast<unsigned long>(expected_crc32));

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
        uint32_t transfer_ms = now_ms - image_rx_start_ms_;
        uint32_t prepare_ms = image_rx_start_ms_ - image_cmd_sent_ms_;
        uint32_t total_ms = now_ms - image_cmd_sent_ms_;
        image_rx_transfer_ms_ = transfer_ms;
        image_rx_done_ms_ = now_ms;
        ESP_LOGI(TAG, "RX complete: session=%u | prepare=%lums transfer=%lums total=%lums",
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

size_t RadioPing::snapshot_opus(uint8_t *out, size_t max_bytes)
{
    return opus_ringbuf_.snapshot(out, max_bytes);
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
        ESP_LOGI(TAG, "CAD detected activity, switching to FLRC RX (8s window)");
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
