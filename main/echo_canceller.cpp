#include "echo_canceller.hpp"

#include <cstring>

#include "app_config.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

namespace {

constexpr const char *TAG = "afe_aec";

int16_t *allocate_psram_samples(size_t samples)
{
    return static_cast<int16_t *>(heap_caps_aligned_calloc(
        16, samples, sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
}

} // namespace

EchoCanceller::~EchoCanceller()
{
    release();
    if (aec_mutex_ != nullptr) vSemaphoreDelete(aec_mutex_);
    if (reference_mutex_ != nullptr) vSemaphoreDelete(reference_mutex_);
    aec_mutex_ = nullptr;
    reference_mutex_ = nullptr;
}

bool EchoCanceller::init()
{
    if (ready()) return true;

    if (aec_mutex_ == nullptr) aec_mutex_ = xSemaphoreCreateMutex();
    if (reference_mutex_ == nullptr) {
        reference_mutex_ = xSemaphoreCreateMutex();
    }
    if (aec_mutex_ == nullptr || reference_mutex_ == nullptr) {
        ESP_LOGE(TAG, "mutex allocation failed");
        release();
        return false;
    }

    aec_handle_ = afe_aec_create("MR", APP_AFE_AEC_FILTER_LENGTH,
                                 AFE_TYPE_VC, AFE_MODE_HIGH_PERF);
    if (aec_handle_ == nullptr || aec_handle_->handle == nullptr) {
        ESP_LOGE(TAG, "official direct AEC create failed");
        release();
        return false;
    }

    const int frame_samples = afe_aec_get_chunksize(aec_handle_);
    const afe_pcm_config_t &pcm_config = aec_handle_->pcm_config;
    if (aec_handle_->mode != AEC_MODE_VOIP_HIGH_PERF ||
        aec_handle_->handle->config.mode != AEC_MODE_VOIP_HIGH_PERF ||
        frame_samples <= 0 ||
        pcm_config.sample_rate != static_cast<int>(APP_AUDIO_SAMPLE_RATE_HZ) ||
        pcm_config.total_ch_num != 2 || pcm_config.mic_num != 1 ||
        pcm_config.ref_num != 1) {
        ESP_LOGE(TAG,
                 "unexpected direct AEC format: mode=%d core_mode=%d "
                 "rate=%d frame=%d channels=%d mic=%d ref=%d",
                 static_cast<int>(aec_handle_->mode),
                 static_cast<int>(aec_handle_->handle->config.mode),
                 pcm_config.sample_rate, frame_samples,
                 pcm_config.total_ch_num, pcm_config.mic_num,
                 pcm_config.ref_num);
        release();
        return false;
    }

    frame_samples_ = static_cast<size_t>(frame_samples);
    if (frame_samples_ > kReferenceRingSamples) {
        ESP_LOGE(TAG, "direct AEC frame too large: frame=%u",
                 static_cast<unsigned>(frame_samples_));
        release();
        return false;
    }
    pending_capacity_ = frame_samples_ + APP_AUDIO_FRAME_SAMPLES;
    bridge_delay_samples_ =
        ((frame_samples_ + APP_AUDIO_FRAME_SAMPLES - 1U) /
         APP_AUDIO_FRAME_SAMPLES) * APP_AUDIO_FRAME_SAMPLES;
    output_capacity_ = bridge_delay_samples_ + frame_samples_;

    reference_ring_ = allocate_psram_samples(kReferenceRingSamples);
    mic_pending_ = allocate_psram_samples(pending_capacity_);
    reference_pending_ = allocate_psram_samples(pending_capacity_);
    aec_input_ = allocate_psram_samples(frame_samples_ * 2U);
    aec_output_ = allocate_psram_samples(frame_samples_);
    output_ring_ = allocate_psram_samples(output_capacity_);
    if (reference_ring_ == nullptr || mic_pending_ == nullptr ||
        reference_pending_ == nullptr || aec_input_ == nullptr ||
        aec_output_ == nullptr || output_ring_ == nullptr) {
        ESP_LOGE(TAG, "PSRAM buffer allocation failed");
        release();
        return false;
    }

    reset_stream_buffers();
    ready_.store(true, std::memory_order_release);
    ESP_LOGI(TAG,
             "ESP-SR direct AEC ready: api=afe_aec format=MR type=VC "
             "mode=VOIP_HIGH_PERF nlp=%d aec_frame=%u io_frame=%u "
             "bridge_delay=%lums reference=fifo filter=%u worker_tasks=0",
             static_cast<int>(aec_handle_->handle->config.nlp_level),
             static_cast<unsigned>(frame_samples_),
             static_cast<unsigned>(APP_AUDIO_FRAME_SAMPLES),
             static_cast<unsigned long>(
                 bridge_delay_samples_ * 1000U / APP_AUDIO_SAMPLE_RATE_HZ),
             static_cast<unsigned>(APP_AFE_AEC_FILTER_LENGTH));
    return true;
}

void EchoCanceller::release()
{
    ready_.store(false, std::memory_order_release);

    const bool have_aec_lock = aec_mutex_ != nullptr &&
        xSemaphoreTake(aec_mutex_, portMAX_DELAY) == pdTRUE;
    const bool have_reference_lock = reference_mutex_ != nullptr &&
        xSemaphoreTake(reference_mutex_, portMAX_DELAY) == pdTRUE;

    if (aec_handle_ != nullptr) afe_aec_destroy(aec_handle_);
    aec_handle_ = nullptr;

    heap_caps_free(reference_ring_);
    heap_caps_free(mic_pending_);
    heap_caps_free(reference_pending_);
    heap_caps_free(aec_input_);
    heap_caps_free(aec_output_);
    heap_caps_free(output_ring_);
    reference_ring_ = nullptr;
    mic_pending_ = nullptr;
    reference_pending_ = nullptr;
    aec_input_ = nullptr;
    aec_output_ = nullptr;
    output_ring_ = nullptr;
    frame_samples_ = 0;
    pending_capacity_ = 0;
    pending_samples_ = 0;
    output_capacity_ = 0;
    output_read_ = 0;
    output_write_ = 0;
    output_samples_ = 0;
    bridge_delay_samples_ = 0;
    reference_read_ = 0;
    reference_write_ = 0;
    reference_samples_ = 0;
    error_count_ = 0;
    reference_overflow_count_ = 0;
    reference_wait_frames_ = 0;
    reference_started_ = false;

    if (have_reference_lock) xSemaphoreGive(reference_mutex_);
    if (have_aec_lock) xSemaphoreGive(aec_mutex_);
}

void EchoCanceller::reset()
{
    if (aec_mutex_ == nullptr ||
        xSemaphoreTake(aec_mutex_, portMAX_DELAY) != pdTRUE) return;
    if (!ready()) {
        xSemaphoreGive(aec_mutex_);
        return;
    }

    if (xSemaphoreTake(reference_mutex_, portMAX_DELAY) == pdTRUE) {
        // Keep the official AEC instance resident between calls. The direct
        // API has no reset entry point, so only clear project-owned streams.
        reset_stream_buffers();
        xSemaphoreGive(reference_mutex_);
    }
    xSemaphoreGive(aec_mutex_);
}

void EchoCanceller::reset_stream_buffers()
{
    std::memset(reference_ring_, 0,
                kReferenceRingSamples * sizeof(int16_t));
    std::memset(mic_pending_, 0, pending_capacity_ * sizeof(int16_t));
    std::memset(reference_pending_, 0,
                pending_capacity_ * sizeof(int16_t));
    std::memset(aec_input_, 0, frame_samples_ * 2U * sizeof(int16_t));
    std::memset(aec_output_, 0, frame_samples_ * sizeof(int16_t));
    std::memset(output_ring_, 0, output_capacity_ * sizeof(int16_t));

    pending_samples_ = 0;
    output_read_ = 0;
    output_write_ = bridge_delay_samples_ % output_capacity_;
    output_samples_ = bridge_delay_samples_;
    reference_read_ = 0;
    reference_write_ = 0;
    reference_samples_ = 0;
    error_count_ = 0;
    reference_overflow_count_ = 0;
    reference_wait_frames_ = 0;
    reference_started_ = false;
}

void EchoCanceller::push_reference(const int16_t *pcm, size_t samples)
{
    if (pcm == nullptr || samples == 0 || reference_mutex_ == nullptr) return;
    if (xSemaphoreTake(reference_mutex_, portMAX_DELAY) != pdTRUE) return;
    if (!ready()) {
        xSemaphoreGive(reference_mutex_);
        return;
    }

    if (samples > kReferenceRingSamples) {
        pcm += samples - kReferenceRingSamples;
        samples = kReferenceRingSamples;
    }
    const size_t free_samples = kReferenceRingSamples - reference_samples_;
    if (samples > free_samples) {
        const size_t drop = samples - free_samples;
        reference_read_ = (reference_read_ + drop) % kReferenceRingSamples;
        reference_samples_ -= drop;
        ++reference_overflow_count_;
        if (reference_overflow_count_ == 1U ||
            (reference_overflow_count_ % 100U) == 0U) {
            ESP_LOGW(TAG,
                     "reference FIFO overflow: dropped=%u events=%lu",
                     static_cast<unsigned>(drop),
                     static_cast<unsigned long>(reference_overflow_count_));
        }
    }
    for (size_t i = 0; i < samples; ++i) {
        reference_ring_[reference_write_] = pcm[i];
        reference_write_ = (reference_write_ + 1U) % kReferenceRingSamples;
    }
    reference_samples_ += samples;
    xSemaphoreGive(reference_mutex_);
}

void EchoCanceller::process_capture(int16_t *pcm, size_t samples)
{
    if (pcm == nullptr || samples == 0) return;
    if (aec_mutex_ == nullptr ||
        xSemaphoreTake(aec_mutex_, portMAX_DELAY) != pdTRUE) {
        std::memset(pcm, 0, samples * sizeof(int16_t));
        return;
    }
    if (!ready() || samples != APP_AUDIO_FRAME_SAMPLES ||
        pending_samples_ + samples > pending_capacity_) {
        if (ready()) {
            report_error("io_frame", static_cast<int>(samples));
        }
        std::memset(pcm, 0, samples * sizeof(int16_t));
        xSemaphoreGive(aec_mutex_);
        return;
    }

    std::memcpy(mic_pending_ + pending_samples_, pcm,
                samples * sizeof(int16_t));

    size_t reference_copied = 0;
    if (xSemaphoreTake(reference_mutex_, portMAX_DELAY) == pdTRUE) {
        reference_copied =
            reference_samples_ < samples ? reference_samples_ : samples;
        for (size_t i = 0; i < reference_copied; ++i) {
            reference_pending_[pending_samples_ + i] =
                reference_ring_[reference_read_];
            reference_read_ =
                (reference_read_ + 1U) % kReferenceRingSamples;
        }
        reference_samples_ -= reference_copied;
        xSemaphoreGive(reference_mutex_);
    }

    if (reference_copied < samples) {
        ++reference_wait_frames_;
        std::memset(reference_pending_ + pending_samples_ + reference_copied,
                    0, (samples - reference_copied) * sizeof(int16_t));
    } else if (!reference_started_) {
        reference_started_ = true;
        ESP_LOGI(TAG, "playback reference FIFO active after %lu capture frames",
                 static_cast<unsigned long>(reference_wait_frames_));
    }
    pending_samples_ += samples;

    while (pending_samples_ >= frame_samples_) {
        for (size_t i = 0; i < frame_samples_; ++i) {
            aec_input_[i * 2U] = mic_pending_[i];
            aec_input_[i * 2U + 1U] = reference_pending_[i];
        }

        const size_t output_bytes =
            afe_aec_process(aec_handle_, aec_input_, aec_output_);
        const size_t expected_bytes = frame_samples_ * sizeof(int16_t);
        if (output_bytes != expected_bytes) {
            report_error("process", static_cast<int>(output_bytes));
            std::memset(aec_output_, 0, expected_bytes);
        }
        append_output(aec_output_, frame_samples_);

        pending_samples_ -= frame_samples_;
        if (pending_samples_ != 0) {
            std::memmove(mic_pending_, mic_pending_ + frame_samples_,
                         pending_samples_ * sizeof(int16_t));
            std::memmove(reference_pending_,
                         reference_pending_ + frame_samples_,
                         pending_samples_ * sizeof(int16_t));
        }
    }

    if (output_samples_ < samples) {
        report_error("output_underflow", static_cast<int>(output_samples_));
        std::memset(pcm, 0, samples * sizeof(int16_t));
    } else {
        for (size_t i = 0; i < samples; ++i) {
            pcm[i] = output_ring_[output_read_];
            output_read_ = (output_read_ + 1U) % output_capacity_;
        }
        output_samples_ -= samples;
    }
    xSemaphoreGive(aec_mutex_);
}

void EchoCanceller::append_output(const int16_t *pcm, size_t samples)
{
    if (samples > output_capacity_) {
        pcm += samples - output_capacity_;
        samples = output_capacity_;
    }
    const size_t free_samples = output_capacity_ - output_samples_;
    if (samples > free_samples) {
        const size_t drop = samples - free_samples;
        output_read_ = (output_read_ + drop) % output_capacity_;
        output_samples_ -= drop;
        report_error("output_overflow", static_cast<int>(drop));
    }
    for (size_t i = 0; i < samples; ++i) {
        output_ring_[output_write_] = pcm[i];
        output_write_ = (output_write_ + 1U) % output_capacity_;
    }
    output_samples_ += samples;
}

void EchoCanceller::report_error(const char *operation, int result)
{
    ++error_count_;
    if (error_count_ == 1U || (error_count_ % 100U) == 0U) {
        ESP_LOGW(TAG, "direct AEC %s failed/result=%d count=%lu", operation,
                 result, static_cast<unsigned long>(error_count_));
    }
}
