#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_aec.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class EchoCanceller {
public:
    EchoCanceller() = default;
    ~EchoCanceller();

    EchoCanceller(const EchoCanceller &) = delete;
    EchoCanceller &operator=(const EchoCanceller &) = delete;

    bool init();
    void deinit();
    bool ready() const { return ready_.load(std::memory_order_acquire); }
    void reset();
    void push_reference(const int16_t *pcm, size_t samples);
    void process_capture(int16_t *pcm, size_t samples);

private:
    static constexpr size_t kReferenceRingSamples = 8192;

    void release();
    void append_output(const int16_t *pcm, size_t samples);
    void reset_stream_buffers();
    void report_error(const char *operation, int result);

    aec_handle_t *aec_handle_ = nullptr;
    SemaphoreHandle_t aec_mutex_ = nullptr;
    SemaphoreHandle_t reference_mutex_ = nullptr;

    int16_t *reference_ring_ = nullptr;
    int16_t *mic_pending_ = nullptr;
    int16_t *reference_pending_ = nullptr;
    int16_t *aec_output_ = nullptr;
    int16_t *output_ring_ = nullptr;

    size_t frame_samples_ = 0;
    size_t pending_capacity_ = 0;
    size_t pending_samples_ = 0;
    size_t output_capacity_ = 0;
    size_t output_read_ = 0;
    size_t output_write_ = 0;
    size_t output_samples_ = 0;
    size_t bridge_delay_samples_ = 0;
    size_t reference_read_ = 0;
    size_t reference_write_ = 0;
    size_t reference_samples_ = 0;
    uint32_t error_count_ = 0;
    uint32_t reference_overflow_count_ = 0;
    uint32_t reference_underflow_count_ = 0;
    uint32_t reference_wait_frames_ = 0;
    bool reference_started_ = false;

    std::atomic<bool> ready_{false};
};
