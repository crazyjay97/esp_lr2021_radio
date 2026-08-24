#include "echo_canceller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "app_config.h"

void EchoCanceller::reset()
{
    std::memset(reference_ring_, 0, sizeof(reference_ring_));
    std::memset(coefficients_, 0, sizeof(coefficients_));
    reference_samples_.store(0, std::memory_order_release);
}

void EchoCanceller::push_reference(const int16_t *pcm, size_t samples)
{
    if (pcm == nullptr || samples == 0) return;

    uint32_t total = reference_samples_.load(std::memory_order_relaxed);
    for (size_t i = 0; i < samples; ++i) {
        reference_ring_[(total + i) % kRingSamples] = pcm[i];
    }
    reference_samples_.store(total + static_cast<uint32_t>(samples),
                             std::memory_order_release);
}

void EchoCanceller::process_capture(int16_t *pcm, size_t samples)
{
    if (pcm == nullptr || samples == 0) return;

    constexpr uint32_t delay_samples =
        APP_AUDIO_SAMPLE_RATE_HZ * APP_AEC_REFERENCE_DELAY_MS / 1000U;
    const uint32_t total = reference_samples_.load(std::memory_order_acquire);
    if (total < delay_samples + samples + kFilterTaps) return;

    const uint32_t reference_start = total - delay_samples -
        static_cast<uint32_t>(samples);
    float far_energy = 0.0f;
    float near_energy = 0.0f;
    for (size_t i = 0; i < samples; ++i) {
        const float far = reference_ring_[(reference_start + i) % kRingSamples] /
            32768.0f;
        const float near = pcm[i] / 32768.0f;
        far_energy += far * far;
        near_energy += near * near;
    }
    if (far_energy < APP_AEC_MIN_REFERENCE_ENERGY) return;

    const bool adapt = near_energy <= far_energy * APP_AEC_DOUBLE_TALK_RATIO;
    for (size_t i = 0; i < samples; ++i) {
        const uint32_t ref_index = reference_start + static_cast<uint32_t>(i);
        float estimate = 0.0f;
        float norm = APP_AEC_NLMS_EPSILON;
        for (size_t tap = 0; tap < kFilterTaps; ++tap) {
            const float x = reference_ring_[(ref_index - tap) % kRingSamples] /
                32768.0f;
            estimate += coefficients_[tap] * x;
            norm += x * x;
        }

        const float near = pcm[i] / 32768.0f;
        const float error = near - estimate;
        if (adapt) {
            const float step = APP_AEC_NLMS_STEP * error / norm;
            for (size_t tap = 0; tap < kFilterTaps; ++tap) {
                const float x = reference_ring_[(ref_index - tap) % kRingSamples] /
                    32768.0f;
                coefficients_[tap] += step * x;
            }
        }

        const float limited = std::max(-1.0f, std::min(0.999969f, error));
        pcm[i] = static_cast<int16_t>(std::lrintf(limited * 32768.0f));
    }
}
