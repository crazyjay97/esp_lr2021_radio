#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

class EchoCanceller {
public:
    void reset();
    void push_reference(const int16_t *pcm, size_t samples);
    void process_capture(int16_t *pcm, size_t samples);

private:
    static constexpr size_t kRingSamples = 4096;
    static constexpr size_t kFilterTaps = 256;

    int16_t reference_ring_[kRingSamples] = {};
    float coefficients_[kFilterTaps] = {};
    std::atomic<uint32_t> reference_samples_{0};
};
