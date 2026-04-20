#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

struct OpusDecoder;
struct OpusEncoder;

class OpusCodec {
public:
    ~OpusCodec();

    esp_err_t init();
    bool ready() const { return encoder_ != nullptr && decoder_ != nullptr; }

    int encode(const int16_t *pcm, int frame_samples, uint8_t *out, size_t out_capacity);
    int decode(const uint8_t *packet, size_t packet_len, int16_t *pcm, int frame_samples);
    int decode_lost(int16_t *pcm, int frame_samples);
    void reset_encoder();
    void reset_decoder();

private:
    OpusEncoder *encoder_ = nullptr;
    OpusDecoder *decoder_ = nullptr;
};
