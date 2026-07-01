#include "image_transfer.hpp"

#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_jpeg_enc.h"
#include "esp_jpeg_dec.h"
#include "esp_jpeg_common.h"

namespace {
constexpr const char *TAG = "img_xfer";

uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1U) ? (crc >> 1) ^ 0xEDB88320U : (crc >> 1);
        }
    }
    return ~crc;
}
}

esp_err_t ImageTransfer::encode_frame(const uint8_t *yuv422, size_t yuv_len,
                                      uint32_t width, uint32_t height, uint32_t pixfmt,
                                      uint8_t **out_jpeg, size_t *out_jpeg_len)
{
    if (!yuv422 || !out_jpeg || !out_jpeg_len) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_jpeg = nullptr;
    *out_jpeg_len = 0;

    const size_t expected_len = width * height * 2;
    if (yuv_len < expected_len) {
        ESP_LOGE(TAG, "YUV buffer too small: %u < %u",
                 static_cast<unsigned>(yuv_len), static_cast<unsigned>(expected_len));
        return ESP_ERR_INVALID_SIZE;
    }

    // SP0A39 outputs UYVY = [Cb, Y0, Cr, Y1]
    // Encoder expects YCbYCr = [Y0, Cb, Y1, Cr]
    // Swizzle: [U, Y0, V, Y1] → [Y0, U, Y1, V]
    uint8_t *enc_input = static_cast<uint8_t *>(
        jpeg_calloc_align(expected_len, 16));
    if (!enc_input) {
        ESP_LOGE(TAG, "enc input alloc failed: %u bytes",
                 static_cast<unsigned>(expected_len));
        return ESP_ERR_NO_MEM;
    }

    for (size_t i = 0; i + 3 < expected_len; i += 4) {
        enc_input[i + 0] = yuv422[i + 1]; // Y0
        enc_input[i + 1] = yuv422[i + 0]; // U (Cb)
        enc_input[i + 2] = yuv422[i + 3]; // Y1
        enc_input[i + 3] = yuv422[i + 2]; // V (Cr)
    }

    jpeg_enc_config_t enc_cfg = DEFAULT_JPEG_ENC_CONFIG();
    enc_cfg.width = static_cast<int>(width);
    enc_cfg.height = static_cast<int>(height);
    enc_cfg.src_type = JPEG_PIXEL_FORMAT_YCbYCr;
    enc_cfg.subsampling = JPEG_SUBSAMPLE_420;
    enc_cfg.quality = APP_IMAGE_JPEG_QUALITY;
    enc_cfg.task_enable = false;

    jpeg_enc_handle_t encoder = nullptr;
    jpeg_error_t jerr = jpeg_enc_open(&enc_cfg, &encoder);
    if (jerr != JPEG_ERR_OK || !encoder) {
        ESP_LOGE(TAG, "jpeg_enc_open failed: %d", jerr);
        jpeg_free_align(enc_input);
        return ESP_FAIL;
    }

    uint8_t *jpeg_buf = static_cast<uint8_t *>(
        heap_caps_malloc(APP_IMAGE_MAX_JPEG_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!jpeg_buf) {
        ESP_LOGE(TAG, "jpeg output buffer alloc failed");
        jpeg_enc_close(encoder);
        jpeg_free_align(enc_input);
        return ESP_ERR_NO_MEM;
    }

    int out_size = 0;
    jerr = jpeg_enc_process(encoder, enc_input, static_cast<int>(expected_len),
                            jpeg_buf, static_cast<int>(APP_IMAGE_MAX_JPEG_SIZE),
                            &out_size);
    jpeg_enc_close(encoder);
    jpeg_free_align(enc_input);

    if (jerr != JPEG_ERR_OK || out_size <= 0) {
        ESP_LOGE(TAG, "jpeg_enc_process failed: %d out_size=%d", jerr, out_size);
        heap_caps_free(jpeg_buf);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "JPEG encoded: %lux%lu → %d bytes (Q=%d)",
             static_cast<unsigned long>(width), static_cast<unsigned long>(height),
             out_size, APP_IMAGE_JPEG_QUALITY);

    *out_jpeg = jpeg_buf;
    *out_jpeg_len = static_cast<size_t>(out_size);
    return ESP_OK;
}

void ImageTransfer::rx_begin(uint16_t session_id, uint16_t total_fragments)
{
    rx_reset();

    size_t buf_size = static_cast<size_t>(total_fragments) * APP_IMAGE_FRAGMENT_DATA_SIZE;
    bool using_psram = false;
    rx_buf_ = static_cast<uint8_t *>(
        heap_caps_calloc(1, buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    rx_frag_lens_ = static_cast<uint16_t *>(
        heap_caps_calloc(total_fragments, sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    rx_received_map_ = static_cast<bool *>(
        heap_caps_calloc(total_fragments, sizeof(bool), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    if (!rx_buf_ || !rx_frag_lens_ || !rx_received_map_) {
        rx_reset();
        ESP_LOGW(TAG, "rx_begin internal SRAM alloc failed, falling back to PSRAM");
        using_psram = true;
        rx_buf_ = static_cast<uint8_t *>(
            heap_caps_calloc(1, buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        rx_frag_lens_ = static_cast<uint16_t *>(
            heap_caps_calloc(total_fragments, sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        rx_received_map_ = static_cast<bool *>(
            heap_caps_calloc(total_fragments, sizeof(bool), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    }

    if (!rx_buf_ || !rx_frag_lens_ || !rx_received_map_) {
        ESP_LOGE(TAG, "rx_begin alloc failed for %u fragments", total_fragments);
        rx_reset();
        return;
    }

    rx_session_id_ = session_id;
    rx_total_ = total_fragments;
    rx_received_ = 0;
    rx_jpeg_size_ = 0;

    ESP_LOGI(TAG, "rx_begin: session=%u total=%u bytes=%u caps=%s",
             session_id, total_fragments, static_cast<unsigned>(buf_size),
             using_psram ? "psram" : "internal");
}

bool ImageTransfer::rx_fragment(uint16_t session_id, uint16_t frag_index,
                                uint16_t total_fragments, const uint8_t *data, uint16_t len)
{
    if (!rx_buf_ || session_id != rx_session_id_) {
        rx_begin(session_id, total_fragments);
        if (!rx_buf_) {
            return false;
        }
    }

    if (frag_index >= rx_total_) {
        ESP_LOGW(TAG, "rx_fragment: index %u >= total %u", frag_index, rx_total_);
        return false;
    }

    if (len > APP_IMAGE_FRAGMENT_DATA_SIZE) {
        ESP_LOGW(TAG, "rx_fragment: len %u > max %u", len, APP_IMAGE_FRAGMENT_DATA_SIZE);
        return false;
    }

    if (rx_received_map_[frag_index]) {
        return rx_complete();
    }

    size_t offset = static_cast<size_t>(frag_index) * APP_IMAGE_FRAGMENT_DATA_SIZE;
    std::memcpy(rx_buf_ + offset, data, len);
    rx_frag_lens_[frag_index] = len;
    rx_received_map_[frag_index] = true;
    rx_received_++;

    if (rx_complete()) {
        rx_jpeg_size_ = 0;
        for (uint16_t i = 0; i < rx_total_; i++) {
            rx_jpeg_size_ += rx_frag_lens_[i];
        }
        ESP_LOGI(TAG, "rx complete: session=%u size=%u",
                 rx_session_id_, static_cast<unsigned>(rx_jpeg_size_));
        return true;
    }
    return false;
}

uint16_t ImageTransfer::rx_get_missing(uint16_t *out_indices, uint16_t max_count) const
{
    if (!rx_received_map_ || !out_indices) {
        return 0;
    }
    uint16_t count = 0;
    for (uint16_t i = 0; i < rx_total_ && count < max_count; i++) {
        if (!rx_received_map_[i]) {
            out_indices[count++] = i;
        }
    }
    return count;
}

bool ImageTransfer::rx_build_bitmap(uint8_t *bitmap, uint16_t max_bytes,
                                    uint16_t *out_first_missing, uint16_t *out_byte_count) const
{
    if (!rx_received_map_ || !bitmap || rx_total_ == 0) {
        return false;
    }

    uint16_t first_missing = rx_total_;
    for (uint16_t i = 0; i < rx_total_; i++) {
        if (!rx_received_map_[i]) {
            first_missing = i;
            break;
        }
    }
    if (first_missing >= rx_total_) {
        *out_first_missing = 0;
        *out_byte_count = 0;
        return true;
    }

    uint16_t bits_needed = rx_total_ - first_missing;
    uint16_t bytes_needed = (bits_needed + 7) / 8;
    if (bytes_needed > max_bytes) {
        bytes_needed = max_bytes;
    }

    std::memset(bitmap, 0, bytes_needed);
    for (uint16_t i = 0; i < bytes_needed * 8 && (first_missing + i) < rx_total_; i++) {
        if (rx_received_map_[first_missing + i]) {
            bitmap[i / 8] |= (1U << (i % 8));
        }
    }

    *out_first_missing = first_missing;
    *out_byte_count = bytes_needed;
    return true;
}

uint32_t ImageTransfer::rx_crc32() const
{
    if (!rx_buf_ || !rx_complete()) {
        return 0;
    }

    uint32_t crc = 0;
    for (uint16_t i = 0; i < rx_total_; i++) {
        size_t frag_offset = static_cast<size_t>(i) * APP_IMAGE_FRAGMENT_DATA_SIZE;
        crc = crc32_update(crc, rx_buf_ + frag_offset, rx_frag_lens_[i]);
    }
    return crc;
}

esp_err_t ImageTransfer::decode_to_rgb565(uint8_t **out_rgb565, uint32_t *out_w, uint32_t *out_h)
{
    if (!rx_buf_ || !rx_complete() || rx_jpeg_size_ == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!out_rgb565 || !out_w || !out_h) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_rgb565 = nullptr;
    *out_w = 0;
    *out_h = 0;

    // Reassemble JPEG into contiguous buffer (fragments may have varying lengths)
    uint8_t *jpeg_contiguous = static_cast<uint8_t *>(
        heap_caps_malloc(rx_jpeg_size_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!jpeg_contiguous) {
        return ESP_ERR_NO_MEM;
    }

    size_t offset = 0;
    for (uint16_t i = 0; i < rx_total_; i++) {
        size_t frag_offset = static_cast<size_t>(i) * APP_IMAGE_FRAGMENT_DATA_SIZE;
        std::memcpy(jpeg_contiguous + offset, rx_buf_ + frag_offset, rx_frag_lens_[i]);
        offset += rx_frag_lens_[i];
    }

    // Decode and scale to fit LCD preview area
    jpeg_dec_config_t dec_cfg = DEFAULT_JPEG_DEC_CONFIG();
    dec_cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_LE;
    dec_cfg.scale.width = 240;
    dec_cfg.scale.height = 176;

    jpeg_dec_handle_t decoder = nullptr;
    jpeg_error_t jerr = jpeg_dec_open(&dec_cfg, &decoder);
    if (jerr != JPEG_ERR_OK || !decoder) {
        ESP_LOGE(TAG, "jpeg_dec_open failed: %d", jerr);
        heap_caps_free(jpeg_contiguous);
        return ESP_FAIL;
    }

    jpeg_dec_io_t io = {};
    io.inbuf = jpeg_contiguous;
    io.inbuf_len = static_cast<int>(rx_jpeg_size_);

    jpeg_dec_header_info_t header = {};
    jerr = jpeg_dec_parse_header(decoder, &io, &header);
    if (jerr != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "jpeg_dec_parse_header failed: %d", jerr);
        jpeg_dec_close(decoder);
        heap_caps_free(jpeg_contiguous);
        return ESP_FAIL;
    }

    int outbuf_len = 0;
    jerr = jpeg_dec_get_outbuf_len(decoder, &outbuf_len);
    if (jerr != JPEG_ERR_OK || outbuf_len <= 0) {
        ESP_LOGE(TAG, "jpeg_dec_get_outbuf_len failed: %d len=%d", jerr, outbuf_len);
        jpeg_dec_close(decoder);
        heap_caps_free(jpeg_contiguous);
        return ESP_FAIL;
    }

    uint8_t *rgb_buf = static_cast<uint8_t *>(jpeg_calloc_align(outbuf_len, 16));
    if (!rgb_buf) {
        ESP_LOGE(TAG, "decode output alloc failed: %d bytes", outbuf_len);
        jpeg_dec_close(decoder);
        heap_caps_free(jpeg_contiguous);
        return ESP_ERR_NO_MEM;
    }

    io.outbuf = rgb_buf;
    jerr = jpeg_dec_process(decoder, &io);
    jpeg_dec_close(decoder);
    heap_caps_free(jpeg_contiguous);

    if (jerr != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "jpeg_dec_process failed: %d", jerr);
        jpeg_free_align(rgb_buf);
        return ESP_FAIL;
    }

    *out_rgb565 = rgb_buf;
    *out_w = dec_cfg.scale.width;
    *out_h = dec_cfg.scale.height;
    ESP_LOGI(TAG, "JPEG decoded to RGB565: %lux%lu (%d bytes)",
             static_cast<unsigned long>(*out_w), static_cast<unsigned long>(*out_h), outbuf_len);
    return ESP_OK;
}

esp_err_t ImageTransfer::rx_reassemble(uint8_t **out_buf, size_t *out_len)
{
    if (!rx_buf_ || !rx_complete() || rx_jpeg_size_ == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!out_buf || !out_len) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t *buf = static_cast<uint8_t *>(
        heap_caps_malloc(rx_jpeg_size_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!buf) {
        return ESP_ERR_NO_MEM;
    }

    size_t offset = 0;
    for (uint16_t i = 0; i < rx_total_; i++) {
        size_t frag_offset = static_cast<size_t>(i) * APP_IMAGE_FRAGMENT_DATA_SIZE;
        std::memcpy(buf + offset, rx_buf_ + frag_offset, rx_frag_lens_[i]);
        offset += rx_frag_lens_[i];
    }

    *out_buf = buf;
    *out_len = offset;
    return ESP_OK;
}

void ImageTransfer::rx_reset()
{
    if (rx_buf_) {
        heap_caps_free(rx_buf_);
        rx_buf_ = nullptr;
    }
    if (rx_frag_lens_) {
        heap_caps_free(rx_frag_lens_);
        rx_frag_lens_ = nullptr;
    }
    if (rx_received_map_) {
        heap_caps_free(rx_received_map_);
        rx_received_map_ = nullptr;
    }
    rx_session_id_ = 0;
    rx_total_ = 0;
    rx_received_ = 0;
    rx_jpeg_size_ = 0;
}
