#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/i2s_common.h"
#include "freertos/FreeRTOS.h"

class CameraUartStreamer {
public:
    esp_err_t init();
    esp_err_t start();

private:
    static void task_entry(void *arg);
    void task();

    esp_err_t power_on_camera();
    esp_err_t start_mclk();
    esp_err_t attach_gc032a();
    esp_err_t select_gc032a_address();
    esp_err_t gc032a_read_reg(uint8_t reg, uint8_t *val);
    esp_err_t gc032a_write_reg(uint8_t reg, uint8_t val);
    void probe_gc032a();
    esp_err_t init_gc032a_sensor();
    void dump_gc032a_spi_regs();
    esp_err_t alloc_sample_buffer();
    void probe_i2c();
    void dump_probe_burst(bool sample_falling, uint32_t seq);

    void dvp_dma_probe_loop();
    void capture_and_send_frame(uint32_t seq);
    void analyze_and_dump(const uint8_t *raw_pairs, size_t pair_count,
                          bool sample_falling, uint32_t seq);
    void emit_frame(uint32_t seq, const uint8_t *data, size_t len,
                    uint32_t width, uint32_t height);

    void write_packet(uint8_t type, uint32_t seq, const void *payload, uint32_t len);
    void write_status(const char *msg);
    void write_hex_dump(const char *tag, const uint8_t *data, size_t len);

    i2c_master_dev_handle_t gc032a_ = nullptr;
    uint8_t *raw_pair_buf_ = nullptr;   // 1 PCLK sample per byte, low 2 bits = {D1,D0}
    uint8_t *packed_buf_   = nullptr;   // 4 pairs packed per byte
    uint8_t *frame_buf_    = nullptr;   // complete YVYU frame in PSRAM when full-frame mode is enabled
    size_t   raw_capacity_ = 0;
    size_t   frame_capacity_ = 0;
    uint8_t  gc032a_addr_  = 0;
    i2s_chan_handle_t mclk_tx_ = nullptr;
    bool     initialized_  = false;
};
