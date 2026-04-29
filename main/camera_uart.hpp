#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
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
    esp_err_t init_gc032a_sensor();
    void dump_gc032a_init_regs();
    void dump_gc032a_spi_regs();
    void probe_i2c();
    void write_status(const char *msg);
    void dvp_stream_loop();
    void spi_slave_stream_loop();
    i2c_master_dev_handle_t gc032a_ = nullptr;
    uint8_t  gc032a_addr_  = 0;
    bool     mclk_started_ = false;
    bool     initialized_  = false;
};
