#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

class CameraUartStreamer {
public:
    esp_err_t init();
    esp_err_t start();
    esp_err_t capture_frame(uint8_t **out_data,
                            size_t *out_len,
                            uint32_t *out_width,
                            uint32_t *out_height,
                            uint32_t *out_pixelformat);
    esp_err_t power_down();

private:
    static void task_entry(void *arg);
    void task();

    esp_err_t configure_camera_pins();
    esp_err_t init_uart();
    esp_err_t set_pwdn(bool asserted);
    esp_err_t reset_sensor();
    void read_sp0a39_id();
    esp_err_t init_sp0a39_dvp_video();
    esp_err_t capture_one_frame(uint8_t **out_data,
                                size_t *out_len,
                                uint32_t *out_width,
                                uint32_t *out_height,
                                uint32_t *out_pixelformat);

    bool initialized_ = false;
    bool uart_initialized_ = false;
    bool video_initialized_ = false;
};
