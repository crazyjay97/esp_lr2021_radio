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
    esp_err_t configure_camera_pins();
    esp_err_t set_pwdn(bool asserted);
    esp_err_t reset_sensor();

    bool initialized_ = false;
};
