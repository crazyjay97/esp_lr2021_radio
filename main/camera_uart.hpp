#pragma once

#include "esp_err.h"

class CameraUartStreamer {
public:
    esp_err_t init();
    esp_err_t start();

private:
    static void task_entry(void *arg);
    void task();

    esp_err_t configure_camera_pins();
    esp_err_t init_uart();
    esp_err_t set_pwdn(bool asserted);
    void read_sp0a39_id();
    esp_err_t init_sp0a39_video();
    esp_err_t capture_and_send_one_frame();

    bool initialized_ = false;
    bool uart_initialized_ = false;
    bool video_initialized_ = false;
};
