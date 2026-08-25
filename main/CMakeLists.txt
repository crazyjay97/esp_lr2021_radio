idf_component_register(SRCS
                           "app_main.cpp"
                           "audio_diagnostics.cpp"
                           "audio_processor.cpp"
                           "echo_canceller.cpp"
                           "camera_dvp.cpp"
                           "image_transfer.cpp"
                           "opus_codec.cpp"
                           "radio_ping.cpp"
                           "ui_gateway.c"
                           "wifi_manager.c"
                           "qrcodegen.c"
                           "bsp_con6.c"
                           "bsp_i2c.c"
                           "bsp_lcd.c"
                           "bsp_audio.c"
                           "bsp_button.c"
                           "bsp_vbat.c"
                           "spi_psram_dma_wrap.c"
                       INCLUDE_DIRS "."
                       REQUIRES
                           lierda-iot__esp_lora_driver
                           espressif__esp-sr
                           esp_driver_gpio
                           esp_driver_cam
                           esp_driver_i2c
                           esp_driver_i2s
                           esp_driver_ledc
                           esp_driver_spi
                           esp_driver_uart
                           esp_lcd
                           esp_timer
                           esp_adc
                           nvs_flash
                           lvgl
                           espressif__esp_new_jpeg
                           esp_wifi
                           esp_netif
                           wifi_provisioning
                           protocomm
                           esp_http_server)

target_link_libraries(${COMPONENT_LIB} INTERFACE
                      "-Wl,--wrap=spi_device_queue_trans")
target_link_libraries(${COMPONENT_LIB} INTERFACE
                      "-u __wrap_spi_device_queue_trans")
target_link_libraries(${COMPONENT_LIB} INTERFACE
                      "-Wl,--wrap=spi_device_polling_transmit")
target_link_libraries(${COMPONENT_LIB} INTERFACE
                      "-u __wrap_spi_device_polling_transmit")
