#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Shared I2C0 bus + TCA9554A GPIO expander -------------------- */
esp_err_t bsp_i2c_init(void);
i2c_master_bus_handle_t bsp_i2c_bus(void);

/* Probe each 7-bit address on I2C0 and log the ones that ACK. Handy for
 * bringing up a new board. Returns ESP_OK if the scan ran; the caller can
 * read the log output to see which addresses responded. */
esp_err_t bsp_i2c_scan(void);

typedef enum {
    BSP_CON6_PERIPHERAL_LCD_ST7789 = 0,
    BSP_CON6_PERIPHERAL_CAMERA_GC032A,
} bsp_con6_peripheral_t;

/* Detect what is attached to CON6.  GC032A has priority when its I2C address
 * ACKs; otherwise the connector is treated as the ST7789T3 LCD variant. */
esp_err_t bsp_con6_detect(bsp_con6_peripheral_t *out_peripheral);

/* Set a single pin on the TCA9554A expander (configures it as output if it
 * is not already). Pins used by the BSP: P0/P1/P2 = RGB LED, P3 = LCD reset,
 * P6 = PA enable. */
esp_err_t bsp_ioexp_set_pin(uint8_t pin, bool level);

/* ---------- LCD (ST7789T3 on CON6 4-wire SPI) --------------------------- */
esp_err_t bsp_lcd_init(void);
esp_err_t bsp_lcd_show_test_pattern(void);
esp_err_t bsp_lcd_start_lvgl_demo(void);

/* ---------- RGB LED ----------------------------------------------------- */
esp_err_t bsp_led_init(void);
void      bsp_led_set(bool r, bool g, bool b);

/* ---------- Audio (ES8311 codec + CST8302A PA + I2S) -------------------- */
esp_err_t bsp_audio_init(uint32_t sample_rate_hz);
esp_err_t bsp_audio_pa_enable(bool on);                  /* PA enable via P6 */
esp_err_t bsp_audio_set_volume(uint8_t volume_percent);  /* 0..100 for DAC  */
esp_err_t bsp_audio_set_mic_gain_db(uint8_t gain_db);    /* 0..42 PGA       */
esp_err_t bsp_audio_write(const void *buf, size_t bytes, size_t *out_written);
esp_err_t bsp_audio_read (void       *buf, size_t bytes, size_t *out_read);

/* ---------- Buttons ----------------------------------------------------- */
typedef enum {
    BSP_BTN_BOOT = 0,   /* GPIO0 strap button (K2)     */
    BSP_BTN_USER1,      /* K3 -> ~1.11 V on KEY_ADC    */
    BSP_BTN_VOL_DN,     /* K4 -> ~2.41 V               */
    BSP_BTN_PTT,        /* K5 -> ~1.65 V               */
    BSP_BTN_VOL_UP,     /* K6 -> ~0.82 V               */
    BSP_BTN_COUNT,
} bsp_btn_id_t;

typedef void (*bsp_btn_cb_t)(bsp_btn_id_t id, bool pressed, void *user);

esp_err_t bsp_button_init(bsp_btn_cb_t cb, void *user);

#ifdef __cplusplus
}
#endif
