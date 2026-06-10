#pragma once
#include "esp_err.h"
#include <stdint.h>

/* GYSDISPLAY 1.5" 200x200 4-colour EPD (UC8151C / IL0373 controller).
 * SPI3, BUSY=GPIO4, DC=GPIO48, CS=GPIO47, RST via IO-expander P3.
 *
 * Colour plane encoding (two 200x200 bit planes, BW + RY):
 *   BW plane  RY plane  ->  pixel colour
 *      0          0     ->  black
 *      1          0     ->  white
 *      0          1     ->  red/orange
 *      1          1     ->  yellow
 */

#define EPD_WIDTH  200
#define EPD_HEIGHT 200

typedef enum {
    EPD_BLACK  = 0,
    EPD_WHITE  = 1,
    EPD_RED    = 2,
    EPD_YELLOW = 3,
} epd_color_t;

esp_err_t epd_init(void);

/* Clear both planes and full-refresh. */
esp_err_t epd_clear(epd_color_t bg);

/* Draw ASCII text at pixel position (x, y) using built-in 8x8 font.
 * fg / bg control glyph foreground and background colour. */
esp_err_t epd_print_text(int x, int y, const char *text,
                         epd_color_t fg, epd_color_t bg);

/* Push framebuffer to panel and trigger full refresh. */
esp_err_t epd_refresh(void);

/* Deep-sleep the controller to save power. */
esp_err_t epd_sleep(void);
