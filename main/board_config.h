/*
 * Pin map for the Lierda L-LRMAM36-FANN4-DK01 development board.
 *
 * The L-LRMAM36-FANN4 module contains both an ESP32-S3 MCU and a Semtech LR2021
 * radio; the ESP32-S3 <-> LR2021 SPI wiring is internal to the module and is
 * configured by the esp_lora_driver component (see Kconfig / sdkconfig).
 *
 * This header only documents the GPIOs the module brings out to the
 * development board's external peripherals.
 */

#pragma once

#include "driver/gpio.h"

/* ---------- UART0 (CP2105 bridge - console/programming) ------------------ */
/* Routed through the module's TX0/RX0 pins; pins are handled by ROM bootloader
 * and the default ESP-IDF UART0 console, no need to set manually. */

/* ---------- I2C0 (ES8311 codec, TCA9554A GPIO expander, touch panel) ----- */
#define BSP_I2C0_SCL_GPIO           GPIO_NUM_45
#define BSP_I2C0_SDA_GPIO           GPIO_NUM_46
#define BSP_I2C0_FREQ_HZ            (400 * 1000)

#define BSP_I2C_ADDR_ES8311         0x18   /* Audio codec                     */
#define BSP_I2C_ADDR_IO_EXPANDER    0x38   /* TCA9554A, confirmed by scan     */
/* Touch panel I2C address depends on the controller fitted on the FPC. */

/* ---------- I2S0 (ES8311 codec audio path) ------------------------------- */
#define BSP_I2S0_MCLK_GPIO          GPIO_NUM_18
#define BSP_I2S0_BCLK_GPIO          GPIO_NUM_1
#define BSP_I2S0_LRCK_GPIO          GPIO_NUM_15
#define BSP_I2S0_DIN_GPIO           GPIO_NUM_16   /* ESP -> codec (SDIN)     */
#define BSP_I2S0_DOUT_GPIO          GPIO_NUM_14   /* codec -> ESP (SDOUT)    */

/* Audio PA (CST8302A) mute/enable is on the GPIO expander P6 (active high). */
#define BSP_IO_EXP_PA_MUTE_PIN      6

/* ---------- QSPI LCD (dual-display capable, 0.5 mm 20 pin FPC) ----------- */
#define BSP_LCD_QSPI_CLK_GPIO       GPIO_NUM_12
#define BSP_LCD_QSPI_D0_GPIO        GPIO_NUM_13
#define BSP_LCD_QSPI_D1_GPIO        GPIO_NUM_2
#define BSP_LCD_QSPI_D2_GPIO        GPIO_NUM_9
#define BSP_LCD_QSPI_D3_GPIO        GPIO_NUM_4
#define BSP_LCD_CS1_GPIO            GPIO_NUM_10   /* primary display CS      */
#define BSP_LCD_CS2_GPIO            GPIO_NUM_8    /* secondary display CS    */
#define BSP_LCD_TE_GPIO             GPIO_NUM_6    /* tearing effect          */
#define BSP_LCD_BL_GPIO             GPIO_NUM_7    /* backlight enable        */

/* LCD reset is on the GPIO expander P7 (active low). */
#define BSP_IO_EXP_LCD_RST_PIN      7

/* ---------- Capacitive touch panel (shares I2C0) ------------------------- */
#define BSP_TP_INT_GPIO             GPIO_NUM_11
#define BSP_TP_RST_GPIO             GPIO_NUM_3

/* ---------- RGB status LED (driven by the GPIO expander) ----------------- */
#define BSP_IO_EXP_LED_G_PIN        0
#define BSP_IO_EXP_LED_R_PIN        1
#define BSP_IO_EXP_LED_B_PIN        2

/* ---------- User buttons ------------------------------------------------- */
/*
 * K1 -> CHIP_PU (reset, hardwired)
 * K2 -> GPIO0 / BOOT (strap button)
 * K3..K6 share a resistor-divider ADC ladder on GPIO5:
 *     K4 (27 k)  -> ~2.41 V  "VOL-"
 *     K5 (10 k)  -> ~1.65 V  "PTT"
 *     K6 (3.3 k) -> ~0.82 V  "VOL+"
 *     K3 (5.1 k) -> ~1.11 V  "USER1"
 * Read via ADC1 channel on GPIO5.
 */
#define BSP_KEY_ADC_GPIO            GPIO_NUM_5
#define BSP_KEY_ADC_UNIT            ADC_UNIT_1
#define BSP_KEY_ADC_CHANNEL         ADC_CHANNEL_4   /* GPIO5 = ADC1_CH4       */

#define BSP_BOOT_KEY_GPIO           GPIO_NUM_0

/* ---------- LR2021 radio (internal to the module, pre-wired) ------------- *
 * Configured by esp_lora_driver when CONFIG_LIERDA_BOARD_LRMAM36_FANN4 = y:
 *   SPI host   : SPI2
 *   NSS        : GPIO39
 *   SCLK       : GPIO40
 *   MOSI       : GPIO41
 *   MISO       : GPIO42
 *   BUSY       : GPIO17
 *   NRST       : GPIO38
 *   DIO7 (IRQ) : GPIO21
 * These are not available on the module's external pads.
 */
