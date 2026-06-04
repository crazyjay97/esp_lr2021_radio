# L-LRMAM36-FANN4-DK01 V02 Camera/LCD Debug Notes

Last updated: 2026-06-04 14:06 CST

## Goal

Bring up the V02 board schematic changes:

- SP0A39 camera uses 8-bit DVP again.
- LCD driver IC is ST7789V3.
- LCD UI uses LVGL.
- Touch is used to trigger a photo capture.
- Captured camera frame is displayed on the LCD to verify camera, LCD, and touch.

## Current Firmware Mode

`APP_CAMERA_LCD_BRINGUP` is enabled in `main/app_config.h`.

Boot flow:

1. Initialize I2C.
2. Scan I2C bus.
3. Keep SP0A39 powered down while LCD owns shared GPIOs.
4. Initialize ST7789V3 LCD.
5. Start LVGL touch UI.
6. Touch `Capture` to:
   - release LCD SPI bus,
   - power/reset SP0A39,
   - capture one DVP grayscale frame through `/dev/video2`,
   - power down SP0A39,
   - reinitialize LCD,
   - show the captured grayscale preview.

## Confirmed V02 Pin Map

### I2C0

The V02 schematic text was initially misread because old code still used GPIO45/GPIO46. Board logs confirmed the correct V02 I2C pins:

| Signal | GPIO |
| --- | --- |
| I2C0_SCL | GPIO18 |
| I2C0_SDA | GPIO16 |

Expected scan on the tested board:

```text
device ACK @ 0x18
device ACK @ 0x38
```

### I2C Devices

| Device | Address | Notes |
| --- | --- | --- |
| ES8311 | 0x18 | Confirmed by scan |
| FT6206 touch | 0x38 | Confirmed by user; current scan sees 0x38 |
| TCA9554A IO expander | 0x20..0x27 | User confirmed the expander is still TCA9554A; expected base address is currently `0x20`, but boot scan does not ACK any TCA9554A address yet |
| Touch old candidates | 0x15 or 0x2A | CST816-compatible candidates from earlier code, not present on this board |
| SP0A39 SCCB | 0x21 | May not appear in initial scan because camera is kept in PWDN while LCD owns shared lines |

### Audio I2S

Adjusted to avoid conflict with V02 I2C:

| Signal | GPIO |
| --- | --- |
| CODEC_I2S0_MCLK | GPIO46 |
| CODEC_I2S0_BCLK | GPIO1 |
| CODEC_I2S0_LRCK | GPIO15 |
| CODEC_I2S0_DIN | GPIO14 |
| CODEC_I2S0_DOUT | GPIO45 |

### ST7789V3 LCD

The current implementation treats the LCD as 4-wire SPI through `esp_lcd_new_panel_st7789`.

| LCD signal | GPIO / IO expander |
| --- | --- |
| LCD_CLK | GPIO44 |
| LCD_D0 / MOSI | GPIO48 |
| LCD_D1 / D-C in current firmware | GPIO43 |
| LCD_CS1 | GPIO47 |
| LCD_TE | GPIO4 |
| LCD_RST | IO expander P3 |
| LCD_LED / backlight | IO expander P5 |
| TP_INT | GPIO12 |
| TP_RST | IO expander P4 |

Important uncertainty:

- V02 schematic labels the connector as `QSPI dual LCD + Touch panel`.
- There is no explicit D/C label in the PDF.
- Current firmware assumes `LCD_D1/GPIO43` can act as ST7789 SPI D/C.
- If the panel is actually QSPI-style ST7789V3 instead of 4-wire SPI, LCD init will need a different IO mode.

### SP0A39 DVP Camera

| Camera signal | GPIO / IO expander |
| --- | --- |
| VSYNC | GPIO7 |
| HSYNC / DE | GPIO6 |
| MCLK / XCLK | GPIO3 |
| PCLK | GPIO8 |
| D0 | GPIO2 |
| D1 | GPIO9 |
| D2 | GPIO13 |
| D3 | GPIO4 |
| D4 | GPIO43 |
| D5 | GPIO44 |
| D6 | GPIO48 |
| D7 | GPIO47 |
| Camera_RST | GPIO10 |
| PWDN | IO expander P7, active high |

## Shared GPIO Constraint

LCD and camera share several ESP GPIOs:

| GPIO | LCD use | Camera use |
| --- | --- | --- |
| GPIO4 | LCD_TE | D3 |
| GPIO43 | LCD_D1 / D-C currently | D4 |
| GPIO44 | LCD_CLK | D5 |
| GPIO48 | LCD_D0 / MOSI | D6 |
| GPIO47 | LCD_CS1 | D7 |

Current workaround:

- LCD owns these pins during UI/display.
- Before capture, firmware suspends LVGL flushes, deletes LCD panel IO, frees the SPI bus, and turns backlight off.
- Camera is then powered/reset and DVP capture starts.
- After capture, camera is powered down and LCD is reinitialized.

## Issues Found And Fixed

### I2C scan returned 0 devices

Observed log:

```text
I bsp_i2c: scan done, 0 device(s)
E bsp_i2c: ioexp write failed (ESP_ERR_INVALID_STATE)
```

Root cause:

- Firmware was still using old I2C pins `GPIO45/GPIO46`.

Fix:

- `BSP_I2C0_SCL_GPIO = GPIO18`
- `BSP_I2C0_SDA_GPIO = GPIO16`

### FT6206 touch at 0x38 vs TCA9554A uncertainty

Observed log after I2C pin fix:

```text
device ACK @ 0x18
device ACK @ 0x38
```

User confirmed:

- Touch IC is FT6206.
- FT6206 normally uses `0x38`.

Implication:

- The scanned `0x38` is likely the FT6206 touch controller, not the IO expander.
- Treating `0x38` as TCA9554A and writing P3/P4/P5/P7 can corrupt touch-controller registers.
- Firmware was changed to probe FT6206 at `0x38` and to stop assuming the IO expander is at `0x38`.
- IO expander remains configured as TCA9554A address `0x20`.
- Firmware now scans only the TCA9554A valid address range `0x20..0x27` when attaching the IO expander.
- Current boot scan does not ACK any address in `0x20..0x27`, so check TCA9554A address straps, power, reset/enable if present, soldering, or whether it is on a different I2C segment.
- LCD reset/backlight and touch reset through IO expander are now warning-only during bring-up so LCD/touch can still be tested.

### App printed ready even after init failed

Observed:

```text
E app: camera init: ESP_ERR_INVALID_STATE
I app: V02 camera/LCD validation UI ready...
```

Fix:

- In V02 bring-up mode, `app_main()` now returns immediately after camera, LCD, or UI init failure.

## Build Status

Verified:

```text
idf.py build
```

Current build result:

```text
Project build complete
Generated build/lr2021_radio.bin
```

## Next Debug Checklist

1. Reflash after IO expander address fix.
2. Confirm boot log:
   - I2C scan sees `0x18` and `0x38`.
   - `0x38` touch attach succeeds as FT6206-compatible.
   - TCA9554A attach logs either `TCA9554A attached at 0x20` or reports no ACK in `0x20..0x27`.
3. Confirm LCD:
   - backlight turns on,
   - ST7789V3 init does not fail,
   - LVGL UI appears.
4. Confirm touch:
   - touch attach logs show `0x38` selected as FT6206-compatible,
   - tapping `Capture` triggers capture task.
5. Confirm camera:
   - during capture, SP0A39 ID read succeeds,
   - `/dev/video2` opens,
   - one GREY frame is captured.
6. If LCD remains blank:
   - current symptom from 2026-06-04: LCD backlight is on, but LVGL content is not visible,
   - re-check whether ST7789V3 module is 4-wire SPI or QSPI,
   - verify whether `LCD_D1/GPIO43` is a valid D/C line,
   - verify whether LCD reset/backlight really go through TCA9554A P3/P5 and why `0x20..0x27` is not ACKing.
7. If capture fails:
   - inspect SP0A39 SCCB ACK after PWDN low and reset,
   - verify VSYNC/HSYNC/PCLK polarity,
   - verify shared LCD pins are truly released before DVP init.
