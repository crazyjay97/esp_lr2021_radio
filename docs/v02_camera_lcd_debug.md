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
device ACK @ 0x39
```

### I2C Devices

| Device | Address | Notes |
| --- | --- | --- |
| ES8311 | 0x18 | Confirmed by scan |
| FT6206 touch | 0x38 | User confirmed the chip is FT6206 |
| TCA9554A IO expander | 0x39 | User changed A0 and confirmed the expander address moved to `0x39`; TCA9554A valid range is `0x38..0x3F` |
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

User re-checked the LCD FPC pin map and confirmed the panel has a separate
`LCD_DC` line. The firmware drives ST7789V3 as ordinary 4-wire SPI; only
`MOSI/CLK/CS/DC/RST/BL` are required for basic display. TE is not used.

| LCD signal | GPIO / IO expander |
| --- | --- |
| LCD_CLK | GPIO12 |
| LCD_MOSI | GPIO2 |
| LCD_DC | GPIO13 |
| LCD_CS | GPIO10 |
| LCD_TE | NC for firmware bring-up |
| LCD_RST | IO expander P3 |
| LCD_LED / backlight | IO expander P5 |
| TP_INT | GPIO11 |
| TP_RST | IO expander P4 |

Important uncertainty:

- Earlier firmware used the wrong LCD pin group and then briefly tried 3-line 9-bit serial.
- Corrected firmware uses `LCD_DC=GPIO13`, `LCD_MOSI=GPIO2`, `LCD_CLK=GPIO12`, and `LCD_CS=GPIO10`.
- `LCD_D2/GPIO9`, `LCD_D3/GPIO4`, `LCD_CS2/GPIO8`, and `LCD_TE/GPIO6` are not needed for the ordinary SPI bring-up path.
- Resolution is set to the previously verified screen default: `240x320`, with `x_gap=0`, `y_gap=0`.

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
| GPIO2 | LCD_MOSI | D0 |
| GPIO10 | LCD_CS | Camera_RST |
| GPIO13 | LCD_DC | D2 |

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

### FT6206 touch at 0x38 and TCA9554A at 0x39

Observed log after I2C pin fix:

```text
device ACK @ 0x18
device ACK @ 0x38
device ACK @ 0x39
```

User confirmed:

- Touch IC is FT6206.
- FT6206 address is `0x38`.
- TCA9554A A0 was changed and the IO expander address is now `0x39`.

Implication:

- The scanned `0x38` is the FT6206 touch controller.
- The scanned `0x39` is the TCA9554A IO expander.
- Firmware probes FT6206 at `0x38`.
- Firmware attaches TCA9554A at `0x39`.
- If fallback scanning is needed, firmware scans TCA9554A valid range `0x38..0x3F` but skips the FT6206 touch address `0x38`.
- LCD reset/backlight and touch reset through IO expander are now warning-only during bring-up so LCD/touch can still be tested.

Latest observed failure:

```text
I bsp_i2c: skip 0x38 while scanning TCA9554A: FT6206 touch address
E bsp_i2c: no TCA9554A ACK in 0x38..0x3F after skipping FT6206 touch address
```

Implication:

- Firmware is no longer treating FT6206 `0x38` as the IO expander.
- Configured TCA9554A address `0x39` is not ACKing.
- Re-check TCA9554A A0/A1/A2 strap levels. For TCA9554A, expected address is `0x38 + A2:A1:A0`.
- User confirmed ES8311, FT6206, and TCA9554A are on the same I2C bus in the schematic, so do not treat this as a different-I2C-segment issue.
- Re-check TCA9554A VCC/GND, A0/A1/A2 strap levels, and soldering around its SDA/SCL pins.
- If the full boot scan still only shows `0x18` and `0x38`, the TCA9554A itself is not visible on the shared bus yet.
- I2C speed can be a factor if pull-ups, trace capacitance, or soldering margin are weak. Firmware was temporarily lowered from 400 kHz to 100 kHz for bring-up. If `0x39` appears only at 100 kHz, inspect pull-up value, waveform rise time, and bus capacitance.

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
   - I2C scan sees `0x18`, `0x38`, and `0x39`.
   - `0x38` touch attach succeeds as FT6206-compatible.
   - TCA9554A attach logs `TCA9554A attached at 0x39`.
3. Confirm LCD:
   - backlight turns on,
   - ST7789V3 init does not fail,
   - LCD test color bars appear for about 800 ms,
   - LVGL UI appears after the color bars.
4. Confirm touch:
   - touch attach logs show `0x38` selected as FT6206-compatible,
   - tapping `Capture` triggers capture task.
5. Confirm camera:
   - during capture, SP0A39 ID read succeeds,
   - `/dev/video2` opens,
   - one GREY frame is captured.
6. If LCD remains blank:
   - current symptom from 2026-06-04: LCD backlight is on, but LVGL content is not visible,
   - firmware now draws a hardware color-bar test pattern before LVGL, using 10 MHz SPI for bring-up,
   - if color bars are visible but LVGL is not, debug LVGL flush/tick/buffer path,
   - if color bars are also invisible, debug LCD interface/reset/init first,
   - current firmware uses ST7789V3 4-wire SPI with `LCD_DC=GPIO13`,
   - verify whether the panel module expects `LCD_CS` or `LCD_CS2`,
   - verify whether LCD reset/backlight really go through TCA9554A P3/P5.
7. If capture fails:
   - inspect SP0A39 SCCB ACK after PWDN low and reset,
   - verify VSYNC/HSYNC/PCLK polarity,
   - verify shared LCD pins are truly released before DVP init.
