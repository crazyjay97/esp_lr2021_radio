# SP0A39 Image Sensor Specification (Markdown Conversion)

This document is a Markdown rendition of the **SP0A39** image sensor specification (Commercial version 1.5, dated 2018‑11‑02).  The SP0A39 is a 1/10‑inch 0.3‑megapixel CMOS image sensor targeted at mobile devices such as mobile phones, tablets and PC/web cameras.  It provides VGA‑resolution (640×480 pixels) output at 30 frames per second and integrates extensive on‑chip image processing functions.  The following sections mirror the original PDF specification.

## Overview

### General Description

The **SP0A39** sensor is a cost‑effective, high‑performance 1/10‑inch 0.3‑megapixel CMOS image sensor for capturing still images and HD video.  It uses a 2.2 μm × 2.2 μm square pixel and supports operating temperatures from **0 °C to 50 °C** (stable operation) and **–20 °C to 70 °C** (operating range).  Key features include windowing, black‑level calibration, auto white balance (AWB), auto exposure (AE), gamma correction, lens shading correction and support for high frame rate through DVP or SPI output interfaces【7†L9-L14】.

### Function Diagram

The sensor’s functional blocks include:

- Auto black level calibration
- Auto white balance
- Auto exposure control
- Gamma correction
- Lens shading calibration
- De‑mosaic and de‑noise functions
- Color correction
- Defect pixel correction
- DVP and SPI output interfaces【7†L9-L14】

These blocks operate on the pixel array to produce processed image data which can be output via DVP or SPI.

### Typical Applications

The SP0A39 targets small‑form‑factor imaging applications:

| Application | Examples |
|---|---|
| **Mobile devices** | Mobile phone cameras, tablets |
| **PC/web cameras** | Laptop webcams, USB web cameras |
| **Consumer electronics** | Toys and other imaging gadgets【7†L16-L18】 |

### Key Performance Parameters

| Parameter | Value | Notes |
|---|---|---|
| **Active pixel array** | 640 × 480 (VGA) | 0.3 megapixels |
| **Pixel size** | 2.2 µm × 2.2 µm | Square pixel |
| **Lens size** | 1/10‑inch | — |
| **Chief ray angle (CRA)** | 29.99° | See CRA information table |
| **Color filter** | Bayer pattern with primary colors | — |
| **Power supply** | 1.7–3.0 V (I/O), 2.6–3.0 V (analog) | — |
| **Power consumption** | Active < 70 mW, Standby < 30 µA | Measured at typical voltages |
| **Data format** | YUV 4:2:2, RAW8, Y‑only | — |
| **Output interfaces** | DVP (8‑bit), SPI (1/2/4‑bit) | — |
| **Input clock** | 6–48 MHz | External clock source |
| **Maximum frame rate** | 30 fps @ VGA | — |
| **Operating temperature** | –20 °C to 70 °C | Stable 0–50 °C |
| **Package** | CSP (Chip‑Scale Package) | —【8†L1-L15】 |

### Feature Summary

* Supports VGA (640×480 pixels) output at up to 30 fps.
* 2.2 μm × 2.2 μm pixel architecture with high quantum efficiency and low‑light sensitivity.
* Embedded image pre‑processing: auto black level calibration, auto white balance, auto exposure control, gamma correction, lens shading correction, de‑mosaic and de‑noise functions, color correction, defect pixel correction, windowing and special effects.
* Supports **2×2 mono binning** mode.
* On‑chip I²C register control, external frame synchronization, and dual output interfaces (DVP and SPI)【9†L1-L10】.

## Function Description

### Pixel Array Structure

The SP0A39 uses a pixel array of **656 columns × 500 rows**, of which **648 × 488** constitute active pixels.  The array includes dummy and boundary pixels to allow windowing and calibration.  The first pixel at the right‑top corner uses a **blue** color filter【10†L1-L7】.

### Image Signal Processing

The sensor integrates a suite of on‑chip image processing features:

- **Mirror and Flip:** configurable horizontal/vertical mirroring controlled by register `0x31[2:1]` (00 = normal, 01 = mirror, 10 = flip, 11 = mirror + flip)【12†L1-L16】.
- **Test Pattern:** built‑in color bar test pattern for system testing.
- **Auto Black Level Calibration (BLC):** uses optically black pixels to automatically calibrate the black level across the frame【13†L1-L7】.
- **Auto White Balance (AWB):** adjusts color balance using pre‑gain settings to compensate for lighting conditions【13†L8-L14】.
- **Auto Exposure Control (AEC):** adjusts integration time, analog gain and digital gain to achieve proper luminance based on Y (luminance) values【13†L14-L18】.
- **Gamma Correction:** compensates sensor response non‑linearity using a configurable gamma curve【13†L18-L22】.
- **Lens Shading Calibration:** corrects vignetting by applying per‑pixel gain tables derived from reference frames【13†L23-L27】.
- **De‑mosaic Function:** converts raw Bayer data to RGB using an edge‑sensitive interpolation algorithm【14†L1-L7】.
- **De‑noise Function:** reduces noise while preserving edges【14†L8-L11】.
- **Color Correction:** applies color correction matrices for improved color fidelity【14†L11-L15】.
- **Defect Pixel Correction (DPC):** detects and replaces defective pixels using neighbor interpolation【14†L15-L20】.
- **Special Effects:** supports monochrome, emboss, sketch, neon and other effects with corresponding example images【14†L20-L24】.

### Output Interfaces

Two output interfaces are available:

1. **DVP (Digital Video Port):** an 8‑bit parallel interface with VSYNC/HSYNC signaling.  It supports multiple formats—full‑resolution YUV or RAW/Y, subsampled (320×240) and 2×2 binning.  DVP clock frequencies depend on the mode (e.g., 24 MHz for full‑resolution YUV at 30 fps)【16†L1-L14】.
2. **SPI:** a serial interface supporting 1‑/2‑/4‑bit transfers for compact connections【16†L1-L6】.

### I²C Bus Protocol

The sensor’s registers are accessed over an I²C bus using device addresses **0x42 (write)** and **0x43 (read)**.  Single‑byte read/write transactions follow standard I²C timing.  Key bus parameters include a maximum clock of **400 kHz**, setup/hold times and acknowledge timing【20†L1-L13】.  The timing parameters table defines minimum and maximum values for each I²C timing symbol【21†L1-L14】.

### Electrical Characteristics

#### DC Specifications

| Symbol | Description | Min | Typ | Max | Unit |
|---|---|---|---|---|---|
| **AVDD** | Analog/I/O supply voltage | 2.6 | 2.8 | 3.0 | V |
| **DVDDIO** | Digital I/O supply voltage | 1.7 | 1.8 | 2.0 | V |
| **VIH** | Input high voltage | 0.7 × DVDDIO | — | 3.0 | V |
| **VIL** | Input low voltage | 0 | — | 0.3 × DVDDIO | V |
| **VOH** | Output high voltage @ 8 mA | 0.7 × DVDDIO | — | — | V |
| **VOL** | Output low voltage @ 8 mA | 0 | — | 0.3 × DVDDIO | V |
| **Tj** | Junction temperature | –20 | 25 | 70 | °C |

#### Power Consumption (at AVDD = 2.8 V, DVDDIO = 1.8 V)

| Mode | Supply current | Condition |
|---|---|---|
| **IDVDDIO** | 15.5 mA | DVP output, 30 fps @ VGA |
| **IAVDD** | 7.2 mA | DVP output, 30 fps @ VGA【22†L1-L10】 |

#### Absolute Maximum Ratings

| Parameter | Rating |
|---|---|
| **Supply voltages (AVDD, DVDDIO)** | up to 4.5 V |
| **Input/output voltages** | –0.3 V to (DVDDIO + 1 V) |
| **Storage temperature** | –40 °C to 150 °C |
| **ESD ratings** | Human body model – 3000 V; Charge device model – 500 V; Machine model – 200 V |
| **Peak solder temperature** | 260 °C (10 s)【23†L1-L10】 |

## Power‑Up and Power‑Off Sequence

Proper sequencing of power rails, clock and I²C initialization is required:

### Power‑Up Sequence

1. **AVDD on** (2.6–3.0 V) → delay T1.
2. **DVDDIO on** (1.7–3.0 V) after T1; wait for DVDDIO to stabilize (T2 ≥ 5 ms).
3. **MCLK** input starts after AVDD (T3 ≥ 0 ms).
4. **I²C initialization** is performed T4 ≥ 4 ms after the sensor power is stable.  PWDN must remain low at all times【24†L1-L15】.

### Power‑Off Sequence

1. **Disable MCLK** (T5 ≥ 0 ms).
2. **Power‑down sensor** using the PWDN pin; wait T6 ≥ 0 ms.
3. **Turn off DVDDIO** followed by **AVDD** (T7 ≥ 0 ms)【25†L1-L10】.

## Chief Ray Angle and Pixel Array Dimensions

The chief ray angle (CRA) is the angle at which incident light is most efficiently captured.  The SP0A39 supports a CRA up to ~30°, with field and image height values tabulated in the CRA graph.  The pixel array dimensions (RIC) relative to the image center are defined in the pixel array information table【26†L1-L8】.

## Package Description

The sensor is packaged in a chip‑scale package (CSP) with a **top view** (image side) and **BGA bottom view**.  The package centre aligns with the chip centre, and the pixel array has an offset relative to the package centre.  The dimensions table gives body dimensions, ball pitch and ball diameter.  Ball names and their corresponding pins are summarized below【29†L1-L10】.

### Ball Pinout and Descriptions

| Pin | Ball name | I/O | Description |
|---|---|---|---|
| A2 | **VSYNC** | I/O | Vertical sync signal |
| A3 | **PWDN** | I | Power‑down control (active low) |
| A4 | **D1/SPI_D1** | O | DVP DATA[3] (default) or SPI DATA<3> |
| A5 | **D0/SPI_D0** | O | DVP DATA[2] (default) or SPI DATA<2> |
| A7 | **PCLK/SPICLK** | O | Pixel clock or SPI clock |
| B3 | **MCLK** | I | External master clock |
| B4 | **D2** | O | DVP DATA[1] (default) or SPI DATA<1> |
| B5 | **D3** | O | DVP DATA[0] (default) or SPI DATA<0> |
| C5 | **AGND** | G | Analog ground |
| C6 | **DGND** | G | Digital ground |
| D1 | **VSYNC** | I/O | Vertical sync (same as A2) |
| D2 | **SBCL** | I/O | I²C clock input |
| D3 | **SBDA** | I/O | I²C data input/output |
| D7 | **EVSYNC** | O | External vsync |
| D9 | **AVDD** | P | Analog power supply【31†L1-L8】 |

## Register Map

The majority of the specification documents the register map for the SP0A39 sensor and its image signal processor (ISP).  Registers are grouped by page (P0 = sensor registers, P1/P2 = ISP registers).  Each entry includes the address, name, bit fields, description and default value.  The tables below summarise the register map in Markdown format.

### Sensor Registers (Page 0)

| Address | Register name | Bits | Description | Default |
|---|---|---|---|---|
| **P0:0x00** | `chip ID` | 7:0 | Chip ID (high byte) | `0x0A` |
| **P0:0x01** | `chip ID` | 7:0 | Chip ID (low byte) | `0x39` |
| **P0:0x03** | `exp_3msb` | 2:0 | Exposure time (high 3 bits) | `0x00` |
| **P0:0x04** | `exp_8lsb` | 7:0 | Exposure time (low 8 bits) | `0x9A` |
| **P0:0x06** | `vblank_buf_8lsb` | 7:0 | Vsync blank | `0x00` |
| **P0:0x07** | `clk_cp_sel`, `ie_date7`, `ie_vs`, `buf_dis_d`, `buf_dis_s`, `sa_clk_delay` | 7:0 | Clock selection and delays | `0x98` |
| **P0:0x08** | `vpos_blank` | 7:0 | Vpos blank | `0x00` |
| **P0:0x09** | `hblank_4msb` | 3:0 | Horizontal blank (high bits) | `0x00` |
| **P0:0x0A** | `hblank_8lsb` | 7:0 | Horizontal blank (low 8 bits) | `0x00` |
| **P0:0x0B** | `exter_sync_ctl` | 7:0 | External sync control | `0x00` |
| **P0:0x0C** | `exter_sync_out_width` | 7:0 | External sync pulse width | `0x08` |
| **P0:0x0D** | `frame_exp_separate_en`, `timing_test_en`, `frame_length_sel` | 4:0 | Frame/exposure control | `0x00` |
| **P0:0x0E** | `frame_length_num[10:8]` | 2:0 | Frame length (high bits) | `0x01` |
| **P0:0x0F** | `frame_length_num[7:0]` | 7:0 | Frame length (low 8 bits) | `0xF2` |
| **P0:0x13** | `cp_num`, `spi_clk_delay`, `vcp_sel_ctl` | 5:0 | Charge pump and SPI timing | `0x24` |
| **P0:0x14** | `ds_hsync`, `ds_vsync`, `ds_pclk` | 5:0 | DVP sync delays | `0x00` |
| **P0:0x15** | `spi_data0_delay`–`spi_data3_delay` | 7:0 | SPI data lane delays | `0x00` |
| **P0:0x16** | `dac_mode`, `FPN_33ms_timing_sel` | 1:0 | DAC mode and FPN timing | `0x00` |
| **P0:0x17** | `exter_sync_frame_num` | 7:0 | External sync frame number | `0x0A` |
| **P0:0x18** | `rst_del_en`, `rst_colrow_en` | 4:0 | Reset delays | `0x00` |
| **P0:0x19** | `icomp1`, `icomp2` | 6:0 | Internal compensation currents | `0x42` |
| **P0:0x1C** | `out_en_hs`, `out_en_vs`, `out_enp`, `out_end7`, `out_end_high4`, `out_en_data` | 7:0 | Output enable control | `0xFF` |
| **P0:0x1D** | `ds_i2c`, `ds_data`, `pd_dig` | 7:0 | Data streaming and power‑down bits | `0x04` |
| **P0:0x21–0x22** | `row_length_readonly` | 12:0 | Read‑only row length | `0x000` |
| **P0:0x23** | `rpc` | 7:0 | Reset/pixel control | `0x20` |
| **P0:0x24** | `pga_gain_ctl` | 7:0 | PGA gain control | `0x20` |
| **P0:0x25** | `ipix` | 1:0 | Pixel current control | `0x02` |
| **P0:0x2C** | `spi_ctrl2` | 5:0 | Additional SPI control | `0x00` |
| **P0:0x2E** | `spi_out_control` | 7:0 | SPI output control | `0x00` |
| **P0:0x30** | `clk_mode` | 7:0 | Clock mode and division (dac_clk invert enable, pclk division, dclk control) | `0x01` |
| **P0:0x31** | `comm_ctrl_reg` | 5:0 | Communication control (mirror/flip bits) | `0x01` |
| **P0:0x32** | `ext_sync_sel`, `pvdd_sel` | 2:0 | External sync and power selection | `0x02` |
| **P0:0x35–0x36** | `glb_gain` | 8:0 | Global analog gain | `0x1E3` (LSB/bit0 = `0xE3`, MSB/bit8 = `0x00`) |
| **P0:0xDA** | `dat_format` | 7:0 | Data format selection (YUV/RAW) | `0x00` |
| **P0:0xE1–0xE4** | `black_level_*` | 7:0 | Black level calibration (GB/B/R/GR) | `0x00` |
| **P0:0xE7** | `reg_update_mode`, `reg_update_cmd` | 1:0 | Register update control | `0x00` |
| **P0:0xE8** | `rpc_tmp1` | 7:0 | Temporary RPC setting | `0x20` |
| **P0:0xF0–0xF3** | `gb_suboffset`, `gr_suboffset`, `red_suboffset`, `blue_suboffset` | 7:0 | Suboffsets for BLC | `0x00` |
| **P0:0xF4** | `i2c_dev_addr` | 7:1 | I²C device address (default 0x42) | `0x42` |
| **P0:0xF8** | `bl_position_set` | 1:0 | Black level position setting | `0x00` |
| **P0:0xFB** | `abl` | 5:0 | Auto black level update control | `0x36` |
| **P0:0xFC** | `blc_dpc_th_p_8lsb` | 7:0 | Positive defect threshold for BLC DPC | `0x40` |
| **P0:0xFE** | `blc_dpc_th_n_8lsb` | 7:0 | Negative defect threshold for BLC DPC | `0xC0` |
| **P0:0xFD** | `page_flg` | 1:0 | Page select | `0x00` |

*(Only selected registers are shown; the full map includes additional control and status registers defined in the specification.)*

### ISP Registers (Pages 1–2)

The ISP registers control auto white balance, auto exposure, gamma, color correction, lens shading, defect pixel correction and other post‑processing functions.  Below is a condensed table highlighting notable registers.  Registers are grouped by page (P1 for core ISP functions and P2 for extended functions and calibration tables).

#### P1 Registers (Image Processing Control)

| Address | Register name | Bits | Description | Default |
|---|---|---|---|---|
| **P1:0x00** | `fix_state_en`, `fix_state_mode` | 4:0 | Fixed state enable and mode | `0x00` |
| **P1:0x01** | `ft_test_status` | 1:0 | Factory test status (read‑only) | `0x00` |
| **P1:0x02–0x05** | `exp_max_indr`, `exp_min_indr`, `rpc_max_indr`, `rpc_min_indr` | 7:0 | Indoor exposure and reset/pixel thresholds | `0x0C`, `0x01`, `0xC0`, `0x24` |
| **P1:0x06–0x0B** | `exp_max_outdr`, `exp_min_outdr`, `rpc_max_outdr`, `rpc_min_outdr` | 7:0 | Outdoor exposure and reset/pixel thresholds | various defaults |
| **P1:0x0E–0x17** | `heq_mean_nr`, `k_max`, `ku_outdoor`, … | 7:0 | Histogram equalization and AEC parameters | see defaults |
| **P1:0x18** | `test_done_328x40`, `error_flag_328x40`, `bist_en` | 2:0 | Built‑in self‑test flags | `0x00` |
| **P1:0x1B–0x1F** | `ku_offset`, `kl_offset`, `auto_contrast_ctrl`, `lsc_sig_ru/lu`, `lsc_mg_sel` | various | AWB and lens shading offsets | see defaults |
| **P1:0x26–0x27** | `dpc_range_ratio_*` | 7:0 | Defect pixel correction ranges | `0x99` |
| **P1:0x28–0x2D** | `v_start`, `v_size`, `h_start`, `h_size` | — | Image start positions and sizes for windowing | defaults set to VGA (480×640) |
| **P1:0x32** | `auto_mode` | 7:0 | Auto mode enable (AWB/AEC etc.) | `0x00` |
| **P1:0x33** | `lsc_dpc_en` | 7:0 | Lens shading and DPC enable bits | `0xEF` |
| **P1:0x34** | `isp_mode` | 7:0 | Enables gamma, sharpness and color correction | `0xEF` |
| **P1:0x35** | `outmode1` | 7:0 | Output mode: raw/YUV ordering, byte swaps, Y output enable | `0x00` |
| **P1:0x36** | `outmode2` | 7:0 | Scaling, subsampling and sync inversion | `0x00` |
| **P1:0x38–0x3C** | `dpc_vt_*` and `dpc_dt_eff` | 7:0 | DPC vertical and threshold settings | see defaults |
| **P1:0x47** | `solarize_en`, `sketch_en`, `mono_en`, `sepia_en`, `gray_neg_en`, `color_neg_en`, `enchase_en`, `emboss_en` | 7:0 | Special effect enables | `0x00` |
| **P1:0x48–0x49** | `cr_tone`, `cb_tone` | 7:0 | Tone control for special effects | `0x80`, `0x80` |
| **P1:0x5C** | `heq_auto_mode` | 2:0 | Current automatic mode (read‑only) | `0x01` |
| **P1:0x5D** | `lsc_position_set`, `awb_position_set`, `bayer_order` | 5:0 | Raw order settings for AWB and lens shading | `0x01` |
| **P1:0x5E** | `vsync_delay_num` | 3:0 | VSYNC delay number | `0x00` |
| **P1:0x62–0x65** | `dpc_grad_thr_*` | 7:0 | DPC gradient thresholds for different modes | `0x10` |
| **P1:0x6E–0x83** | `gamma_pt0` … `gamma_pt21` | 7:0 | Gamma correction points (22 coefficients) | see defaults |
| **P1:0x84–0x99** | `lsc_*` | 7:0 | Lens shading parameters for R/G/B channels (left/right/up/down corners) | see defaults |
| **P1:0x9C–0x9F** | `u_v_th_*` | 7:0 | UV thresholds for modes | `0xAA` or `0x77` |
| **P1:0xA0–0xA7** | `mean_compn`, `col_center`, `row_center`, `lum_limit`, `lum_set`, `black_vt`, `white_vt` | 7:0 | Luminance and position settings | see defaults |
| **P1:0xA8–0xAB** | `raw_dif_thr_*` | 7:0 | Raw difference thresholds for AWB classification | `0x04` |
| **P1:0xB1–0xBA** | `skin_sharp_delta`, `autosa_en`, `sat_u_s*`, `sat_v_s*`, `heq_offset` | 7:0 | Skin detection, saturation and HEQ offsets | see defaults |
| **P1:0xBF** | `y_mean_th` | 7:0 | Y‑mean threshold for brightness | `0xFF` |
| **P1:0xC0–0xC7** | `rpc_*base_max` | 7:0 | Maximum reset pixel control values for different exposure bases | see defaults |
| **P1:0xCB–0xCE** | `target_indr/outdr`, `lock_range`, `hold_range` | 7:0 | DPC thresholds for indoor/outdoor | see defaults |
| **P1:0xCF–0xE6** | Various `raw_*` and `sat_*` registers | 7:0 | Raw filter thresholds and saturation controls | see defaults |
| **P1:0xE7–0xEE** | `sat_bot`, `ae_thr_low`, `slope_k`, `ae_gain_min_ratio`, `ae_rst` | 7:0 | AEC control registers | see defaults |
| **P1:0xF1–0xF2** | `mean`, `lum_down_en`, `exp_max_en`, `exp_accr_sel`, `outdoor_mode_en`, `mean_mode_reg` | various | Frame luminance statistics and AEC modes | `0x00`, `0x6D` |
| **P1:0xF3–0xF7** | `rpc_12base_max`, `rpc_13base_max`, `y_bot_ae`, `y_top_ae`, `ABF_exp_base_8lsb` | 7:0 | Additional AEC/ABF settings | see defaults |
| **P1:0xFD** | `page_flg` | 1:0 | Page select | `0x00` |

#### P2 Registers (Extended ISP and Calibration)

Page 2 registers provide additional AWB thresholds, color temperature detection, gamma tables, auto saturation parameters, white region boundaries and sharpening parameters.  A partial summary:

| Address | Register | Description |
|---|---|---|
| **P2:0x00–0x01** | `raw_gb_gain`, `raw_gr_gain` | Raw green‑blue and green‑red gains (default `0x80`) |
| **P2:0x02–0x05** | `exp_outdoor_th_*`, `rpc_outdoor_th`, `skin_num_th2`, `para_m[8:0]` | Outdoor exposure threshold, reset pixel control threshold, skin detection thresholds and parameter `m` |
| **P2:0x08–0x0F** | `y_low_th`, `uv_low_th`, `k_up/low`, `b_up/low`, `skin_num_th`, `cet_ofst` | Skin detection boundaries in YUV space and center offsets |
| **P2:0x10–0x11** | `br_offset`, `br_offset_f` | Brightness offsets |
| **P2:0x14** | `neon_en`, `posteraize_en`, `awb_green_en`, `awb_grey_en`, `awb_skin_en` | Enables neon/posterize effects and AWB pixel display |
| **P2:0x15–0x16** | `f_limit_b`, `f_limit_r` | F‑light boundary limits for color temperature detection |
| **P2:0x18–0x19** | `wb_fine_gain_step`, `wb_rough_gain_step`, `wb_dif_fine_th`, `wb_dif_rough_th` | AWB gain step sizes and difference thresholds |
| **P2:0x1C–0x1F** | `u_top/bot`, `v_top/bot` | U/V thresholds for skin detection |
| **P2:0x20–0x23** | `ggain_top_nr/bot_nr`, `bgain_top_nr/bot_nr` | R/G/B gain limits in normal range |
| **P2:0x25** | `f_flg`, `color_temp_rd` | Flags for F‑light detection and color temperature result (read‑only) |
| **P2:0x26–0x33** | `rgain2isp`, `bgain2isp`, Y thresholds and AWB window coordinates | AWB gains and window definitions |
| **P2:0x34–0x38** | `gw_uv_radius`, `uv_fix_dat`, `y_offset_th`, `gw_step`, `gw_jdg_th` | Gray‑world AWB parameters |
| **P2:0x3B** | `wt_th` | White pixel threshold for AWB | `0x09` |
| **P2:0x48–0x53** | `green_u_center`, `green_v_center`, `green_uv_radius`, `gamma2_pt[0–8]` | Green reference point and second gamma curve |
| **P2:0x61–0x6F** | White region boundaries for F‑light, D65, TL84 and CWF lighting conditions (`f_rg_bot_low8`, `d65_bg_top_low8`, etc.) plus high‑bit registers | define AWB white region rectangles |
| **P2:0x70–0x7F** | Additional white region boundaries and test modes | define AWB white region rectangles for various light sources |
| **P2:0x80–0x9F** | White region for indoor light and color correction matrices (`c00_eff1_8lsb` … `c22_eff1_8lsb`) | AWB indoor white region and color correction coefficients |
| **P2:0xA0–0xB4** | `c00_eff2_8lsb` … `c22_eff2_8lsb` | Second set of color correction coefficients |
| **P2:0xB8–0xCF** | Mode transition thresholds (`mean_nr_dummy`, `rpc_heq_low`, `exp_heq_dummy`, `sharp_flat_thr0`) | thresholds for switching between normal, dummy and low‑light modes |
| **P2:0xD0–0xDD** | Sharpening and edge thresholds (`sharp_ofst_pos`, `sharp_flat_thr1–4`, `skin_sharp_sel`, `raw_gflt_en_*`, `raw_denoise_en_*`, `sharpen_en_*`) | Fine‑tunes sharpening and denoise functions |
| **P2:0xE6–0xEF** | Sharpening gains for different modes (`gray_sharp_kp/kn`, `u_sharp_k`, `v_sharp_k`, `sharp_fac_pos/neg_*`) | Gains for Y‑increasing/decreasing sharpening |
| **P2:0xF0–0xF8** | Additional sharpen thresholds and AWB gain limits for outdoor mode | — |
| **P2:0xF9–0xFD** | `dc_test_en`, `dc_mist_set`, `page_flg` | Test and page selection bits |

## Revision History

| Version | Date | Description |
|---|---|---|
| **Commercial 1.0** | 2018‑03‑26 | First version for customers |
| **Commercial 1.2** | 2018‑05‑17 | Minor word updates |
| **Commercial 1.3** | 2018‑09‑07 | Minor word updates |
| **Commercial 1.4** | 2018‑10‑25 | Added CRA, ESD data and power consumption details |
| **Commercial 1.5** | 2018‑11‑02 | Added package description (current version)【48†L1-L8】 |

## Summary

The **SP0A39** is a versatile 1/10‑inch VGA CMOS image sensor with integrated image processing, dual output interfaces and extensive register configurability.  Its built‑in functions—including auto black level calibration, auto white balance, auto exposure, gamma and color correction, lens shading compensation, defect pixel correction and various special effects—make it suitable for compact imaging devices.  The detailed register map allows fine‑tuning of exposure, gain, color balance and processing algorithms to achieve optimal image quality across different lighting conditions.
