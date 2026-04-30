#pragma once

/*
 * Application-level voice link configuration.
 *
 * Keep user-tunable audio, codec, radio, and PTT parameters here so the voice
 * pipeline can be adjusted without hunting through task logic.
 */

/* ----- Audio capture/playback ------------------------------------------------ */

/* ES8311/I2S sample rate used by both the microphone and speaker paths. */
#define APP_AUDIO_SAMPLE_RATE_HZ        16000U

/* The voice path is mono; stereo I2S samples are mixed down before encoding. */
#define APP_AUDIO_CHANNELS              1U

/* PCM sample format passed to the Opus encoder and produced by the decoder. */
#define APP_AUDIO_BITS_PER_SAMPLE       16U

/* 10 ms reduces per-frame encode blocking and makes packet loss less audible. */
#define APP_AUDIO_FRAME_MS              10U

/* Number of mono PCM samples in one Opus frame at APP_AUDIO_SAMPLE_RATE_HZ. */
#define APP_AUDIO_FRAME_SAMPLES         ((APP_AUDIO_SAMPLE_RATE_HZ * APP_AUDIO_FRAME_MS) / 1000U)

/* Number of bytes in one mono 16-bit PCM frame before Opus compression. */
#define APP_AUDIO_FRAME_BYTES           (APP_AUDIO_FRAME_SAMPLES * (APP_AUDIO_BITS_PER_SAMPLE / 8U))

/* Size used for blocking I2S reads/writes in the diagnostic local audio path. */
#define APP_AUDIO_IO_CHUNK_BYTES        2048U

/* ----- Opus voice codec ------------------------------------------------------ */

/* Low-delay mode avoids the heavier SILK VOIP path that was hitting WDT. */
#define APP_OPUS_APPLICATION            OPUS_APPLICATION_RESTRICTED_LOWDELAY

/* 16 kHz speech has enough FLRC budget; 24 kbps improves low-delay quality. */
#define APP_OPUS_BITRATE_BPS            24000

/* Fixed bitrate makes packet sizing and radio scheduling easier to debug. */
#define APP_OPUS_USE_VBR                0

/* Keep encoder CPU bounded on ESP32-S3; raise only after WDT/headroom tests. */
#define APP_OPUS_COMPLEXITY             0

/* Keep DTX off for the first bring-up so packet timing remains predictable. */
#define APP_OPUS_USE_DTX                0

/* Reserve enough room for a 20 ms voice packet at the target bitrate plus slack. */
#define APP_OPUS_MAX_PACKET_BYTES       96U

/* ----- FLRC radio link ------------------------------------------------------- */

/* Center frequency. Confirm the exact channel is legal for the deployment area. */
#define APP_FLRC_FREQUENCY_HZ           868000000UL

/* LR2021 FLRC high-rate mode requested for this project. */
#define APP_FLRC_BITRATE_BPS            2600000UL

/* Double-sided FLRC bandwidth paired with 2.6 Mbps in the LR2021 driver. */
#define APP_FLRC_BANDWIDTH_HZ           2666000UL

/* Initial TX power for 868 MHz bench tests; adjust for range, heat, and regulations. */
#define APP_FLRC_TX_POWER_DBM           20

/* Use FEC during early tests; switch to RAL_FLRC_CR_1_1 only after range tests. */
#define APP_FLRC_CODING_RATE            RAL_FLRC_CR_1_2

/* BT=0.5 keeps spectrum cleaner than no shaping at high FLRC data rates. */
#define APP_FLRC_PULSE_SHAPE            RAL_FLRC_PULSE_SHAPE_BT_05

/* Keep payloads small so each packet carries one low-latency voice frame. */
#define APP_FLRC_MAX_PAYLOAD_BYTES      255U

/* Pack several 10 ms Opus frames per FLRC packet to reduce TX overhead while
 * keeping each radio packet to about 50 ms of audio. */
#define APP_FLRC_OPUS_FRAMES_PER_PACKET 5U

/* RX timeout used by the packet receiver before it re-arms listening. */
#define APP_FLRC_RX_TIMEOUT_MS          100U

/* Extra gap after each voice TX packet. Keep at 0 for continuous 20 ms audio. */
#define APP_FLRC_VOICE_TX_GAP_MS        0U

/* Log one voice frame every N packets to avoid flooding the serial console. */
#define APP_VOICE_LOG_EVERY_N           25U

/* Poll period for the RAC engine task. Keep well below one audio frame so TX
 * done/RX done is handled without stretching the 20 ms voice cadence. */
#define APP_RADIO_TASK_POLL_MS          2U

/* FreeRTOS priority for the radio engine task. */
#define APP_RADIO_TASK_PRIORITY         4

/* Stack for RAC callbacks plus Opus encode/decode. Opus needs much more than
 * the old ping-only path, so keep this conservative during bring-up. */
#define APP_RADIO_TASK_STACK_BYTES      32768U

/* Keep direct RAL radio control on CPU0. */
#define APP_RADIO_TASK_CORE             0

/* Dedicated sync word for this project's FLRC test/audio packets. */
#define APP_FLRC_SYNC_WORD_0            0x4CU
#define APP_FLRC_SYNC_WORD_1            0x52U
#define APP_FLRC_SYNC_WORD_2            0x32U
#define APP_FLRC_SYNC_WORD_3            0x31U

/* ----- PTT behavior ---------------------------------------------------------- */

/* Button used as push-to-talk. K5 is the ADC-ladder key near 1.65 V. */
#define APP_PTT_BUTTON                  BSP_BTN_PTT

/* Receiver-side jitter buffer target before starting speaker playback. */
#define APP_RX_JITTER_BUFFER_MS         60U

/* Number of encoded voice frames to queue before starting speaker playback. */
#define APP_RX_JITTER_FRAMES            ((APP_RX_JITTER_BUFFER_MS + APP_AUDIO_FRAME_MS - 1U) / APP_AUDIO_FRAME_MS)

/* Conceal one missing aggregated FLRC packet before resyncing. */
#define APP_RX_MAX_PLC_FRAMES           APP_FLRC_OPUS_FRAMES_PER_PACKET

/* Stop playback if no voice packet arrives within this interval. */
#define APP_RX_AUDIO_TIMEOUT_MS         200U

/* Number of encoded voice packets buffered between radio RX and playback. */
#define APP_VOICE_RX_QUEUE_LEN          12U

/* Number of encoded voice frames buffered between microphone and radio TX. */
#define APP_VOICE_TX_QUEUE_LEN          25U

/* Only print one TX queue overflow warning every N dropped voice frames. */
#define APP_TX_DROP_LOG_EVERY_N         25U

/* Opus decode plus I2S write run here so radio RX can re-arm quickly. */
#define APP_VOICE_PLAY_TASK_PRIORITY    5
#define APP_VOICE_PLAY_TASK_STACK_BYTES 32768U

/* Run Opus decode/playback away from direct radio control. */
#define APP_VOICE_PLAY_TASK_CORE        1

/* Opus encode plus I2S read run here so RAC polling is not blocked by audio. */
#define APP_VOICE_TX_TASK_PRIORITY      5
#define APP_VOICE_TX_TASK_STACK_BYTES   32768U

/* Keep Opus encode away from the radio/control task on CPU0. */
#define APP_VOICE_TX_TASK_CORE          1

/* ----- Local diagnostic tones ------------------------------------------------ */

#define APP_BEEP_FREQ_HZ                1800
#define APP_BEEP_ON_MS                  140
#define APP_BEEP_GAP_MS                 90
#define APP_BEEP_TAIL_MS                80
#define APP_BEEP_AMP                    12000
#define APP_PA_SETTLE_MS                40
/* Set to 0 to make boot silent. */
#define APP_STARTUP_CHIME_ENABLE        1

/* Short low-amplitude boot chime; avoids the old 5 second square-wave tone. */
#define APP_STARTUP_CHIME_FREQ1_HZ      660
#define APP_STARTUP_CHIME_FREQ2_HZ      880
#define APP_STARTUP_CHIME_TONE_MS       90
#define APP_STARTUP_CHIME_GAP_MS        35
#define APP_STARTUP_CHIME_AMP           3500

/* ----- Camera UART bring-up ------------------------------------------------- */

/* Enable the camera-adapter bring-up task. It powers the adapter, probes the
 * shared I2C bus for a sensor, and streams framed image data on UART2. */
#define APP_CAMERA_UART_ENABLE          1

/* Camera-only bring-up mode. This skips audio, LR2021 radio, buttons, LEDs,
 * LCD detection/demo, and startup chime so camera sampling runs alone. */
#define APP_CAMERA_ONLY_BRINGUP         1

/* Force CON6 into GC032A camera mode for camera bring-up and logic analyzer
 * capture, even if the automatic I2C probe does not see the sensor. */
#define APP_CON6_FORCE_CAMERA           1

/* Packed GC032A SPI byte stream on UART2. Host-side hex view should show
 * GC032A default sync bytes such as FF FF FF 01/02/40/80 directly. */
#define APP_CAMERA_UART_BAUD            2000000

/* 1: GC032A 单线 SDR；0: 双线 2-bit，只采 D0/D1。 */
#define APP_GC032A_SPI_1SDR_ENABLE      1
/* 1SDR 当前逻辑分析仪显示数据在 SD1/SDO1 上，因此 ESP 侧采 D1。 */
#define APP_GC032A_SPI_1SDR_USE_SD1     1

/* 摄像头调试路径：在 PSRAM 保存一帧 packet 预算大小。
 * DMA 消费时先在线查找 FF FF FF 01，找到帧头后才开始存 packet。 */
#define APP_CAMERA_FRAME_PACKET_BYTES \
    (9U + (APP_CAMERA_SENSOR_HEIGHT * \
           (12U + APP_CAMERA_SENSOR_WIDTH * 2U + 1U)) + 4U)
#define APP_CAMERA_PACKED_CAPTURE_BYTES APP_CAMERA_FRAME_PACKET_BYTES

/* GC032A MCLK。当前保留 20 MHz，便于和逻辑分析仪时序对齐。 */
#define APP_GC032A_MCLK_HZ              8000000U
#define APP_GC032A_I2C_ADDR             0x21U

/* LCD_CAM 采样 PCLK 边沿。逻辑分析仪确认后只保留这个开关。 */
#define APP_CAMERA_DVP_PCLK_INVERT      0

/* GC032A packet 模式输出 YVYU/YUV422。 */
#define APP_CAMERA_SENSOR_WIDTH         320U
#define APP_CAMERA_SENSOR_HEIGHT        240U
#define APP_CAMERA_UART_CHUNK_BYTES     512U
#define APP_CAMERA_TASK_PRIORITY        3
#define APP_CAMERA_TASK_STACK_BYTES     12288U
#define APP_CAMERA_TASK_CORE            1

/* ----- CON6 ST7789T3 LCD ---------------------------------------------------- */

#define APP_LCD_H_RES                   240U
#define APP_LCD_V_RES                   280U
#define APP_LCD_X_GAP                   0
#define APP_LCD_Y_GAP                   20
#define APP_LCD_SPI_PCLK_HZ             (20U * 1000U * 1000U)
#define APP_LCD_SPI_QUEUE_DEPTH         10U
#define APP_LCD_TEST_PATTERN_ROWS       20U
#define APP_LCD_LVGL_BUFFER_ROWS        40U
#define APP_LCD_LVGL_TICK_MS            2U
#define APP_LCD_LVGL_TASK_DELAY_MS      10U
#define APP_LCD_LVGL_TASK_STACK_BYTES   8192U
#define APP_LCD_LVGL_TASK_PRIORITY      2
#define APP_LCD_LVGL_TASK_CORE          1
