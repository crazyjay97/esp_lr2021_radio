# PTT 对讲系统遗留代码清理

## 背景

项目中残留了一套完整的 PTT（Push-to-Talk）半双工语音对讲系统，节点端长按 K6 发送语音、松开切接收。该功能已无使用场景，需要清除。

## 触发方式（当前）

- Camera 模式：长按 K6（ADC ~0.82V）→ `ptt_long_press_cb()` → `g_radio.handle_button(PTT, true)` 开始录音发射
- 松开 K6 → `g_radio.handle_button(PTT, false)` 停止

## 需要删除的代码

### radio_ping.cpp

| 行号 | 函数/逻辑 | 说明 |
|------|-----------|------|
| 112-113 | `voice_queue_` 创建 | init() 中 |
| 137-138 | `tx_queue_` 创建 | init() 中 |
| 180-182 | `voice_queue_` 创建 | init_gateway() 中 |
| 228, 245, 253 | `voice_play` / `voice_tx` task 创建 | xTaskCreatePinnedToCore |
| 268-308 | `handle_button()` | PTT 按键处理整个函数 |
| 310-324 | `suspend()` / `resume()` 中清空 voice_queue 部分 | 保留 suspend/resume 框架 |
| 355-362 | `tx_task_trampoline` / `play_task_trampoline` | 跳板函数 |
| 439-468 | `play_task()` | 接收语音包→解码→播放，整个函数删 |
| 580-617 | `schedule_tx()` 中语音发射逻辑 | `build_voice_packet` 相关 |
| 641-681 | `build_voice_packet()` | 组装语音 radio 包 |
| 684-728 | `capture_voice_packet()` | 编码入 tx_queue |
| 759-760 | `handle_rx_packet()` 中 `kPacketTypeVoice` 分支 | 收到语音包分发 |
| 794-837 | `queue_voice_packet()` | 解析语音帧入队 |
| 856-898 | `wait_for_jitter_buffer()` / `conceal_missing_frames()` | jitter + PLC |
| 931-950 | `play_mono_frame()` | 如果只被对讲调用则删，需确认 |

### radio_ping.hpp

| 行号 | 内容 |
|------|------|
| 31 | `void handle_button(bsp_btn_id_t id, bool pressed)` |
| 71-72 | `tx_task_trampoline` / `play_task_trampoline` 声明 |
| 77-78 | `tx_task()` / `play_task()` 声明 |
| 84-85 | `build_voice_packet` / `capture_voice_packet` 声明 |
| 87-94 | `queue_voice_packet` / jitter / PLC / playback 声明 |
| 109-120 | `VoicePacket` / `TxFrame` 结构体 |
| 133 | `AudioProcessor audio_proc_`（仅对讲用） |
| 137-138 | `voice_queue_` / `tx_queue_` |
| 141-144 | `ptt_active_` / `tx_burst_active_` / `tx_flush_pending_` |
| 149-150 | `tx_pcm_` / `rx_pcm_` |
| 152-165 | `tx_seq_` / playback 状态变量群 |

### app_config.h

| 行号 | 内容 |
|------|------|
| 90 | `APP_FLRC_VOICE_TX_GAP_MS` |
| 93 | `APP_VOICE_LOG_EVERY_N` |
| 115-153 | 整个 PTT behavior 区块 |
| 167 | `APP_AUDIO_DSP_TX_MUTE_FRAMES` |

### app_main.cpp

| 行号 | 内容 |
|------|------|
| 287-298 | `ptt_long_press_cb()` 中调用 `g_radio.handle_button` 的部分 |
| 296 | `g_radio.handle_button(APP_PTT_BUTTON, true)` |
| 909-929 | `on_button()` 中 Camera 模式 K6 的对讲逻辑 |
| 920-925 | 松开时调 `g_radio.handle_button(id, false)` |

### audio_processor.hpp / audio_processor.cpp

整个文件仅被 `RadioPing::audio_proc_` 使用（对讲 DSP），可整体删除。

### ui_gateway.c

| 行号 | 内容 |
|------|------|
| 895, 983, 1019 | `BSP_BTN_PTT` 相关的网关端 UI 对讲逻辑 |

## 必须保留的代码（与 PTT 混在一起）

`tx_task()` (radio_ping.cpp:377-437) 中包含以下非对讲逻辑，需要剥离保留：

1. **`read_mono_frame()` → `audio_ringbuf_.write()`** — MIC 采集写入回溯环形缓冲区（pre-capture 用）
2. **Sound trigger 检测** (line 392-412) — RMS 阈值判断 → 触发拍照
3. **PIR trigger 检测** (line 414-426) — 红外触发拍照
4. **Opus pre-encoding** (line 428-435) — 预编码写入 `opus_ringbuf_`

这些应重构为独立的 `mic_capture_task()` 或类似命名，脱离 PTT 框架。

## 可能保留的公共组件

| 组件 | 是否保留 | 原因 |
|------|----------|------|
| `opus_codec.hpp/.cpp` | 保留 | `play_audio_clip()` 和 opus_preenc 都在用 |
| `audio_ringbuf.hpp/.cpp` | 保留 | pre-capture 回溯录音用 |
| `opus_ringbuf.hpp/.cpp` | 保留 | opus 预编码缓冲用 |
| `read_mono_frame()` | 保留 | MIC 采集基础函数 |
| `bsp_audio_read/write` | 保留 | 底层 I2S 读写 |

## 清理顺序建议

1. 从 `tx_task()` 中剥离 sound/PIR/ringbuf/preenc 逻辑到新任务
2. 删除 `play_task()` 及相关声明
3. 删除 `handle_button()` / `build_voice_packet()` / `capture_voice_packet()` / `queue_voice_packet()` / jitter / PLC
4. 删除 `VoicePacket` / `TxFrame` 结构体和相关成员变量
5. 删除 `audio_processor.*`
6. 清理 `app_config.h` 中 PTT 参数区块
7. 清理 `app_main.cpp` 和 `ui_gateway.c` 中 PTT 按键逻辑
8. 编译验证
