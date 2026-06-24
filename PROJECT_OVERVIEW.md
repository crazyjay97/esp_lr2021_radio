# ESP LR2021 Radio — 项目功能概述

## 项目简介

基于 **Lierda L-LRMAM36-FANN4-DK01** 开发板（ESP32-S3 + Semtech LR2021 射频一体模组）的 **LoRa/FLRC 对讲机 + 远程相机系统**。

系统支持两种工作模式（NVS 持久化，K5 按键切换后重启生效）：

| 模式 | 角色 | 功能 |
|------|------|------|
| Radio（网关） | Gateway | 全功能 LVGL 界面，触发远程拍照、接收图像、PTT 语音 |
| Camera（节点） | Node | 摄像头采集，响应拍照请求，JPEG 编码 & 图传，PTT 语音 |

---

## 主要功能

1. **半双工 PTT 语音通信** — Opus 压缩，24 kbps CBR，16 kHz 单声道，10 ms 帧
2. **远程图像采集与传输** — 网关触发节点拍照，节点编码 JPEG 后通过 FLRC 无线链路回传
3. **图像接收与显示** — 网关端重组 JPEG、解码为 RGB565、LCD 实时预览
4. **本地录放音环回测试** — 长按 K5 录制最长 3 秒，松开回放
5. **音量控制** — K4/K3 调节
6. **音频 DSP 链路** — DC 阻隔 → 频谱降噪 → 噪声门 → 语音带通(300-3400 Hz) → AGC → 预/去加重 → 软限幅
7. **开机提示音** — 660 Hz / 880 Hz 双音三角波

---

## 射频通信参数

| 参数 | 值 |
|------|-----|
| 频率 | 903 MHz |
| 调制方式 | FLRC（Fast Long Range Communication） |
| 比特率 | 2.6 Mbps |
| 带宽 | 2.666 MHz |
| 编码率 | 3/4 FEC |
| 脉冲整形 | BT=0.5 高斯 |
| 发射功率 | 10 dBm（可调，最大 22 dBm） |
| 最大载荷 | 255 字节 |
| 前导码 | 32 bits |
| 同步字 | `0x4C 0x52 0x32 0x31`（"LR21"） |
| 硬件 CRC | 2 字节 |
| RX 超时 | 100 ms（常规），图传接收时持续监听 |

---

## 数据包格式

### 通用包头（14 字节）

```
[0..3]   Magic: 'L','R','P','1'
[4]      包类型
[5]      版本号
[6..7]   序列号 / 会话 ID (LE)
[8..11]  时间戳 / 负载专用字段 (LE)
[12..13] 帧计数 / 负载专用字段
```

### 包类型定义

| 值 | 类型 | 方向 | 说明 |
|----|------|------|------|
| 1 | Ping | — | 链路探测 |
| 2 | Voice | 双向 | 语音数据（5 帧聚合） |
| 3 | ImageCmd | 网关→节点 | 触发拍照 |
| 4 | ImageData | 节点→网关 | 图像分片数据 |
| 5 | ImageNack | 网关→节点 | ACK / 缺失列表 |
| 6 | ImageDone | 网关→节点 | 传输完成确认（旧版） |
| 7 | ImageEOT | 节点→网关 | 传输结束标志 |
| 8 | ImageStart | 节点→网关 | 传输开始通告 |

---

## 图像传输机制

### 编码流程（TX 端 — 节点）

1. SP0A39 摄像头采集 **640×480 YUV422**（DVP 接口）
2. UYVY 像素重排为 YCbYCr 格式
3. 软件 JPEG 编码（`esp_new_jpeg` 组件），**质量 = 50**，**4:2:0 子采样**
4. 最大 JPEG 大小：**60 KB**

### 分片策略

- 每片有效数据：**239 字节**（255 - 16 字节头部）
- 每片结构：14 字节通用头 + 负载数据 + 2 字节 CRC16-CCITT
- 典型 25 KB JPEG ≈ 105 片

### 传输协议（Stop-and-Wait + NACK 重传）

```
┌─────────┐                          ┌─────────┐
│  Node   │                          │ Gateway │
│  (TX)   │                          │  (RX)   │
└────┬────┘                          └────┬────┘
     │                                    │
     │  ① ImageStart                      │
     │  (session_id, total_frags, CRC32)  │
     │ ─────────────────────────────────► │
     │                                    │
     │  ② ImageNack (missing=0 → Ready)   │
     │ ◄───────────────────────────────── │
     │                                    │
     │  ③ Blast ALL ImageData fragments   │
     │  (连续发送，无间隔)                  │
     │ ─────────────────────────────────► │
     │                                    │
     │  ④ ImageEOT                        │
     │ ─────────────────────────────────► │
     │                                    │
     │  ⑤ ImageNack (missing_list[])      │
     │ ◄───────────────────────────────── │
     │                                    │
     │  ⑥ 仅重传缺失分片                   │
     │ ─────────────────────────────────► │
     │                                    │
     │  ⑦ 重复 ④⑤⑥ 直到 missing=0         │
     │     或达到最大重传轮次(30)           │
     │                                    │
```

### 可靠性保障

- 每片 CRC16-CCITT 校验
- 完整 JPEG 端到端 CRC32 校验
- NACK 最多携带 120 个缺失分片索引
- EOT 超时重试：5 次，间隔 200 ms
- 最大 NACK 重传轮次：30 轮

### 解码流程（RX 端 — 网关）

1. 重组分片为连续 JPEG buffer
2. JPEG 解码为 RGB565，缩放至 **240×176**（适配 LCD 预览区域）
3. R↔B 通道交换（适配 BGR 面板）后显示

---

## 语音通信流程

### TX 链路

```
麦克风 → I2S 立体声读取 → 单声道混合 → DSP处理 → Opus编码 → TX队列 → 5帧聚合 → FLRC发射
```

### RX 链路

```
FLRC接收 → 解析聚合包 → 语音帧队列 → 抖动缓冲(60ms) → PLC丢包补偿 → Opus解码 → DSP处理 → 立体声上混 → I2S输出 → PA → 扬声器
```

### 语音参数

| 参数 | 值 |
|------|-----|
| 采样率 | 16 kHz |
| 声道 | 单声道 |
| 帧长 | 10 ms（160 采样） |
| 编码器 | Opus CBR 24 kbps, complexity=0, restricted low-delay |
| 每包帧数 | 5 帧（50 ms 音频/包） |
| 抖动缓冲 | 60 ms 目标延迟 |
| 静默超时 | 200 ms |

---

## 硬件接口

| 外设 | 芯片/类型 | 接口 | 说明 |
|------|-----------|------|------|
| 射频 | Semtech LR2021 | SPI2 | NSS=39, SCLK=40, MOSI=41, MISO=42, BUSY=17, NRST=38, IRQ=GPIO21 |
| 音频编解码 | ES8311 | I2C(0x18) + I2S0 | 16-bit Philips, MCLK=256×Fs, 从机模式 |
| 功放 | CST8302A | GPIO 扩展器 P6 | 高电平使能 |
| LCD | ST7789V3 240×320 | SPI3 | SCLK=44, MOSI=43, DC=48, CS=47, RST=P3, BL=P5 |
| 触摸 | CST816/FT6206 | I2C(0x15/0x2A/0x38) | 电容式, INT=GPIO12, RST=P4 |
| 摄像头 | SP0A39 | DVP(LCD_CAM) + I2C(0x21) | 640×480 YUV422, 24 MHz MCLK, PWDN=P7 |
| GPIO 扩展器 | TCA9554A | I2C(0x39) | P0-P2=RGB LED, P3-P7 见上 |
| 按键 | ADC 电阻阶梯 | ADC1_CH4(GPIO5) | K3-K6; K2=GPIO0(BOOT) |
| PSRAM | 8 MB Octal | 内部 | 40 MHz |

**CPU 主频：** 160 MHz（240 MHz 关闭以避免 DVP 采集异常）

---

## LVGL 界面（网关模式，4 页）

| 页面 | 内容 | 说明 |
|------|------|------|
| **IMAGE** | 216×162 画布 + 元数据面板（时间/大小/链路质量） | 默认首页，显示最后接收的图像 |
| **RX** | 百分比、进度条、已收/总包数、速率、耗时 | 图传接收时自动跳转 |
| **LINK** | FLRC 模式、RSSI、SNR、速率、丢包率、重传率 | 链路统计 |
| **CONFIG** | JPEG 质量、分辨率、触发模式、间隔、音频设置 | 配置页 |

### 按键映射

**网关模式：**
- K4 = 下一页 | K5 = 上一页 | K3 = 拍照/确认 | K6 短按 = 回首页，长按(1.5s) = 切换至相机模式

**相机模式：**
- K5 = 切换至网关模式（重启） | K6 长按 = PTT 发射 | K4/K3 = 音量±

---

## 源文件结构

```
main/
├── app_main.cpp          # 入口，模式分发，按键处理
├── app_config.h          # 所有可调参数（射频/音频/图像/DSP）
├── radio_ping.cpp/.hpp   # 射频控制，语音收发，图传协议
├── image_transfer.cpp/.hpp # JPEG 编解码，分片重组
├── opus_codec.cpp/.hpp   # Opus 语音编解码器
├── audio_processor.cpp/.hpp # DSP 处理链
├── audio_diagnostics.cpp/.hpp # 录放音测试，开机提示音
├── camera_dvp.cpp        # SP0A39 DVP 摄像头采集
├── camera_uart.hpp       # 串口摄像头（备用）
├── ui_gateway.c/.h       # 网关 LVGL 界面
├── bsp_lcd.c             # ST7789V3 LCD + LVGL + 触摸
├── bsp_audio.c           # ES8311 + I2S + PA
├── bsp_button.c          # ADC 阶梯按键
├── bsp_i2c.c             # I2C 总线 + TCA9554A
├── bsp_con6.c            # 相机/LCD 连接器自动检测
└── board_config.h        # 完整 GPIO 引脚定义
```

---

## 依赖组件

| 组件 | 版本 | 用途 |
|------|------|------|
| `78/esp-opus` | 1.0.5 | Opus 编解码 |
| `lierda-iot/esp_lora_driver` | 0.0.5 | LR2021 射频驱动 |
| `lvgl/lvgl` | 8.4.0 | GUI 框架 |
| `espressif/esp_new_jpeg` | 0.6.1 | JPEG 软件编解码 |
| `espressif/esp_cam_sensor` | 1.7.0 | 摄像头传感器驱动 |
