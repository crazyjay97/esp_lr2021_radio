# Implementation Plan — ESP32 LoRa WildCam Display Gateway

Based on `design.html` spec vs current implementation.

---

## 现状评估

### P0：90% 通过

| 验收项 | 状态 | 备注 |
|--------|------|------|
| 开机进入首页，未连接时显示等待节点 | ✅ | 显示 "Waiting for node..." |
| 收到 IMAGE_START 自动进入接收进度页 | ✅ | handle_image_start → progress_cb → ui_gw_rx_begin |
| 接收完成后自动回首页并显示图片 | ✅ | ui_gw_rx_complete 切回 IMAGE 页 |
| 首页显示触发原因、图片大小、RSSI、速率 | ⚠️ | 图片大小/RSSI/速率有，**触发原因缺失**（IMAGE_START 不带 trigger 字段） |
| 链路详情页显示 RSSI/SNR、速率、丢包率、重传次数 | ⚠️ | RSSI/速率有，**丢包率和重传次数**写死 "--%" 没取真实值 |
| 侧键可完成切页、返回、手动拍照命令 | ✅ | K4/K5切页，K6返回，K3拍照 |
| 接收失败时显示错误，保留上一张成功图片 | ✅ | rx_failed 红色标签，s_has_image 保留旧图 |

### P1：0% 功能性

| 验收项 | 状态 | 备注 |
|--------|------|------|
| 配置页可修改拍照周期、图片尺寸、质量、触发方式 | ❌ | UI 有静态列表骨架，不能编辑/保存/下发 |
| 可开启 Wi-Fi AP 配置入口 | ❌ | 无 Wi-Fi 代码 |
| 控制页可下发播放音频、休眠、刷新状态命令 | ❌ | 无控制页、无命令协议（只有 ImageCmd） |
| 设备状态页显示本机/远端电量、固件版本和最后心跳 | ❌ | 无 STATUS/HELLO 协议，无设备状态页 |
| 触摸屏操作与侧键一致 | ❌ | Gateway UI 没注册触摸回调 |

协议层现有包类型：Ping(1), Voice(2), ImageCmd(3), ImageData(4), ImageNack(5), ImageDone(6), ImageEOT(7), ImageStart(8)。  
P1 所需的 STATUS/CONFIG/CONTROL 命令通道完全空白。

---

## 一、功能层 (Function)

先把数据通路、协议、状态机搞通，UI 最后统一刷。

### F1: IMAGE_START 扩展 trigger reason [P0 补丁]

IMAGE_START 当前只带 session_id + total_frags + crc32。

需要：
- 包格式增加 1 byte trigger_reason (offset 14)
- enum: 0=Timer, 1=PIR, 2=Sound, 3=Manual, 4=Cmd
- Camera Node 发送端填入值
- Gateway radio_ping 解析后缓存到全局变量

---

### F2: 丢包/重传统计接入 UI [P0 补丁]

数据已经存在于 radio_ping 中（nack_count_, image_rx_nack_sent_ 等），只是没传给 UI。

需要：
- ui_gw_rx_complete 增加参数或新接口传入 nack_rounds / retransmitted_count
- 链路页从缓存变量取值显示（替代写死的 "--"）

---

### F3: STATUS/HELLO 协议 + 在线状态机 [P1 基础]

P1 设备状态页、状态栏实时更新的数据源。

需要：
- 新增 kPacketTypeStatus = 9, kPacketTypeHello = 10
- STATUS 包结构：firmware_version(8B), uptime(4B), trigger_mode(1B), 预留
- Camera Node 周期发送 STATUS（如 30s 一次）
- Gateway radio_ping 解析后写入全局 `gw_node_state_t`
- 超时判断（>60s 无 STATUS → OFFLINE）

---

### F4: 通用远程命令通道 [P1]

P1 控制页和配置下发的底层支撑。

需要：
- 新增 kPacketTypeCommand = 11
- Command 子类型：
  - CMD_CAPTURE = 0x01（替代现有 ImageCmd，兼容）
  - CMD_PLAY_AUDIO = 0x02
  - CMD_SLEEP = 0x03
  - CMD_REFRESH_STATUS = 0x04
  - CMD_CONFIG = 0x10（带配置 payload）
- Camera Node 收到后执行对应动作
- 可选：ACK 确认机制

---

### F5: 配置项 NVS 存储 [P1]

需要：
- 配置结构体：interval, resolution, quality, trigger_mode, audio_trigger
- NVS namespace "cfg" 读写
- 修改后通过 F4 的 CMD_CONFIG 下发

---

### F6: Wi-Fi AP 模式 [P1]

需要：
- ESP32 SoftAP 启动/关闭
- 简易 HTTP server（查看最后一张图片、修改配置）
- 状态标记供 UI 显示

---

## 二、UI 层 (UI)

### U1: 首页触发原因 chip [P0 补丁]

标题栏 chip 从固定 "OK" 改为动态 trigger reason 文本。

依赖：F1

---

### U2: 链路页丢包/重传真实值 [P0 补丁]

从 "--%" 改为从统计数据取值。

依赖：F2

---

### U3: 配置页交互 [P1]

- K5/K4 移动选中行高亮（当前不响应）
- K3 进入编辑模式，K5/K4 调值，K3 保存
- 保存触发 NVS 写入 + LoRa 下发

依赖：F4, F5

---

### U4: 控制页 [P1]

- 新增 UI_PAGE_CONTROL
- 按钮矩阵：手动拍照 / 播放音频 / 刷新状态 / 进入休眠
- 选中 + K3 执行 → 调用 F4 命令

依赖：F4

---

### U5: 设备状态页 [P1]

- 新增 UI_PAGE_DEVICE
- 显示：固件版本、运行模式、在线/离线、最后心跳时间
- （电量忽略，TYPE-C 供电）

依赖：F3

---

### U6: 状态栏动态化 [P1]

- 1s timer 刷新
- 左侧：Node ID + 模式（FLRC / RX / OFF）
- 右侧：Online 状态或接收中 RSSI
- 接收中 500ms 刷新

依赖：F3

---

### U7: 触摸屏兼容 [P1]

- Gateway UI 注册 LVGL indev 触摸驱动
- 点击列表行 = K3，左右滑动 = K5/K4 切页

依赖：无（纯 UI 层）

---

## 三、实施顺序

```
=== P0 补丁（2项）===
F1 (trigger reason) → U1
F2 (丢包统计接UI) → U2

=== P1 功能层 ===
F3 (STATUS 协议)
F4 (远程命令通道)
F5 (NVS 配置)
F6 (Wi-Fi AP)

=== P1 UI 层 ===
U3 (配置页交互)
U4 (控制页)
U5 (设备状态页)
U6 (状态栏动态化)
U7 (触摸屏)
```

---

*Updated: 2026-06-24*
