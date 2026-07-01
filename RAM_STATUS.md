# ESP32-S3 内部 DRAM 使用分析

ESP32-S3 内部 SRAM 总共 ~512 KB，去掉 ICache/DCache/静态数据后可用 DRAM 约 320-370 KB。
外部 PSRAM：Octal 40 MHz，通过 `CONFIG_SPIRAM_USE_MALLOC=y` 启用。

## 当前状态（2026-07-01）

已应用修改：
- **关闭 BLE**（`CONFIG_BT_ENABLED=n`）→ 释放 ~15-20 KB 内部 RAM + ~170 KB flash
- **WiFi/LwIP 动态缓冲走 PSRAM**（`CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`）
- **Flash 改 8 MB**，factory 分区扩展到 ~8 MB

## 各模块内存占用

| 模块 | 内部 DRAM 占用 | 说明 |
|------|---------------|------|
| WiFi 静态 RX 缓冲 | ~10 KB | 代码覆盖为 6×1600B（sdkconfig 默认值 10 未生效） |
| WiFi 动态 RX/TX | ~0（走 PSRAM） | SPIRAM_TRY_ALLOCATE_WIFI_LWIP 已开启 |
| WiFi 管理帧+驱动结构 | ~15 KB | 管理短缓冲 32 个 + 内部结构体 |
| LVGL 绘图缓冲 | **38.4 KB** | 2×(240×40×2)，MALLOC_CAP_DMA，必须内部 |
| LVGL 内部堆 | **32 KB** | LV_MEM_SIZE_KILOBYTES=32 |
| I2S DMA | ~5 KB | 2 通道 × DMA 描述符+缓冲 |
| LCD SPI DMA | ~1 KB | SPI 事务描述符 |
| 任务栈（内部） | ~30 KB | lvgl 8K + main 3.5K + btn 3K + audio 4K + camera 12K |
| 任务栈（可能在 PSRAM） | ~96 KB | radio 32K + voice_tx 32K + voice_play 32K |
| 系统/FreeRTOS | ~25 KB | idle×2, timer, IPC, event loop, tcpip 3K 栈 |
| mbedTLS | ~16 KB | INTERNAL_MEM_ALLOC=y（仅 TLS 连接时） |
| ~~BLE NimBLE~~ | ~~15-20 KB~~ | **已移除** |

**预估当前总占用：~240-270 KB**（改前 ~300-340 KB）

## 后续可优化项（如果还不够）

| 优先级 | 改动 | 可释放 | 风险 |
|--------|------|--------|------|
| P1 | LVGL 绘图行数 40→20（或去掉双缓冲） | ~19 KB | 刷新率略降 |
| P1 | LV_MEM_SIZE 32→16 KB | 16 KB | 复杂 UI 可能不够 |
| P2 | mbedTLS 改 `DEFAULT_MEM_ALLOC`（大块走 PSRAM） | ~16 KB | 低风险 |
| P3 | radio/voice 任务栈放 PSRAM | ~64 KB | 需确认栈变量无 DMA 操作 |
| P3 | camera 任务栈 12K→8K | 4 KB | 需验证栈深度 |
| P4 | SPIRAM_MALLOC_RESERVE_INTERNAL 32K→16K | 间接释放 | 更多 malloc 走 PSRAM |
| P4 | LVGL 绘图缓冲用 PSRAM（ESP32-S3 GDMA 支持） | 38 KB | 需验证 SPI LCD DMA 能否从 PSRAM 读 |

## 关键 sdkconfig 值

```
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_40M=y
CONFIG_SPIRAM_USE_MALLOC=y
CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384
CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=32768
CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y
CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y
CONFIG_BT_ENABLED=n
CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM=10（代码覆盖为 6）
CONFIG_LV_MEM_SIZE_KILOBYTES=32
CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC=y
CONFIG_ESPTOOLPY_FLASHSIZE="8MB"
```

## 备注

- WiFi 静态 RX 缓冲必须在内部 RAM（硬件 DMA 要求）
- LVGL DMA 绘图缓冲必须内部（除非验证 GDMA+PSRAM 可行）
- `esp_wifi_init()` 延迟调用：无保存凭证时开机不初始化 WiFi 硬件
- 任务栈已开启 `SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y`，可用 PSRAM（需显式标志）
