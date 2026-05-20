# SP0A39 摄像头当前流程

本文档描述当前工程中 SP0A39 摄像头的一帧采集流程。当前实现使用乐鑫 `esp_video` 的 SPI video device 和 V4L2 风格接口，不再走裸 `spi_slave` 直接读数据。

## 总体链路

当前数据链路是：

```text
app_main
  -> CameraUartStreamer::start()
    -> xTaskCreatePinnedToCore("cam_power")
      -> CameraUartStreamer::task()
        -> init_sp0a39_video()
          -> esp_video_init_with_flags(..., ESP_VIDEO_INIT_FLAGS_SPI)
          -> /dev/video3 注册完成
        -> read_sp0a39_id()
          -> 直接通过 SCCB/I2C 读 SP0A39 PID
        -> capture_and_send_one_frame()
          -> open("/dev/video3")
          -> V4L2 查询/配置/开流/取帧
          -> UART2 输出 PGM 灰度图
```

硬件信号映射：

```text
MCLK       GPIO3
D0/MOSI    GPIO2
DCLK/SCLK  GPIO8
VSYNC/CS   GPIO7, SP0A39 为 active-high
UART2 TX   GPIO47, 2 Mbps
UART2 RX   GPIO48
PWDN       TCA9554 P7, 拉低释放
```

## 初始化阶段

`CameraUartStreamer::init()` 做三件事：

1. 初始化 BSP I2C。
2. 把摄像头数据/同步脚配置成高阻输入。
3. 初始化 UART2，然后通过 IO expander 拉低 SP0A39 PWDN，等待传感器稳定。

核心代码：

```cpp
esp_err_t CameraUartStreamer::init()
{
    if (initialized_) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c init");
    ESP_RETURN_ON_ERROR(configure_camera_pins(), TAG, "camera pins");
    ESP_RETURN_ON_ERROR(init_uart(), TAG, "uart2");

    ESP_LOGI(TAG, "releasing SP0A39 PWDN: IO expander P%d low",
             BSP_SP0A39_PWDN_IOEXP_PIN);
    ESP_RETURN_ON_ERROR(set_pwdn(false), TAG, "pwdn low");
    vTaskDelay(kPwdnSettleTicks);

    initialized_ = true;
    return ESP_OK;
}
```

摄像头引脚当前只做输入，不在这里绑定 SPI 外设：

```cpp
esp_err_t CameraUartStreamer::configure_camera_pins()
{
    uint64_t mask = 0;
    mask |= 1ULL << BSP_SP0A39_PCLK_GPIO;
    mask |= 1ULL << BSP_SP0A39_VSYNC_GPIO;
    mask |= 1ULL << BSP_SP0A39_HSYNC_GPIO;
    mask |= 1ULL << BSP_SP0A39_D0_GPIO;
    mask |= 1ULL << BSP_SP0A39_D1_GPIO;
    mask |= 1ULL << BSP_SP0A39_D2_GPIO;
    mask |= 1ULL << BSP_SP0A39_D3_GPIO;

    gpio_config_t input_conf = {};
    input_conf.pin_bit_mask = mask;
    input_conf.mode = GPIO_MODE_INPUT;
    input_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    input_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    input_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_RETURN_ON_ERROR(gpio_config(&input_conf), TAG, "gpio input config");
    ESP_LOGI(TAG, "camera data/sync pins configured as high-Z inputs");
    return ESP_OK;
}
```

## esp_video SPI 设备初始化

`init_sp0a39_video()` 使用当前 BSP 的 I2C bus，把 SP0A39 配成 `esp_video` SPI video device。设备路径是 `/dev/video3`。

当前配置：

```cpp
esp_video_init_spi_config_t spi = {};
spi.sccb_config.init_sccb = false;
spi.sccb_config.i2c_handle = bus;
spi.sccb_config.freq = BSP_I2C0_FREQ_HZ;
spi.intf = ESP_CAM_CTLR_SPI_CAM_INTF_SPI;
spi.io_mode = ESP_CAM_CTLR_SPI_CAM_IO_MODE_1BIT;
spi.spi_port = SPI3_HOST;
spi.spi_cs_pin = BSP_SP0A39_VSYNC_GPIO;
spi.spi_sclk_pin = BSP_SP0A39_PCLK_GPIO;
spi.spi_data0_io_pin = BSP_SP0A39_D0_GPIO;
spi.spi_data1_io_pin = GPIO_NUM_NC;
spi.spi_data2_io_pin = GPIO_NUM_NC;
spi.spi_data3_io_pin = GPIO_NUM_NC;
spi.reset_pin = GPIO_NUM_NC;
spi.pwdn_pin = GPIO_NUM_NC;
spi.xclk_source = ESP_CAM_SENSOR_XCLK_LEDC;
spi.xclk_freq = APP_SP0A39_MCLK_HZ;
spi.xclk_pin = BSP_SP0A39_MCLK_GPIO;
```

完整初始化入口：

```cpp
esp_err_t CameraUartStreamer::init_sp0a39_video()
{
    if (video_initialized_) {
        return ESP_OK;
    }

    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_video_init_spi_config_t spi = {};
    spi.sccb_config.init_sccb = false;
    spi.sccb_config.i2c_handle = bus;
    spi.sccb_config.freq = BSP_I2C0_FREQ_HZ;
    spi.intf = ESP_CAM_CTLR_SPI_CAM_INTF_SPI;
    spi.io_mode = ESP_CAM_CTLR_SPI_CAM_IO_MODE_1BIT;
    spi.spi_port = SPI3_HOST;
    spi.spi_cs_pin = BSP_SP0A39_VSYNC_GPIO;
    spi.spi_sclk_pin = BSP_SP0A39_PCLK_GPIO;
    spi.spi_data0_io_pin = BSP_SP0A39_D0_GPIO;
    spi.spi_data1_io_pin = GPIO_NUM_NC;
    spi.spi_data2_io_pin = GPIO_NUM_NC;
    spi.spi_data3_io_pin = GPIO_NUM_NC;
    spi.reset_pin = GPIO_NUM_NC;
    spi.pwdn_pin = GPIO_NUM_NC;
    spi.xclk_source = ESP_CAM_SENSOR_XCLK_LEDC;
    spi.xclk_freq = APP_SP0A39_MCLK_HZ;
    spi.xclk_pin = BSP_SP0A39_MCLK_GPIO;
#if CONFIG_CAMERA_XCLK_USE_LEDC
    spi.xclk_ledc_cfg.timer = kMclkTimer;
    spi.xclk_ledc_cfg.clk_cfg = kMclkClockSource;
    spi.xclk_ledc_cfg.channel = kMclkChannel;
#endif

    esp_video_init_config_t config = {};
    config.spi = &spi;

    ESP_RETURN_ON_ERROR(esp_video_init_with_flags(&config, ESP_VIDEO_INIT_FLAGS_SPI),
                        TAG, "esp_video_init SPI");
    force_spi3_cs_active_high_input();

    video_initialized_ = true;
    return ESP_OK;
}
```

## VSYNC active-high 处理

SP0A39 的 `VSYNC/CS` 是 active-high。当前 `esp_video_init_spi_config_t` 没有公开字段可以直接配置 SPI CS active-high，所以代码在 app 层通过 GPIO matrix 把 GPIO7 输入到 SPI3 CS0 时反相：

```cpp
void force_spi3_cs_active_high_input()
{
    esp_rom_gpio_connect_in_signal(BSP_SP0A39_VSYNC_GPIO, SPI3_CS0_IN_IDX, true);
}
```

这个函数当前在这些位置调用：

```text
esp_video_init_with_flags() 之后
VIDIOC_STREAMON 之后
每次 VIDIOC_DQBUF 之前
跳过空帧并重新 QBUF 后
成功 QBUF 返回后
```

这样做的目的：外部 GPIO7 仍然是 VSYNC 高有效；内部 SPI slave 看到的是低有效 CS。

## V4L2 捕获流程

`capture_and_send_one_frame()` 是当前采集核心。它打开 `/dev/video3`，按乐鑫 `capture_stream` 例程的模式配置和取帧。

流程如下：

```text
open /dev/video3
VIDIOC_QUERYCAP
VIDIOC_S_DQBUF_TIMEOUT, 3 秒
VIDIOC_G_EXT_CTRLS, 读取 chip id
VIDIOC_G_SENSOR_FMT, 读取当前 sensor format
VIDIOC_ENUM_FMT, 打印支持格式
VIDIOC_S_FMT, 设置当前采集格式
VIDIOC_ENUM_FRAMEINTERVALS / VIDIOC_S_PARM, 设置帧率参数
VIDIOC_G_FMT, 确认实际 capture format
VIDIOC_REQBUFS, 申请 2 个 MMAP buffer
VIDIOC_QUERYBUF + mmap
VIDIOC_QBUF, 入队所有 buffer
VIDIOC_STREAMON
循环 VIDIOC_DQBUF，最多 8 次
  - 有 DONE 且 bytesused > 0：输出 UART PGM
  - 空帧或错误帧：QBUF 放回继续等
VIDIOC_STREAMOFF
munmap + close
```

核心代码：

```cpp
esp_err_t CameraUartStreamer::capture_and_send_one_frame()
{
    int fd = open(ESP_VIDEO_SPI_DEVICE_NAME, O_RDONLY);
    ESP_RETURN_ON_FALSE(fd >= 0, ESP_FAIL, TAG, "open %s failed errno=%d",
                        ESP_VIDEO_SPI_DEVICE_NAME, errno);

    esp_err_t ret = ESP_OK;
    uint8_t *buffers[kCaptureBuffers] = {};
    size_t buffer_lengths[kCaptureBuffers] = {};
    bool streaming = false;
    int stream_type = kVideoType;
    struct v4l2_capability capability = {};
    struct v4l2_format format = {};
    esp_cam_sensor_format_t sensor_format = {};
    struct v4l2_requestbuffers req = {};
    struct v4l2_buffer frame = {};
    esp_cam_sensor_id_t chip_id = {};
    struct v4l2_ext_control control = {};
    struct v4l2_ext_controls controls = {};
    struct v4l2_frmivalenum frmival = {};
    struct timeval dqbuf_timeout = {};

    ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_QUERYCAP, &capability, "VIDIOC_QUERYCAP"),
                      cleanup, TAG, "querycap");

    dqbuf_timeout.tv_sec = 3;
    dqbuf_timeout.tv_usec = 0;
    if (ioctl(fd, VIDIOC_S_DQBUF_TIMEOUT, &dqbuf_timeout) != 0) {
        ESP_LOGW(TAG, "VIDIOC_S_DQBUF_TIMEOUT failed errno=%d", errno);
    }

    controls.ctrl_class = V4L2_CTRL_CLASS_ESP_CAM_IOCTL;
    controls.count = 1;
    controls.controls = &control;
    control.id = ESP_CAM_SENSOR_IOC_G_CHIP_ID;
    control.p_u8 = reinterpret_cast<uint8_t *>(&chip_id);
    control.size = sizeof(chip_id);
    if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &controls) == 0) {
        ESP_LOGI(TAG, "video chip id: 0x%04" PRIx16, chip_id.pid);
    }

    ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_G_SENSOR_FMT, &sensor_format,
                                    "VIDIOC_G_SENSOR_FMT"),
                      cleanup, TAG, "get sensor format");

    format.type = kVideoType;
    format.fmt.pix.width = sensor_format.width;
    format.fmt.pix.height = sensor_format.height;
    format.fmt.pix.pixelformat = sensor_pixformat_to_v4l2(sensor_format.format);
    if (sensor_format.spi_info.frame_info) {
        format.fmt.pix.sizeimage = sensor_format.spi_info.frame_info->frame_size;
    }

    ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_S_FMT, &format, "VIDIOC_S_FMT"),
                      cleanup, TAG, "set format");

    frmival.index = 0;
    frmival.pixel_format = format.fmt.pix.pixelformat;
    frmival.type = kVideoType;
    frmival.width = format.fmt.pix.width;
    frmival.height = format.fmt.pix.height;
    if (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) == 0) {
        struct v4l2_streamparm sparm = {};
        sparm.type = kVideoType;
        sparm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
        sparm.parm.capture.timeperframe = frmival.discrete;
        ioctl(fd, VIDIOC_S_PARM, &sparm);
    }

    ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_G_FMT, &format, "VIDIOC_G_FMT"),
                      cleanup, TAG, "get format");

    req.count = kCaptureBuffers;
    req.type = kVideoType;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_REQBUFS, &req, "VIDIOC_REQBUFS"),
                      cleanup, TAG, "request buffers");

    for (int i = 0; i < kCaptureBuffers; ++i) {
        struct v4l2_buffer buf = {};
        buf.type = kVideoType;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_QUERYBUF, &buf, "VIDIOC_QUERYBUF"),
                          cleanup, TAG, "query buffer");

        buffers[i] = static_cast<uint8_t *>(mmap(nullptr, buf.length,
                                                 PROT_READ | PROT_WRITE,
                                                 MAP_SHARED, fd, buf.m.offset));
        buffer_lengths[i] = buf.length;

        ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_QBUF, &buf, "VIDIOC_QBUF"),
                          cleanup, TAG, "queue buffer");
    }

    ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_STREAMON, &stream_type, "VIDIOC_STREAMON"),
                      cleanup, TAG, "stream on");
    streaming = true;
    force_spi3_cs_active_high_input();

    for (int attempt = 0; attempt < kMaxFrameDequeues; ++attempt) {
        frame = {};
        frame.type = kVideoType;
        frame.memory = V4L2_MEMORY_MMAP;
        force_spi3_cs_active_high_input();
        ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_DQBUF, &frame, "VIDIOC_DQBUF"),
                          cleanup, TAG, "dequeue frame");

        if (frame.bytesused > 0 && (frame.flags & V4L2_BUF_FLAG_DONE)) {
            break;
        }

        ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_QBUF, &frame, "VIDIOC_QBUF skip"),
                          cleanup, TAG, "return skipped buffer");
        frame = {};
        force_spi3_cs_active_high_input();
        vTaskDelay(1);
    }

    ESP_GOTO_ON_FALSE(frame.bytesused > 0 && (frame.flags & V4L2_BUF_FLAG_DONE),
                      ESP_ERR_INVALID_SIZE, cleanup, TAG,
                      "no valid camera frame after %d dequeues", kMaxFrameDequeues);

    ESP_GOTO_ON_ERROR(uart_write_pgm_image(buffers[frame.index],
                                           frame.bytesused,
                                           format.fmt.pix.pixelformat,
                                           format.fmt.pix.width,
                                           format.fmt.pix.height),
                      cleanup, TAG, "write UART PGM frame");

    ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_QBUF, &frame, "VIDIOC_QBUF return"),
                      cleanup, TAG, "return buffer");

cleanup:
    if (streaming) {
        int type = kVideoType;
        ioctl(fd, VIDIOC_STREAMOFF, &type);
    }
    for (int i = 0; i < kCaptureBuffers; ++i) {
        if (buffers[i] && buffers[i] != MAP_FAILED) {
            munmap(buffers[i], buffer_lengths[i]);
        }
    }
    close(fd);
    return ret;
}
```

## 输出格式

当前输出不是 JPEG，也不是原始 SP0A39 SPI packet，而是 PGM 灰度图：

```text
P5
<width> <height>
255
<raw gray bytes>
```

如果 `esp_video` 返回 `V4L2_PIX_FMT_GREY`，直接输出灰度像素；如果返回 `V4L2_PIX_FMT_UYVY`，只抽取 Y 分量输出灰度。

核心代码：

```cpp
if (pixelformat == V4L2_PIX_FMT_GREY) {
    return uart_write_all(data, pixel_count);
}

if (pixelformat == V4L2_PIX_FMT_UYVY) {
    uint8_t gray_line[APP_CAMERA_SENSOR_WIDTH];
    for (uint32_t y = 0; y < height; ++y) {
        const uint8_t *src = data + static_cast<size_t>(y) * width * 2U;
        for (uint32_t x = 0; x < width; x += 2U) {
            gray_line[x] = src[x * 2U + 1U];
            if (x + 1U < width) {
                gray_line[x + 1U] = src[x * 2U + 3U];
            }
        }
        ESP_RETURN_ON_ERROR(uart_write_all(gray_line, width),
                            TAG, "write UART UYVY luma line");
    }
    return ESP_OK;
}
```

## 当前调试观察点

刷机后重点看这些日志：

```text
esp_video SPI_1BIT init: dev=/dev/video3 ...
video chip id: 0x0a39
sensor format: ... frame_size=... line_size=... cs_active_high=1
enum format[...]
stream interval set/current
capture format: ...
captured frame attempt=N index=M bytesused=X flags=...
UART frame output complete: ...
```

判断规则：

```text
能看到 chip id 0x0a39:
  SCCB/I2C 和传感器上电基本正常。

能看到 cs_active_high=1:
  SP0A39 sensor metadata 确认 VSYNC/CS 是 active-high。

DQBUF 超时或 bytesused=0:
  /dev/video3 已经存在，但 SPI camera backend 没有收到有效帧，重点看 CS 极性/时序和 SPI backend 对 high_level_active 的处理。

bytesused > 0 且 flags 有 V4L2_BUF_FLAG_DONE:
  esp_video 已经解出一帧，随后会通过 UART2 输出 PGM。
```

## 相关文件

```text
main/camera_uart.cpp
main/camera_uart.hpp
main/app_config.h
managed_components/espressif__esp_video/examples/capture_stream/main/capture_stream_main.c
managed_components/espressif__esp_cam_sensor/sensors/sp0a39/sp0a39.c
```
