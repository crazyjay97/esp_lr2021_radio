#include "camera_uart.hpp"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "esp_cam_sensor_types.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "linux/videodev2.h"
#include "soc/gpio_sig_map.h"
#include "app_config.h"
#include "bsp.h"

namespace {

constexpr const char *TAG = "camera_video";

constexpr ledc_mode_t kMclkSpeedMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kMclkTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kMclkChannel = LEDC_CHANNEL_0;
constexpr ledc_clk_cfg_t kMclkClockSource = LEDC_USE_APB_CLK;
constexpr TickType_t kPwdnSettleTicks = pdMS_TO_TICKS(1000);
constexpr int kCaptureBuffers = 2;
constexpr int kMaxFrameDequeues = 8;
constexpr int kVideoType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
constexpr int kUartTxTimeoutTicks = pdMS_TO_TICKS(1000);

void force_spi3_cs_active_high_input()
{
    esp_rom_gpio_connect_in_signal(BSP_SP0A39_VSYNC_GPIO, SPI3_CS0_IN_IDX, true);
}

uint32_t sensor_pixformat_to_v4l2(esp_cam_sensor_output_format_t format)
{
    switch (format) {
    case ESP_CAM_SENSOR_PIXFORMAT_GRAYSCALE:
        return V4L2_PIX_FMT_GREY;
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422_UYVY:
        return V4L2_PIX_FMT_UYVY;
    default:
        return 0;
    }
}

esp_err_t checked_ioctl(int fd, unsigned long cmd, void *arg, const char *name)
{
    if (ioctl(fd, cmd, arg) != 0) {
        ESP_LOGE(TAG, "%s failed errno=%d", name, errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t uart_write_all(const uint8_t *data, size_t len)
{
    size_t offset = 0;
    while (offset < len) {
        const size_t chunk = (len - offset) > APP_CAMERA_UART_CHUNK_BYTES ?
            APP_CAMERA_UART_CHUNK_BYTES : (len - offset);
        const int written = uart_write_bytes(UART_NUM_2,
                                             reinterpret_cast<const char *>(data + offset),
                                             chunk);
        if (written < 0) {
            return ESP_FAIL;
        }
        offset += static_cast<size_t>(written);
        if (written == 0) {
            vTaskDelay(1);
        }
    }
    ESP_RETURN_ON_ERROR(uart_wait_tx_done(UART_NUM_2, kUartTxTimeoutTicks),
                        TAG, "uart wait tx done");
    return ESP_OK;
}

esp_err_t uart_write_pgm_image(const uint8_t *data,
                               size_t len,
                               uint32_t pixelformat,
                               uint32_t width,
                               uint32_t height)
{
    char header[96];
    const int header_len = snprintf(header, sizeof(header),
                                    "P5\n%lu %lu\n255\n",
                                    static_cast<unsigned long>(width),
                                    static_cast<unsigned long>(height));
    ESP_RETURN_ON_FALSE(header_len > 0 && header_len < static_cast<int>(sizeof(header)),
                        ESP_FAIL, TAG, "format UART header");
    ESP_RETURN_ON_ERROR(uart_write_all(reinterpret_cast<const uint8_t *>(header),
                                       static_cast<size_t>(header_len)),
                        TAG, "write UART header");

    const size_t pixel_count = static_cast<size_t>(width) * height;
    if (pixelformat == V4L2_PIX_FMT_GREY) {
        ESP_RETURN_ON_FALSE(len >= pixel_count, ESP_ERR_INVALID_SIZE, TAG,
                            "short GREY frame: %u/%u bytes",
                            static_cast<unsigned>(len),
                            static_cast<unsigned>(pixel_count));
        return uart_write_all(data, pixel_count);
    }

    if (pixelformat == V4L2_PIX_FMT_UYVY) {
        const size_t yuv_bytes = pixel_count * 2U;
        ESP_RETURN_ON_FALSE(width <= APP_CAMERA_SENSOR_WIDTH, ESP_ERR_INVALID_SIZE,
                            TAG, "PGM line buffer too small for width %lu",
                            static_cast<unsigned long>(width));
        ESP_RETURN_ON_FALSE(len >= yuv_bytes, ESP_ERR_INVALID_SIZE, TAG,
                            "short UYVY frame: %u/%u bytes",
                            static_cast<unsigned>(len),
                            static_cast<unsigned>(yuv_bytes));

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

    ESP_LOGE(TAG, "unsupported UART frame pixelformat=0x%08lx",
             static_cast<unsigned long>(pixelformat));
    return ESP_ERR_NOT_SUPPORTED;
}

} // namespace

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

esp_err_t CameraUartStreamer::start()
{
    ESP_RETURN_ON_ERROR(init(), TAG, "init");

    const BaseType_t ok = xTaskCreatePinnedToCore(
        task_entry, "cam_power",
        APP_CAMERA_TASK_STACK_BYTES, this,
        APP_CAMERA_TASK_PRIORITY, nullptr,
        APP_CAMERA_TASK_CORE);

    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void CameraUartStreamer::task_entry(void *arg)
{
    static_cast<CameraUartStreamer *>(arg)->task();
}

void CameraUartStreamer::task()
{
    ESP_LOGW(TAG, "SP0A39 one-frame capture via official esp_video SPI device");
    esp_err_t ret = init_sp0a39_video();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_video SP0A39 init failed: %s", esp_err_to_name(ret));
    } else {
        read_sp0a39_id();
        ret = capture_and_send_one_frame();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "capture/send frame failed: %s", esp_err_to_name(ret));
        }
    }

    vTaskDelete(nullptr);
}

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

esp_err_t CameraUartStreamer::init_uart()
{
    if (uart_initialized_) {
        return ESP_OK;
    }

    uart_config_t cfg = {};
    cfg.baud_rate = APP_CAMERA_UART_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_RETURN_ON_ERROR(uart_driver_install(UART_NUM_2, 4096, 0, 0, nullptr, 0),
                        TAG, "uart driver install");
    ESP_RETURN_ON_ERROR(uart_param_config(UART_NUM_2, &cfg), TAG, "uart param");
    ESP_RETURN_ON_ERROR(uart_set_pin(UART_NUM_2,
                                     BSP_UART2_TX_GPIO,
                                     BSP_UART2_RX_GPIO,
                                     UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE),
                        TAG, "uart pins");

    uart_initialized_ = true;
    ESP_LOGI(TAG, "UART2 ready: %u baud TX GPIO%d RX GPIO%d",
             APP_CAMERA_UART_BAUD, BSP_UART2_TX_GPIO, BSP_UART2_RX_GPIO);
    return ESP_OK;
}

esp_err_t CameraUartStreamer::set_pwdn(bool asserted)
{
    return bsp_ioexp_set_pin(BSP_SP0A39_PWDN_IOEXP_PIN, asserted);
}

void CameraUartStreamer::read_sp0a39_id()
{
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) {
        ESP_LOGW(TAG, "read SP0A39 id: I2C bus unavailable");
        return;
    }

    i2c_master_dev_handle_t dev = nullptr;
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = APP_SP0A39_I2C_ADDR;
    dev_cfg.scl_speed_hz = BSP_I2C0_FREQ_HZ;

    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "read SP0A39 id: add device failed: %s", esp_err_to_name(ret));
        return;
    }

    uint8_t page_sel[2] = {0xfd, 0x00};
    ret = i2c_master_transmit(dev, page_sel, sizeof(page_sel), 100);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "read SP0A39 id: select page0 failed: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev);
        return;
    }

    uint8_t id_h = 0;
    uint8_t id_l = 0;
    uint8_t reg = 0x00;
    ret = i2c_master_transmit_receive(dev, &reg, 1, &id_h, 1, 100);
    if (ret == ESP_OK) {
        reg = 0x01;
        ret = i2c_master_transmit_receive(dev, &reg, 1, &id_l, 1, 100);
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SP0A39 id read: reg00=0x%02X reg01=0x%02X pid=0x%02X%02X",
                 id_h, id_l, id_h, id_l);
    } else {
        ESP_LOGW(TAG, "read SP0A39 id failed: %s", esp_err_to_name(ret));
    }

    i2c_master_bus_rm_device(dev);
}

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

    ESP_LOGI(TAG, "esp_video SPI_1BIT init: dev=%s MCLK GPIO%d DATA0 GPIO%d DCLK GPIO%d CS/VSYNC GPIO%d",
             ESP_VIDEO_SPI_DEVICE_NAME,
             BSP_SP0A39_MCLK_GPIO,
             BSP_SP0A39_D0_GPIO,
             BSP_SP0A39_PCLK_GPIO,
             BSP_SP0A39_VSYNC_GPIO);
    ESP_RETURN_ON_ERROR(esp_video_init_with_flags(&config, ESP_VIDEO_INIT_FLAGS_SPI),
                        TAG, "esp_video_init SPI");
    force_spi3_cs_active_high_input();

    video_initialized_ = true;
    ESP_LOGI(TAG, "esp_video SP0A39 SPI_1BIT device initialized");

    return ESP_OK;
}

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
    ESP_LOGI(TAG, "version: %u.%u.%u",
             static_cast<unsigned>((capability.version >> 16) & 0xff),
             static_cast<unsigned>((capability.version >> 8) & 0xff),
             static_cast<unsigned>(capability.version & 0xff));
    ESP_LOGI(TAG, "video driver=%s card=%s bus=%s caps=0x%08lx device_caps=0x%08lx",
             capability.driver, capability.card, capability.bus_info,
             static_cast<unsigned long>(capability.capabilities),
             static_cast<unsigned long>(capability.device_caps));

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
    } else {
        ESP_LOGW(TAG, "VIDIOC_G_EXT_CTRLS chip id failed errno=%d", errno);
    }

    ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_G_SENSOR_FMT, &sensor_format,
                                    "VIDIOC_G_SENSOR_FMT"),
                      cleanup, TAG, "get sensor format");

    format.type = kVideoType;
    format.fmt.pix.width = sensor_format.width;
    format.fmt.pix.height = sensor_format.height;
    format.fmt.pix.pixelformat = sensor_pixformat_to_v4l2(sensor_format.format);
    ESP_GOTO_ON_FALSE(format.fmt.pix.pixelformat != 0, ESP_ERR_NOT_SUPPORTED,
                      cleanup, TAG, "unsupported sensor pixel format=%d",
                      sensor_format.format);

    if (sensor_format.spi_info.frame_info) {
        format.fmt.pix.sizeimage = sensor_format.spi_info.frame_info->frame_size;
        ESP_LOGI(TAG,
                 "sensor format: %s %ux%u frame_size=%lu line_size=%lu cs_active_high=%u",
                 sensor_format.name ? sensor_format.name : "(unnamed)",
                 sensor_format.width,
                 sensor_format.height,
                 static_cast<unsigned long>(sensor_format.spi_info.frame_info->frame_size),
                 static_cast<unsigned long>(sensor_format.spi_info.frame_info->line_size),
                 static_cast<unsigned>(sensor_format.spi_info.frame_info->high_level_active));
    }

    for (uint32_t index = 0; index < 8; ++index) {
        struct v4l2_fmtdesc fmtdesc = {};
        fmtdesc.index = index;
        fmtdesc.type = kVideoType;
        if (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != 0) {
            break;
        }
        ESP_LOGI(TAG, "enum format[%lu]: %s fourcc=0x%08lx",
                 static_cast<unsigned long>(index),
                 reinterpret_cast<char *>(fmtdesc.description),
                 static_cast<unsigned long>(fmtdesc.pixelformat));
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
        if (ioctl(fd, VIDIOC_S_PARM, &sparm) != 0) {
            ESP_LOGW(TAG, "VIDIOC_S_PARM failed errno=%d", errno);
        } else {
            ESP_LOGI(TAG, "stream interval set: %lu/%lu s",
                     static_cast<unsigned long>(frmival.discrete.numerator),
                     static_cast<unsigned long>(frmival.discrete.denominator));
        }
    } else {
        struct v4l2_streamparm sparm = {};
        sparm.type = kVideoType;
        sparm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
        if (ioctl(fd, VIDIOC_G_PARM, &sparm) == 0 &&
            sparm.parm.capture.timeperframe.denominator != 0) {
            ESP_LOGI(TAG, "stream interval current: %lu/%lu s",
                     static_cast<unsigned long>(sparm.parm.capture.timeperframe.numerator),
                     static_cast<unsigned long>(sparm.parm.capture.timeperframe.denominator));
        }
    }

    ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_G_FMT, &format, "VIDIOC_G_FMT"),
                      cleanup, TAG, "get format");
    ESP_LOGI(TAG, "capture format: %lux%lu fourcc=0x%08lx sizeimage=%lu",
             static_cast<unsigned long>(format.fmt.pix.width),
             static_cast<unsigned long>(format.fmt.pix.height),
             static_cast<unsigned long>(format.fmt.pix.pixelformat),
             static_cast<unsigned long>(format.fmt.pix.sizeimage));

    req.count = kCaptureBuffers;
    req.type = kVideoType;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_REQBUFS, &req, "VIDIOC_REQBUFS"),
                      cleanup, TAG, "request buffers");
    ESP_GOTO_ON_FALSE(req.count >= kCaptureBuffers, ESP_ERR_NO_MEM, cleanup,
                      TAG, "not enough video buffers: %lu", static_cast<unsigned long>(req.count));

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
        ESP_GOTO_ON_FALSE(buffers[i] && buffers[i] != MAP_FAILED, ESP_FAIL, cleanup,
                          TAG, "mmap buffer %d failed errno=%d", i, errno);
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

        ESP_GOTO_ON_FALSE(frame.index < kCaptureBuffers, ESP_ERR_INVALID_SIZE, cleanup,
                          TAG, "bad frame index %lu", static_cast<unsigned long>(frame.index));
        ESP_LOGI(TAG, "captured frame attempt=%d index=%lu bytesused=%lu flags=0x%08lx length=%lu",
                 attempt + 1,
                 static_cast<unsigned long>(frame.index),
                 static_cast<unsigned long>(frame.bytesused),
                 static_cast<unsigned long>(frame.flags),
                 static_cast<unsigned long>(frame.length));

        if (frame.bytesused > 0 && (frame.flags & V4L2_BUF_FLAG_DONE)) {
            break;
        }

        ESP_LOGW(TAG, "skip empty/error camera frame attempt=%d flags=0x%08lx",
                 attempt + 1, static_cast<unsigned long>(frame.flags));
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
    ESP_LOGI(TAG, "UART frame output complete: %lu bytes",
             static_cast<unsigned long>(frame.bytesused));

    ESP_GOTO_ON_ERROR(checked_ioctl(fd, VIDIOC_QBUF, &frame, "VIDIOC_QBUF return"),
                      cleanup, TAG, "return buffer");
    frame = {};
    force_spi3_cs_active_high_input();

cleanup:
    if (streaming) {
        int type = kVideoType;
        if (ioctl(fd, VIDIOC_STREAMOFF, &type) != 0) {
            ESP_LOGW(TAG, "VIDIOC_STREAMOFF failed errno=%d", errno);
        }
    }
    for (int i = 0; i < kCaptureBuffers; ++i) {
        if (buffers[i] && buffers[i] != MAP_FAILED) {
            munmap(buffers[i], buffer_lengths[i]);
        }
    }
    close(fd);
    return ret;
}
