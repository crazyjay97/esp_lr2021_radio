#include "bsp.h"

#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lvgl.h"

#include "app_config.h"

static const char *TAG = "bsp_lcd";

/* LR2021 uses SPI2 inside the module; keep the external LCD on SPI3. */
#define BSP_LCD_SPI_HOST SPI3_HOST

static esp_lcd_panel_io_handle_t s_lcd_io;
static esp_lcd_panel_handle_t s_lcd_panel;
static bool s_lcd_bus_ready;
static bool s_lvgl_started;
static i2c_master_dev_handle_t s_touch;
static uint8_t s_touch_addr;
static lv_disp_drv_t *s_lvgl_disp_drv;

typedef struct {
    uint8_t cmd;
    const uint8_t *data;
    uint8_t len;
    uint16_t delay_ms;
} lcd_init_cmd_t;

static esp_err_t lcd_tx(uint8_t cmd, const uint8_t *data, size_t len)
{
    return esp_lcd_panel_io_tx_param(s_lcd_io, cmd, data, len);
}

static esp_err_t touch_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    if (!s_touch) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(s_touch, &reg, 1, data, len, 20);
}

static esp_err_t lcd_send_vendor_init(void)
{
    static const uint8_t madctl[] = {0x08};
    static const uint8_t colmod[] = {0x05};
    static const uint8_t porch[] = {0x0c, 0x0c, 0x00, 0x33, 0x33};
    static const uint8_t gate[] = {0x00};
    static const uint8_t vcom[] = {0x36};
    static const uint8_t vdv_vrh_en[] = {0x01};
    static const uint8_t vrh[] = {0x13};
    static const uint8_t vdv[] = {0x20};
    static const uint8_t frame_rate[] = {0x0f};
    static const uint8_t gate_ctrl[] = {0xa1};
    static const uint8_t power[] = {0xa4, 0xa1};
    static const uint8_t gamma_pos[] = {
        0xf0, 0x08, 0x0e, 0x09, 0x08, 0x04, 0x2f,
        0x33, 0x45, 0x36, 0x13, 0x12, 0x2a, 0x2d,
    };
    static const uint8_t gamma_neg[] = {
        0xf0, 0x0e, 0x12, 0x0c, 0x0a, 0x15, 0x2e,
        0x32, 0x44, 0x39, 0x17, 0x18, 0x2b, 0x2f,
    };

    static const lcd_init_cmd_t cmds[] = {
        {LCD_CMD_MADCTL, madctl, sizeof(madctl), 0},
        {LCD_CMD_COLMOD, colmod, sizeof(colmod), 0},
        {0xb2, porch, sizeof(porch), 0},
        {0xb7, gate, sizeof(gate), 0},
        {0xbb, vcom, sizeof(vcom), 0},
        {0xc2, vdv_vrh_en, sizeof(vdv_vrh_en), 0},
        {0xc3, vrh, sizeof(vrh), 0},
        {0xc4, vdv, sizeof(vdv), 0},
        {0xc6, frame_rate, sizeof(frame_rate), 0},
        {0xd6, gate_ctrl, sizeof(gate_ctrl), 0},
        {0xd0, power, sizeof(power), 0},
        {0xe0, gamma_pos, sizeof(gamma_pos), 0},
        {0xe1, gamma_neg, sizeof(gamma_neg), 0},
        {LCD_CMD_COLMOD, colmod, sizeof(colmod), 0},
    };

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); ++i) {
        ESP_RETURN_ON_ERROR(lcd_tx(cmds[i].cmd, cmds[i].data, cmds[i].len),
                            TAG, "lcd cmd 0x%02x", cmds[i].cmd);
        if (cmds[i].delay_ms) {
            vTaskDelay(pdMS_TO_TICKS(cmds[i].delay_ms));
        }
    }
    return ESP_OK;
}

static esp_err_t lcd_reset_gpio(void)
{
    gpio_config_t reset = {
        .pin_bit_mask = 1ULL << BSP_LCD_ST7789_RST_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&reset), TAG, "reset gpio");

    gpio_set_level(BSP_LCD_ST7789_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(BSP_LCD_ST7789_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    return ESP_OK;
}

static void lvgl_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(APP_LCD_LVGL_TICK_MS);
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area,
                          lv_color_t *color_map)
{
    esp_err_t err = esp_lcd_panel_draw_bitmap(s_lcd_panel, area->x1, area->y1,
                                              area->x2 + 1, area->y2 + 1,
                                              color_map);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lvgl flush failed: %s", esp_err_to_name(err));
        lv_disp_flush_ready(drv);
    }
}

static bool lcd_color_trans_done_cb(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx)
{
    (void)panel_io;
    (void)edata;
    (void)user_ctx;
    if (s_lvgl_disp_drv) {
        lv_disp_flush_ready(s_lvgl_disp_drv);
    }
    return false;
}

static esp_err_t touch_reset(void)
{
    gpio_config_t rst = {
        .pin_bit_mask = 1ULL << BSP_LCD_TOUCH_RST_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&rst), TAG, "touch rst gpio");

    gpio_config_t intr = {
        .pin_bit_mask = 1ULL << BSP_LCD_TOUCH_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&intr), TAG, "touch int gpio");

    gpio_set_level(BSP_LCD_TOUCH_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(BSP_LCD_TOUCH_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(80));
    return ESP_OK;
}

static esp_err_t touch_attach(void)
{
    if (s_touch) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(touch_reset(), TAG, "touch reset");
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) {
        ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c");
        bus = bsp_i2c_bus();
    }
    if (!bus) {
        return ESP_ERR_INVALID_STATE;
    }

    static const uint8_t candidates[] = {
        BSP_I2C_ADDR_TOUCH_CST816,
        BSP_I2C_ADDR_TOUCH_CST816_ALT,
    };
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        uint8_t addr = candidates[i];
        esp_err_t pe = i2c_master_probe(bus, addr, 50);
        if (pe != ESP_OK) {
            ESP_LOGI(TAG, "touch addr 0x%02X not present (%s)",
                     addr, esp_err_to_name(pe));
            continue;
        }

        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = BSP_I2C0_FREQ_HZ,
        };
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &s_touch),
                            TAG, "touch add");
        s_touch_addr = addr;
        ESP_LOGI(TAG, "touch controller selected at 0x%02X (CST816-compatible)",
                 s_touch_addr);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "no supported touch controller found on I2C");
    return ESP_ERR_NOT_FOUND;
}

static bool touch_read_point(lv_coord_t *x, lv_coord_t *y)
{
    uint8_t buf[5] = {};
    if (touch_read_reg(0x02, buf, sizeof(buf)) != ESP_OK) {
        return false;
    }

    uint8_t points = buf[0] & 0x0f;
    if (points == 0 || points > 2) {
        return false;
    }

    uint16_t raw_x = ((uint16_t)(buf[1] & 0x0f) << 8) | buf[2];
    uint16_t raw_y = ((uint16_t)(buf[3] & 0x0f) << 8) | buf[4];
    int32_t adj_x = (int32_t)raw_x - APP_LCD_X_GAP;
    int32_t adj_y = (int32_t)raw_y - APP_LCD_Y_GAP;
    if (adj_x < 0 || adj_y < 0 ||
        adj_x >= APP_LCD_H_RES || adj_y >= APP_LCD_V_RES) {
        return false;
    }

    *x = (lv_coord_t)adj_x;
    *y = (lv_coord_t)adj_y;
    return true;
}

static void lvgl_touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    (void)drv;
    static lv_coord_t last_x;
    static lv_coord_t last_y;

    if (s_touch && touch_read_point(&last_x, &last_y)) {
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
    data->point.x = last_x;
    data->point.y = last_y;
}

static void lvgl_create_demo_ui(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101820), 0);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "LR2021 Radio");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

    lv_obj_t *subtitle = lv_label_create(scr);
    lv_label_set_text(subtitle, "ST7789T3 + LVGL + Touch");
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0xa7b0be), 0);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 42);

    lv_obj_t *bar = lv_bar_create(scr);
    lv_obj_set_size(bar, 190, 16);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 76);
    lv_bar_set_range(bar, 0, 100);
    lv_bar_set_value(bar, 72, LV_ANIM_OFF);

    lv_obj_t *slider = lv_slider_create(scr);
    lv_obj_set_width(slider, 190);
    lv_obj_align(slider, LV_ALIGN_TOP_MID, 0, 118);
    lv_slider_set_range(slider, 0, 100);
    lv_slider_set_value(slider, 35, LV_ANIM_OFF);

    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 120, 42);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 158);
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Touch");
    lv_obj_center(btn_label);

    lv_obj_t *status = lv_label_create(scr);
    lv_label_set_text(status, s_touch ? "Touch: ready" : "Touch: not found");
    lv_obj_set_style_text_color(status, lv_color_hex(0xd9e6f2), 0);
    lv_obj_align(status, LV_ALIGN_BOTTOM_MID, 0, -18);
}

static void lvgl_task(void *arg)
{
    (void)arg;
    while (true) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(APP_LCD_LVGL_TASK_DELAY_MS));
    }
}

esp_err_t bsp_lcd_init(void)
{
    if (s_lcd_panel) {
        return ESP_OK;
    }

    gpio_config_t backlight = {
        .pin_bit_mask = 1ULL << BSP_LCD_ST7789_BL_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&backlight), TAG, "backlight gpio");
    gpio_set_level(BSP_LCD_ST7789_BL_GPIO, 0);

    if (!s_lcd_bus_ready) {
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = BSP_LCD_SPI_MOSI_GPIO,
            .miso_io_num = -1,
            .sclk_io_num = BSP_LCD_SPI_SCLK_GPIO,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = APP_LCD_H_RES * APP_LCD_LVGL_BUFFER_ROWS * sizeof(uint16_t),
        };
        ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO),
                            TAG, "spi bus");
        s_lcd_bus_ready = true;
    }

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = BSP_LCD_SPI_CS_GPIO,
        .dc_gpio_num = BSP_LCD_SPI_DC_GPIO,
        .spi_mode = 0,
        .pclk_hz = APP_LCD_SPI_PCLK_HZ,
        .trans_queue_depth = APP_LCD_SPI_QUEUE_DEPTH,
        .on_color_trans_done = lcd_color_trans_done_cb,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_HOST,
                                                 &io_cfg, &s_lcd_io),
                        TAG, "panel io");

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(s_lcd_io, &panel_cfg, &s_lcd_panel),
                        TAG, "st7789 panel");

    ESP_RETURN_ON_ERROR(lcd_reset_gpio(), TAG, "lcd hw reset");
    gpio_set_level(BSP_LCD_ST7789_BL_GPIO, 1);
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_lcd_panel), TAG, "lcd sw reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_lcd_panel), TAG, "lcd init");
    vTaskDelay(pdMS_TO_TICKS(380));
    ESP_RETURN_ON_ERROR(lcd_send_vendor_init(), TAG, "lcd vendor init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(s_lcd_panel, APP_LCD_X_GAP,
                                              APP_LCD_Y_GAP),
                        TAG, "lcd gap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_lcd_panel, true), TAG, "lcd on");

    ESP_LOGI(TAG, "ST7789T3 LCD ready: %ux%u, SPI3 sclk=%d mosi=%d dc=%d cs=%d bl=%d rst=%d",
             APP_LCD_H_RES, APP_LCD_V_RES, BSP_LCD_SPI_SCLK_GPIO,
             BSP_LCD_SPI_MOSI_GPIO, BSP_LCD_SPI_DC_GPIO, BSP_LCD_SPI_CS_GPIO,
             BSP_LCD_ST7789_BL_GPIO, BSP_LCD_ST7789_RST_GPIO);
    return ESP_OK;
}

esp_err_t bsp_lcd_show_test_pattern(void)
{
    if (!s_lcd_panel) {
        ESP_RETURN_ON_ERROR(bsp_lcd_init(), TAG, "lcd init");
    }

    const size_t rows = APP_LCD_TEST_PATTERN_ROWS;
    const size_t pixels = APP_LCD_H_RES * rows;
    uint16_t *line = heap_caps_malloc(pixels * sizeof(uint16_t),
                                      MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!line) {
        return ESP_ERR_NO_MEM;
    }

    static const uint16_t colors[] = {
        0xf800, 0x07e0, 0x001f, 0xffe0, 0x07ff, 0xf81f, 0xffff, 0x0000,
    };
    for (uint32_t y = 0; y < APP_LCD_V_RES; y += rows) {
        uint32_t draw_rows = APP_LCD_V_RES - y;
        if (draw_rows > rows) {
            draw_rows = rows;
        }
        for (uint32_t row = 0; row < draw_rows; ++row) {
            for (uint32_t x = 0; x < APP_LCD_H_RES; ++x) {
                uint32_t band = (x * (sizeof(colors) / sizeof(colors[0]))) / APP_LCD_H_RES;
                line[row * APP_LCD_H_RES + x] = colors[band];
            }
        }
        esp_err_t err = esp_lcd_panel_draw_bitmap(s_lcd_panel, 0, y,
                                                  APP_LCD_H_RES, y + draw_rows,
                                                  line);
        if (err != ESP_OK) {
            heap_caps_free(line);
            ESP_RETURN_ON_ERROR(err, TAG, "draw test");
        }
    }

    heap_caps_free(line);
    ESP_LOGI(TAG, "LCD test pattern drawn");
    return ESP_OK;
}

esp_err_t bsp_lcd_start_lvgl_demo(void)
{
    if (s_lvgl_started) {
        return ESP_OK;
    }
    if (!s_lcd_panel) {
        ESP_RETURN_ON_ERROR(bsp_lcd_init(), TAG, "lcd init");
    }

    lv_init();

    const size_t pixels = APP_LCD_H_RES * APP_LCD_LVGL_BUFFER_ROWS;
    lv_color_t *buf1 = heap_caps_malloc(pixels * sizeof(lv_color_t),
                                        MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    lv_color_t *buf2 = heap_caps_malloc(pixels * sizeof(lv_color_t),
                                        MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!buf1 || !buf2) {
        heap_caps_free(buf1);
        heap_caps_free(buf2);
        return ESP_ERR_NO_MEM;
    }

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, pixels);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = APP_LCD_H_RES;
    disp_drv.ver_res = APP_LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    s_lvgl_disp_drv = &disp_drv;
    lv_disp_drv_register(&disp_drv);

    esp_err_t touch_err = touch_attach();
    if (touch_err == ESP_OK) {
        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init(&indev_drv);
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = lvgl_touch_read_cb;
        lv_indev_drv_register(&indev_drv);
    }

    lvgl_create_demo_ui();

    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lvgl_tick",
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t tick_timer = NULL;
    ESP_RETURN_ON_ERROR(esp_timer_create(&tick_args, &tick_timer), TAG,
                        "lvgl tick create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(tick_timer,
                                                 APP_LCD_LVGL_TICK_MS * 1000U),
                        TAG, "lvgl tick start");

    BaseType_t ok = xTaskCreatePinnedToCore(lvgl_task, "lvgl",
                                            APP_LCD_LVGL_TASK_STACK_BYTES, NULL,
                                            APP_LCD_LVGL_TASK_PRIORITY, NULL,
                                            APP_LCD_LVGL_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_lvgl_started = true;
    ESP_LOGI(TAG, "LVGL demo started%s", s_touch ? " with touch" : "");
    return ESP_OK;
}
