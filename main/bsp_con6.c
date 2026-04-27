#include "bsp.h"

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

#include "app_config.h"

static const char *TAG = "bsp_con6";

static esp_err_t con6_start_camera_mclk_for_detect(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = APP_GC032A_MCLK_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "detect mclk timer");

    ledc_channel_config_t ch = {
        .gpio_num = BSP_GC032A_MCLK_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 1,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {},
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch), TAG, "detect mclk channel");
    ESP_LOGI(TAG, "GC032A detect MCLK: LEDC %uHz duty=50%% gpio=%d",
             APP_GC032A_MCLK_HZ, BSP_GC032A_MCLK_GPIO);
    return ESP_OK;
}

static esp_err_t con6_prepare_for_detect(void)
{
    gpio_config_t safe_out = {
        .pin_bit_mask = (1ULL << BSP_LCD_CS1_GPIO) |
                        (1ULL << BSP_LCD_CS2_GPIO) |
                        (1ULL << BSP_LCD_BL_GPIO) |
                        (1ULL << BSP_LCD_ST7789_BL_GPIO) |
                        (1ULL << BSP_LCD_TOUCH_RST_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&safe_out), TAG, "safe gpio");
    gpio_set_level(BSP_LCD_CS1_GPIO, 1);
    gpio_set_level(BSP_LCD_CS2_GPIO, 1);
    gpio_set_level(BSP_LCD_BL_GPIO, 0);
    gpio_set_level(BSP_LCD_ST7789_BL_GPIO, 0);
    gpio_set_level(BSP_LCD_TOUCH_RST_GPIO, 1);

    gpio_config_t cam_out = {
        .pin_bit_mask = (1ULL << BSP_CAMERA_PWR_EN_GPIO) |
                        (1ULL << BSP_GC032A_PWDN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cam_out), TAG, "camera detect gpio");
    gpio_set_level(BSP_CAMERA_PWR_EN_GPIO, 1);
    ESP_RETURN_ON_ERROR(con6_start_camera_mclk_for_detect(), TAG, "camera detect mclk");
    gpio_set_level(BSP_GC032A_PWDN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(BSP_GC032A_PWDN_GPIO, 0);
    ESP_LOGI(TAG, "GC032A detect PWDN: GPIO%d low (active-high powerdown released)",
             BSP_GC032A_PWDN_GPIO);
    vTaskDelay(pdMS_TO_TICKS(200));
    return ESP_OK;
}

esp_err_t bsp_con6_detect(bsp_con6_peripheral_t *out_peripheral)
{
    if (!out_peripheral) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c");
    ESP_RETURN_ON_ERROR(con6_prepare_for_detect(), TAG, "prepare");

    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t probe = i2c_master_probe(bus, BSP_I2C_ADDR_GC032A, 50);
    if (probe != ESP_OK) {
        *out_peripheral = BSP_CON6_PERIPHERAL_LCD_ST7789;
        ESP_LOGI(TAG, "GC032A addr 0x%02X not present (%s); selecting ST7789T3 LCD",
                 BSP_I2C_ADDR_GC032A, esp_err_to_name(probe));
        return ESP_OK;
    }

    uint8_t idh = 0;
    uint8_t idl = 0;
    esp_err_t e0 = ESP_FAIL;
    esp_err_t e1 = ESP_FAIL;
    i2c_master_dev_handle_t dev = NULL;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BSP_I2C_ADDR_GC032A,
        .scl_speed_hz = BSP_I2C0_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) == ESP_OK) {
        uint8_t reg = 0xf0;
        e0 = i2c_master_transmit_receive(dev, &reg, 1, &idh, 1, 100);
        reg = 0xf1;
        e1 = i2c_master_transmit_receive(dev, &reg, 1, &idl, 1, 100);
        i2c_master_bus_rm_device(dev);
    }

    *out_peripheral = BSP_CON6_PERIPHERAL_CAMERA_GC032A;
    if (e0 == ESP_OK && e1 == ESP_OK) {
        ESP_LOGI(TAG, "GC032A addr 0x%02X ACK, id=0x%02X%02X; selecting camera",
                 BSP_I2C_ADDR_GC032A, idh, idl);
    } else {
        ESP_LOGW(TAG, "GC032A addr 0x%02X ACK but ID read failed (%s/%s); selecting camera by address",
                 BSP_I2C_ADDR_GC032A, esp_err_to_name(e0), esp_err_to_name(e1));
    }
    return ESP_OK;
}
