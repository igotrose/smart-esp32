#include "drv_i2c_touch.h"

static char* TAG = "DEV_I2C_TOUCH";

esp_lcd_touch_handle_t tp = NULL;

esp_err_t dev_i2c_touch_init(void)
{
    esp_err_t ret;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = {
        .dev_addr = 0x38,
        .scl_speed_hz = BSP_I2C_FREQ_HZ,
        .control_phase_bytes = 1,
        .dc_bit_offset = 0,
        .lcd_cmd_bits = 8,
        .flags = {
            .disable_control_phase = 1,
        }
    };
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = DEV_LCD_H_RES,
        .y_max = DEV_LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    ret = esp_lcd_new_panel_io_i2c_v2(i2c_bus_handle, &tp_io_config, &tp_io_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create LCD panel IO handle: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init touch panel");
        return ret;
    }
    ESP_LOGI(TAG, "Touch panel init success");
    return ESP_OK;
}

static void dev_i2c_touch_task(void* arg)
{
    uint16_t touch_x[1];
    uint16_t touch_y[1];
    uint16_t touch_strength[1];
    uint8_t touch_cnt = 0;

    while (1)
    {
        esp_lcd_touch_read_data(tp);
        bool touched = esp_lcd_touch_get_coordinates(tp, touch_x, touch_y, touch_strength, &touch_cnt, 1);

        if (touched && touch_cnt > 0)
        {
            ESP_LOGI(TAG, "Touch at (%d, %d) with strength %d", touch_x[0], touch_y[0], touch_strength[0]);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

}

void aiot_esp32_s3_07_demo_i2c_touch(void)
{
    xTaskCreate(dev_i2c_touch_task, "dev_i2c_touch_task", 4096, NULL, 5, NULL);
}