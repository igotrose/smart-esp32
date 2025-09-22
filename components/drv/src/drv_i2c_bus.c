#include "drv_i2c_bus.h"

static const char* TAG = "DRV_I2C_BUS";

i2c_master_bus_handle_t  i2c_bus_handle = NULL;

esp_err_t dev_i2c_bus_init(void)
{
    if (i2c_bus_handle)
    {
        ESP_LOGW(TAG, "I2C bus already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_master_bus_config_t i2c_msg_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
        .glitch_ignore_cnt = 7,
        .i2c_port = BSP_I2C_NUM,
        .scl_io_num = BSP_I2C_SCL,
        .sda_io_num = BSP_I2C_SDA,
    };

    esp_err_t err = i2c_new_master_bus(&i2c_msg_config, &i2c_bus_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C bus initialized on port %d (SDA: %d, SCL: %d)",
        BSP_I2C_NUM, BSP_I2C_SDA, BSP_I2C_SCL);

    return ESP_OK;
}

esp_err_t dev_i2c_bus_add_devices(uint8_t dev_addr, i2c_master_dev_handle_t* handle)
{
    if (i2c_bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (handle == NULL)
    {
        ESP_LOGE(TAG, "Invalid device handle");
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = BSP_I2C_FREQ_HZ,
        .device_address = dev_addr,
    };

    esp_err_t err = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add device on I2C bus: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Added device on I2C bus with address 0x%02x", dev_addr);
    return ESP_OK;
}

esp_err_t dev_i2c_bus_remove_devices(i2c_master_dev_handle_t* handle)
{
    if (i2c_bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (handle == NULL)
    {
        ESP_LOGE(TAG, "Invalid handle");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = i2c_master_bus_rm_device(*handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to remove device from I2C bus: %s", esp_err_to_name(err));
        return err;
    }

    *handle = NULL;

    return ESP_OK;
}

esp_err_t dev_i2c_bus_deinit(void)
{
    if (i2c_bus_handle == NULL)
    {
        ESP_LOGW(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = i2c_del_master_bus(i2c_bus_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to deinitialize I2C bus: %s", esp_err_to_name(err));
        return err;
    }

    i2c_bus_handle = NULL;
    ESP_LOGI(TAG, "I2C bus deinitialized");
    return ESP_OK;
}

