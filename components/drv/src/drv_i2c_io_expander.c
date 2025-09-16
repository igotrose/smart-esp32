#include "drv_i2c_io_expander.h" 

static char* TAG = "DRV_IO_EXPANDER";

i2c_master_dev_handle_t io_expander_dev_handle;


static esp_err_t dev_io_expander_reg_read_byte(uint8_t reg_addr, uint8_t* data, size_t len)
{
    if (io_expander_dev_handle == NULL)
    {
        ESP_LOGE(TAG, "%s: io_expander_dev_handle is NULL", __FUNCTION__);
        return ESP_FAIL;
    }
    return i2c_master_transmit_receive(io_expander_dev_handle, &reg_addr, 1, data, len, -1);
}

static esp_err_t dev_io_expander_reg_write_byte(uint8_t reg_addr, uint8_t data)
{
    if (io_expander_dev_handle == NULL)
    {
        ESP_LOGE(TAG, "%s: io_expander_dev_handle is NULL", __FUNCTION__);
        return ESP_FAIL;
    }
    uint8_t write_buffer[2] = { reg_addr, data };
    return i2c_master_transmit(io_expander_dev_handle, (const uint8_t*)write_buffer, sizeof(write_buffer), (1000 / portTICK_PERIOD_MS));
}

esp_err_t dev_io_expander_get_mode(uint8_t pin, uint8_t* mode)
{
    uint8_t config_reg;
    esp_err_t err = dev_io_expander_reg_read_byte(DEV_IO_EXPANDER_CONFIG, &config_reg, 1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read configuration register");
        return err;
    }
     *mode = ((config_reg & pin) == pin) ? 1 : 0;
    ESP_LOGI(TAG, "get mode of pin:%d, mode:%d", pin, *mode);
    return ESP_OK;
}

esp_err_t dev_io_expander_set_mode(uint8_t pin, uint8_t mode)
{
    uint8_t config_reg;
    esp_err_t ret = dev_io_expander_reg_read_byte(DEV_IO_EXPANDER_CONFIG, &config_reg, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read configuration register");
        return ret;
    }
    if (mode)
    {
        config_reg |= pin;
    }
    else
    {
        config_reg &= ~pin;
    }
    ret = dev_io_expander_reg_write_byte(DEV_IO_EXPANDER_CONFIG, config_reg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write configuration register");
        return ret;
    }
    return ESP_OK;
}

esp_err_t dev_io_expander_get_input_value(uint8_t pin, uint8_t* pin_value)
{
    uint8_t input_value;
    esp_err_t ret = dev_io_expander_reg_read_byte(DEV_IO_EXPANDER_INPUT_PORT, &input_value, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read input port");
        return ret;
    }
    *pin_value = ((input_value & pin) == pin) ? 1 : 0;
    return ESP_OK;
}

esp_err_t dev_io_expander_set_output_value(uint8_t pin, uint8_t pin_value)
{
    uint8_t output_value;
    esp_err_t ret = dev_io_expander_get_output_value(pin, &output_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get output value");
        return ret;
    }
    output_value = (pin_value) ? (output_value | pin) : (output_value & ~pin);
    ret = dev_io_expander_reg_write_byte(DEV_IO_EXPANDER_OUTPUT_PORT, output_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write output port");
        return ret;
    }
    return ESP_OK;
}

esp_err_t dev_io_expander_get_output_value(uint8_t pin, uint8_t* pin_value)
{
    uint8_t output_value;
    esp_err_t ret = dev_io_expander_reg_read_byte(DEV_IO_EXPANDER_OUTPUT_PORT, &output_value, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read output port");
        return ret;
    }
    *pin_value = (output_value & pin) ? 1 : 0;
    return ESP_OK;
}

esp_err_t dev_io_expander_init(void)
{
    esp_err_t ret = dev_i2c_bus_add_devices(DEV_IO_EXPANDER_ADDR, &io_expander_dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "dev_i2c_bus_add_devices failed, ret:%d", ret);
        return ret;
    }
#if 0
    uint8_t mode, pin_value;
    esp_err_t ret = dev_io_expander_set_mode(IO_EXPANDER_IO0 | IO_EXPANDER_IO1 | IO_EXPANDER_IO2, 0);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set mode of IO0, IO1, IO2");
    }
    ret = dev_io_expander_get_mode(IO_EXPANDER_IO0 | IO_EXPANDER_IO1 | IO_EXPANDER_IO2, &mode);
    if (ret)
    {
        ESP_LOGE(TAG, "Failed to get mode of IO0, IO1, IO2");
    }
    ESP_LOGI(TAG, "Already set mode of IO0, IO1, IO2, mode:%d", mode);

    ret = dev_io_expander_set_output_value(IO_EXPANDER_IO0 | IO_EXPANDER_IO1 | IO_EXPANDER_IO2, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set output value of IO0, IO1, IO2");
    }
    ESP_LOGI(TAG, "Set output value of IO0, IO1, IO2");
    ret = dev_io_expander_get_output_value(IO_EXPANDER_IO0 | IO_EXPANDER_IO1 | IO_EXPANDER_IO2, &pin_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get output value of IO0, IO1, IO2");
    }
    ESP_LOGI(TAG, "Get output value of IO0, IO1, IO2, pin_value:%d", pin_value);
#endif
    
    return ESP_OK;
}


void aiot_esp32_s3_04_demo_io_expander(void)
{
#if 0
    uint8_t default_command[4];
    dev_io_expander_reg_read_byte(DEV_IO_EXPANDER_INPUT_PORT, &default_command[0], 1);
    dev_io_expander_reg_read_byte(DEV_IO_EXPANDER_OUTPUT_PORT, &default_command[1], 1);
    dev_io_expander_reg_read_byte(DEV_IO_EXPANDER_POLARITY_INVERSION, &default_command[2], 1);
    dev_io_expander_reg_read_byte(DEV_IO_EXPANDER_CONFIG, &default_command[3], 1);
    ESP_LOGI(TAG, "Input Port: 0x%02x", default_command[0]);
    ESP_LOGI(TAG, "Output Port: 0x%02x", default_command[1]);
    ESP_LOGI(TAG, "Prolarity Inversion: 0x%02x", default_command[2]);
    ESP_LOGI(TAG, "Configuration: 0x%02x", default_command[3]);
#endif
    uint8_t mode, pin_value;
    esp_err_t ret = dev_io_expander_set_mode(IO_EXPANDER_IO0 | IO_EXPANDER_IO1 | IO_EXPANDER_IO2, 0);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set mode of IO0, IO1, IO2");
    }
    ret = dev_io_expander_get_mode(IO_EXPANDER_IO0 | IO_EXPANDER_IO1 | IO_EXPANDER_IO2, &mode);
    if (ret)
    {
        ESP_LOGE(TAG, "Failed to get mode of IO0, IO1, IO2");
    }
    ESP_LOGI(TAG, "Already set mode of IO0, IO1, IO2, mode:%d", mode);

    ret = dev_io_expander_set_output_value(IO_EXPANDER_IO0 | IO_EXPANDER_IO1 | IO_EXPANDER_IO2, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set output value of IO0, IO1, IO2");
    }
    ESP_LOGI(TAG, "Set output value of IO0, IO1, IO2");
    ret = dev_io_expander_get_output_value(IO_EXPANDER_IO0 | IO_EXPANDER_IO1 | IO_EXPANDER_IO2, &pin_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get output value of IO0, IO1, IO2");
    }
    ESP_LOGI(TAG, "Get output value of IO0, IO1, IO2, pin_value:%d", pin_value);
}


    