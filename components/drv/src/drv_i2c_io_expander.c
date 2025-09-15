#include "drv_i2c_io_expander.h" 

static char* TAG = "DRV_IO_EXPANDER";

i2c_master_dev_handle_t io_expander_dev_handle;


static esp_err_t dev_io_expander_reg_read_byte(uint8_t reg_addr, uint8_t* data, size_t len)
{
    if(io_expander_dev_handle == NULL)
    {
        ESP_LOGE(TAG, "%s: io_expander_dev_handle is NULL", __FUNCTION__);
        return ESP_FAIL;
    }
    return i2c_master_transmit_receive(io_expander_dev_handle, &reg_addr, 1, data, len, -1);
}

static esp_err_t dev_io_expander_reg_write_byte(uint8_t reg_addr, uint8_t data)
{
    if(io_expander_dev_handle == NULL)
    {
        ESP_LOGE(TAG, "%s: io_expander_dev_handle is NULL", __FUNCTION__);
        return ESP_FAIL;
    }
    uint8_t write_buffer[2] = {reg_addr, data};
    return i2c_master_transmit(io_expander_dev_handle,(const uint8_t*)write_buffer, sizeof(write_buffer), (1000 / portTICK_PERIOD_MS));
}




esp_err_t dev_io_expander_init(void)
{
    esp_err_t ret = dev_i2c_bus_add_devices(DEV_IO_EXPANDER_ADDR, &io_expander_dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "dev_i2c_bus_add_devices failed, ret:%d", ret);
        return ret ;
    }
    return ESP_OK;
}

void aiot_esp32_s3_04_demo_io_expander(void)
{
    uint8_t read_buf[4] = { 0 };
    dev_io_expander_reg_read_byte(DEV_IO_EXPANDER_INPUT_PORT, read_buf, 1);
    ESP_LOGI(TAG, "Input Port: 0x%02x", read_buf[0]);
}