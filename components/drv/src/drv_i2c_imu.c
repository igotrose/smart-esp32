#include "drv_i2c_imu.h"

static char* TAG = "DRV_I2C_IMU";

i2c_master_bus_handle_t bus_handle = { 0 };
i2c_master_dev_handle_t dev_handle = { 0 };

t_sQMI8658 i2c_imu = { 0 };

esp_err_t bsp_i2c_init(void)
{
    i2c_master_bus_config_t i2c_mst_config =
    {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
        .glitch_ignore_cnt = 7,
        .i2c_port = BSP_I2C_NUM,
        .scl_io_num = BSP_I2C_SCL,
        .sda_io_num = BSP_I2C_SDA,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = BSP_I2C_FREQ_HZ,
        .device_address = DEV_IMU_ADDR,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    ESP_LOGI(TAG, "I2C bus init OK");

    return ESP_OK;
}

esp_err_t dev_imu_reg_read_byte(uint8_t reg_addr, uint8_t* data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, -1);
}

esp_err_t dev_imu_reg_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t wirte_buffer[] = { reg_addr, data };

    return i2c_master_transmit(dev_handle, (const uint8_t*)wirte_buffer, sizeof(wirte_buffer), (1000 / portTICK_PERIOD_MS));
}

esp_err_t dev_imu_init(void)
{
    uint8_t id = 0, revision_id = 0;

    dev_imu_reg_read_byte(QMI8658_WHO_AM_I, &id, 1);
    while (id != 0x05 || revision_id != 0x7C)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (id != 0x05)
        {
            dev_imu_reg_read_byte(QMI8658_WHO_AM_I, &id, 1);
        }
        else if (revision_id != 0x7C)
        {
            dev_imu_reg_read_byte(QMI8658_REVISION_ID, &revision_id, 1);
        }
    }
    ESP_LOGI(TAG, "DEV IMU ID = 0x%x, REVISION_ID = 0x%x\r\n", id, revision_id);

    // reset dev imu
    dev_imu_reg_write_byte(QMI8658_RESET, 0xb0);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // serial interface and sensor enable  
    dev_imu_reg_write_byte(QMI8658_CTRL1, 0x40);
    // accelerometer setting
    dev_imu_reg_write_byte(QMI8658_CTRL2, 0x05);
    // gyroscope setting
    dev_imu_reg_write_byte(QMI8658_CTRL3, 0x55);
    // sensor data processing setting 
    dev_imu_reg_write_byte(QMI8658_CTRL5, 0x33);
    // enable sensor and configure data reads 
    dev_imu_reg_write_byte(QMI8658_CTRL7, 0x03);
    // motion detection control
    dev_imu_reg_write_byte(QMI8658_CTRL8, 0x1f);

    return ESP_OK;
}

void dev_imu_read_temperature(t_sQMI8658* p)
{
    uint8_t temp_h, temp_l;

    dev_imu_reg_read_byte(QMI8658_TEMP_H, &temp_h, 1);
    dev_imu_reg_read_byte(QMI8658_TEMP_L, &temp_l, 1);

    p->Temperature = temp_h + temp_l / 256.0;
}

void dev_imu_read_acceleration(t_sQMI8658* p)
{
    uint8_t ax_l, ax_h, ay_l, ay_h, az_l, az_h;
    dev_imu_reg_read_byte(QMI8658_AX_L, &ax_l, 1);
    dev_imu_reg_read_byte(QMI8658_AX_H, &ax_h, 1);
    dev_imu_reg_read_byte(QMI8658_AY_L, &ay_l, 1);
    dev_imu_reg_read_byte(QMI8658_AY_H, &ay_h, 1);
    dev_imu_reg_read_byte(QMI8658_AZ_L, &az_l, 1);
    dev_imu_reg_read_byte(QMI8658_AZ_H, &az_h, 1);

    p->raw_acc_x = (int16_t)((ax_h << 8) | ax_l);
    p->raw_acc_y = (int16_t)((ay_h << 8) | ay_l);
    p->raw_acc_z = (int16_t)((az_h << 8) | az_l);

#if 0
    ESP_LOGI(TAG, "Raw Acc: X=%6d, Y=%6d, Z=%6d", p->raw_acc_x, p->raw_acc_y, p->raw_acc_z);
#endif
    
    const float scale_factor = 2.0f / 32768;

    p->acc_x = p->raw_acc_x * scale_factor;
    p->acc_y = p->raw_acc_y * scale_factor;
    p->acc_z = p->raw_acc_z * scale_factor;

}

void dev_imu_read_angular_rate(t_sQMI8658* p)
{
    uint8_t gx_l, gx_h, gy_l, gy_h, gz_l, gz_h;
    dev_imu_reg_read_byte(QMI8658_GX_L, &gx_l, 1);
    dev_imu_reg_read_byte(QMI8658_GX_H, &gx_h, 1);
    dev_imu_reg_read_byte(QMI8658_GY_L, &gy_l, 1);
    dev_imu_reg_read_byte(QMI8658_GY_H, &gy_h, 1);
    dev_imu_reg_read_byte(QMI8658_GZ_L, &gz_l, 1);
    dev_imu_reg_read_byte(QMI8658_GZ_H, &gz_h, 1);

    p->raw_gyr_x = (int16_t)((gx_h << 8) | gx_l);
    p->raw_gyr_y = (int16_t)((gy_h << 8) | gy_l);
    p->raw_gyr_z = (int16_t)((gz_h << 8) | gz_l);

#if 0
    ESP_LOGI(TAG, "Raw Ang: X=%6d, Y=%6d, Z=%6d", p->raw_gyr_x, p->raw_gyr_y, p->raw_gyr_z);
#endif

    const float GYRO_SCALE = 2048.0f / 32768.0f;

    p->gyr_x = p->raw_gyr_x * GYRO_SCALE;
    p->gyr_y = p->raw_gyr_y * GYRO_SCALE;
    p->gyr_z = p->raw_gyr_z * GYRO_SCALE;

}

void i2c_imu_example_task(void* arg)
{
    bsp_i2c_init();
    dev_imu_init();

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        dev_imu_read_temperature(&i2c_imu);
        dev_imu_read_acceleration(&i2c_imu);
        dev_imu_read_angular_rate(&i2c_imu);
#if 1
        ESP_LOGI(TAG, "Temperature = %.2f °C", i2c_imu.Temperature);
        ESP_LOGI(TAG, "Acceleration: X=%.2f g, Y=%.2f g, Z=%.2f g", i2c_imu.acc_x, i2c_imu.acc_y, i2c_imu.acc_z);
        ESP_LOGI(TAG, "Gyro: X=%.2f °/s, Y=%.2f °/s, Z=%.2f °/s", i2c_imu.gyr_x, i2c_imu.gyr_y, i2c_imu.gyr_z);
#endif
    }

}

void aiot_exp32_c3_02_demo_i2c_imu(void)
{
    xTaskCreate(i2c_imu_example_task, "i2c_imu_example_task", 4096, NULL, 5, NULL);
}