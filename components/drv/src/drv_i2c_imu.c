#include "drv_i2c_imu.h"

static char* TAG = "DRV_I2C_IMU";

i2c_master_bus_handle_t bus_handle = { 0 };
i2c_master_dev_handle_t dev_handle = { 0 };

qmi8658_data i2c_imu = { 0 };

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
    uint8_t write_buffer[] = { reg_addr, data };

    return i2c_master_transmit(dev_handle, (const uint8_t*)write_buffer, sizeof(write_buffer), (1000 / portTICK_PERIOD_MS));
}

esp_err_t dev_imu_init(void)
{
    uint8_t id = 0, revision_id = 0;

    for (int i = 0; i < 10; i++)
    {
        dev_imu_reg_read_byte(QMI8658_WHO_AM_I, &id, 1);
        dev_imu_reg_read_byte(QMI8658_REVISION_ID, &revision_id, 1);
        if (id == 0x05 && revision_id == 0x7C)
        {
            break;
        }
        if (i == 9)
        {
            ESP_LOGE(TAG, "DEV IMU not found");
            return ESP_FAIL;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "DEV IMU ID = 0x%x, REVISION_ID = 0x%x\r\n", id, revision_id);

    // reset dev imu
    dev_imu_reg_write_byte(QMI8658_RESET, 0xb0);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // celebrate dev imu 
    dev_imu_reg_write_byte(QMI8658_CTRL9, 0xa2);
    // serial interface and sensor enable  
    dev_imu_reg_write_byte(QMI8658_CTRL1, 0x60);
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

void dev_imu_config_accelerator(uint8_t self_test, enum qmi8658_accel_range range, enum qmi8658_accel_odr odr)
{
    uint8_t ctrl2_reg = 0;

    /* aST */
    if (self_test)
    {
        ctrl2_reg |= self_test << 7;
    }
    /* aFS */
    ctrl2_reg |= range;
    /* aODR */
    ctrl2_reg |= odr;

    dev_imu_reg_write_byte(QMI8658_CTRL2, ctrl2_reg);
}

void dev_imu_gyroscope_setting(uint8_t self_test, enum qmi8658_gyro_range range, enum qmi8658_gyro_odr odr)
{
    uint8_t ctrl3_reg = 0;

    /* gST */
    if (self_test)
    {
        ctrl3_reg |= self_test << 7;
    }
    /* gFS */
    ctrl3_reg |= range;
    /* gODR */
    ctrl3_reg |= odr;

    dev_imu_reg_write_byte(QMI8658_CTRL3, ctrl3_reg);
}

void dev_imu_sensor_data_processing_setting(enum qmi8658_LPF_Mode gLPF_MODE, uint8_t gLPF_EN, enum qmi8658_LPF_Mode aLPF_MODE, uint8_t aLPF_EN)
{
    uint8_t ctrl5_reg = 0;
    if (gLPF_EN)
    {
        ctrl5_reg |= gLPF_MODE;
        ctrl5_reg |= gLPF_EN << 4;
    }
    if (aLPF_EN)
    {
        ctrl5_reg |= aLPF_MODE;
        ctrl5_reg |= aLPF_EN;
    }

    dev_imu_reg_write_byte(QMI8658_CTRL5, ctrl5_reg);
}

void dev_imu_read_temperature(qmi8658_data* p)
{
    uint8_t temp[2] = {0};

    dev_imu_reg_read_byte(QMI8658_TEMP_H, &temp[0], 1);
    dev_imu_reg_read_byte(QMI8658_TEMP_L, &temp[1], 1);

    p->Temperature = temp[0] + temp[1] / 256.0;
}

void dev_imu_read_acceleration(qmi8658_data* p)
{
    uint8_t accel[6] = { 0 };

    dev_imu_reg_read_byte(QMI8658_AX_L, accel, 6);

    p->raw_acc_x = (int16_t)((accel[1] << 8) | accel[0]);
    p->raw_acc_y = (int16_t)((accel[3] << 8) | accel[2]);
    p->raw_acc_z = (int16_t)((accel[5] << 8) | accel[4]);

#if 0
    ESP_LOGI(TAG, "Raw Acc: X=%6d, Y=%6d, Z=%6d", p->raw_acc_x, p->raw_acc_y, p->raw_acc_z);
#endif

    const float scale_factor = 2.0f / 32768;

    p->acc_x = p->raw_acc_x * scale_factor;
    p->acc_y = p->raw_acc_y * scale_factor;
    p->acc_z = p->raw_acc_z * scale_factor;

}

void dev_imu_read_angular_rate(qmi8658_data* p)
{
    uint8_t gyro[6] = { 0 };

    dev_imu_reg_read_byte(QMI8658_GX_L, gyro, 6);

    p->raw_gyr_x = (int16_t)((gyro[1] << 8) | gyro[0]);
    p->raw_gyr_y = (int16_t)((gyro[3] << 8) | gyro[2]);
    p->raw_gyr_z = (int16_t)((gyro[5] << 8) | gyro[4]);

#if 0
    ESP_LOGI(TAG, "Raw Ang: X=%6d, Y=%6d, Z=%6d", p->raw_gyr_x, p->raw_gyr_y, p->raw_gyr_z);
#endif

    const float GYRO_SCALE = 2048.0f / 32768.0f;

    p->gyr_x = p->raw_gyr_x * GYRO_SCALE;
    p->gyr_y = p->raw_gyr_y * GYRO_SCALE;
    p->gyr_z = p->raw_gyr_z * GYRO_SCALE;

}

void dev_imu_in_loW_power_mode(bool lowpower)
{
    if (lowpower)
    {

    }
    else
    {
        
    }
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