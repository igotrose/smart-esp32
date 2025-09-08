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
    uint8_t id = 0;

    dev_imu_reg_read_byte(QMI8658_WHO_AM_I, &id, 1);
    while (id != 0x05)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        dev_imu_reg_read_byte(QMI8658_WHO_AM_I, &id, 1);
    }
    ESP_LOGI(TAG, "DEV IMU OK");

    dev_imu_reg_write_byte(QMI8658_RESET, 0xb0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    dev_imu_reg_write_byte(QMI8658_CTRL1, 0x40);
    dev_imu_reg_write_byte(QMI8658_CTRL7, 0x03);
    dev_imu_reg_write_byte(QMI8658_CTRL2, 0x95);
    dev_imu_reg_write_byte(QMI8658_CTRL3, 0xd5);


    return ESP_OK;
}

void dev_imu_read_acc_gry(t_sQMI8658* p)
{
    uint8_t status, data_ready = 0;
    int16_t buf[6];

    dev_imu_reg_read_byte(QMI8658_STATUS0, &status, 1);
    if (status & 0x03)
    {
        data_ready = 1;
    }
    if (data_ready)
    {
        data_ready = 0;
        
        dev_imu_reg_read_byte(QMI8658_AX_L, (uint8_t*)&buf[0], 1);
        dev_imu_reg_read_byte(QMI8658_AX_H, (uint8_t*)&buf[1], 1);
        dev_imu_reg_read_byte(QMI8658_AY_L, (uint8_t*)&buf[2], 1);
        dev_imu_reg_read_byte(QMI8658_AY_H, (uint8_t*)&buf[3], 1);
        dev_imu_reg_read_byte(QMI8658_AZ_L, (uint8_t*)&buf[4], 1);
        dev_imu_reg_read_byte(QMI8658_AZ_H, (uint8_t*)&buf[5], 1);
        dev_imu_reg_read_byte(QMI8658_GX_L, (uint8_t*)&buf[6], 1);
        dev_imu_reg_read_byte(QMI8658_GX_H, (uint8_t*)&buf[7], 1);
        dev_imu_reg_read_byte(QMI8658_GY_L, (uint8_t*)&buf[8], 1);
        dev_imu_reg_read_byte(QMI8658_GY_H, (uint8_t*)&buf[9], 1);
        dev_imu_reg_read_byte(QMI8658_GZ_L, (uint8_t*)&buf[10], 1);
        dev_imu_reg_read_byte(QMI8658_GZ_H, (uint8_t*)&buf[11], 1);

        p->acc_x = buf[0];
        p->acc_y = buf[1];
        p->acc_z = buf[2];
        p->gyr_x = buf[3];
        p->gyr_y = buf[4];
        p->gyr_z = buf[5];
    }
}

void dev_imu_fetch_angleFromAcc(t_sQMI8658* p)
{
    float temp;

    dev_imu_read_acc_gry(p);

    temp = (float)p->acc_x / sqrt(((float)p->acc_y * (float)p->acc_y + (float)p->acc_z * (float)p->acc_z));
    p->AngleX = atan(temp) * 57.29578f; // 180/π=57.29578

    temp = (float)p->acc_y / sqrt(((float)p->acc_x * (float)p->acc_x + (float)p->acc_z * (float)p->acc_z));
    p->AngleY = atan(temp) * 57.29578f; // 180/π=57.29578

    temp = sqrt(((float)p->acc_x * (float)p->acc_x + (float)p->acc_y * (float)p->acc_y)) / (float)p->acc_z;
    p->AngleZ = atan(temp) * 57.29578f; // 180/π=57.29578

}
void i2c_imu_example_task(void* arg)
{
    bsp_i2c_init();
    dev_imu_init();

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        dev_imu_fetch_angleFromAcc(&i2c_imu);
        // ESP_LOGI(TAG, "angle_x = %.1f  angle_y = %.1f angle_z = %.1f", i2c_imu.AngleX, i2c_imu.AngleY, i2c_imu.AngleZ);
    }

}

void aiot_exp32_c3_02_demo_i2c_imu(void)
{
    xTaskCreate(i2c_imu_example_task, "i2c_imu_example_task", 4096, NULL, 5, NULL);
}