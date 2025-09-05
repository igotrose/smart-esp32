#ifndef _DRV_I2C_IMU_H_
#define _DRV_I2C_IMU_H_

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

#include "math.h"

#define BSP_I2C_SDA         (GPIO_NUM_1)
#define BSP_I2C_SCL         (GPIO_NUM_2)

#define BSP_I2C_NUM         (I2C_NUM_0)
#define BSP_I2C_FREQ_HZ     100000

#define DEV_IMU_ADDR        0x6A


typedef struct{
    int16_t acc_y;
    int16_t acc_x;
    int16_t acc_z;
    int16_t gyr_y;
    int16_t gyr_x;
    int16_t gyr_z;
    float AngleX;
    float AngleY;
    float AngleZ;
}t_sQMI8658;

enum qmi8658_reg
{
    QMI8658_WHO_AM_I,
    QMI8658_REVISION_ID,
    QMI8658_CTRL1,
    QMI8658_CTRL2,
    QMI8658_CTRL3,
    QMI8658_CTRL4,
    QMI8658_CTRL5,
    QMI8658_CTRL6,
    QMI8658_CTRL7,
    QMI8658_CTRL8,
    QMI8658_CTRL9,
    QMI8658_CATL1_L,
    QMI8658_CATL1_H,
    QMI8658_CATL2_L,
    QMI8658_CATL2_H,
    QMI8658_CATL3_L,
    QMI8658_CATL3_H,
    QMI8658_CATL4_L,
    QMI8658_CATL4_H,
    QMI8658_FIFO_WTM_TH,
    QMI8658_FIFO_CTRL,
    QMI8658_FIFO_SMPL_CNT,
    QMI8658_FIFO_STATUS,
    QMI8658_FIFO_DATA,
    QMI8658_I2CM_STATUS = 44,
    QMI8658_STATUSINT,
    QMI8658_STATUS0,
    QMI8658_STATUS1,
    QMI8658_TIMESTAMP_LOW,
    QMI8658_TIMESTAMP_MID,
    QMI8658_TIMESTAMP_HIGH,
    QMI8658_TEMP_L,
    QMI8658_TEMP_H,
    QMI8658_AX_L,
    QMI8658_AX_H,
    QMI8658_AY_L,
    QMI8658_AY_H,
    QMI8658_AZ_L,
    QMI8658_AZ_H,
    QMI8658_GX_L,
    QMI8658_GX_H,
    QMI8658_GY_L,
    QMI8658_GY_H,
    QMI8658_GZ_L,
    QMI8658_GZ_H,
    QMI8658_MX_L,
    QMI8658_MX_H,
    QMI8658_MY_L,
    QMI8658_MY_H,
    QMI8658_MZ_L,
    QMI8658_MZ_H,
    QMI8658_dQW_L = 73,
    QMI8658_dQW_H,
    QMI8658_dQX_L,
    QMI8658_dQX_H,
    QMI8658_dQY_L,
    QMI8658_dQY_H,
    QMI8658_dQZ_L,
    QMI8658_dQZ_H,
    QMI8658_dVX_L,
    QMI8658_dVX_H,
    QMI8658_dVY_L,
    QMI8658_dVY_H,
    QMI8658_dVZ_L,
    QMI8658_dVZ_H,
    QMI8658_AE_REG1,
    QMI8658_AE_REG2,
    QMI8658_RESET = 96
};

extern 

esp_err_t bsp_i2c_init(void);

esp_err_t dev_imu_reg_read_byte(uint8_t reg_addr, uint8_t* data, size_t len);
esp_err_t dev_imu_reg_write_byte(uint8_t reg_addr, uint8_t data);
void dev_imu_read_acc_gry(t_sQMI8658* p);
void dev_imu_fetch_angleFromAcc(t_sQMI8658* p);

esp_err_t dev_imu_init(void);

void aiot_exp32_c3_02_demo_i2c_imu(void);


#endif /* _DRV_I2C_IMU_H_ */