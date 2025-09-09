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

#define DEV_IMU_ADDR                0x6A
#define DEV_IMU_ACC_GYR_DISABLE     0x00
#define DEV_IMU_ACC_ENABLE         0x01
#define DEV_IMU_GYR_ENABLE         0x02
#define DEV_IMU_ACC_GYR_ENABLE     DEV_IMU_ACC_ENABLE | DEV_IMU_GYR_ENABLE


typedef struct
{
    // origin adc value
    int16_t raw_acc_x;
    int16_t raw_acc_y;
    int16_t raw_acc_z;
    int16_t raw_gyr_x;
    int16_t raw_gyr_y;
    int16_t raw_gyr_z;

    // physical quantity
    float acc_x;
    float acc_y;
    float acc_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;

    float Temperature;

    float pitch;
    float roll;
    float yaw;
} qmi8658_data;

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

enum qmi8658_LPF_Mode
{                                   /* bandwidth HZ */
    aLPF_MODE_0 = 0x00 << 1,        /* 2.66% of ODR */
    aLPF_MODE_1 = 0x01 << 1,        /* 3.63% of ODR */
    aLPF_MODE_2 = 0x02 << 1,        /* 5.39% of ODR */
    aLPF_MODE_3 = 0x03 << 1,        /* 13.37% of ODR */

    gLPF_MODE_0 = 0x00 << 5,        /* 2.66% of ODR */
    gLPF_MODE_1 = 0x01 << 5,        /* 3.63% of ODR */
    gLPF_MODE_2 = 0x02 << 5,        /* 5.39% of ODR */
    gLPF_MODE_3 = 0x03 << 5,        /* 13.37% of ODR */
};

enum qmi8658_accel_range
{
    acc_full_scale_2g = 0x00 << 4,
    acc_full_scale_4g = 0x01 << 4,
    acc_full_scale_8g = 0x02 << 4,
    acc_full_scale_16g = 0x03 << 4
};

enum qmi8658_accel_odr
{
    accel_odr_8000Hz = 0x00,
    accel_odr_4000Hz = 0x01,
    accel_odr_2000Hz = 0x02,
    accel_odr_1000Hz = 0x03,
    accel_odr_500Hz = 0x04,
    accel_odr_250Hz = 0x05,
    accel_odr_125Hz = 0x06,
    accel_odr_62_5Hz = 0x07,
    accel_odr_31_25Hz = 0x08,
    accel_odr_LowPower_128Hz = 0x0c,
    accel_odr_LowPower_21Hz = 0x0d,
    accel_odr_LowPower_11Hz = 0x0e,
    accel_odr_LowPower_3Hz = 0x0f
};

enum qmi8658_gyro_range
{
    gyro_range_16dps = 0 << 4,
    gyro_range_32dps = 1 << 4,
    gyro_range_64dps = 2 << 4,
    gyro_range_128dps = 3 << 4,
    gyro_range_256dps = 4 << 4,
    gyro_range_512dps = 5 << 4,
    gyro_range_1024dps = 6 << 4,
    gyro_range_2048dps = 7 << 4
};

enum qmi8658_gyro_odr
{
    gyro_odr_8000Hz = 0x00,
    gyro_odr_4000Hz = 0x01,
    gyro_odr_2000Hz = 0x02,
    gyro_odr_1000Hz = 0x03,
    gyro_odr_500Hz = 0x04,
    gyro_odr_250Hz = 0x05,
    gyro_odr_125Hz = 0x06,
    gyro_odr_62_5Hz = 0x07,
    gyro_odr_31_25Hz = 0x08
};

typedef struct
{
    uint8_t en_sensors;
    enum qmi8658_accel_range acc_range;
    enum qmi8658_accel_odr acc_odr;
    enum qmi8658_gyro_range gyro_range;
    enum qmi8658_gyro_odr gyro_odr;
    uint8_t sync_sample;
};

esp_err_t bsp_i2c_init(void);

/* config interface */
void dev_imu_accelerator_setting(uint8_t self_test,
                                    enum qmi8658_accel_range range, 
                                    enum qmi8658_accel_odr odr);
void dev_imu_gyroscope_setting(uint8_t self_test, 
                                    enum qmi8658_gyro_range range, 
                                    enum qmi8658_gyro_odr odr);
void dev_imu_sensor_data_processing_setting(enum qmi8658_LPF_Mode gLPF_MODE, 
                                                uint8_t gLPF_EN, 
                                                enum qmi8658_LPF_Mode aLPF_MODE, 
                                                uint8_t aLPF_EN);
/* read interface */
void dev_imu_read_temperature(qmi8658_data* p);
void dev_imu_read_acceleration(qmi8658_data* p);
void dev_imu_read_angular_rate(qmi8658_data* p);

/* function interface */
void dev_imu_in_low_power_mode(bool lowpower);
void dev_imu_step_count(qmi8658_data* p);
void dev_imu_motion_detect(qmi8658_data* p);
void dev_imu_fall_detect(qmi8658_data* p);

esp_err_t dev_imu_init(void);

void aiot_exp32_c3_02_demo_i2c_imu(void);


#endif /* _DRV_I2C_IMU_H_ */
