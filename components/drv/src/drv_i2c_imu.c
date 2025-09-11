#include "drv_i2c_imu.h"

static char* TAG = "DRV_I2C_IMU";

i2c_master_bus_handle_t bus_handle = { 0 };
i2c_master_dev_handle_t dev_handle = { 0 };

dev_imu_data dev_imu = { 0 };

static uint8_t imu_calibrated = 0;

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
    dev_imu_accelerator_setting(0, acc_full_scale_2g, accel_odr_1000Hz);
    // gyroscope setting
    dev_imu_gyroscope_setting(0, gyro_range_128dps, gyro_odr_500Hz);
    // filter setting 
    dev_imu_filter_setting(aLPF_MODE_1, 1, gLPF_MODE_1, 1);
    // enable sensor and configure data reads 
    dev_imu_sensors_enable(DEV_IMU_ACC_GYR_ENABLE);
    // motion detection control
    dev_imu_reg_write_byte(QMI8658_CTRL8, 0x1f);

    return ESP_OK;
}

void dev_imu_sensors_enable(uint8_t enable)
{
    dev_imu.imu_cfg.en_sensors = enable;
    dev_imu_reg_write_byte(QMI8658_CTRL7, enable);
}

void dev_imu_accelerator_setting(uint8_t self_test, enum qmi8658_accel_range range, enum qmi8658_accel_odr odr)
{
    switch (range)
    {
    case acc_full_scale_2g:
        dev_imu.imu_cfg.ssvt_a = (1 << 14);
        break;
    case acc_full_scale_4g:
        dev_imu.imu_cfg.ssvt_a = (1 << 13);
        break;
    case acc_full_scale_8g:
        dev_imu.imu_cfg.ssvt_a = (1 << 12);
        break;
    case acc_full_scale_16g:
        dev_imu.imu_cfg.ssvt_a = (1 << 11);
        break;
    default:
        range = acc_full_scale_8g;
        dev_imu.imu_cfg.ssvt_a = (1 << 12);
        break;
    }

#if 0
    ESP_LOGI(TAG, "Accelerometer sensitivity: %d", dev_imu.imu_cfg.ssvt_a);
#endif 

    uint8_t ctrl2_reg = ((self_test & 0x01) << 7) | range | odr;
    dev_imu_reg_write_byte(QMI8658_CTRL2, ctrl2_reg);
}

void dev_imu_gyroscope_setting(uint8_t self_test, enum qmi8658_gyro_range range, enum qmi8658_gyro_odr odr)
{
    switch (range)
    {
    case gyro_range_16dps:
        dev_imu.imu_cfg.ssvt_g = 2048;
        break;
    case gyro_range_32dps:
        dev_imu.imu_cfg.ssvt_g = 1024;
        break;
    case gyro_range_64dps:
        dev_imu.imu_cfg.ssvt_g = 512;
        break;
    case gyro_range_128dps:
        dev_imu.imu_cfg.ssvt_g = 256;
        break;
    case gyro_range_256dps:
        dev_imu.imu_cfg.ssvt_g = 128;
        break;
    case gyro_range_512dps:
        dev_imu.imu_cfg.ssvt_g = 64;
        break;
    case gyro_range_1024dps:
        dev_imu.imu_cfg.ssvt_g = 32;
        break;
    case gyro_range_2048dps:
        dev_imu.imu_cfg.ssvt_g = 16;
        break;
    default:
        range = gyro_range_512dps;
        dev_imu.imu_cfg.ssvt_g = 64;
        break;
    }
#if 0
    ESP_LOGI(TAG, "Gyroscope sensitivity: %d", dev_imu.imu_cfg.ssvt_g);
#endif
    uint8_t ctrl3_reg = ((self_test & 0x01) << 7) | range | odr;
    dev_imu_reg_write_byte(QMI8658_CTRL3, ctrl3_reg);
}

void dev_imu_filter_setting(enum qmi8658_LPF_Mode gLPF_MODE, uint8_t gLPF_EN, enum qmi8658_LPF_Mode aLPF_MODE, uint8_t aLPF_EN)
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

void dev_imu_running_mode(bool low_power_mode)
{
    dev_imu_sensors_enable(DEV_IMU_ACC_GYR_DISABLE);
    if (low_power_mode)
    {
        dev_imu_accelerator_setting(0, acc_full_scale_8g, accel_odr_LowPower_11Hz);
        dev_imu_sensors_enable(DEV_IMU_ACC_ENABLE);
    }
    else
    {
        dev_imu_accelerator_setting(0, acc_full_scale_16g, accel_odr_500Hz);
        dev_imu_gyroscope_setting(0, gyro_range_128dps, gyro_odr_500Hz);
        dev_imu_sensors_enable(DEV_IMU_ACC_GYR_ENABLE);
    }
}

static void dev_imu_read_raw_data(int16_t* acc_raw, int16_t* gyro_raw)
{
    uint8_t buf[12] = { 0 };
    esp_err_t ret = dev_imu_reg_read_byte(QMI8658_AX_L, buf, sizeof(buf));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read IMU data registers, error: %d", ret);
        return;
    }

    if (acc_raw)
    {
        acc_raw[0] = (int16_t)((buf[1] << 8) | buf[0]);
        acc_raw[1] = (int16_t)((buf[3] << 8) | buf[2]);
        acc_raw[2] = (int16_t)((buf[5] << 8) | buf[4]);
    }
    if (gyro_raw)
    {
        gyro_raw[0] = (int16_t)((buf[7] << 8) | buf[6]);
        gyro_raw[1] = (int16_t)((buf[9] << 8) | buf[8]);
        gyro_raw[2] = (int16_t)((buf[11] << 8) | buf[10]);
    }
}
void dev_imu_read_temperature(float* temperature)
{
    uint8_t temp[2] = { 0 };

    dev_imu_reg_read_byte(QMI8658_TEMP_H, temp, 2);
    short temp_raw = (temp[0] << 8) | temp[1];
    *temperature = (float)temp_raw / 256.0f;

}

void dev_imu_read_accelerometer_gyroscope(float* acc, float* gyr)
{
    short raw_acc[3], raw_gry[3];
    unsigned char axis = 0;
    static int cali_count = 0;
    static float offset_acc[3] = { 0.0f, 0.0f, 0.0f };
    static float offset_gyr[3] = { 0.0, 0.0, 0.0 };

    static float acc_calibration_sum[3] = { 0.0f, 0.0f, 0.0f };
    static float gyr_calibration_sum[3] = { 0.0f, 0.0f, 0.0f };

    float acc_raw[3];
    float gyr_raw[3];

    dev_imu_read_raw_data(raw_acc, raw_gry);

#if 0
    ESP_LOGI(TAG, "Raw acc: %d, %d, %d", raw_acc[0], raw_acc[1], raw_acc[2]);
    ESP_LOGI(TAG, "Raw gyro: %d, %d, %d", raw_gry[0], raw_gry[1], raw_gry[2]);
#endif

    acc_raw[0] = (float)(raw_acc[0] * DEV_IMU_ONE_G) / dev_imu.imu_cfg.ssvt_a;
    acc_raw[1] = (float)(raw_acc[1] * DEV_IMU_ONE_G) / dev_imu.imu_cfg.ssvt_a;
    acc_raw[2] = (float)(raw_acc[2] * DEV_IMU_ONE_G) / dev_imu.imu_cfg.ssvt_a;

    gyr_raw[0] = (float)(raw_gry[0] * DEV_IMU_M_PAI) / dev_imu.imu_cfg.ssvt_g;
    gyr_raw[1] = (float)(raw_gry[1] * DEV_IMU_M_PAI) / dev_imu.imu_cfg.ssvt_g;
    gyr_raw[2] = (float)(raw_gry[2] * DEV_IMU_M_PAI) / dev_imu.imu_cfg.ssvt_g;

#if 0
    ESP_LOGI(TAG, "Raw acc: %.3f, %.3f, %.3f", acc_raw[0], acc_raw[1], acc_raw[2]);
    ESP_LOGI(TAG, "Raw gyro: %.3f, %.3f, %.3f", gyr_raw[0], gyr_raw[1], gyr_raw[2]);
#endif

    if (imu_calibrated != 1)
    {
        if (cali_count == 0)
        {
            memset((void*)acc_calibration_sum, 0, sizeof(acc_calibration_sum));
            memset((void*)gyr_calibration_sum, 0, sizeof(gyr_calibration_sum));
            cali_count++;
        }
        else if (cali_count < MAX_CALI_COUNT)
        {
            for (axis = 0; axis < 3; axis++)
            {
                if (axis == 2)
                {
                    acc_calibration_sum[axis] += (acc_raw[axis] - DEV_IMU_ONE_G);
                }
                else
                {
                    acc_calibration_sum[axis] += acc_raw[axis];
                }
                gyr_calibration_sum[axis] += gyr_raw[axis];
            }
            cali_count++;
        }
        else if (cali_count == MAX_CALI_COUNT)
        {
            for (axis = 0; axis < 3; axis++)
            {
                offset_gyr[axis] = 0.0f - (gyr_calibration_sum[axis] / (MAX_CALI_COUNT - 1));
                offset_acc[axis] = 0.0f - (acc_calibration_sum[axis] / (MAX_CALI_COUNT - 1));
            }
            imu_calibrated = 1;
            cali_count++;
        }
    }
    else
    {
        for (axis = 0; axis < 3; axis++)
        {
            acc[axis] = acc_raw[axis] + offset_acc[axis];
            gyr[axis] = gyr_raw[axis] + offset_gyr[axis];
        }
    }
#if 0
    ESP_LOGI(TAG, "Acceleration: X=%.3f g, Y=%.3f g, Z=%.3f g", acc[0], acc[1], acc[2]);
    ESP_LOGI(TAG, "Gyro: X=%.2f °/s, Y=%.2f °/s, Z=%.2f °/s", gyr[0], gyr[1], gyr[2]);
    ESP_LOGI(TAG, "cali_count = %d", cali_count);
    ESP_LOGI(TAG, "imu_calibrated = %d", imu_calibrated);
#endif 
}

void dev_imu_read_accelerometer_gyroscope_after_judge(float* acc, float* gyr)
{
    unsigned char status = 0;
    unsigned char data_ready = 0;
    int retry = 0;
    while (retry++ < 3)
    {
        dev_imu_reg_read_byte(QMI8658_STATUS0, &status, 1);
        if (status & 0x03)
        {
            data_ready = 1;
            break;
        }
    }
    if (data_ready)
    {
        dev_imu_read_accelerometer_gyroscope(acc, gyr);
        dev_imu.acc[0] = acc[0];
        dev_imu.acc[1] = acc[1];
        dev_imu.acc[2] = acc[2];
        dev_imu.gyr[0] = gyr[0];
        dev_imu.gyr[1] = gyr[1];
        dev_imu.gyr[2] = gyr[2];
    }
    else
    {
        acc[0] = dev_imu.acc[0];
        acc[1] = dev_imu.acc[1];
        acc[2] = dev_imu.acc[2];
        gyr[0] = dev_imu.gyr[0];
        gyr[1] = dev_imu.gyr[1];
        gyr[2] = dev_imu.gyr[2];
    }
#if 0
    ESP_LOGI(TAG, "Acceleration: X=%.3f g, Y=%.3f g, Z=%.3f g", acc[0], acc[1], acc[2]);
    ESP_LOGI(TAG, "Gyro: X=%.2f °/s, Y=%.2f °/s, Z=%.2f °/s", gyr[0], gyr[1], gyr[2]);
#endif 
}

#define ACCZ_SAMPLE     80                      /* 采样数 */

static float Kp = 10.0f;                        /* 比例增益 */
static float Ki = 0.02f;                        /* 积分增益 */
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;      /* 积分误差累计 */

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;    /* 四元数 */
static float rMat[3][3];                        /* 旋转矩阵 */

static float maxError = 0.0f;                   /* 最大误差 */
static bool  isGravityCalibrated = false;       /* 是否校准完成 */
static float baseAcc[3] = { 0.0f, 0.0f, 1.0f };   /* 静态加速度 */

/* 卡尔曼滤波器变量 */
static float q_angle = 0.030f;    /* 过程噪声 */
static float q_bias = 0.003f;    /* 陀螺仪噪声 */
static float r_measure = 0.60f;     /* 测量噪声 */

static float angle = 0.0f;          /* 角度 */
static float bias = 0.0f;          /* 偏置 */
static float P[2][2] = { { 0, 0 }, { 0, 0 } };  /* 协方差矩阵 */

/**
 * @brief   开方函数
 * @param   x : 待开方的值
 * @retval  开方结果
 */
static float inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

/**
 * @brief   卡尔曼滤波算法
 * @param   newAngle : 欧拉角
 * @param   newRate : 陀螺仪测量值
 * @param   dt : 时间步长
 * @retval  无
 */
static void kalman_filter(float newAngle, float newRate, float dt)
{
    /* 预测更新 */
    angle += dt * (newRate - bias);
    P[0][0] += dt * (dt * P[1][1] + q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += q_bias * dt;

    /* 计算卡尔曼增益 */
    float S = P[0][0] + r_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    /* 更新 */
    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    /* 更新协方差矩阵 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
}

/**
 * @brief   计算旋转矩阵
 * @param   无
 * @retval  无
 */
static void compute_rotation_matrix(void)
{
    float q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
    float q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q2 = q1 * q2, q1q3 = q1 * q3, q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * (q2q2 + q3q3);
    rMat[0][1] = 2.0f * (q1q2 - q0q3);
    rMat[0][2] = 2.0f * (q1q3 + q0q2);

    rMat[1][0] = 2.0f * (q1q2 + q0q3);
    rMat[1][1] = 1.0f - 2.0f * (q1q1 + q3q3);
    rMat[1][2] = 2.0f * (q2q3 - q0q1);

    rMat[2][0] = 2.0f * (q1q3 - q0q2);
    rMat[2][1] = 2.0f * (q2q3 + q0q1);
    rMat[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);
}

/**
 * @brief   加速度校准
 * @param   acc:加速度传感器值
 * @retval  无
 */
static void calibrate_gravity(float* acc)
{
    static unsigned short cnt = 0;
    static float accZMin = 1.5f, accZMax = 0.5f;
    static float sumAcc[3] = { 0.0f };

    if (cnt == 0)
    {
        accZMin = acc[2];
        accZMax = acc[2];
        for (unsigned char i = 0; i < 3; i++)
            sumAcc[i] = 0.0f;
    }

    for (unsigned char i = 0; i < 3; i++)
        sumAcc[i] += acc[i];

    if (acc[2] < accZMin) accZMin = acc[2];
    if (acc[2] > accZMax) accZMax = acc[2];

    if (++cnt >= ACCZ_SAMPLE)
    {
        cnt = 0;
        maxError = accZMax - accZMin;
        if (maxError < 100.0f)
        {
            for (unsigned char i = 0; i < 3; i++)
                baseAcc[i] = sumAcc[i] / ACCZ_SAMPLE;
            isGravityCalibrated = true;
        }
        for (unsigned char i = 0; i < 3; i++)
            sumAcc[i] = 0.0f;
    }
}

/**
 * @brief       获取欧拉角数据
 * @note        姿态解算融合, 核心算法，互补滤波算法，卡尔曼滤波算法
 *              尽量保证该函数的调用频率为: IMU_DELTA_T , 否则YAW会相应的偏大/偏小
 * @param       gyro:3轴陀螺仪数据
 * @param       acc:3轴加速度数据
 * @param       rpy:欧拉角存放buf
 * @param       dt:调用频率
 * @retval      无
 */
void dev_imu_get_eulerian_angels(float* acc, float* gyr, float* ang, float dt)
{
    float normalise;
    float ex, ey, ez;
    float halfT = 0.5f * dt;
    float accBuf[3] = { 0.0f };

    /* 加速度计输出有效时，利用加速度计补偿陀螺仪 */
    if (acc[0] != 0.0f || acc[1] != 0.0f || acc[2] != 0.0f)
    {
        /* 单位化加速计测量值 */
        normalise = inv_sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        acc[0] *= normalise;
        acc[1] *= normalise;
        acc[2] *= normalise;

        /* 加速计读取的方向与重力加速计方向的差值，用向量叉乘计算 */
        ex = acc[1] * rMat[2][2] - acc[2] * rMat[2][1];
        ey = acc[2] * rMat[2][0] - acc[0] * rMat[2][2];
        ez = acc[0] * rMat[2][1] - acc[1] * rMat[2][0];

        /* 误差累计，与积分常数相乘 */
        exInt += Ki * ex * dt;
        eyInt += Ki * ey * dt;
        ezInt += Ki * ez * dt;

        /* 用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量 */
        gyr[0] += Kp * ex + exInt;
        gyr[1] += Kp * ey + eyInt;
        gyr[2] += Kp * ez + ezInt;
    }

    /* 一阶近似算法，四元数运动学方程的离散化形式和积分 */
    float q0Last = q0, q1Last = q1, q2Last = q2, q3Last = q3;
    q0 += (-q1Last * gyr[0] - q2Last * gyr[1] - q3Last * gyr[2]) * halfT;
    q1 += (q0Last * gyr[0] + q2Last * gyr[2] - q3Last * gyr[1]) * halfT;
    q2 += (q0Last * gyr[1] - q1Last * gyr[2] + q3Last * gyr[0]) * halfT;
    q3 += (q0Last * gyr[2] + q1Last * gyr[1] - q2Last * gyr[0]) * halfT;

    /* 单位化四元数 */
    normalise = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= normalise;
    q1 *= normalise;
    q2 *= normalise;
    q3 *= normalise;

    compute_rotation_matrix();

    /* 计算欧拉角 */
    ang[0] = asinf(rMat[2][0]) * RAD2DEG;
    ang[1] = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
    ang[2] = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;

    /* 使用卡尔曼滤波器来更新 YAW */
    kalman_filter(ang[2], gyr[2], dt);

    /* 返回经过滤波的角度 */
    ang[2] = angle;

    if (!isGravityCalibrated)
    {
        accBuf[2] = acc[0] * rMat[2][0] + acc[1] * rMat[2][1] + acc[2] * rMat[2][2];
        calibrate_gravity(accBuf);
    }
}

void i2c_imu_example_task(void* arg)
{
    bsp_i2c_init();
    dev_imu_init();

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        dev_imu_read_temperature(&dev_imu.temperature);

        dev_imu_read_accelerometer_gyroscope_after_judge(dev_imu.acc, dev_imu.gyr);
        if (imu_calibrated)
        {
            dev_imu_get_eulerian_angels(dev_imu.acc, dev_imu.gyr, dev_imu.ang, IMU_DELTA_T);
        }

#if 1
        ESP_LOGI(TAG, "Temperature = %.2f °C", dev_imu.temperature);
        if (imu_calibrated)
        {
            ESP_LOGI(TAG, "Acceleration: X=%.3f g, Y=%.3f g, Z=%.3f g",
                dev_imu.acc[0], dev_imu.acc[1], dev_imu.acc[2]);
            ESP_LOGI(TAG, "Gyro: X=%.2f °/s, Y=%.2f °/s, Z=%.2f °/s",
                dev_imu.gyr[0], dev_imu.gyr[1], dev_imu.gyr[2]);
            ESP_LOGI(TAG, "Eulerian Angles: Roll=%.2f °, Pitch=%.2f °, Yaw=%.2f °",
                dev_imu.ang[0], dev_imu.ang[1], dev_imu.ang[2]);
        }
#endif
    }
}

void aiot_exp32_c3_02_demo_i2c_imu(void)
{
    xTaskCreate(i2c_imu_example_task, "i2c_imu_example_task", 4096, NULL, 5, NULL);
}