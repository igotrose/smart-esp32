#ifndef __DRV_DVP_CAM_H__
#define __DRV_DVP_CAM_H__

#include "esp_camera.h"
#include "drv_i2c_bus.h"
#include "drv_i2c_io_expander.h"
#include "drv_spi_lcd.h"

#define CAM_PIN_PWDN        -1 //power down is not used
#define CAM_PIN_RESET       -1 //software reset will be performed
#define CAM_PIN_XCLK        GPIO_NUM_5
#define CAM_PIN_SIOD        BSP_I2C_SDA
#define CAM_PIN_SIOC        BSP_I2C_SCL
#define CAM_I2C_PORT        BSP_I2C_NUM

#define CAM_PIN_D7          GPIO_NUM_9
#define CAM_PIN_D6          GPIO_NUM_4
#define CAM_PIN_D5          GPIO_NUM_6
#define CAM_PIN_D4          GPIO_NUM_15
#define CAM_PIN_D3          GPIO_NUM_17
#define CAM_PIN_D2          GPIO_NUM_8
#define CAM_PIN_D1          GPIO_NUM_18
#define CAM_PIN_D0          GPIO_NUM_16
#define CAM_PIN_VSYNC       GPIO_NUM_3
#define CAM_PIN_HREF        GPIO_NUM_46
#define CAM_PIN_PCLK        GPIO_NUM_7

esp_err_t dev_dvp_cam_init(void);
void aiot_esp32_s3_06_demo_dvp_cam(void);

#endif


