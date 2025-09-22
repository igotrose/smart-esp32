#ifndef _DRV_I2C_BUS_H_
#define _DRV_I2C_BUS_H_

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include <stdio.h>

#define BSP_I2C_SDA         (GPIO_NUM_1)
#define BSP_I2C_SCL         (GPIO_NUM_2)
#define BSP_I2C_FREQ_HZ     (100000)
#define BSP_I2C_NUM         (I2C_NUM_0)

extern i2c_master_bus_handle_t  i2c_bus_handle;

esp_err_t dev_i2c_bus_init(void);
esp_err_t dev_i2c_bus_add_devices(uint8_t dev_addr, i2c_master_dev_handle_t* handle);
esp_err_t dev_i2c_bus_remove_devices(i2c_master_dev_handle_t* handle);
esp_err_t dev_i2c_bus_deinit(void);

#endif 


