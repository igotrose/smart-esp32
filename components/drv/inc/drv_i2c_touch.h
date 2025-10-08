#ifndef _DRV_I2C_TOUCH_H_
#define _DRV_I2C_TOUCH_H_

#include "freertos/FreeRTOS.h"
#include "drv_i2c_bus.h"
#include "drv_spi_lcd.h"
#include "esp_lcd_touch_ft5x06.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_lcd_io_i2c.h"

extern esp_lcd_touch_handle_t tp;


esp_err_t dev_i2c_touch_init(void);
void aiot_esp32_s3_07_demo_i2c_touch(void);

#endif

