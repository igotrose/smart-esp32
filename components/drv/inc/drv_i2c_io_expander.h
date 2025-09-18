#ifndef __DRV_I2C_IO_EXPANDER_H__
#define __DRV_I2C_IO_EXPANDER_H__

#include "freertos/FreeRTOS.h"
#include "drv_i2c_bus.h"
#include "driver/i2c_master.h"

#define DEV_IO_EXPANDER_ADDR                    0x19

#define DEV_IO_EXPANDER_INPUT_PORT              0x00
#define DEV_IO_EXPANDER_OUTPUT_PORT             0x01
#define DEV_IO_EXPANDER_POLARITY_INVERSION      0x02
#define DEV_IO_EXPANDER_CONFIG                  0x03

#define IO_EXPANDER_IO0                         0x01
#define IO_EXPANDER_IO1                         0x02
#define IO_EXPANDER_IO2                         0x04
#define IO_EXPANDER_IO3                         0x08
#define IO_EXPANDER_IO4                         0x10
#define IO_EXPANDER_IO5                         0x20
#define IO_EXPANDER_IO6                         0x40
#define IO_EXPANDER_IO7                         0x80

#define LCD_CS                                  IO_EXPANDER_IO0
#define PA_EN                                   IO_EXPANDER_IO1  
#define DVP_PWDN                                IO_EXPANDER_IO2

esp_err_t dev_io_expander_init(void);
esp_err_t dev_io_expander_get_mode(uint8_t pin, uint8_t* mode);
esp_err_t dev_io_expander_set_mode(uint8_t pin, uint8_t mode);
esp_err_t dev_io_expander_get_input_value(uint8_t pin, uint8_t* pin_value);
esp_err_t dev_io_expander_set_output_value(uint8_t pin, uint8_t pin_value);
esp_err_t dev_io_expander_get_output_value(uint8_t pin, uint8_t* pin_value);

void aiot_esp32_s3_04_demo_io_expander(void);



#endif
