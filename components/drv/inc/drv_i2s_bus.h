#ifndef __DRV_I2S_BUS_H__
#define __DRV_I2S_BUS_H__

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define BSP_I2S_NUM         I2S_NUM_1
#define BSP_I2S_MCK         GPIO_NUM_38
#define BSP_I2S_BCK         GPIO_NUM_14
#define BSP_I2S_WS          GPIO_NUM_12
#define BSP_I2S_DI          GPIO_NUM_12
#define BSP_I2S_DO          GPIO_NUM_45 

extern i2s_chan_handle_t i2s_tx_handle;
extern i2s_chan_handle_t i2s_rx_handle;

esp_err_t dev_i2s_bus_init(void);
void dev_i2s_bus_deinit(void);


#endif


