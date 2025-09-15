#ifndef __DRV_GPIO_KEY_H__
#define __DRV_GPIO_KEY_H__

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

extern QueueHandle_t gpio_evt_queue;

void aiot_esp32_s3_01_demo_gpio(void);

#endif /* __DRV_GPIO_KEY_H__ */