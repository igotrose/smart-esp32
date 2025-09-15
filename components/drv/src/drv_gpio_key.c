#include "drv_gpio_key.h"

QueueHandle_t gpio_evt_queue;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t io_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &io_num, NULL);
}

static void gpio_task_example(void* parameters)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY) != pdFALSE)
        {
            printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

void aiot_esp32_s3_01_demo_gpio(void)
{
    gpio_config_t io0_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1 << GPIO_NUM_0,
        .pull_down_en = 0,
        .pull_up_en = 1,
    };

    gpio_config(&io0_conf);

    gpio_evt_queue = xQueueCreate(5, sizeof(uint32_t));
    xTaskCreate(gpio_task_example, "gpio_key", 2048, NULL, 1, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

    gpio_isr_handler_add(GPIO_NUM_0, gpio_isr_handler, (void*)GPIO_NUM_0);

}
