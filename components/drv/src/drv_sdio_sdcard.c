#include "drv_sdio_sdcard.h"

#define TAG "DEV_SDCARD"
esp_err_t bsp_sdio_init(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .allocation_unit_size = 16 * 1024,
        .format_if_mount_failed = false,
        .max_files = 5,
    };
    sdmmc_card_t* card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Using SDMMC peripheral");

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = BSP_SD_CLK;
    slot_config.cmd = BSP_SD_CMD;
    slot_config.d0 = BSP_SD_DATA0;
    slot_config.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ESP_LOGI(TAG, "Mounting filesystem");
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));

        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    sdmmc_card_print_info(stdout, card);
    

}

void sdio_sdcard_example_task(void* arg)
{
    while (1)
    {

    }
}



void aiot_exp32_c3_03_demo_sdio_sdcard(void)
{
    xTaskCreate(sdio_sdcard_example_task, "sdio_sdcard_example_task", 4096, NULL, 5, NULL);
}
