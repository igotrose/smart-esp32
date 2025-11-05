#include "drv_sdio_sdcard.h"

#define TAG "DEV_SDCARD"
#define MAX_CHAR_SIZE 64

sdmmc_card_t* card;
static uint8_t sdmmc_mount_flag = 0x00;

esp_err_t dev_sdio_sdcard_init(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .allocation_unit_size = 16 * 1024,
        .format_if_mount_failed = false,
        .max_files = 5,
    };
    
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Using SDMMC peripheral");

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = BSP_SD_CLK;
    slot_config.cmd = BSP_SD_CMD;
    slot_config.d0 = BSP_SD_DAT0;
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
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Filesystem mounted");

    sdmmc_card_print_info(stdout, card);
    sdmmc_mount_flag = 0x01;
    
    return ESP_OK;
}

esp_err_t dev_sdcard_write_file(const char* path, char* data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE* f = fopen(path, "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);

    ESP_LOGI(TAG, "File written");
    return ESP_OK;
}

esp_err_t dev_sdcard_read_file(const char* path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos)
    {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    return ESP_OK;
}

esp_err_t dev_sdcard_get_status(sdmmc_card_t* card)
{
    ESP_LOGI(TAG, "Card status:");
    sdmmc_get_status(card);
    return ESP_OK;
}

esp_err_t dev_sdcard_umount(void)
{
    ESP_LOGI(TAG, "Card unmounted");
    if (sdmmc_mount_flag)
    {
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        sdmmc_mount_flag = 0x00;
    }
    return ESP_OK;
}
void aiot_esp32_s3_03_demo_sdio_sdcard(void)
{
    dev_sdcard_write_file("/sdcard/test.txt", "Hello, world!\n");
    dev_sdcard_read_file("/sdcard/test.txt");
    dev_sdcard_get_status(card);
    dev_sdcard_umount();
}