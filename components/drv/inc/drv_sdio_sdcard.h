#ifndef _DRV_SDIO_SDCARD_H_
#define _DRV_SDIO_SDCARD_H_

#include "freertos/FreeRTOS.h"
#include "driver/sdmmc_types.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_err.h"
#include "esp_log.h"

#include "app_storage_nvs.h"

#include <string.h>

#define MOUNT_POINT "/sdcard"

#define BSP_SD_CLK          GPIO_NUM_47
#define BSP_SD_CMD          GPIO_NUM_48 
#define BSP_SD_DAT0         GPIO_NUM_21


esp_err_t dev_sdio_sdcard_init(void);

esp_err_t dev_sdcard_write_file(const char* path, char* data);
esp_err_t dev_sdcard_read_file(const char* path);
esp_err_t dev_sdcard_get_status(sdmmc_card_t* card);

esp_err_t dev_sdcard_umount(void);

void aiot_esp32_s3_03_demo_sdio_sdcard(void);


#endif


