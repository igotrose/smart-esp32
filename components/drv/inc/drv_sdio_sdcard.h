#ifndef _DRV_SDIO_SDCARD_H_
#define _DRV_SDIO_SDCARD_H_

#include "freertos/FreeRTOS.h"
#include "driver/sdmmc_types.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_err.h"
#include "esp_log.h"

#define MOUNT_POINT "/sdcard"

esp_err_t bsp_sdio_init(void);

void aiot_exp32_c3_03_demo_sdio_sdcard(void);
 

#endif


