#ifndef __APP_NVS_STORAGE_H__
#define __APP_NVS_STORAGE_H__

#include "nvs_flash.h"
#include "esp_log.h"

void app_nvs_storage_init(void);
bool app_nvs_storage_is_sdcard_initialized(void);
esp_err_t app_nvs_storage_set_sdcard_initialized(bool is_initialized);

#endif /* __APP_NVS_STORAGE_H__ */