#ifndef __APP_STORAGE_NVS_H__
#define __APP_STORAGE_NVS_H__

#include "nvs_flash.h"
#include "esp_log.h"

void app_storage_nvs_init(void);
bool app_storage_nvs_is_sdcard_initialized(void);
esp_err_t app_storage_nvs_set_sdcard_initialized(bool is_initialized);

#endif /* __APP_NVS_STORAGE_H__ */