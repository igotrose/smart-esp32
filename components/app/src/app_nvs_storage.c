#include "app_nvs_storage.h"

static const char* TAG = "APP_NVS_STORAGE";

void app_nvs_storage_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS partition is full or version mismatch, erasing and reinitializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS initialized successfully");
}

bool app_nvs_storage_is_sdcard_initialized(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS handle. Error code: %d", err);
        return false;
    }

    uint8_t sdcard_initialized = 0;
    err = nvs_get_u8(handle, "sdcard_initialized", &sdcard_initialized);
    switch (err)
    {
    case ESP_OK:
        ESP_LOGI(TAG, "SD Card Initialized value is: %d", sdcard_initialized);
        nvs_close(handle);
        return sdcard_initialized != 0;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGI(TAG, "The value is not initialized yet!");
        nvs_close(handle);
        return false;
    default:
        ESP_LOGE(TAG, "Error getting sdcard_initialized value from NVS. Error code: %d", err);
        nvs_close(handle);
        return false;
    }
}

esp_err_t app_nvs_storage_set_sdcard_initialized(bool is_initialized)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS handle. Error code: %d", err);
        return err;
    }

    uint8_t value = is_initialized ? 1 : 0;
    err = nvs_set_u8(handle, "sdcard_initialized", value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting sdcard_initialized value in NVS. Error code: %d", err);
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error committing NVS changes. Error code: %d", err);
    }
    else
    {
        ESP_LOGI(TAG, "SD Card Initialized value set to: %d", is_initialized);
    }

    nvs_close(handle);
    return ESP_OK;
}
