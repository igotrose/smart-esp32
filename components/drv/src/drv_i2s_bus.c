#include "drv_i2s_bus.h"

static char* TAG = "DRV_I2S_BUS";

i2s_chan_handle_t i2s_tx_handle = NULL;
i2s_chan_handle_t i2s_rx_handle = NULL;

esp_err_t dev_i2s_bus_init(void)
{
    esp_err_t ret;

    if (i2s_tx_handle && i2s_rx_handle)
    {
        ESP_LOGI(TAG, "i2s bus has been initialized");
        return ESP_OK;
    }


    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ret = i2s_new_channel(&chan_cfg, &i2s_tx_handle, &i2s_rx_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2s_new_channel error, ret: %d", ret);
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .bclk = BSP_I2S_BCK,
            .mclk = BSP_I2S_MCK,
            .ws = BSP_I2S_WS,
            .din = BSP_I2S_DI,
            .dout = BSP_I2S_DO,
        },
    };

    if (i2s_tx_handle != NULL)
    {
        ret = i2s_channel_init_std_mode(i2s_tx_handle, &std_cfg);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "i2s_channel_init_std_mode error, ret: %d", ret);
            return ret;
        }
        ret = i2s_channel_enable(i2s_tx_handle);
        {
            ESP_LOGE(TAG, "i2s_channel_enable error, ret: %d", ret);
            return ret;
        }
    }
    if (i2s_rx_handle != NULL)
    {
        ret = i2s_channel_init_std_mode(i2s_rx_handle, &std_cfg);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "i2s_channel_init_std_mode error, ret: %d", ret);
            return ret;
        }
        ret = i2s_channel_enable(i2s_rx_handle);
        {
            ESP_LOGE(TAG, "i2s_channel_enable error, ret: %d", ret);
            return ret;
        }
    }




    
    return ESP_OK;
}

void dev_i2s_bus_deinit(void)
{
    i2s_channel_disable(i2s_tx_handle);
    i2s_channel_disable(i2s_rx_handle);
    i2s_del_channel(i2s_tx_handle);
    i2s_del_channel(i2s_rx_handle);
}