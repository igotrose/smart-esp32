#include "drv_i2s_audio.h"

static char* TAG = "DRV_I2S_AUDIO";

i2s_chan_handle_t i2s_tx_handle = NULL;
i2s_chan_handle_t i2s_rx_handle = NULL;
const audio_codec_data_if_t* i2s_data_if = NULL;

esp_codec_dev_handle_t dev_speaker_hanlde = NULL;
esp_codec_dev_handle_t dev_microphone_hanlde = NULL;

esp_err_t dev_i2s_bus_init(void)
{
    esp_err_t ret;

    if (i2s_tx_handle && i2s_rx_handle)
    {
        ESP_LOGI(TAG, "i2s bus has been initialized");
        return ESP_OK;
    }
    i2s_slot_mode_t channel_fmt = I2S_SLOT_MODE_MONO;
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
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, channel_fmt),
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
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "i2s_channel_enable error, ret: %d, line: %d", ret, __LINE__);
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
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "i2s_channel_enable error, ret: %d, line: %d", ret, __LINE__);
            return ret;
        }
    }

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = BSP_I2S_NUM,
        .rx_handle = i2s_rx_handle,
        .tx_handle = i2s_tx_handle,
    };

    i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (i2s_data_if == NULL)
    {
        ESP_LOGE(TAG, "");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "i2s bus init success");
    return ESP_OK;
}

void dev_i2s_bus_deinit(void)
{
    i2s_channel_disable(i2s_tx_handle);
    i2s_channel_disable(i2s_rx_handle);
    i2s_del_channel(i2s_tx_handle);
    i2s_del_channel(i2s_rx_handle);
}

esp_codec_dev_handle_t dev_audio_codec_speaker_init(void)
{
    const audio_codec_gpio_if_t* gpio_if = audio_codec_new_gpio();
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = (ES8311_ADDRRES_0 << 1),
        .bus_handle = i2c_bus_handle,
    };

    const audio_codec_ctrl_if_t* i2c_cfg_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (i2c_cfg_if == NULL)
    {
        ESP_LOGE(TAG, "audio_codec_new_i2c_ctrl error, line: %d", __LINE__);
        return NULL;
    }

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };

    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = i2c_cfg_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = GPIO_NUM_NC,
        .use_mclk = false,
        .hw_gain = gain,
    };

    const audio_codec_if_t* es8311_if = es8311_codec_new(&es8311_cfg);
    if (es8311_if == NULL)
    {
        ESP_LOGE(TAG, "es8311_codec_new error");
        return NULL;
    }

    esp_codec_dev_cfg_t codec_es8311_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = es8311_if,
        .data_if = i2s_data_if,
    };

    return esp_codec_dev_new(&codec_es8311_dev_cfg);
}

esp_codec_dev_handle_t dev_audio_codec_microphone_init(void)
{
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = (ES7210_ADDRESS_01 << 1),
        .bus_handle = i2c_bus_handle,
    };

    const audio_codec_ctrl_if_t* i2c_cfg_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (i2c_cfg_if == NULL)
    {
        ESP_LOGE(TAG, "audio_codec_new_i2c_ctrl error, line: %d", __LINE__);
        return NULL;
    }

    es7210_codec_cfg_t es7210_cfg = {
        .ctrl_if = i2c_cfg_if,
        .mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC2 | ES7210_SEL_MIC3,
    };

    const audio_codec_if_t* es7310_dev = es7210_codec_new(&es7210_cfg);
    if (es7310_dev == NULL)
    {
        ESP_LOGE(TAG, "es7210_codec_new error");
        return NULL;
    }

    esp_codec_dev_cfg_t codec_es7210_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = es7310_dev,
        .data_if = i2s_data_if,
    };

    return esp_codec_dev_new(&codec_es7210_dev_cfg);
}

esp_err_t dev_audio_codec_init(void)
{
    esp_err_t ret = ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(10));
    dev_speaker_hanlde = dev_audio_codec_speaker_init();
    if (dev_speaker_hanlde == NULL)
    {
        ESP_LOGE(TAG, "audio codec speaker init error");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    dev_microphone_hanlde = dev_audio_codec_microphone_init();
    if (dev_microphone_hanlde == NULL)
    {
        ESP_LOGE(TAG, "audio codec microphone init error");
        return ret;
    }

    ret = dev_audio_codec_set_fs(CODEC_DEFAULT_SAMPLE_RATE, CODEC_DEFAULT_BIT_WIDTH, CODEC_DEFAULT_CHANNEL);
    
    return ret;
}

esp_err_t dev_audio_codec_set_fs(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;

    esp_codec_dev_sample_info_t fs = {
        .sample_rate = rate,
        .channel = ch,
        .bits_per_sample = bits_cfg,
    };

    if (dev_speaker_hanlde)
    {
        ret = esp_codec_dev_close(dev_speaker_hanlde);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_codec_dev_close error, ret: %d", ret);
            return ret;
        }
    }

    if (dev_speaker_hanlde)
    {
        ret = esp_codec_dev_open(dev_speaker_hanlde, &fs);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_codec_dev_open error, ret: %d", ret);
            return ret;
        }
    }

    return ESP_OK;
}   

esp_err_t dev_audio_codec_mute_set(bool enable)
{
    esp_err_t ret = ESP_OK;
    ret = esp_codec_dev_set_out_mute(dev_speaker_hanlde, enable);
    return ret;
}

esp_err_t dev_audio_codec_volume_get(int* volume)
{
    esp_err_t ret = ESP_OK;
    ret = esp_codec_dev_get_out_vol(dev_speaker_hanlde, volume);
    return ret;
}

esp_err_t dev_audio_codec_volume_set(int volume, int* volume_set)
{
    esp_err_t ret = ESP_OK;
    float vol = (float)volume / 100.0;
    ret = esp_codec_dev_set_out_vol(dev_speaker_hanlde, vol);
    return ret;
}

esp_err_t dev_audio_codec_play(const int16_t* data, int length, TickType_t ticks_to_wait)
{
    size_t bytes_write = 0;
    esp_err_t ret = ESP_OK;
    if (!dev_speaker_hanlde)
    {
        return ESP_FAIL;
    }
    ret = esp_codec_dev_write(dev_speaker_hanlde, (void*)data, length);

    return ret;
}

esp_err_t dev_audio_codec_get_feed_data(bool is_get_raw_channel, int16_t* buffer, int buffer_len)
{
    esp_err_t ret = ESP_OK;
    size_t bytes_read;
    int audio_chunksize = buffer_len / (sizeof(uint16_t) * ADC_I2S_CHANNEL_NUM);
    ret = esp_codec_dev_read(dev_microphone_hanlde, (void*)buffer, buffer_len);
    if (!is_get_raw_channel)
    {
        for (int i = 0; i < audio_chunksize; i++)
        {
            int16_t ref = buffer[i * ADC_I2S_CHANNEL_NUM + 0];
            buffer[(ADC_I2S_CHANNEL_NUM - 1) * i + 0] = buffer[i * ADC_I2S_CHANNEL_NUM + 1];
            buffer[(ADC_I2S_CHANNEL_NUM - 1) * i + 1] = buffer[i * ADC_I2S_CHANNEL_NUM + 3];
            buffer[(ADC_I2S_CHANNEL_NUM - 1) * i + 2] = ref;
        }
    }

    return ret;
}

char* dev_audio_codec_get_input_format(void)
{
    return "RMNM";
}

void aiot_esp32_s3_08_demo_i2s_audio(void)
{

}