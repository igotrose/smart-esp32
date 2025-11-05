#ifndef __DRV_I2S_AUDIO_H__
#define __DRV_I2S_AUDIO_H__

#include "freertos/FreeRTOS.h"
#include "driver/i2s_std.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "drv_i2c_bus.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "es8311.h"
#include "es7210.h"

#include "drv_sdio_sdcard.h"

#define BSP_I2S_NUM         I2S_NUM_1
#define BSP_I2S_MCK         GPIO_NUM_38
#define BSP_I2S_BCK         GPIO_NUM_14
#define BSP_I2S_WS          GPIO_NUM_13
#define BSP_I2S_DI          GPIO_NUM_12
#define BSP_I2S_DO          GPIO_NUM_45 

#define CODEC_DEFAULT_SAMPLE_RATE           (16000)
#define CODEC_DEFAULT_BIT_WIDTH             (16)
#define CODEC_DEFAULT_ADC_VOLUME            (24.0)
#define CODEC_DEFAULT_CHANNEL               (2)
#define CODEC_DEFAULT_CHANNEL_FORMAT        I2S_SLOT_MODE_MONO

#define ADC_I2S_CHANNEL_NUM                  (4)

extern i2s_chan_handle_t i2s_tx_handle;
extern i2s_chan_handle_t i2s_rx_handle;
extern const audio_codec_data_if_t* i2s_data_if;

esp_err_t dev_i2s_bus_init(void);
void dev_i2s_bus_deinit(void);

esp_codec_dev_handle_t dev_audio_codec_microphone_init(void);
esp_codec_dev_handle_t dev_audio_codec_speaker_init(void);
esp_err_t dev_audio_codec_set_fs(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch);
esp_err_t dev_audio_codec_mute_set(bool enable);
esp_err_t dev_audio_codec_volume_set(int volume, int* volume_set);
esp_err_t dev_audio_codec_volume_get(int* volume);
esp_err_t dev_audio_codec_play(const int16_t* data, int length, TickType_t ticks_to_wait);
esp_err_t dev_audio_codec_get_feed_data(bool is_get_raw_channel, int16_t* buffer, int buffer_len);
char* dev_audio_codec_get_input_format(void);

esp_err_t dev_audio_codec_init(void);

void aiot_esp32_s3_08_demo_i2s_audio(void);

#endif


