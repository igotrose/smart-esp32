#include "app_voice_sr.h"

static const char* TAG = "APP_VOICE_SR";

static model_iface_data_t* model_data = NULL;
static const esp_mn_iface_t* multinet = NULL;
static const esp_afe_sr_iface_t* afe_handle = NULL;
static srmodel_list_t* srmodels = NULL;

static QueueHandle_t result_queue = NULL;

const char* cmd_phoneme[] = {
    "da kai kong qi jing hua qi",
    "guan bi kong qi jing hua qi",
    "da kai tai deng",
    "guan bi tai deng",
    "tai deng tiao liang",
    "tai deng tiao an",
    "da kai deng dai",
    "guan bi deng dai",
    "bo fang yin yue",
    "ting zhi bo fang",
    "da kai shi jian",
    "da kai ri li"
};

static void app_voice_sr_feed_task(void* param)
{
    size_t bytes_read = 0;
    esp_afe_sr_data_t* afe_data = (esp_afe_sr_data_t*)param;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int channels = afe_handle->get_channel_num(afe_data);
    ESP_LOGI(TAG, "audio_chunksize = %d, feed_channel = %d", audio_chunksize, channels);

    // Allocate audio buffer and check for result 
    int16_t* audio_buffer = heap_caps_malloc(audio_chunksize * sizeof(int16_t) * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (NULL == audio_buffer)
    {
        esp_system_abort("Memory allocation failed for audio buffer");
    }
    ESP_LOGI(TAG, "Audio buffer allocated successfully");

    while (1)
    {
        dev_audio_codec_get_feed_data(false, audio_buffer, audio_chunksize * sizeof(int16_t) * channels);
        afe_handle->feed(afe_data, audio_buffer);
    }
    if (audio_buffer)
    {
        free(audio_buffer);
        audio_buffer = NULL;
    }

    vTaskDelete(NULL);
}

static void app_voice_sr_detect_task(void* param)
{
    bool detect_flag = false;
    esp_afe_sr_data_t* afe_data = (esp_afe_sr_data_t*)param;

    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    int mu_chunksize = multinet->get_samp_chunksize(model_data);

    ESP_LOGI(TAG, "------------detect start------------\n");

    while (1)
    {
        // Obtain the processed audio data collected from the acoustic front end.
        afe_fetch_result_t* res = afe_handle->fetch(afe_data);

        if (!res || res->ret_value == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to fetch data from AFE");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (res->wakeup_state == WAKENET_DETECTED)
        {
            ESP_LOGI(TAG, "Wake word detected");
            sr_result_t result = {
                .command_id = 0,
                .state = ESP_MN_STATE_DETECTING,
                .wakenet_mode = WAKENET_DETECTED,
            };
            xQueueSend(result_queue, &result, 10);
        }
        else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED)
        {
            ESP_LOGI(TAG, "Wake word channel verified");
            detect_flag = true;
            afe_handle->disable_wakenet(afe_data);
        }
        if (detect_flag)
        {
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);
            if (mn_state == ESP_MN_STATE_DETECTING)
            {
                continue;
            }
            if (mn_state == ESP_MN_STATE_DETECTED)
            {
                ESP_LOGI(TAG, "Multinet detected");
                esp_mn_results_t* mn_result = multinet->get_results(model_data);
                for (int i = 0; i < mn_result->num; i++)
                {
                    ESP_LOGI(TAG, "TOP %d, command_id: %d, phrase_id: %d, prob: %f",
                        i + 1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->prob[i]);
                }
                int sr_command_id = mn_result->command_id[0];
                ESP_LOGI(TAG, "Command ID: %d", sr_command_id);
                sr_result_t result = {
                    .wakenet_mode = WAKENET_NO_DETECT,
                    .state = mn_state,
                    .command_id = sr_command_id,
                };
                xQueueSend(result_queue, &result, 10);
                continue;
            }
            if (mn_state == ESP_MN_STATE_TIMEOUT)
            {
                ESP_LOGW(TAG, "Multinet timeout");
                sr_result_t result = {
                    .wakenet_mode = WAKENET_NO_DETECT,
                    .state = mn_state,
                    .command_id = 0,
                };
                xQueueSend(result_queue, &result, 10);
                afe_handle->enable_wakenet(afe_data);
                detect_flag = false;
                continue;
            }

            ESP_LOGE(TAG, "Exception unhandled");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    afe_handle->destroy(afe_data);

    vTaskDelete(NULL);
}

static void app_voice_sr_handler_task(void* param)
{
    while (1)
    {
        sr_result_t result;
        if (xQueueReceive(result_queue, &result, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            ESP_LOGI(TAG, "Command ID: %d, State: %d, Wakenet Mode: %d", result.command_id, result.state, result.wakenet_mode);
            if (result.state == ESP_MN_STATE_TIMEOUT)
            {
                ESP_LOGW(TAG, "Timeout");
                continue;
            }
            if (result.wakenet_mode == WAKENET_DETECTED)
            {
                ESP_LOGI(TAG, "wakenet detected");
                continue;
            }
            if (result.state & ESP_MN_STATE_DETECTED)
            {
                ESP_LOGI(TAG, "Multinet detected");
                switch (result.command_id)
                {
                case 0:
                    break;
                case 1:
                    break;
                case 2:
                    break;
                case 3:
                    break;
                case 4:
                    break;
                case 5:
                    break;
                case 6:
                    break;
                case 7:
                    break;
                case 8:
                    break;
                case 9:
                    break;
                case 10:
                    break;
                case 11:
                    break;
                case 12:
                    break;

                default:
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

}


esp_err_t app_voice_sr_init(void)
{
    // Create an identification result queue
    result_queue = xQueueCreate(1, sizeof(sr_result_t));
    ESP_RETURN_ON_FALSE(NULL != result_queue, ESP_ERR_NO_MEM, TAG, "Failed to create result queue");
    // Obtain the list of all models
    srmodels = esp_srmodel_init("model");
    // Check whether the models have been initialized successfully
    ESP_RETURN_ON_FALSE(NULL != srmodels && srmodels->num > 0, ESP_ERR_INVALID_STATE, TAG, "Failed to load srmodels");
#if 0
    for (int i = 0; i < srmodels->num; i++)
    {
        ESP_LOGI(TAG, "Loaded model: %s", srmodels->model_name[i]);
    }
#endif
    // Initialize an acoustic front-end handle and the configuration structure of the AFE using the default settings.
    afe_config_t* afe_config = afe_config_init("MM", srmodels, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    afe_config_print(afe_config);
    afe_handle = esp_afe_handle_from_config(afe_config);
    // Filter out the names of the previously configured wake-up word models from the model queue
    afe_config->wakenet_model_name = esp_srmodel_filter(srmodels, ESP_WN_PREFIX, NULL);
    afe_config->aec_init = true;
    // VAD (Voice Activity Detection): Optimizing parameters to filter out short-term noise
    afe_config->vad_init = true;
    afe_config->vad_mode = VAD_MODE_2;
    afe_config->vad_model_name = NULL;
    afe_config->vad_min_speech_ms = 200;
    afe_config->vad_min_noise_ms = 800;
    afe_config->vad_delay_ms = 128;
    afe_config->vad_mute_playback = false;
    afe_config->vad_enable_channel_trigger = false;
    // AGC (Automatic Gain Control): Balances volume and prevents noise amplification
    afe_config->agc_init = true;
    afe_config->agc_mode = AFE_AGC_MODE_WAKENET;
    afe_config->agc_compression_gain_db = 10;
    afe_config->agc_target_level_dbfs = 8;
    afe_config->afe_ringbuf_size = 100;
    // Using this configuration structure of AFE to create the AFE data object "afe_data" enables the storage of all the status information related to AFE.
    esp_afe_sr_data_t* afe_data = afe_handle->create_from_config(afe_config);
    ESP_LOGI(TAG, "load wakenet: %s", afe_config->wakenet_model_name);

    // Loading the wake-up word model
#if 0
    for (int i = 0; i < srmodels->num; i++)
    {
        ESP_LOGI(TAG, "Current Model: %s", srmodels->model_name[i]);
    }
#endif
    char* mn_name = esp_srmodel_filter(srmodels, ESP_MN_CHINESE, NULL);
    if (mn_name == NULL)
    {
        ESP_LOGE(TAG, "No multinet model found");
        return ESP_FAIL;
    }
    multinet = esp_mn_handle_from_name(mn_name);
    model_data = multinet->create(mn_name, 5760);
    ESP_LOGI(TAG, "load multinet: %s", mn_name);
    // Loading of command word model
    esp_mn_commands_clear();

    for (int i = 0; i < sizeof(cmd_phoneme) / sizeof(cmd_phoneme[0]); i++)
    {
        esp_mn_commands_add(i, (char*)cmd_phoneme[i]);
    }
    esp_mn_commands_update();
#if 0
    esp_mn_commands_print();
    multinet->print_active_speech_commands(model_data);
#endif
    // Registration of voice command completed

    BaseType_t ret_vel = xTaskCreatePinnedToCore(app_voice_sr_feed_task, "Feed Task", 4 * 1024, afe_data, 5, NULL, 1);
    ESP_RETURN_ON_FALSE(ret_vel == pdPASS, ESP_FAIL, TAG, "Failed to create feed task");

    ret_vel = xTaskCreatePinnedToCore(app_voice_sr_detect_task, "Detect Task", 6 * 1024, afe_data, 4, NULL, 0);
    ESP_RETURN_ON_FALSE(ret_vel == pdPASS, ESP_FAIL, TAG, "Failed to create detect task");

    ret_vel = xTaskCreatePinnedToCore(app_voice_sr_handler_task, "Handler Task", 4 * 1024, NULL, 3, NULL, 1);
    ESP_RETURN_ON_FALSE(ret_vel == pdPASS, ESP_FAIL, TAG, "Failed to create handler task");

    return ESP_OK;
}