#include "drv_dvp_cam.h"

static char* TAG = "DEV_DVP_CAM";
static camera_config_t camera_config = { 0 };
QueueHandle_t cam_frame_queue = NULL;

esp_err_t dev_dvp_cam_init(void)
{
    esp_err_t ret;

    ret = dev_io_expander_set_output_value(DVP_PWDN, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "camera power on failed");
        return ret;
    }

    camera_config.pin_d0 = CAM_PIN_D0;
    camera_config.pin_d1 = CAM_PIN_D1;
    camera_config.pin_d2 = CAM_PIN_D2;
    camera_config.pin_d3 = CAM_PIN_D3;
    camera_config.pin_d4 = CAM_PIN_D4;
    camera_config.pin_d5 = CAM_PIN_D5;
    camera_config.pin_d6 = CAM_PIN_D6;
    camera_config.pin_d7 = CAM_PIN_D7;

    camera_config.pin_sccb_sda = -1;
    camera_config.pin_sccb_scl = CAM_PIN_SIOC;
    camera_config.sccb_i2c_port = CAM_I2C_PORT;

    camera_config.pin_xclk = CAM_PIN_XCLK;
    camera_config.pin_pclk = CAM_PIN_PCLK;
    camera_config.pin_vsync = CAM_PIN_VSYNC;
    camera_config.pin_href = CAM_PIN_HREF;
    camera_config.pin_pwdn = -1;
    camera_config.pin_reset = -1;

    camera_config.xclk_freq_hz = 20000000;
    camera_config.ledc_channel = LEDC_CHANNEL_1;
    camera_config.ledc_timer = LEDC_TIMER_0;

    camera_config.pixel_format = PIXFORMAT_RGB565;
    camera_config.frame_size = FRAMESIZE_QVGA;
    camera_config.jpeg_quality = 12;
    camera_config.fb_count = 2;
    camera_config.fb_location = CAMERA_FB_IN_PSRAM;
    camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    ret = esp_camera_init(&camera_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "camera init failed");
        return ret; 
    }

    return ESP_OK;
}

static void demo_cam_task(void* arg)
{
    camera_fb_t* fb = NULL;
    while (1)
    {
        fb = esp_camera_fb_get();
        if (fb)
        {
            xQueueSend(cam_frame_queue, &fb, portMAX_DELAY);
        }
    }

}

static void demo_lcd_task(void* arg)
{
    camera_fb_t* fb = NULL;
    while (1)
    {
        if (xQueueReceive(cam_frame_queue, &fb, portMAX_DELAY) == pdTRUE)
        {
            esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, fb->width, fb->height, (uint16_t*)fb->buf);
            esp_camera_fb_return(fb);
        }
    }
}

void aiot_esp32_s3_06_demo_dvp_cam(void)
{
    cam_frame_queue = xQueueCreate(10, sizeof(camera_fb_t));
    xTaskCreatePinnedToCore(demo_cam_task, "demo_cam_task", 3 * 1024, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(demo_lcd_task, "demo_lcd_task", 4 * 1024, NULL, 5, NULL, 0);
}

