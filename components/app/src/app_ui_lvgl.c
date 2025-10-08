#include "app_ui_lvgl.h"

static lv_disp_t* disp = NULL;
static lv_indev_t* indev = NULL;
static char* TAG = "APP_UI_LVGL";

void app_ui_lvgl_init(void)
{
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    ESP_LOGI(TAG, "Initializing display");
    const lvgl_port_display_cfg_t display_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = DEV_LCD_H_RES * DEV_LCD_DRAW_BUF_HEIGHT,
        .double_buffer = false,
        .hres = DEV_LCD_H_RES,
        .vres = DEV_LCD_V_RES,
        .monochrome = false,
        .rotation = {
            .swap_xy = true,
            .mirror_x = true,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
        },
    };

    disp = lvgl_port_add_disp(&display_cfg);

    lv_coord_t hor_res = DEV_LCD_H_RES;
    lv_coord_t ver_res = DEV_LCD_V_RES;
    

    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = tp,
        .scale = {
            .x = 0,
            .y = 0,
        },
    };

    indev = lvgl_port_add_touch(&touch_cfg);
}

void aiot_esp32_s3_09_demo_lvgl(void)
{
    // lv_demo_benchmark();
    // lv_demo_stress();
    lv_demo_widgets();
}
