idf_component_register(SRCS "src/app_ui_lvgl.c"
                            "src/app_nvs_storage.c"
                    INCLUDE_DIRS "inc"
                    REQUIRES 
                    drv 
                    lvgl__lvgl
                    espressif__esp_lvgl_port
                    nvs_flash
                    )