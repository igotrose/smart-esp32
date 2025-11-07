idf_component_register(SRCS "src/app_ui_lvgl.c"
                            "src/app_storage_nvs.c"
                            "src/app_voice_sr.c"
                    INCLUDE_DIRS "inc"
                    REQUIRES 
                    drv 
                    lvgl__lvgl
                    espressif__esp_lvgl_port
                    espressif__esp-sr
                    nvs_flash
                    )