idf_component_register(SRCS "src/drv_gpio_key.c" 
                            "src/drv_i2c_imu.c"
                            "src/drv_sdio_sdcard.c" 
                    INCLUDE_DIRS 
                            "inc"
                    REQUIRES 
                        esp_driver_gpio
                        esp_driver_sdmmc                        
                        sdmmc
                        fatfs
                        driver)