idf_component_register(SRCS "src/drv_gpio_key.c" 
                            "src/drv_i2c_imu.c" 
                    INCLUDE_DIRS 
                            "inc"
                    REQUIRES 
                        esp_driver_gpio
                        driver)