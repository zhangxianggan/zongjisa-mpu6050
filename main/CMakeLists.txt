idf_component_register(SRCS "main.c" "mpu6050.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_i2c esp_timer freertos)
