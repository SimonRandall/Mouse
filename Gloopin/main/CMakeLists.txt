idf_component_register(SRCS "hello_world_main.c" "mpu9250_simple.c"
                       PRIV_REQUIRES spi_flash driver esp_driver_i2c esp_driver_ledc
                       INCLUDE_DIRS "")
