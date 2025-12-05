#pragma once
#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

#define MPU9250_ADDR 0x68   // AD0 = 0. For AD0 = 1 use 0x69

esp_err_t mpu9250_init(i2c_port_t port);

esp_err_t mpu9250_read_accel(i2c_port_t port,
                             float *ax, float *ay, float *az);

esp_err_t mpu9250_read_gyro(i2c_port_t port,
                            float *gx, float *gy, float *gz);
