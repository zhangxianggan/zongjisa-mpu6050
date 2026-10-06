#pragma once

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    float x_g;
    float y_g;
    float z_g;
} mpu6050_acceleration_t;

/** Initialize an MPU6050 on an existing I2C master bus. */
esp_err_t mpu6050_init(i2c_master_bus_handle_t bus, uint8_t address);

/** Read the three accelerometer axes, with acceleration also converted to g. */
esp_err_t mpu6050_read_acceleration(mpu6050_acceleration_t *acceleration);
