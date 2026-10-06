#include "mpu6050.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_WHO_AM_I_VALUE   0x68
#define MPU6050_ACCEL_LSB_PER_G  16384.0f
#define I2C_TIMEOUT_MS           1000

static i2c_master_dev_handle_t s_mpu6050_device;

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(s_mpu6050_device, data, sizeof(data), I2C_TIMEOUT_MS);
}

static esp_err_t read_registers(uint8_t first_reg, uint8_t *data, size_t length)
{
    return i2c_master_transmit_receive(s_mpu6050_device,
                                       &first_reg,
                                       sizeof(first_reg),
                                       data,
                                       length,
                                       I2C_TIMEOUT_MS);
}

esp_err_t mpu6050_init(i2c_master_bus_handle_t bus, uint8_t address)
{
    if (bus == NULL || address > 0x7F) {
        return ESP_ERR_INVALID_ARG;
    }

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 400000,
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &device_config, &s_mpu6050_device);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t who_am_i = 0;
    err = read_registers(MPU6050_REG_WHO_AM_I, &who_am_i, sizeof(who_am_i));
    if (err != ESP_OK) {
        return err;
    }
    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Wake the sensor, enable 260 Hz bandwidth, and select a 1 kHz sample rate. */
    err = write_register(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    err = write_register(MPU6050_REG_CONFIG, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    err = write_register(MPU6050_REG_SMPLRT_DIV, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    /* Keep the +/-2 g range: 16384 LSB/g. */
    return write_register(MPU6050_REG_ACCEL_CONFIG, 0x00);
}

esp_err_t mpu6050_read_acceleration(mpu6050_acceleration_t *acceleration)
{
    if (acceleration == NULL || s_mpu6050_device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[6];
    esp_err_t err = read_registers(MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    acceleration->x_raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    acceleration->y_raw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    acceleration->z_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

    acceleration->x_g = acceleration->x_raw / MPU6050_ACCEL_LSB_PER_G;
    acceleration->y_g = acceleration->y_raw / MPU6050_ACCEL_LSB_PER_G;
    acceleration->z_g = acceleration->z_raw / MPU6050_ACCEL_LSB_PER_G;

    return ESP_OK;
}
