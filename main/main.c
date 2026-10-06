#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "mpu6050.h"

#define I2C_SDA_GPIO GPIO_NUM_8
#define I2C_SCL_GPIO GPIO_NUM_9
#define MPU6050_I2C_ADDRESS 0x68

#define SAMPLE_RATE_HZ 1000
#define SAMPLE_PERIOD_US (1000000 / SAMPLE_RATE_HZ)
#define WINDOW_SIZE 100       // 100 ms of samples for carrier-frequency estimate
#define ANALYSIS_HOP_SIZE 20  // Update output and on/off state every 20 ms

#define HP_FILTER_ALPHA 0.7616f // First-order high-pass, approximately 50 Hz at 1 kHz
#define VIBRATION_ON_RMS_G 0.015f
#define VIBRATION_OFF_RMS_G 0.010f
#define VIBRATION_MIN_HZ 75.0f  // Small tolerance around the requested 80-180 Hz band
#define VIBRATION_MAX_HZ 185.0f

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    bool vibration;
    float carrier_hz;
    float start_stop_hz;
    float vibration_rms_g;
    uint32_t missed_ticks;
    esp_err_t read_error;
} telemetry_t;

static TaskHandle_t s_sample_task;
static QueueHandle_t s_telemetry_queue;

static void sample_timer_callback(void *arg)
{
    TaskHandle_t task = (TaskHandle_t)arg;
    if (task != NULL) {
        xTaskNotifyGive(task);
    }
}

static void publish_read_error(esp_err_t err, uint32_t missed_ticks)
{
    const telemetry_t telemetry = {
        .missed_ticks = missed_ticks,
        .read_error = err,
    };
    xQueueOverwrite(s_telemetry_queue, &telemetry);
}

static void sample_task(void *arg)
{
    (void)arg;

    float raw_window[3][WINDOW_SIZE] = {{0}};
    float hp_window[3][WINDOW_SIZE] = {{0}};
    float previous_raw[3] = {0};
    float previous_hp[3] = {0};
    bool filter_initialized = false;
    size_t write_index = 0;
    size_t samples_filled = 0;
    uint32_t sample_count = 0;
    uint32_t missed_ticks = 0;

    bool previous_vibration = false;
    int64_t last_vibration_start_us = 0;
    int64_t last_vibration_stop_us = 0;
    int64_t last_state_change_us = 0;
    float start_stop_hz = 0.0f;

    while (true) {
        const uint32_t pending_ticks = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (pending_ticks > 1) {
            missed_ticks += pending_ticks - 1;
        }

        mpu6050_acceleration_t accel;
        const esp_err_t read_err = mpu6050_read_acceleration(&accel);
        if (read_err != ESP_OK) {
            publish_read_error(read_err, missed_ticks);
            continue;
        }

        const float raw[3] = {accel.x_g, accel.y_g, accel.z_g};
        float high_passed[3];
        if (!filter_initialized) {
            for (size_t axis = 0; axis < 3; ++axis) {
                previous_raw[axis] = raw[axis];
                previous_hp[axis] = 0.0f;
                high_passed[axis] = 0.0f;
            }
            filter_initialized = true;
        } else {
            for (size_t axis = 0; axis < 3; ++axis) {
                high_passed[axis] = HP_FILTER_ALPHA *
                                    (previous_hp[axis] + raw[axis] - previous_raw[axis]);
                previous_raw[axis] = raw[axis];
                previous_hp[axis] = high_passed[axis];
            }
        }

        for (size_t axis = 0; axis < 3; ++axis) {
            raw_window[axis][write_index] = raw[axis];
            hp_window[axis][write_index] = high_passed[axis];
        }
        write_index = (write_index + 1) % WINDOW_SIZE;
        if (samples_filled < WINDOW_SIZE) {
            samples_filled++;
        }
        sample_count++;

        if (samples_filled < WINDOW_SIZE || (sample_count % ANALYSIS_HOP_SIZE) != 0) {
            continue;
        }

        float means[3] = {0};
        float hp_means[3] = {0};
        float hp_squares[3] = {0};
        for (size_t i = 0; i < WINDOW_SIZE; ++i) {
            const size_t index = (write_index + i) % WINDOW_SIZE;
            for (size_t axis = 0; axis < 3; ++axis) {
                means[axis] += raw_window[axis][index];
                hp_means[axis] += hp_window[axis][index];
                hp_squares[axis] += hp_window[axis][index] * hp_window[axis][index];
            }
        }

        size_t strongest_axis = 0;
        float strongest_rms = 0.0f;
        for (size_t axis = 0; axis < 3; ++axis) {
            means[axis] /= WINDOW_SIZE;
            hp_means[axis] /= WINDOW_SIZE;
            const float variance = (hp_squares[axis] / WINDOW_SIZE) -
                                   (hp_means[axis] * hp_means[axis]);
            const float rms = sqrtf(variance > 0.0f ? variance : 0.0f);
            if (rms > strongest_rms) {
                strongest_rms = rms;
                strongest_axis = axis;
            }
        }

        int sign = 0;
        uint32_t zero_crossings = 0;
        const float deadband_g = 0.003f;
        for (size_t i = 0; i < WINDOW_SIZE; ++i) {
            const size_t index = (write_index + i) % WINDOW_SIZE;
            const float value = hp_window[strongest_axis][index] - hp_means[strongest_axis];
            int current_sign = 0;
            if (value > deadband_g) {
                current_sign = 1;
            } else if (value < -deadband_g) {
                current_sign = -1;
            }
            if (current_sign != 0) {
                if (sign != 0 && current_sign != sign) {
                    zero_crossings++;
                }
                sign = current_sign;
            }
        }

        const float carrier_hz = (zero_crossings * SAMPLE_RATE_HZ) /
                                 (2.0f * WINDOW_SIZE);
        const float rms_threshold = previous_vibration ?
                                    VIBRATION_OFF_RMS_G : VIBRATION_ON_RMS_G;
        const bool in_vibration_band = carrier_hz >= VIBRATION_MIN_HZ &&
                                       carrier_hz <= VIBRATION_MAX_HZ;
        const bool vibration = in_vibration_band && strongest_rms >= rms_threshold;
        const int64_t now_us = esp_timer_get_time();

        if (vibration != previous_vibration) {
            if (vibration) {
                if (last_vibration_start_us != 0) {
                    const int64_t period_us = now_us - last_vibration_start_us;
                    if (period_us >= 250000 && period_us <= 10000000) {
                        start_stop_hz = 1000000.0f / period_us;
                    }
                }
                last_vibration_start_us = now_us;
            } else {
                if (last_vibration_stop_us != 0) {
                    const int64_t period_us = now_us - last_vibration_stop_us;
                    if (period_us >= 250000 && period_us <= 10000000) {
                        start_stop_hz = 1000000.0f / period_us;
                    }
                }
                last_vibration_stop_us = now_us;
            }
            previous_vibration = vibration;
            last_state_change_us = now_us;
        } else if (last_state_change_us != 0 &&
                   now_us - last_state_change_us > 15000000) {
            start_stop_hz = 0.0f;
        }

        const telemetry_t telemetry = {
            .ax_g = means[0],
            .ay_g = means[1],
            .az_g = means[2],
            .vibration = vibration,
            .carrier_hz = vibration ? carrier_hz : 0.0f,
            .start_stop_hz = start_stop_hz,
            .vibration_rms_g = strongest_rms,
            .missed_ticks = missed_ticks,
            .read_error = ESP_OK,
        };
        xQueueOverwrite(s_telemetry_queue, &telemetry);
    }
}

static void telemetry_task(void *arg)
{
    (void)arg;
    telemetry_t telemetry;

    while (true) {
        if (xQueueReceive(s_telemetry_queue, &telemetry, portMAX_DELAY) == pdTRUE) {
            if (telemetry.read_error != ESP_OK) {
                printf("#ERROR,MPU6050 read,%s\r\n", esp_err_to_name(telemetry.read_error));
            } else {
                printf("%.4f,%.4f,%.4f,%d,%.1f,%.2f,%.4f,%lu\r\n",
                       telemetry.ax_g,
                       telemetry.ay_g,
                       telemetry.az_g,
                       telemetry.vibration ? 1 : 0,
                       telemetry.carrier_hz,
                       telemetry.start_stop_hz,
                       telemetry.vibration_rms_g,
                       (unsigned long)telemetry.missed_ticks);
            }
            fflush(stdout);
        }
    }
}

void app_main(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t i2c_bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus);
    if (err != ESP_OK) {
        printf("#ERROR,I2C bus init,%s\r\n", esp_err_to_name(err));
        fflush(stdout);
        return;
    }

    uint32_t probe_attempts = 0;
    while ((err = i2c_master_probe(i2c_bus, MPU6050_I2C_ADDRESS, 200)) != ESP_OK) {
        if (err != ESP_ERR_NOT_FOUND && err != ESP_ERR_TIMEOUT) {
            printf("#ERROR,I2C probe 0x%02X,%s\r\n",
                   MPU6050_I2C_ADDRESS,
                   esp_err_to_name(err));
            fflush(stdout);
            return;
        }

        probe_attempts++;
        if ((probe_attempts % 10) == 1) {
            printf("#WAIT,MPU6050 0x%02X no ACK; check power/GND/SDA=GPIO8/SCL=GPIO9\r\n",
                   MPU6050_I2C_ADDRESS);
            fflush(stdout);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    printf("#INFO,MPU6050 detected at 0x%02X\r\n", MPU6050_I2C_ADDRESS);
    fflush(stdout);

    err = mpu6050_init(i2c_bus, MPU6050_I2C_ADDRESS);
    if (err != ESP_OK) {
        printf("#ERROR,MPU6050 init,%s\r\n", esp_err_to_name(err));
        fflush(stdout);
        return;
    }

    s_telemetry_queue = xQueueCreate(1, sizeof(telemetry_t));
    if (s_telemetry_queue == NULL) {
        printf("#ERROR,telemetry queue init\r\n");
        fflush(stdout);
        return;
    }

    printf("ax_g,ay_g,az_g,vibration,carrier_hz,start_stop_hz,rms_g,missed_ticks\r\n");
    fflush(stdout);

    if (xTaskCreate(telemetry_task, "mpu6050_uart", 4096, NULL, 2, NULL) != pdPASS ||
        xTaskCreate(sample_task, "mpu6050_sample", 4096, NULL, 10, &s_sample_task) != pdPASS) {
        printf("#ERROR,task creation failed\r\n");
        fflush(stdout);
        return;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = sample_timer_callback,
        .arg = s_sample_task,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "mpu6050_sample",
    };
    esp_timer_handle_t sample_timer = NULL;
    err = esp_timer_create(&timer_args, &sample_timer);
    if (err != ESP_OK) {
        printf("#ERROR,sample timer create,%s\r\n", esp_err_to_name(err));
        fflush(stdout);
        return;
    }

    err = esp_timer_start_periodic(sample_timer, SAMPLE_PERIOD_US);
    if (err != ESP_OK) {
        printf("#ERROR,sample timer start,%s\r\n", esp_err_to_name(err));
        fflush(stdout);
    }
}
