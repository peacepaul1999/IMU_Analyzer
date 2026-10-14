#pragma once

#include "esp_err.h"
#include <stdbool.h>

/* IMU: MPU9250 SMPLRT_DIV=4 on 1 kHz -> 200 Hz (matches MPU_FIFO_DT = 5 ms).
 * WebSocket: one frame every IMU_WS_DIV task periods of IMU_TASK_PERIOD_MS. */
#define IMU_SAMPLE_RATE_HZ 200
#define IMU_TASK_PERIOD_MS 20
#define IMU_WS_DIV 5
#define IMU_WS_RATE_HZ (1000 / (IMU_TASK_PERIOD_MS * IMU_WS_DIV))

/* Units of imu_gui_sample_t fields: acc [g], gyro [rad/s], mag [uT],
 * yaw/roll/pitch [deg]. */
typedef struct
{
    float yaw, roll, pitch, acc[3], gyro[3], mag[3];
    bool ready, calibrating, north_ready;
} imu_gui_sample_t;

esp_err_t imu_web_start(void);
void imu_web_publish(const imu_gui_sample_t *sample);
const char *firmware_version_get(void);
