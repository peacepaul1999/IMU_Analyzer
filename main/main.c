#define Firmware_Version "1.0.0"

#include <math.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "MY_I2C.h"
#include "MPU9250.h"
#include "pin_config.h"
#include "imu_web.h"
#include "system_info.h"

const char *firmware_version_get(void)
{
        return Firmware_Version;
}

static const char *TAG = "IMU";
static void imu_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    unsigned gui_div = 0, log_div = 0;

    while (1)
    {
        /* Preserve the 20 ms FIFO-drain cadence and existing EKF. */
        imu_gui_sample_t s = {0};
        s.yaw = mpu9250_read_yaw();
        float raw_mag[3];

        mpu9250_get_last_raw(s.acc, raw_mag, s.gyro);
        mpu9250_get_cal_mag(s.mag);
        s.roll = atan2f(s.acc[1], s.acc[2]) * 180.0f / (float)M_PI;
        s.pitch = atan2f(-s.acc[0], sqrtf(s.acc[1] * s.acc[1] + s.acc[2] * s.acc[2])) * 180.0f / (float)M_PI;
        s.ready = mpu9250_ready_for_north() != 0;
        s.north_ready = s.ready;
        s.calibrating = mpu9250_is_calibrating() != 0;

        if (++gui_div >= IMU_WS_DIV)
        {
            // ~10 Hz browser update, independent of 200 Hz sensor FIFO
            gui_div = 0;
            imu_web_publish(&s);
        }

        if (++log_div >= 50)
        {
            log_div = 0;
            ESP_LOGI(TAG, "Yaw %.2f | Roll %.2f | Pitch %.2f", s.yaw, s.roll, s.pitch);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting application...");
    ESP_LOGI(TAG, "project: IMU Analyzer");
    ESP_LOGI(TAG, "Firmware Version: %s", firmware_version_get());
    print_mac_address();
    ESP_LOGI(TAG, "Serial: %s", get_serial_number());
    ESP_LOGI(TAG, "========== PSRAM INFO ==========");
    ESP_LOGI(TAG, "PSRAM Total: %u bytes", (unsigned)heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "PSRAM Free: %u bytes", (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "================================");

    ESP_ERROR_CHECK(i2c_master_init(SDA_PIN, SCL_PIN, I2C_PORT_NUM, I2C_CLK_SPEED_HZ));
#if CONFIG_ENABLE_I2C_SCAN
    i2c_scan_devices();
#endif
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu9250_init();
    ESP_ERROR_CHECK(imu_web_start());
#if CONFIG_FREERTOS_UNICORE
    const BaseType_t core = 0;
#else
    const BaseType_t core = 1;
#endif
    configASSERT(xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 6, NULL, core) == pdPASS);
}
