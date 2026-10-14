#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#define MPU9250_ADDR 0x69
#define AK8963_ADDR 0x0C

    /* =========================================================
    INIT
    ========================================================= */

    /**
     * @brief ล็อค I2C เพื่อป้องกันการอ่าน/เขียนขณะกำลังทำงานกับ MPU9250
     * ใช้ร่วมกับ MY_I2C_Read/Write ในกรณีที่ต้องการทำธุรกรรม I2C ที่ซับซ้อนกว่า (เช่น burst read) โดยไม่ต้องกังวลเรื่องการชนกันของ I2C
     */
    void i2c_lock(void);

    /**
     * @brief ปลดล็อค I2C
     */
    void i2c_unlock(void);

    /**
     * @brief ตรวจสอบ MPU9250
     */
    bool mpu9250_check(void);

    /**
     * @brief ตรวจสอบ AK8963
     */
    bool ak8963_check(void);

    /**
     * @brief Initialize MPU9250 + MAG + NVS
     */
    void mpu9250_init(void);

    /* =========================================================
    MAIN
    ========================================================= */

    /**
     * @brief Calibrate the gyroscope
     */
    void imu_calibrate_gyro(void);

    /**
     * @brief อ่านค่า yaw (0–359°)
     */
    /* Drain accel/gyro FIFO at 200 Hz, predict each sample at 5 ms, then fuse
     * current mag once. Call every ~20 ms from the IMU owner task only.
     * Returns fractional degrees in [0, 360); holds output on empty/fault. */
    float mpu9250_read_yaw(void);

    /**
     * @brief ตั้งค่า offset ทิศเหนือ
     */
    /* Asynchronous request, applied by the IMU task on a valid sample. */
    void mpu9250_set_north_offset(void);

    /**
     * @brief ตรวจสอบว่า MPU9250 พร้อมสำหรับการตั้งทิศเหนือหรือไม่
     */
    int mpu9250_ready_for_north(void);

    /* =========================================================
    MAG CALIBRATION (ONLINE)
    ========================================================= */

    /**
     * @brief เริ่ม calibration (reset min/max)
     */
    /* Asynchronous requests; the latest pending start/stop request wins. */
    void mag_start_calibration(void);

    /**
     * @brief หยุด calibration + save NVS
     */
    void mag_stop_calibration(void);

    /**
     * @brief ตรวจสถานะว่าอยู่ใน calibration mode หรือไม่
     */
    int mpu9250_is_calibrating(void);

    /**
     * @brief บังคับ save calibration ลง NVS
     */
    void mag_save_nvs(void);

    /**
     * @brief โหลด calibration จาก NVS
     */
    int mag_load_nvs(void);

    /**
     * @brief Low-pass filter for magnetometer (optional)
     */
    void mag_lowpass(float in[3], float out[3]);

    /* =========================================================
    RAW / DEBUG
    ========================================================= */

    /**
     * @brief อ่าน raw sensor
     */
    void read_raw(float acc[3], float mag[3], float *gz);

    /**
     * @brief process mag (bias + scale + matrix)
     */
    void processMag(float raw[3], float out[3]);

    /**
     * @brief Get the last calibrated + lowpassed mag (set by mpu9250_read_yaw).
     * Use this in imu_task instead of re-running processMag to avoid double filtering.
     */
    void mpu9250_get_cal_mag(float out[3]);

    /**
     * @brief Get the last sensor values read by mpu9250_read_yaw().
     * Use instead of read_raw() in imu_task to avoid a redundant I2C transaction.
     */
    void mpu9250_get_last_raw(float acc_out[3], float mag_out[3], float gyro_out[3]);

    /* =========================================================
    INTERNAL (OPTIONAL DEBUG)
    ========================================================= */

    /**
     * @brief TRIAD yaw (debug)
     */
    void triad_compute(float acc[3], float mag[3], float *yaw);

#ifdef __cplusplus
}
#endif
