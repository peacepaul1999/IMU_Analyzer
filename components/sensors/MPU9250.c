#include "mpu9250.h"
#include <math.h>
#include <stdbool.h>
#include <stdatomic.h>
#include "MY_I2C.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"

/* =========================================================
CONFIG
========================================================= */

#define MPU9250_ADDR 0x69
#define AK8963_ADDR 0x0C

#define MPU_PWR_MGMT_1 0x6B
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_INT_PIN_CFG 0x37
#define MPU_FIFO_EN 0x23
#define MPU_INT_STATUS 0x3A
#define MPU_USER_CTRL 0x6A
#define MPU_FIFO_COUNTH 0x72
#define MPU_FIFO_R_W 0x74
#define MPU_FIFO_CAPACITY 512U
#define MPU_FIFO_PACKET_SIZE 12U
#define MPU_FIFO_DT 0.005f
#define IMU_STILL_GYRO_RAD_S 0.012f
#define IMU_STILL_SAMPLES 100U /* 500 ms at 200 Hz */
static bool imu_stationary = false;
static unsigned stationary_samples = 0;
static float stationary_acc_ref[3];
static bool fifo_running = false;
static bool fifo_gap = false;
static uint32_t fifo_overflows = 0;
static uint32_t fifo_errors = 0;
static void read_mag(float mag[3]);
void ekf_predict(float gx, float gy, float gz, float dt);

#define AK8963_ST1 0x02
#define AK8963_HXL 0x03
#define AK8963_ST2 0x09
#define AK8963_CNTL1 0x0A
#define AK8963_ASAX 0x10

#define ACC_SCALE (8.0f / 32768.0f)
#define GYRO_SCALE (2000.0f / 32768.0f)
#define MAG_SCALE_RAW (4912.0f / 32760.0f)

static i2c_master_dev_handle_t mpu_dev = NULL;
static i2c_master_dev_handle_t mag_dev = NULL;

/* =========================================================
NORTH REQUEST
========================================================= */
static atomic_bool north_request = false;
/* Commands are consumed only by the IMU owner task; latest calibration request wins. */
static atomic_int cal_request = 0;
static atomic_int calibration_status = 1;
static bool imu_sample_valid = false;
static uint32_t mag_sample_ms = 0;
static bool heading_save_allowed = false;
static uint32_t heading_still_since_ms = 0;

/* =========================================================
NVS
========================================================= */

#define NVS_NAMESPACE "mag_cal"
#define NVS_KEY_VERSION "ver"
#define NVS_KEY_BIAS "bias"
#define NVS_KEY_SCALE "scale"
#define NVS_KEY_MATRIX "matrix"
#define NVS_KEY_YAW_OFFSET "yaw_ofs_v2"
#define NVS_KEY_YAW_OFFSET_ONLY "yaw_ofs_1"
#define NVS_KEY_LAST_YAW "last_yaw_v1"
#define NVS_KEY_LAST_YAW_VALID "last_yaw_ok1"

#define MAG_CAL_VERSION 2

/* =========================================================
ONLINE CALIBRATION
========================================================= */

static float magBias[3] = {0};
static float magScale[3] = {1, 1, 1};
static float magSoftIron[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}};
static float magAdjust[3] = {1, 1, 1};
static float lastMagRaw[3] = {0};
static float lastAcc[3] = {0};
static float lastGyro[3] = {0};
static int magHasSample = 0;

static float mag_min[3] = {1e9, 1e9, 1e9};
static float mag_max[3] = {-1e9, -1e9, -1e9};
static float mag_sum[3] = {0};
static float mag_cross[3][3] = {{0}};
static uint32_t mag_samples = 0;

static int magCalibrating = 1;
static float mag_filt[3] = {0};
static float mag_ref = 1.0f;
static float mag_cal_radius = 1.0f;
static float mag_last_accepted[3] = {0};
static int mag_last_accepted_valid = 0;

/* =========================================================
EKF STATE
========================================================= */

typedef struct
{
    float q[4];
    float bg[3];
} EKFState;

static EKFState ekf = {
    .q = {1, 0, 0, 0},
    .bg = {0, 0, 0}};

static float yaw_offset = 0;

/* =========================================================
EKF COVARIANCE (NEW)
========================================================= */

static float P[4][4] = {
    {0.01f, 0, 0, 0},
    {0, 0.01f, 0, 0},
    {0, 0, 0.01f, 0},
    {0, 0, 0, 0.01f}};

static float Q[4][4] = {
    {1e-5f, 0, 0, 0},
    {0, 1e-5f, 0, 0},
    {0, 0, 1e-5f, 0},
    {0, 0, 0, 1e-5f}};

/* =========================================================
STATE TRACKING
========================================================= */

static int is_static = 0;
static float trust = 0;

static float yaw_hold = 0;
static int yaw_hold_initialized = 0;
static int boot_heading_lock_cnt = 0;
static float saved_yaw = 0;
static int saved_yaw_valid = 0;
static uint32_t last_yaw_save_ms = 0;

#define HEADING_SAVE_INTERVAL_MS 60000U

/* mag_ref bootstrap: snap to first valid norm instead of slow-converging from 1.0 */
static int mag_ref_initialized = 0;

/* EKF warmup: force TRIAD alignment for first N cycles */
static int ekf_warmup = 20;

/* Last processed (bias+scale+softIron) mag for external use */
static float last_cal_mag[3] = {0};

static void ekf_set_yaw_deg(float yaw_deg);
static int mpu9250_prime_yaw_state(void);
static void yaw_offset_save_nvs(void);
static void yaw_offset_load_nvs(void);
static void heading_memory_load_nvs(void);
static void heading_memory_save_now(float yaw);
static void heading_memory_maybe_save(float yaw);
static void apply_north_offset_now(float yaw);
static esp_err_t mpu9250_init_nvs(void);

// flag to indicate if we've done the I2C recovery check at least once, to avoid spamming logs on first boot before recovery is attempted
static bool mpu_initialized = false;

/* =========================================================
MATH
========================================================= */

/* ================= GYRO CAL ================= */
static float gyro_bias_z = 0;

void imu_calibrate_gyro()
{
    float acc[3], mag[3], gz;
    int N = 500;

    float sum[3] = {0};
    unsigned valid = 0;

    vTaskDelay(pdMS_TO_TICKS(500));

    for (int i = 0; i < N; i++)
    {
        read_raw(acc, mag, &gz);
        if (imu_sample_valid)
        {
            for (int axis = 0; axis < 3; ++axis) sum[axis] += lastGyro[axis];
            ++valid;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (valid)
    {
        ekf.bg[0] = sum[0] / valid;
        ekf.bg[1] = sum[1] / valid;
        gyro_bias_z = sum[2] / valid;
        ekf.bg[2] = 0.0f;
    }
    ESP_LOGI("CAL", "gyro bias XYZ = %.5f %.5f %.5f (%u valid samples)",
             ekf.bg[0], ekf.bg[1], gyro_bias_z, valid);
}

static void normalize3(float v[3])
{
    float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n < 1e-6f)
        return;
    v[0] /= n;
    v[1] /= n;
    v[2] /= n;
}

static void quat_normalize(float q[4])
{
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-6f)
        return;
    for (int i = 0; i < 4; i++)
        q[i] /= n;
}

static float wrap360(float a)
{
    while (a < 0)
        a += 360;
    while (a >= 360)
        a -= 360;
    return a;
}

static float yaw_to_output_deg(float yaw)
{
    float output = wrap360(360.0f - yaw);
    static uint32_t last_log_ms = 0;
    uint32_t now = esp_timer_get_time() / 1000;
    if (now - last_log_ms >= 1000U)
    {
        last_log_ms = now;
        ESP_LOGI("IMU", "yaw=%.2f trust=%.2f still=%d gyro_res=%.4f,%.4f,%.4f",
                 output, trust, imu_stationary, lastGyro[0] - ekf.bg[0],
                 lastGyro[1] - ekf.bg[1], lastGyro[2] - gyro_bias_z - ekf.bg[2]);
    }
    return output;
}

/* AK8963 to MPU body frame: swap X/Y and invert Z (determinant +1). */
static void mag_to_body(const float in[3], float out[3])
{
    out[0] = in[1];
    out[1] = in[0];
    out[2] = -in[2];
}

static float angle_diff_deg(float target, float current)
{
    float diff = target - current;
    if (diff > 180.0f)
        diff -= 360.0f;
    if (diff < -180.0f)
        diff += 360.0f;
    return diff;
}

static void matrix_identity3(float m[3][3])
{
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
            m[row][col] = (row == col) ? 1.0f : 0.0f;
    }
}

static void matrix_vec_mul3(const float m[3][3], const float v[3], float out[3])
{
    for (int row = 0; row < 3; row++)
    {
        out[row] =
            m[row][0] * v[0] +
            m[row][1] * v[1] +
            m[row][2] * v[2];
    }
}

static void jacobi_eigen_3x3(float a[3][3], float eigenvectors[3][3], float eigenvalues[3])
{
    matrix_identity3(eigenvectors);

    for (int sweep = 0; sweep < 12; sweep++)
    {
        int p = 0;
        int q = 1;
        float max_off = fabsf(a[0][1]);

        if (fabsf(a[0][2]) > max_off)
        {
            p = 0;
            q = 2;
            max_off = fabsf(a[0][2]);
        }

        if (fabsf(a[1][2]) > max_off)
        {
            p = 1;
            q = 2;
            max_off = fabsf(a[1][2]);
        }

        if (max_off < 1e-6f)
            break;

        float app = a[p][p];
        float aqq = a[q][q];
        float apq = a[p][q];

        float phi = 0.5f * atan2f(2.0f * apq, aqq - app);
        float c = cosf(phi);
        float s = sinf(phi);

        for (int k = 0; k < 3; k++)
        {
            float aik = a[p][k];
            float aqk = a[q][k];
            a[p][k] = c * aik - s * aqk;
            a[q][k] = s * aik + c * aqk;
        }

        for (int k = 0; k < 3; k++)
        {
            float akp = a[k][p];
            float akq = a[k][q];
            a[k][p] = c * akp - s * akq;
            a[k][q] = s * akp + c * akq;
        }

        a[p][q] = 0.0f;
        a[q][p] = 0.0f;

        for (int k = 0; k < 3; k++)
        {
            float vip = eigenvectors[k][p];
            float viq = eigenvectors[k][q];
            eigenvectors[k][p] = c * vip - s * viq;
            eigenvectors[k][q] = s * vip + c * viq;
        }
    }

    for (int i = 0; i < 3; i++)
        eigenvalues[i] = a[i][i];
}

static int mag_finalize_soft_iron(void)
{
    if (mag_samples < 32)
        return 0;

    float span[3] = {
        mag_max[0] - mag_min[0],
        mag_max[1] - mag_min[1],
        mag_max[2] - mag_min[2]};

    if (span[0] < 10.0f || span[1] < 10.0f || span[2] < 10.0f)
        return 0;

    float inv_samples = 1.0f / (float)mag_samples;
    float mean[3];
    float cov[3][3];

    for (int i = 0; i < 3; i++)
        mean[i] = mag_sum[i] * inv_samples;

    /* Use pure statistical covariance E[xy]-E[x]E[y] so the eigenvectors
       reflect the true ellipsoid shape independent of which bias estimator
       is currently active. */
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            cov[row][col] = mag_cross[row][col] * inv_samples - mean[row] * mean[col];
        }
    }

    for (int row = 0; row < 3; row++)
    {
        for (int col = row + 1; col < 3; col++)
        {
            float sym = 0.5f * (cov[row][col] + cov[col][row]);
            cov[row][col] = sym;
            cov[col][row] = sym;
        }
    }

    float eigenvectors[3][3];
    float eigenvalues[3];
    jacobi_eigen_3x3(cov, eigenvectors, eigenvalues);

    float avg_axis = 0.0f;
    for (int i = 0; i < 3; i++)
    {
        if (eigenvalues[i] < 1e-6f)
            return 0;
        avg_axis += sqrtf(eigenvalues[i]);
    }
    avg_axis /= 3.0f;

    /* Store expected output radius so mag_ref can be initialised correctly
       after calibration.  For data on a sphere of radius R, each axis has
       RMS spread R/sqrt(3), so avg_axis = R/sqrt(3) → R = avg_axis*sqrt(3). */
    mag_cal_radius = avg_axis * 1.7321f;

    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            float value = 0.0f;
            for (int axis = 0; axis < 3; axis++)
            {
                float gain = avg_axis / sqrtf(eigenvalues[axis]);
                value += gain * eigenvectors[row][axis] * eigenvectors[col][axis];
            }
            magSoftIron[row][col] = value;
        }
    }

    magScale[0] = 1.0f;
    magScale[1] = 1.0f;
    magScale[2] = 1.0f;

    return 1;
}

/* =========================================================
NVS SAVE / LOAD
========================================================= */

void mag_save_nvs()
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
        return;

    nvs_set_u32(handle, NVS_KEY_VERSION, MAG_CAL_VERSION);
    nvs_set_blob(handle, NVS_KEY_BIAS, magBias, sizeof(magBias));
    nvs_set_blob(handle, NVS_KEY_SCALE, magScale, sizeof(magScale));
    nvs_set_blob(handle, NVS_KEY_MATRIX, magSoftIron, sizeof(magSoftIron));
    nvs_set_blob(handle, NVS_KEY_YAW_OFFSET, &yaw_offset, sizeof(yaw_offset));
    nvs_set_blob(handle, NVS_KEY_YAW_OFFSET_ONLY, &yaw_offset, sizeof(yaw_offset));

    nvs_commit(handle);
    nvs_close(handle);
}

static void yaw_offset_save_nvs(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
        return;

    esp_err_t ret = nvs_set_blob(handle, NVS_KEY_YAW_OFFSET_ONLY, &yaw_offset, sizeof(yaw_offset));
    /* An old heading belongs to the previous north reference. */
    if (ret == ESP_OK)
        ret = nvs_set_u8(handle, NVS_KEY_LAST_YAW_VALID, 0);
    if (ret == ESP_OK)
        ret = nvs_commit(handle);
    if (ret != ESP_OK)
        ESP_LOGW("MPU", "North offset save failed: %s", esp_err_to_name(ret));
    nvs_close(handle);
}

static void yaw_offset_load_nvs(void)
{
    nvs_handle_t handle;

    size_t size = sizeof(yaw_offset);

    esp_err_t ret =
        nvs_open(
            NVS_NAMESPACE,
            NVS_READONLY,
            &handle);

    if (ret != ESP_OK)
    {
        ESP_LOGW("MPU",
                 "Yaw offset open failed: %s",
                 esp_err_to_name(ret));

        yaw_offset = 0.0f;

        return;
    }

    if (nvs_get_blob(
            handle,
            NVS_KEY_YAW_OFFSET_ONLY,
            &yaw_offset,
            &size) != ESP_OK)
    {
        size = sizeof(yaw_offset);

        if (nvs_get_blob(
                handle,
                NVS_KEY_YAW_OFFSET,
                &yaw_offset,
                &size) != ESP_OK)
        {
            yaw_offset = 0.0f;
        }
    }

    nvs_close(handle);

    /* Validate value before use */
    if (isnan(yaw_offset) || isinf(yaw_offset))
    {
        ESP_LOGW("MPU",
                 "Invalid saved yaw offset");

        yaw_offset = 0.0f;
    }

    yaw_offset = wrap360(yaw_offset);

    ESP_LOGI("MPU",
             "Yaw offset loaded: %.1f",
             yaw_offset);
}

static void heading_memory_load_nvs(void)
{
    nvs_handle_t handle;
    uint8_t valid = 0;
    size_t size;

    saved_yaw_valid = 0;
    saved_yaw = 0.0f;

    esp_err_t ret =
        nvs_open(
            NVS_NAMESPACE,
            NVS_READONLY,
            &handle);

    if (ret != ESP_OK)
    {
        ESP_LOGW("MPU",
                 "Heading memory open failed: %s",
                 esp_err_to_name(ret));
        return;
    }

    ret =
        nvs_get_u8(
            handle,
            NVS_KEY_LAST_YAW_VALID,
            &valid);

    if (ret == ESP_OK && valid)
    {
        size = sizeof(saved_yaw);

        ret =
            nvs_get_blob(
                handle,
                NVS_KEY_LAST_YAW,
                &saved_yaw,
                &size);

        if (ret == ESP_OK)
        {
            /* ===== INVALID CHECK ===== */

            if (isnan(saved_yaw) || isinf(saved_yaw))
            {
                ESP_LOGW("MPU",
                         "Invalid saved yaw (NaN/INF)");

                saved_yaw = 0.0f;
                saved_yaw_valid = 0;
            }
            else
            {
                saved_yaw = wrap360(saved_yaw);

                if (saved_yaw < 0.0f || saved_yaw >= 360.0f)
                {
                    ESP_LOGW("MPU",
                             "Invalid saved yaw range");

                    saved_yaw = 0.0f;
                    saved_yaw_valid = 0;
                }
                else
                {
                    saved_yaw_valid = 1;

                    last_yaw_save_ms =
                        esp_timer_get_time() / 1000;

                    ESP_LOGI("MPU",
                             "Heading memory loaded: %.1f",
                             saved_yaw);
                }
            }
        }
        else
        {
            ESP_LOGW("MPU",
                     "Heading memory read failed: %s",
                     esp_err_to_name(ret));
        }
    }
    else if (ret != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGW("MPU",
                 "Heading memory valid-flag read failed: %s",
                 esp_err_to_name(ret));
    }

    nvs_close(handle);
}

static void heading_memory_save_now(float yaw)
{
    uint32_t now = esp_timer_get_time() / 1000;

    if (!heading_save_allowed || now - last_yaw_save_ms < HEADING_SAVE_INTERVAL_MS)
        return;
    /* Rate-limit failed attempts too. */
    last_yaw_save_ms = now;
    yaw = wrap360(yaw);

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK)
    {
        ESP_LOGW("MPU", "Heading memory save open failed: %s", esp_err_to_name(ret));
        return;
    }

    uint8_t valid = 1;
    ret = nvs_set_blob(handle, NVS_KEY_LAST_YAW, &yaw, sizeof(yaw));
    if (ret == ESP_OK)
        ret = nvs_set_u8(handle, NVS_KEY_LAST_YAW_VALID, valid);
    if (ret == ESP_OK)
        ret = nvs_commit(handle);
    nvs_close(handle);

    if (ret != ESP_OK)
    {
        ESP_LOGW("MPU", "Heading memory save failed: %s", esp_err_to_name(ret));
        return;
    }

    saved_yaw = yaw;
    saved_yaw_valid = 1;
    last_yaw_save_ms = now;
    ESP_LOGI("MPU", "Heading memory saved: %.1f", saved_yaw);
}

static void heading_memory_maybe_save(float yaw)
{
    heading_memory_save_now(yaw);
}

static esp_err_t mpu9250_init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW("MPU", "NVS init returned %s, erasing storage", esp_err_to_name(ret));
        ret = nvs_flash_erase();
        if (ret != ESP_OK)
        {
            ESP_LOGE("MPU", "NVS erase failed: %s", esp_err_to_name(ret));
            return ret;
        }
        ret = nvs_flash_init();
    }

    if (ret != ESP_OK)
    {
        ESP_LOGE("MPU", "NVS init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

int mag_load_nvs()
{
    nvs_handle_t handle;
    int loaded = 0;
    uint32_t version = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
        return 0;

    if (nvs_get_u32(handle, NVS_KEY_VERSION, &version) != ESP_OK || version != MAG_CAL_VERSION)
    {
        nvs_close(handle);
        return 0;
    }

    size_t size = sizeof(magBias);

    if (nvs_get_blob(handle, NVS_KEY_BIAS, magBias, &size) != ESP_OK)
    {
        nvs_close(handle);
        return 0;
    }

    size = sizeof(magScale);

    if (nvs_get_blob(handle, NVS_KEY_SCALE, magScale, &size) != ESP_OK)
    {
        nvs_close(handle);
        return 0;
    }

    size = sizeof(magSoftIron);
    if (nvs_get_blob(handle, NVS_KEY_MATRIX, magSoftIron, &size) != ESP_OK)
    {
        matrix_identity3(magSoftIron);
    }

    size = sizeof(yaw_offset);
    if (nvs_get_blob(handle, NVS_KEY_YAW_OFFSET, &yaw_offset, &size) != ESP_OK)
    {
        yaw_offset = 0.0f;
    }

    nvs_close(handle);
    loaded = 1;
    return loaded;
}

/* =========================================================
ONLINE CALIBRATION
========================================================= */

void mag_update_online(float raw[3])
{
    if (!magCalibrating)
        return;

    /* novelty gate — reject samples too close to last accepted */
    if (mag_last_accepted_valid)
    {
        float d2 = 0.0f;
        for (int i = 0; i < 3; i++)
        {
            float d = raw[i] - mag_last_accepted[i];
            d2 += d * d;
        }
        /* threshold ~5 % of typical span (spans are hundreds of raw counts) */
        float span_est = (mag_max[0] - mag_min[0] + mag_max[1] - mag_min[1] + mag_max[2] - mag_min[2]) / 3.0f;
        float thr = fmaxf(span_est * 0.03f, 5.0f);
        if (d2 < thr * thr)
            return;
    }
    for (int i = 0; i < 3; i++)
        mag_last_accepted[i] = raw[i];
    mag_last_accepted_valid = 1;

    mag_samples++;

    for (int row = 0; row < 3; row++)
    {
        mag_sum[row] += raw[row];
        for (int col = 0; col < 3; col++)
            mag_cross[row][col] += raw[row] * raw[col];
    }

    for (int i = 0; i < 3; i++)
    {
        if (raw[i] < mag_min[i])
            mag_min[i] = raw[i];
        if (raw[i] > mag_max[i])
            mag_max[i] = raw[i];
    }

    for (int i = 0; i < 3; i++)
        magBias[i] = (mag_max[i] + mag_min[i]) * 0.5f;

    float dx = (mag_max[0] - mag_min[0]) * 0.5f;
    float dy = (mag_max[1] - mag_min[1]) * 0.5f;
    float dz = (mag_max[2] - mag_min[2]) * 0.5f;

    float avg = (dx + dy + dz) / 3.0f;

    if (dx > 1e-3f)
        magScale[0] = avg / dx;
    if (dy > 1e-3f)
        magScale[1] = avg / dy;
    if (dz > 1e-3f)
        magScale[2] = avg / dz;

    if ((mag_samples % 16U) == 0U)
        mag_finalize_soft_iron();
}

static void mag_start_calibration_now(void)
{
    for (int i = 0; i < 3; i++)
    {
        mag_min[i] = 1e9;
        mag_max[i] = -1e9;
        mag_sum[i] = 0.0f;
        magScale[i] = 1.0f;
        for (int j = 0; j < 3; j++)
            mag_cross[i][j] = 0.0f;
    }

    matrix_identity3(magSoftIron);
    mag_samples = 0;
    mag_last_accepted_valid = 0;
    mag_ref = 1.0f;
    mag_ref_initialized = 0;
    mag_filt[0] = 0.0f;
    mag_filt[1] = 0.0f;
    mag_filt[2] = 0.0f;
    magCalibrating = 1;

    ESP_LOGI("CAL", "Calibration started");
}

static void mag_stop_calibration_now(void)
{
    magCalibrating = 0;

    if (mag_finalize_soft_iron())
    {
        /* Good calibration — seed mag_ref from computed sphere radius */
        mag_ref = mag_cal_radius;
        mag_ref_initialized = 1;
    }
    else
    {
        matrix_identity3(magSoftIron);
        /* Let mag_ref snap from first real sample */
        mag_ref_initialized = 0;
    }

    /* Re-bootstrap EKF so yaw aligns with new calibration */
    ekf_warmup = 50;

    for (int i = 0; i < 3; i++)
    {
        mag_min[i] = magBias[i];
        mag_max[i] = magBias[i];
    }

    mag_save_nvs();
    ESP_LOGI("CAL", "Calibration stopped, samples=%lu", (unsigned long)mag_samples);
}

void mag_start_calibration(void)
{
    atomic_store(&cal_request, 1);
}

void mag_stop_calibration(void)
{
    atomic_store(&cal_request, 2);
}

int mpu9250_is_calibrating(void)
{
    return atomic_load(&calibration_status);
}

// optional: call this periodically to help user rotate for calibration
void mag_lowpass(float in[3], float out[3])
{
    if (magCalibrating)
    {
        for (int i = 0; i < 3; i++)
        {
            mag_filt[i] = in[i];
            out[i] = in[i];
        }
        return;
    }

    float alpha = 0.35f;

    for (int i = 0; i < 3; i++)
    {
        mag_filt[i] = (1 - alpha) * mag_filt[i] + alpha * in[i];
        out[i] = mag_filt[i];
    }
}

/* =========================================================
INIT
========================================================= */

static void mpu9250_i2c_attach()
{
    if (mpu_dev != NULL)
        return;

    ESP_ERROR_CHECK(
        i2c_add_device(
            MPU9250_ADDR,
            &mpu_dev));

    ESP_ERROR_CHECK(
        i2c_add_device(
            AK8963_ADDR,
            &mag_dev));

    ESP_LOGI("MPU", "I2C device attached");
}

static esp_err_t mpu_write(uint8_t reg, uint8_t val)
{
    return MY_I2C_Write(
        mpu_dev,
        reg,
        &val,
        1);
}

static esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len)
{
    return MY_I2C_Read(
        mpu_dev,
        reg,
        data,
        len);
}

static esp_err_t mag_write(uint8_t reg, uint8_t val)
{
    return MY_I2C_Write(
        mag_dev,
        reg,
        &val,
        1);
}

static esp_err_t mag_read(uint8_t reg, uint8_t *data, size_t len)
{
    return MY_I2C_Read(
        mag_dev,
        reg,
        data,
        len);
}

void mpu9250_init()
{
    uint8_t asa[3] = {128, 128, 128};

    mpu9250_i2c_attach();

    /* ===== RESET ===== */

    ESP_ERROR_CHECK(
        mpu_write(
            MPU_PWR_MGMT_1,
            0x80));

    vTaskDelay(pdMS_TO_TICKS(100));

    /* ===== CLOCK SOURCE ===== */

    ESP_ERROR_CHECK(
        mpu_write(
            MPU_PWR_MGMT_1,
            0x01));

    vTaskDelay(pdMS_TO_TICKS(10));

    /* ===== ENABLE ALL AXIS ===== */

    ESP_ERROR_CHECK(
        mpu_write(
            0x6C,
            0x00));

    vTaskDelay(pdMS_TO_TICKS(20));

    /* ===== GYRO CONFIG ===== */

    ESP_ERROR_CHECK(
        mpu_write(
            0x1B,
            0x18));

    /* ===== ACC CONFIG ===== */

    ESP_ERROR_CHECK(
        mpu_write(
            0x1C,
            0x10));

    /* ===== DLPF ===== */

    ESP_ERROR_CHECK(
        mpu_write(
            0x1A,
            0x43));

    /* Accel DLPF enabled, 41 Hz bandwidth, 1 kHz output before divider. */
    ESP_ERROR_CHECK(mpu_write(0x1D, 0x03));

    /* ===== SAMPLE RATE ===== */

    ESP_ERROR_CHECK(
        mpu_write(
            0x19,
            0x04));

    /* ===== I2C BYPASS ===== */

    ESP_ERROR_CHECK(
        mpu_write(
            MPU_INT_PIN_CFG,
            0x02));

    vTaskDelay(pdMS_TO_TICKS(10));

    /* ===== MAG INIT ===== */

    ESP_ERROR_CHECK(
        mag_write(
            AK8963_CNTL1,
            0x00));

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(
        mag_write(
            AK8963_CNTL1,
            0x0F));

    vTaskDelay(pdMS_TO_TICKS(10));

    if (mag_read(
            AK8963_ASAX,
            asa,
            3) != ESP_OK)
    {
        ESP_LOGW("MPU",
                 "ASA read failed, using defaults");
    }

    for (int i = 0; i < 3; i++)
    {
        magAdjust[i] =
            (((float)asa[i] - 128.0f) / 256.0f) + 1.0f;
    }

    ESP_ERROR_CHECK(
        mag_write(
            AK8963_CNTL1,
            0x00));

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(
        mag_write(
            AK8963_CNTL1,
            0x16));

    /* ===== WAIT ===== */

    vTaskDelay(pdMS_TO_TICKS(200));

    /* ===== DEBUG ===== */

    uint8_t who = 0;

    if (mpu_read(0x75, &who, 1) == ESP_OK)
    {
        ESP_LOGI("MPU",
                 "WHOAMI = 0x%02X",
                 who);
    }
    else
    {
        ESP_LOGW("MPU",
                 "WHOAMI read failed");
    }

    uint8_t pwr2 = 0;

    if (mpu_read(0x6C, &pwr2, 1) == ESP_OK)
    {
        ESP_LOGI("MPU",
                 "PWR_MGMT_2(after) = 0x%02X",
                 pwr2);
    }

    uint8_t test[14] = {0};

    if (mpu_read(
            MPU_ACCEL_XOUT_H,
            test,
            14) == ESP_OK)
    {
        ESP_LOGI("MPU",
                 "GZ_H=%d GZ_L=%d",
                 test[12],
                 test[13]);
    }

    /* ===== NVS ===== */

    ESP_ERROR_CHECK(
        mpu9250_init_nvs());

    if (mag_load_nvs())
    {
        magCalibrating = 0;

        ESP_LOGI("CAL",
                 "Loaded saved calibration from NVS");
    }
    else
    {
        magCalibrating = 0;

        ESP_LOGW("CAL",
                 "No saved calibration found");
    }

    yaw_offset_load_nvs();

    heading_memory_load_nvs();

    imu_calibrate_gyro();

    if (!mpu9250_prime_yaw_state())
    {
        ESP_LOGW("MPU",
                 "Yaw prime skipped");
    }

    /* gz already compensated */
    ekf.bg[2] = 0.0f;

    mpu_initialized = true;

    ESP_LOGI("MPU",
             "MPU9250 ready");
}

/* Start lazily from the owner task, after all blocking sensor initialization. */
static esp_err_t mpu9250_fifo_reset(void)
{
    fifo_running = false;
    fifo_gap = true;
    imu_stationary = false;
    stationary_samples = 0;
    heading_save_allowed = false;
    heading_still_since_ms = 0;
    esp_err_t ret;
    if ((ret = mpu_write(MPU_FIFO_EN, 0x00)) != ESP_OK) return ret;
    if ((ret = mpu_write(MPU_USER_CTRL, 0x00)) != ESP_OK) return ret;
    if ((ret = mpu_write(MPU_USER_CTRL, 0x04)) != ESP_OK) return ret;
    /* FIFO_RST self-clears; do not enable the FIFO until it has completed. */
    for (int i = 0; i < 10; ++i)
    {
        uint8_t ctrl;
        if ((ret = mpu_read(MPU_USER_CTRL, &ctrl, 1)) != ESP_OK) return ret;
        if (!(ctrl & 0x04))
        {
            uint8_t status;
            if ((ret = mpu_read(MPU_INT_STATUS, &status, 1)) != ESP_OK) return ret;
            /* Enable overflow status; INT pin wiring is not required for polling. */
            if ((ret = mpu_write(0x38, 0x10)) != ESP_OK) return ret;
            if ((ret = mpu_write(MPU_USER_CTRL, 0x40)) != ESP_OK) return ret;
            /* Accel XYZ, gyro XYZ; temperature and external slaves excluded. */
            if ((ret = mpu_write(MPU_FIFO_EN, 0x78)) != ESP_OK) return ret;
            fifo_running = true;
            return ESP_OK;
        }
        vTaskDelay(1);
    }
    return ESP_ERR_TIMEOUT;
}

/* Read only complete packets from one count snapshot. New arrivals stay queued. */
static size_t mpu9250_fifo_read(uint8_t packets[504])
{
    uint8_t status, count_bytes[2];
    if (mpu_read(MPU_INT_STATUS, &status, 1) != ESP_OK ||
        mpu_read(MPU_FIFO_COUNTH, count_bytes, 2) != ESP_OK)
        goto io_error;
    unsigned count = ((unsigned)count_bytes[0] << 8) | count_bytes[1];
    if ((status & 0x10) || count >= MPU_FIFO_CAPACITY)
        goto overflow;
    size_t bytes = (count / MPU_FIFO_PACKET_SIZE) * MPU_FIFO_PACKET_SIZE;
    if (bytes == 0) return 0;
    if (mpu_read(MPU_FIFO_R_W, packets, bytes) != ESP_OK)
        goto io_error;
    /* Catch overflow while preempted during the transfer as well. */
    if (mpu_read(MPU_INT_STATUS, &status, 1) != ESP_OK)
        goto io_error;
    if (status & 0x10) goto overflow;
    return bytes / MPU_FIFO_PACKET_SIZE;

overflow:
    ++fifo_overflows;
    ESP_LOGW("IMU", "FIFO overflow #%lu: samples lost, resetting",
             (unsigned long)fifo_overflows);
    (void)mpu9250_fifo_reset();
    return 0;
io_error:
    ++fifo_errors;
    ESP_LOGW("IMU", "FIFO read error #%lu: packet alignment lost, resetting",
             (unsigned long)fifo_errors);
    (void)mpu9250_fifo_reset();
    return 0;
}

/* Every FIFO sample participates: a burst of motion cannot be hidden by a
 * stationary final sample. Accel reference stays fixed throughout the dwell. */
static void imu_update_stationary(float gx, float gy, float gz)
{
    float norm = sqrtf(lastAcc[0] * lastAcc[0] + lastAcc[1] * lastAcc[1] +
                       lastAcc[2] * lastAcc[2]);
    bool candidate = fabsf(norm - 1.0f) < 0.03f &&
        gx * gx + gy * gy + gz * gz < IMU_STILL_GYRO_RAD_S * IMU_STILL_GYRO_RAD_S;
    float unit[3] = {0};
    if (candidate)
    {
        float delta2 = 0;
        for (int i = 0; i < 3; ++i)
        {
            unit[i] = lastAcc[i] / norm;
            if (stationary_samples == 0) stationary_acc_ref[i] = unit[i];
            float delta = unit[i] - stationary_acc_ref[i];
            delta2 += delta * delta;
        }
        candidate = delta2 < 0.0001f; /* approximately 0.57 degrees */
    }
    if (!candidate) stationary_samples = 0;
    else if (stationary_samples < IMU_STILL_SAMPLES) ++stationary_samples;
    imu_stationary = stationary_samples >= IMU_STILL_SAMPLES;
}

static void mpu9250_fifo_integrate(const uint8_t *packet)
{
    for (int axis = 0; axis < 3; ++axis)
    {
        int16_t a = (int16_t)(((uint16_t)packet[2 * axis] << 8) |
                            packet[2 * axis + 1]);
        int16_t g = (int16_t)(((uint16_t)packet[6 + 2 * axis] << 8) |
                            packet[7 + 2 * axis]);
        lastAcc[axis] = a * ACC_SCALE;
        lastGyro[axis] = g * GYRO_SCALE * (M_PI / 180.0f);
    }
    float gz = lastGyro[2] - gyro_bias_z;
    imu_update_stationary(lastGyro[0] - ekf.bg[0], lastGyro[1] - ekf.bg[1],
                          gz - ekf.bg[2]);
    if (imu_stationary)
    {
        /* Learn residual bias only after confirmed rest, using un-deadbanded
         * measurements. Pass bias to predict so its subtraction yields zero. */
        const float alpha = 0.001f;
        ekf.bg[0] += alpha * (lastGyro[0] - ekf.bg[0]);
        ekf.bg[1] += alpha * (lastGyro[1] - ekf.bg[1]);
        ekf.bg[2] += alpha * (gz - ekf.bg[2]);
        ekf_predict(ekf.bg[0], ekf.bg[1], ekf.bg[2], MPU_FIFO_DT);
    }
    else
        ekf_predict(lastGyro[0], lastGyro[1], gz, MPU_FIFO_DT);
}

static bool mpu9250_recover(void)
{
    ESP_LOGW("MPU", "Soft recovering MPU9250");

    if (mpu_write(MPU_PWR_MGMT_1, 0x01) != ESP_OK)
    {
        ESP_LOGE("MPU", "Recover step failed: PWR_MGMT_1");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    if (mpu_write(MPU_INT_PIN_CFG, 0x02) != ESP_OK)
    {
        ESP_LOGE("MPU", "Recover step failed: INT_PIN_CFG");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    if (mag_write(AK8963_CNTL1, 0x00) != ESP_OK)
    {
        ESP_LOGE("MPU", "Recover step failed: MAG RESET");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    if (mag_write(AK8963_CNTL1, 0x16) != ESP_OK)
    {
        ESP_LOGE("MPU", "Recover step failed: MAG CONT MODE");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    /* Reapply sample configuration after a bus/device fault. */
    if (mpu_write(0x1B, 0x18) != ESP_OK ||
        mpu_write(0x1C, 0x10) != ESP_OK ||
        mpu_write(0x1A, 0x43) != ESP_OK ||
        mpu_write(0x1D, 0x03) != ESP_OK ||
        mpu_write(0x19, 0x04) != ESP_OK ||
        mpu9250_fifo_reset() != ESP_OK)
        return false;
    ESP_LOGW("MPU", "Recovery success");

    return true;
}

/* =========================================================
RAW READ
========================================================= */

void read_raw(float acc[3], float mag[3], float *gz)
{
    imu_sample_valid = false;
    uint8_t buf[14];

    /* ===== MPU READ (FIXED) ===== */
    if (mpu_read(MPU_ACCEL_XOUT_H, buf, 14) != ESP_OK)
    {
        ESP_LOGE("IMU", "I2C read fail");

        acc[0] = 0;
        acc[1] = 0;
        acc[2] = 1.0f;

        mag[0] = 0;
        mag[1] = 0;
        mag[2] = 0;

        *gz = 0;

        vTaskDelay(pdMS_TO_TICKS(10));

        return;
    }

    imu_sample_valid = true;
    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t g = (int16_t)((buf[12] << 8) | buf[13]);

    /* DEBUG */
    static uint32_t dbg = 0;
    uint32_t log_ms = esp_timer_get_time() / 1000;
    if (log_ms - dbg >= 1000U)
    {
        dbg = log_ms;

#if CONFIG_ENABLE_IMU_RAW_LOG

        ESP_LOGI("RAWCHK",
                 "RAW - AX=%d AY=%d AZ=%d GZ=%d",
                 ax, ay, az, g);

#endif
    }

    acc[0] = ax * ACC_SCALE;
    acc[1] = ay * ACC_SCALE;
    acc[2] = az * ACC_SCALE;

    *gz = g * GYRO_SCALE * (M_PI / 180.0f);

    /* DEBUG - Log scaled accelerometer and gyroscope values */
    static uint32_t dbg_scaled = 0;
    if (log_ms - dbg_scaled >= 1000U)
    {
        dbg_scaled = log_ms;

#if CONFIG_ENABLE_IMU_RAW_LOG

        ESP_LOGI("IMU_ACCEL_GYRO",
                 "Accel: X=%.4f Y=%.4f Z=%.4f m/s² | Gyro Z=%.4f rad/s",
                 acc[0], acc[1], acc[2], *gz);

#endif
    }

    /* cache */
    lastAcc[0] = acc[0];
    lastAcc[1] = acc[1];
    lastAcc[2] = acc[2];
    lastGyro[0] = gx * GYRO_SCALE * (M_PI / 180.0f);
    lastGyro[1] = gy * GYRO_SCALE * (M_PI / 180.0f);
    lastGyro[2] = *gz;

    read_mag(mag);
}

static void read_mag(float mag[3])
{
    if (magHasSample)
    {
        mag[0] = lastMagRaw[0];
        mag[1] = lastMagRaw[1];
        mag[2] = lastMagRaw[2];
    }
    else
    {
        mag[0] = mag[1] = mag[2] = 0;
    }

    uint8_t st1;

    /* 🔥 FIX: ใช้ mag_read */
    if (mag_read(AK8963_ST1, &st1, 1) != ESP_OK)
        return;

    if (!(st1 & 1))
        return;

    uint8_t m[7];

    if (mag_read(AK8963_HXL, m, 7) != ESP_OK)
        return;

    if (m[6] & 0x08)
        return;

    mag[0] = (float)((int16_t)((m[1] << 8) | m[0])) * magAdjust[0] * MAG_SCALE_RAW;
    mag[1] = (float)((int16_t)((m[3] << 8) | m[2])) * magAdjust[1] * MAG_SCALE_RAW;
    mag[2] = (float)((int16_t)((m[5] << 8) | m[4])) * magAdjust[2] * MAG_SCALE_RAW;

    mag_sample_ms = esp_timer_get_time() / 1000;
    lastMagRaw[0] = mag[0];
    lastMagRaw[1] = mag[1];
    lastMagRaw[2] = mag[2];
    magHasSample = 1;
}

void mpu9250_get_last_raw(float acc_out[3], float mag_out[3], float gyro_out[3])
{
    acc_out[0] = lastAcc[0];
    acc_out[1] = lastAcc[1];
    acc_out[2] = lastAcc[2];
    mag_out[0] = lastMagRaw[0];
    mag_out[1] = lastMagRaw[1];
    mag_out[2] = lastMagRaw[2];
    gyro_out[0] = lastGyro[0];
    gyro_out[1] = lastGyro[1];
    gyro_out[2] = lastGyro[2];
}

/* =========================================================
MAG PROCESS (HYBRID)
========================================================= */

void processMag(float raw[3], float out[3])
{
    float tmp[3];

    // online update
    mag_update_online(raw);

    // bias
    tmp[0] = raw[0] - magBias[0];
    tmp[1] = raw[1] - magBias[1];
    tmp[2] = raw[2] - magBias[2];

    // scale
    tmp[0] *= magScale[0];
    tmp[1] *= magScale[1];
    tmp[2] *= magScale[2];

    matrix_vec_mul3(magSoftIron, tmp, out);

    // for (int i = 0; i < 3; i++)
    // {
    //     if (out[i] > 1.5f)
    //         out[i] = 1.5f;
    //     if (out[i] < -1.5f)
    //         out[i] = -1.5f;
    // }

    // // optional: normalize
    // float n = sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
    // if (n > 1e-6f)
    // {
    //     out[0] /= n;
    //     out[1] /= n;
    //     out[2] /= n;
    // }
}

/* =========================================================
TRIAD + EKF
========================================================= */

void triad_compute(float acc[3], float mag[3], float *yaw)
{
    float g[3] = {acc[0], acc[1], acc[2]};
    float m[3] = {mag[0], mag[1], mag[2]};

    normalize3(g);
    normalize3(m);

    float east[3] = {
        m[1] * g[2] - m[2] * g[1],
        m[2] * g[0] - m[0] * g[2],
        m[0] * g[1] - m[1] * g[0]};
    normalize3(east);

    float north[3] = {
        g[1] * east[2] - g[2] * east[1],
        g[2] * east[0] - g[0] * east[2],
        g[0] * east[1] - g[1] * east[0]};

    *yaw = wrap360(atan2f(north[1], north[0]) * 180.0f / M_PI);
}

/* EKF functions */

float computeYaw()
{
    float q0 = ekf.q[0], q1 = ekf.q[1], q2 = ekf.q[2], q3 = ekf.q[3];

    float yaw = atan2f(
                    2 * (q1 * q2 + q0 * q3),
                    q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) *
                180.0f / M_PI;

    return wrap360(yaw);
}

static void ekf_set_yaw_deg(float yaw_deg)
{
    float yaw_rad = yaw_deg * (M_PI / 180.0f);
    float half_yaw = 0.5f * yaw_rad;

    ekf.q[0] = cosf(half_yaw);
    ekf.q[1] = 0.0f;
    ekf.q[2] = 0.0f;
    ekf.q[3] = sinf(half_yaw);
}

/* Rotate only world yaw; keep roll/pitch intact while synchronizing the hold. */
static void ekf_align_yaw_deg(float yaw_deg)
{
    float half = angle_diff_deg(yaw_deg, computeYaw()) * (M_PI / 360.0f);
    float c = cosf(half), s = sinf(half);
    float q0 = ekf.q[0], q1 = ekf.q[1], q2 = ekf.q[2], q3 = ekf.q[3];
    ekf.q[0] = c * q0 - s * q3;
    ekf.q[1] = c * q1 - s * q2;
    ekf.q[2] = c * q2 + s * q1;
    ekf.q[3] = c * q3 + s * q0;
    quat_normalize(ekf.q);
}

static bool mpu9250_hold_static_yaw(void)
{
    if (!imu_stationary || magCalibrating || !yaw_hold_initialized)
        return false;
    ekf_align_yaw_deg(wrap360(yaw_hold - yaw_offset));
    heading_memory_maybe_save(yaw_hold);
    return true;
}

static int mpu9250_prime_yaw_state(void)
{
    float acc[3], mag_raw[3], gz;
    float mag_processed[3], mag_filtered[3];

    for (int attempt = 0; attempt < 20; attempt++)
    {
        read_raw(acc, mag_raw, &gz);

        if (!magHasSample)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        processMag(mag_raw, mag_processed);
        mag_lowpass(mag_processed, mag_filtered);

        last_cal_mag[0] = mag_filtered[0];
        last_cal_mag[1] = mag_filtered[1];
        last_cal_mag[2] = mag_filtered[2];

        float mag_n[3];
        mag_to_body(mag_filtered, mag_n);
        normalize3(mag_n);

        float yaw_meas;
        triad_compute(acc, mag_n, &yaw_meas);

        ekf_set_yaw_deg(yaw_meas);
        yaw_hold = wrap360(yaw_meas + yaw_offset);

        if (saved_yaw_valid)
        {
            yaw_offset = wrap360(yaw_offset + angle_diff_deg(saved_yaw, yaw_hold));
            yaw_hold = saved_yaw;
            boot_heading_lock_cnt = 30;
        }
        else
        {
            boot_heading_lock_cnt = 0;
        }

        yaw_hold_initialized = 1;
        ekf_warmup = 0;

        ESP_LOGI("MPU", "Yaw primed at boot: %.1f", yaw_hold);
        return 1;
    }

    return 0;
}

/* =========================================================
EKF PREDICT
========================================================= */

void ekf_predict(float gx, float gy, float gz, float dt)
{
    float q0 = ekf.q[0], q1 = ekf.q[1], q2 = ekf.q[2], q3 = ekf.q[3];

    gx -= ekf.bg[0];
    gy -= ekf.bg[1];
    gz -= ekf.bg[2];

    // quaternion update
    ekf.q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    ekf.q[1] += 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt;
    ekf.q[2] += 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt;
    ekf.q[3] += 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt;

    quat_normalize(ekf.q);

    // covariance predict (simplified)
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            P[i][j] += Q[i][j] * dt;
        }
    }
}

/* =========================================================
EKF UPDATE YAW
========================================================= */

void ekf_update_yaw(float yaw_meas, float dt)
{
    float yaw_est = atan2f(
                        2 * (ekf.q[1] * ekf.q[2] + ekf.q[0] * ekf.q[3]),
                        ekf.q[0] * ekf.q[0] + ekf.q[1] * ekf.q[1] - ekf.q[2] * ekf.q[2] - ekf.q[3] * ekf.q[3]) *
                    180.0f / M_PI;

    float err = yaw_meas - yaw_est;

    if (err > 180)
        err -= 360;
    if (err < -180)
        err += 360;

    float err_rad = err * (M_PI / 180.0f);

    // adaptive noise
    float R_yaw = 5.0f / fmaxf(trust, 0.1f);

    float S = P[3][3] + R_yaw;

    float K[4];
    for (int i = 0; i < 4; i++)
        K[i] = P[i][3] / S;

    // update quaternion
    for (int i = 0; i < 4; i++)
        ekf.q[i] += K[i] * err_rad;

    quat_normalize(ekf.q);

    // update covariance
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            P[i][j] -= K[i] * P[3][j];
        }
    }
}

/* =========================================================
MAIN
========================================================= */

static float mpu9250_fuse_latest(float dt)
{
    static uint32_t disagreement_since_ms = 0;
    heading_save_allowed = false;
    if (fifo_gap)
    {
        disagreement_since_ms = 0;
        heading_still_since_ms = 0;
        fifo_gap = false;
    }
    float acc[3] = {lastAcc[0], lastAcc[1], lastAcc[2]};
    float magRaw[3], gz = lastGyro[2];
    read_mag(magRaw);

    if (atomic_exchange(&north_request, false))
        apply_north_offset_now(computeYaw());

    gz -= gyro_bias_z;

    float mag[3], mag_tmp[3];
    processMag(magRaw, mag_tmp);
    mag_lowpass(mag_tmp, mag);

    last_cal_mag[0] = mag[0];
    last_cal_mag[1] = mag[1];
    last_cal_mag[2] = mag[2];

    float mag2[3];
    mag_to_body(mag, mag2);

    uint32_t t = esp_timer_get_time() / 1000;

    /* =========================
    STATIC DETECTION (VECTOR ANGLE)
    ========================= */
    float acc_norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (acc_norm < 1e-6f) return yaw_to_output_deg(yaw_hold);
    int still_candidate = imu_stationary;
    is_static = imu_stationary;

    /* =========================
       EKF PREDICT
    ========================= */
    /* All queued gyro samples have already been integrated at exactly 5 ms. */

    /* tilt correction */
    float acc_err = fabsf(acc_norm - 1.0f);
    float acc_trust = 1.0f - (acc_err / 0.12f);
    acc_trust = fminf(fmaxf(acc_trust, 0.0f), 1.0f);

    /* Acceleration away from 1 g makes TRIAD tilt compensation unreliable. */

    /* =========================
       TRIAD (mag yaw)
    ========================= */
    float mag_n[3] = {mag2[0], mag2[1], mag2[2]};
    normalize3(mag_n);

    float yaw_meas;
    triad_compute(acc, mag_n, &yaw_meas);

    /* =========================
       MAG TRUST (DPV tuned)
    ========================= */
    float norm = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);

    if (!magCalibrating && norm > 1e-3f)
    {
        if (!mag_ref_initialized)
        {
            mag_ref = norm;
            mag_ref_initialized = 1;
        }
        else
        {
            mag_ref = 0.995f * mag_ref + 0.005f * norm;
        }
    }

    float err = fabsf(norm - mag_ref);
    trust = 1.0f - (err / fmaxf(0.25f * mag_ref, 1e-3f));

    trust = fminf(fmaxf(trust, 0.0f), 1.0f);

    if (norm < 0.25f * mag_ref || norm > 1.75f * mag_ref)
        trust = 0.0f;
    trust *= acc_trust;
    bool mag_fresh = magHasSample && (t - mag_sample_ms <= 100U);
    if (!mag_fresh || magCalibrating)
        trust = 0.0f;

    /* =========================
       MODE SELECT (สำคัญสุด🔥)
    ========================= */
    typedef enum
    {
        MODE_GYRO_ONLY,
        MODE_BLEND,
        MODE_MAG_LOCK
    } YawMode;

    static YawMode mode;

    if (trust > 0.85f &&
        is_static &&
        fabsf(gz) < 0.01f)
        mode = MODE_MAG_LOCK;
    else if (trust > 0.5f)
        mode = MODE_BLEND;
    else
        mode = MODE_GYRO_ONLY;

    /* =========================
       OUTPUT
    ========================= */
    float yaw = computeYaw();

    yaw = wrap360(yaw + yaw_offset);
    float yaw_meas_adj = wrap360(yaw_meas + yaw_offset);

    if (saved_yaw_valid && boot_heading_lock_cnt > 0 && trust > 0.85f)
    {
        yaw_offset = wrap360(yaw_offset + angle_diff_deg(saved_yaw, yaw_meas_adj));
        yaw_meas_adj = saved_yaw;
        yaw = saved_yaw;
        yaw_hold = saved_yaw;
        yaw_hold_initialized = 1;
        boot_heading_lock_cnt--;
        return yaw_to_output_deg(yaw_hold);
    }

    if (!yaw_hold_initialized)
    {
        if (!magHasSample)
            return 0;

        ekf_set_yaw_deg(yaw_meas);
        yaw = yaw_meas_adj;
        yaw_hold = yaw_meas_adj;
        yaw_hold_initialized = 1;
    }

    /* =========================
       MAG SPIKE KILLER 🔥
    ========================= */
    static float prev_mag_yaw = 0;
    static int mag_jump_init = 0;

    float mag_jump = 0;

    if (!mag_jump_init)
    {
        prev_mag_yaw = yaw_meas_adj;
        mag_jump_init = 1;
    }
    else
    {
        mag_jump =
            fabsf(angle_diff_deg(
                yaw_meas_adj,
                prev_mag_yaw));

        prev_mag_yaw = yaw_meas_adj;

        if (mag_jump > 8.0f)
        {
            trust *= 0.1f;
        }
    }

    float dYaw = yaw_meas_adj - yaw_hold;
    if (dYaw > 180.0f)
        dYaw -= 360.0f;
    if (dYaw < -180.0f)
        dYaw += 360.0f;

    if (fabsf(dYaw) > 12.0f)
    {
        if (trust > 0.85f && mag_jump < 1.0f && still_candidate)
        {
            if (disagreement_since_ms == 0) disagreement_since_ms = t;
            if (t - disagreement_since_ms > 2000U)
            {
                yaw_hold = yaw_meas_adj;
                ekf_set_yaw_deg(yaw_meas);
                disagreement_since_ms = 0;
            }
            else trust = 0.0f;
        }
        else
        {
            disagreement_since_ms = 0;
            trust = 0.0f;
        }
    }
    else disagreement_since_ms = 0;

    /* Recompute mode AFTER all trust gates, including spikes and acceleration. */
    mode = trust > 0.85f && is_static ? MODE_MAG_LOCK :
           trust > 0.5f ? MODE_BLEND : MODE_GYRO_ONLY;
    if (still_candidate && trust > 0.85f && mag_jump < 1.0f)
    {
        if (heading_still_since_ms == 0) heading_still_since_ms = t;
        heading_save_allowed = t - heading_still_since_ms >= 2000U;
    }
    else heading_still_since_ms = 0;

    /* Run AFTER disagreement recovery, so a valid >12 degree recovery still
     * works. Ordinary mag noise must not move a confirmed stationary output. */
    if (mpu9250_hold_static_yaw())
        return yaw_to_output_deg(yaw_hold);


    /* =========================
       EKF UPDATE ตาม MODE
    ========================= */
    if (mode == MODE_MAG_LOCK)
    {
        ekf_update_yaw(yaw_meas, dt);
    }
    else if (mode == MODE_BLEND)
    {
        ekf_update_yaw(yaw_meas, dt * 0.5f);
    }

    yaw = computeYaw();
    yaw = wrap360(yaw + yaw_offset);

    if (ekf_warmup > 0 && trust > 0.5f)
    {
        ekf_warmup--;
        yaw = yaw_meas_adj;
    }

    if (mode == MODE_MAG_LOCK && is_static)
    {
        yaw_hold = yaw_meas_adj;
        heading_memory_maybe_save(yaw_hold);
        return yaw_to_output_deg(yaw_hold);
    }

    /* =========================
       FREEZE LOGIC (ปลอดภัย)
    ========================= */
    /* Static lock is handled before EKF update so the filter does not drift while parked. */

    /* =========================
       SMOOTHING (adaptive)
    ========================= */
    float dy = yaw - yaw_hold;
    if (dy > 180)
        dy -= 360;
    if (dy < -180)
        dy += 360;

    float alpha;
    if (mode == MODE_MAG_LOCK)
        alpha = 0.1f;
    else if (mode == MODE_BLEND)
        alpha = 0.15f;
    else
        alpha = 0.25f;

    yaw = yaw_hold + alpha * dy;
    yaw = wrap360(yaw);
    yaw_hold = yaw;
    heading_memory_maybe_save(yaw_hold);

    /* ===== DEBUG ===== */

    /* ===== FIX: HARD FREEZE ===== */
    // if (is_static && fabsf(gz) < 0.02f && trust > 0.7f)
    // {
    //     ekf.q[1] = 0;
    //     ekf.q[2] = 0;
    //     ekf.q[3] = sinf((yaw_hold * M_PI / 180.0f) * 0.5f);
    //     ekf.q[0] = cosf((yaw_hold * M_PI / 180.0f) * 0.5f);
    // }

    /* =========================
       FINAL LOCK
    ========================= */
    if (mode == MODE_MAG_LOCK && is_static)
    {
        heading_memory_maybe_save(yaw_hold);
        return yaw_to_output_deg(yaw_hold);
    }

    return yaw_to_output_deg(yaw);
}

float mpu9250_read_yaw(void)
{
    static uint32_t last_recover_ms = 0;
    int command = atomic_exchange(&cal_request, 0);
    if (command == 1) mag_start_calibration_now();
    if (command == 2) mag_stop_calibration_now();
    atomic_store(&calibration_status, magCalibrating);

    if (mpu_initialized && MY_I2C_NeedsRecovery())
    {
        uint32_t now = esp_log_timestamp();
        if (now - last_recover_ms >= 2000U)
        {
            last_recover_ms = now;
            if (mpu9250_recover())
            {
                MY_I2C_ClearRecoveryFlag();
                magHasSample = 0;
            }
        }
    }
    if (!fifo_running)
    {
        (void)mpu9250_fifo_reset();
        return yaw_to_output_deg(yaw_hold);
    }

    uint8_t packets[504];
    size_t samples = mpu9250_fifo_read(packets);
    if (samples == 0)
        return yaw_to_output_deg(yaw_hold);

    /* Chronological integration, including every sample after a 100 ms stall. */
    for (size_t i = 0; i < samples; ++i)
        mpu9250_fifo_integrate(&packets[i * MPU_FIFO_PACKET_SIZE]);

    /* The bypass magnetometer is current, not a historical FIFO measurement. */
    return mpu9250_fuse_latest(samples * MPU_FIFO_DT);
}

void mpu9250_get_cal_mag(float out[3])
{
    out[0] = last_cal_mag[0];
    out[1] = last_cal_mag[1];
    out[2] = last_cal_mag[2];
}

/* =========================================================
SERIAL CMD set NORTH OFFSET + CALIBRATION
========================================================= */

void mpu9250_set_north_offset(void)
{
    atomic_store(&north_request, true);
}

int mpu9250_ready_for_north(void)
{
    return 1;
}

static void apply_north_offset_now(float yaw)
{
    yaw_offset = wrap360(-yaw);
    yaw_hold = 0;
    yaw_hold_initialized = 1;
    ekf_set_yaw_deg(yaw);
    yaw_offset_save_nvs();
    boot_heading_lock_cnt = 0;
    saved_yaw_valid = 0;
}
