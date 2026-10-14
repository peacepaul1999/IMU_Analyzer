#include "MY_I2C.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <string.h>

static const char *TAG = "I2C";

static i2c_master_bus_handle_t i2c_bus;
static SemaphoreHandle_t i2c_mutex;
static volatile bool i2c_bus_recovered = false;
static gpio_num_t s_i2c_sda = GPIO_NUM_NC;
static gpio_num_t s_i2c_scl = GPIO_NUM_NC;
static i2c_port_t s_i2c_port = I2C_NUM_0;

uint32_t i2c_clk_hz = 100000; // 100kHz

static esp_err_t i2c_lock(void)
{
    if (i2c_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE ? ESP_OK : ESP_FAIL;
}

static void i2c_unlock(void)
{
    if (i2c_mutex != NULL)
    {
        xSemaphoreGive(i2c_mutex);
    }
}

/* ================= Init ================= */
esp_err_t i2c_master_init(gpio_num_t sda_io,
                          gpio_num_t scl_io,
                          i2c_port_t port,
                          uint32_t clk_speed_hz)
{
    s_i2c_sda = sda_io;
    s_i2c_scl = scl_io;
    s_i2c_port = port;

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = port,
        .scl_io_num = scl_io,
        .sda_io_num = sda_io,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &i2c_bus);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Free any slave holding SDA low from a previous session */
    i2c_master_bus_reset(i2c_bus);
    vTaskDelay(pdMS_TO_TICKS(10));

    i2c_clk_hz = clk_speed_hz;
    if (i2c_mutex == NULL)
    {
        i2c_mutex = xSemaphoreCreateMutex();
        if (i2c_mutex == NULL)
        {
            ESP_LOGE(TAG, "I2C mutex init failed");
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_LOGI(TAG, "I2C init OK, freq=%lu Hz", i2c_clk_hz);
    return ESP_OK;
}

/* ================= Write ================= */
esp_err_t i2c_write_reg(uint8_t dev_addr,
                        uint8_t reg_addr,
                        const uint8_t *data,
                        size_t len)
{
    if (!i2c_bus || !data || len == 0)
        return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(i2c_lock(), TAG, "lock");

    i2c_master_dev_handle_t dev;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = i2c_clk_hz,
    };

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev);
    if (ret != ESP_OK)
    {
        i2c_unlock();
        return ret;
    }

    uint8_t buf[1 + len];
    buf[0] = reg_addr;
    memcpy(&buf[1], data, len);

    ret = i2c_master_transmit(dev, buf, sizeof(buf), 100);
    if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGE(TAG, "I2C write timeout");
        i2c_bus_recovered = true;
        MY_I2C_RecoverBus();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C write fail: %s", esp_err_to_name(ret));
    }
    i2c_master_bus_rm_device(dev);
    i2c_unlock();
    return ret;
}

/* ================= Read ================= */
esp_err_t i2c_read_reg(uint8_t dev_addr,
                       uint8_t reg_addr,
                       uint8_t *data,
                       size_t len)
{
    if (!i2c_bus || !data || len == 0)
        return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(i2c_lock(), TAG, "lock");

    i2c_master_dev_handle_t dev;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = i2c_clk_hz,
    };

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev);
    if (ret != ESP_OK)
    {
        i2c_unlock();
        return ret;
    }

    ret = i2c_master_transmit_receive(
        dev,
        &reg_addr,
        1,
        data,
        len,
        100);

    if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGE(TAG, "I2C read timeout");
        i2c_bus_recovered = true;
        MY_I2C_RecoverBus();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read fail: %s", esp_err_to_name(ret));
    }

    i2c_master_bus_rm_device(dev);
    i2c_unlock();
    return ret;
}

void i2c_scan_devices(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");

    if (!i2c_bus)
    {
        ESP_LOGW(TAG, "I2C bus not initialized");
        return;
    }

    for (uint8_t addr = 1; addr < 127; addr++)
    {
        esp_err_t ret = i2c_master_probe(i2c_bus, addr,  5);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Found device at address 0x%02X", addr);
        }
    }

    ESP_LOGI(TAG, "I2C scan complete.");
}

esp_err_t i2c_add_device(uint8_t dev_addr,
                         i2c_master_dev_handle_t *dev_handle)
{
    if (!i2c_bus)
        return ESP_ERR_INVALID_STATE;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = i2c_clk_hz,
    };

    return i2c_master_bus_add_device(i2c_bus, &dev_cfg, dev_handle);
}

esp_err_t MY_I2C_Write(i2c_master_dev_handle_t dev,
                         uint8_t reg,
                         const uint8_t *data,
                         size_t len)
{
    if (dev == NULL || data == NULL || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(i2c_lock(), TAG, "lock");

    uint8_t buf[1 + len];
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    esp_err_t ret = i2c_master_transmit(dev, buf, sizeof(buf), 100);
    if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGE(TAG, "I2C write timeout");
        i2c_bus_recovered = true;
        MY_I2C_RecoverBus();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C write fail: %s", esp_err_to_name(ret));
        i2c_bus_recovered = true;
    }

    i2c_unlock();
    return ret;
}

esp_err_t MY_I2C_Read(i2c_master_dev_handle_t dev,
                        uint8_t reg,
                        uint8_t *data,
                        size_t len)
{
    if (dev == NULL || data == NULL || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(i2c_lock(), TAG, "lock");

    esp_err_t ret = i2c_master_transmit_receive(dev, &reg, 1, data, len, 100);
    if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGE(TAG, "I2C read timeout");
        i2c_bus_recovered = true;
        MY_I2C_RecoverBus();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read fail: %s", esp_err_to_name(ret));
        i2c_bus_recovered = true;
    }

    i2c_unlock();
    return ret;
}

void MY_I2C_ScanList(const uint8_t *addr_list,
                       size_t count)
{
    if (!i2c_bus)
    {
        ESP_LOGW(TAG, "I2C bus not initialized");
        return;
    }

    if (addr_list == NULL || count == 0)
    {
        ESP_LOGW(TAG, "Invalid scan list");
        return;
    }

    for (size_t i = 0; i < count; ++i)
    {
        esp_err_t ret = i2c_master_probe(i2c_bus, addr_list[i],  5);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr_list[i]);
        }
        else
        {
            ESP_LOGW(TAG, "Device not found at 0x%02X", addr_list[i]);
        }
    }
}

i2c_master_bus_handle_t i2c_get_master_bus(void)
{
    return i2c_bus;
}

esp_err_t i2c_write_device(uint8_t dev_addr,
                           const uint8_t *data,
                           size_t len)
{
    if (!i2c_bus || !data || len == 0)
        return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(i2c_lock(), TAG, "lock");

    i2c_master_dev_handle_t dev;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = i2c_clk_hz,
    };

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev);
    if (ret != ESP_OK)
    {
        i2c_unlock();
        return ret;
    }

    ret = i2c_master_transmit(dev, data, len, 100);
    i2c_master_bus_rm_device(dev);
    i2c_unlock();
    return ret;
}

esp_err_t i2c_write_cmd(i2c_master_dev_handle_t dev,
                        uint8_t cmd)
{
    if (dev == NULL)
        return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(i2c_lock(), TAG, "lock");
    esp_err_t ret = i2c_master_transmit(dev, &cmd, 1, 100);
    if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGE(TAG, "I2C cmd timeout");
        i2c_bus_recovered = true;
        MY_I2C_RecoverBus();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C cmd fail: %s", esp_err_to_name(ret));
        i2c_bus_recovered = true;
    }
    i2c_unlock();
    return ret;
}

esp_err_t i2c_read_bytes(i2c_master_dev_handle_t dev,
                         uint8_t *data,
                         size_t len)
{
    if (dev == NULL || data == NULL || len == 0)
        return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(i2c_lock(), TAG, "lock");
    esp_err_t ret = i2c_master_receive(dev, data, len, 100);
    if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGE(TAG, "I2C receive timeout");
        i2c_bus_recovered = true;
        MY_I2C_RecoverBus();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    else if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C receive fail: %s", esp_err_to_name(ret));
        i2c_bus_recovered = true;
    }
    i2c_unlock();
    return ret;
}

esp_err_t MY_I2C_RecoverBus(void)
{
    if (i2c_bus == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "Starting I2C bus recovery");

    /* Use i2c_master_bus_reset() — generates 9 SCL pulses through the I2C
     * peripheral itself. Do NOT use gpio_config() here: reconfiguring the
     * I2C pins via the GPIO driver on ESP-IDF 5.x permanently removes them
     * from the I2C peripheral matrix, causing all subsequent transactions
     * to fail with ESP_ERR_INVALID_STATE. */
    esp_err_t ret = i2c_master_bus_reset(i2c_bus);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed re-init I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGW(TAG, "I2C bus recovered");
    return ESP_OK;
}

bool MY_I2C_NeedsRecovery(void)
{
    return i2c_bus_recovered;
}

void MY_I2C_ClearRecoveryFlag(void)
{
    i2c_bus_recovered = false;
}
