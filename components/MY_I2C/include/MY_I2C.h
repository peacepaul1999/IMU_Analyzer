#pragma once

#ifndef MY_I2C_H
#define MY_I2C_H

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

esp_err_t i2c_master_init(gpio_num_t sda_io, gpio_num_t scl_io, i2c_port_t port , uint32_t clk_speed_hz);

esp_err_t i2c_write_reg(uint8_t dev_addr,
                        uint8_t reg_addr,
                        const uint8_t *data,
                        size_t len);

esp_err_t i2c_read_reg(uint8_t dev_addr,
                       uint8_t reg_addr,
                       uint8_t *data,
                       size_t len);

void i2c_scan_devices(void);

esp_err_t i2c_add_device(uint8_t dev_addr,
                         i2c_master_dev_handle_t *dev_handle);

esp_err_t MY_I2C_Read(i2c_master_dev_handle_t dev,
                        uint8_t reg,
                        uint8_t *data,
                        size_t len);

esp_err_t MY_I2C_Write(i2c_master_dev_handle_t dev,
                         uint8_t reg,
                         const uint8_t *data,
                         size_t len);

void MY_I2C_ScanList(const uint8_t *addr_list,
                       size_t count);

bool MY_I2C_NeedsRecovery(void);

void MY_I2C_ClearRecoveryFlag(void);

esp_err_t MY_I2C_RecoverBus(void);

i2c_master_bus_handle_t i2c_get_master_bus(void);

esp_err_t i2c_write_device(uint8_t dev_addr,
                           const uint8_t *data,
                           size_t len);

esp_err_t i2c_write_cmd(i2c_master_dev_handle_t dev,
                        uint8_t cmd);

esp_err_t i2c_read_bytes(i2c_master_dev_handle_t dev,
                         uint8_t *data,
                         size_t len);

#endif // MY_I2C_H