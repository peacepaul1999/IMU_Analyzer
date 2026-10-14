#pragma once

#include "driver/gpio.h"
/* ESP32-S3: GPIO22 does not exist. Modify to match your wiring. */
#define SDA_PIN GPIO_NUM_8
#define SCL_PIN GPIO_NUM_9

#define I2C_PORT_NUM I2C_NUM_0
#define I2C_CLK_SPEED_HZ 400000
#define I2C_TIMEOUT_MS 100
