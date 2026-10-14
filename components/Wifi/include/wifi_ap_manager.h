#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define AP_SSID "IMU-Analyzer"
#define AP_PASSWORD "12345678" // Change before deploying in public spaces.
#define AP_DEFAULT_IP "192.168.4.1"

typedef void (*wifi_ap_client_event_callback_t)(bool connected, void *user_data);

typedef struct
{
    const char *ssid;
    const char *password;
    uint8_t channel;
    uint8_t max_connection;
    wifi_ap_client_event_callback_t client_event_cb;
    void *user_data;
} wifi_ap_manager_config_t;

esp_err_t wifi_ap_manager_start(const wifi_ap_manager_config_t *config);
bool wifi_ap_manager_is_client_connected(void);
int wifi_ap_manager_get_connected_clients(void);
uint64_t wifi_ap_manager_get_last_active_time_ms(void);
const char *wifi_ap_manager_get_ip_address(void);
const char *wifi_ap_manager_get_ssid(void);
