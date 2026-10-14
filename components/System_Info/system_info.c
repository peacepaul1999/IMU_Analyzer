#include "system_info.h"

#include <stdio.h>
#include "esp_mac.h"
#include "esp_log.h"

#define TAG "SYS_INFO"

void print_mac_address(void)
{
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

const char *get_serial_number(void)
{
  static char serialStr[20];
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(serialStr, sizeof(serialStr), "IMU_Analyzer_%02X%02X", mac[4], mac[5]);
  ESP_LOGI(TAG, "Serial Number: %s", serialStr);
  return serialStr;
}
