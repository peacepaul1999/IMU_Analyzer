#include "wifi_ap_manager.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "lwip/ip4_addr.h"

/* =========================================================
   CONFIG
========================================================= */

static const char *TAG = "WIFI_AP";

/* =========================================================
   GLOBAL
========================================================= */

static wifi_ap_manager_config_t s_config = {};

static esp_netif_t *s_wifi_ap_netif = NULL;

static esp_event_handler_instance_t
    s_wifi_event_instance = NULL;

static bool s_wifi_event_registered = false;

static bool s_wifi_initialized = false;
static bool s_ap_started = false;

static int s_connected_clients = 0;

static uint64_t s_last_active_ms = 0;
static char s_active_ssid[33] = {};

/* =========================================================
   CLIENT EVENT
========================================================= */

static void wifi_ap_client_event_handler(
    bool connected,
    void *user_data)
{
    (void)user_data;

    if (connected)
    {
        s_connected_clients++;

        s_last_active_ms =
            esp_timer_get_time() / 1000ULL;

        ESP_LOGI(TAG,
                 "Client connected (%d)",
                 s_connected_clients);

        return;
    }

    s_connected_clients--;

    if (s_connected_clients < 0)
    {
        s_connected_clients = 0;
    }

    ESP_LOGI(TAG,
             "Client disconnected (%d)",
             s_connected_clients);
}

/* =========================================================
   WIFI EVENT
========================================================= */

static void wifi_ap_event_handler(
    void *arg,
    esp_event_base_t base,
    int32_t id,
    void *data)
{
    (void)arg;
    (void)data;

    if (base != WIFI_EVENT)
    {
        return;
    }

    switch (id)
    {
    case WIFI_EVENT_AP_STACONNECTED:

        ESP_LOGI(TAG, "STA CONNECTED");

        if (s_config.client_event_cb)
        {
            s_config.client_event_cb(
                true,
                s_config.user_data);
        }

        break;

    case WIFI_EVENT_AP_STADISCONNECTED:

        ESP_LOGI(TAG, "STA DISCONNECTED");

        if (s_config.client_event_cb)
        {
            s_config.client_event_cb(
                false,
                s_config.user_data);
        }

        break;

    default:
        break;
    }
}

/* =========================================================
   START AP
========================================================= */

esp_err_t wifi_ap_manager_start(
    const wifi_ap_manager_config_t *config)
{
    if (config == NULL ||
        config->ssid == NULL ||
        config->password == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(config->password) > 0 &&
        strlen(config->password) < 8)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ap_started)
    {
        if (strncmp(s_active_ssid, config->ssid, sizeof(s_active_ssid)) == 0)
        {
            ESP_LOGI(TAG,
                     "SoftAP already running with SSID: %s",
                     s_active_ssid);

            return ESP_OK;
        }

        ESP_LOGW(TAG,
                 "SoftAP already running as %s, keeping existing SSID and skipping restart request for %s",
                 s_active_ssid,
                 config->ssid);

        return ESP_OK;
    }

    /* =====================================================
       NETIF
    ===================================================== */

    esp_err_t err = esp_netif_init();

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    /* =====================================================
       EVENT LOOP
    ===================================================== */

    err = esp_event_loop_create_default();

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    /* =====================================================
       CREATE AP
    ===================================================== */

    if (s_wifi_ap_netif == NULL)
    {
        s_wifi_ap_netif = esp_netif_create_default_wifi_ap();

        if (s_wifi_ap_netif == NULL)
        {
            return ESP_FAIL;
        }

        /* =================================================
           STATIC IP
        ================================================= */

        esp_netif_ip_info_t ip_info = {};

        IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);

        IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);

        IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

        ESP_ERROR_CHECK(esp_netif_dhcps_stop(s_wifi_ap_netif));
        ESP_ERROR_CHECK(esp_netif_set_ip_info(s_wifi_ap_netif, &ip_info));
        ESP_ERROR_CHECK(esp_netif_dhcps_start(s_wifi_ap_netif));

        ESP_LOGI(TAG, "Static IP: %d.%d.%d.%d", IP2STR(&ip_info.ip));
    }

    /* =====================================================
       WIFI INIT
    ===================================================== */

    if (!s_wifi_initialized)
    {
        wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
        err = esp_wifi_init(&init_config);

        if (err != ESP_OK)
        {
            return err;
        }

        s_wifi_initialized = true;
    }

    s_config = *config;

    /* =====================================================
       EVENT REGISTER
    ===================================================== */

    if (!s_wifi_event_registered)
    {
        err =
            esp_event_handler_instance_register(
                WIFI_EVENT,
                ESP_EVENT_ANY_ID,
                &wifi_ap_event_handler,
                NULL,
                &s_wifi_event_instance);

        if (err != ESP_OK)
        {
            return err;
        }

        s_wifi_event_registered = true;
    }

    /* =====================================================
       WIFI CONFIG
    ===================================================== */

    wifi_config_t wifi_config = {};

    snprintf((char *)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s", config->ssid);

    snprintf((char *)wifi_config.ap.password, sizeof(wifi_config.ap.password), "%s", config->password);

    wifi_config.ap.ssid_len = strlen((char *)wifi_config.ap.ssid);

    wifi_config.ap.authmode =
        strlen((char *)wifi_config.ap.password) == 0
            ? WIFI_AUTH_OPEN
            : WIFI_AUTH_WPA2_PSK;

    wifi_config.ap.max_connection = config->max_connection;
    wifi_config.ap.channel = config->channel;

    /* =====================================================
       AP MODE
    ===================================================== */

    err = esp_wifi_set_mode(WIFI_MODE_AP);

    if (err != ESP_OK)
    {
        return err;
    }

    /* =====================================================
       SET CONFIG
    ===================================================== */

    err =
        esp_wifi_set_config(
            WIFI_IF_AP,
            &wifi_config);

    if (err != ESP_OK)
    {
        return err;
    }

    /* =====================================================
       START WIFI
    ===================================================== */

    err = esp_wifi_start();

    if (err != ESP_OK)
    {
        return err;
    }

    /* =====================================================
       HOSTNAME
    ===================================================== */

    esp_netif_set_hostname(s_wifi_ap_netif, "imu-analyzer");

    /* =====================================================
       INFO
    ===================================================== */

    esp_netif_ip_info_t current_ip = {};

    esp_netif_get_ip_info(s_wifi_ap_netif, &current_ip);

    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "IMU ANALYZER AP STARTED");
    ESP_LOGI(TAG, "SSID : %s", (char *)wifi_config.ap.ssid);
    ESP_LOGI(TAG, "PASS : %s", (char *)wifi_config.ap.password);
    ESP_LOGI(TAG, "WEB  : http://" IPSTR, IP2STR(&current_ip.ip));
    ESP_LOGI(TAG, "================================");

    snprintf(s_active_ssid, sizeof(s_active_ssid), "%s", (char *)wifi_config.ap.ssid);
    s_ap_started = true;

    return ESP_OK;
}

/* =========================================================
   STATUS
========================================================= */

bool wifi_ap_manager_is_client_connected(void)
{
    return s_connected_clients > 0;
}

const char *wifi_ap_manager_get_ip_address(void)
{
    static char ip_address[16] = {};

    if (s_wifi_ap_netif == NULL)
    {
        return AP_DEFAULT_IP;
    }

    esp_netif_ip_info_t current_ip = {};
    esp_err_t err = esp_netif_get_ip_info(s_wifi_ap_netif, &current_ip);

    if (err != ESP_OK)
    {
        return AP_DEFAULT_IP;
    }

    snprintf(ip_address, sizeof(ip_address), IPSTR, IP2STR(&current_ip.ip));

    return ip_address;
}

const char *wifi_ap_manager_get_ssid(void)
{
    return (s_config.ssid != NULL) ? s_config.ssid : AP_SSID;
}

int wifi_ap_manager_get_connected_clients(void)
{
    return s_connected_clients;
}

uint64_t wifi_ap_manager_get_last_active_time_ms(void)
{
    return s_last_active_ms;
}