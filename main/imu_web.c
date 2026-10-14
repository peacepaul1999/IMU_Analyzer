#include "imu_web.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "wifi_ap_manager.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "MPU9250.h"

static const char *TAG = "IMU_WEB";
static httpd_handle_t s_server;

extern const unsigned char index_html_start[] asm("_binary_index_html_start");
extern const unsigned char index_html_end[] asm("_binary_index_html_end");
extern const unsigned char style_css_start[] asm("_binary_style_css_start");
extern const unsigned char style_css_end[] asm("_binary_style_css_end");
extern const unsigned char main_js_start[] asm("_binary_main_js_start");
extern const unsigned char main_js_end[] asm("_binary_main_js_end");

static esp_err_t static_get(httpd_req_t *req)
{
    const unsigned char *start, *end;
    const char *mime;

    if (!strcmp(req->uri, "/") || !strcmp(req->uri, "/index.html"))
    {
        start = index_html_start;
        end = index_html_end;
        mime = "text/html";
    }

    else if (!strcmp(req->uri, "/style.css"))
    {
        start = style_css_start;
        end = style_css_end;
        mime = "text/css";
    }

    else if (!strcmp(req->uri, "/main.js"))
    {
        start = main_js_start;
        end = main_js_end;
        mime = "application/javascript";
    }

    else
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");

    httpd_resp_set_type(req, mime);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    return httpd_resp_send(req, (const char *)start, end - start);
}

static esp_err_t firmware_version_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, firmware_version_get());
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
        return ESP_OK;

    httpd_ws_frame_t frame = {.type = HTTPD_WS_TYPE_TEXT};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);

    if (err != ESP_OK)
        return err;

    if (frame.len == 0 || frame.len > 63 || frame.type != HTTPD_WS_TYPE_TEXT)
        return ESP_OK;

    char cmd[64] = {0};
    frame.payload = (uint8_t *)cmd;
    err = httpd_ws_recv_frame(req, &frame, sizeof(cmd) - 1);

    if (err != ESP_OK)
        return err;

    cmd[frame.len] = '\0';
    /* These MPU driver commands are asynchronous requests to the IMU owner task. */
    if (!strcmp(cmd, "CAL_START"))
        mag_start_calibration();

    else if (!strcmp(cmd, "CAL_STOP"))
        mag_stop_calibration();

    else if (!strcmp(cmd, "SET_NORTH"))
        mpu9250_set_north_offset();

    else
        ESP_LOGW(TAG, "Unknown command: %s", cmd);

    return ESP_OK;
}

typedef struct
{
    char json[512];
} ws_work_t;

static void ws_send_work(void *arg)
{
    ws_work_t *work = (ws_work_t *)arg;

    if (!s_server)
    {
        free(work);
        return;
    }

    size_t n = 8;
    int fds[8];

    if (httpd_get_client_list(s_server, &n, fds) == ESP_OK)
    {
        httpd_ws_frame_t frame = {
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)work->json,
            .len = strlen(work->json)};

        for (size_t i = 0; i < n; i++)
        {
            if (httpd_ws_get_fd_info(s_server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET)
                httpd_ws_send_frame_async(s_server, fds[i], &frame);
        }
    }

    free(work);
}

void imu_web_publish(const imu_gui_sample_t *s)
{
    if (!s_server || !s)
        return;

    ws_work_t *work = malloc(sizeof(*work));

    if (!work)
        return;

    int len = snprintf(work->json, sizeof(work->json),
                       "{\"yaw\":%.3f,\"roll\":%.3f,\"pitch\":%.3f,"
                       "\"acc\":[%.4f,%.4f,%.4f],\"gyro\":[%.4f,%.4f,%.4f],"
                       "\"mag\":[%.4f,%.4f,%.4f],\"ready\":%s,\"calibrating\":%s,\"northReady\":%s,"
                       "\"t\":%lld,\"imuHz\":%d,\"wsHz\":%d}",
                       s->yaw, s->roll, s->pitch, s->acc[0], s->acc[1], s->acc[2],
                       s->gyro[0], s->gyro[1], s->gyro[2], s->mag[0], s->mag[1], s->mag[2],
                       s->ready ? "true" : "false", s->calibrating ? "true" : "false", s->north_ready ? "true" : "false",
                       (long long)(esp_timer_get_time() / 1000), IMU_SAMPLE_RATE_HZ, IMU_WS_RATE_HZ);

    if (len <= 0 || len >= sizeof(work->json))
    {
        free(work);
        return;
    }

    if (httpd_queue_work(s_server, ws_send_work, work) != ESP_OK)
        free(work);
}

esp_err_t imu_web_start(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if (err != ESP_OK)
        return err;

    const wifi_ap_manager_config_t wifi_cfg = {
        .ssid = AP_SSID,
        .password = AP_PASSWORD,
        .channel = 1,
        .max_connection = 4,
        .client_event_cb = NULL,
        .user_data = NULL,
    };

    err = wifi_ap_manager_start(&wifi_cfg);

    if (err != ESP_OK)
        return err;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 7;
    config.max_uri_handlers = 6;

    err = httpd_start(&s_server, &config);
    if (err != ESP_OK)
        return err;

    const httpd_uri_t pages[] = {
        {.uri = "/", .method = HTTP_GET, .handler = static_get},
        {.uri = "/index.html", .method = HTTP_GET, .handler = static_get},
        {.uri = "/style.css", .method = HTTP_GET, .handler = static_get},
        {.uri = "/main.js", .method = HTTP_GET, .handler = static_get},
        {.uri = "/version", .method = HTTP_GET, .handler = firmware_version_get_handler},
        {.uri = "/ws", .method = HTTP_GET, .handler = ws_handler, .is_websocket = true}};

    for (size_t i = 0; i < sizeof(pages) / sizeof(pages[0]); i++)
        ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &pages[i]));

    ESP_LOGI(TAG, "Wi-Fi AP: %s / http://%s", wifi_ap_manager_get_ssid(), wifi_ap_manager_get_ip_address());

    return ESP_OK;
}
