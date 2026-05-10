#include "web_task.h"
#include "state.h"
#include "can_task.h"
#include "robstride.h"
#include "sdkconfig.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/param.h>

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "cJSON.h"

static const char *TAG = "web";

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

static char s_ssid[32];
static httpd_handle_t s_httpd = NULL;
static esp_netif_t   *s_netif_ap = NULL;
static int s_ap_ip[4] = {192,168,4,1};

#define MAX_WS_CLIENTS 4
static int s_ws_fds[MAX_WS_CLIENTS];
static SemaphoreHandle_t s_ws_mutex;

const char *web_ap_ssid(void) { return s_ssid; }

// ---- DNS captive portal -----------------------------------------------------

static void dns_task(void *arg) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { ESP_LOGE(TAG,"dns socket"); vTaskDelete(NULL); return; }
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(53),
                                .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG,"dns bind"); close(sock); vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG,"DNS captive portal listening");
    uint8_t buf[512];
    while (1) {
        struct sockaddr_in src; socklen_t slen = sizeof(src);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr*)&src, &slen);
        if (n < 12) continue;
        // Build a minimal response that points every A query to our AP IP.
        // Flags: response, authoritative, no error.
        buf[2] = 0x81; buf[3] = 0x80;
        // ANCOUNT = 1
        buf[6] = 0; buf[7] = 1;
        // NSCOUNT = ARCOUNT = 0
        buf[8] = 0; buf[9] = 0; buf[10] = 0; buf[11] = 0;
        // Skip question name + qtype + qclass to find end of question.
        int p = 12;
        while (p < n && buf[p] != 0) p += buf[p] + 1;
        p += 1 + 4;  // null + qtype(2) + qclass(2)
        if (p + 16 > (int)sizeof(buf)) continue;
        // Answer: pointer to name (0xc00c), type A, class IN, TTL 60, RDLEN 4, IP
        buf[p++] = 0xC0; buf[p++] = 0x0C;
        buf[p++] = 0;    buf[p++] = 1;
        buf[p++] = 0;    buf[p++] = 1;
        buf[p++] = 0;    buf[p++] = 0; buf[p++] = 0; buf[p++] = 60;
        buf[p++] = 0;    buf[p++] = 4;
        buf[p++] = s_ap_ip[0]; buf[p++] = s_ap_ip[1];
        buf[p++] = s_ap_ip[2]; buf[p++] = s_ap_ip[3];
        sendto(sock, buf, p, 0, (struct sockaddr*)&src, slen);
    }
}

// ---- HTTP handlers ----------------------------------------------------------

static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, index_html_start, index_html_end - index_html_start);
}

static esp_err_t captive_redirect(httpd_req_t *req) {
    httpd_resp_set_status(req, "302 Found");
    char loc[64];
    snprintf(loc, sizeof(loc), "http://%d.%d.%d.%d/",
             s_ap_ip[0], s_ap_ip[1], s_ap_ip[2], s_ap_ip[3]);
    httpd_resp_set_hdr(req, "Location", loc);
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t run_test_handler(httpd_req_t *req, diag_req_kind_t kind) {
    char *json = NULL;
    esp_err_t err = diag_request(kind, &json, 30000);
    if (err != ESP_OK || !json) {
        const char *msg = "{\"ok\":false,\"verdict\":\"test runner busy or timed out\"}";
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, msg, strlen(msg));
    }
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, json, strlen(json));
    free(json);
    return r;
}

static esp_err_t tier0_handler (httpd_req_t *r) { return run_test_handler(r, DIAG_REQ_TIER0);  }
static esp_err_t tier1_handler (httpd_req_t *r) { return run_test_handler(r, DIAG_REQ_TIER1);  }
static esp_err_t tier2_handler (httpd_req_t *r) { return run_test_handler(r, DIAG_REQ_TIER2);  }
static esp_err_t all_handler   (httpd_req_t *r) { return run_test_handler(r, DIAG_REQ_ALL);    }
static esp_err_t rescan_handler(httpd_req_t *r) { return run_test_handler(r, DIAG_REQ_RESCAN); }

// ---- WebSocket --------------------------------------------------------------

static void ws_add(int fd) {
    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (s_ws_fds[i] == fd) { xSemaphoreGive(s_ws_mutex); return; }
    }
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (s_ws_fds[i] == 0) { s_ws_fds[i] = fd; break; }
    }
    xSemaphoreGive(s_ws_mutex);
}
static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        // Handshake complete; register the client.
        int fd = httpd_req_to_sockfd(req);
        ws_add(fd);
        ESP_LOGI(TAG, "WS client connected, fd=%d", fd);
        return ESP_OK;
    }
    httpd_ws_frame_t frame = {0};
    frame.type = HTTPD_WS_TYPE_TEXT;
    // Receive (and discard) -- we don't expect inbound messages in V1.
    if (httpd_ws_recv_frame(req, &frame, 0) == ESP_OK && frame.len) {
        frame.payload = malloc(frame.len + 1);
        if (frame.payload) {
            httpd_ws_recv_frame(req, &frame, frame.len);
            free(frame.payload);
        }
    }
    return ESP_OK;
}

// ---- Telemetry broadcaster --------------------------------------------------

static char *build_telem_json(void) {
    motor_state_t   ms;  state_get_motor(&ms);
    bus_stats_t     bs;  state_get_bus(&bs);
    battery_state_t ba;  state_get_battery(&ba);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "telem");
    cJSON_AddStringToObject(root, "ap",   s_ssid);
    char ip[16]; snprintf(ip, sizeof(ip), "%d.%d.%d.%d",
                          s_ap_ip[0], s_ap_ip[1], s_ap_ip[2], s_ap_ip[3]);
    cJSON_AddStringToObject(root, "ip", ip);

    cJSON *mt = cJSON_CreateObject();
    cJSON_AddBoolToObject  (mt, "detected", ms.detected);
    if (ms.detected) {
        cJSON_AddNumberToObject(mt, "id",   (double)ms.motor_id);
        char hex[20]; snprintf(hex, sizeof(hex), "%02X%02X%02X%02X%02X%02X%02X%02X",
            ms.uid[0],ms.uid[1],ms.uid[2],ms.uid[3],
            ms.uid[4],ms.uid[5],ms.uid[6],ms.uid[7]);
        cJSON_AddStringToObject(mt, "uid", hex);
        cJSON_AddNumberToObject(mt, "vbus", (double)ms.vbus_v);
        cJSON_AddNumberToObject(mt, "pos",  (double)ms.mech_pos_rad);
        cJSON_AddNumberToObject(mt, "vel",  (double)ms.mech_vel_rps);
        cJSON_AddNumberToObject(mt, "iq",   (double)ms.iq_a);
        cJSON_AddNumberToObject(mt, "temp", (double)ms.temperature_c);
        cJSON_AddNumberToObject(mt, "run_mode",    (double)ms.run_mode);
        cJSON_AddNumberToObject(mt, "fault_bits",  (double)ms.fb_fault_bits);
        cJSON_AddNumberToObject(mt, "faults",      (double)ms.faults);
        cJSON_AddNumberToObject(mt, "warnings",    (double)ms.warnings);
        cJSON_AddNumberToObject(mt, "limit_torque",(double)ms.limit_torque);
        cJSON_AddNumberToObject(mt, "limit_spd",   (double)ms.limit_spd);
        cJSON_AddNumberToObject(mt, "limit_cur",   (double)ms.limit_cur);
    }
    cJSON_AddItemToObject(root, "motor", mt);

    cJSON *bj = cJSON_CreateObject();
    cJSON_AddNumberToObject(bj, "alerts",     (double)bs.cum_alerts);
    cJSON_AddNumberToObject(bj, "rx",         (double)bs.rx_total);
    cJSON_AddNumberToObject(bj, "rx_unknown", (double)bs.rx_unknown_type);
    cJSON_AddItemToObject(root, "bus", bj);

    cJSON *batj = cJSON_CreateObject();
    cJSON_AddNumberToObject(batj, "mv",  (double)ba.vbat_mv);
    cJSON_AddNumberToObject(batj, "pct", (double)ba.percent);
    cJSON_AddBoolToObject  (batj, "ok",  ba.ok);
    cJSON_AddItemToObject(root, "battery", batj);

    cJSON_AddNumberToObject(root, "ts_ms", (double)(esp_timer_get_time() / 1000));
    char *s = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return s;
}

static void ws_send_text_to_fd(int fd, const char *text, size_t len) {
    httpd_ws_frame_t f = {
        .final = true,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)text,
        .len = len,
    };
    httpd_ws_send_frame_async(s_httpd, fd, &f);
}

static int s_last_test_gen = 0;

static void broadcast_task(void *arg) {
    while (1) {
        // 1. Telemetry @ 5 Hz
        char *t = build_telem_json();
        if (t) {
            size_t len = strlen(t);
            xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
            for (int i = 0; i < MAX_WS_CLIENTS; i++) {
                if (s_ws_fds[i]) ws_send_text_to_fd(s_ws_fds[i], t, len);
            }
            xSemaphoreGive(s_ws_mutex);
            free(t);
        }
        // 2. Forward latest test result if it changed.
        int g = state_get_test_generation();
        if (g != s_last_test_gen) {
            s_last_test_gen = g;
            char *buf = malloc(8192);
            if (buf) {
                int n = state_copy_test_result(buf, 8192);
                if (n > 0) {
                    char *wrapped = malloc(n + 64);
                    if (wrapped) {
                        int wn = snprintf(wrapped, n + 64,
                                          "{\"type\":\"test_result\",%s",
                                          buf[0] == '{' ? buf + 1 : buf);
                        xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
                        for (int i = 0; i < MAX_WS_CLIENTS; i++) {
                            if (s_ws_fds[i]) ws_send_text_to_fd(s_ws_fds[i], wrapped, wn);
                        }
                        xSemaphoreGive(s_ws_mutex);
                        free(wrapped);
                    }
                }
                free(buf);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ---- HTTP startup -----------------------------------------------------------

static const httpd_uri_t s_uris[] = {
    { .uri="/",                          .method=HTTP_GET,  .handler=index_handler },
    { .uri="/generate_204",              .method=HTTP_GET,  .handler=captive_redirect },
    { .uri="/gen_204",                   .method=HTTP_GET,  .handler=captive_redirect },
    { .uri="/hotspot-detect.html",       .method=HTTP_GET,  .handler=captive_redirect },
    { .uri="/connecttest.txt",           .method=HTTP_GET,  .handler=captive_redirect },
    { .uri="/ncsi.txt",                  .method=HTTP_GET,  .handler=captive_redirect },
    { .uri="/api/test/tier0",            .method=HTTP_POST, .handler=tier0_handler  },
    { .uri="/api/test/tier1",            .method=HTTP_POST, .handler=tier1_handler  },
    { .uri="/api/test/tier2",            .method=HTTP_POST, .handler=tier2_handler  },
    { .uri="/api/test/all",              .method=HTTP_POST, .handler=all_handler    },
    { .uri="/api/test/rescan",           .method=HTTP_POST, .handler=rescan_handler },
};

static esp_err_t catch_all_404(httpd_req_t *req, httpd_err_code_t err) {
    return captive_redirect(req);
}

static void start_httpd(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 16;
    cfg.lru_purge_enable = true;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed"); return;
    }
    for (size_t i = 0; i < sizeof(s_uris)/sizeof(s_uris[0]); i++)
        httpd_register_uri_handler(s_httpd, &s_uris[i]);
    httpd_uri_t ws = { .uri="/ws", .method=HTTP_GET, .handler=ws_handler,
                       .user_ctx=NULL, .is_websocket=true, .handle_ws_control_frames=false };
    httpd_register_uri_handler(s_httpd, &ws);
    httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, catch_all_404);
    ESP_LOGI(TAG, "HTTP server up on http://%d.%d.%d.%d/",
             s_ap_ip[0], s_ap_ip[1], s_ap_ip[2], s_ap_ip[3]);
}

// ---- WiFi softAP -----------------------------------------------------------

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGI(TAG, "STA disconnected: " MACSTR, MAC2STR(e->mac));
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "STA connected: " MACSTR, MAC2STR(e->mac));
    }
}

static void start_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_netif_ap = esp_netif_create_default_wifi_ap();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
                        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL, NULL));

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_ssid, sizeof(s_ssid), "TritonDiag-%02X%02X", mac[4], mac[5]);

    wifi_config_t ap = {0};
    strncpy((char*)ap.ap.ssid, s_ssid, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(s_ssid);
    strncpy((char*)ap.ap.password, CONFIG_DIAG_AP_PASSWORD, sizeof(ap.ap.password));
    ap.ap.max_connection = 4;
    ap.ap.authmode = strlen(CONFIG_DIAG_AP_PASSWORD) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ap.ap.channel = 6;
    ap.ap.beacon_interval = 100;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ipinfo;
    esp_netif_get_ip_info(s_netif_ap, &ipinfo);
    s_ap_ip[0] = (ipinfo.ip.addr      ) & 0xFF;
    s_ap_ip[1] = (ipinfo.ip.addr >>  8) & 0xFF;
    s_ap_ip[2] = (ipinfo.ip.addr >> 16) & 0xFF;
    s_ap_ip[3] = (ipinfo.ip.addr >> 24) & 0xFF;
    ESP_LOGI(TAG, "AP up: SSID=%s pwd=%s IP=%d.%d.%d.%d",
             s_ssid, CONFIG_DIAG_AP_PASSWORD,
             s_ap_ip[0], s_ap_ip[1], s_ap_ip[2], s_ap_ip[3]);
}

void web_task_start(void) {
    s_ws_mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < MAX_WS_CLIENTS; i++) s_ws_fds[i] = 0;
    start_softap();
    start_httpd();
    xTaskCreatePinnedToCore(dns_task,       "dns",  3072, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(broadcast_task, "wsbc", 6144, NULL, 5, NULL, 0);
}
