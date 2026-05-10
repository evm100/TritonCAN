/*
 * Triton Droids — RobStride diagnostic dongle (V1, observe-only).
 *
 *   ESP32-S3-DevKitC-1 + SN65HVD230 + SSD1306 128x32 + onboard WS2812.
 *   Hosts a WiFi AP and serves a single-page diagnostic dashboard.
 *
 *   V1 invariant: only Type-0 (GET_DEVICE_ID) and Type-17 (READ_PARAMETER)
 *   are ever transmitted. There is no code path that writes parameters,
 *   enables, stops, or commands motion. Motion tests (Tier 3) are
 *   intentionally absent and will be added when E-stop hardware lands.
 */

#include "state.h"
#include "can_task.h"
#include "web_task.h"
#include "oled.h"
#include "argb.h"
#include "battery.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

static void oled_refresh_task(void *arg) {
    while (1) {
        oled_refresh_from_state(web_ap_ssid());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Triton Diag Dongle V1 booting");

    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    state_init();
    argb_start();          // breathing white while we bring everything up

    bool oled_ok = oled_init();
    if (!oled_ok) ESP_LOGW(TAG, "OLED missing or wiring wrong — continuing without it");

    battery_start();
    web_task_start();
    can_task_start();

    if (oled_ok) {
        xTaskCreate(oled_refresh_task, "oled", 4096, NULL, 3, NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(800));  // let services settle
    argb_set_booting(false);
    ESP_LOGI(TAG, "Boot complete -- connect to AP %s", web_ap_ssid());
}
