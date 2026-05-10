#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"

static const char *TAG = "TWAI_RX";

// Configure CAN pins & bitrate
#define CAN_TX_PIN   GPIO_NUM_20    // (if dual-board you might swap TX/RX)
#define CAN_RX_PIN   GPIO_NUM_21
#define CAN_BITRATE  TWAI_TIMING_CONFIG_1MBITS()  // match your bus

static void init_twai_listener(void)
{
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = CAN_BITRATE;
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(err));
        abort();
    }
    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(err));
        abort();
    }

    ESP_LOGI(TAG, "TWAI listener started");
}

void app_main(void)
{
    init_twai_listener();

    while (true) {
        twai_message_t msg;
        esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(1000));
        if (err == ESP_OK) {
            // Print out the frame
            char buf[64];
            int pos = snprintf(buf, sizeof(buf), "ID=0x%03lX DLC=%u Data: ",
                            (unsigned long)msg.identifier,
                            msg.data_length_code);

            for (int i = 0; i < msg.data_length_code; i++) {
                pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X ", msg.data[i]);
            }

            ESP_LOGI(TAG, "%s", buf);
        }
        // else if timeout, just loop again
    }
}
