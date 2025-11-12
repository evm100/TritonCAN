#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"

static const char *TAG = "TWAI_RX";

static void init_twai(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_20, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

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
}

void app_main(void)
{
    init_twai();

    ESP_LOGI(TAG, "TWAI receiver started");

    while (true) {
        twai_message_t message;
        esp_err_t err = twai_receive(&message, portMAX_DELAY);
        if (err == ESP_OK) {
            char rx_str[9] = {0};
            size_t copy_len = message.data_length_code;
            if (copy_len > sizeof(rx_str) - 1) {
                copy_len = sizeof(rx_str) - 1;
            }
            memcpy(rx_str, message.data, copy_len);
            ESP_LOGI(TAG, "Received message ID=0x%03X DLC=%d Data='%s'", message.identifier, message.data_length_code, rx_str);
        } else {
            ESP_LOGE(TAG, "Failed to receive message: %s", esp_err_to_name(err));
        }
    }
}
