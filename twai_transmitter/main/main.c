#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"

static const char *TAG = "TWAI_TX";

static void init_twai(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_20, GPIO_NUM_21, TWAI_MODE_NORMAL);
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

    ESP_LOGI(TAG, "TWAI transmitter started");

    const char *hello_str = "Hello";

    while (true) {
        twai_message_t message = {
            .identifier = 0x123,
            .data_length_code = 3,
            .data = {0}
        };

        memcpy(message.data, hello_str, message.data_length_code);

        esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(1000));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to transmit message: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Sent message: %.*s", message.data_length_code, hello_str);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
