#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"

// Pin Definitions based on uploaded files
#define TX_GPIO_NUM 20
#define RX_GPIO_NUM 21

static const char *TAG = "SLCAN";

// Setup TWAI (CAN) Driver
void setup_twai_driver() {
    // Initialize configuration structures using helper macros
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
    
    // RobStride motors operate at 1Mbps
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        ESP_LOGI(TAG, "Driver installed");
    } else {
        ESP_LOGE(TAG, "Failed to install driver");
        return;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK) {
        ESP_LOGI(TAG, "Driver started");
    } else {
        ESP_LOGE(TAG, "Failed to start driver");
    }
}

// Task to read CAN frames and send to USB (SLCAN format)
void rx_task(void *arg) {
    twai_message_t rx_msg;
    char slcan_pkt[30];
    
    while (1) {
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
            // Format: Tiiiiiidd... (T = Extended ID, i = ID, l = len, d = data)
            // RobStride uses Extended Frames (29-bit)
            if (rx_msg.extd) {
                int written = sprintf(slcan_pkt, "T%08lX%d", rx_msg.identifier, rx_msg.data_length_code);
                for (int i = 0; i < rx_msg.data_length_code; i++) {
                    written += sprintf(slcan_pkt + written, "%02X", rx_msg.data[i]);
                }
                sprintf(slcan_pkt + written, "\r");
                
                // Send to USB
                printf("%s", slcan_pkt);
                fflush(stdout);
            }
        }
    }
}

// Helper to parse hex char
uint8_t hex2int(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

// Task to read USB and send to CAN
void usb_task(void *arg) {
    char line[64];
    while (1) {
        // Read line from USB Serial
        char *pos = fgets(line, sizeof(line), stdin);
        if (pos != NULL) {
            // Basic SLCAN parsing for Transmit (t/T)
            // Format: Tiiiiiidd... (Extended) or tiiidd... (Standard)
            twai_message_t tx_msg = {0};
            size_t len = strlen(line);
            
            // Remove newline
            if (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n')) {
                line[len-1] = 0;
                len--;
            }

            if (line[0] == 'T') { // Extended Frame (29-bit) - RobStride uses this
                tx_msg.extd = 1;
                char id_str[9] = {0};
                memcpy(id_str, &line[1], 8);
                tx_msg.identifier = strtoul(id_str, NULL, 16);
                
                tx_msg.data_length_code = line[9] - '0';
                
                for (int i = 0; i < tx_msg.data_length_code; i++) {
                    char byte_str[3] = {0};
                    memcpy(byte_str, &line[10 + i*2], 2);
                    tx_msg.data[i] = (uint8_t)strtoul(byte_str, NULL, 16);
                }
                
                twai_transmit(&tx_msg, pdMS_TO_TICKS(10));
            }
            // Handle 'O' (Open), 'C' (Close), 'S' (Speed) commands usually sent by slcand
            // Since we hardcoded speed, we can mostly ignore or just acknowledge
            else if (line[0] == 'V') { printf("V0101\r"); } // Version
            else if (line[0] == 'v') { printf("v0101\r"); }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void) {
    // Setup USB Serial JTAG as console
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .tx_buffer_size = 256,
        .rx_buffer_size = 256,
    };
    usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    esp_vfs_usb_serial_jtag_use_driver();
    esp_vfs_dev_uart_register(); // Optional, map UART to VFS for standard I/O

    // Disable buffering on stdout
    setvbuf(stdout, NULL, _IONBF, 0);

    setup_twai_driver();

    xTaskCreate(rx_task, "rx_task", 4096, NULL, 10, NULL);
    xTaskCreate(usb_task, "usb_task", 4096, NULL, 10, NULL);
}