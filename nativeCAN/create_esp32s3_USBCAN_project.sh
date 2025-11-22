#!/bin/bash

PROJECT_NAME="esp32_s3_can_v32_stable"

if [ -d "$PROJECT_NAME" ]; then rm -rf $PROJECT_NAME; fi
mkdir -p $PROJECT_NAME/main

echo "Creating v32 (Stable Production - Reverted Optimizations)..."

# 1. CMakeLists
cat <<EOF > $PROJECT_NAME/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
include(\$ENV{IDF_PATH}/tools/cmake/project.cmake)
project($PROJECT_NAME)
EOF

# 2. Manifest
cat <<EOF > $PROJECT_NAME/main/idf_component.yml
dependencies:
  espressif/tinyusb: "^0.15.0"
EOF

# 3. sdkconfig
# REVERTED: Back to standard 100Hz for maximum stability (Matches v30)
cat <<EOF > $PROJECT_NAME/sdkconfig.defaults
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=n
CONFIG_ESP_CONSOLE_UART_DEFAULT=y
CONFIG_TINYUSB=y
CONFIG_TINYUSB_ENABLED=y
CONFIG_ESP32S3_INTR_ALLOC_OPTIONS=y
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
EOF

# 4. main/CMakeLists
cat <<EOF > $PROJECT_NAME/main/CMakeLists.txt
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer tinyusb esp_phy usb freertos)
idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
target_include_directories(\${tusb_lib} PRIVATE ".")
EOF

# 5. tusb_config.h
cat <<EOF > $PROJECT_NAME/main/tusb_config.h
#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_
#ifdef __cplusplus
extern "C" {
#endif
#define CFG_TUSB_MCU               OPT_MCU_ESP32S3
#define CFG_TUSB_RHPORT0_MODE      OPT_MODE_DEVICE
#define CFG_TUSB_OS                OPT_OS_FREERTOS
#define CFG_TUD_ENABLED            1
#define CFG_TUD_ENDPOINT0_SIZE     64
#define CFG_TUD_VENDOR             1
#define CFG_TUD_VENDOR_RX_BUFSIZE  1024
#define CFG_TUD_VENDOR_TX_BUFSIZE  4096 
#define CFG_TUD_CONTROL_COMPLETE_CALLBACK 1
#ifdef __cplusplus
}
#endif
#endif
EOF

# 6. gs_usb.h
cat <<EOF > $PROJECT_NAME/main/gs_usb.h
#pragma once
#include <stdint.h>
#define GS_USB_BREQ_HOST_FORMAT 0
#define GS_USB_BREQ_BITTIMING 1
#define GS_USB_BREQ_MODE 2
#define GS_USB_BREQ_BT_CONST 4
#define GS_USB_BREQ_DEVICE_CONFIG 5
#define GS_CAN_MODE_RESET 0
#define GS_CAN_MODE_START 1
#pragma pack(push, 1)
struct gs_host_config { uint32_t byte_order; };
struct gs_device_config { 
    uint8_t reserved1; uint8_t reserved2; uint8_t reserved3; 
    uint8_t icount; 
    uint32_t sw_version; 
    uint32_t hw_version; 
};
struct gs_device_bittiming { 
    uint32_t prop_seg; uint32_t phase_seg1; uint32_t phase_seg2; 
    uint32_t sjw; uint32_t brp; 
};
struct gs_device_bt_const { 
    uint32_t feature; 
    uint32_t fclk_can; 
    uint32_t tseg1_min; uint32_t tseg1_max; 
    uint32_t tseg2_min; uint32_t tseg2_max; 
    uint32_t sjw_max; 
    uint32_t brp_min; uint32_t brp_max; uint32_t brp_inc; 
};
struct gs_device_mode { uint32_t mode; uint32_t flags; };
struct gs_host_frame { 
    uint32_t echo_id; uint32_t can_id; uint8_t can_dlc; 
    uint8_t channel; uint8_t flags; uint8_t reserved; uint8_t data[8]; 
};
#pragma pack(pop)
EOF

# 7. main/main.c
cat <<EOF > $PROJECT_NAME/main/main.c
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "tusb.h"
#include "gs_usb.h"
#include "esp_private/usb_phy.h" 
#include "esp_attr.h" 

#define TX_PIN GPIO_NUM_4
#define RX_PIN GPIO_NUM_5
#define USB_VID 0x1D50 
#define USB_PID 0x606F 

// Set to 1 only for bench debugging. 0 for Production.
#define DEBUG_ALL_FRAMES 0

static const char *TAG = "GS_USB";
static bool is_can_started = false;
static usb_phy_handle_t phy_handle = NULL;
static QueueHandle_t can_to_usb_queue;

// Stats
static volatile uint32_t rx_pps = 0;
static volatile uint32_t tx_pps = 0;
static volatile uint32_t last_can_id = 0;

#define MAGIC_FLAG 0xFFFFFFFF

DMA_ATTR __attribute__((aligned(4))) static struct gs_device_bittiming pending_bt;
DMA_ATTR __attribute__((aligned(4))) static struct gs_device_mode pending_mode;
DMA_ATTR __attribute__((aligned(4))) static struct gs_host_config pending_host_config; 
DMA_ATTR __attribute__((aligned(4))) static struct gs_device_config dconf = {
    .icount = 0, .sw_version = 2, .hw_version = 1
};
DMA_ATTR __attribute__((aligned(4))) static struct gs_device_bt_const bt_const = { 
    .fclk_can = 80000000, .tseg1_max = 16, .tseg2_max = 8, .sjw_max = 4, .brp_max = 128, .brp_inc = 1 
};

// --- CAN DRIVER ---
static void stop_can() {
    if (is_can_started) {
        twai_stop(); 
        twai_driver_uninstall();
        is_can_started = false;
        ESP_LOGW(TAG, "CAN Stopped");
    }
}

static esp_err_t start_can(const struct gs_device_bittiming *bt) {
    stop_can();
    
    // REVERTED: Removed ESP_INTR_FLAG_IRAM
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 128; 
    g_config.rx_queue_len = 128;
    
    twai_timing_config_t t_config = {0};
    t_config.brp = bt->brp; 
    t_config.tseg_1 = bt->prop_seg + bt->phase_seg1;
    t_config.tseg_2 = bt->phase_seg2;
    t_config.sjw = bt->sjw;
    t_config.triple_sampling = false;
    
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        if (twai_start() == ESP_OK) {
            is_can_started = true;
            ESP_LOGI(TAG, "CAN Started (BRP: %lu)", bt->brp);
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "TWAI Start Failed");
        }
    } else {
        ESP_LOGE(TAG, "TWAI Install Failed");
    }
    return ESP_FAIL;
}

// --- USB DESCRIPTORS ---
uint8_t const * tud_descriptor_device_cb(void) {
    static const tusb_desc_device_t desc_device = {
        .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200, .bDeviceClass = 0x00, .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00, .bMaxPacketSize0 = 64,
        .idVendor = USB_VID, .idProduct = USB_PID, .bcdDevice = 0x0100,
        .iManufacturer = 0x01, .iProduct = 0x02, .iSerialNumber = 0x03, .bNumConfigurations = 0x01
    };
    return (uint8_t const *) &desc_device;
}
uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
    static const uint8_t desc_configuration[] = {
        0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
        0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x00,
        0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,
        0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00
    };
    return desc_configuration;
}
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    static uint16_t _desc_str[32];
    const char* str_arr[] = { (const char[]) { 0x09, 0x04 }, "Triton", "ESP32-S3 CAN", "1.0" };
    if (index == 0) {
        memcpy(&_desc_str[1], str_arr[0], 2); _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2 + 2);
        return _desc_str;
    }
    if (index >= 4) return NULL;
    const char* str = str_arr[index]; uint8_t len = (uint8_t) strlen(str); if (len > 31) len = 31;
    for (uint8_t i=0; i<len; i++) _desc_str[1+i] = str[i];
    _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*len + 2);
    return _desc_str;
}

// --- USB CALLBACKS ---
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request) {
    if (stage != CONTROL_STAGE_SETUP) return true;
    switch (request->bRequest) {
        case GS_USB_BREQ_HOST_FORMAT: 
            return tud_control_xfer(rhport, request, &pending_host_config, sizeof(struct gs_host_config));
        case GS_USB_BREQ_BITTIMING: 
            return tud_control_xfer(rhport, request, &pending_bt, sizeof(struct gs_device_bittiming));
        case GS_USB_BREQ_MODE: 
            pending_mode.flags = MAGIC_FLAG;
            return tud_control_xfer(rhport, request, &pending_mode, sizeof(struct gs_device_mode));
        case GS_USB_BREQ_BT_CONST:
            return tud_control_xfer(rhport, request, &bt_const, sizeof(struct gs_device_bt_const));
        case GS_USB_BREQ_DEVICE_CONFIG:
            return tud_control_xfer(rhport, request, &dconf, sizeof(struct gs_device_config));
        default: 
            return tud_control_xfer(rhport, request, NULL, 0);
    }
}

void tud_vendor_rx_cb(uint8_t itf) {
    if (!is_can_started) { tud_vendor_read_flush(); return; }
    
    struct gs_host_frame frame;
    while (tud_vendor_available() >= sizeof(frame)) {
        if (tud_vendor_read(&frame, sizeof(frame)) == sizeof(frame)) {
            twai_message_t msg = {0};
            msg.identifier = frame.can_id; 
            msg.data_length_code = frame.can_dlc;
            if (frame.can_id & 0x80000000) { 
                msg.extd = 1; msg.identifier &= 0x1FFFFFFF; 
            }
            memcpy(msg.data, frame.data, 8);
            
            // Transmit to Physical Bus
            if (twai_transmit(&msg, 0) == ESP_OK) {
                tx_pps++;
                
                // === FIX START: Send Echo back to Linux ===
                // Linux gs_usb driver waits for this to free the buffer slot
                struct gs_host_frame echo_frame;
                memset(&echo_frame, 0, sizeof(echo_frame));
                
                echo_frame.echo_id = frame.echo_id; // CRITICAL: Match the ID Linux sent
                echo_frame.can_id = frame.can_id;
                echo_frame.can_dlc = frame.can_dlc;
                echo_frame.channel = frame.channel;
                echo_frame.flags = 0; // 0 = Normal Frame (Echo)
                memcpy(echo_frame.data, frame.data, 8);
                
                // Send to the same queue that handles RX frames
                xQueueSend(can_to_usb_queue, &echo_frame, 0);
                // === FIX END ===

                #if DEBUG_ALL_FRAMES
                ESP_LOGI(TAG, "TX -> ID: %lx", msg.identifier);
                #endif
            }
        }
    }
}

// --- TASKS ---
void usb_manager_task(void *arg) {
    ESP_LOGI(TAG, "USB Manager Started");
    int stats_timer = 0;
    while (1) {
        tud_task(); 
        
        // Stats: 100 ticks = 1 second (at 100Hz tick rate)
        if (++stats_timer % 100 == 0) {
            if (is_can_started) {
                // Only print if there is activity to reduce noise
                if (rx_pps > 0 || tx_pps > 0) {
                    ESP_LOGI(TAG, "STATS | RX: %lu pps | TX: %lu pps | Last ID: %03lx", rx_pps, tx_pps, last_can_id);
                }
            }
            rx_pps = 0; tx_pps = 0;
        }
        vTaskDelay(1); 
    }
}

void can_forward_task(void *arg) {
    struct gs_host_frame frame;
    while (!tud_mounted()) vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "USB Mounted - System Ready");

    while (1) { 
        if (pending_mode.flags != MAGIC_FLAG) {
             if (pending_mode.mode == GS_CAN_MODE_START) start_can(&pending_bt);
             else if (pending_mode.mode == GS_CAN_MODE_RESET) stop_can();
             pending_mode.flags = MAGIC_FLAG; 
        }

        while (uxQueueMessagesWaiting(can_to_usb_queue) > 0) {
            if (tud_vendor_write_available() < sizeof(struct gs_host_frame)) break;
            if (xQueueReceive(can_to_usb_queue, &frame, 0) == pdTRUE) {
                if (tud_vendor_write(&frame, sizeof(frame)) == sizeof(frame)) {
                     tud_vendor_write_flush();
                }
            }
        }
        vTaskDelay(1);
    }
}

void can_rx_task(void *arg) {
    twai_message_t msg; 
    struct gs_host_frame frame;
    ESP_LOGI(TAG, "CAN Listener Ready");

    while (1) {
        if (!is_can_started) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        if (twai_receive(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
            rx_pps++;
            last_can_id = msg.identifier;
            
            #if DEBUG_ALL_FRAMES
            ESP_LOGI(TAG, "RX <- ID: %lx", msg.identifier);
            #endif

            memset(&frame, 0, sizeof(frame));
            frame.echo_id = 0xFFFFFFFF; 
            frame.can_id = msg.identifier;
            if (msg.extd) frame.can_id |= 0x80000000;
            frame.can_dlc = msg.data_length_code;
            memcpy(frame.data, msg.data, 8);
            xQueueSend(can_to_usb_queue, &frame, 0);
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== v32 STABLE PRODUCTION ===");
    pending_mode.flags = MAGIC_FLAG;
    can_to_usb_queue = xQueueCreate(128, sizeof(struct gs_host_frame));

    usb_phy_config_t phy_conf = { .controller = USB_PHY_CTRL_OTG, .target = USB_PHY_TARGET_INT, .otg_mode = USB_OTG_MODE_DEVICE };
    usb_new_phy(&phy_conf, &phy_handle);
    tusb_init();

    xTaskCreate(usb_manager_task, "usb_mgr", 4096, NULL, 5, NULL);
    xTaskCreate(can_forward_task, "fwd_task", 4096, NULL, 4, NULL);
    xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 4, NULL);
}
EOF

echo "Done. Flash v32."
