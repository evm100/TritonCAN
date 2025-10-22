#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#include "slcan.h"

static const char *TAG = "SLCAN";

// === Pins (can be changed in menuconfig) ===
#define TWAI_TX_GPIO CONFIG_SLCAN_TWAI_TX_GPIO
#define TWAI_RX_GPIO CONFIG_SLCAN_TWAI_RX_GPIO

// RX line buffer for SLCAN
#define SLCAN_LINE_MAX 128

// Globals
static slcan_state_t g_sl = {.opened=false, .bitrate=CONFIG_SLCAN_DEFAULT_BITRATE};

// Prototypes
static void init_usb_cdc(void);
static void init_twai(int bitrate);
static void deinit_twai(void);
static void task_usb_rx(void *arg);
static void task_can_rx(void *arg);

// TinyUSB CDC callbacks (optional minimal)
static void tud_cdc_rx_cb(int itf, cdcacm_event_t *event)
{
    (void)itf; (void)event;
    // We read in task context; no action here.
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 USB<->CAN (SLCAN) bridge starting...");
    init_usb_cdc();

    // Start CAN with default bitrate but not in listen-only, normal mode
    init_twai(g_sl.bitrate);

    // Tasks: USB->CAN, CAN->USB
    xTaskCreatePinnedToCore(task_usb_rx, "slcan_usb_rx", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(task_can_rx, "slcan_can_rx", 4096, NULL, 10, NULL, 1);
}

// -------- TinyUSB CDC init --------
static void init_usb_cdc(void)
{
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    // Create one CDC instance
    tinyusb_config_cdcacm_t cdc_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 512,
        .callback_rx = tud_cdc_rx_cb,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&cdc_cfg));
    ESP_LOGI(TAG, "TinyUSB CDC ACM ready");
}

// -------- TWAI (CAN) init/deinit --------
static void init_twai(int bitrate)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config;

    switch (bitrate) {
        case 10000:   t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_10KBITS(); break;
        case 20000:   t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_20KBITS(); break;
        case 50000:   t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_50KBITS(); break;
        case 100000:  t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_100KBITS(); break;
        case 125000:  t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS(); break;
        case 250000:  t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS(); break;
        case 500000:  t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS(); break;
        case 800000:  // No direct macro in older IDFs; approximate using 40MHz APB default:
                      // If your IDF lacks 800k, consider removing S7 or mapping to 1M.
                      t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS(); // fallback
                      ESP_LOGW(TAG, "800k not directly supported; falling back to 500k");
                      break;
        case 1000000: t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS(); break;
        default:      t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS(); break;
    }

    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI started at %d bps (TX=%d, RX=%d)", bitrate, TWAI_TX_GPIO, TWAI_RX_GPIO);
}

static void deinit_twai(void)
{
    twai_stop();
    twai_driver_uninstall();
}

// -------- USB->CAN task: parse SLCAN lines and send CAN --------
static void task_usb_rx(void *arg)
{
    char line[SLCAN_LINE_MAX];
    int  idx = 0;

    while (1) {
        uint8_t buf[64];
        size_t n = 0;
        esp_err_t r = tud_cdc_acm_read(TINYUSB_CDC_ACM_0, buf, sizeof(buf), &n);
        if (r == ESP_OK && n > 0) {
            for (size_t i=0;i<n;i++) {
                char c = (char)buf[i];
                if (c == '\r' || c == '\n') {
                    if (idx > 0) {
                        line[idx] = 0;
                        // Process one SLCAN line
                        twai_message_t msg;
                        bool is_ctrl = false;
                        char resp[32];
                        int res = slcan_parse_line(line, &msg, &g_sl, &is_ctrl, resp, sizeof(resp));

                        if (is_ctrl) {
                            // Handle control: Sx / O / C / v / V
                            if (line[0] == 'O') {
                                // open: (re)start CAN with current bitrate
                                deinit_twai();
                                init_twai(g_sl.bitrate);
                            } else if (line[0] == 'C') {
                                // close
                                deinit_twai();
                            } else if (line[0] == 'S') {
                                // Only applies when bus is closed per SLCAN; user should 'C' then 'Sx' then 'O'
                                // We accept it anytime and reinit if opened.
                                bool was_open = g_sl.opened;
                                if (was_open) deinit_twai();
                                init_twai(g_sl.bitrate);
                            }
                            if (resp[0]) {
                                size_t w;
                                tud_cdc_acm_write(TINYUSB_CDC_ACM_0, (const uint8_t*)resp, strlen(resp), &w);
                                tud_cdc_acm_write_flush(TINYUSB_CDC_ACM_0);
                            }
                        } else if (res >= 0) {
                            if (g_sl.opened) {
                                // Transmit CAN frame
                                if (twai_transmit(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
                                    // Acknowledge with '\r' per Lawicel on success
                                    const char ok[] = "\r";
                                    size_t w;
                                    tud_cdc_acm_write(TINYUSB_CDC_ACM_0, (const uint8_t*)ok, 1, &w);
                                    tud_cdc_acm_write_flush(TINYUSB_CDC_ACM_0);
                                } else {
                                    const char err[] = "\a"; // bell on error
                                    size_t w;
                                    tud_cdc_acm_write(TINYUSB_CDC_ACM_0, (const uint8_t*)err, 1, &w);
                                    tud_cdc_acm_write_flush(TINYUSB_CDC_ACM_0);
                                }
                            } else {
                                const char err[] = "\a";
                                size_t w;
                                tud_cdc_acm_write(TINYUSB_CDC_ACM_0, (const uint8_t*)err, 1, &w);
                                tud_cdc_acm_write_flush(TINYUSB_CDC_ACM_0);
                            }
                        } else {
                            // parse error
                            const char err[] = "\a";
                            size_t w;
                            tud_cdc_acm_write(TINYUSB_CDC_ACM_0, (const uint8_t*)err, 1, &w);
                            tud_cdc_acm_write_flush(TINYUSB_CDC_ACM_0);
                        }
                        idx = 0;
                    }
                } else {
                    if (idx < (SLCAN_LINE_MAX-1)) line[idx++] = c;
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

// -------- CAN->USB task: forward CAN frames as SLCAN lines --------
static void task_can_rx(void *arg)
{
    twai_message_t msg;
    char out[64];

    while (1) {
        if (!g_sl.opened) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (twai_receive(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
            int n = slcan_format_frame(&msg, out, sizeof(out));
            if (n > 0) {
                size_t w;
                tud_cdc_acm_write(TINYUSB_CDC_ACM_0, (const uint8_t*)out, n, &w);
                tud_cdc_acm_write_flush(TINYUSB_CDC_ACM_0);
            }
        } else {
            // no frame
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

