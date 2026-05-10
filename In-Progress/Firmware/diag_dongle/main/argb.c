#include "argb.h"
#include "state.h"
#include "sdkconfig.h"

#include <stdbool.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "driver/twai.h"

static const char *TAG = "argb";

static led_strip_handle_t s_strip;
static volatile bool      s_booting = true;

void argb_set_booting(bool booting) { s_booting = booting; }

static void set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    if (!s_strip) return;
    led_strip_set_pixel(s_strip, 0, r, g, b);
    led_strip_refresh(s_strip);
}

static void argb_loop(void *arg) {
    int64_t t0 = esp_timer_get_time();
    while (1) {
        int64_t now = esp_timer_get_time();
        float t = (float)((now - t0) / 1000) / 1000.0f;  // seconds since boot

        if (s_booting) {
            // Sine breathing white
            float v = 0.5f + 0.5f * sinf(t * 3.0f);
            uint8_t w = (uint8_t)(v * 64.0f);
            set_rgb(w, w, w);
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }

        battery_state_t ba; state_get_battery(&ba);
        if (ba.vbat_mv > 0 && ba.vbat_mv < CONFIG_DIAG_BATT_CRIT_MV) {
            // Critical: rapid magenta strobe
            bool on = ((int)(t * 8.0f)) & 1;
            set_rgb(on ? 80 : 0, 0, on ? 80 : 0);
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }
        if (ba.vbat_mv > 0 && ba.vbat_mv < CONFIG_DIAG_BATT_LOW_MV) {
            bool on = ((int)(t * 1.5f)) & 1;
            set_rgb(on ? 50 : 0, 0, on ? 50 : 0);
            vTaskDelay(pdMS_TO_TICKS(80));
            continue;
        }

        bus_stats_t   bs; state_get_bus(&bs);
        motor_state_t ms; state_get_motor(&ms);

        bool fatal_bus = (bs.cum_alerts & (TWAI_ALERT_BUS_OFF | TWAI_ALERT_TX_FAILED
                                         | TWAI_ALERT_BUS_ERROR)) != 0;
        bool fault_present = ms.detected && (ms.faults || ms.warnings || ms.fb_fault_bits);

        if (!ms.detected && fatal_bus) {
            set_rgb(60, 0, 0);   // solid red
        } else if (!ms.detected) {
            bool on = ((int)(t * 1.2f)) & 1;
            set_rgb(on ? 40 : 0, on ? 30 : 0, 0);   // slow yellow blink
        } else if (fault_present) {
            set_rgb(60, 20, 0);  // orange
        } else {
            set_rgb(0, 50, 0);   // green
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void argb_start(void) {
    // WS2812 defaults to GRB component order in led_strip; don't set
    // `color_component_format` so we stay compatible with led_strip <2.5.
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = CONFIG_DIAG_ARGB_GPIO,
        .max_leds       = 1,
        .led_model      = LED_MODEL_WS2812,
        .flags          = { .invert_out = 0 },
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags = { .with_dma = 0 },
    };
    if (led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip) != ESP_OK) {
        ESP_LOGE(TAG, "ARGB init failed");
        return;
    }
    set_rgb(8, 8, 8);
    xTaskCreate(argb_loop, "argb", 3072, NULL, 4, NULL);
}
