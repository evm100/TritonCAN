#include "battery.h"
#include "state.h"
#include "sdkconfig.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "battery";

static int gpio_to_adc1_channel(int gpio) {
    if (gpio >= 1 && gpio <= 10) return gpio - 1; // ADC1_CH0..CH9 -> GPIO 1..10 on S3
    return -1;
}

// Map battery voltage to a coarse percentage (1S Li-Po, typical curve).
static int vbat_to_percent(int mv) {
    if (mv >= 4200) return 100;
    if (mv >= 4060) return 90 + (mv - 4060) * 10 / 140;
    if (mv >= 3920) return 75 + (mv - 3920) * 15 / 140;
    if (mv >= 3800) return 55 + (mv - 3800) * 20 / 120;
    if (mv >= 3700) return 40 + (mv - 3700) * 15 / 100;
    if (mv >= 3600) return 25 + (mv - 3600) * 15 / 100;
    if (mv >= 3400) return 10 + (mv - 3400) * 15 / 200;
    if (mv >= 3000) return  0 + (mv - 3000) * 10 / 400;
    return 0;
}

static void battery_loop(void *arg) {
    int channel = gpio_to_adc1_channel(CONFIG_DIAG_BATT_ADC_GPIO);
    if (channel < 0) {
        ESP_LOGE(TAG, "GPIO %d not on ADC1", CONFIG_DIAG_BATT_ADC_GPIO);
        vTaskDelete(NULL);
        return;
    }

    adc_oneshot_unit_handle_t unit;
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id   = ADC_UNIT_1,
        .ulp_mode  = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&unit_cfg, &unit) != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed"); vTaskDelete(NULL); return;
    }
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,    // up to ~3.1 V at the pin (Vbat/2 = ~2.1 V at 4.2 V)
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_config_channel(unit, (adc_channel_t)channel, &chan_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed"); vTaskDelete(NULL); return;
    }

    adc_cali_handle_t cali = NULL;
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = (adc_channel_t)channel,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    bool have_cali = (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali) == ESP_OK);
    if (!have_cali) {
        ESP_LOGW(TAG, "ADC calibration unavailable -- voltage will be approximate");
    }

    while (1) {
        int sum_mv = 0; int n = 0;
        for (int i = 0; i < 16; i++) {
            int raw = 0;
            if (adc_oneshot_read(unit, (adc_channel_t)channel, &raw) != ESP_OK) continue;
            int mv = 0;
            if (have_cali) {
                if (adc_cali_raw_to_voltage(cali, raw, &mv) != ESP_OK) continue;
            } else {
                mv = raw * 3300 / 4095;
            }
            sum_mv += mv;
            n++;
        }
        if (n > 0) {
            int avg_pin_mv = sum_mv / n;
            // Multiply by 2 for the 100k/100k divider on the cell.
            int vbat_mv = avg_pin_mv * 2;
            int pct = vbat_to_percent(vbat_mv);
            bool ok = vbat_mv > CONFIG_DIAG_BATT_CRIT_MV;
            state_set_battery(vbat_mv, pct, ok);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void battery_start(void) {
    xTaskCreate(battery_loop, "battery", 3072, NULL, 3, NULL);
}
