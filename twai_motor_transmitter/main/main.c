#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "RS02_DEMO";

#define RS02_P_MIN (-12.57f)
#define RS02_P_MAX (12.57f)
#define RS02_V_MIN (-44.0f)
#define RS02_V_MAX (44.0f)
#define RS02_KP_MIN (0.0f)
#define RS02_KP_MAX (500.0f)
#define RS02_KD_MIN (0.0f)
#define RS02_KD_MAX (5.0f)
#define RS02_T_MIN (-17.0f)
#define RS02_T_MAX (17.0f)

#define RS02_MASTER_ID 0x0000u
#define RS02_MODE_OPERATION_CONTROL 0x01u
#define RS02_MODE_ENABLE 0x03u
#define RS02_MODE_STOP 0x04u

#define DEMO_SPEED_RAD_S (1.0f)
#define DEMO_TORQUE_FF (0.0f)
#define DEMO_POSITION_REF (0.0f)
#define DEMO_KP (0.0f)
#define DEMO_KD (1.0f)

static inline uint16_t rs02_float_to_uint(float value, float min, float max)
{
    float clamped = value;
    if (clamped > max) {
        clamped = max;
    } else if (clamped < min) {
        clamped = min;
    }

    const float span = max - min;
    const float normalized = (clamped - min) * (((float)((1 << 16) - 1)) / span);
    if (normalized < 0.0f) {
        return 0;
    }
    if (normalized > 65535.0f) {
        return 0xFFFF;
    }
    return (uint16_t)normalized;
}

static inline void rs02_store_u16_be(uint8_t *dest, uint16_t value)
{
    dest[0] = (uint8_t)(value >> 8);
    dest[1] = (uint8_t)(value & 0xFF);
}

static inline uint32_t rs02_make_identifier(uint8_t mode, uint16_t data_field)
{
    const uint32_t reserved = 0;
    return (reserved << 29) |
           (((uint32_t)mode & 0x1Fu) << 24) |
           (((uint32_t)data_field & 0xFFFFu) << 8) |
           ((uint32_t)CONFIG_TWAI_RS02_MOTOR_ID & 0xFFu);
}

static esp_err_t rs02_send_frame(uint8_t mode, uint16_t data_field, const uint8_t payload[8])
{
    twai_message_t message = {
        .identifier = rs02_make_identifier(mode, data_field),
        .data_length_code = 8,
        .flags = TWAI_MSG_FLAG_EXTD,
    };
    memcpy(message.data, payload, sizeof(message.data));

    esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send mode %u frame: %s", mode, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t rs02_enable_motor(void)
{
    uint8_t payload[8] = {0};
    ESP_LOGI(TAG, "Enabling RS02 motor (ID=%u)", CONFIG_TWAI_RS02_MOTOR_ID);
    return rs02_send_frame(RS02_MODE_ENABLE, RS02_MASTER_ID, payload);
}

static esp_err_t rs02_stop_motor(void)
{
    uint8_t payload[8] = {0};
    ESP_LOGI(TAG, "Stopping RS02 motor");
    return rs02_send_frame(RS02_MODE_STOP, RS02_MASTER_ID, payload);
}

static esp_err_t rs02_motion_command(float torque, float position, float velocity, float kp, float kd)
{
    uint8_t payload[8] = {0};

    rs02_store_u16_be(&payload[0], rs02_float_to_uint(position, RS02_P_MIN, RS02_P_MAX));
    rs02_store_u16_be(&payload[2], rs02_float_to_uint(velocity, RS02_V_MIN, RS02_V_MAX));
    rs02_store_u16_be(&payload[4], rs02_float_to_uint(kp, RS02_KP_MIN, RS02_KP_MAX));
    rs02_store_u16_be(&payload[6], rs02_float_to_uint(kd, RS02_KD_MIN, RS02_KD_MAX));

    uint16_t torque_field = rs02_float_to_uint(torque, RS02_T_MIN, RS02_T_MAX);
    ESP_LOGI(TAG, "Commanding torque=%.2f, velocity=%.2f rad/s", torque, velocity);
    return rs02_send_frame(RS02_MODE_OPERATION_CONTROL, torque_field, payload);
}

static void init_twai(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_20, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}

void app_main(void)
{
    init_twai();
    ESP_LOGI(TAG, "TWAI motor transmitter ready (motor ID %u)", CONFIG_TWAI_RS02_MOTOR_ID);

    if (rs02_enable_motor() != ESP_OK) {
        ESP_LOGE(TAG, "Unable to enable motor; halting demo");
        return;
    }

    const TickType_t spin_duration = pdMS_TO_TICKS(3000);
    const TickType_t stop_duration = pdMS_TO_TICKS(1000);

    while (true) {
        rs02_motion_command(DEMO_TORQUE_FF, DEMO_POSITION_REF, DEMO_SPEED_RAD_S, DEMO_KP, DEMO_KD);
        vTaskDelay(spin_duration);

        rs02_stop_motor();
        vTaskDelay(stop_duration);

        rs02_motion_command(DEMO_TORQUE_FF, DEMO_POSITION_REF, -DEMO_SPEED_RAD_S, DEMO_KP, DEMO_KD);
        vTaskDelay(spin_duration);

        rs02_stop_motor();
        vTaskDelay(stop_duration);
    }
}
