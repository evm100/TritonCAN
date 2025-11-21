#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"

static const char *TAG = "RS02_BASIC";

// ----- RS02 limits from manual (private protocol, operation-control mode) -----
#define P_MIN   -12.57f   // rad  (approx -4π)
#define P_MAX    12.57f   // rad  (approx 4π)
#define V_MIN   -44.0f    // rad/s
#define V_MAX    44.0f    // rad/s
#define KP_MIN    0.0f
#define KP_MAX  500.0f
#define KD_MIN    0.0f
#define KD_MAX    5.0f
#define T_MIN   -17.0f    // N·m
#define T_MAX    17.0f

// Default IDs for a fresh RS02 (motor CAN_ID = 1, host/master CAN_ID = 1)
#define RS02_MOTOR_ID   1
#define RS02_MASTER_ID  1

// ---- Helpers -----------------------------------------------------------------

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;

    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * Build extended 29-bit identifier in the same layout as the GD32 example:
 *   bits  0..7   : motor CAN_ID
 *   bits  8..23  : master_id (host CAN_ID)
 *   bits 24..28  : communication type (mode)
 *   bits 29..31  : reserved (0)
 *
 * This matches the IDs you see like 0x3000101 for type=3, master=1, motor=1.
 */
static uint32_t build_ext_id(uint8_t motor_id, uint16_t master_id, uint8_t type)
{
    uint32_t id = 0;
    id |= (uint32_t)motor_id;
    id |= ((uint32_t)master_id << 8);
    id |= ((uint32_t)type << 24);
    return id;
}

// ---- CAN / TWAI initialization ----------------------------------------------

static void can_init(void)
{
    // TX = GPIO 20, RX = GPIO 21, normal mode
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
            GPIO_NUM_20, GPIO_NUM_21, TWAI_MODE_NORMAL);

    // 1 Mbit/s timing (built-in macro)
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();

    // Accept all frames
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI (CAN) started at 1 Mbps on TX=20, RX=21");
}

// ---- RS02 command helpers ----------------------------------------------------

/**
 * Send communication type 3: "Motor enabled to run"
 * (Data bytes are all zero.)
 */
static esp_err_t rs02_send_enable(void)
{
    twai_message_t msg = {0};
    msg.extd = 1;                      // extended frame
    msg.rtr = 0;
    msg.identifier = build_ext_id(RS02_MOTOR_ID, RS02_MASTER_ID, 3);
    msg.data_length_code = 8;
    memset(msg.data, 0, 8);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Sent ENABLE (type 3)");
    } else {
        ESP_LOGE(TAG, "Failed to send ENABLE: %s", esp_err_to_name(err));
    }
    return err;
}

/**
 * Send communication type 1: operation control mode motor command
 *
 * Parameters:
 *   torque_ff : feed-forward torque (N·m)
 *   pos       : target position (rad)  – we keep this at 0.0
 *   vel       : target speed (rad/s)
 *   kp, kd    : gains
 *
 * Note: on a fresh motor, it's already in operation-control mode after power-on.
 */
static esp_err_t rs02_send_op_control(float torque_ff,
                                      float pos,
                                      float vel,
                                      float kp,
                                      float kd)
{
    twai_message_t msg = {0};
    msg.extd = 1;
    msg.rtr = 0;
    msg.identifier = build_ext_id(RS02_MOTOR_ID, RS02_MASTER_ID, 1);
    msg.data_length_code = 8;

    uint16_t pos_u = (uint16_t)float_to_uint(pos,   P_MIN,  P_MAX, 16);
    uint16_t vel_u = (uint16_t)float_to_uint(vel,   V_MIN,  V_MAX, 16);
    uint16_t kp_u  = (uint16_t)float_to_uint(kp,    KP_MIN, KP_MAX,16);
    uint16_t kd_u  = (uint16_t)float_to_uint(kd,    KD_MIN, KD_MAX,16);

    // Byte layout per the manual example: t_ff in ID "data" field, then P, V, Kp, Kd as 16-bit big-endian.
    uint16_t torque_u = (uint16_t)float_to_uint(torque_ff, T_MIN, T_MAX, 16);

    // However, in the reference code torque is carried in the ID "data" field:
    // txCanIdEx.data = float_to_uint(torque, T_MIN, T_MAX, 16);
    // For simplicity we just keep torque_ff small (0) and ignore it here.
    (void)torque_u; // not used – keep compiler happy

    msg.data[0] = (uint8_t)(pos_u >> 8);
    msg.data[1] = (uint8_t)(pos_u & 0xFF);
    msg.data[2] = (uint8_t)(vel_u >> 8);
    msg.data[3] = (uint8_t)(vel_u & 0xFF);
    msg.data[4] = (uint8_t)(kp_u  >> 8);
    msg.data[5] = (uint8_t)(kp_u  & 0xFF);
    msg.data[6] = (uint8_t)(kd_u  >> 8);
    msg.data[7] = (uint8_t)(kd_u  & 0xFF);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send op-control cmd: %s", esp_err_to_name(err));
    }
    return err;
}

// Optional: simple RX logger (feedback / fault frames)
static void can_rx_task(void *arg)
{
    twai_message_t rx_msg;
    while (1) {
        if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
            if (rx_msg.extd) {
                printf("RX: ID=0x%08lx DLC=%d Data:", (unsigned long)rx_msg.identifier,
                       rx_msg.data_length_code);
            } else {
                printf("RX: STD ID=0x%03lx DLC=%d Data:", (unsigned long)rx_msg.identifier,
                       rx_msg.data_length_code);
            }
            for (int i = 0; i < rx_msg.data_length_code; i++) {
                printf(" %02X", rx_msg.data[i]);
            }
            printf("\n");
        }
    }
}

// ---- app_main ---------------------------------------------------------------

void app_main(void)
{
    can_init();

    // Optional: start RX logger
    xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 5, NULL);

    vTaskDelay(pdMS_TO_TICKS(500));  // small delay after power-up

    // 1) Enable the motor (communication type 3)
    rs02_send_enable();

    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Starting basic motion test");

    // 2) Repeatedly send a small positive speed command.
    //
    //    Operation-control mode suggestion from manual:
    //      t_ff = 0
    //      v_set = 1 rad/s
    //      p_set = 0
    //      Kp   = 0
    //      Kd   = 1
    //
    //    This should make the motor spin slowly in one direction with light damping.
    while (1) {
        rs02_send_op_control(
            0.0f,   // torque_ff
            0.0f,   // position setpoint (rad)
            1.0f,   // velocity setpoint (rad/s) – keep small
            0.0f,   // Kp
            1.0f    // Kd
        );

        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz command rate is plenty for a simple test
    }
}
