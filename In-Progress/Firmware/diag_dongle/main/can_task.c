#include "can_task.h"
#include "robstride.h"
#include "state.h"
#include "diag_tests.h"
#include "sdkconfig.h"

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

static const char *TAG = "can_task";

#define HOST_ID  ((uint16_t)CONFIG_DIAG_HOST_ID)

// Monitor cadence: poll one parameter every MONITOR_PERIOD_MS.
#define MONITOR_PERIOD_MS  100
// If we haven't heard from the motor for this long, assume it disconnected.
#define MOTOR_STALE_MS     1500
// Telemetry sample push to ring buffer (separate from poll cadence).
#define TELEM_SAMPLE_MS    100

typedef struct diag_pending {
    diag_req_kind_t kind;
    SemaphoreHandle_t done;
    char *result_json;     // can_task fills this in
    esp_err_t status;
} diag_pending_t;

static QueueHandle_t s_req_queue;

// Round-robin parameter list polled while a motor is detected.
static const uint16_t s_poll_params[] = {
    RS_PARAM_VBUS,
    RS_PARAM_MECH_POS,
    RS_PARAM_MECH_VEL,
    RS_PARAM_IQF,
    RS_PARAM_RUN_MODE,
};
static const int s_poll_param_count = sizeof(s_poll_params) / sizeof(s_poll_params[0]);

// ---- Public API: submit a request from any thread ---------------------------

esp_err_t diag_request(diag_req_kind_t kind, char **out_json, uint32_t timeout_ms) {
    if (out_json) *out_json = NULL;

    diag_pending_t *p = calloc(1, sizeof(*p));
    if (!p) return ESP_ERR_NO_MEM;
    p->kind = kind;
    p->done = xSemaphoreCreateBinary();
    if (!p->done) { free(p); return ESP_ERR_NO_MEM; }

    if (xQueueSend(s_req_queue, &p, pdMS_TO_TICKS(100)) != pdPASS) {
        vSemaphoreDelete(p->done);
        free(p);
        return ESP_FAIL;
    }
    if (xSemaphoreTake(p->done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        // Best-effort: leak the pending struct rather than free racing with
        // can_task. In practice timeouts shouldn't happen.
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t status = p->status;
    if (out_json) *out_json = p->result_json;
    else if (p->result_json) free(p->result_json);
    vSemaphoreDelete(p->done);
    free(p);
    return status;
}

// ---- Internal: TX -----------------------------------------------------------

esp_err_t can_send_query(uint8_t comm_type, uint16_t data16, uint8_t motor_id,
                         const uint8_t payload[8]) {
    // Hard invariant: V1 only ever transmits Type 0 or Type 17.
    if (comm_type != RS_COMM_GET_DEVICE_ID && comm_type != RS_COMM_READ_PARAMETER) {
        ESP_LOGE(TAG, "refusing TX of comm_type %u (V1 is observe-only)", comm_type);
        return ESP_ERR_NOT_SUPPORTED;
    }
    twai_message_t m = {0};
    m.extd = 1;
    m.rtr  = 0;
    m.identifier        = rs_build_ext_id(comm_type, data16, motor_id);
    m.data_length_code  = 8;
    if (payload) memcpy(m.data, payload, 8);
    return twai_transmit(&m, pdMS_TO_TICKS(50));
}

// ---- Internal: RX dispatch ---------------------------------------------------

void can_handle_rx(const twai_message_t *m) {
    if (!m->extd) {
        // Standard frames aren't part of the RobStride private protocol.
        state_record_rx(true);
        return;
    }
    rs_id_t id = rs_decode_ext_id(m->identifier);
    bool known = false;

    switch (id.comm_type) {
        case RS_COMM_GET_DEVICE_ID: {
            // Reply: id_byte = host_id, data field low byte = motor_id, payload = MCU UID.
            uint8_t motor_id = (uint8_t)(id.data & 0xFF);
            state_set_motor_detected(motor_id, m->data);
            known = true;
            break;
        }
        case RS_COMM_MOTOR_FEEDBACK: {
            rs_feedback_t fb;
            if (rs_decode_feedback(m->identifier, m->data, m->data_length_code, &fb)) {
                state_apply_feedback(fb.motor_id, fb.fault_bits, fb.mode,
                                     fb.position_rad, fb.velocity_rps,
                                     fb.torque_nm, fb.temperature_c);
                known = true;
            }
            break;
        }
        case RS_COMM_READ_PARAMETER: {
            rs_param_reply_t pr;
            if (rs_decode_param_reply(m->data, m->data_length_code, &pr)) {
                state_apply_param(pr.index, pr.raw);
                known = true;
            }
            break;
        }
        case RS_COMM_FAULT_REPORT: {
            rs_fault_t f;
            if (rs_decode_fault(m->data, m->data_length_code, &f)) {
                state_apply_fault(f.faults, f.warnings);
                known = true;
            }
            break;
        }
        default:
            break;
    }
    state_record_rx(!known);
}

esp_err_t can_wait_match(can_match_fn pred, void *user,
                         uint32_t timeout_ms, twai_message_t *out) {
    int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    twai_message_t m;
    while (1) {
        int64_t now = esp_timer_get_time();
        if (now >= deadline_us) return ESP_ERR_TIMEOUT;
        TickType_t to = pdMS_TO_TICKS((deadline_us - now) / 1000 + 1);
        esp_err_t err = twai_receive(&m, to);
        if (err == ESP_ERR_TIMEOUT) return ESP_ERR_TIMEOUT;
        if (err != ESP_OK) return err;
        can_handle_rx(&m);
        if (pred && pred(&m, user)) {
            if (out) *out = m;
            return ESP_OK;
        }
    }
}

void can_drain_rx_nonblocking(void) {
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) {
        can_handle_rx(&m);
    }
}

uint32_t can_read_alerts(void) {
    uint32_t alerts = 0;
    twai_read_alerts(&alerts, 0);
    if (alerts) state_apply_alerts(alerts);
    return alerts;
}

// ---- TWAI bring-up ----------------------------------------------------------

static void can_init(void) {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)CONFIG_DIAG_CAN_TX_GPIO,
            (gpio_num_t)CONFIG_DIAG_CAN_RX_GPIO,
            TWAI_MODE_NORMAL);
    g.alerts_enabled = TWAI_ALERT_BUS_ERROR
                     | TWAI_ALERT_BUS_OFF
                     | TWAI_ALERT_TX_FAILED
                     | TWAI_ALERT_RX_QUEUE_FULL
                     | TWAI_ALERT_ARB_LOST
                     | TWAI_ALERT_ABOVE_ERR_WARN
                     | TWAI_ALERT_ERR_PASS;
    g.rx_queue_len = 64;
    g.tx_queue_len = 8;

    twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI up: 1 Mbps, TX=%d RX=%d, host_id=0x%04X",
             CONFIG_DIAG_CAN_TX_GPIO, CONFIG_DIAG_CAN_RX_GPIO, HOST_ID);
}

// ---- Monitor cycle ----------------------------------------------------------

static bool match_any(const twai_message_t *m, void *user) {
    (void)m; (void)user;
    return true;
}

static void monitor_step(int64_t now_us, int64_t *next_telem_us) {
    motor_state_t snap;
    state_get_motor(&snap);

    if (!snap.detected) {
        // Not yet detected: ping every ID in slow round-robin (8 per cycle).
        static uint8_t s_next_id = 0;
        for (int i = 0; i < 8; i++) {
            uint8_t target = s_next_id;
            s_next_id = (uint8_t)((s_next_id + 1) & 0x7F);
            uint8_t pl[8] = {0};
            can_send_query(RS_COMM_GET_DEVICE_ID, HOST_ID, target, pl);
            twai_message_t reply;
            // Cheap 6 ms wait per ping; if a motor responds we'll bail.
            if (can_wait_match(match_any, NULL, 6, &reply) == ESP_OK) {
                // Re-check state -- handle_rx will have set detected.
                state_get_motor(&snap);
                if (snap.detected) break;
            }
        }
    } else {
        // Detected: round-robin one parameter read.
        if ((now_us - snap.last_read_us) > (int64_t)MOTOR_STALE_MS * 1000) {
            // Lost contact -- clear detection so we re-scan next cycle.
            ESP_LOGW(TAG, "motor %u stale (%lld ms), clearing detection",
                     snap.motor_id, (long long)((now_us - snap.last_read_us) / 1000));
            state_clear_motor_detected();
            return;
        }

        static int s_idx = 0;
        uint16_t param = s_poll_params[s_idx % s_poll_param_count];
        s_idx++;
        uint8_t pl[8];
        rs_build_read_param_payload(param, pl);
        if (can_send_query(RS_COMM_READ_PARAMETER, HOST_ID, snap.motor_id, pl) == ESP_OK) {
            twai_message_t reply;
            (void)can_wait_match(match_any, NULL, 30, &reply);
        }
    }

    if (now_us >= *next_telem_us) {
        state_record_telem_sample(now_us);
        *next_telem_us = now_us + (int64_t)TELEM_SAMPLE_MS * 1000;
    }
}

// ---- Test dispatch ----------------------------------------------------------

static void run_test(diag_pending_t *p) {
    char *json = NULL;
    esp_err_t st = ESP_OK;
    switch (p->kind) {
        case DIAG_REQ_TIER0:  json = diag_run_tier0();  break;
        case DIAG_REQ_TIER1:  json = diag_run_tier1();  break;
        case DIAG_REQ_TIER2:  json = diag_run_tier2();  break;
        case DIAG_REQ_ALL:    json = diag_run_all();    break;
        case DIAG_REQ_RESCAN: json = diag_run_rescan(); break;
        default:              st = ESP_ERR_INVALID_ARG; break;
    }
    if (!json && st == ESP_OK) st = ESP_ERR_NO_MEM;
    p->result_json = json;
    p->status = st;

    // Also publish to global so the WebSocket clients pick it up.
    if (json) {
        char *copy = strdup(json);
        if (copy) state_publish_test_result(copy);
    }
}

// ---- Main loop --------------------------------------------------------------

static void can_main(void *arg) {
    can_init();
    int64_t next_monitor_us = esp_timer_get_time();
    int64_t next_telem_us   = esp_timer_get_time();
    while (1) {
        // 1. Test request?
        diag_pending_t *p = NULL;
        if (xQueueReceive(s_req_queue, &p, 0) == pdTRUE) {
            ESP_LOGI(TAG, "running test kind=%d", p->kind);
            run_test(p);
            xSemaphoreGive(p->done);
            // Reset cadence after a long test.
            next_monitor_us = esp_timer_get_time();
        }

        // 2. Periodic monitor.
        int64_t now = esp_timer_get_time();
        if (now >= next_monitor_us) {
            monitor_step(now, &next_telem_us);
            next_monitor_us = now + (int64_t)MONITOR_PERIOD_MS * 1000;
        }

        // 3. Drain RX (Type-21 fault arrives unsolicited).
        can_drain_rx_nonblocking();

        // 4. Pump alerts.
        can_read_alerts();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void can_task_start(void) {
    s_req_queue = xQueueCreate(4, sizeof(diag_pending_t *));
    xTaskCreatePinnedToCore(can_main, "can_task", 8192, NULL, 18, NULL, 1);
}
