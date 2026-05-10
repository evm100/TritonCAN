#include "state.h"
#include "robstride.h"

#include <stdlib.h>
#include <string.h>
#include "esp_timer.h"

static motor_state_t     s_motor;
static telemetry_ring_t  s_telem;
static bus_stats_t       s_bus;
static battery_state_t   s_batt;
static test_result_t     s_test;

static SemaphoreHandle_t s_mutex;

#define LOCK()   xSemaphoreTake(s_mutex, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s_mutex)

void state_init(void) {
    memset(&s_motor, 0, sizeof(s_motor));
    memset(&s_telem, 0, sizeof(s_telem));
    memset(&s_bus,   0, sizeof(s_bus));
    memset(&s_batt,  0, sizeof(s_batt));
    memset(&s_test,  0, sizeof(s_test));
    s_mutex      = xSemaphoreCreateMutex();
    s_test.mutex = xSemaphoreCreateMutex();
}

void state_get_motor(motor_state_t *out) {
    LOCK(); *out = s_motor; UNLOCK();
}
void state_get_telem(telemetry_ring_t *out) {
    LOCK(); *out = s_telem; UNLOCK();
}
void state_get_bus(bus_stats_t *out) {
    LOCK(); *out = s_bus; UNLOCK();
}
void state_get_battery(battery_state_t *out) {
    LOCK(); *out = s_batt; UNLOCK();
}

void state_set_motor_detected(uint8_t id, const uint8_t uid[8]) {
    LOCK();
    if (!s_motor.detected || s_motor.motor_id != id) {
        // New detection: clear stale telemetry.
        memset(&s_motor, 0, sizeof(s_motor));
        memset(&s_telem, 0, sizeof(s_telem));
    }
    s_motor.detected = true;
    s_motor.motor_id = id;
    if (uid) memcpy(s_motor.uid, uid, 8);
    UNLOCK();
}

void state_clear_motor_detected(void) {
    LOCK();
    memset(&s_motor, 0, sizeof(s_motor));
    UNLOCK();
}

void state_apply_feedback(uint8_t motor_id, uint8_t fault_bits, uint8_t mode,
                          float pos, float vel, float torque, float temp) {
    LOCK();
    s_motor.detected         = true;
    s_motor.motor_id         = motor_id;
    s_motor.fb_fault_bits    = fault_bits;
    s_motor.fb_mode          = mode;
    s_motor.mech_pos_rad     = pos;
    s_motor.mech_vel_rps     = vel;
    s_motor.iq_a             = torque;   // approximate -- in MIT mode "torque" feedback is iq-like
    s_motor.temperature_c    = temp;
    s_motor.last_feedback_us = esp_timer_get_time();
    UNLOCK();
}

void state_apply_fault(uint32_t faults, uint32_t warnings) {
    LOCK();
    s_motor.faults        = faults;
    s_motor.warnings      = warnings;
    s_motor.last_fault_us = esp_timer_get_time();
    UNLOCK();
}

void state_apply_param(uint16_t index, const uint8_t raw[4]) {
    LOCK();
    s_motor.last_read_us = esp_timer_get_time();
    switch (index) {
        case RS_PARAM_RUN_MODE:        s_motor.run_mode      = rs_param_to_u8 (raw); break;
        case RS_PARAM_VBUS:            s_motor.vbus_v        = rs_param_to_f32(raw); break;
        case RS_PARAM_MECH_POS:        s_motor.mech_pos_rad  = rs_param_to_f32(raw); break;
        case RS_PARAM_MECH_VEL:        s_motor.mech_vel_rps  = rs_param_to_f32(raw); break;
        case RS_PARAM_IQF:             s_motor.iq_a          = rs_param_to_f32(raw); break;
        case RS_PARAM_ROTATION:        s_motor.rotation      = rs_param_to_i16(raw); break;
        case RS_PARAM_LIMIT_TORQUE:    s_motor.limit_torque  = rs_param_to_f32(raw); break;
        case RS_PARAM_LIMIT_SPD:       s_motor.limit_spd     = rs_param_to_f32(raw); break;
        case RS_PARAM_LIMIT_CUR:       s_motor.limit_cur     = rs_param_to_f32(raw); break;
        default: /* ignore unknown params silently */ break;
    }
    UNLOCK();
}

void state_record_telem_sample(int64_t ts_us) {
    LOCK();
    int h = s_telem.head;
    s_telem.ts_us[h] = ts_us;
    s_telem.pos [h]  = s_motor.mech_pos_rad;
    s_telem.vel [h]  = s_motor.mech_vel_rps;
    s_telem.iq  [h]  = s_motor.iq_a;
    s_telem.vbus[h]  = s_motor.vbus_v;
    s_telem.head     = (h + 1) % DIAG_TELEM_HISTORY_LEN;
    if (s_telem.count < DIAG_TELEM_HISTORY_LEN) s_telem.count++;
    UNLOCK();
}

void state_apply_alerts(uint32_t alerts) {
    LOCK();
    s_bus.last_alert_flags = alerts;
    s_bus.cum_alerts      |= alerts;
    s_bus.last_alert_us    = esp_timer_get_time();
    UNLOCK();
}

void state_record_rx(bool unknown) {
    LOCK();
    s_bus.rx_total++;
    if (unknown) s_bus.rx_unknown_type++;
    UNLOCK();
}

void state_set_battery(int vbat_mv, int percent, bool ok) {
    LOCK();
    s_batt.vbat_mv         = vbat_mv;
    s_batt.percent         = percent;
    s_batt.ok              = ok;
    s_batt.last_sample_us  = esp_timer_get_time();
    UNLOCK();
}

void state_publish_test_result(char *json) {
    if (!json) return;
    xSemaphoreTake(s_test.mutex, portMAX_DELAY);
    if (s_test.json) free(s_test.json);
    s_test.json        = json;
    s_test.generation += 1;
    xSemaphoreGive(s_test.mutex);
}

int state_get_test_generation(void) {
    int g;
    xSemaphoreTake(s_test.mutex, portMAX_DELAY);
    g = s_test.generation;
    xSemaphoreGive(s_test.mutex);
    return g;
}

int state_copy_test_result(char *out, size_t out_len) {
    if (!out || out_len == 0) return 0;
    xSemaphoreTake(s_test.mutex, portMAX_DELAY);
    int n = 0;
    if (s_test.json) {
        n = (int)strnlen(s_test.json, out_len - 1);
        memcpy(out, s_test.json, n);
        out[n] = '\0';
    } else {
        out[0] = '\0';
    }
    xSemaphoreGive(s_test.mutex);
    return n;
}
