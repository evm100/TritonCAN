#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/*
 * Shared, mutex-protected state. All readers/writers go through the helpers
 * in state.c -- never touch fields directly.
 */

#define DIAG_TELEM_HISTORY_LEN 300   // 30 s @ 10 Hz

typedef struct {
    bool      detected;
    uint8_t   motor_id;
    uint8_t   uid[8];

    // From Type-2 feedback (auto-report or piggy-back; rare in V1).
    uint8_t   fb_fault_bits;
    uint8_t   fb_mode;
    int64_t   last_feedback_us;

    // From Type-21 fault report.
    uint32_t  faults;
    uint32_t  warnings;
    int64_t   last_fault_us;

    // From Type-17 reads.
    uint8_t   run_mode;
    float     vbus_v;
    float     mech_pos_rad;
    float     mech_vel_rps;
    float     iq_a;
    int16_t   rotation;
    float     limit_torque;
    float     limit_spd;
    float     limit_cur;
    float     temperature_c;     // RS-02 has no dedicated temp param; from feedback frames if seen
    int64_t   last_read_us;
} motor_state_t;

typedef struct {
    int64_t   ts_us [DIAG_TELEM_HISTORY_LEN];
    float     pos   [DIAG_TELEM_HISTORY_LEN];
    float     vel   [DIAG_TELEM_HISTORY_LEN];
    float     iq    [DIAG_TELEM_HISTORY_LEN];
    float     vbus  [DIAG_TELEM_HISTORY_LEN];
    int       head;
    int       count;
} telemetry_ring_t;

typedef struct {
    uint32_t  last_alert_flags;  // most recent twai_read_alerts() result
    uint32_t  cum_alerts;        // OR of every alert ever reported
    int64_t   last_alert_us;
    uint32_t  rx_total;
    uint32_t  rx_unknown_type;
    uint32_t  bus_off_recoveries;
} bus_stats_t;

typedef struct {
    bool      ok;
    int       vbat_mv;
    int       percent;
    int64_t   last_sample_us;
} battery_state_t;

typedef struct {
    int       generation;       // bumps every new completion
    char     *json;             // last test result, NULL initially; caller of diag.h owns frees during update only
    SemaphoreHandle_t mutex;
} test_result_t;

void state_init(void);

// Snapshot helpers (acquire mutex internally).
void state_get_motor   (motor_state_t   *out);
void state_get_telem   (telemetry_ring_t *out);
void state_get_bus     (bus_stats_t     *out);
void state_get_battery (battery_state_t *out);

// Mutators -- only the can_task / monitors should use these.
void state_set_motor_detected(uint8_t id, const uint8_t uid[8]);
void state_clear_motor_detected(void);
void state_apply_feedback(uint8_t motor_id, uint8_t fault_bits, uint8_t mode,
                          float pos, float vel, float torque, float temp);
void state_apply_fault(uint32_t faults, uint32_t warnings);
void state_apply_param(uint16_t index, const uint8_t raw[4]);
void state_record_telem_sample(int64_t ts_us);
void state_apply_alerts(uint32_t alerts);
void state_record_rx(bool unknown);
void state_set_battery(int vbat_mv, int percent, bool ok);

// Test-result publishing. The dongle keeps the most recent result string
// and increments `generation` so the WS broadcaster can decide whether to
// resend it. `json` is heap-allocated; ownership transfers to state.
void state_publish_test_result(char *json);
int  state_get_test_generation(void);
// Copy the latest result into `out` (truncated to out_len-1). Returns 0 if none.
int  state_copy_test_result(char *out, size_t out_len);
