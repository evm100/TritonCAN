#include "diag_tests.h"
#include "can_task.h"
#include "robstride.h"
#include "state.h"
#include "sdkconfig.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"

static const char *TAG = "diag";
#define HOST_ID  ((uint16_t)CONFIG_DIAG_HOST_ID)

// ---- Small helpers ----------------------------------------------------------

static cJSON *make_test_obj(const char *id, const char *name, bool pass,
                            const char *detail, cJSON *metrics_or_null) {
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "id",   id);
    cJSON_AddStringToObject(o, "name", name);
    cJSON_AddBoolToObject  (o, "pass", pass);
    if (detail)            cJSON_AddStringToObject(o, "detail", detail);
    if (metrics_or_null)   cJSON_AddItemToObject(o, "metrics", metrics_or_null);
    return o;
}

static char *finalize_json(const char *tier, cJSON *tests, const char *verdict,
                           const char *next_action, bool overall_ok) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "tier", tier);
    cJSON_AddBoolToObject  (root, "ok",   overall_ok);
    cJSON_AddItemToObject  (root, "tests", tests);
    cJSON_AddStringToObject(root, "verdict", verdict ? verdict : "");
    cJSON_AddStringToObject(root, "next_action", next_action ? next_action : "");
    cJSON_AddNumberToObject(root, "ts_ms", (double)(esp_timer_get_time() / 1000));
    char *s = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return s;
}

// ---- Match predicates -------------------------------------------------------

typedef struct {
    uint8_t want_motor_id;
    uint8_t want_comm_type;
    uint16_t want_param_index;   // only used when comm_type == 17
    bool any_motor;
} match_ctx_t;

static bool match_id_reply(const twai_message_t *m, void *user) {
    match_ctx_t *c = (match_ctx_t *)user;
    if (!m->extd) return false;
    rs_id_t id = rs_decode_ext_id(m->identifier);
    if (id.comm_type != RS_COMM_GET_DEVICE_ID) return false;
    uint8_t replier = (uint8_t)(id.data & 0xFF);
    if (c->any_motor) return true;
    return replier == c->want_motor_id;
}

static bool match_param_reply(const twai_message_t *m, void *user) {
    match_ctx_t *c = (match_ctx_t *)user;
    if (!m->extd || m->data_length_code < 8) return false;
    rs_id_t id = rs_decode_ext_id(m->identifier);
    if (id.comm_type != RS_COMM_READ_PARAMETER) return false;
    uint8_t replier = (uint8_t)(id.data & 0xFF);
    if (replier != c->want_motor_id) return false;
    uint16_t idx = (uint16_t)m->data[0] | ((uint16_t)m->data[1] << 8);
    return idx == c->want_param_index;
}

// ---- Tier 0: bus sanity -----------------------------------------------------

static cJSON *t0_alerts(bool *out_pass, uint32_t *out_alerts) {
    can_drain_rx_nonblocking();
    // Sample alerts now (cumulative, lifetime so far).
    (void)can_read_alerts();
    bus_stats_t bs;
    state_get_bus(&bs);
    *out_alerts = bs.cum_alerts;

    bool fatal  = (bs.cum_alerts & (TWAI_ALERT_BUS_OFF | TWAI_ALERT_ABOVE_ERR_WARN)) != 0;
    bool warn   = (bs.cum_alerts & (TWAI_ALERT_BUS_ERROR | TWAI_ALERT_TX_FAILED
                                 | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_ARB_LOST
                                 | TWAI_ALERT_ERR_PASS)) != 0;
    *out_pass = !fatal && !warn;

    cJSON *m = cJSON_CreateObject();
    cJSON_AddNumberToObject(m, "alerts_hex", (double)bs.cum_alerts);
    cJSON_AddBoolToObject  (m, "bus_off",     (bs.cum_alerts & TWAI_ALERT_BUS_OFF) != 0);
    cJSON_AddBoolToObject  (m, "bus_error",   (bs.cum_alerts & TWAI_ALERT_BUS_ERROR) != 0);
    cJSON_AddBoolToObject  (m, "tx_failed",   (bs.cum_alerts & TWAI_ALERT_TX_FAILED) != 0);
    cJSON_AddBoolToObject  (m, "rx_full",     (bs.cum_alerts & TWAI_ALERT_RX_QUEUE_FULL) != 0);
    cJSON_AddBoolToObject  (m, "arb_lost",    (bs.cum_alerts & TWAI_ALERT_ARB_LOST) != 0);
    cJSON_AddBoolToObject  (m, "err_pass",    (bs.cum_alerts & TWAI_ALERT_ERR_PASS) != 0);

    const char *detail =
        fatal ? "Fatal bus alert (BUS_OFF or error-warning) — controller not transmitting."
        : warn ? "Bus errors observed; investigate termination/wiring."
        : "No alert flags raised since boot.";
    cJSON *o = make_test_obj("0.1", "TWAI alert flags", *out_pass, detail, m);
    return o;
}

static cJSON *t0_idle_count(bool *out_pass) {
    // Count frames received in 250 ms with no commands sent.
    bus_stats_t before, after;
    state_get_bus(&before);
    int64_t deadline = esp_timer_get_time() + 250000;
    while (esp_timer_get_time() < deadline) {
        can_drain_rx_nonblocking();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    state_get_bus(&after);
    uint32_t delta = after.rx_total - before.rx_total;
    bool pass = delta < 16;  // <64 fps
    cJSON *m = cJSON_CreateObject();
    cJSON_AddNumberToObject(m, "frames_per_250ms", (double)delta);
    char detail[96];
    snprintf(detail, sizeof(detail),
             "%lu frames in 250 ms (%lu fps).%s",
             (unsigned long)delta, (unsigned long)(delta * 4),
             pass ? "" : " Bus is unusually busy — another master or stuck active-report?");
    *out_pass = pass;
    return make_test_obj("0.2", "Idle frame count", pass, detail, m);
}

static cJSON *t0_passive_sniff(bool *out_pass) {
    // Listen 250 ms and report the number of unique senders.
    int64_t deadline = esp_timer_get_time() + 250000;
    uint8_t seen[128] = {0};
    int distinct = 0;
    twai_message_t m;
    while (esp_timer_get_time() < deadline) {
        if (twai_receive(&m, pdMS_TO_TICKS(5)) == ESP_OK) {
            can_handle_rx(&m);
            if (m.extd) {
                uint8_t replier = (uint8_t)((m.identifier >> 8) & 0xFF);
                if (replier < 128 && !seen[replier]) {
                    seen[replier] = 1;
                    distinct++;
                }
            }
        }
    }
    cJSON *met = cJSON_CreateObject();
    cJSON_AddNumberToObject(met, "distinct_senders", (double)distinct);
    *out_pass = true;
    char detail[64];
    snprintf(detail, sizeof(detail), "Saw frames from %d distinct sender ID(s).", distinct);
    return make_test_obj("0.3", "Passive sniff", true, detail, met);
}

// ---- Tier 1: discovery ------------------------------------------------------

typedef struct {
    int     count;
    uint8_t ids [128];
    uint8_t uids[128][8];
} scan_result_t;

static void scan_ids(scan_result_t *out) {
    memset(out, 0, sizeof(*out));
    for (int target = 0; target < 128; target++) {
        uint8_t pl[8] = {0};
        if (can_send_query(RS_COMM_GET_DEVICE_ID, HOST_ID, (uint8_t)target, pl) != ESP_OK) {
            continue;
        }
        match_ctx_t mc = { .want_motor_id = (uint8_t)target };
        twai_message_t reply;
        if (can_wait_match(match_id_reply, &mc, 8, &reply) == ESP_OK) {
            uint8_t replier = (uint8_t)((reply.identifier >> 8) & 0xFF);
            if (replier < 128) {
                bool already = false;
                for (int i = 0; i < out->count; i++) {
                    if (out->ids[i] == replier) { already = true; break; }
                }
                if (!already) {
                    out->ids[out->count] = replier;
                    memcpy(out->uids[out->count], reply.data, 8);
                    out->count++;
                }
            }
        }
    }
    // Final drain in case stragglers come in.
    int64_t until = esp_timer_get_time() + 100000;
    while (esp_timer_get_time() < until) can_drain_rx_nonblocking();
}

static cJSON *t1_scan(scan_result_t *sr, bool *out_pass) {
    scan_ids(sr);
    cJSON *m = cJSON_CreateObject();
    cJSON_AddNumberToObject(m, "found", (double)sr->count);
    cJSON *ids = cJSON_CreateArray();
    for (int i = 0; i < sr->count; i++) {
        cJSON *e = cJSON_CreateObject();
        cJSON_AddNumberToObject(e, "id", (double)sr->ids[i]);
        char hex[32];
        snprintf(hex, sizeof(hex), "%02X%02X%02X%02X%02X%02X%02X%02X",
                 sr->uids[i][0], sr->uids[i][1], sr->uids[i][2], sr->uids[i][3],
                 sr->uids[i][4], sr->uids[i][5], sr->uids[i][6], sr->uids[i][7]);
        cJSON_AddStringToObject(e, "uid", hex);
        cJSON_AddItemToArray(ids, e);
    }
    cJSON_AddItemToObject(m, "ids", ids);
    *out_pass = sr->count >= 1;
    char detail[80];
    snprintf(detail, sizeof(detail),
             *out_pass ? "Found %d motor(s)." : "No motor responded to ID 0..127.",
             sr->count);
    return make_test_obj("1.1", "ID scan 0-127", *out_pass, detail, m);
}

static cJSON *t1_identity(uint8_t motor_id, bool *out_pass) {
    // Best-effort read of run_mode, limit_torque, limit_spd, limit_cur.
    uint16_t params[] = { RS_PARAM_RUN_MODE, RS_PARAM_LIMIT_TORQUE,
                          RS_PARAM_LIMIT_SPD, RS_PARAM_LIMIT_CUR };
    int got = 0;
    for (size_t i = 0; i < sizeof(params)/sizeof(params[0]); i++) {
        uint8_t pl[8];
        rs_build_read_param_payload(params[i], pl);
        if (can_send_query(RS_COMM_READ_PARAMETER, HOST_ID, motor_id, pl) != ESP_OK) continue;
        match_ctx_t mc = { .want_motor_id = motor_id, .want_param_index = params[i] };
        twai_message_t reply;
        if (can_wait_match(match_param_reply, &mc, 50, &reply) == ESP_OK) got++;
    }
    motor_state_t ms; state_get_motor(&ms);
    cJSON *m = cJSON_CreateObject();
    cJSON_AddNumberToObject(m, "motor_id",      (double)motor_id);
    cJSON_AddNumberToObject(m, "run_mode",      (double)ms.run_mode);
    cJSON_AddNumberToObject(m, "limit_torque",  (double)ms.limit_torque);
    cJSON_AddNumberToObject(m, "limit_spd",     (double)ms.limit_spd);
    cJSON_AddNumberToObject(m, "limit_cur",     (double)ms.limit_cur);
    cJSON_AddNumberToObject(m, "params_read",   (double)got);
    *out_pass = got >= 3;
    char detail[96];
    snprintf(detail, sizeof(detail),
             "Read %d/4 identity params (mode=%u).", got, ms.run_mode);
    return make_test_obj("1.2", "Motor identity", *out_pass, detail, m);
}

static cJSON *t1_latency(uint8_t motor_id, bool *out_pass) {
    const int N = 50;
    int64_t total_us = 0;
    int64_t max_us = 0;
    int64_t min_us = INT64_MAX;
    int got = 0;
    for (int i = 0; i < N; i++) {
        uint8_t pl[8] = {0};
        int64_t t0 = esp_timer_get_time();
        if (can_send_query(RS_COMM_GET_DEVICE_ID, HOST_ID, motor_id, pl) != ESP_OK) continue;
        match_ctx_t mc = { .want_motor_id = motor_id };
        twai_message_t reply;
        if (can_wait_match(match_id_reply, &mc, 50, &reply) == ESP_OK) {
            int64_t dt = esp_timer_get_time() - t0;
            total_us += dt;
            if (dt > max_us) max_us = dt;
            if (dt < min_us) min_us = dt;
            got++;
        }
    }
    cJSON *m = cJSON_CreateObject();
    if (got == 0) {
        *out_pass = false;
        cJSON_AddNumberToObject(m, "samples", 0);
        return make_test_obj("1.3", "Round-trip latency", false,
                             "No replies during latency probe.", m);
    }
    double mean_ms = (double)total_us / got / 1000.0;
    double max_ms  = (double)max_us / 1000.0;
    double min_ms  = (double)min_us / 1000.0;
    cJSON_AddNumberToObject(m, "samples", (double)got);
    cJSON_AddNumberToObject(m, "mean_ms", mean_ms);
    cJSON_AddNumberToObject(m, "max_ms",  max_ms);
    cJSON_AddNumberToObject(m, "min_ms",  min_ms);
    *out_pass = (mean_ms < 5.0) && (max_ms < 20.0) && (got >= N * 8 / 10);
    char detail[96];
    snprintf(detail, sizeof(detail),
             "%d/%d replies, mean=%.2f ms, max=%.2f ms.", got, N, mean_ms, max_ms);
    return make_test_obj("1.3", "Round-trip latency", *out_pass, detail, m);
}

static cJSON *t1_frame_loss(uint8_t motor_id, bool *out_pass) {
    const int N = 200;
    int got = 0;
    for (int i = 0; i < N; i++) {
        uint8_t pl[8];
        rs_build_read_param_payload(RS_PARAM_VBUS, pl);
        if (can_send_query(RS_COMM_READ_PARAMETER, HOST_ID, motor_id, pl) != ESP_OK) continue;
        match_ctx_t mc = { .want_motor_id = motor_id, .want_param_index = RS_PARAM_VBUS };
        twai_message_t reply;
        if (can_wait_match(match_param_reply, &mc, 30, &reply) == ESP_OK) got++;
    }
    int lost = N - got;
    double pct = 100.0 * lost / N;
    *out_pass = pct < 1.0;
    cJSON *m = cJSON_CreateObject();
    cJSON_AddNumberToObject(m, "sent", (double)N);
    cJSON_AddNumberToObject(m, "received", (double)got);
    cJSON_AddNumberToObject(m, "loss_pct", pct);
    char detail[80];
    snprintf(detail, sizeof(detail), "Lost %d/%d frames (%.2f%%).", lost, N, pct);
    return make_test_obj("1.4", "Frame loss", *out_pass, detail, m);
}

// English-language verdict from cumulative TWAI alerts + scan result.
static const char *bus_verdict(uint32_t alerts, int found, const char **next) {
    if (alerts & TWAI_ALERT_BUS_OFF) {
        *next = "Power-cycle the dongle. Verify CAN_H/CAN_L wiring and that the motor is ALSO on 1 Mbps (it should be).";
        return "Bus offline (BUS_OFF). Wrong baud, both terminations missing, or hard short.";
    }
    if (found == 0 && (alerts & TWAI_ALERT_TX_FAILED)) {
        *next = "Multimeter the motor power rail. Visually re-check CAN_H/CAN_L are not swapped.";
        return "No motor ACK. Most likely: motor unpowered, CAN_H/CAN_L swapped, or motor not on bus.";
    }
    if (found == 0 && (alerts & TWAI_ALERT_BUS_ERROR)) {
        *next = "Confirm there is exactly one 120 ohm terminator at each bus end.";
        return "Bus errors with no responses. Termination resistor likely missing.";
    }
    if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
        *next = "Disable any other CAN master on this bus, or power-cycle the motor to clear stuck active-report.";
        return "Bus is unusually busy (RX queue overflowed). Another master, or stuck active-report.";
    }
    if (found == 0) {
        *next = "Confirm motor is powered. Try the rescan button after a 2-second wait.";
        return "No motor responded. Bus looks idle.";
    }
    if (alerts & (TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ARB_LOST | TWAI_ALERT_ERR_PASS)) {
        *next = "Re-seat the harness, then run Tier 1 again.";
        return "Found a motor, but the bus has intermittent errors — wiring or termination is marginal.";
    }
    *next = "Run Tier 2 to read motor health, or move on.";
    return "Bus + motor healthy.";
}

// ---- Tier 2: motor health ---------------------------------------------------

static cJSON *t2_vbus(uint8_t motor_id, bool *out_pass) {
    uint8_t pl[8]; rs_build_read_param_payload(RS_PARAM_VBUS, pl);
    twai_message_t reply;
    bool got = false;
    if (can_send_query(RS_COMM_READ_PARAMETER, HOST_ID, motor_id, pl) == ESP_OK) {
        match_ctx_t mc = { .want_motor_id = motor_id, .want_param_index = RS_PARAM_VBUS };
        if (can_wait_match(match_param_reply, &mc, 50, &reply) == ESP_OK) got = true;
    }
    motor_state_t ms; state_get_motor(&ms);
    cJSON *m = cJSON_CreateObject();
    cJSON_AddBoolToObject  (m, "read_ok", got);
    cJSON_AddNumberToObject(m, "vbus_v",  (double)ms.vbus_v);
    // We don't know the user's expected rail; pass if vbus is "plausible" (10..70 V).
    *out_pass = got && (ms.vbus_v > 10.0f) && (ms.vbus_v < 70.0f);
    char detail[80];
    if (!got)   snprintf(detail, sizeof(detail), "VBUS read timed out.");
    else        snprintf(detail, sizeof(detail), "VBUS = %.2f V.", (double)ms.vbus_v);
    return make_test_obj("2.1", "Bus voltage (VBUS)", *out_pass, detail, m);
}

static cJSON *t2_pos_drift(uint8_t motor_id, bool *out_pass) {
    // Sample mech_pos 8 times over ~400 ms.
    float samples[8] = {0};
    int got = 0;
    for (int i = 0; i < 8; i++) {
        uint8_t pl[8]; rs_build_read_param_payload(RS_PARAM_MECH_POS, pl);
        if (can_send_query(RS_COMM_READ_PARAMETER, HOST_ID, motor_id, pl) != ESP_OK) continue;
        match_ctx_t mc = { .want_motor_id = motor_id, .want_param_index = RS_PARAM_MECH_POS };
        twai_message_t reply;
        if (can_wait_match(match_param_reply, &mc, 30, &reply) == ESP_OK) {
            motor_state_t ms; state_get_motor(&ms);
            samples[got++] = ms.mech_pos_rad;
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    cJSON *m = cJSON_CreateObject();
    if (got < 4) {
        *out_pass = false;
        cJSON_AddNumberToObject(m, "samples", (double)got);
        return make_test_obj("2.2", "Encoder stability (mech_pos)", false,
                             "Too few samples received.", m);
    }
    float mn = samples[0], mx = samples[0];
    for (int i = 1; i < got; i++) {
        if (samples[i] < mn) mn = samples[i];
        if (samples[i] > mx) mx = samples[i];
    }
    float spread = mx - mn;
    *out_pass = spread < 0.02f;   // ±0.01 rad ~ ±0.6 deg
    cJSON_AddNumberToObject(m, "samples",   (double)got);
    cJSON_AddNumberToObject(m, "spread_rad",(double)spread);
    cJSON_AddNumberToObject(m, "min_rad",   (double)mn);
    cJSON_AddNumberToObject(m, "max_rad",   (double)mx);
    char detail[96];
    snprintf(detail, sizeof(detail),
             "%d samples, spread %.4f rad (%.2f deg).",
             got, (double)spread, (double)spread * 57.2958);
    return make_test_obj("2.2", "Encoder stability (mech_pos)", *out_pass, detail, m);
}

static cJSON *t2_run_mode(bool *out_pass) {
    motor_state_t ms; state_get_motor(&ms);
    cJSON *m = cJSON_CreateObject();
    cJSON_AddNumberToObject(m, "run_mode", (double)ms.run_mode);
    const char *name = "unknown";
    switch (ms.run_mode) {
        case 0: name = "MIT (operation control)"; break;
        case 1: name = "position";  break;
        case 2: name = "velocity";  break;
        case 3: name = "torque";    break;
    }
    cJSON_AddStringToObject(m, "mode_name", name);
    *out_pass = true;
    char detail[80];
    snprintf(detail, sizeof(detail), "run_mode = %u (%s).", ms.run_mode, name);
    return make_test_obj("2.3", "Run mode", true, detail, m);
}

static cJSON *t2_fault_register(bool *out_pass) {
    motor_state_t ms; state_get_motor(&ms);
    cJSON *m = cJSON_CreateObject();
    cJSON_AddNumberToObject(m, "faults",   (double)ms.faults);
    cJSON_AddNumberToObject(m, "warnings", (double)ms.warnings);
    cJSON_AddNumberToObject(m, "fb_fault_bits", (double)ms.fb_fault_bits);
    *out_pass = (ms.faults == 0) && (ms.warnings == 0) && (ms.fb_fault_bits == 0);
    char detail[96];
    if (*out_pass) snprintf(detail, sizeof(detail), "No faults reported.");
    else snprintf(detail, sizeof(detail),
                  "faults=0x%08lX warnings=0x%08lX fb_bits=0x%02X",
                  (unsigned long)ms.faults, (unsigned long)ms.warnings, ms.fb_fault_bits);
    return make_test_obj("2.4", "Fault register", *out_pass, detail, m);
}

static cJSON *t2_param_envelope(bool *out_pass) {
    motor_state_t ms; state_get_motor(&ms);
    bool t_ok = ms.limit_torque > 0.1f && ms.limit_torque <= RS02_T_MAX  + 0.01f;
    bool s_ok = ms.limit_spd    > 0.1f && ms.limit_spd    <= RS02_V_MAX  + 0.01f;
    bool c_ok = ms.limit_cur    > 0.1f && ms.limit_cur    <= 30.0f;
    *out_pass = t_ok && s_ok && c_ok;
    cJSON *m = cJSON_CreateObject();
    cJSON_AddNumberToObject(m, "limit_torque", (double)ms.limit_torque);
    cJSON_AddNumberToObject(m, "limit_spd",    (double)ms.limit_spd);
    cJSON_AddNumberToObject(m, "limit_cur",    (double)ms.limit_cur);
    char detail[160];
    if (*out_pass) snprintf(detail, sizeof(detail), "Limits sane (T=%.2f, V=%.2f, I=%.2f).",
                            (double)ms.limit_torque, (double)ms.limit_spd, (double)ms.limit_cur);
    else snprintf(detail, sizeof(detail),
                  "Suspicious: %slimit_torque=%.2f%s%slimit_spd=%.2f%s%slimit_cur=%.2f%s",
                  t_ok ? "" : "*", (double)ms.limit_torque, t_ok ? " " : "* ",
                  s_ok ? "" : "*", (double)ms.limit_spd,    s_ok ? " " : "* ",
                  c_ok ? "" : "*", (double)ms.limit_cur,    c_ok ? "" : "*");
    return make_test_obj("2.5", "Parameter envelope", *out_pass, detail, m);
}

// ---- Public entry points ----------------------------------------------------

char *diag_run_tier0(void) {
    cJSON *tests = cJSON_CreateArray();
    bool a, b, c;
    uint32_t alerts = 0;
    cJSON_AddItemToArray(tests, t0_alerts(&a, &alerts));
    cJSON_AddItemToArray(tests, t0_idle_count(&b));
    cJSON_AddItemToArray(tests, t0_passive_sniff(&c));
    bool ok = a && b && c;
    const char *next = ok ? "Run Tier 1 to discover motors."
                          : "Resolve bus alerts before scanning.";
    return finalize_json("tier0", tests, ok ? "Bus electrically idle and quiet." :
                                              "Bus has issues — see test details.",
                         next, ok);
}

char *diag_run_tier1(void) {
    cJSON *tests = cJSON_CreateArray();
    scan_result_t sr;
    bool a, b = true, c = true, d = true;
    cJSON_AddItemToArray(tests, t1_scan(&sr, &a));
    if (sr.count >= 1) {
        uint8_t mid = sr.ids[0];
        state_set_motor_detected(mid, sr.uids[0]);
        cJSON_AddItemToArray(tests, t1_identity   (mid, &b));
        cJSON_AddItemToArray(tests, t1_latency    (mid, &c));
        cJSON_AddItemToArray(tests, t1_frame_loss (mid, &d));
    }
    bus_stats_t bs; state_get_bus(&bs);
    const char *next = NULL;
    const char *verdict = bus_verdict(bs.cum_alerts, sr.count, &next);
    bool ok = a && b && c && d && (sr.count >= 1);
    return finalize_json("tier1", tests, verdict, next, ok);
}

char *diag_run_tier2(void) {
    cJSON *tests = cJSON_CreateArray();
    motor_state_t ms; state_get_motor(&ms);
    if (!ms.detected) {
        cJSON_AddItemToArray(tests, make_test_obj(
            "2.0", "Precondition", false,
            "No motor detected — run Tier 1 first.", NULL));
        return finalize_json("tier2", tests,
                             "Tier 2 needs a detected motor — run Tier 1 first.",
                             "Run Tier 1 (or the green button).",
                             false);
    }
    bool a, b, c, d, e;
    cJSON_AddItemToArray(tests, t2_vbus           (ms.motor_id, &a));
    cJSON_AddItemToArray(tests, t2_pos_drift      (ms.motor_id, &b));
    cJSON_AddItemToArray(tests, t2_run_mode       (&c));
    cJSON_AddItemToArray(tests, t2_fault_register (&d));
    cJSON_AddItemToArray(tests, t2_param_envelope (&e));
    bool ok = a && b && c && d && e;
    const char *verdict = ok ? "Motor reports healthy state."
                             : "Motor responding but reports a problem (see failed tests).";
    const char *next    = ok ? "When you have an E-stop, run Tier 3 motion tests."
                             : "Decode the fault detail before powering anything else.";
    return finalize_json("tier2", tests, verdict, next, ok);
}

char *diag_run_all(void) {
    // Run 0+1+2, splice into a single envelope.
    cJSON *tests = cJSON_CreateArray();

    bool t0a, t0b, t0c;
    uint32_t alerts = 0;
    cJSON_AddItemToArray(tests, t0_alerts(&t0a, &alerts));
    cJSON_AddItemToArray(tests, t0_idle_count(&t0b));
    cJSON_AddItemToArray(tests, t0_passive_sniff(&t0c));

    scan_result_t sr;
    bool s_ok, id_ok = true, lat_ok = true, loss_ok = true;
    cJSON_AddItemToArray(tests, t1_scan(&sr, &s_ok));
    bool t2_ok = true;
    if (sr.count >= 1) {
        uint8_t mid = sr.ids[0];
        state_set_motor_detected(mid, sr.uids[0]);
        cJSON_AddItemToArray(tests, t1_identity   (mid, &id_ok));
        cJSON_AddItemToArray(tests, t1_latency    (mid, &lat_ok));
        cJSON_AddItemToArray(tests, t1_frame_loss (mid, &loss_ok));
        bool a, b, c, d, e;
        cJSON_AddItemToArray(tests, t2_vbus           (mid, &a));
        cJSON_AddItemToArray(tests, t2_pos_drift      (mid, &b));
        cJSON_AddItemToArray(tests, t2_run_mode       (&c));
        cJSON_AddItemToArray(tests, t2_fault_register (&d));
        cJSON_AddItemToArray(tests, t2_param_envelope (&e));
        t2_ok = a && b && c && d && e;
    }
    bus_stats_t bs; state_get_bus(&bs);
    const char *next = NULL;
    const char *verdict = bus_verdict(bs.cum_alerts, sr.count, &next);

    bool overall = t0a && t0b && t0c && s_ok && id_ok && lat_ok && loss_ok && t2_ok;
    return finalize_json("all", tests, verdict, next, overall);
}

char *diag_run_rescan(void) {
    cJSON *tests = cJSON_CreateArray();
    scan_result_t sr;
    bool ok;
    cJSON_AddItemToArray(tests, t1_scan(&sr, &ok));
    if (sr.count >= 1) state_set_motor_detected(sr.ids[0], sr.uids[0]);
    return finalize_json("rescan", tests,
                         ok ? "Scan complete." : "No motor responded.",
                         ok ? "Continue with Tier 2 or All."
                            : "Confirm motor power and wiring, try again.",
                         ok);
}

// Silence unused warning for TAG when log level set to ERROR.
__attribute__((unused)) static const char *unused_tag_holder(void) { return TAG; }
