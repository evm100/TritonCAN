#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/twai.h"

/*
 * Single-owner TWAI driver wrapper.
 *
 * The can_task is the *only* thread that calls into the TWAI driver, which
 * keeps RX bookkeeping single-threaded. Test routines (Tier 0/1/2) and the
 * background monitor both run from inside can_task: HTTP handlers post a
 * "test request" and block until can_task processes it.
 *
 * V1 invariants enforced here:
 *   - only Type-0 and Type-17 are ever transmitted; the public TX helpers
 *     don't even expose the other comm-types.
 */

void can_task_start(void);

// ---- Public test-runner request API (called from HTTP / web handlers) ----

typedef enum {
    DIAG_REQ_TIER0,
    DIAG_REQ_TIER1,
    DIAG_REQ_TIER2,
    DIAG_REQ_ALL,
    DIAG_REQ_RESCAN,    // re-run ID scan only, used to find a moved/changed motor
} diag_req_kind_t;

// Submit a diagnostic request and wait for completion. Returns ESP_OK and
// fills *out_json with a malloc'd JSON blob (caller frees). On failure
// returns an esp_err_t and *out_json is set to NULL.
esp_err_t diag_request(diag_req_kind_t kind, char **out_json,
                       uint32_t timeout_ms);

// ---- Callable from inside can_task only -----------------------------------
// (exposed in the header just so diag_tests.c can call them; not for HTTP)

typedef bool (*can_match_fn)(const twai_message_t *m, void *user);

// Send a frame with the given extended ID and 8-byte payload.
// Only Type-0 (GET_DEVICE_ID) and Type-17 (READ_PARAMETER) are accepted.
esp_err_t can_send_query(uint8_t comm_type, uint16_t data16, uint8_t motor_id,
                         const uint8_t payload[8]);

// Wait until twai_receive yields a frame matching `pred` or until timeout.
// During the wait, every received frame is also fed to the state-update
// pipeline (so unsolicited fault frames aren't lost). Returns ESP_OK with
// *out filled on match, ESP_ERR_TIMEOUT otherwise.
esp_err_t can_wait_match(can_match_fn pred, void *user,
                         uint32_t timeout_ms, twai_message_t *out);

// Drain any queued RX frames non-blocking (also runs the state pipeline).
void can_drain_rx_nonblocking(void);

// Read & clear TWAI alert flags. Updates bus_stats.
uint32_t can_read_alerts(void);

// Helper to fan-out an arbitrary frame through the state pipeline. Public
// only because diag_tests.c shares the dispatch.
void can_handle_rx(const twai_message_t *m);
