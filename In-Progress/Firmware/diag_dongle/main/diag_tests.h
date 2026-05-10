#pragma once

/*
 * Diagnostic test sequences. Each function returns a malloc'd JSON string
 * (caller frees) with a result envelope:
 *
 *   { "tier": "tier1", "ok": true,
 *     "tests":  [ { "id": "1.1", "name": "ID scan",
 *                   "pass": true, "metrics": {...}, "detail": "..." }, ... ],
 *     "verdict": "Bus + motor healthy.",
 *     "next_action": "Run Tier 2 motor health checks." }
 *
 * All TX traffic is restricted to Type-0 (GET_DEVICE_ID) and Type-17
 * (READ_PARAMETER). Run only from within can_task.
 */

char *diag_run_tier0 (void);
char *diag_run_tier1 (void);
char *diag_run_tier2 (void);
char *diag_run_all   (void);
char *diag_run_rescan(void);
