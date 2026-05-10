#pragma once

#include <stdbool.h>

/*
 * Onboard WS2812 status LED state machine.
 *
 * Pattern is derived from shared state every 50 ms:
 *   white breathing      = boot before WiFi up
 *   slow yellow blink    = AP up, no motor detected
 *   solid green          = motor detected, no faults
 *   solid orange         = motor detected, fault flag set
 *   solid red            = bus alert (BUS_ERROR / TX_FAILED / BUS_OFF)
 *   magenta blink (slow) = battery LOW (overrides)
 *   magenta strobe       = battery CRITICAL (overrides)
 */

void argb_start(void);

// Boot/init phase override -- shows breathing white.
void argb_set_booting(bool booting);
