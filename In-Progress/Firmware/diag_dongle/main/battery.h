#pragma once

/*
 * 1S Li-Po monitor on a 100k/100k divider into ADC1.
 * Updates state.battery_state at ~1 Hz.
 */

void battery_start(void);
