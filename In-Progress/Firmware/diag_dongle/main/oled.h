#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * SSD1306 0.91" 128x32 OLED driver. 6x8 cell, 21 chars per line, 4 lines.
 * Address fixed at 0x3C. I2C pins from Kconfig.
 */

bool oled_init(void);

// Render the four-line status frame from current shared state.
// Called from a low-priority refresh task at ~2 Hz.
void oled_refresh_from_state(const char *ap_ssid);
