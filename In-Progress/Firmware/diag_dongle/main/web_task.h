#pragma once

#include <stdint.h>

void web_task_start(void);

// SSID generated at boot ("TritonDiag-XXXX"). Provided for OLED.
const char *web_ap_ssid(void);
