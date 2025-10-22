#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/twai.h"

#ifdef __cplusplus
extern "C" {
#endif

// Map SLCAN 'Sx' codes to bitrates (per Lawicel)
typedef enum {
    SLCAN_SPEED_10K   = 0,  // S0
    SLCAN_SPEED_20K   = 1,  // S1
    SLCAN_SPEED_50K   = 2,  // S2
    SLCAN_SPEED_100K  = 3,  // S3
    SLCAN_SPEED_125K  = 4,  // S4
    SLCAN_SPEED_250K  = 5,  // S5
    SLCAN_SPEED_500K  = 6,  // S6
    SLCAN_SPEED_800K  = 7,  // S7
    SLCAN_SPEED_1M    = 8   // S8
} slcan_speed_t;

typedef struct {
    bool opened;      // 'O' open vs 'C' close
    int  bitrate;     // active bitrate in bps
} slcan_state_t;

// Format a TWAI message into SLCAN ASCII. Returns length.
int slcan_format_frame(const twai_message_t *msg, char *out, int out_max);

// Parse a single complete SLCAN line (without trailing '\r') into action.
// Returns:
//   >=0  : number of bytes in CAN frame encoded in msg (or 0 for non-frame control like O/C/S)
//   -1   : not enough / bad format
//   -2   : unsupported command
//   -3   : invalid DLC
int slcan_parse_line(const char *line, twai_message_t *msg, slcan_state_t *st, bool *is_ctrl, char *ctrl_resp, int ctrl_resp_max);

// Convert Sx code to bitrate
bool slcan_speed_to_bitrate(int s_code, int *bps);

// Helpers for hex conversion
int  slcan_hexn(const char *s, int n);
bool slcan_ishex(char c);
char slcan_hex1(int v);

#ifdef __cplusplus
}
#endif

