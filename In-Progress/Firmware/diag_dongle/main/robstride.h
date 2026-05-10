#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * RobStride RS-02/03/04 private extended-frame protocol.
 * Mirrors MotorTest/robstride.py. V1 firmware is observe-only:
 * we only ever transmit Type 0 (GET_DEVICE_ID) and Type 17 (READ_PARAMETER).
 */

// Communication types (29-bit CAN ID, bits 24..28).
typedef enum {
    RS_COMM_GET_DEVICE_ID        = 0,
    RS_COMM_OPERATION_CONTROL    = 1,   // V2 only
    RS_COMM_MOTOR_FEEDBACK       = 2,
    RS_COMM_ENABLE               = 3,   // V2 only
    RS_COMM_STOP                 = 4,   // V2 only
    RS_COMM_SET_ZERO_POSITION    = 6,   // V2 only
    RS_COMM_SET_CAN_ID           = 7,
    RS_COMM_READ_PARAMETER       = 17,
    RS_COMM_WRITE_PARAMETER      = 18,  // V2 only
    RS_COMM_FAULT_REPORT         = 21,
    RS_COMM_SAVE_PARAMETERS      = 22,  // V2 only
    RS_COMM_SET_BAUD_RATE        = 23,  // V2 only
    RS_COMM_ENABLE_ACTIVE_REPORT = 24,
    RS_COMM_SET_PROTOCOL         = 25,
} rs_comm_t;

// RS-02 MIT scaling (used to decode Type 2 feedback frames).
#define RS02_P_MAX   12.57f   // ~4*pi rad
#define RS02_V_MAX   44.0f    // rad/s
#define RS02_T_MAX   17.0f    // N*m
#define RS02_KP_MAX  500.0f
#define RS02_KD_MAX  5.0f

// Selected parameter indices we actually read in V1 diagnostics.
typedef enum {
    RS_PARAM_RUN_MODE        = 0x7005, // u8
    RS_PARAM_LIMIT_TORQUE    = 0x700B, // f32
    RS_PARAM_POSITION_TARGET = 0x7016, // f32
    RS_PARAM_LIMIT_SPD       = 0x7017, // f32
    RS_PARAM_LIMIT_CUR       = 0x7018, // f32
    RS_PARAM_MECH_POS        = 0x7019, // f32
    RS_PARAM_IQF             = 0x701A, // f32
    RS_PARAM_MECH_VEL        = 0x701B, // f32
    RS_PARAM_VBUS            = 0x701C, // f32
    RS_PARAM_ROTATION        = 0x701D, // i16
    RS_PARAM_LOC_KP          = 0x701E, // f32
    RS_PARAM_SPD_KP          = 0x701F, // f32
    RS_PARAM_SPD_KI          = 0x7020, // f32
} rs_param_t;

// Decoded view of a 29-bit ID.
typedef struct {
    uint8_t  comm_type;  // bits 24..28
    uint16_t data;       // bits  8..23 (host_id in TX, motor_id+flags in RX)
    uint8_t  id_byte;    // bits  0..7  (motor_id in TX, host_id in RX)
} rs_id_t;

// Decoded MIT-scaled feedback (Type 2) payload.
typedef struct {
    float   position_rad;
    float   velocity_rps;
    float   torque_nm;
    float   temperature_c;
    uint8_t mode;       // bits 22..23 of CAN id data field per manual
    uint8_t fault_bits; // bits 16..21 of CAN id data field
    uint8_t motor_id;   // bits 8..15 of CAN id data field
} rs_feedback_t;

// Decoded read-parameter (Type 17) reply payload.
typedef struct {
    uint16_t index;
    uint8_t  raw[4];   // value bytes 4..7 of payload
} rs_param_reply_t;

// Build an extended 29-bit ID per the standard layout.
// type:    bits 24..28
// data16:  bits  8..23 (host_id when querying, motor-side data when receiving)
// id_byte: bits  0..7  (motor_id when querying, host_id when receiving)
static inline uint32_t rs_build_ext_id(uint8_t type, uint16_t data16, uint8_t id_byte) {
    return ((uint32_t)(type & 0x1F) << 24)
         | ((uint32_t)(data16    ) <<  8)
         |  (uint32_t)id_byte;
}

// Decode a 29-bit ID into its three parts.
static inline rs_id_t rs_decode_ext_id(uint32_t id) {
    rs_id_t out;
    out.comm_type = (uint8_t) ((id >> 24) & 0x1F);
    out.data      = (uint16_t)((id >>  8) & 0xFFFF);
    out.id_byte   = (uint8_t) ( id        & 0xFF);
    return out;
}

// Decode a Type-2 feedback frame.
// Returns true on success, false if the frame is too short.
bool rs_decode_feedback(uint32_t can_id, const uint8_t *data, size_t dlc,
                        rs_feedback_t *out);

// Decode a Type-17 read-parameter reply.
bool rs_decode_param_reply(const uint8_t *data, size_t dlc, rs_param_reply_t *out);

// Decode a Type-21 fault report. Returns the 32-bit fault flags + 32-bit warning
// flags. Exact bit meanings vary by firmware revision -- exposed raw, decoded
// in the web UI.
typedef struct {
    uint32_t faults;
    uint32_t warnings;
} rs_fault_t;
bool rs_decode_fault(const uint8_t *data, size_t dlc, rs_fault_t *out);

// Helpers to read a little-endian f32/i16/u8 out of a 4-byte param payload.
float    rs_param_to_f32(const uint8_t b[4]);
int16_t  rs_param_to_i16(const uint8_t b[4]);
uint8_t  rs_param_to_u8 (const uint8_t b[4]);

// Build the 8-byte payload for a Type-17 READ_PARAMETER request.
// Layout: [u16 index little-endian][6 bytes zero].
void rs_build_read_param_payload(uint16_t index, uint8_t out[8]);
