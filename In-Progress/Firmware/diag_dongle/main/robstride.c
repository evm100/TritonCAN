#include "robstride.h"

#include <string.h>

static float u2f(uint16_t u, float x_min, float x_max) {
    float span = x_max - x_min;
    return (float)u * span / 65535.0f + x_min;
}

bool rs_decode_feedback(uint32_t can_id, const uint8_t *data, size_t dlc,
                        rs_feedback_t *out) {
    if (dlc < 8 || out == NULL || data == NULL) return false;

    rs_id_t id = rs_decode_ext_id(can_id);
    out->motor_id   = (uint8_t)( id.data       & 0xFF);
    out->fault_bits = (uint8_t)((id.data >> 8) & 0x3F);
    out->mode       = (uint8_t)((id.data >> 14) & 0x03);

    // Big-endian u16s in payload (matches >HHHH pack in robstride.py).
    uint16_t p_u = ((uint16_t)data[0] << 8) | data[1];
    uint16_t v_u = ((uint16_t)data[2] << 8) | data[3];
    uint16_t t_u = ((uint16_t)data[4] << 8) | data[5];
    uint16_t T_u = ((uint16_t)data[6] << 8) | data[7];

    out->position_rad   = u2f(p_u, -RS02_P_MAX, RS02_P_MAX);
    out->velocity_rps   = u2f(v_u, -RS02_V_MAX, RS02_V_MAX);
    out->torque_nm      = u2f(t_u, -RS02_T_MAX, RS02_T_MAX);
    out->temperature_c  = (float)T_u / 10.0f;
    return true;
}

bool rs_decode_param_reply(const uint8_t *data, size_t dlc, rs_param_reply_t *out) {
    if (dlc < 8 || out == NULL || data == NULL) return false;
    out->index = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    memcpy(out->raw, &data[4], 4);
    return true;
}

bool rs_decode_fault(const uint8_t *data, size_t dlc, rs_fault_t *out) {
    if (dlc < 8 || out == NULL || data == NULL) return false;
    out->faults   = (uint32_t)data[0]
                  | ((uint32_t)data[1] << 8)
                  | ((uint32_t)data[2] << 16)
                  | ((uint32_t)data[3] << 24);
    out->warnings = (uint32_t)data[4]
                  | ((uint32_t)data[5] << 8)
                  | ((uint32_t)data[6] << 16)
                  | ((uint32_t)data[7] << 24);
    return true;
}

float rs_param_to_f32(const uint8_t b[4]) {
    float f;
    memcpy(&f, b, sizeof(f));
    return f;
}

int16_t rs_param_to_i16(const uint8_t b[4]) {
    return (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

uint8_t rs_param_to_u8(const uint8_t b[4]) {
    return b[0];
}

void rs_build_read_param_payload(uint16_t index, uint8_t out[8]) {
    memset(out, 0, 8);
    out[0] = (uint8_t)( index       & 0xFF);
    out[1] = (uint8_t)((index >> 8) & 0xFF);
}
