#include "slcan.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>

static const int speed_table[9] = {10000,20000,50000,100000,125000,250000,500000,800000,1000000};

bool slcan_speed_to_bitrate(int s_code, int *bps) {
    if (s_code < 0 || s_code > 8) return false;
    if (bps) *bps = speed_table[s_code];
    return true;
}

bool slcan_ishex(char c) {
    return (c>='0'&&c<='9')||(c>='A'&&c<='F')||(c>='a'&&c<='f');
}

static int hex1v(char c) {
    if (c>='0'&&c<='9') return c-'0';
    if (c>='A'&&c<='F') return 10 + (c-'A');
    if (c>='a'&&c<='f') return 10 + (c-'a');
    return -1;
}

int slcan_hexn(const char *s, int n) {
    int v=0;
    for (int i=0;i<n;i++) {
        int h = hex1v(s[i]);
        if (h<0) return -1;
        v = (v<<4) | h;
    }
    return v;
}

char slcan_hex1(int v) {
    v &= 0xF;
    return (v<10)? ('0'+v) : ('A'+(v-10));
}

// Format TWAI frame into SLCAN text line (ending with '\r', no timestamp)
int slcan_format_frame(const twai_message_t *msg, char *out, int out_max) {
    // Format: tIIIDLDD.. or TIIIIIIIIDLDD..
    // where I.. = ID hex, D = DLC hex nibble, L.. = data bytes hex pairs
    int idx = 0;
    if (msg->extd) {
        if (out_max < 1) return -1;
        out[idx++] = msg->rtr ? 'R' : 'T';
        // 29-bit ID (8 hex)
        if (out_max < idx+8) return -1;
        for (int i=7;i>=0;i--) out[idx++] = slcan_hex1((msg->identifier >> (i*4)) & 0xF);
    } else {
        if (out_max < 1) return -1;
        out[idx++] = msg->rtr ? 'r' : 't';
        // 11-bit ID (3 hex)
        if (out_max < idx+3) return -1;
        for (int i=2;i>=0;i--) out[idx++] = slcan_hex1((msg->identifier >> (i*4)) & 0xF);
    }

    // DLC
    if (out_max < idx+1) return -1;
    out[idx++] = slcan_hex1(msg->data_length_code & 0xF);

    // Data (if not RTR)
    if (!msg->rtr) {
        if (out_max < idx + (msg->data_length_code*2)) return -1;
        for (int i=0;i<msg->data_length_code;i++) {
            out[idx++] = slcan_hex1((msg->data[i] >> 4) & 0xF);
            out[idx++] = slcan_hex1(msg->data[i] & 0xF);
        }
    }

    if (out_max < idx+1) return -1;
    out[idx++] = '\r';
    return idx;
}

// Parse one SLCAN line (without trailing '\r')
int slcan_parse_line(const char *line, twai_message_t *msg, slcan_state_t *st, bool *is_ctrl, char *ctrl_resp, int ctrl_resp_max) {
    const int len = (int)strlen(line);
    if (len <= 0) return -1;

    if (is_ctrl) *is_ctrl = false;
    if (ctrl_resp && ctrl_resp_max>0) ctrl_resp[0] = 0;

    char cmd = line[0];

    // Control commands
    if (cmd=='O') { // open
        if (st) st->opened = true;
        if (is_ctrl) *is_ctrl = true;
        if (ctrl_resp && ctrl_resp_max>=2) { ctrl_resp[0]='\r'; ctrl_resp[1]=0; }
        return 0;
    }
    if (cmd=='C') { // close
        if (st) st->opened = false;
        if (is_ctrl) *is_ctrl = true;
        if (ctrl_resp && ctrl_resp_max>=2) { ctrl_resp[0]='\r'; ctrl_resp[1]=0; }
        return 0;
    }
    if (cmd=='S' && len==2 && slcan_ishex(line[1])) { // set speed Sx
        int code = hex1v(line[1]);
        int bps;
        if (!slcan_speed_to_bitrate(code, &bps)) return -2;
        if (st) st->bitrate = bps;
        if (is_ctrl) *is_ctrl = true;
        if (ctrl_resp && ctrl_resp_max>=2) { ctrl_resp[0]='\r'; ctrl_resp[1]=0; }
        return 0;
    }
    if (cmd=='V') { // hardware version (minimal)
        if (is_ctrl) *is_ctrl = true;
        if (ctrl_resp && ctrl_resp_max>=6) { ctrl_resp[0]='V'; ctrl_resp[1]='1'; ctrl_resp[2]='0'; ctrl_resp[3]='0'; ctrl_resp[4]='\r'; ctrl_resp[5]=0; }
        return 0;
    }
    if (cmd=='v') { // software version
        if (is_ctrl) *is_ctrl = true;
        if (ctrl_resp && ctrl_resp_max>=6) { ctrl_resp[0]='v'; ctrl_resp[1]='1'; ctrl_resp[2]='0'; ctrl_resp[3]='0'; ctrl_resp[4]='\r'; ctrl_resp[5]=0; }
        return 0;
    }

    // Frame commands
    // tIII DL DD..   (std)
    // TIIIIIIII DL DD.. (ext)
    // r/R for RTR (no data)
    twai_message_t m = {0};
    bool ext=false, rtr=false;

    if (cmd=='t' || cmd=='r') { ext=false; rtr = (cmd=='r'); }
    else if (cmd=='T' || cmd=='R') { ext=true; rtr = (cmd=='R'); }
    else return -2;

    int idx = 1;
    int id_hex = ext ? 8 : 3;
    if (len < idx+id_hex+1) return -1;

    int id = slcan_hexn(&line[idx], id_hex);
    if (id<0) return -1;
    idx += id_hex;

    int dlc = hex1v(line[idx++]);
    if (dlc<0 || dlc>8) return -3;

    m.identifier = id;
    m.extd = ext;
    m.rtr = rtr;
    m.data_length_code = dlc;

    if (!rtr) {
        if (len < idx + dlc*2) return -1;
        for (int i=0;i<dlc;i++) {
            int b = slcan_hexn(&line[idx], 2);
            if (b<0) return -1;
            m.data[i] = (uint8_t)b;
            idx += 2;
        }
    }

    if (msg) *msg = m;
    return dlc; // non-negative means "frame parsed"
}

