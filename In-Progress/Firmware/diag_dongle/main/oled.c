#include "oled.h"
#include "state.h"
#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "oled";

#define OLED_ADDR    0x3C
#define OLED_WIDTH   128
#define OLED_HEIGHT  32
#define OLED_PAGES   (OLED_HEIGHT / 8)   // 4 pages
#define OLED_LINE_CHARS  21              // 128 / 6

static i2c_master_bus_handle_t  s_bus  = NULL;
static i2c_master_dev_handle_t  s_dev  = NULL;
static bool                     s_ok   = false;
static uint8_t                  s_fb[OLED_WIDTH * OLED_PAGES];

// 5x7 ASCII font (public domain, widely redistributed; one byte per column,
// LSB = top pixel). Covers ASCII 0x20..0x7E. Char cell drawn as 5 cols + 1 gap.
static const uint8_t s_font[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // 0x20 ' '
    {0x00,0x00,0x5F,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
    {0x23,0x13,0x08,0x64,0x62}, // '%'
    {0x36,0x49,0x55,0x22,0x50}, // '&'
    {0x00,0x05,0x03,0x00,0x00}, // '\''
    {0x00,0x1C,0x22,0x41,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00}, // ')'
    {0x14,0x08,0x3E,0x08,0x14}, // '*'
    {0x08,0x08,0x3E,0x08,0x08}, // '+'
    {0x00,0x50,0x30,0x00,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08}, // '-'
    {0x00,0x60,0x60,0x00,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02}, // '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
    {0x00,0x36,0x36,0x00,0x00}, // ':'
    {0x00,0x56,0x36,0x00,0x00}, // ';'
    {0x08,0x14,0x22,0x41,0x00}, // '<'
    {0x14,0x14,0x14,0x14,0x14}, // '='
    {0x00,0x41,0x22,0x14,0x08}, // '>'
    {0x02,0x01,0x51,0x09,0x06}, // '?'
    {0x32,0x49,0x79,0x41,0x3E}, // '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x0C,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x07,0x08,0x70,0x08,0x07}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // '['
    {0x02,0x04,0x08,0x10,0x20}, // '\\'
    {0x00,0x41,0x41,0x7F,0x00}, // ']'
    {0x04,0x02,0x01,0x02,0x04}, // '^'
    {0x40,0x40,0x40,0x40,0x40}, // '_'
    {0x00,0x01,0x02,0x04,0x00}, // '`'
    {0x20,0x54,0x54,0x54,0x78}, // 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 't'
    {0x3C,0x40,0x40,0x20,0x7C}, // 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 'z'
    {0x00,0x08,0x36,0x41,0x00}, // '{'
    {0x00,0x00,0x7F,0x00,0x00}, // '|'
    {0x00,0x41,0x36,0x08,0x00}, // '}'
    {0x08,0x08,0x2A,0x1C,0x08}, // '~'
};
#define FONT_FIRST 0x20
#define FONT_LAST  0x7E

static esp_err_t oled_cmd(uint8_t c) {
    uint8_t buf[2] = { 0x00, c };  // Co=0, D/C#=0
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 50);
}

static esp_err_t oled_cmds(const uint8_t *cmds, size_t n) {
    for (size_t i = 0; i < n; i++) {
        esp_err_t e = oled_cmd(cmds[i]);
        if (e != ESP_OK) return e;
    }
    return ESP_OK;
}

static esp_err_t oled_send_fb(void) {
    // Set address window for full screen.
    static const uint8_t setup[] = {
        0x21, 0, 127,        // column addr range
        0x22, 0, OLED_PAGES - 1,
    };
    if (oled_cmds(setup, sizeof(setup)) != ESP_OK) return ESP_FAIL;

    // Send fb in chunks with leading data tag (0x40).
    uint8_t chunk[33];
    chunk[0] = 0x40;
    size_t off = 0;
    while (off < sizeof(s_fb)) {
        size_t n = sizeof(s_fb) - off;
        if (n > 32) n = 32;
        memcpy(&chunk[1], &s_fb[off], n);
        esp_err_t e = i2c_master_transmit(s_dev, chunk, n + 1, 50);
        if (e != ESP_OK) return e;
        off += n;
    }
    return ESP_OK;
}

static void oled_clear(void) { memset(s_fb, 0, sizeof(s_fb)); }

static void put_glyph(int col_px, int page, char c) {
    if (col_px < 0 || col_px + 5 > OLED_WIDTH) return;
    if (page  < 0 || page >= OLED_PAGES) return;
    if (c < FONT_FIRST || c > FONT_LAST) c = '?';
    const uint8_t *g = s_font[c - FONT_FIRST];
    int base = page * OLED_WIDTH + col_px;
    for (int i = 0; i < 5; i++) s_fb[base + i] = g[i];
    s_fb[base + 5] = 0;  // 1px gap
}

static void put_text(int line, const char *s) {
    if (line < 0 || line >= OLED_PAGES) return;
    int col = 0;
    while (*s && col + 6 <= OLED_WIDTH) {
        put_glyph(col, line, *s++);
        col += 6;
    }
}

bool oled_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .clk_source                  = I2C_CLK_SRC_DEFAULT,
        .i2c_port                    = I2C_NUM_0,
        .scl_io_num                  = (gpio_num_t)CONFIG_DIAG_OLED_SCL_GPIO,
        .sda_io_num                  = (gpio_num_t)CONFIG_DIAG_OLED_SDA_GPIO,
        .glitch_ignore_cnt           = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed");
        return false;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = OLED_ADDR,
        .scl_speed_hz    = 400000,
    };
    if (i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev) != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed");
        return false;
    }
    static const uint8_t init_seq[] = {
        0xAE,                  // display off
        0xD5, 0x80,            // display clock div
        0xA8, 0x1F,            // multiplex (32-1)
        0xD3, 0x00,            // display offset
        0x40,                  // start line 0
        0x8D, 0x14,            // charge pump on
        0x20, 0x00,            // memory mode = horizontal
        0xA1,                  // segment remap
        0xC8,                  // com scan dec
        0xDA, 0x02,            // com pins for 128x32
        0x81, 0x8F,            // contrast
        0xD9, 0xF1,            // pre-charge
        0xDB, 0x40,            // vcomh deselect
        0xA4,                  // resume to ram
        0xA6,                  // normal display
        0x2E,                  // deactivate scroll
        0xAF,                  // display on
    };
    if (oled_cmds(init_seq, sizeof(init_seq)) != ESP_OK) {
        ESP_LOGW(TAG, "OLED not responding (no display attached?)");
        return false;
    }
    oled_clear();
    if (oled_send_fb() != ESP_OK) return false;
    s_ok = true;
    ESP_LOGI(TAG, "OLED up: 128x32 SSD1306 on SDA=%d SCL=%d",
             CONFIG_DIAG_OLED_SDA_GPIO, CONFIG_DIAG_OLED_SCL_GPIO);
    return true;
}

void oled_refresh_from_state(const char *ap_ssid) {
    if (!s_ok) return;
    motor_state_t   ms;  state_get_motor(&ms);
    bus_stats_t     bs;  state_get_bus(&bs);
    battery_state_t ba;  state_get_battery(&ba);

    // Slightly oversized: snprintf may format more than 21 chars; put_text()
    // clips at the OLED width regardless. Avoids -Werror=format-truncation.
    char l1[64];
    char l2[64];
    char l3[64];
    char l4[64];

    // Line 1: AP name + battery voltage.
    snprintf(l1, sizeof(l1), "%-14s %1d.%02dV",
             ap_ssid ? ap_ssid : "TritonDiag",
             ba.vbat_mv / 1000, (ba.vbat_mv % 1000) / 10);

    // Line 2: motor identity.
    if (!ms.detected) {
        if (bs.cum_alerts & 0x01) snprintf(l2, sizeof(l2), "Bus error - check wiring");
        else                      snprintf(l2, sizeof(l2), "No motor detected...");
    } else if (ms.faults || ms.warnings || ms.fb_fault_bits) {
        snprintf(l2, sizeof(l2), "M%-3u FAULT 0x%02X", ms.motor_id,
                 (unsigned)(ms.fb_fault_bits | (ms.faults & 0xFF)));
    } else {
        snprintf(l2, sizeof(l2), "M%-3u RS-02 OK", ms.motor_id);
    }

    // Line 3: telemetry.
    if (ms.detected) {
        snprintf(l3, sizeof(l3), "P%+5.2f V%4.1f T%4.1fC",
                 (double)ms.mech_pos_rad, (double)ms.vbus_v, (double)ms.temperature_c);
    } else {
        snprintf(l3, sizeof(l3), "RX %lu frames",
                 (unsigned long)bs.rx_total);
    }

    // Line 4: last test verdict, truncated. Keep it short — user has phone.
    char tj[256];
    int n = state_copy_test_result(tj, sizeof(tj));
    if (n > 0) {
        // Find "verdict":"..." substring
        const char *p = strstr(tj, "\"verdict\":\"");
        if (p) {
            p += strlen("\"verdict\":\"");
            char tmp[64] = {0};
            int  k = 0;
            while (*p && *p != '"' && k < (int)sizeof(tmp) - 1) tmp[k++] = *p++;
            snprintf(l4, sizeof(l4), "%s", tmp);
        } else {
            snprintf(l4, sizeof(l4), "test result available");
        }
    } else if (ba.vbat_mv > 0 && ba.vbat_mv < CONFIG_DIAG_BATT_LOW_MV) {
        snprintf(l4, sizeof(l4), "Battery LOW");
    } else {
        snprintf(l4, sizeof(l4), "Open phone -> AP web");
    }

    oled_clear();
    put_text(0, l1);
    put_text(1, l2);
    put_text(2, l3);
    put_text(3, l4);
    oled_send_fb();
}
