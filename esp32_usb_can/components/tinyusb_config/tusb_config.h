#pragma once

// Minimal TinyUSB device config for CDC
#define CFG_TUSB_MCU OPT_MCU_ESP32S3
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE)

// Max number of device endpoints
#define CFG_TUD_ENDPOINT0_SIZE 64

// CDC: one interface
#define CFG_TUD_CDC 1
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0

// CDC RX/TX buffer sizes
#define CFG_TUD_CDC_RX_BUFSIZE (512)
#define CFG_TUD_CDC_TX_BUFSIZE (512)
#define CFG_TUD_CDC_EP_BUFSIZE (64)

