#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_
#ifdef __cplusplus
extern "C" {
#endif
#define CFG_TUSB_MCU               OPT_MCU_ESP32S3
#define CFG_TUSB_RHPORT0_MODE      OPT_MODE_DEVICE
#define CFG_TUSB_OS                OPT_OS_FREERTOS
#define CFG_TUD_ENABLED            1
#define CFG_TUD_ENDPOINT0_SIZE     64
#define CFG_TUD_VENDOR             1
#define CFG_TUD_VENDOR_RX_BUFSIZE  512
#define CFG_TUD_VENDOR_TX_BUFSIZE  512
#define CFG_TUD_CONTROL_COMPLETE_CALLBACK 1
#ifdef __cplusplus
}
#endif
#endif
