# Triton Diag Dongle — V1 (observe-only)

Standalone CAN diagnostic tool for RobStride RS-02 motors. Plug it into a single
motor's harness, connect a phone or laptop to its WiFi AP, and run tiered
diagnostics from the browser.

**V1 invariant:** the firmware only ever transmits Type-0 (`GET_DEVICE_ID`) and
Type-17 (`READ_PARAMETER`). It cannot enable, write parameters, set zero, save,
or send operation-control frames. Tier-3 motion tests will be added when the
hardware E-stop arrives.

## Hardware

| Function | Pin / part |
|---|---|
| MCU board | ESP32-S3-DevKitC-1 |
| CAN transceiver | SN65HVD230 (3V3, slope-control) |
| CAN TX → transceiver D | GPIO 20 |
| CAN RX ← transceiver R | GPIO 21 |
| Bus termination | 120 Ω fixed across CAN_H / CAN_L on the dongle |
| Phoenix 3.5 mm terminal | CAN_H · CAN_L · CAN_GND  (DO NOT route motor power) |
| OLED | SSD1306 128×32 I²C @ 0x3C, SDA=GPIO 8, SCL=GPIO 9 |
| Onboard ARGB | WS2812 on GPIO 48 (DevKitC-1 default) |
| Battery sense | 100 k / 100 k divider from Li-Po into GPIO 4 (ADC1_CH3) |
| Power | 3.7 V Li-Po → USB-C charger module → 5 V into the DevKit's `5V` pin |
| Power switch | Inline with Li-Po positive (or on the charger module) |

Wire CAN_GND to the dongle GND through the Phoenix center pin — the SN65HVD230
is **not** galvanically isolated.

## Build & flash

```bash
. $IDF_PATH/export.sh                       # ESP-IDF 5.5+
idf.py set-target esp32s3
idf.py menuconfig                           # optional: tweak AP password under "Triton Diag Dongle"
idf.py -p /dev/ttyUSB0 flash monitor
```

Default AP credentials: `TritonDiag-XXXX` / `tritondroids` (where `XXXX` is the
last 4 hex of the MAC). After connecting, any browser request redirects to the
dashboard (captive portal).

## Using it

1. Plug the Phoenix connector into a single motor's CAN harness.
2. Power the dongle (Li-Po or USB-C).
3. Wait ~2 seconds; the onboard ARGB will go from breathing-white to one of:
   - **solid green** — motor responding, no faults
   - **solid orange** — motor responding, fault flag set
   - **slow yellow blink** — AP up, no motor detected
   - **solid red** — bus alert (BUS_OFF / BUS_ERROR / TX_FAILED)
   - **magenta blink** — battery LOW
4. Connect a phone/laptop to the SSID, open any URL; the dashboard appears.
5. Click **🟢 Run safe diagnostics**. ~3 seconds later you'll see a verdict.

## Test tiers

| Tier | Tests | Notes |
|---|---|---|
| 0 | TWAI alert flags · idle frame count · passive sniff | electrical bus sanity, no TX |
| 1 | ID scan 0–127 · identity readout · round-trip latency · frame loss | finds & probes the motor |
| 2 | VBUS read · encoder stability · run-mode · fault register · param envelope | motor-side health checks |
| 3 | (intentionally absent in V1) | motion tests — added with E-stop hardware |

## File map

```
main/
  main.c          app entry, task spawn
  state.[ch]      mutex-protected shared state + telemetry ring
  robstride.[ch]  RS-02 protocol constants + frame decode (mirrors MotorTest/robstride.py)
  can_task.[ch]   single TWAI owner: monitor loop, test dispatch, RX fan-out
  diag_tests.[ch] Tier 0/1/2 sequences -> JSON envelope
  oled.[ch]       SSD1306 128x32 driver, 4-line status UI
  argb.[ch]       WS2812 state machine
  battery.[ch]    ADC1 oneshot Li-Po monitor
  web_task.[ch]   WiFi AP + DNS captive portal + HTTP/WS server
  web/index.html  embedded single-page dashboard
```
