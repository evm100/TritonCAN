# RoboStride RS04 CAN Dossier

*Last reviewed: 2024-05-19*

## 1. Device Overview
- **Device type:** High-torque articulated actuator
- **Intended subsystem:** Large-joint actuation, heavy payload platforms
- **Key capabilities:** 40 N·m continuous torque, 120 N·m peak, 9:1 gearing, integrated FOC
- **Official documentation:** [`RS04-EN.pdf`](./RS04-EN.pdf)

## 2. Mechanical & Electrical Specs
| Parameter | Value | Notes |
|-----------|-------|-------|
| Rated voltage | 48 VDC | Operating range 24–60 VDC |
| Rated load torque | 40 N·m | CW reference direction |
| Rated load speed | 167 rpm ±10% | No-load speed 200 rpm ±10% |
| Rated phase current (peak) | 27 A pk ±10% | Maximum 90 A pk ±10% |
| Peak torque | 120 N·m | Maintain thermal cap at 130 °C |
| Gear ratio | 9:1 | Output shaft |
| Weight | 1420 g ±20 g | |
| Operating temperature | −20 °C to 50 °C | Storage −30 °C to 70 °C |
| Humidity range | 5–85 % RH | No condensation |

## 3. Harness & Pinout
- Same wiring bundle and DIP-switch guidance as RS02/RS03.
- Ensure adequate gauge power leads; peak current draws can exceed 90 A (phase) under short
  bursts.

## 4. CAN Bus Interface
- **Physical layer:** CAN 2.0B @ 1 Mbps.
- **Extended frame structure:** Private protocol uses extended identifiers identical to RS02/RS03.
- **Protocol switching:** Command 8 / Type 25 selects Private, CANopen, or MIT (reboot required).

## 5. Command & Telemetry Map (Private Protocol)
The RS04 uses the same command types, telemetry, and fault registers as RS02. Reuse the
automation logic; only torque/current scaling differs.

## 6. Parameter Dictionary Highlights
Shared parameter set with RS02/RS03. Update safety thresholds to account for higher current and
thermal output.

## 7. Integration Playbooks
- Commission using Type 3 → Type 1 loop tests → Type 6 zero → Type 22 save.
- MIT mode switching flows identical to RS02; update target torque/speed envelopes for the higher
  continuous current.

## 8. API Module Mapping
- **Module name:** `robostride.rs04`
- Provide the same API functions with updated metadata: `max_torque = 120 N·m`,
  `max_current = 90 A pk`, recommended torque slew limits, etc.

## 9. Validation & Testing Notes
- Monitor winding temperature rise closely during sustained 40 N·m tests; log Type 21 faults for
  over-temperature events.

## 10. Revision History
| Date | Notes |
|------|-------|
| 2024-05-19 | Initial import from RS04-EN.pdf with shared protocol notes. |
