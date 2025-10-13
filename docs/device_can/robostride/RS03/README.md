# RoboStride RS03 CAN Dossier

*Last reviewed: 2024-05-19*

## 1. Device Overview
- **Device type:** Mid-torque articulated actuator
- **Intended subsystem:** Knee/hip joints, medium payload manipulators
- **Key capabilities:** 20 N·m continuous torque with 9:1 gearing, FOC drive, integrated encoder
- **Official documentation:** [`RS03-EN.pdf`](./RS03-EN.pdf)

## 2. Mechanical & Electrical Specs
| Parameter | Value | Notes |
|-----------|-------|-------|
| Rated voltage | 48 VDC | Operating range 24–60 VDC |
| Rated load torque | 20 N·m | CW reference direction |
| Rated load speed | 180 rpm ±10% | No-load speed 200 rpm ±10% |
| Rated phase current (peak) | 13 A pk ±10% | Maximum 43 A pk ±10% |
| Peak torque | 60 N·m | Observe thermal limits (130 °C winding cap) |
| Gear ratio | 9:1 | Output shaft |
| Weight | 880 g ±20 g | |
| Operating temperature | −20 °C to 50 °C | Storage −30 °C to 70 °C |
| Humidity range | 5–85 % RH | No condensation |

## 3. Harness & Pinout
- Same connector stack as RS02 (power/CAN harness, download port, indicator LED).
- Verify mechanical clearance; mounting screws must not exceed housing thread depth.
- For the Lingzu USB–CAN adapter: DIP 1 = Boot (keep OFF), DIP 2 = termination (enable for
  bench work).

## 4. CAN Bus Interface
- **Physical layer:** CAN 2.0B @ 1 Mbps. Private protocol uses extended frames; MIT mode uses
  standard frames.
- **Recommended interface:** RoboStride USB–CAN (CH340) with serial frame header `0x41 0x54`
  and tail `0x0D 0x0A`.
- **Extended identifier layout:** Same `exCanIdInfo` structure as RS02. Example conversion
  `0x9007E80C` → `0x1200FD01` (mode/id/master bits) demonstrates how the private protocol packs
  command type and node ID into the extended identifier.
- **Protocol switching:** Command 8 (or Type 25) toggles Private ↔ CANopen ↔ MIT. Takes effect
  after save + power cycle.

## 5. Command & Telemetry Map (Private Protocol)
Refer to the RS02 table—the RS03 shares the exact command surface (Types 0,1,2,3,4,6,7,17,18,21,22,23,24,25) and fault registers `0x3022/0x3024/0x3025`.

## 6. Parameter Dictionary Highlights
Identical to RS02 (`zero_sta`, `add_offset`, `damper`, `EPScan_time`, `protocol_1`). Update
values with Type 18 and persist with Type 22.

## 7. Integration Playbooks
- **Commissioning:** Enable with Type 3 → send Type 1 motion commands → zero with Type 6 → save
  via Type 22.
- **MIT Velocity/Position Modes:** Follow the same command sequences (6 → 1 → 11/10 → 2) with
  torque/speed limits sized for the RS03 ratings.

## 8. API Module Mapping
- **Module name:** `robostride.rs03`
- Mirror RS02 API surface but adjust metadata for torque/current limits so automation can enforce
  bounds (e.g., `max_current = 43 A pk`, `max_torque = 60 N·m`).

## 9. Validation & Testing Notes
- Use the upper-computer oscilloscope to watch Iq and winding temperature under 20 N·m load.
- Log Type 21 fault frames during endurance tests to verify thermal headroom.

## 10. Revision History
| Date | Notes |
|------|-------|
| 2024-05-19 | Initial import from RS03-EN.pdf with protocol alignment. |
