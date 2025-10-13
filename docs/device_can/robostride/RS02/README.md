# RoboStride RS02 CAN Dossier

*Last reviewed: 2024-05-19*

## 1. Device Overview
- **Device type:** Compact articulated actuator
- **Intended subsystem:** Lower-limb joints and small robotic appendages
- **Key capabilities:** 6 N·m continuous torque with 7.75:1 gearing, FOC drive, integrated
  encoder feedback
- **Official documentation:** [`RS02-EN.pdf`](./RS02-EN.pdf)

## 2. Mechanical & Electrical Specs
| Parameter | Value | Notes |
|-----------|-------|-------|
| Rated voltage | 48 VDC | Operating range 24–60 VDC |
| Rated load torque | 6 N·m | CW reference direction |
| Rated load speed | 360 rpm ±10% | No-load speed 410 rpm ±10% |
| Rated phase current (peak) | 7 A pk ±10% | Maximum 23 A pk ±10% |
| Peak torque | 17 N·m | Do not exceed to avoid warranty loss |
| Gear ratio | 7.75:1 | Output shaft |
| Weight | 380 g ±3 g | |
| Operating temperature | −20 °C to 50 °C | Storage −30 °C to 70 °C |
| Humidity range | 5–85 % RH | No condensation |

## 3. Harness & Pinout
- Dedicated **power + CAN harness** bundled with the actuator. Verify continuity before use and
  ensure the mating connector depth does not exceed the housing thread depth.
- **Download/debug port** available for firmware updates and parameter edits via the RoboStride
  upper-computer tool.
- **Status LED** assists with connection diagnostics. Inspect prior to deployment.
- When using the Lingzu USB–CAN adapter, set DIP switch 1 to *OFF* (normal mode) and DIP
  switch 2 to *ON* to engage the 120 Ω terminator during bench testing.

## 4. CAN Bus Interface
- **Physical layer:** CAN 2.0B @ 1 Mbps. Private protocol uses extended frames; MIT mode uses
  standard frames.
- **Recommended interface:** RoboStride USB–CAN (CH340). Serial framing uses header `0x41 0x54`
  and tail `0x0D 0x0A` when tunneling through the upper-computer tool.
- **Extended identifier layout:** The manual example converts the serial frame ID
  `0x9007E80C` → extended CAN ID `0x1200FD01`. Bits are partitioned into
  `{mode[4:0], id[7:0], master_id[?], reserved[2:0]}` matching the sample code’s
  `exCanIdInfo` structure. Use the helper macros in the example firmware to pack/unpack IDs.
- **Default mode:** Device powers up in Motion Control (private protocol). Use Command 8 / Type 25
  to switch to CANopen or MIT after saving and power-cycling.

## 5. Command & Telemetry Map (Private Protocol)
| Type | Purpose | Notes |
|------|---------|-------|
| 0 | Get device ID | Returns node ID + MCU 64-bit unique ID |
| 1 | Motion control command | Sends torque, position, speed, KP, KD (5 parameters) |
| 2 | Motor feedback | Position, velocity, temperature, status (used by responses) |
| 3 | Enable motor run | Expect feedback frame type 2 |
| 4 | Stop motor | Feedback frame type 2 |
| 6 | Set mechanical zero | Feedback frame type 2 |
| 7 | Set motor CAN ID | Takes effect immediately; reply is broadcast frame (type 0) |
| 17 | Read single parameter | Use for items like `loc_kp` |
| 18 | Write single parameter (volatile) | Pair with Type 22 to persist |
| 21 | Fault feedback | Publishes diagnostic info |
| 22 | Save parameters | Commits volatile changes to flash |
| 23 | Modify baud rate | Takes effect after reboot |
| 24 | Enable active reporting | Configures periodic Type 2 telemetry |
| 25 | Modify protocol | Switches Private/CANopen/MIT (requires reboot) |

**Fault map:** Function code `0x3022` bitfields report overload, encoder, over/under-voltage,
driver failure, and over-temperature. Codes `0x3024` and `0x3025` expose driver chip faults.
`CAN_TIMEOUT` defines watchdog timing (value `20000` → 1 s timeout).

## 6. Parameter Dictionary Highlights
- `zero_sta`: Zero-point flag (0 ⇒ power-on range 0–2π, 1 ⇒ π–π). Save with Type 22.
- `add_offset`: Position offset applied on boot; use to shift mechanical zero safely.
- `damper`: Set to 1 to disable passive damping when unpowered.
- `EPScan_time`: Controls active reporting interval when Type 24 is enabled.
- Protocol selection stored in `protocol_1` (mirrors Command 8 behavior).

## 7. Integration Playbooks
### Commissioning (Private Protocol)
1. Connect via USB–CAN with DIP switch 2 enabled for termination.
2. Issue Type 3 to enable, then Type 1 motion commands to validate torque/position loops.
3. Set mechanical zero with Type 6, verify feedback via Type 2, and persist using Type 22.

### MIT Velocity Mode Example
1. Command 6 with Mode = 2 (velocity).
2. Command 1 to enable.
3. Command 11 with desired max current and target velocity.
4. Command 2 to stop.

### MIT Position Mode Example
1. Command 6 with Mode = 1 (position/CSP).
2. Command 1 to enable.
3. Command 10 with target position and max speed limits.
4. Command 2 to stop.

## 8. API Module Mapping
- **Module name:** `robostride.rs02`
- **Suggested API calls:**
  - `enable()` → Type 3 / Command 1
  - `disable()` → Type 4 / Command 2
  - `set_motion(torque, position, speed, kp, kd)` → Type 1
  - `set_velocity(target_velocity, max_torque)` → Command 11
  - `set_position(target_position, max_speed)` → Command 10
  - `set_zero()` → Type 6 or Command 4 (depending on protocol)
  - `write_parameter(key, value, persist=False)` → Type 18 (+ Type 22 when `persist`)
  - `read_parameter(key)` → Type 17
  - `set_protocol(mode)` → Type 25 / Command 8
  - `subscribe_faults()` → Type 21 listener
- **Automation notes:** Metadata entries should include scaling for the five motion control
  fields and fault bit definitions to auto-generate API bindings.

## 9. Validation & Testing Notes
- Use the oscilloscope view in the upper-computer software to confirm encoder counts, current,
  and velocity while exercising each control mode.
- Capture baseline Type 2 telemetry at 10 ms interval before modifying `EPScan_time`.

## 10. Revision History
| Date | Notes |
|------|-------|
| 2024-05-19 | Initial import from RS02-EN.pdf and integration guidance prepared. |
