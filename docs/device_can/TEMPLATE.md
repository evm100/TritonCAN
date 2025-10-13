# <Vendor> <Device> CAN Dossier

*Last reviewed: <YYYY-MM-DD>*

## 1. Device Overview
- **Device type:** <actuator/sensor/etc>
- **Intended subsystem:** <robotic leg, arm, etc>
- **Key capabilities:** <torque range, sensors, etc>
- **Official documentation:** [`<filename>`](./<filename>)

## 2. Mechanical & Electrical Specs
| Parameter | Value | Notes |
|-----------|-------|-------|
| Rated voltage |  | |
| Operating voltage |  | |
| Rated torque/load |  | |
| Peak torque |  | |
| Gear ratio |  | |
| Weight |  | |

Add any other salient parameters needed for integration (temperature limits, connector part
numbers, etc.).

## 3. Harness & Pinout
- Describe power and CAN connectors, wire colors, and any termination requirements.
- Include wiring diagrams or images if available.

## 4. CAN Bus Interface
- **Physical layer:** e.g., CAN 2.0B, 1 Mbps, requires 120 Ω termination.
- **Default CAN ID range:** note factory defaults and how to change them.
- **Frame format:** describe standard vs extended frame layout, including how command type,
  node ID, and payload length map into the identifier and payload bytes.
- Provide worked example frames where possible.

## 5. Command & Telemetry Map
Break commands into logical groups:
- Startup/shutdown (enable, disable, fault reset)
- Control modes (torque/current, velocity, position)
- Parameter read/write operations
- Fault and diagnostic reporting

For each command include:
- Trigger (API call name we expose)
- CAN frame requirements (ID, DLC, payload schema)
- Expected response frame(s)
- Error conditions / fault bits

## 6. Parameter Dictionary
Document adjustable parameters with identifiers, scaling, valid ranges, and persistence behavior
(e.g., requires Type 22 save, volatile until reboot, etc.).

## 7. Integration Playbooks
Provide cookbook-style workflows for common tasks (e.g., commissioning, zeroing, running in
velocity mode). Reference the commands defined above.

## 8. API Module Mapping
Sketch how this device should surface inside the dynamic API layer:
- Proposed module name and version
- Plain-language function names (e.g., `set_joint_torque`, `home_encoder`)
- Underlying CAN command(s) each function should call
- Notes for automation on how to generate argument/response schemas

## 9. Validation & Testing Notes
List any available test scripts, log captures, or acceptance criteria.

## 10. Revision History
Keep a changelog of updates to this dossier and firmware revisions tested.
