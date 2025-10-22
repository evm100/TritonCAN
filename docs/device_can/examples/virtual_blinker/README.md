# Virtual Blinker Demo Device

*Last reviewed: 2024-06-09*

## 1. Device Overview
- **Device type:** Software-only demonstration node
- **Intended subsystem:** Development workstations using SocketCAN `vcan` devices
- **Key capabilities:** Sends and receives a single "blink" command with an incrementing counter
- **Official documentation:** This README (self-documented example)

The virtual blinker illustrates the minimum wiring required to exercise the
TritonCAN daemon with two endpoints sharing the same CAN bus. It is designed for
team members who want a "hello world" workflow before touching real hardware.

## 2. Mechanical & Electrical Specs
- **Physical layer:** Linux `vcan` virtual CAN interface
- **Bitrate:** 500 kbit/s (default SocketCAN speed, configurable in YAML)
- **Frame format:** Standard 11-bit identifiers
- **Supply:** None – frames are injected directly from the workstation

## 3. Harness & Pinout
Because the virtual blinker runs entirely in software, there are no connectors
or wiring harnesses. Ensure the host computer has the `vcan` kernel module
available (`modprobe vcan`) and that termination is not applied to the
interface.

## 4. CAN Bus Interface
The demo uses two arbitration IDs on the same virtual bus:

| Message name   | ID (hex) | Producer   | Consumer   | Purpose                     |
|----------------|----------|------------|------------|-----------------------------|
| `BlinkFromA`   | `0x200`  | Device A   | Device B   | Device A sends blink state  |
| `BlinkFromB`   | `0x201`  | Device B   | Device A   | Device B sends blink state  |

Each payload is two bytes:

| Signal       | Length | Units | Notes                                |
|--------------|--------|-------|--------------------------------------|
| `blink_state`| 8 bits | bool  | `0` = off, `1` = on (other values ok) |
| `sequence`   | 8 bits | count | Increments with every transmission    |

## 5. Command & Telemetry Map
| Command          | Transmitted By | Description                                |
|------------------|----------------|--------------------------------------------|
| `blink on`       | Either device  | Sets `blink_state = 1` on the target device|
| `blink off`      | Either device  | Sets `blink_state = 0` on the target device|
| `blink <value>`  | Either device  | Sends an arbitrary 0–255 value             |
| `toggle`         | Either device  | Toggles the local blink state before send  |

Telemetry is symmetric: when a device receives a frame it prints the state and
sequence counter provided by its counterpart.

## 6. Integration Playbook
1. Create two virtual CAN interfaces (only one is used but both are acceptable):
   ```bash
   sudo modprobe vcan
   sudo ip link add dev vcan0 type vcan
   sudo ip link set up vcan0
   ```
2. Launch the helper script `td_CAN/scripts/vcan_blink_launch.sh`. This spawns
   two terminal panes (via `tmux`) running `vcan_blink_device.py` for Device A
   and Device B respectively.
3. In either pane type one of the commands from Section 5 (e.g. `blink on`). The
   opposite pane should immediately print the received state and incrementing
   sequence number.
4. Exit with `quit` (or `Ctrl+C`). The script gracefully stops the daemon and
   closes the virtual bus.

## 7. API Module Mapping
- **Module name:** `examples.virtual_blinker`
- **Suggested helper calls:**
  - `send_blink(state: int)` → wraps the `blink` command for automation
  - `toggle()` → convenience wrapper around `toggle`
  - `observe(callback)` → subscribe to the RX handler for automated testing

## 8. Validation & Testing Notes
- The YAML/DBC pair live in `td_CAN/config/vcan_blink_demo.yaml` and
  `td_CAN/td_can_bridges/schemas/vcan_blink.dbc`.
- The interactive script logs all decoded frames to stdout. Use `--log-level
  DEBUG` to see encoded payloads as they are dispatched.
- This example intentionally avoids ROS to keep the feedback loop short.

## 9. Revision History
| Date       | Notes                                |
|------------|--------------------------------------|
| 2024-06-09 | Initial virtual blinker documentation |
