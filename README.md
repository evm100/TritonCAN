# TritonCAN Monorepo

TritonCAN collects the firmware, ROS 2 integration tooling, and reference
documentation used to bridge robotics controllers onto Controller Area Network
(CAN) buses. The repository is organized as a monorepo so that hardware bring-up,
desktop development, and integration docs stay in sync.

## Projects

### `docs/` – Reference documentation
* Device CAN dossiers stored under `docs/device_can/` with vendor PDFs, metadata,
  and integration guides for individual devices.
* Templates that keep new additions consistent so the automation pipeline can
  ingest device specifications in the future.
* Start here when you need authoritative information about wiring, signal
  meaning, and CAN payload layouts.

### `esp32_usb_can/` – ESP32-S3 USB↔CAN bridge firmware
* ESP-IDF project that turns an ESP32-S3 into a USB CDC (SLCAN) to CAN bus
  adapter using the on-chip TWAI controller.
* Implements TinyUSB for the CDC transport and spins FreeRTOS tasks to shuttle
  frames between USB and CAN with optional bitrate control via SLCAN commands.
* Build with the ESP-IDF toolchain (`idf.py build`) and flash to supported
  ESP32-S3 development boards to obtain a plug-and-play desktop CAN interface.

### `python_can/` – ROS 2 SocketCAN bridge package
* ROS 2 package (`td_can_bridges`) that manages one or more SocketCAN interfaces
  and republishes CAN traffic as typed ROS topics using DBC schemas.
* Provides example YAML configs, launch files, and schemas for both single- and
  multi-bus setups.
* Install inside a ROS 2 workspace (`colcon build`) and launch with
  `ros2 launch td_can_bridges td_can_multibus.launch.py config:=<path>` to bring
  up the bridge node.

## Getting Started

1. Clone the repo and initialize the ESP-IDF/ROS 2 environments as needed for
   the components you plan to work on.
2. Review the documentation under `docs/device_can/` for hardware-specific
   wiring and message definitions.
3. Pick the relevant project:
   * Firmware developers work out of `esp32_usb_can/` using the ESP-IDF tools.
   * Robotics/ROS developers use the `python_can/` package with `colcon`.

## Contributing

* Open issues or pull requests against the relevant sub-project.
* Keep documentation updates alongside firmware or bridge changes to ensure the
  docs remain in sync with the code.

