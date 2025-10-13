# TritonCAN Service Layer API

This document describes the stable YAML/DBC driven interface for the CAN
service and the Python API that powers both ROS and non-ROS integrations.
The intent is to make it easy for any team to add devices without knowing the
internals of `rclpy` or the ROS 2 launch system.

## 1. High level architecture

```
YAML config  ─┐
              ├─> `load_bridge_config(...)` ──> `BridgeConfig`
DBC files   ─┘
                               │
                               ▼
                        `CanBusService`
                               │
                ┌──────────────┴──────────────┐
                ▼                             ▼
         ROS bridge (`BusWorker`)        Custom clients
```

* **BridgeConfig** – parsed view of a YAML file containing one or more CAN
  buses and their frame mappings.
* **CanBusService** – reusable runtime that opens SocketCAN, loads the DBC
  database and takes care of RX/TX loops and CAN message encoding/decoding.
* **ROS bridge** – thin wrapper that maps ROS publishers/subscribers to the
  service (`BusWorker`, `TopicTxBinding`, `RxBinding`).

Your project can either consume the YAML definitions through the ROS bridge or
instantiate `CanBusService` directly to hook up any other application.

## 2. YAML schema

Each YAML file contains three top-level keys:

```yaml
buses:            # REQUIRED – list of CAN interfaces
logging: {}       # OPTIONAL – extra metadata, e.g. log level
qos: {}           # OPTIONAL – ROS-specific QoS presets (ignored by non-ROS)
```

### 2.1 Bus entries

A minimal bus definition looks like this:

```yaml
buses:
  - name: motor_bus            # Unique name for logs and metrics
    interface: can0            # SocketCAN channel
    bitrate: 500000            # Optional (default 500k)
    fd: false                  # Optional CAN-FD flag
    dbc_file: "../schemas/motors.dbc"  # Relative paths resolved against the YAML file
```

Additional optional keys:

* `dbitrate`: Data bitrate when using CAN-FD
* `filters`: Acceptance filter dictionaries supported by python-can
* Arbitrary extra keys are preserved in `BusConfig.metadata`

### 2.2 Transmit bindings (`tx_topics`)

`tx_topics` maps an identifier (for ROS it is the topic name) to a DBC
message. The YAML snippet below creates a binding from a ROS topic to the
`RS02_Command` DBC message.

```yaml
  tx_topics:
    "/td/rs02/command_velocity":
      dbc_message: "RS02_Command"
      type: "std_msgs/msg/Float32"   # Optional ROS metadata
      qos: "command"                  # Optional ROS QoS preset name
      fields:
        data: target_velocity_rads    # ROS field -> DBC signal
```

* `fields` (optional) maps client-side field names to DBC signals. If omitted,
  the payload is treated as a dictionary keyed by DBC signal names.
* Any additional key/value pairs become part of `TxBindingConfig.metadata` and
  are ignored by the base service.

### 2.3 Receive bindings (`rx_frames`)

Each entry describes how to publish decoded CAN frames. The key is an
identifier chosen by the author. Use `dbc_message` when you want to publish the
same CAN frame multiple times with different projections.

```yaml
  rx_frames:
    "RS02_Status1__velocity":
      dbc_message: "RS02_Status1"     # Optional, defaults to the entry key
      topic: "/td/rs02/velocity_rad_s"
      type: "std_msgs/msg/Float32"
      fields:
        mech_velocity_rads: data      # DBC signal -> client field
```

* `fields` maps DBC signal names to client-side field names (for ROS this is
  usually the message attribute).
* Any extra values are stored in `RxBindingConfig.metadata` and ignored by the
  base service.

## 3. Python service API

```python
from td_can_bridges import load_bridge_config, CanBusService

cfg = load_bridge_config("config/example_singlebus.yaml")
bus_cfg = cfg.get_bus("motor_bus")
service = CanBusService(bus_cfg)

# Register bindings
for binding in bus_cfg.tx_bindings.values():
    service.register_tx_binding(binding)

for binding in bus_cfg.rx_bindings.values():
    def handler(payload, binding=binding):
        print(f"Received {binding.message}: {payload}")
    service.register_rx_binding(binding, handler)

service.start()
service.send("/td/rs02/command_velocity", {"data": 12.5})
```

Key methods:

* `CanBusService.register_tx_binding(binding)` – enable a transmit mapping.
* `CanBusService.send(binding_key, payload)` – encode and send a CAN frame
  using aliases defined in the YAML `fields` mapping.
* `CanBusService.register_rx_binding(binding, handler)` – register a callback.
  The handler receives a dictionary keyed by the aliases defined in the YAML
  `fields` mapping.
* `CanBusService.start()` / `shutdown()` – manage the background RX loop.

The service uses the standard `logging` module (`td_can_bridges.service`). Set
`logging.basicConfig(level=logging.INFO)` in your application to see runtime
messages.

## 4. Working with ROS

The ROS bridge (`td_can_bridges.bridge_node`) now delegates almost all runtime
work to `CanBusService`. When the node starts it:

1. Calls `load_bridge_config` to parse the YAML file.
2. Instantiates a `BusWorker` per bus, which in turn:
   * Registers all TX/RX bindings with the service.
   * Creates ROS publishers/subscribers using metadata from the YAML file.
   * Starts the CAN RX loop.

The previous YAML format remains valid. Existing ROS deployments should keep
working while non-ROS clients can share the same config file.

## 5. Adding a new device

1. Extend the appropriate DBC file with the new message/signals.
2. Update the YAML config:
   * Add `tx_topics` entries for commands the device should receive.
   * Add `rx_frames` entries for telemetry the device publishes. Use
     `dbc_message` when mapping the same frame to multiple ROS topics or custom
     payloads.
3. (ROS) Launch `td_can_bridge` with the updated YAML.
4. (Non-ROS) Import `CanBusService` and reuse the same YAML.

Following these steps keeps the CAN service definitions in one place and makes
it straightforward for other teams to integrate without touching ROS code.
