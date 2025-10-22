# TritonCAN Service Layer API

This document describes the stable YAML/DBC driven interface for the CAN
service and the Python API that powers both ROS and non-ROS integrations. The
intent is to make it easy for any team to add devices without touching raw CAN
frames or knowing the internals of `rclpy` or the ROS 2 launch system.

> **Key principle:** The CAN API is a standalone application focused solely on
> TritonCAN's CAN protocol. It is **not** a ROS 2 node. Any ROS 2 integration is
> implemented in a separate ROS 2 → CAN API adapter that consumes this service
> layer. That adapter is responsible for mapping ROS publishers, subscribers,
> services, and actions to the device protocol defined in our docs and the
> future device registry. Keeping the boundaries strict ensures the CAN API
> remains the single, well-documented entry point for interacting with devices
> without touching raw CAN frames.

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
* **ROS bridge** – a separate ROS 2 application that maps publishers,
  subscribers, services, and actions to the CAN service using
  `BusWorker`, `TopicTxBinding`, and `RxBinding`.

Your project can either consume the YAML definitions through the ROS bridge or
instantiate `CanBusService` directly to hook up any other application. The
device registry described below produces the same YAML artifacts for both
consumers, so embedded engineers update one description and every client stays
in sync.

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

The ROS bridge (`td_can_bridges.bridge_node`) is a **consumer** of the CAN API,
not part of it. The bridge is responsible for translating ROS concepts—topics,
services, and actions—into the TritonCAN device protocol exposed by this
service. It loads the same YAML/DBC definitions as any other client and wires
them up to ROS primitives at runtime.

## 5. Virtual blink demo quickstart

For a hands-on introduction without hardware, the repository ships with a
two-node virtual blinker:

1. Create a SocketCAN interface: `sudo modprobe vcan && sudo ip link add dev
   vcan0 type vcan && sudo ip link set up vcan0`.
2. Launch `td_CAN/scripts/vcan_blink_launch.sh`. The helper script opens a tmux
   session containing two terminals, one per virtual device.
3. In either pane type `blink on`, `blink off`, `blink <value>`, or `toggle` to
   send frames. The opposite pane prints the decoded payload via the CAN
   service daemon.
4. Exit with `quit` or detach from tmux (`Ctrl+B`, then `D`).

The demo uses `td_CAN/config/vcan_blink_demo.yaml` and
`td_CAN/td_can_bridges/schemas/vcan_blink.dbc`, so it doubles as a reference for
authoring new device definitions.

When the ROS bridge launches it:

1. Calls `load_bridge_config` to parse the YAML file.
2. Instantiates a `BusWorker` per bus, which in turn:
   * Registers all TX/RX bindings with the service.
   * Creates ROS publishers, subscribers, services, and actions using metadata
     from the YAML file or its own bridge-specific configuration.
   * Starts the CAN RX loop.

The previous YAML format remains valid. Existing ROS deployments should keep
working while non-ROS clients can share the same config file.

## 6. Device registry and auto-generated APIs

The long-term contract for TritonCAN is a **device registry** that captures the
full set of messages, telemetry fields, and callable functions for each device
type. Every consumer—CLI tools, embedded diagnostics, the future ROS bridge—can
generate their bindings from that registry instead of hand-editing YAML. The
registry has three layers:

1. **Authoritative device docs** (Markdown) describing the hardware behavior in
   plain language.
2. **Structured manifests** (YAML) checked into `docs/device_registry/` that
   reference DBC frames, list commands, and define friendly API names.
3. **Codegen** that turns the manifests into TritonCAN configs for
   `CanBusService` and any adapters.

### 6.1 Authoring a new device type

1. Extend the DBC file with all frames and signals the device emits or
   receives.
2. Create or update the device's Markdown documentation under `docs/devices/`.
   Capture:
   * A plain-language overview of the device's purpose and operating modes.
   * Tables for telemetry signals (name, units, DBC signal reference).
   * Tables for commands, including the default values and behavior notes.

### 6.2 Create the manifest

Add a new YAML manifest (e.g. `docs/device_registry/rs02.yaml`) with three
sections:

```yaml
device: RS02
dbc_file: ../td_CAN/schemas/rs02.dbc

commands:
  set_velocity:
    dbc_message: RS02_Command
    description: Set wheel velocity in rad/s.
    fields:
      target_velocity_rads: velocity

telemetry:
  velocity:
    dbc_message: RS02_Status1
    fields:
      mech_velocity_rads: velocity
```

Key ideas:

* `commands` expose **plain-name API calls** (e.g. `set_velocity`) that are easy
  to invoke from Python or, later, ROS services/actions. Each entry maps the
  friendly name to a DBC message and the signal aliases to expose.
* `telemetry` lists the outputs the service should publish, again keyed by a
  friendly name. Additional metadata (units, scaling, safety constraints) can be
  embedded here for tooling.

### 6.3 Generate service configs

A generator script (added in a future change) will translate the manifest into
`BridgeConfig` YAML by:

1. Expanding every `commands` entry into a `tx_topics` binding whose key is the
   plain-language command name.
2. Emitting `rx_frames` entries for each telemetry block and attaching any
   metadata needed by downstream clients.
3. Producing adapter-specific extras (e.g. ROS message types) from optional
   fields in the manifest.

The output YAML feeds directly into `load_bridge_config`. That keeps the CAN API
focused on protocol translation, while embedded engineers stay productive by
authoring manifests instead of glue code.

### 6.4 Deploying the new device

1. Run the generator to refresh the TritonCAN YAML.
2. (ROS) Launch the separate ROS 2 → CAN API bridge with the regenerated YAML
   and implement any ROS-specific logic (services/actions) in that adapter.
3. (Non-ROS) Import `CanBusService` directly and call the friendly command names
   exposed by the generator.

Following this flow keeps the CAN service definitions in one place, gives new
teammates a repeatable playbook, and paves the way for the dedicated ROS bridge
to consume the exact same device contracts.
