# RoboStride Actuator Family

The RoboStride RS0x articulated actuators share a common electrical harness and CAN
communication stack. Each subfolder captures the model-specific specifications while this
overview highlights behaviors that are common across the RS02/RS03/RS04 family.

## Shared Traits
- **Physical layer:** CAN 2.0B at 1 Mbps. Extended frames are used for the private protocol;
  standard frames are used for MIT mode.
- **Connector set:** Dedicated power/CAN harness with separate download/diagnostic port and
  indicator LEDs (see individual manuals for exact pinouts).
- **Upper-computer tooling:** RoboStride provides a Windows configuration utility that speaks
  through a USB–CAN adapter (CH340-based). DIP switch 1 controls boot mode, DIP switch 2 adds
  the 120 Ω termination resistor on the adapter.
- **Protocol switching:** Motors boot into the private extended-frame protocol. Command 8
  switches between Private, CANopen, and MIT modes and takes effect after a power cycle.
- **Persistence:** Parameters written with Type 18 are volatile until saved with Type 22.

Refer to each device dossier for mechanical ratings and the command surface that should be
exposed to the TritonCAN API.
