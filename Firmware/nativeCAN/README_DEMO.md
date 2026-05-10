# RoboStride RS02 Motor Demo

This demo script (`motor_demo.py`) allows you to test the USB-CAN tool with a RoboStride RS02 motor using the MIT protocol.

## Prerequisites

1.  **Hardware Setup**:
    *   Connect the USB-CAN ESP32S3 device to your computer.
    *   Connect the RoboStride RS02 motor to the CAN bus (CAN_H, CAN_L).
    *   Ensure the bus is terminated with 120Ω resistors.
    *   Power the motor (usually 24V or 48V).

2.  **Software Setup**:
    *   Ensure the `can0` interface is up and running at 1 Mbps.
        ```bash
        sudo ip link set can0 up type can bitrate 1000000
        ```
    *   Install `python-can`:
        ```bash
        pip install python-can
        ```

## Running the Demo

Run the script with Python:

```bash
python3 motor_demo.py
```

## What to Expect

1.  The script will attempt to **Enable** the motor (Motor ID 1).
2.  If successful, the motor should stiffen slightly.
3.  The motor will then spin at approximately **2 rad/s** for **5 seconds**.
4.  The motor will stop and **Disable**.

## Troubleshooting

*   **Motor doesn't move**:
    *   Check if the Motor ID is correct (default is 1). You can change `MOTOR_ID` in the script.
    *   Check if the motor is in MIT Mode. If it's in CANopen mode, you may need to switch it using the manufacturer's tool or a specific command sequence.
    *   **Protocol Mismatch**: Some documentation suggests RS02 might use Extended CAN IDs (29-bit). If Standard IDs don't work, try changing `is_extended_id=True` in `motor_demo.py`.
    *   Verify CAN connections and termination.
    *   Use `candump can0` to see if any messages are being sent/received.

*   **"Network is down" error**:
    *   Run the `ip link set ...` command above.

*   **"No buffer space available"**:
    *   This can happen if the USB-CAN device is overwhelmed. Try increasing the queue size in the firmware or reducing the update rate in the script.
