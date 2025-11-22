This documentation covers the hardware, firmware architecture, and host-side integration for the **ESP32-S3 Native GS\_USB CAN Adapter (v32)**.

-----

# ESP32-S3 Native GS\_USB CAN Adapter

## 1\. Overview

This device functions as a high-speed, bi-directional USB-to-CAN bridge. It utilizes the **ESP32-S3's native USB Serial/JTAG controller** (acting as a USB OTG device) and the internal **TWAI (Two-Wire Automotive Interface)** controller.

It implements the **`gs_usb` protocol**, which is natively supported by the Linux kernel. This means no proprietary drivers are required on the robot computer; it appears as a standard network interface (`can0`).

### Key Specifications

  * **Protocol:** `gs_usb` (compatible with SocketCAN).
  * **Max Bitrate:** 1 Mbit/s.
  * **USB Speed:** USB 2.0 Full Speed (12 Mbit/s).
  * \*\* buffering:\*\* \* **RX (Device-\>Host):** 128-frame Deep Queue + 4KB USB FIFO.
      * **TX (Host-\>Device):** Direct ISR forwarding.
  * **Features:** Hardware Timestamping (Pass-through), Bus Error Reporting.

-----

## 2\. Hardware Setup

### Wiring Diagram

The ESP32-S3 requires an external CAN Transceiver (3.3V logic) to interface with the physical bus.

| Component | Pin Name | ESP32-S3 GPIO | Note |
| :--- | :--- | :--- | :--- |
| **USB Host** | D- | GPIO 19 | Native USB D- |
| **USB Host** | D+ | GPIO 20 | Native USB D+ |
| **Transceiver** | TX | **GPIO 4** | Transmits to Bus |
| **Transceiver** | RX | **GPIO 5** | Receives from Bus |
| **Bus** | CAN\_H / CAN\_L | N/A | **Requires 120Ω termination** at ends of the bus |

> **Critical Note:** Ensure the CAN Bus is terminated with a 120 Ohm resistor between CAN\_H and CAN\_L at the furthest ends of the bus. Lack of termination is the \#1 cause of "Bus Off" errors.

-----

## 3\. Firmware Architecture (v32 Stable)

The firmware solves the "FreeRTOS vs. TinyUSB" concurrency race condition by splitting duties into three distinct tasks.

### A. The "Three-Task" Model

1.  **`usb_manager_task` (Priority 5 - High):**

      * **Role:** Solely responsible for calling `tud_task()`.
      * **Reason:** Keeps the USB Heartbeat alive. If this stops, the Linux host disconnects the device.
      * **Stats:** Calculates and prints RX/TX packets-per-second to UART every 1s.

2.  **`can_forward_task` (Priority 4 - Medium):**

      * **Role:** Drains the internal FreeRTOS Queue and pushes data into the USB FIFO (`tud_vendor_write`).
      * **Flow Control:** Checks `tud_vendor_write_available()` to prevent buffer overflows.

3.  **`can_rx_task` (Priority 4 - Medium):**

      * **Role:** Listens to the CAN Bus (TWAI driver).
      * **Action:** When a frame arrives, it wraps it in a `gs_host_frame` struct and pushes it to the `can_to_usb_queue`.

### B. Data Flow

  * **RX (Robot \<- Motor):** Motor -\> PHY -\> TWAI ISR -\> `can_rx_task` -\> `Queue` -\> `can_forward_task` -\> USB FIFO -\> Linux.
  * **TX (Robot -\> Motor):** Linux -\> USB ISR -\> `tud_vendor_rx_cb` -\> TWAI Driver -\> PHY -\> Motor.

-----

## 4\. Host Integration (Linux/Robot)

Since the firmware mimics a standard device, setup is handled via `iproute2`.

### 1\. Bring Up Interface

Run the following on the robot's Linux computer:

```bash
# Load kernel module (usually loaded by default)
sudo modprobe gs_usb

# Configure the interface (Set bitrate to match your motors, e.g., 1M)
sudo ip link set can0 up type can bitrate 1000000

# Verify status
ip -s -d link show can0
```

### 2\. Testing Communication

Use `can-utils` to verify connectivity.

```bash
sudo apt install can-utils

# Sniff Traffic (View data coming from motors)
candump can0

# Send Test Packet (Standard ID 0x123, Data DE AD BE EF)
cansend can0 123#DEADBEEF
```

-----

## 5\. Robot Code Integration

Because the device exposes itself as a native network interface (`can0`), you do **not** use Serial/UART libraries. You use **SocketCAN**.

### Python Example (`python-can`)

```bash
pip install python-can
```

```python
import can

# Initialize the bus
bus = can.interface.Bus(channel='can0', bustype='socketcan')

# Sending
msg = can.Message(arbitration_id=0x123, data=[0, 1, 2, 3, 4, 5, 6, 7], is_extended_id=False)
bus.send(msg)

# Receiving
while True:
    message = bus.recv()
    print(f"ID: {hex(message.arbitration_id)} Data: {message.data}")
```

### C++ Example (SocketCAN)

```cpp
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>

int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
struct ifreq ifr;
strcpy(ifr.ifr_name, "can0");
ioctl(s, SIOCGIFINDEX, &ifr);

struct sockaddr_can addr;
addr.can_family = AF_CAN;
addr.can_ifindex = ifr.ifr_ifindex;
bind(s, (struct sockaddr *)&addr, sizeof(addr));

// Now use read() and write() on socket 's'
```

-----

## 6\. Troubleshooting

| Symptom | Diagnosis | Fix |
| :--- | :--- | :--- |
| **`candump` is empty** | Host is ready, but ESP32 isn't sending. | Check UART logs on ESP32 (`idf.py monitor`). If `RX` stats are increasing, the queue is stuck. Restart `can0` interface. |
| **"Bus Off" Error** | Physical layer failure. | Check 120Ω termination resistors. Check TX/RX pin swap (GPIO 4/5). |
| **Device not found (`lsusb`)** | USB enumeration failed. | Check D+/D- wiring. Ensure `usb_manager_task` is running. |
| **Lag / Latency** | Buffer bloat. | The firmware uses a deep 128-frame queue. This absorbs bursts but adds latency. If latency is critical, reduce `can_to_usb_queue` size in `app_main`. |
| **`ip link set up` hangs** | Driver install failure. | Firmware v32 fixes this. Ensure you are not using `ESP_INTR_FLAG_IRAM` in the firmware config. |

-----

## 7\. Firmware Build Instructions

To re-flash or modify the adapter:

1.  Install **ESP-IDF v5.x**.
2.  Navigate to project directory.
3.  Connect ESP32-S3 via USB (UART port for flashing, Native port for CAN).
4.  Run:
    ```bash
    idf.py set-target esp32s3
    idf.py build
    idf.py -p /dev/ttyUSB0 flash monitor
    ```
