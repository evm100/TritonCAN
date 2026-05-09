#!/usr/bin/env python3
"""
RoboStride CAN bus scanner.

Sends a Type 0 (Get Device ID) frame to every motor id 0-127 and prints
which ones reply, along with the 64-bit MCU UID embedded in the response.

Usage: sudo python3 scan_ids.py [interface] [host_id]
   e.g. sudo python3 scan_ids.py can0
"""

import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robstride import (
    RobstrideBus, Motor, CommunicationType,
    _CAN_FRAME_FMT, _CAN_FRAME_SIZE, _CAN_EFF_FLAG,
)


SCAN_RANGE = range(0, 128)
PER_ID_TIMEOUT = 0.05    # seconds to wait for a reply after each probe
DRAIN_AFTER    = 0.20    # final drain window to catch stragglers


def scan(interface: str = "can0", host_id: int = 0xFD) -> None:
    bus = RobstrideBus(interface, motors={}, host_id=host_id)
    bus.connect(handshake=False)
    sock = bus._sock
    sock.setblocking(False)

    found: dict[int, bytes] = {}

    def drain(deadline: float) -> None:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            try:
                raw = sock.recv(_CAN_FRAME_SIZE)
            except BlockingIOError:
                time.sleep(0.001)
                continue
            except OSError:
                return
            cid, _dlc, data = struct.unpack(_CAN_FRAME_FMT, raw)
            cid &= 0x1FFFFFFF
            mode = (cid >> 24) & 0x1F
            # Type 0 reply: the responding motor's id is in the ID's `data`
            # field low byte; the 8-byte payload carries its MCU UID.
            if mode == 0:
                replier = (cid >> 8) & 0xFF
                if replier not in found:
                    found[replier] = bytes(data)

    print(f"🔎 Scanning ids 0-127 on {interface} (host_id=0x{host_id:02X})...")
    t0 = time.monotonic()
    for mid in SCAN_RANGE:
        cid = RobstrideBus._build_id(CommunicationType.GET_DEVICE_ID, host_id, mid)
        frame = struct.pack(_CAN_FRAME_FMT, cid | _CAN_EFF_FLAG, 8, b"\x00" * 8)
        try:
            sock.send(frame)
        except OSError as e:
            print(f"⚠️  send to id {mid} failed: {e}")
            continue
        drain(time.monotonic() + PER_ID_TIMEOUT)
        if mid % 16 == 15:
            sys.stdout.write(f"  ...probed {mid + 1}/128\r")
            sys.stdout.flush()

    drain(time.monotonic() + DRAIN_AFTER)
    bus.disconnect()
    elapsed = time.monotonic() - t0

    print(" " * 40, end="\r")
    print(f"✅ Scan complete in {elapsed:.2f}s. {len(found)} motor(s) responded.\n")
    if not found:
        print("No replies. Check wiring, termination, baud rate (1 Mbps), and power.")
        return

    print(f"{'ID (dec)':<10} {'ID (hex)':<10} MCU UID (8 bytes)")
    print("-" * 50)
    for mid in sorted(found):
        uid = found[mid].hex(" ")
        print(f"{mid:<10} 0x{mid:02X}       {uid}")


def main() -> None:
    iface = sys.argv[1] if len(sys.argv) > 1 else "can0"
    host = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0xFD
    try:
        scan(iface, host)
    except KeyboardInterrupt:
        print("\n🛑 Aborted.")


if __name__ == "__main__":
    main()
