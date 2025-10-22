#!/usr/bin/env python3
"""Interactive virtual CAN blinker endpoint."""

from __future__ import annotations

import argparse
import logging
import signal
import sys
from pathlib import Path
from typing import Any, Dict

from td_can_bridges.service import (
    CanBusService,
    RxBindingConfig,
    TxBindingConfig,
    load_bridge_config,
)

DEVICE_ROLES: Dict[str, Dict[str, str]] = {
    "device_a": {
        "tx": "device_a_command",
        "rx": "device_a_inbox",
        "label": "Device A",
        "peer": "Device B",
    },
    "device_b": {
        "tx": "device_b_command",
        "rx": "device_b_inbox",
        "label": "Device B",
        "peer": "Device A",
    },
}


def _install_signal_handlers(service: CanBusService) -> None:
    def _handle(signum, _frame):
        logging.getLogger(__name__).info("Received signal %s, shutting down", signum)
        service.shutdown()
        sys.exit(0)

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _handle)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Virtual CAN blinker endpoint shell")
    parser.add_argument(
        "--config",
        default=Path(__file__).resolve().parents[1] / "config" / "vcan_blink_demo.yaml",
        type=Path,
        help="Path to the YAML config describing the virtual blink bus.",
    )
    parser.add_argument(
        "--bus",
        default="vcan_demo",
        help="Bus name from the YAML file to attach to.",
    )
    parser.add_argument(
        "--device",
        choices=sorted(DEVICE_ROLES),
        required=True,
        help="Which virtual device role to emulate.",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        help="Logging level (DEBUG, INFO, WARNING, ...).",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    log = logging.getLogger("vcan_blink_device")

    cfg = load_bridge_config(args.config)
    bus_cfg = cfg.get_bus(args.bus)
    if bus_cfg is None:
        parser.error(f"Bus '{args.bus}' not found in {args.config}")

    role = DEVICE_ROLES[args.device]

    service = CanBusService(bus_cfg)

    tx_binding: TxBindingConfig | None = bus_cfg.tx_bindings.get(role["tx"])
    if tx_binding is None:
        parser.error(f"TX binding '{role['tx']}' missing from config {args.config}")
    service.register_tx_binding(tx_binding)

    rx_binding: RxBindingConfig | None = bus_cfg.rx_bindings.get(role["rx"])
    if rx_binding is None:
        parser.error(f"RX binding '{role['rx']}' missing from config {args.config}")

    def _handle_rx(payload: Dict[str, Any], binding: RxBindingConfig) -> None:
        raw_blink = payload.get("blink", 0)
        raw_seq = payload.get("seq", 0)
        try:
            blink_value = int(raw_blink)
        except (TypeError, ValueError):
            blink_value = 0
        try:
            seq_value = int(raw_seq)
        except (TypeError, ValueError):
            seq_value = 0
        state_text = "ON" if blink_value else "OFF"
        print(
            f"\n[{role['label']}] Received from {role['peer']}: "
            f"state={state_text} ({blink_value}), seq={seq_value}"
        )
        print("blink> ", end="", flush=True)

    service.register_rx_binding(rx_binding, _handle_rx)
    service.start()
    _install_signal_handlers(service)

    print(f"{role['label']} connected to bus '{bus_cfg.name}' on interface '{bus_cfg.interface}'.")
    print(f"Type commands to send frames to {role['peer']} (blink on/off/<value>, toggle, quit).")

    sequence = 0
    blink_state = 0
    prompt = "blink> "
    try:
        while True:
            try:
                line = input(prompt)
            except EOFError:
                line = "quit"
            line = line.strip().lower()
            if not line:
                continue
            if line in {"quit", "exit"}:
                break
            if line == "toggle":
                blink_state = 0 if blink_state else 1
            elif line in {"on", "blink on"}:
                blink_state = 1
            elif line in {"off", "blink off"}:
                blink_state = 0
            elif line.startswith("blink "):
                value = line.split(maxsplit=1)[1]
                try:
                    blink_state = int(value, 0)
                except ValueError:
                    print("Invalid value. Use an integer between 0 and 255.")
                    continue
                if not 0 <= blink_state <= 255:
                    print("Value out of range. Use 0-255.")
                    continue
            else:
                print("Commands: on, off, blink <0-255>, toggle, quit")
                continue

            sequence = (sequence + 1) % 256
            payload = {"blink": blink_state, "seq": sequence}
            log.debug("Sending %s with payload %s", role["tx"], payload)
            try:
                service.send(role["tx"], payload)
            except Exception as exc:  # pragma: no cover - depends on runtime
                log.error("Failed to send frame: %s", exc)
    finally:
        service.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
