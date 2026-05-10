"""Reusable CAN service layer.

This module exposes a Python API that can be used by both ROS and non-ROS
clients to describe CAN devices via YAML/DBC files, open SocketCAN
interfaces, and register handlers for transmission and reception of frames.

The goal is to keep all ROS-specific logic out of this module so that other
teams can build on the same infrastructure without pulling in rclpy.
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Mapping, MutableMapping, Optional

import can
import cantools
import yaml


LOG = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Configuration dataclasses


@dataclass(frozen=True)
class TxBindingConfig:
    """Configuration for a transmitted DBC message.

    ``fields`` maps user-friendly keys (e.g. ROS message field names or plain
    dictionary keys) to DBC signal names. Additional metadata from the YAML
    file is stored in ``metadata`` and left uninterpreted so higher layers can
    attach their own meaning (topic name, QoS profile, etc.).
    """

    key: str
    message: str
    fields: Mapping[str, str] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RxBindingConfig:
    """Configuration for a received DBC message.

    ``fields`` maps DBC signal names to user-friendly keys. As with
    :class:`TxBindingConfig`, any additional metadata is retained for the
    consumer.
    """

    key: str
    message: str
    fields: Mapping[str, str] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class BusConfig:
    """Description of a single SocketCAN interface."""

    name: str
    interface: str
    dbc_file: Path
    bitrate: int = 500_000
    fd: bool = False
    dbitrate: Optional[int] = None
    filters: Optional[Iterable[MutableMapping[str, int]]] = None
    tx_bindings: Mapping[str, TxBindingConfig] = field(default_factory=dict)
    rx_bindings: Mapping[str, RxBindingConfig] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class BridgeConfig:
    """Full YAML configuration describing one or more CAN buses."""

    path: Path
    buses: List[BusConfig]
    logging: Mapping[str, Any] = field(default_factory=dict)
    qos: Mapping[str, Any] = field(default_factory=dict)

    def get_bus(self, name: str) -> Optional[BusConfig]:
        for bus in self.buses:
            if bus.name == name:
                return bus
        return None


def _require_keys(data: Mapping[str, Any], required: Iterable[str], context: str) -> None:
    missing = [key for key in required if key not in data]
    if missing:
        raise ValueError(f"Missing required key(s) {missing} in {context}")


def load_bridge_config(path: Path | str) -> BridgeConfig:
    """Parse a YAML configuration file and return a :class:`BridgeConfig`.

    Relative DBC paths are resolved relative to the YAML file location.
    """

    cfg_path = Path(path).expanduser().resolve()
    if not cfg_path.exists():
        raise FileNotFoundError(cfg_path)

    with cfg_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}

    buses_cfg: List[BusConfig] = []
    for idx, bus_entry in enumerate(raw.get("buses", [])):
        context = f"buses[{idx}]"
        _require_keys(bus_entry, ["name", "interface", "dbc_file"], context)

        dbc_path = Path(bus_entry["dbc_file"])
        if not dbc_path.is_absolute():
            dbc_path = (cfg_path.parent / dbc_path).resolve()

        tx_bindings: Dict[str, TxBindingConfig] = {}
        for key, spec in (bus_entry.get("tx_topics") or {}).items():
            _require_keys(spec, ["dbc_message"], f"{context}.tx_topics['{key}']")
            fields = spec.get("fields", {}) or {}
            metadata = {k: v for k, v in spec.items() if k not in {"dbc_message", "fields"}}
            tx_bindings[key] = TxBindingConfig(
                key=key,
                message=spec["dbc_message"],
                fields=fields,
                metadata=metadata,
            )

        rx_bindings: Dict[str, RxBindingConfig] = {}
        for key, spec in (bus_entry.get("rx_frames") or {}).items():
            message_name = spec.get("dbc_message", key)
            metadata = {k: v for k, v in spec.items() if k not in {"fields", "dbc_message"}}
            fields = spec.get("fields", {}) or {}
            rx_bindings[key] = RxBindingConfig(
                key=key,
                message=message_name,
                fields=fields,
                metadata=metadata,
            )

        metadata = {k: v for k, v in bus_entry.items() if k not in {
            "name",
            "interface",
            "dbc_file",
            "bitrate",
            "fd",
            "dbitrate",
            "filters",
            "tx_topics",
            "rx_frames",
        }}

        buses_cfg.append(
            BusConfig(
                name=bus_entry["name"],
                interface=bus_entry["interface"],
                dbc_file=dbc_path,
                bitrate=bus_entry.get("bitrate", 500_000),
                fd=bus_entry.get("fd", False),
                dbitrate=bus_entry.get("dbitrate"),
                filters=bus_entry.get("filters"),
                tx_bindings=tx_bindings,
                rx_bindings=rx_bindings,
                metadata=metadata,
            )
        )

    return BridgeConfig(
        path=cfg_path,
        buses=buses_cfg,
        logging=raw.get("logging", {}),
        qos=raw.get("qos", {}),
    )


# ---------------------------------------------------------------------------
# Runtime service implementation


class FrameEncoder:
    """Encode named payloads to CAN frames using a DBC."""

    def __init__(self, dbc, binding: TxBindingConfig):
        self.binding = binding
        self.msg_def = dbc.get_message_by_name(binding.message)
        self.alias_to_signal = dict(binding.fields)

    def encode(self, payload: Mapping[str, Any]) -> can.Message:
        if not isinstance(payload, Mapping):
            raise TypeError("payload must be a mapping of field -> value")

        values: Dict[str, Any] = {}
        if self.alias_to_signal:
            for alias, signal in self.alias_to_signal.items():
                if alias not in payload:
                    raise KeyError(f"Missing field '{alias}' for binding '{self.binding.key}'")
                values[signal] = payload[alias]
        else:
            values = dict(payload)

        data = self.msg_def.encode(values)
        return can.Message(
            arbitration_id=self.msg_def.frame_id,
            data=data,
            is_extended_id=self.msg_def.is_extended_frame,
        )


class FrameDecoder:
    """Decode CAN frames to user-friendly dictionaries."""

    def __init__(self, dbc, binding: RxBindingConfig):
        self.binding = binding
        self.msg_def = dbc.get_message_by_name(binding.message)
        self.signal_to_alias = dict(binding.fields)

    @property
    def frame_id(self) -> int:
        return self.msg_def.frame_id

    def decode(self, raw_bytes: bytes) -> Dict[str, Any]:
        decoded = self.msg_def.decode(raw_bytes)
        if self.signal_to_alias:
            return {alias: decoded.get(signal) for signal, alias in self.signal_to_alias.items()}
        return dict(decoded)


RxHandler = Callable[[Dict[str, Any], RxBindingConfig], None]


class CanBusService:
    """Manage a SocketCAN interface backed by a DBC file."""

    def __init__(self, cfg: BusConfig):
        self.cfg = cfg
        self.dbc = cantools.database.load_file(str(cfg.dbc_file))
        self.bus = self._open_bus(cfg)
        self._tx_bindings: Dict[str, FrameEncoder] = {}
        self._rx_bindings: Dict[int, tuple[FrameDecoder, RxBindingConfig, RxHandler]] = {}
        self._stop = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None

        if cfg.filters:
            try:
                self.bus.set_filters(list(cfg.filters))
            except Exception:  # pragma: no cover - depends on driver support
                LOG.warning("[%s] failed to apply CAN filters", cfg.name, exc_info=True)

    # ------------------------------------------------------------------
    # Configuration helpers

    def register_tx_binding(self, binding: TxBindingConfig) -> None:
        LOG.debug("[%s] register TX binding %s -> %s", self.cfg.name, binding.key, binding.message)
        self._tx_bindings[binding.key] = FrameEncoder(self.dbc, binding)

    def register_rx_binding(self, binding: RxBindingConfig, handler: RxHandler) -> None:
        decoder = FrameDecoder(self.dbc, binding)
        LOG.debug(
            "[%s] register RX binding %s (0x%X)",
            self.cfg.name,
            binding.message,
            decoder.frame_id,
        )
        self._rx_bindings[decoder.frame_id] = (decoder, binding, handler)

    # ------------------------------------------------------------------
    # Runtime operations

    def start(self) -> None:
        if self._rx_thread and self._rx_thread.is_alive():
            return
        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, name=f"{self.cfg.name}-rx", daemon=True)
        self._rx_thread.start()

    def shutdown(self) -> None:
        self._stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        try:
            self.bus.shutdown()
        except Exception:  # pragma: no cover - depends on driver support
            LOG.debug("[%s] error during bus shutdown", self.cfg.name, exc_info=True)

    def send(self, key: str, payload: Mapping[str, Any]) -> None:
        if key not in self._tx_bindings:
            raise KeyError(f"Unknown TX binding '{key}'")
        frame = self._tx_bindings[key].encode(payload)
        LOG.debug(
            "[%s] TX 0x%X (%s) %s",
            self.cfg.name,
            frame.arbitration_id,
            self._tx_bindings[key].msg_def.name,
            payload,
        )
        self.bus.send(frame)

    # ------------------------------------------------------------------
    # Internal helpers

    @staticmethod
    def _open_bus(cfg: BusConfig) -> can.BusABC:
        kwargs = dict(interface="socketcan", channel=cfg.interface, bitrate=cfg.bitrate, fd=cfg.fd)
        if cfg.fd and cfg.dbitrate:
            kwargs["data_bitrate"] = cfg.dbitrate
        return can.Bus(**kwargs)

    def _rx_loop(self) -> None:
        reader = can.BufferedReader()
        notifier = can.Notifier(self.bus, [reader], timeout=0.01)
        LOG.info("[%s] RX loop started", self.cfg.name)
        try:
            while not self._stop.is_set():
                msg = reader.get_message(timeout=0.1)
                if msg is None:
                    continue
                binding_entry = self._rx_bindings.get(msg.arbitration_id)
                if not binding_entry:
                    continue
                decoder, binding, handler = binding_entry
                try:
                    payload = decoder.decode(bytes(msg.data))
                    LOG.debug(
                        "[%s] RX 0x%X (%s) %s",
                        self.cfg.name,
                        msg.arbitration_id,
                        decoder.msg_def.name,
                        payload,
                    )
                    handler(payload, binding)
                except Exception:
                    LOG.exception("[%s] Failed to decode frame 0x%X", self.cfg.name, msg.arbitration_id)
        finally:
            notifier.stop()
            LOG.info("[%s] RX loop stopped", self.cfg.name)


__all__ = [
    "BridgeConfig",
    "BusConfig",
    "CanBusService",
    "RxBindingConfig",
    "TxBindingConfig",
    "load_bridge_config",
]

