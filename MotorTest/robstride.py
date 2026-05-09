"""
Self-contained RoboStride RS-02 driver for Linux SocketCAN.

Pure-stdlib replacement for the `robstride_dynamics` SDK so motor_hub.py and
swing_test.py can run with nothing more than python3 and a configured CAN
interface (e.g. `sudo ip link set can0 up type can bitrate 1000000`).

Public surface mirrors the names the original scripts import:
    RobstrideBus, Motor, ParameterType, CommunicationType
plus `robstride.table` (also exposed as `robstride_dynamics.table` shim) with
the MODEL_MIT_*_TABLE dicts the scripts monkey-patch on import.
"""

from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple


# --- SocketCAN raw frame layout ----------------------------------------------
# struct can_frame { canid_t can_id; __u8 can_dlc; __u8 __pad; __u8 __res0;
#                    __u8 len8_dlc; __u8 data[8]; };  -> 16 bytes total
_CAN_FRAME_FMT = "=IB3x8s"
_CAN_FRAME_SIZE = struct.calcsize(_CAN_FRAME_FMT)
_CAN_EFF_FLAG = 0x80000000  # extended (29-bit) identifier flag


# --- RS-02 MIT scaling limits (manual §"Control mode instructions") ----------
P_MAX = 12.57   # 4 * pi rad
V_MAX = 44.0    # rad/s
T_MAX = 17.0    # Nm
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0


def _f2u(x: float, x_min: float, x_max: float, bits: int = 16) -> int:
    """Linearly map a float in [x_min, x_max] to an unsigned int of `bits`."""
    span = x_max - x_min
    if x > x_max:
        x = x_max
    elif x < x_min:
        x = x_min
    return int((x - x_min) * ((1 << bits) - 1) / span) & ((1 << bits) - 1)


def _u2f(u: int, x_min: float, x_max: float, bits: int = 16) -> float:
    """Inverse of _f2u."""
    span = x_max - x_min
    return u * span / ((1 << bits) - 1) + x_min


# --- Communication types (29-bit ID, bits [28:24]) ---------------------------
class CommunicationType:
    GET_DEVICE_ID         = 0
    OPERATION_CONTROL     = 1
    MOTOR_FEEDBACK        = 2
    ENABLE                = 3
    STOP                  = 4
    SET_ZERO_POSITION     = 6
    SET_CAN_ID            = 7
    READ_PARAMETER        = 17
    WRITE_PARAMETER       = 18
    FAULT_REPORT          = 21
    SAVE_PARAMETERS       = 22
    SET_BAUD_RATE         = 23
    ENABLE_ACTIVE_REPORT  = 24
    SET_PROTOCOL          = 25


# --- Parameter table ---------------------------------------------------------
# Each entry is (index, python_type, name, struct_fmt). Tuple-unpacks as
# (index, dtype, name) for the legacy callers in motor_hub.cmd_read_parameter
# / cmd_write_parameter. The struct fmt drives encode/decode.
class _ParamSpec(tuple):
    """3-tuple (index, dtype, name) with an extra `.fmt` attribute carrying
    the struct format used to encode/decode the parameter value."""

    def __new__(cls, index: int, dtype: type, name: str, fmt: str):
        obj = super().__new__(cls, (index, dtype, name))
        # tuple subclasses get a __dict__ by default unless __slots__ blocks it
        obj.fmt = fmt
        return obj


def _P(index: int, dtype: type, name: str, fmt: str) -> _ParamSpec:
    return _ParamSpec(index, dtype, name, fmt)


class ParameterType:
    # Mode / control config
    MODE                    = _P(0x7005, int,   "run_mode",        "<B")
    IQ_REF                  = _P(0x7006, float, "iq_ref",          "<f")
    VELOCITY_TARGET         = _P(0x700A, float, "spd_ref",         "<f")
    TORQUE_LIMIT            = _P(0x700B, float, "limit_torque",    "<f")
    CUR_KP                  = _P(0x7010, float, "cur_kp",          "<f")
    CUR_KI                  = _P(0x7011, float, "cur_ki",          "<f")
    CUR_FILT_GAIN           = _P(0x7014, float, "cur_filt_gain",   "<f")
    POSITION_TARGET         = _P(0x7016, float, "loc_ref",         "<f")
    SPEED_LIMIT             = _P(0x7017, float, "limit_spd",       "<f")
    CURRENT_LIMIT           = _P(0x7018, float, "limit_cur",       "<f")
    MECH_POSITION           = _P(0x7019, float, "mechPos",         "<f")
    IQF                     = _P(0x701A, float, "iqf",             "<f")
    MECH_VELOCITY           = _P(0x701B, float, "mechVel",         "<f")
    VBUS                    = _P(0x701C, float, "VBUS",            "<f")
    ROTATION                = _P(0x701D, int,   "rotation",        "<h")
    POSITION_KP             = _P(0x701E, float, "loc_kp",          "<f")
    SPEED_KP                = _P(0x701F, float, "spd_kp",          "<f")
    SPEED_KI                = _P(0x7020, float, "spd_ki",          "<f")
    SPEED_FILT_GAIN         = _P(0x7021, float, "spd_filt_gain",   "<f")
    VEL_ACCELERATION_TARGET = _P(0x7022, float, "acc_rad",         "<f")
    POS_VEL_MAX             = _P(0x7024, float, "vel_max",         "<f")
    POS_ACC_SET             = _P(0x7025, float, "acc_set",         "<f")
    EPSCAN_TIME             = _P(0x7026, int,   "EPScan_time",     "<H")
    ZERO_STA                = _P(0x2000, int,   "zero_sta",        "<B")
    ADD_OFFSET              = _P(0x2001, float, "add_offset",      "<f")
    DAMPER                  = _P(0x2002, int,   "damper",          "<B")


# --- Per-model MIT scaling tables (kept for monkey-patch compatibility) ------
class _Table:
    MODEL_MIT_POSITION_TABLE: Dict[str, float] = {"rs-02": P_MAX}
    MODEL_MIT_VELOCITY_TABLE: Dict[str, float] = {"rs-02": V_MAX}
    MODEL_MIT_TORQUE_TABLE:   Dict[str, float] = {"rs-02": T_MAX}
    MODEL_MIT_KP_TABLE:       Dict[str, float] = {"rs-02": KP_MAX}
    MODEL_MIT_KD_TABLE:       Dict[str, float] = {"rs-02": KD_MAX}


table = _Table()


# --- Motor descriptor --------------------------------------------------------
@dataclass
class Motor:
    id: int
    model: str = "rs-02"


# --- The bus -----------------------------------------------------------------
class RobstrideBus:
    """Linux SocketCAN-backed driver for one CAN interface and N motors."""

    def __init__(
        self,
        interface: str,
        motors: Dict[str, Motor],
        config: Optional[dict] = None,
        host_id: int = 0xFD,
        rx_timeout: float = 0.5,
    ) -> None:
        self.interface = interface
        self.motors = motors
        self.config = config or {}
        self.host_id = host_id
        self.rx_timeout = rx_timeout
        self._sock: Optional[socket.socket] = None

    # ---- Connection ----------------------------------------------------
    def connect(self, handshake: bool = True) -> None:
        if not hasattr(socket, "AF_CAN"):
            raise RuntimeError(
                "Python build lacks SocketCAN support (socket.AF_CAN). "
                "This driver requires Linux."
            )
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            sock.bind((self.interface,))
        except OSError as e:
            sock.close()
            raise RuntimeError(
                f"Could not bind to '{self.interface}': {e}. "
                f"Is the interface up? Try: "
                f"sudo ip link set {self.interface} up type can bitrate 1000000"
            ) from e
        sock.settimeout(self.rx_timeout)
        self._sock = sock

        if handshake:
            for name, motor in self.motors.items():
                ok = self._handshake(motor)
                if not ok:
                    print(
                        f"⚠️  No reply from motor '{name}' (id={motor.id}) "
                        f"during handshake — continuing anyway."
                    )

    def disconnect(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def _handshake(self, motor: Motor) -> bool:
        """Type 0: ask for device ID, hope something comes back."""
        try:
            self._send_frame(self._build_id(CommunicationType.GET_DEVICE_ID,
                                            self.host_id, motor.id), b"\x00" * 8)
            self._recv_for(motor.id, expect_mode=None, timeout=0.2)
            return True
        except socket.timeout:
            return False
        except Exception:
            return False

    # ---- Low-level frame I/O ------------------------------------------
    def _check(self) -> socket.socket:
        if self._sock is None:
            raise RuntimeError("Bus not connected. Call connect() first.")
        return self._sock

    @staticmethod
    def _build_id(mode: int, data: int, motor_id: int) -> int:
        return ((mode & 0x1F) << 24) | ((data & 0xFFFF) << 8) | (motor_id & 0xFF)

    def _send_frame(self, can_id: int, payload: bytes) -> None:
        sock = self._check()
        if len(payload) > 8:
            raise ValueError("CAN payload >8 bytes")
        if len(payload) < 8:
            payload = payload + b"\x00" * (8 - len(payload))
        frame = struct.pack(_CAN_FRAME_FMT, can_id | _CAN_EFF_FLAG, 8, payload)
        sock.send(frame)

    def _recv_frame(self, timeout: Optional[float] = None) -> Tuple[int, bytes]:
        sock = self._check()
        if timeout is not None:
            sock.settimeout(timeout)
        try:
            raw = sock.recv(_CAN_FRAME_SIZE)
        finally:
            if timeout is not None:
                sock.settimeout(self.rx_timeout)
        can_id, dlc, data = struct.unpack(_CAN_FRAME_FMT, raw)
        return can_id & 0x1FFFFFFF, data[:dlc]

    def _recv_for(
        self,
        motor_id: int,
        expect_mode: Optional[int],
        timeout: Optional[float] = None,
    ) -> Tuple[int, bytes]:
        """Drain the bus until a frame from `motor_id` (and optionally
        `expect_mode`) shows up. Drops any unrelated frames."""
        deadline = None if timeout is None else (time.monotonic() + timeout)
        while True:
            remaining = None
            if deadline is not None:
                remaining = max(0.0, deadline - time.monotonic())
                if remaining == 0.0:
                    raise socket.timeout("recv_for timed out")
            cid, data = self._recv_frame(remaining)
            mode = (cid >> 24) & 0x1F
            # On feedback frames the motor_id is in the ID's `data` field
            # low byte (bits [8:15]); the `id` byte (bits [0:7]) is the host id.
            mid_in_data = (cid >> 8) & 0xFF
            mid_in_id   = cid & 0xFF
            mid = mid_in_data if mid_in_data == motor_id else mid_in_id
            if mid != motor_id:
                continue
            if expect_mode is not None and mode != expect_mode:
                continue
            return cid, data

    # ---- Helpers to find motor objects --------------------------------
    def _motor(self, name: str) -> Motor:
        if name not in self.motors:
            raise KeyError(f"Unknown motor '{name}'")
        return self.motors[name]

    def _scale(self, motor: Motor, table_dict: Dict[str, float], default: float) -> float:
        return table_dict.get(motor.model, default)

    # ---- Public command API -------------------------------------------
    def transmit(self, comm_type: int, data16: int, motor_id: int,
                 payload: bytes = b"") -> None:
        """Generic raw transmit. `data16` is the 16-bit `data` field of the
        extended CAN ID (commonly the host_id, sometimes a sub-command)."""
        cid = self._build_id(comm_type, data16, motor_id)
        self._send_frame(cid, payload)

    def enable(self, name: str) -> None:
        m = self._motor(name)
        self.transmit(CommunicationType.ENABLE, self.host_id, m.id)
        try:
            self._recv_for(m.id, CommunicationType.MOTOR_FEEDBACK, timeout=0.2)
        except socket.timeout:
            pass

    def disable(self, name: str) -> None:
        m = self._motor(name)
        self.transmit(CommunicationType.STOP, self.host_id, m.id)
        try:
            self._recv_for(m.id, CommunicationType.MOTOR_FEEDBACK, timeout=0.2)
        except socket.timeout:
            pass

    # ---- Parameter read / write ---------------------------------------
    def write(self, name: str, param: _ParamSpec, value) -> None:
        m = self._motor(name)
        index, _dtype, _pname = param
        fmt = param.fmt
        # Pack the value into bytes; right-pad to 4 bytes at offset 4.
        encoded = struct.pack(fmt, value)
        if len(encoded) > 4:
            raise ValueError(f"Param '{_pname}' encoding >4 bytes")
        encoded = encoded + b"\x00" * (4 - len(encoded))
        payload = struct.pack("<H", index) + b"\x00\x00" + encoded
        cid = self._build_id(CommunicationType.WRITE_PARAMETER, self.host_id, m.id)
        self._send_frame(cid, payload)
        try:
            self._recv_for(m.id, CommunicationType.MOTOR_FEEDBACK, timeout=0.2)
        except socket.timeout:
            pass

    def read(self, name: str, param: _ParamSpec):
        m = self._motor(name)
        index, _dtype, _pname = param
        fmt = param.fmt
        payload = struct.pack("<H", index) + b"\x00" * 6
        cid = self._build_id(CommunicationType.READ_PARAMETER, self.host_id, m.id)
        self._send_frame(cid, payload)
        _cid, data = self._recv_for(m.id, CommunicationType.READ_PARAMETER,
                                    timeout=self.rx_timeout)
        size = struct.calcsize(fmt)
        return struct.unpack(fmt, data[4:4 + size])[0]

    # ---- MIT operation frame (Type 1) ---------------------------------
    def write_operation_frame(
        self, name: str,
        position: float, kp: float, kd: float,
        velocity: float, torque_ff: float,
    ) -> None:
        m = self._motor(name)
        p_max  = self._scale(m, table.MODEL_MIT_POSITION_TABLE, P_MAX)
        v_max  = self._scale(m, table.MODEL_MIT_VELOCITY_TABLE, V_MAX)
        t_max  = self._scale(m, table.MODEL_MIT_TORQUE_TABLE,   T_MAX)
        kp_max = self._scale(m, table.MODEL_MIT_KP_TABLE,       KP_MAX)
        kd_max = self._scale(m, table.MODEL_MIT_KD_TABLE,       KD_MAX)

        torque_u16 = _f2u(torque_ff, -t_max, t_max, 16)
        cid = self._build_id(CommunicationType.OPERATION_CONTROL, torque_u16, m.id)
        payload = struct.pack(
            ">HHHH",
            _f2u(position, -p_max, p_max, 16),
            _f2u(velocity, -v_max, v_max, 16),
            _f2u(kp,       KP_MIN, kp_max, 16),
            _f2u(kd,       KD_MIN, kd_max, 16),
        )
        self._send_frame(cid, payload)

    def read_operation_frame(self, name: str) -> Tuple[float, float, float, float]:
        """Wait for the next Type-2 feedback frame from `name` and decode it.
        Returns (position_rad, velocity_rad_s, torque_Nm, temperature_C)."""
        m = self._motor(name)
        p_max = self._scale(m, table.MODEL_MIT_POSITION_TABLE, P_MAX)
        v_max = self._scale(m, table.MODEL_MIT_VELOCITY_TABLE, V_MAX)
        t_max = self._scale(m, table.MODEL_MIT_TORQUE_TABLE,   T_MAX)

        _cid, data = self._recv_for(m.id, CommunicationType.MOTOR_FEEDBACK,
                                    timeout=self.rx_timeout)
        if len(data) < 8:
            raise RuntimeError(f"Short feedback frame ({len(data)}B) from id={m.id}")
        p_u, v_u, t_u, temp_u = struct.unpack(">HHHH", data[:8])
        return (
            _u2f(p_u, -p_max, p_max, 16),
            _u2f(v_u, -v_max, v_max, 16),
            _u2f(t_u, -t_max, t_max, 16),
            temp_u / 10.0,
        )


__all__ = [
    "RobstrideBus", "Motor", "ParameterType", "CommunicationType", "table",
    "P_MAX", "V_MAX", "T_MAX", "KP_MAX", "KD_MAX",
]
