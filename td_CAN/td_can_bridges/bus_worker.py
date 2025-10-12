
import threading
import time
import can
import cantools
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from .mapping import TopicTxBinding, RxBinding

def make_qos(profile_dict, default_depth=10):
    q = QoSProfile(depth=profile_dict.get('depth', default_depth))
    rel = profile_dict.get('reliability', 'reliable').upper()
    dur = profile_dict.get('durability', 'volatile').upper()
    q.reliability = getattr(ReliabilityPolicy, rel, ReliabilityPolicy.RELIABLE)
    q.durability  = getattr(DurabilityPolicy,  dur, DurabilityPolicy.VOLATILE)
    q.history = HistoryPolicy.KEEP_LAST
    return q

class BusWorker:
    """Owns one SocketCAN channel, bidirectional ROS<->CAN mapping, and health."""
    def __init__(self, node, cfg, qos_defaults):
        self.node = node
        self.cfg = cfg
        self.name = cfg['name']
        self.interface = cfg['interface']
        self._stop = threading.Event()

        bitrate = cfg.get('bitrate', 500000)
        fd = cfg.get('fd', False)
        dbitrate = cfg.get('dbitrate', None)

        # Open SocketCAN
        if fd and dbitrate:
            self.bus = can.Bus(interface='socketcan', channel=self.interface, fd=True, bitrate=bitrate, data_bitrate=dbitrate)
        else:
            self.bus = can.Bus(interface='socketcan', channel=self.interface, bitrate=bitrate, fd=fd)

        # Acceptance filters (list of dicts with can_id, can_mask)
        if cfg.get('filters'):
            self.bus.set_filters(cfg['filters'])

        # Load DBC
        dbc_path = cfg.get('dbc_file')
        if not dbc_path:
            raise RuntimeError(f"[{self.name}] Missing 'dbc_file' in config.")
        self.dbc = cantools.database.load_file(dbc_path)

        # Build mappings
        self.tx_bindings = []  # ROS -> CAN
        for topic, spec in cfg.get('tx_topics', {}).items():
            qos_name = spec.get('qos', 'command')
            qos = make_qos(qos_defaults.get(qos_name, {}), default_depth=10)
            self.tx_bindings.append(TopicTxBinding(self.node, self.bus, self.dbc, topic, spec, qos))

        self.rx_bindings_by_id = {}  # CAN -> ROS
        for msg_name, spec in cfg.get('rx_frames', {}).items():
            b = RxBinding(self.node, self.dbc, msg_name, spec, make_qos(qos_defaults.get('sensor', {}), default_depth=20))
            self.rx_bindings_by_id[b.frame_id] = b

        # Start RX thread
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.node.get_logger().info(f"[{self.name}] up on {self.interface}, bitrate={bitrate}, fd={fd}, dbitrate={dbitrate}")

    def _rx_loop(self):
        reader = can.BufferedReader()
        notifier = can.Notifier(self.bus, [reader], timeout=0.01)
        try:
            while not self._stop.is_set():
                msg = reader.get_message(timeout=0.1)
                if msg is None:
                    continue
                b = self.rx_bindings_by_id.get(msg.arbitration_id)
                if not b:
                    # Unknown frame; optionally log at debug level
                    continue
                try:
                    b.publish(bytes(msg.data))
                except Exception as e:
                    self.node.get_logger().warn(f"[{self.name}] decode/publish error: {e}")
        finally:
            notifier.stop()

    def shutdown(self):
        self._stop.set()
        try:
            self._rx_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.bus.shutdown()
        except Exception:
            pass
