
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from .mapping import TopicTxBinding, RxBinding
from .service import BusConfig, CanBusService

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

    def __init__(self, node, cfg: BusConfig, qos_defaults):
        self.node = node
        self.cfg = cfg
        self.name = cfg.name

        self.service = CanBusService(cfg)

        # Build TX bindings (ROS -> CAN)
        self.tx_bindings = []
        for key, binding in cfg.tx_bindings.items():
            metadata = dict(binding.metadata)
            qos_name = metadata.get('qos', 'command')
            qos = make_qos(qos_defaults.get(qos_name, {}), default_depth=10)
            self.tx_bindings.append(TopicTxBinding(self.node, self.service, binding, qos))

        # Build RX bindings (CAN -> ROS)
        self.rx_bindings = []
        for key, binding in cfg.rx_bindings.items():
            qos = make_qos(qos_defaults.get('sensor', {}), default_depth=20)
            self.rx_bindings.append(RxBinding(self.node, self.service, binding, qos))

        self.service.start()
        self.node.get_logger().info(
            f"[{self.name}] up on {self.cfg.interface}, bitrate={self.cfg.bitrate}, "
            f"fd={self.cfg.fd}, dbitrate={self.cfg.dbitrate}"
        )

    def shutdown(self):
        for binding in self.rx_bindings:
            binding.shutdown()
        for binding in self.tx_bindings:
            binding.shutdown()
        self.service.shutdown()
