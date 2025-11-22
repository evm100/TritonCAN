
from importlib import import_module
from typing import Any, Dict

from rclpy.qos import QoSProfile

from .service import CanBusService, TxBindingConfig, RxBindingConfig


def resolve_ros_type(ros_type_str, default='std_msgs/msg/Float32'):
    """Resolve a ROS 2 message type string like ``std_msgs/msg/Float32``."""

    typename = ros_type_str or default
    parts = typename.split('/')
    if len(parts) != 3 or parts[1] != 'msg':
        raise ValueError(f"Invalid ROS type string: {typename}")
    mod = parts[0] + '.' + parts[1]
    cls = parts[2]
    m = import_module(mod)
    return getattr(m, cls)


class TopicTxBinding:
    """ROS → CAN: subscribe to a ROS topic and pack to a DBC frame."""

    def __init__(self, node, service: CanBusService, binding: TxBindingConfig, qos_profile: QoSProfile):
        self.node = node
        self.service = service
        self.binding = binding
        self.service.register_tx_binding(binding)
        self.msg_def = service.dbc.get_message_by_name(binding.message)

        metadata = dict(binding.metadata)
        self.topic = metadata.get('topic', binding.key)
        msg_type = resolve_ros_type(metadata.get('type', 'std_msgs/msg/Float32'))

        self.subscription = node.create_subscription(msg_type, self.topic, self._cb, qos_profile)
        node.get_logger().info(
            f"TX bind: {self.topic} -> DBC:{self.msg_def.name} (id=0x{self.msg_def.frame_id:X})"
        )

    def _cb(self, ros_msg):
        payload: Dict[str, Any] = {}
        if self.binding.fields:
            for ros_field in self.binding.fields.keys():
                if not hasattr(ros_msg, ros_field):
                    self.node.get_logger().error(
                        f"Message on {self.topic} missing field '{ros_field}'",
                        throttle_duration_sec=5.0,
                    )
                    return
                payload[ros_field] = getattr(ros_msg, ros_field)
        elif hasattr(ros_msg, '__slots__'):
            payload = {slot: getattr(ros_msg, slot) for slot in ros_msg.__slots__ if not slot.startswith('_')}
        elif hasattr(ros_msg, '__dict__'):
            payload = dict(ros_msg.__dict__)

        if not payload and hasattr(ros_msg, 'data'):
            payload['data'] = getattr(ros_msg, 'data')

        try:
            self.service.send(self.binding.key, payload)
        except Exception as exc:
            self.node.get_logger().error(
                f"Failed to send CAN frame for topic {self.topic}: {exc}",
                throttle_duration_sec=5.0,
            )

    def shutdown(self):
        if self.subscription:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None


class RxBinding:
    """CAN → ROS: decode a DBC frame and publish to a ROS topic."""

    def __init__(self, node, service: CanBusService, binding: RxBindingConfig, qos_profile: QoSProfile):
        self.node = node
        self.service = service
        self.binding = binding

        metadata = dict(binding.metadata)
        self.topic = metadata.get('topic', binding.key)
        msg_type = resolve_ros_type(metadata.get('type', 'std_msgs/msg/Float32'))
        self.msg_type = msg_type
        self.pub = node.create_publisher(msg_type, self.topic, qos_profile)

        service.register_rx_binding(binding, self._handle_frame)
        self.msg_def = service.dbc.get_message_by_name(binding.message)
        self.frame_id = self.msg_def.frame_id
        node.get_logger().info(
            f"RX bind: DBC:{self.msg_def.name} (id=0x{self.frame_id:X}) -> {self.topic}"
        )

    def _handle_frame(self, payload: Dict[str, Any], binding: RxBindingConfig):
        msg = self.msg_type()
        if hasattr(msg, 'data') and len(payload) == 1:
            msg.data = next(iter(payload.values()))
        else:
            for field, value in payload.items():
                if hasattr(msg, field):
                    setattr(msg, field, value)
        self.pub.publish(msg)

    def shutdown(self):
        if self.pub:
            self.node.destroy_publisher(self.pub)
            self.pub = None
