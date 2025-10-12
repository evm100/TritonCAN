
from importlib import import_module
from rclpy.qos import QoSProfile
import can

# Minimal dynamic type resolver: e.g. 'std_msgs/msg/Float32'
def resolve_ros_type(ros_type_str, default='std_msgs/msg/Float32'):
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
    def __init__(self, node, bus, dbc, topic, spec, qos_profile: QoSProfile):
        self.node = node
        self.bus = bus
        dbc_message = spec['dbc_message']
        self.msg_def = dbc.get_message_by_name(dbc_message)
        self.field_map = spec.get('fields', {})  # ros_field -> dbc_signal
        msg_type = resolve_ros_type(spec.get('type', 'std_msgs/msg/Float32'))
        self.node.create_subscription(msg_type, topic, self._cb, qos_profile)
        self.node.get_logger().info(f"TX bind: {topic} -> DBC:{self.msg_def.name} (id=0x{self.msg_def.frame_id:X})" )

    def _cb(self, ros_msg):
        # Build a dict of DBC signal -> value from ROS message fields
        payload = {}
        for ros_field, dbc_signal in self.field_map.items():
            payload[dbc_signal] = getattr(ros_msg, ros_field)
        data = self.msg_def.encode(payload)
        self.bus.send(can.Message(arbitration_id=self.msg_def.frame_id,
                                  data=data,
                                  is_extended_id=self.msg_def.is_extended_frame))

class RxBinding:
    """CAN → ROS: decode a DBC frame and publish to a ROS topic.
    Supports mapping a single DBC signal to a simple ROS message field, or
    a dict of signal->field mappings for complex types.
    """
    def __init__(self, node, dbc, dbc_msg_name, spec, qos_profile: QoSProfile):
        self.node = node
        self.msg_def = dbc.get_message_by_name(dbc_msg_name)
        self.frame_id = self.msg_def.frame_id
        self.topic = spec['topic']
        self.field_map = spec.get('fields', {})  # dbc_signal -> ros_field
        msg_type = resolve_ros_type(spec.get('type', 'std_msgs/msg/Float32'))
        self.pub = node.create_publisher(msg_type, self.topic, qos_profile)
        self.msg_type = msg_type
        node.get_logger().info(f"RX bind: DBC:{self.msg_def.name} (id=0x{self.frame_id:X}) -> {self.topic}" )

    def publish(self, raw_bytes: bytes):
        decoded = self.msg_def.decode(raw_bytes)
        msg = self.msg_type()
        if hasattr(msg, 'data') and len(self.field_map) == 1:
            # Map the only specified dbc signal to .data
            dbc_signal = next(iter(self.field_map.keys()))
            setattr(msg, 'data', decoded.get(dbc_signal, 0.0))
        else:
            # Generic mapping for richer messages (fields must exist)
            for dbc_signal, ros_field in self.field_map.items():
                if hasattr(msg, ros_field):
                    setattr(msg, ros_field, decoded.get(dbc_signal))
        self.pub.publish(msg)
