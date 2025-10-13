import rclpy
from pathlib import Path
from rclpy.node import Node

from .bus_worker import BusWorker
from .service import load_bridge_config


class TDCANBridge(Node):
    """Single ROS 2 node that can manage one or multiple SocketCAN interfaces."""

    def __init__(self):
        super().__init__('td_can_bridge')
        cfg_path = self.declare_parameter('config', '').get_parameter_value().string_value
        if not cfg_path:
            raise RuntimeError("parameter 'config' is required (path to YAML config).")

        cfg_file = Path(cfg_path)
        if not cfg_file.exists():
            raise FileNotFoundError(f"Config file not found: {cfg_file}")

        self.bridge_cfg = load_bridge_config(cfg_file)

        # Optional global logging level
        log_cfg = self.bridge_cfg.logging
        if log_cfg.get('level'):
            level_name = log_cfg['level'].upper()
            level = getattr(rclpy.logging.LoggingSeverity, level_name, rclpy.logging.LoggingSeverity.INFO)
            self.get_logger().set_level(level)

        self.workers = []

        buses = self.bridge_cfg.buses
        if not buses:
            raise RuntimeError("Config has no 'buses' entries.")

        for bus_cfg in buses:
            worker = BusWorker(self, bus_cfg, self.bridge_cfg.qos)
            self.workers.append(worker)

        self.get_logger().info(f"td_can_bridge started with {len(self.workers)} bus(es).")

    def destroy_node(self):
        for worker in self.workers:
            worker.shutdown()
        super().destroy_node()


def main():
    rclpy.init()
    node = TDCANBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

