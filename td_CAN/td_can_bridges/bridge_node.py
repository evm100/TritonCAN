
import rclpy
from rclpy.node import Node
from pathlib import Path
import yaml
from .bus_worker import BusWorker

class TDCANBridge(Node):
    """Single ROS 2 node that can manage one or multiple SocketCAN interfaces.
    Launch multiple instances with different configs OR one instance with multiple buses.
    """
    def __init__(self):
        super().__init__('td_can_bridge')
        cfg_path = self.declare_parameter('config', '').get_parameter_value().string_value
        if not cfg_path:
            raise RuntimeError("parameter 'config' is required (path to YAML config)." )

        cfg_file = Path(cfg_path)
        if not cfg_file.exists():
            raise FileNotFoundError(f"Config file not found: {cfg_file}")

        with cfg_file.open('r') as f:
            self.cfg = yaml.safe_load(f)

        # Optional global logging level
        log_cfg = self.cfg.get('logging', {})
        if log_cfg.get('level'):
            self.get_logger().set_level(getattr(rclpy.logging.LoggingSeverity, log_cfg['level'].upper(), rclpy.logging.LoggingSeverity.INFO))

        self.workers = []
        base_dir = cfg_file.parent

        buses = self.cfg.get('buses', [])
        if not buses:
            raise RuntimeError("Config has no 'buses' entries.")

        for bus_cfg in buses:
            if 'name' not in bus_cfg or 'interface' not in bus_cfg:
                raise RuntimeError("Each bus entry must have 'name' and 'interface'.")
            if 'dbc_file' in bus_cfg:
                # resolve relative to config dir
                bus_cfg['dbc_file'] = str((base_dir / bus_cfg['dbc_file']).resolve())

            worker = BusWorker(self, bus_cfg, self.cfg.get('qos', {}))
            self.workers.append(worker)

        self.get_logger().info(f"td_can_bridge started with {len(self.workers)} bus(es).")

    def destroy_node(self):
        for w in self.workers:
            w.shutdown()
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
