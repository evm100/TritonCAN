
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share = get_package_share_directory('td_can_bridges')
    motor_cfg  = os.path.join(share, 'config', 'example_singlebus.yaml')
    sensor_cfg = os.path.join(share, 'config', 'example_singlebus.yaml')  # duplicate or provide another file
    return LaunchDescription([
        Node(package='td_can_bridges', executable='td_can_bridge', name='motor_can_bridge',
             output='screen', parameters=[{'config': motor_cfg}]),
        Node(package='td_can_bridges', executable='td_can_bridge', name='sensor_can_bridge',
             output='screen', parameters=[{'config': sensor_cfg}])
    ])
