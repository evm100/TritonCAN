
from setuptools import setup

package_name = 'td_can_bridges'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/example_multibus.yaml', 'config/example_singlebus.yaml']),
        ('share/' + package_name + '/launch', ['launch/td_can_multibus.launch.py', 'launch/td_can_motor_and_sensor_split.launch.py']),
        ('share/' + package_name + '/schemas', ['td_can_bridges/schemas/motors.dbc', 'td_can_bridges/schemas/sensors.dbc']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='td',
    maintainer_email='td@example.com',
    description='Multi-bus ROS 2 <-> SocketCAN bridges with DBC mapping',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'td_can_bridge = td_can_bridges.bridge_node:main',
        ],
    },
)
