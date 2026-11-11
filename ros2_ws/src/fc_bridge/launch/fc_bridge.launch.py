"""Launch the FC bridge.

    # On the truck (USB serial; /dev/fc0 comes from scripts/udev/99-fc-serial.rules):
    ros2 launch fc_bridge fc_bridge.launch.py
    # Dev container against sim/fake_fc.py:
    ros2 launch fc_bridge fc_bridge.launch.py connection:=udpin:0.0.0.0:14551
    # Dev container against the real FC via scripts/fc_udp_forward.py on the host:
    ros2 launch fc_bridge fc_bridge.launch.py connection:=udpin:0.0.0.0:14550
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('fc_bridge'), 'config', 'fc_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Path to fc_bridge parameters YAML.'),
        DeclareLaunchArgument(
            'connection', default_value='/dev/fc0',
            description='pymavlink connection string: serial device or udpin:HOST:PORT.'),
        Node(
            package='fc_bridge',
            executable='mavlink_bridge_node',
            name='mavlink_bridge_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'connection': LaunchConfiguration('connection')},
            ],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
