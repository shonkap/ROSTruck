"""Cameras only (for calibration or checking the rig), without the FC.

    ros2 launch truck_bringup cameras.launch.py
    ros2 launch truck_bringup cameras.launch.py publish_compressed:=true   # calibrate from the PC
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('publish_compressed', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('truck_bringup'), 'launch', 'truck.launch.py')),
            launch_arguments={
                'cameras': 'true', 'slam': 'false', 'fc': 'none',
                'publish_compressed': LaunchConfiguration('publish_compressed'),
            }.items()),
    ])
