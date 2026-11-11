"""Task system: task manager + navigator + explorer (+ web page).

    ros2 launch truck_tasks tasks.launch.py                  # with the web page on :8080
    ros2 launch truck_tasks tasks.launch.py web:=false use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('web', default_value='true'),
        DeclareLaunchArgument('places_file', default_value='~/.ros/places.json'),
        DeclareLaunchArgument('bags_dir', default_value='~/.ros/bags'),
        DeclareLaunchArgument('map_dir', default_value='',
                              description='map_room saves map.png/yaml here when it finishes'),
        DeclareLaunchArgument('low_voltage', default_value='0.0',
                              description='battery V that sends the truck home (0 = off)'),
        DeclareLaunchArgument('critical_voltage', default_value='0.0',
                              description='battery V that stops everything (0 = off)'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('web_token', default_value=''),
        DeclareLaunchArgument('planner', default_value='false',
                              description='LLM planner plugin (PC only: needs the anthropic SDK)'),
        Node(package='truck_tasks', executable='task_manager', name='task_manager',
             output='screen', parameters=[sim], respawn=True, respawn_delay=2.0),
        Node(package='truck_tasks', executable='navigator', name='navigator',
             output='screen', parameters=[sim], respawn=True, respawn_delay=2.0),
        Node(package='truck_tasks', executable='watchdog', name='watchdog', output='screen',
             parameters=[sim, {'low_voltage': LaunchConfiguration('low_voltage'),
                               'critical_voltage': LaunchConfiguration('critical_voltage')}],
             respawn=True, respawn_delay=2.0),
        Node(package='truck_tasks', executable='places_node', name='places',
             output='screen', parameters=[sim, {'file': LaunchConfiguration('places_file')}],
             respawn=True, respawn_delay=2.0),
        Node(package='truck_tasks', executable='explorer', name='explorer',
             output='screen', parameters=[sim, {'save_dir': LaunchConfiguration('map_dir')}], respawn=True, respawn_delay=2.0),
        Node(package='truck_planner', executable='planner_node', name='planner', output='screen',
             condition=IfCondition(LaunchConfiguration('planner')), parameters=[sim]),
        Node(package='truck_bringup', executable='recorder_node', name='recorder', output='screen',
             parameters=[sim, {'dir': LaunchConfiguration('bags_dir')}], respawn=True, respawn_delay=2.0),
        Node(package='truck_web', executable='web_node', name='web_node', output='screen',
             condition=IfCondition(LaunchConfiguration('web')),
             parameters=[sim, {'map_dir': LaunchConfiguration('map_dir'),
                               'port': LaunchConfiguration('web_port'),
                               'token': LaunchConfiguration('web_token')}]),
    ])
