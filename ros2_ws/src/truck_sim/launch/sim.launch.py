"""Full simulation: Gazebo room + Ground Pounder + ArduPilot SITL + fc_bridge + SLAM.

    ros2 launch truck_sim sim.launch.py                 # everything, with GUIs
    ros2 launch truck_sim sim.launch.py gui:=false rviz:=false   # headless
    ros2 launch truck_sim sim.launch.py slam:=false     # just drive
    web page: http://localhost:8080  (map, click-to-drive waypoints, Map room task)

Then hand control to ROS (what the pilot does with the mode switch on the
truck) and drive:
    ros2 run truck_sim sim_ctl guided arm
    ros2 run truck_sim explore_room                     # automatic mapping run
    ros2 run teleop_twist_keyboard teleop_twist_keyboard   # or drive by hand

Data path is the same as on the truck:
    cmd_vel -> fc_bridge -> MAVLink (TCP 5760) -> ArduRover SITL 4.7.2
            -> PWM on SERVO1/2/3 -> Gazebo ArduPilotPlugin -> steering + wheels
    Gazebo cameras -> stereo_relay -> stereo/* -> RTAB-Map (+ fc/imu)
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from truck_bringup.rig_launch import rig_tfs, safety_and_detection

# The repo is bind-mounted here in the sim container (docker-compose.sim.yml).
REPO = os.environ.get('RCTRUCK_REPO', '/workspace')
SIM = os.path.join(REPO, 'sim')
ARDUPILOT = os.environ.get('ARDUPILOT_HOME', '/opt/ardupilot')
HOME = '47.6062,-122.3321,50,0'   # must match <spherical_coordinates> in the world


def _ensure_generated():
    """Room textures/SDF are generated (and gitignored); build them on first run."""
    room = os.path.join(SIM, 'gazebo', 'models', 'living_room')
    if not os.path.exists(os.path.join(room, 'model.sdf')):
        import subprocess
        subprocess.check_call(['python3', os.path.join(room, 'make_room.py')])


def _sim_already_running():
    """True if something (another sim) already listens on SITL's MAVLink port."""
    import socket
    s = socket.socket()
    s.settimeout(0.5)
    try:
        return s.connect_ex(('127.0.0.1', 5760)) == 0
    finally:
        s.close()


def _launch_setup(context, *args, **kwargs):
    if _sim_already_running():
        raise RuntimeError('Another simulator is already running (SITL port 5760 is in use). '
                           'Two sims fight over the same ports and topics - stop it first: '
                           './sim/stop_sim.sh')
    _ensure_generated()
    world = LaunchConfiguration('world').perform(context)
    with open(os.path.join(SIM, 'gazebo', 'models', 'ground_pounder', 'rig.yaml')) as f:
        rig = yaml.safe_load(f)
    indoor = LaunchConfiguration('indoor').perform(context) == 'true'
    map_dir = os.path.join(REPO, 'maps', LaunchConfiguration('map_name').perform(context))
    localization = LaunchConfiguration('localization').perform(context)
    os.makedirs(map_dir, exist_ok=True)
    sitl_dir = os.path.join(SIM, '.sitl')
    os.makedirs(sitl_dir, exist_ok=True)

    actions = [
        ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-r', '-v', '2', world],
            name='gazebo', output='log'),
        ExecuteProcess(
            cmd=['gz', 'sim', '-g', '-v', '1'],
            name='gazebo_gui', output='log',
            condition=IfCondition(LaunchConfiguration('gui'))),
        # Same firmware version as the truck. SERIAL0 (5760) = fc_bridge,
        # SERIAL1 (5762) = GCS on the host, SERIAL2 (5763) = sim_ctl.
        ExecuteProcess(
            cmd=['ardurover', '-S', '-w', '--model', 'JSON', '--speedup', '1',
                 '--sim-address', '127.0.0.1', '-I0', '--home', HOME,
                 '--serial0', 'tcp:0', '--serial1', 'tcp:2', '--serial2', 'tcp:3',
                 '--defaults', ','.join([
                     os.path.join(ARDUPILOT, 'Tools/autotest/default_params/rover.parm'),
                     os.path.join(SIM, 'params', 'ground_pounder_sitl.parm')] +
                    ([os.path.join(SIM, 'params', 'indoor_extnav.parm')] if indoor else []))],
            cwd=sitl_dir, name='sitl', output='log'),

        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/stereo/left/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/stereo/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/stereo/right/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/stereo/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/model/ground_pounder/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ],
            remappings=[
                ('/clock', 'gz/clock'),      # 1 kHz; clock_relay republishes /clock at 100 Hz
                ('/stereo/left/image', 'gz/stereo/left/image'),
                ('/stereo/left/camera_info', 'gz/stereo/left/camera_info'),
                ('/stereo/right/image', 'gz/stereo/right/image'),
                ('/stereo/right/camera_info', 'gz/stereo/right/camera_info'),
                ('/model/ground_pounder/odometry', 'ground_truth/odom'),
            ],
            parameters=[{'use_sim_time': True}], output='log'),

        Node(package='truck_sim', executable='clock_relay', name='clock_relay', output='log'),
        Node(package='truck_sim', executable='stereo_relay', name='stereo_relay',
             parameters=[{'use_sim_time': True, 'baseline': rig['baseline']}], output='screen'),

        # Same TF tree, guards and detector as the truck (sim images are already rectified).
        *rig_tfs(rig, sim=True),
        *safety_and_detection(rig, 'stereo/left/image_raw', 'stereo/right/image_raw', sim=True, output='log',
                              detector_params={'rules': LaunchConfiguration('detector_rules').perform(context)}),

        Node(
            package='fc_bridge', executable='mavlink_bridge_node', name='mavlink_bridge_node',
            output='screen', respawn=True, respawn_delay=2.0,
            parameters=[
                os.path.join(get_package_share_directory('fc_bridge'), 'config', 'fc_bridge.yaml'),
                {'connection': 'tcp:127.0.0.1:5760', 'use_sim_time': True,
                 'vision_pose': indoor,
                 'auto_apply': LaunchConfiguration('auto_apply').perform(context) == 'true'}]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('truck_bringup'), 'launch', 'slam.launch.py')),
            launch_arguments={'use_sim_time': 'true', 'rectify': 'false',
                              'database': os.path.join(map_dir, 'rtabmap.db'),
                              'localization': localization,
                              'lidar': LaunchConfiguration('lidar').perform(context),
                              'odom': LaunchConfiguration('odom').perform(context),
                              'start_at': LaunchConfiguration('start_at'),
                              'fresh_map': LaunchConfiguration('fresh_map').perform(context)}.items(),
            condition=IfCondition(LaunchConfiguration('slam'))),

        # Task system + web page (http://localhost:8080)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('truck_tasks'), 'launch', 'tasks.launch.py')),
            launch_arguments={'use_sim_time': 'true', 'planner': 'true',
                              'places_file': os.path.join(map_dir, 'places.json'),
                              'map_dir': map_dir,
                              'bags_dir': os.path.join(REPO, 'bags')}.items(),
            condition=IfCondition(LaunchConfiguration('tasks'))),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', os.path.join(get_package_share_directory('truck_sim'),
                                          'config', 'sim.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(SIM, 'gazebo', 'worlds', 'room.sdf')),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI (noVNC desktop)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('tasks', default_value='true',
                              description='task manager, navigator, explorer, web page on :8080'),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='fuse the simulated RPLidar A1 into SLAM'),
        DeclareLaunchArgument('odom', default_value='auto',
                              description="odometry: 'auto' (lidar if present), 'stereo' or 'lidar'"),
        DeclareLaunchArgument('detector_rules', default_value='[]',
                              description='JSON list of detection rules, e.g. stop near a person'),
        DeclareLaunchArgument('detector', default_value='true',
                              description='object detector plugin (demo: finds the red bin)'),
        DeclareLaunchArgument('guard', default_value='true',
                              description='collision guards: stereo + lidar (the map guard is in tasks)'),
        DeclareLaunchArgument('map_name', default_value='sim_room',
                              description='map folder under maps/: RTAB-Map database, map PNG, places'),
        DeclareLaunchArgument('start_at', default_value='origin',
                              description="localization: truck starts at the map's origin ('origin') or where the last session ended ('last')"),
        DeclareLaunchArgument('localization', default_value='false',
                              description='true = load map_name and relocalize in it instead of mapping'),
        DeclareLaunchArgument('fresh_map', default_value='true',
                              description='mapping mode: start map_name from scratch (false = extend it)'),
        DeclareLaunchArgument('indoor', default_value='false',
                              description='No GPS: the FC navigates on the SLAM pose (like the real truck indoors)'),
        DeclareLaunchArgument('auto_apply', default_value='false',
                              description='fc_bridge steering monitor writes its fixes to the FC'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join([
            os.path.join(SIM, 'gazebo', 'models'), os.path.join(SIM, 'gazebo', 'worlds'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')])),
        OpaqueFunction(function=_launch_setup),
    ])
