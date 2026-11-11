"""Everything that runs on the truck's Jetson.

    ros2 launch truck_bringup truck.launch.py                    # FC bridge + cameras + TF
    ros2 launch truck_bringup truck.launch.py slam:=true         # + RTAB-Map (when built)
    ros2 launch truck_bringup truck.launch.py cameras:=false     # FC bridge only
    ros2 launch truck_bringup truck.launch.py publish_compressed:=true   # stream to the PC
    web page: http://<jetson-ip>:8080   (tasks:=false to disable)

Running SLAM on the PC instead (until RTAB-Map is built on the Nano): run this
with publish_compressed:=true on the Jetson, and on the PC
    ros2 launch truck_bringup slam.launch.py compressed:=... (see README)
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _static_tf(name, parent, child, xyz, rpy):
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                   '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                   '--frame-id', parent, '--child-frame-id', child])


def _setup(context, *args, **kwargs):
    share = get_package_share_directory('truck_bringup')
    with open(os.path.join(share, 'config', 'rig.yaml')) as f:
        rig = yaml.safe_load(f)
    calib_dir = LaunchConfiguration('calibration_dir').perform(context)
    map_dir = os.path.join(os.path.expanduser(LaunchConfiguration('maps_dir').perform(context)),
                           LaunchConfiguration('map_name').perform(context))
    if not os.path.isdir(map_dir):
        os.makedirs(map_dir)
    half_b = rig['baseline'] / 2.0
    opt = (-1.5707963, 0.0, -1.5707963)   # camera link (x fwd) -> optical (z fwd)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('fc_bridge'), 'launch', 'fc_bridge.launch.py')),
            launch_arguments={'connection': LaunchConfiguration('fc')}.items(),
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('fc'), "' != 'none'"]))),
        Node(
            package='truck_bringup', executable='stereo_camera_node', name='stereo_camera_node',
            output='screen', respawn=True, respawn_delay=3.0,
            condition=IfCondition(LaunchConfiguration('cameras')),
            parameters=[os.path.join(share, 'config', 'cameras.yaml'), {
                'left_calibration': os.path.join(calib_dir, 'left.yaml'),
                'right_calibration': os.path.join(calib_dir, 'right.yaml'),
                'publish_compressed': LaunchConfiguration('publish_compressed'),
            }]),
        Node(package='truck_bringup', executable='detector_node', name='detector', output='screen',
             condition=IfCondition(LaunchConfiguration('detector')),
             parameters=[{'left_topic': 'stereo/left/image_rect_color',
                          'right_topic': 'stereo/right/image_rect'}]),
        Node(package='truck_bringup', executable='collision_guard', name='collision_guard',
             output='screen', condition=IfCondition(LaunchConfiguration('guard')),
             parameters=[{'left_topic': 'stereo/left/image_rect_color',
                          'right_topic': 'stereo/right/image_rect',
                          'cam_height': rig['cam_z'] + 0.1, 'cam_pitch': rig['cam_pitch']}]),
        # Rectified images for the collision guard, detector and SLAM - always
        # on with the cameras (SLAM may run on the PC, or not at all).
        Node(package='image_proc', executable='rectify_node', name='rectify_left',
             namespace='stereo/left', condition=IfCondition(LaunchConfiguration('cameras')),
             remappings=[('image', 'image_raw'), ('image_rect', 'image_rect_color')]),
        Node(package='image_proc', executable='rectify_node', name='rectify_right',
             namespace='stereo/right', condition=IfCondition(LaunchConfiguration('cameras')),
             remappings=[('image', 'image_raw'), ('image_rect', 'image_rect')]),
        Node(package='truck_bringup', executable='scan_guard', name='scan_guard', output='screen',
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('lidar'), "' == 'true' and '",
                                                     LaunchConfiguration('guard'), "' == 'true'"]))),
        # RPLidar A1 (Slamtec driver, built into the Jetson image)
        Node(package='rplidar_ros', executable='rplidar_node', name='rplidar', output='screen',
             condition=IfCondition(LaunchConfiguration('lidar')), respawn=True, respawn_delay=3.0,
             parameters=[{'serial_port': LaunchConfiguration('lidar_port'), 'serial_baudrate': 115200,
                          'frame_id': 'laser', 'angle_compensate': True, 'inverted': False,
                          'scan_mode': 'Sensitivity'}]),
        _static_tf('tf_laser', 'base_link', 'laser',
                   (rig.get('lidar_x', 0.12), rig.get('lidar_y', 0.0), rig.get('lidar_z', 0.13)),
                   (0, 0, rig.get('lidar_yaw', 0.0))),
        _static_tf('tf_fc', 'base_link', 'fc_link', (rig['fc_x'], rig['fc_y'], rig['fc_z']), (0, 0, 0)),
        _static_tf('tf_left_cam', 'base_link', 'left_camera_link',
                   (rig['cam_x'], rig['cam_y'] + half_b, rig['cam_z']), (0, rig['cam_pitch'], 0)),
        _static_tf('tf_right_cam', 'base_link', 'right_camera_link',
                   (rig['cam_x'], rig['cam_y'] - half_b, rig['cam_z']), (0, rig['cam_pitch'], 0)),
        _static_tf('tf_left_opt', 'left_camera_link', 'left_camera_optical_frame', (0, 0, 0), opt),
        _static_tf('tf_right_opt', 'right_camera_link', 'right_camera_optical_frame', (0, 0, 0), opt),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('truck_tasks'), 'launch', 'tasks.launch.py')),
            launch_arguments={'places_file': os.path.join(map_dir, 'places.json'),
                              'map_dir': map_dir}.items(),
            condition=IfCondition(LaunchConfiguration('tasks'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'slam.launch.py')),
            # rectification already runs above; SLAM just consumes image_rect*
            launch_arguments={'rectify': 'false', 'rect_topics': 'true',
                              'lidar': LaunchConfiguration('lidar'),
                              'odom': LaunchConfiguration('odom'),
                              'database': os.path.join(map_dir, 'rtabmap.db'),
                              'localization': LaunchConfiguration('localization'),
                              'start_at': LaunchConfiguration('start_at'),
                              'fresh_map': LaunchConfiguration('fresh_map')}.items(),
            condition=IfCondition(LaunchConfiguration('slam'))),
    ]


def generate_launch_description():
    # The calibration lives in the source tree so it's versioned with the rig;
    # in the Jetson container the repo is at /workspace.
    default_calib = os.environ.get(
        'RCTRUCK_CALIB_DIR', '/workspace/ros2_ws/src/truck_bringup/config/camera_info')
    return LaunchDescription([
        DeclareLaunchArgument('fc', default_value='/dev/fc0',
                              description="FC connection, or 'none' for cameras only"),
        DeclareLaunchArgument('cameras', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('lidar', default_value='false',
                              description='RPLidar A1 on lidar_port (+ fused into SLAM)'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('odom', default_value='auto',
                              description="SLAM odometry: 'auto' (lidar if present), 'stereo' or 'lidar'"),
        DeclareLaunchArgument('maps_dir', default_value='~/.ros/maps'),
        DeclareLaunchArgument('map_name', default_value='default',
                              description='map folder: RTAB-Map database, map PNG, places'),
        DeclareLaunchArgument('start_at', default_value='origin',
                              description="localization: truck starts at the map's origin ('origin') or where the last session ended ('last')"),
        DeclareLaunchArgument('localization', default_value='false',
                              description='true = relocalize in map_name instead of mapping'),
        DeclareLaunchArgument('fresh_map', default_value='true',
                              description='mapping: start map_name from scratch (false = extend)'),
        DeclareLaunchArgument('detector', default_value='false',
                              description='object detector plugin (set backend/model in detector params)'),
        DeclareLaunchArgument('guard', default_value='true',
                              description='stereo collision guard (needs rectified images: slam or rectify)'),
        DeclareLaunchArgument('tasks', default_value='true',
                              description='task manager + navigator + explorer + web page :8080'),
        DeclareLaunchArgument('publish_compressed', default_value='false'),
        DeclareLaunchArgument('calibration_dir', default_value=default_calib),
        OpaqueFunction(function=_setup),
    ])
