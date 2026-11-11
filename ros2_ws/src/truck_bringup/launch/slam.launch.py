"""SLAM with RTAB-Map - stereo cameras, optionally fused with the 2D lidar.
Shared by the sim and the real truck.

No wheel encoders on this truck, so odometry comes from a sensor:
    odom:=auto     lidar if lidar:=true, else stereo (default)
    odom:=stereo   stereo visual odometry (stereo_odometry)
    odom:=lidar    2D lidar scan matching (icp_odometry, RPLidar) - works on blank walls
                   and in the dark. In the sim it was ~3x more accurate than stereo
                   (ATE rms 0.6 vs 1.8 cm over a 19 m drive, scripts/sim_odom_compare.sh).
both helped by the FC's IMU (fc/imu) for gravity / roll-pitch.

With lidar:=true RTAB-Map builds ONE map from both sensors: loop closures and
odometry links are refined with lidar scan matching (Vis+ICP), and the 2D grid
takes walls from the lidar (clean, long range) plus low obstacles from stereo
(things under the lidar's scan plane).

Inputs:  stereo/{left,right}/image_* + camera_info, fc/imu, scan (lidar),
         TF base_link -> cameras / laser
Outputs: odom + TF odom -> base_link, map (OccupancyGrid), cloud_map, TF map -> odom

    ros2 launch truck_bringup slam.launch.py                              # real truck, stereo
    ros2 launch truck_bringup slam.launch.py lidar:=true odom:=lidar      # + RPLidar
    ros2 launch truck_bringup slam.launch.py rectify:=false use_sim_time:=true   # sim
    ros2 launch truck_bringup slam.launch.py localization:=true           # reuse saved map
    ros2 launch truck_bringup slam.launch.py compressed:=true             # on the PC, cameras
        on the Jetson (truck.launch.py publish_compressed:=true): decompress locally first
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 2D lidar scan matching (RPLidar A1: ~360 points, 5.5 Hz).
ICP_2D = {
    'Icp/VoxelSize': '0.05',
    'Icp/MaxCorrespondenceDistance': '0.15',
    'Icp/PointToPlane': 'false',        # 2D scans: point-to-point is the robust choice
    'Icp/Iterations': '30',
    'Icp/Epsilon': '0.001',
    'Icp/MaxTranslation': '0.5',
    'Icp/CorrespondenceRatio': '0.2',
}


def _setup(context, *args, **kwargs):
    a = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    use_sim_time = a('use_sim_time') == 'true'
    rectify = a('rectify') == 'true'
    use_rect = rectify or a('rect_topics') == 'true'
    localization = a('localization') == 'true'
    lidar = a('lidar') == 'true'
    odom = a('odom')
    if odom == 'auto':
        odom = 'lidar' if lidar else 'stereo'
    if odom not in ('stereo', 'lidar'):
        raise RuntimeError("odom must be 'auto', 'stereo' or 'lidar', got " + odom)
    if odom == 'lidar' and not lidar:
        lidar = True                    # lidar odometry implies the lidar is there

    common = {
        'use_sim_time': use_sim_time,
        'frame_id': 'base_link',
        # stereo_camera_node and stereo_relay both give the pair identical stamps.
        'approx_sync': False,
        # rtabmap QoS: 0 = default, 1 = reliable, 2 = best effort. Cameras and
        # fc/imu publish with the sensor-data (best effort) profile.
        'qos': 2,
        'qos_camera_info': 2,
        'qos_imu': 2,
        'qos_scan': 2,
        # Ground vehicle: solve x, y, yaw only.
        'Reg/Force3DoF': 'true',
    }
    remaps = [
        ('left/image_rect', 'stereo/left/image_rect_color' if use_rect else 'stereo/left/image_raw'),
        ('left/camera_info', 'stereo/left/camera_info'),
        ('right/image_rect', 'stereo/right/image_rect' if use_rect else 'stereo/right/image_raw'),
        ('right/camera_info', 'stereo/right/camera_info'),
        ('imu', 'fc/imu'),
        ('odom', 'odom'),
        ('scan', 'scan'),
    ]

    odom_common = dict(common, odom_frame_id='odom', publish_tf=True, wait_imu_to_init=True)
    odom_common['Odom/ResetCountdown'] = '1'     # auto-recover if odometry gets lost
    if odom == 'stereo':
        odom_node = Node(
            package='rtabmap_odom', executable='stereo_odometry', name='stereo_odometry',
            output='screen', remappings=remaps,
            parameters=[dict(odom_common, **{
                'Odom/Strategy': '0',          # frame-to-map
                'Vis/MaxFeatures': '800',
                'Vis/MinInliers': '15',
                'GFTT/MinDistance': '5',
                # 10 cm baseline: 160 px max disparity = ~0.24 m minimum stereo range
                'Stereo/MaxDisparity': '160',
                'OdomF2M/MaxSize': '2000',
            })])
    else:
        odom_node = Node(
            package='rtabmap_odom', executable='icp_odometry', name='icp_odometry',
            output='screen', remappings=remaps,
            parameters=[dict(odom_common, **dict(ICP_2D, **{
                'Odom/ScanKeyFrameThr': '0.6',
                'OdomF2M/ScanSubtractRadius': '0.05',
                'OdomF2M/ScanMaxSize': '10000',
            }))])

    slam_params = dict(common, **{
        'subscribe_stereo': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan': lidar,
        'subscribe_odom_info': True,
        'database_path': a('database'),
        'Mem/IncrementalMemory': 'false' if localization else 'true',
        'Mem/InitWMWithAllNodes': 'true' if localization else 'false',
        # Localization mode: where is the truck at start-up? 'origin' = where the
        # map was started (park it at its home spot); 'last' = where the previous
        # session ended. Relocalization corrects either guess once it
        # recognises the room.
        'RGBD/StartAtOrigin': 'true' if a('start_at') == 'origin' else 'false',
        'RGBD/CreateOccupancyGrid': 'true',
        'Grid/RangeMax': '6.0' if lidar else '4.0',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '1.0',
        'Grid/CellSize': '0.05',
        'Grid/3D': 'false',
        'RGBD/LinearUpdate': '0.05',   # add a map node every 5 cm / ~5 deg
        'RGBD/AngularUpdate': '0.09',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'Optimizer/GravitySigma': '0.3',
        # Visual odometry reports very small covariances (0.3 mm std in sim), so
        # every loop-closure correction looked "inconsistent" at the default
        # ratio of 3 and all closures were rejected. Be more tolerant.
        'RGBD/OptimizeMaxError': '10.0',
        'Vis/MinInliers': '15',
        'Rtabmap/DetectionRate': '2',
    })
    if lidar:
        slam_params.update(ICP_2D)
        slam_params.update({
            'Reg/Strategy': '2',                  # loop closures: visual, refined with ICP
            'RGBD/NeighborLinkRefining': 'true',  # correct odometry links with scan matching
            'RGBD/ProximityBySpace': 'true',      # close loops by scan matching nearby nodes
            'Grid/Sensor': '2',                   # grid from lidar AND stereo
            'Grid/RangeMin': '0.2',
        })
    else:
        slam_params['Grid/Sensor'] = '1'          # grid from stereo depth

    nodes = [odom_node]
    for side, out in (('left', 'image_rect_color'), ('right', 'image_rect')):
        if rectify:
            nodes.append(Node(
                package='image_proc', executable='rectify_node', name='rectify_' + side,
                namespace='stereo/' + side,
                remappings=[('image', 'image_raw'), ('image_rect', out)],
                parameters=[{'use_sim_time': use_sim_time}]))
        if a('compressed') == 'true':
            nodes.append(Node(
                package='image_transport', executable='republish', name='decompress_' + side,
                arguments=['compressed', 'raw'],
                remappings=[('in/compressed', 'stereo/{}/image_raw/compressed'.format(side)),
                            ('out', 'stereo/{}/image_raw'.format(side))],
                parameters=[{'use_sim_time': use_sim_time}]))
    fresh = a('fresh_map') == 'true' and not localization
    nodes.append(Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        parameters=[slam_params], remappings=remaps, arguments=['-d'] if fresh else []))
    if a('rtabmap_viz') == 'true':
        nodes.append(Node(package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz',
                          parameters=[slam_params], remappings=remaps))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rectify', default_value='true',
                              description='Run the rectifiers here (real cameras). false in sim.'),
        DeclareLaunchArgument('rect_topics', default_value='false',
                              description='Inputs are already rectified by another launch (truck.launch.py).'),
        DeclareLaunchArgument('lidar', default_value='false',
                              description='Fuse the 2D lidar (scan topic) into the map.'),
        DeclareLaunchArgument('odom', default_value='auto',
                              description="Odometry: 'auto' (lidar if lidar:=true), 'stereo' or 'lidar'."),
        DeclareLaunchArgument('localization', default_value='false',
                              description='true = localize in an existing map, do not extend it.'),
        DeclareLaunchArgument('database', default_value='~/.ros/rtabmap.db'),
        DeclareLaunchArgument('start_at', default_value='origin',
                              description="localization mode: 'origin' (where the map started) or 'last'"),
        DeclareLaunchArgument('fresh_map', default_value='true',
                              description='Delete the database on start (ignored in localization).'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('compressed', default_value='false',
                              description='Inputs arrive as JPEG (stereo/*/image_raw/compressed) '
                                          'over WiFi from the Jetson: decompress them here.'),
        OpaqueFunction(function=_setup),
    ])
