"""Launch pieces shared by the truck (truck.launch.py) and the sim (sim.launch.py).

Both describe the same rig (rig.yaml: base_link height, camera and lidar
mounts), so the TF tree and the sensor-side safety nodes are built here once.
"""
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

OPTICAL = (-1.5707963, 0.0, -1.5707963)    # camera link (x forward) -> optical (z forward)


def all_true(*names):
    """Condition: every one of these launch args is 'true'."""
    expr = []
    for i, n in enumerate(names):
        expr += [(' and ' if i else '') + "'", LaunchConfiguration(n), "' == 'true'"]
    return IfCondition(PythonExpression(expr))



def rectify_container(sides, params=None, condition=None):
    """Stereo rectification: image_proc::RectifyNode per side, in one container.

    sides: [(side, output_topic)], e.g. [('left', 'image_rect_color')] - reads
    stereo/<side>/image_raw (+camera_info), publishes stereo/<side>/<output>.
    A component, not the rectify_node executable: the Jetson's from-source
    Humble (dustynv r32.7.1) only ships image_proc as components.
    """
    return ComposableNodeContainer(
        name='rectify_container', namespace='', package='rclcpp_components',
        executable='component_container', output='screen', condition=condition,
        parameters=[params or {}],
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc', plugin='image_proc::RectifyNode',
                name='rectify_' + side, namespace='stereo/' + side,
                remappings=[('image', 'image_raw'), ('image_rect', out)],
                parameters=[params or {}])
            for side, out in sides])


def static_tf(name, parent, child, xyz, rpy=(0, 0, 0), sim=False):
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                   '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                   '--frame-id', parent, '--child-frame-id', child],
        parameters=[{'use_sim_time': sim}])


def rig_tfs(rig, sim=False):
    """base_link -> FC, lidar, both cameras and their optical frames."""
    half_b = rig['baseline'] / 2.0
    cam_y, pitch = rig.get('cam_y', 0.0), rig['cam_pitch']
    return [
        static_tf('tf_fc', 'base_link', 'fc_link',
                  (rig.get('fc_x', 0.0), rig.get('fc_y', 0.0), rig.get('fc_z', 0.0)), sim=sim),
        static_tf('tf_laser', 'base_link', 'laser',
                  (rig['lidar_x'], rig.get('lidar_y', 0.0), rig['lidar_z']),
                  (0, 0, rig.get('lidar_yaw', 0.0)), sim=sim),
        static_tf('tf_left_cam', 'base_link', 'left_camera_link',
                  (rig['cam_x'], cam_y + half_b, rig['cam_z']), (0, pitch, 0), sim=sim),
        static_tf('tf_right_cam', 'base_link', 'right_camera_link',
                  (rig['cam_x'], cam_y - half_b, rig['cam_z']), (0, pitch, 0), sim=sim),
        static_tf('tf_left_opt', 'left_camera_link', 'left_camera_optical_frame',
                  (0, 0, 0), OPTICAL, sim=sim),
        static_tf('tf_right_opt', 'right_camera_link', 'right_camera_optical_frame',
                  (0, 0, 0), OPTICAL, sim=sim),
    ]


def safety_and_detection(rig, left_topic, right_topic, sim=False, detector_params=None, output='screen'):
    """Stereo + lidar collision guards and the object detector, configured
    from the rig (camera height/pitch, where the bumper is)."""
    front_x = rig.get('front_x', 0.25)          # front bumper ahead of base_link
    cam = {'use_sim_time': sim, 'left_topic': left_topic, 'right_topic': right_topic,
           'cam_height': rig['base_link_z'] + rig['cam_z'], 'cam_pitch': rig['cam_pitch']}
    # obstacles: 5 cm up to the truck's own height + margin - it fits under a table
    guard = dict(cam, cam_x=rig['cam_x'], front_x=front_x, max_height=rig.get('truck_height', 0.32) + 0.05)
    return [
        Node(package='truck_bringup', executable='collision_guard', name='collision_guard',
             output=output, condition=IfCondition(LaunchConfiguration('guard')),
             parameters=[guard]),
        Node(package='truck_bringup', executable='scan_guard', name='scan_guard', output=output,
             condition=all_true('lidar', 'guard'), parameters=[{'use_sim_time': sim, 'front_x': front_x}]),
        Node(package='truck_bringup', executable='detector_node', name='detector', output=output,
             condition=IfCondition(LaunchConfiguration('detector')),
             parameters=[dict(cam, **(detector_params or {}))]),
    ]
