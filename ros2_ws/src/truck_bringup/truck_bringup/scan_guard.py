"""Lidar collision guard: 360 deg, so it also covers REVERSING. Python 3.6.

    ros2 run truck_bringup scan_guard

From the 2D lidar scan it finds the nearest obstacle in the truck's driving
corridor in front of the front bumper and behind the rear bumper, and
publishes speed caps that fc_bridge applies to every cmd_vel:

    safety/lidar/forward_limit   std_msgs/Float32  m/s (99 = no limit)
    safety/lidar/reverse_limit   std_msgs/Float32  m/s (magnitude; 99 = no limit)
    safety/lidar/front_distance  std_msgs/Float32  m from the front bumper (99 = clear)
    safety/rear_distance         std_msgs/Float32  m from the rear bumper (99 = clear)

The stereo collision_guard still matters: the lidar only sees its own scan
plane (260 mm up), not low things like a shoe or a cable.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

NO_LIMIT = 99.0


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max, mount):
    """LaserScan -> Nx2 points in base_link. mount = (x, y, yaw) of the laser."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r >= max(range_min, 0.01)) & (r <= range_max)
    r, a = r[ok], a[ok] + mount[2]
    return np.c_[mount[0] + r * np.cos(a), mount[1] + r * np.sin(a)]


def corridor_distances(points, front_x, rear_x, half_width, min_points=2):
    """Nearest obstacle ahead of the front bumper (x > front_x) and behind the
    rear bumper (x < rear_x) within |y| <= half_width. Points inside the
    truck's own footprint (hits on the truck / its mast) are ignored.
    Needs min_points hits so a single noisy return doesn't stop the truck."""
    if len(points) == 0:
        return None, None
    x, y = points[:, 0], points[:, 1]
    lane = np.abs(y) <= half_width
    ahead = np.sort(x[lane & (x > front_x)] - front_x)
    behind = np.sort(rear_x - x[lane & (x < rear_x)])
    front = float(ahead[min_points - 1]) if len(ahead) >= min_points else None
    rear = float(behind[min_points - 1]) if len(behind) >= min_points else None
    return front, rear


def limit_for(distance, stop, slow, max_speed):
    if distance is None or distance >= slow:
        return NO_LIMIT
    if distance <= stop:
        return 0.0
    return max_speed * (distance - stop) / (slow - stop)


class ScanGuard(Node):
    def __init__(self):
        super().__init__('scan_guard')
        p = self.declare_parameter
        p('front_x', 0.25)          # m, front bumper ahead of base_link
        p('rear_x', -0.25)          # m, rear bumper
        p('half_width', 0.20)       # m, corridor half width (truck ~0.31 m wide + margin)
        p('stop_distance', 0.20)    # m from the bumper
        p('slow_distance', 0.70)
        p('max_speed', 1.0)
        p('laser_frame', 'laser')
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._g = g
        self._mount = None
        import tf2_ros
        self._tf = tf2_ros.Buffer()
        self._tfl = tf2_ros.TransformListener(self._tf, self)
        self._pubs = {n: self.create_publisher(Float32, n, 10) for n in (
            'safety/lidar/forward_limit', 'safety/lidar/reverse_limit',
            'safety/lidar/front_distance', 'safety/rear_distance')}
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)

    def _mount_pose(self, frame):
        if self._mount is None:
            try:
                t = self._tf.lookup_transform('base_link', frame, Time())
            except Exception:
                return None
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self._mount = (t.transform.translation.x, t.transform.translation.y, yaw)
            self.get_logger().info('laser mount in base_link: x {:.2f} y {:.2f} yaw {:.2f}'.format(*self._mount))
        return self._mount

    def _on_scan(self, msg):
        g = self._g
        mount = self._mount_pose(msg.header.frame_id or g('laser_frame'))
        if mount is None:
            return
        pts = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                             msg.range_max, mount)
        front, rear = corridor_distances(pts, g('front_x'), g('rear_x'), g('half_width'))
        stop, slow, vmax = g('stop_distance'), g('slow_distance'), g('max_speed')
        vals = {
            'safety/lidar/forward_limit': limit_for(front, stop, slow, vmax),
            'safety/lidar/reverse_limit': limit_for(rear, stop, slow, vmax),
            'safety/lidar/front_distance': NO_LIMIT if front is None else front,
            'safety/rear_distance': NO_LIMIT if rear is None else rear,
        }
        for name, v in vals.items():
            self._pubs[name].publish(Float32(data=float(v)))


def main(args=None):
    rclpy.init(args=args)
    node = ScanGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
