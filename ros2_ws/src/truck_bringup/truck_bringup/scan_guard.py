"""Lidar collision guard: 360 deg, so it also covers REVERSING. Python 3.6.

    ros2 run truck_bringup scan_guard

Nearest obstacle in the truck's lane ahead of the front bumper and behind
the rear bumper, from the 2D scan, published on safety/lidar/* (see
truck_common/guards.py). The lidar only sees its own scan plane (260 mm up):
low things are left to the stereo and map guards.
"""
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2

from truck_common.clouds import xy_to_cloud
from truck_common.guards import GuardPublisher, corridor_distances
from truck_common.ros import Tf, run_node


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max, mount):
    """LaserScan -> Nx2 points in base_link. mount = (x, y, yaw) of the laser."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r >= max(range_min, 0.01)) & (r <= range_max)
    r, a = r[ok], a[ok] + mount[2]
    return np.c_[mount[0] + r * np.cos(a), mount[1] + r * np.sin(a)]


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
        self._g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._tf = Tf(self)
        self._mount = None
        self._out = GuardPublisher(self, 'lidar')
        self._points = self.create_publisher(PointCloud2, 'safety/lidar/points', 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg):
        g = self._g
        if self._mount is None:     # static: look it up once
            self._mount = self._tf.pose2d('base_link', msg.header.frame_id or g('laser_frame'))
            if self._mount is None:
                return
            self.get_logger().info('laser mount in base_link: x {:.2f} y {:.2f} yaw {:.2f}'.format(*self._mount))
        pts = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                             msg.range_max, self._mount)
        front, rear = corridor_distances(pts, g('front_x'), g('rear_x'), g('half_width'))
        self._points.publish(xy_to_cloud(pts, 'base_link', msg.header.stamp))
        self._out.publish(front, rear, g('stop_distance'), g('slow_distance'), g('max_speed'))


def main(args=None):
    return run_node(ScanGuard, args)
