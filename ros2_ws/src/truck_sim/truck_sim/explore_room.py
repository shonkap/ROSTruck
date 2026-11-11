"""Drives a mapping pattern in the sim room through cmd_vel (-> fc_bridge -> SITL).

Pattern: slow circles in both directions plus a figure-eight around the
room centre, so the forward-facing stereo pair sweeps every wall from a few
viewpoints and revisits places (gives RTAB-Map loop closures).

Uses only what the real truck has (cmd_vel out), and the sim's ground-truth
odometry purely to *grade* SLAM at the end:

    ros2 run truck_sim explore_room            # needs: sim_ctl guided arm
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

# Circles of radius V / W = 1 m, one full lap each (2*pi / W seconds), so the
# truck returns to the start between laps and stays clear of the furniture.
V, W = 0.35, 0.35
LAP = 2.0 * math.pi / W
# (duration_s, linear m/s, angular rad/s)
PATTERN = [
    (3.0, 0.0, 0.0),        # settle
    (LAP, V, W),            # CCW lap (left of start)
    (LAP, V, -W),           # CW lap (right of start) -> figure eight
    (LAP, V, W),            # CCW again: revisit -> loop closures
    (4.0, 0.0, 0.0),
]


class ExploreRoom(Node):
    def __init__(self):
        super().__init__('explore_room')
        self.declare_parameter('speed_scale', 1.0)
        self._scale = float(self.get_parameter('speed_scale').value)
        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._gt = None
        self._slam = None
        self._slam_odom = None
        self._status = ''
        self.create_subscription(Odometry, 'ground_truth/odom', self._on_gt, qos_profile_sensor_data)
        # rtabmap publishes odom best-effort (qos 2 in slam.launch.py)
        self.create_subscription(Odometry, 'odom', self._on_slam_odom, qos_profile_sensor_data)
        self.create_subscription(String, 'fc/status', self._on_status, 10)
        self._step = 0
        self._step_start = None
        self._gt_start = None
        self._odom_start = None
        self._max_err = 0.0
        self.done = False
        self.create_timer(0.05, self._tick)
        self.create_timer(5.0, self._report)

    def _on_gt(self, msg):
        self._gt = msg.pose.pose.position

    def _on_slam_odom(self, msg):
        self._slam_odom = msg.pose.pose.position

    def _on_status(self, msg):
        self._status = msg.data

    def _tick(self):
        if 'relaying_cmd_vel=True' not in self._status:
            if self._step_start is None:
                self.get_logger().info(
                    'Waiting for fc_bridge to relay (run: ros2 run truck_sim sim_ctl guided arm)',
                    throttle_duration_sec=10.0)
                return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._step_start is None:
            self._step_start = now
            self.get_logger().info('Starting mapping pattern ({:.0f} s)'.format(
                sum(p[0] for p in PATTERN) / self._scale))
        dur, lin, ang = PATTERN[self._step]
        if now - self._step_start > dur / self._scale:
            self._step += 1
            self._step_start = now
            if self._step >= len(PATTERN):
                self._pub.publish(Twist())
                self._finish()
                return
            dur, lin, ang = PATTERN[self._step]
            self.get_logger().info('step {}/{}: v={:.2f} m/s w={:+.2f} rad/s'.format(
                self._step + 1, len(PATTERN), lin * self._scale, ang * self._scale))
        cmd = Twist()
        cmd.linear.x = lin * self._scale
        cmd.angular.z = ang * self._scale
        self._pub.publish(cmd)

    def _drift(self):
        """Distance between visual odometry and ground truth, both relative to start."""
        if self._gt is None or self._slam_odom is None:
            return None
        if self._gt_start is None:
            self._gt_start = (self._gt.x, self._gt.y)
            self._odom_start = (self._slam_odom.x, self._slam_odom.y)
        gx, gy = self._gt.x - self._gt_start[0], self._gt.y - self._gt_start[1]
        ox, oy = self._slam_odom.x - self._odom_start[0], self._slam_odom.y - self._odom_start[1]
        # Ground truth is in the Gazebo world frame, VO starts at 0 facing the
        # truck's initial heading (+x in both, since the truck spawns facing +x).
        return math.hypot(gx - ox, gy - oy), math.hypot(gx, gy)

    def _report(self):
        d = self._drift()
        if d is None:
            self.get_logger().info('no odom yet (gt={} vo={})'.format(
                self._gt is not None, self._slam_odom is not None))
            return
        self._max_err = max(self._max_err, d[0])
        self.get_logger().info('visual odom error {:.2f} m (truck {:.2f} m from start)'.format(*d))

    def _finish(self):
        d = self._drift()
        self.get_logger().info('Pattern done. Final VO error {}, max {:.2f} m'.format(
            'n/a' if d is None else '{:.2f} m'.format(d[0]), self._max_err))
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = ExploreRoom()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node._pub.publish(Twist())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
