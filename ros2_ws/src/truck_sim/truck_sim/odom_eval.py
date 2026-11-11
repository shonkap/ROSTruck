"""Grade odometry against the simulator's ground truth.

    ros2 run truck_sim odom_eval --ros-args -p use_sim_time:=true -p seconds:=90

Records /odom and /ground_truth/odom while the truck drives, then aligns the
two trajectories with the best 2D rotation + translation (odometry starts in
its own frame) and prints the absolute trajectory error: RMS, max, final, and
how many times odometry was lost (null/repeated poses). Use it to compare
odom:=stereo vs odom:=lidar on the same drive (explore_room).
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def align_2d(src, dst):
    """Best rigid 2D transform (R, t) mapping src points onto dst (Kabsch)."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    ms, md = src.mean(axis=0), dst.mean(axis=0)
    h = (src - ms).T @ (dst - md)
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[1, :] *= -1
        r = vt.T @ u.T
    return r, md - r @ ms


def ate(odom_xy, gt_xy):
    r, t = align_2d(odom_xy, gt_xy)
    err = np.linalg.norm((np.asarray(odom_xy) @ r.T + t) - np.asarray(gt_xy), axis=1)
    return float(np.sqrt((err ** 2).mean())), float(err.max()), float(err[-1])


class OdomEval(Node):
    def __init__(self):
        super().__init__('odom_eval')
        self.declare_parameter('seconds', 90.0)
        self._gt = None
        self.pairs = []
        self.lost = 0
        self._last_odom = None
        self.create_subscription(Odometry, 'ground_truth/odom', self._on_gt, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        self.t0 = None

    def _on_gt(self, msg):
        self._gt = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if q.w == 0 and q.x == 0 and q.y == 0 and q.z == 0:     # rtabmap publishes null when lost
            self.lost += 1
            return
        if self._gt is None:
            return
        self.pairs.append(((p.x, p.y), self._gt))

    def done(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None and self.pairs:
            self.t0 = now
        return self.t0 is not None and now - self.t0 > self.get_parameter('seconds').value


def main(args=None):
    rclpy.init(args=args)
    node = OdomEval()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    pairs = node.pairs
    if len(pairs) < 20:
        print('ODOM_EVAL not enough data ({} samples, lost {})'.format(len(pairs), node.lost))
    else:
        o, g = [p[0] for p in pairs], [p[1] for p in pairs]
        length = sum(math.hypot(g[i][0] - g[i - 1][0], g[i][1] - g[i - 1][1]) for i in range(1, len(g)))
        rms, mx, final = ate(o, g)
        print('ODOM_EVAL path {:.1f} m, ATE rms {:.3f} m, max {:.3f} m, final {:.3f} m, '
              'lost {} msgs, {} samples'.format(length, rms, mx, final, node.lost, len(pairs)))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
