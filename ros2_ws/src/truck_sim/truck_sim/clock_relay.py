"""Republish Gazebo's /clock at a sane rate.

Gazebo publishes the clock every physics step (1 kHz). Every ROS node with
use_sim_time subscribes to /clock, so each Python node was burning ~30% of a
core just receiving clock messages. This node is the only 1 kHz subscriber
(gz/clock from the bridge) and republishes /clock at `rate_hz` (default 100 Hz:
10 ms timer resolution, plenty for 10-20 Hz control loops).
Sim only - the real truck runs on wall time.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class ClockRelay(Node):
    def __init__(self):
        super().__init__('clock_relay')
        self.declare_parameter('rate_hz', 100.0)
        self._period_ns = int(1e9 / self.get_parameter('rate_hz').value)
        self._last_ns = -1
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(Clock, '/clock', 10)
        self.create_subscription(Clock, 'gz/clock', self._on_clock, qos)

    def _on_clock(self, msg):
        ns = msg.clock.sec * 1000000000 + msg.clock.nanosec
        if ns < self._last_ns or ns - self._last_ns >= self._period_ns:   # (< : sim reset)
            self._last_ns = ns
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClockRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
