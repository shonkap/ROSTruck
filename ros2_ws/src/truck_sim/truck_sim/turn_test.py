"""Measure how well the FC follows cmd_vel turn-rate requests.

Drives short constant-speed arcs and compares commanded vs. measured yaw rate
(from fc/imu, so it works on the real truck too - wheels on the ground, lots
of space!). Prints the ratio and a suggested ATC_STR_RAT_FF.

    ros2 run truck_sim turn_test                       # sim (needs: sim_ctl guided arm)
    ros2 run truck_sim turn_test --ros-args -p speed:=0.3 -p seconds:=3.0
    # exercise fc_bridge's steering monitor: straights, turns, and max-lock turns
    ros2 run truck_sim turn_test --ros-args -p "rates:=[0.0, 0.35, 0.0, -0.35, 1.5, -1.5]" -p repeat:=4

A rate of 0.0 drives straight (no result line). Rates the truck can't reach
(e.g. 1.5) show where the steering runs out on each side.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class TurnTest(Node):
    def __init__(self):
        super().__init__('turn_test')
        self.declare_parameter('speed', 0.35)
        self.declare_parameter('rates', [0.35, -0.35, 0.6, -0.6])
        self.declare_parameter('seconds', 4.0)
        self.declare_parameter('current_ff', 0.0)   # ATC_STR_RAT_FF now, for the suggestion
        self.declare_parameter('repeat', 1)
        self._v = float(self.get_parameter('speed').value)
        self._rates = list(self.get_parameter('rates').value) * int(self.get_parameter('repeat').value)
        self._secs = float(self.get_parameter('seconds').value)
        self._ff = float(self.get_parameter('current_ff').value)
        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Imu, 'fc/imu', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(String, 'fc/status', self._on_status, 10)
        self._status = ''
        self._yaw_rates = []
        self._i = 0
        self._t0 = None
        self._results = []
        self.done = False
        self.create_timer(0.05, self._tick)

    def _on_status(self, msg):
        self._status = msg.data

    def _on_imu(self, msg):
        if self._t0 is not None:
            self._yaw_rates.append((self._now(), msg.angular_velocity.z))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        if 'relaying_cmd_vel=True' not in self._status:
            self.get_logger().info('waiting for GUIDED + armed (sim: sim_ctl guided arm)',
                                   throttle_duration_sec=5.0)
            return
        now = self._now()
        if self._t0 is None:
            self._t0, self._yaw_rates = now, []
        if now - self._t0 > self._secs:
            # average over the second half (after the controller settles)
            half = self._t0 + self._secs / 2.0
            vals = [w for t, w in self._yaw_rates if t > half]
            got = sum(vals) / len(vals) if vals else 0.0
            want = self._rates[self._i]
            if want:
                self._results.append((want, got))
                self.get_logger().info('commanded {:+.2f} rad/s -> measured {:+.2f} rad/s ({:.0f}%)'.format(
                    want, got, 100.0 * got / want))
            else:
                self.get_logger().info('straight: measured {:+.3f} rad/s (should be ~0)'.format(got))
            self._i += 1
            self._t0 = None
            if self._i >= len(self._rates):
                self._pub.publish(Twist())
                self._finish()
            return
        cmd = Twist()
        cmd.linear.x = self._v
        cmd.angular.z = self._rates[self._i]
        self._pub.publish(cmd)

    def _finish(self):
        ratios = [g / w for w, g in self._results if w]
        r = sum(ratios) / len(ratios) if ratios else 0.0
        self.get_logger().info('mean achieved/commanded = {:.2f}'.format(r))
        if 0.85 <= r <= 1.15:
            self.get_logger().info('turn-rate tracking is fine')
        elif self._ff > 0 and r > 0.05:
            self.get_logger().info('suggest ATC_STR_RAT_FF ~ {:.2f} (now {:.2f})'.format(
                min(2.0, self._ff / r), self._ff))
        else:
            self.get_logger().info('tracking off - raise ATC_STR_RAT_FF if < 1, lower it if > 1')
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = TurnTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node._pub.publish(Twist())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
