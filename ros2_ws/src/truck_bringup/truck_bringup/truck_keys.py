"""Drive the truck from a terminal like a car. Python 3.6 (works over SSH on the Jetson).

    ros2 run truck_bringup truck_keys

    w / s     faster / slower (below 0 = reverse)
    a / d     steer more left / right   (steering holds when you let go)
    c         centre the steering
    space, k  stop (speed 0, steering centred)
    q, Ctrl-C quit (sends a stop)

Why not teleop_twist_keyboard: it publishes once per key press, but fc_bridge
stops the truck if cmd_vel goes quiet for 0.5 s (dead-man), so the wheels kept
snapping back to straight. This keeps the current speed/steering and re-sends
it 10x per second while the program runs; quitting or closing the terminal
stops the truck within 0.5 s - also when the terminal just goes away (closed
window, dropped SSH, `docker exec` session ended): end of input, a hang-up
signal or a dead parent shell all count as quitting. Its j/l also mean
"rotate in place", which a car can't do. Needs the FC in GUIDED + armed; the
collision guards still apply.
"""
import os
import select
import signal
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CarControls(object):
    """Key -> (speed, curvature). Steering is a curvature (1/m), so the path
    stays the same when the speed changes and nothing turns while stopped."""

    def __init__(self, speed_step=0.1, max_speed=0.6, max_reverse=0.3,
                 curv_step=0.5, min_turn_radius=0.35):
        self.v, self.k = 0.0, 0.0
        self.speed_step, self.max_speed, self.max_reverse = speed_step, max_speed, max_reverse
        self.curv_step, self.max_curv = curv_step, 1.0 / min_turn_radius

    def key(self, ch):
        """Returns False to quit."""
        if ch in ('w', 'W'):
            self.v = min(self.max_speed, round(self.v + self.speed_step, 2))
        elif ch in ('s', 'S'):
            self.v = max(-self.max_reverse, round(self.v - self.speed_step, 2))
        elif ch in ('a', 'A'):
            self.k = min(self.max_curv, round(self.k + self.curv_step, 2))
        elif ch in ('d', 'D'):
            self.k = max(-self.max_curv, round(self.k - self.curv_step, 2))
        elif ch in ('c', 'C'):
            self.k = 0.0
        elif ch in (' ', 'k', 'K'):
            self.v, self.k = 0.0, 0.0
        elif ch in ('q', 'Q', '\x03'):
            self.v, self.k = 0.0, 0.0
            return False
        return True

    def twist(self):
        t = Twist()
        t.linear.x = self.v
        t.angular.z = self.k * self.v      # yaw rate = curvature x speed
        return t

    def status(self):
        steer = 'straight' if self.k == 0 else '{} r={:.2f} m'.format(
            'left' if self.k > 0 else 'right', 1.0 / abs(self.k))
        return 'speed {:+.2f} m/s   steering {:<16s}'.format(self.v, steer)


def main(args=None):
    rclpy.init(args=args)
    node = Node('truck_keys')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    car = CarControls()
    if not sys.stdin.isatty():
        print('truck_keys needs an interactive terminal (ros2 run truck_bringup truck_keys)')
        return 1
    print(__doc__.split('\n\n')[1])
    old = termios.tcgetattr(sys.stdin)
    parent = os.getppid()

    def hang_up(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGHUP, hang_up)        # terminal closed
    signal.signal(signal.SIGTERM, hang_up)
    try:
        tty.setcbreak(sys.stdin.fileno())
        last = None
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)     # 10 Hz re-send
            if ready:
                ch = sys.stdin.read(1)
                if ch == '' or not car.key(ch):       # '' = end of input: terminal is gone
                    break
            if os.getppid() != parent:                # our shell died (orphaned)
                break
            pub.publish(car.twist())
            s = car.status()
            if s != last:
                sys.stdout.write('\r' + s)
                sys.stdout.flush()
                last = s
    except KeyboardInterrupt:
        pass
    finally:
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        except termios.error:
            pass                                      # no terminal any more
        for _ in range(3):
            pub.publish(Twist())
        print('\nstopped.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
