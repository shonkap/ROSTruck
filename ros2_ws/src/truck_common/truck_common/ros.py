"""ROS plumbing used by every node. Python 3.6."""
import json
import math

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String

# "Latched": late subscribers get the last message (maps, task status, places).
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

# Camera images/camera_info publishers. RELIABLE because image_proc's
# RectifyNode (and rtabmap) subscribe reliable and ignore a best-effort
# publisher; best-effort subscribers (e.g. over WiFi) still connect to it.
CAMERA = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)


def jmsg(obj):
    return String(data=json.dumps(obj))


def jparse(msg):
    """JSON payload of a std_msgs/String, or None if it isn't JSON."""
    try:
        return json.loads(msg.data)
    except (ValueError, TypeError):
        return None


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rpy_from_quat(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return roll, pitch, yaw_from_quat(q)


def run_node(make, args=None, until=None, on_interrupt=None, on_exit=None):
    """The usual main(): init, build the node, spin, clean up.

    until(node) -> True stops the loop (one-shot tools); on_interrupt(node)
    runs on Ctrl-C (e.g. send a stop); on_exit(node) always runs last."""
    rclpy.init(args=args)
    node = make()
    try:
        if until is None:
            rclpy.spin(node)
        else:
            while rclpy.ok() and not until(node):
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if on_interrupt is not None:
            on_interrupt(node)
    finally:
        if on_exit is not None:
            on_exit(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


class Tf(object):
    """A TF buffer + listener with the lookups the truck needs."""

    def __init__(self, node):
        import tf2_ros
        self._node = node
        self.buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self.buffer, node)

    def lookup(self, target, source, stamp=None):
        """TransformStamped (at `stamp`, else the latest), or None."""
        try:
            return self.buffer.lookup_transform(target, source, stamp if stamp is not None else Time())
        except Exception:
            return None

    def age(self, t):
        return (self._node.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9

    def pose2d(self, target, source, max_age=None):
        """(x, y, yaw) of `source` in `target`, or None if unknown / older than max_age s."""
        t = self.lookup(target, source)
        if t is None or (max_age and self.age(t) > max_age):
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_from_quat(t.transform.rotation)


class PoseTracker(Tf):
    """Robot pose (x, y, yaw) in the map frame. None when unknown or stale -
    e.g. visual odometry lost tracking (cameras too close to a wall)."""

    def __init__(self, node, map_frame='map', base_frame='base_link', max_age=1.0):
        Tf.__init__(self, node)
        self._frames = (map_frame, base_frame)
        self._max_age = max_age

    def pose(self):
        return self.pose2d(self._frames[0], self._frames[1], self._max_age)
