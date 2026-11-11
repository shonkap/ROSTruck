"""Shared ROS helpers for task modules. Python 3.6.

THE TASK BUS (all std_msgs/String carrying JSON, so any language/plugin can join):

  tasks/submit    in   a task {"type": ..., ...}            (web, LLM, detector, CLI)
  tasks/cancel    in   "all" or a task id
  tasks/status    out  queue, active task, history, modules (transient local, 1 Hz)
  tasks/dispatch  out  the task a module should start now ("handler" = module name)
  tasks/abort     out  {"id": n} stop working on that task
  tasks/feedback  in   {"id": n, "state": running|succeeded|failed, "message", "progress"}
  tasks/modules   in   {"name": "...", "handles": ["goto", ...]}  every 2 s (registration)
  tasks/requests  out  natural-language requests ({"type":"request","text":...}) for
                       planner plugins (LLM); they answer on tasks/reports and may
                       submit tasks themselves
  tasks/reports   in   {"request_id": n, "title": "...", "text": "...", "tasks": [...]}

  CLI:  ros2 topic pub --once /tasks/submit std_msgs/msg/String "{data: '{\\"type\\": \\"map_room\\"}'}"
"""
import json
import math
import time

from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String

from truck_tasks.planning import GridMap

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def jmsg(obj):
    return String(data=json.dumps(obj, default=str))


def jparse(msg):
    try:
        return json.loads(msg.data)
    except (ValueError, TypeError):
        return None


class MapCache(object):
    """Latest /map as a GridMap."""

    def __init__(self, node, topic='map'):
        self.map = None
        self.msg = None
        node.create_subscription(OccupancyGrid, topic, self._on_map, LATCHED)

    def _on_map(self, msg):
        i = msg.info
        if i.width == 0 or i.height == 0:
            return
        self.msg = msg
        self.map = GridMap(msg.data, i.width, i.height, i.resolution,
                           i.origin.position.x, i.origin.position.y)


class PoseTracker(object):
    """Robot pose (x, y, yaw) in the map frame from TF."""

    def __init__(self, node, map_frame='map', base_frame='base_link', max_age=1.0):
        import tf2_ros
        self._node = node
        self._buf = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buf, node)
        self._map, self._base = map_frame, base_frame
        self._max_age = max_age

    def pose(self):
        """(x, y, yaw), or None if unknown or older than max_age seconds - e.g.
        visual odometry lost tracking (cameras too close to a wall)."""
        try:
            t = self._buf.lookup_transform(self._map, self._base, Time(), timeout=Duration(seconds=0.0))
        except Exception:
            return None
        stamp = Time.from_msg(t.header.stamp)
        age = (self._node.get_clock().now() - stamp).nanoseconds * 1e-9
        if self._max_age and age > self._max_age:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw


class ModuleAnnouncer(object):
    """Registers a module with the task manager and filters dispatches for it."""

    def __init__(self, node, name, handles, on_start, on_abort):
        self.name, self.handles = name, list(handles)
        self.active_id = None
        self._last_running = 0.0
        self._on_start, self._on_abort = on_start, on_abort
        self._pub_mod = node.create_publisher(String, 'tasks/modules', 10)
        self._pub_fb = node.create_publisher(String, 'tasks/feedback', 10)
        node.create_subscription(String, 'tasks/dispatch', self._on_dispatch, 10)
        node.create_subscription(String, 'tasks/abort', self._on_abort_msg, 10)
        node.create_timer(2.0, self._announce)
        self._announce()

    def _announce(self):
        self._pub_mod.publish(jmsg({'name': self.name, 'handles': self.handles}))

    def _on_dispatch(self, msg):
        t = jparse(msg)
        if not t or t.get('handler') != self.name or t.get('type') not in self.handles:
            return
        self.active_id = t['id']
        self.feedback('running', 'started')
        self._on_start(t)

    def _on_abort_msg(self, msg):
        d = jparse(msg)
        if d and d.get('id') == self.active_id:
            self.active_id = None
            self._on_abort()

    def feedback(self, state, message='', progress=None):
        if self.active_id is None:
            return
        if state == 'running':      # progress updates at most 1 Hz
            if time.time() - self._last_running < 1.0:
                return
            self._last_running = time.time()
        self._pub_fb.publish(jmsg({'id': self.active_id, 'state': state,
                                   'message': message, 'progress': progress}))
        if state in ('succeeded', 'failed', 'canceled'):
            self.active_id = None
