"""Explorer module: the "map_room" task.

    task: {"type": "map_room", "max_minutes": 10, "save_as": "~/maps/room"}

1. warm-up lap: a slow 1 m-radius circle so the forward-looking stereo pair
   sees the room before there's a map to plan on (the truck can't spin in place)
2. frontier exploration: repeatedly drive (via the navigator, nav/goal) to the
   most promising edge between known-free and unknown space
3. done when no reachable frontiers are left (or max_minutes): saves the map
   (PNG + YAML) if save_as is given

Progress = known map area in m^2.
"""
import json
import math
import os

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Empty, String

from truck_tasks.planning import FREE_MAX, approach_point, frontiers, mark_driven, reachable_free
from truck_tasks.ros_common import MapCache, ModuleAnnouncer, PoseTracker


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        p = self.declare_parameter
        p('warmup_speed', 0.3)
        p('warmup_radius', 1.0)
        p('min_frontier_size', 10)      # cells
        p('blacklist_radius', 0.5)      # m around frontiers we failed to reach
        p('robot_radius', 0.20)
        p('min_goal_distance', 0.7)     # m, ignore frontiers right under the truck
        p('standoff', 0.45)             # m, park at least this far from obstacles:
                                        # stereo can't see closer than ~0.25 m, so parking
                                        # with the cameras against furniture loses tracking
        p('approach_max', 1.2)          # m, how far from a frontier we may park to view it
        p('driven_radius', 0.35)        # m, floor around our own path counted as free
        p('save_dir', '')               # when set: map.png/yaml saved here after map_room
        self._map = MapCache(self)
        self._pose = PoseTracker(self)
        self._cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self._goal_pub = self.create_publisher(PoseStamped, 'nav/goal', 10)
        self._cancel_pub = self.create_publisher(Empty, 'nav/cancel', 10)
        self.create_subscription(String, 'nav/state', self._on_nav, 10)
        self.create_subscription(String, 'fc/status', self._on_fc, 10)
        self._task = ModuleAnnouncer(self, 'explorer', ['map_room'], self._on_task, self._abort)
        self._phase = 'idle'
        self._fc_ok = False
        self.create_timer(0.1, self._safe_loop)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_fc(self, msg):
        self._fc_ok = 'relaying_cmd_vel=True' in msg.data

    def _on_task(self, task):
        self._deadline = self._now() + 60.0 * float(task.get('max_minutes', 10))
        sd = self.get_parameter('save_dir').value
        self._save_as = task.get('save_as') or (os.path.join(sd, 'map') if sd else None)
        self._blacklist = []
        self._driven = []           # (x, y) every ~10 cm of our own path
        self._goal = None
        self._warmup_left = 2.0 * math.pi * self.get_parameter('warmup_radius').value / \
            self.get_parameter('warmup_speed').value
        self._phase = 'warmup'
        self.get_logger().info('map_room: warm-up lap ({:.0f} s)'.format(self._warmup_left))

    def _abort(self):
        if self._phase == 'frontier':
            self._cancel_pub.publish(Empty())
        if self._phase != 'idle':
            self._cmd.publish(Twist())
        self._phase = 'idle'

    def _known_area(self):
        gm = self._map.map
        if gm is None:
            return 0.0
        return float(((gm.grid >= 0)).sum()) * gm.res * gm.res

    def _finish(self, ok, why):
        self._abort()
        area = self._known_area()
        msg = '{} - {:.1f} m^2 mapped'.format(why, area)
        if ok and self._save_as and self._map.msg is not None:
            msg += ', saved ' + self._save_map(self._save_as)
        self.get_logger().info('map_room: ' + msg)
        self._task.feedback('succeeded' if ok else 'failed', msg, progress=round(area, 1))

    def _save_map(self, path):
        path = os.path.expanduser(path)
        d = os.path.dirname(path)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        i = self._map.msg.info
        g = np.asarray(self._map.msg.data, dtype=np.int16).reshape(i.height, i.width)
        img = np.full(g.shape, 205, np.uint8)        # nav2 map_server convention
        img[(g >= 0) & (g < FREE_MAX)] = 254
        img[g >= 65] = 0
        cv2.imwrite(path + '.png', np.flipud(img))
        with open(path + '.yaml', 'w') as f:
            f.write('image: {}.png\nmode: trinary\nresolution: {}\norigin: [{}, {}, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'.format(
                        os.path.basename(path), i.resolution, i.origin.position.x, i.origin.position.y))
        return path + '.png'

    def _on_nav(self, msg):
        if self._phase != 'frontier' or self._goal is None:
            return
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        if st.get('state') == 'succeeded':
            # Standing at a frontier doesn't always reveal what's behind it
            # (cameras only look forward); never pick the same one again.
            self._blacklist.append(self._goal)
            self._goal = None
        elif st.get('state') == 'failed':
            self.get_logger().info('frontier ({:.2f}, {:.2f}) unreachable: {}'.format(
                self._goal[0], self._goal[1], st.get('message')))
            self._blacklist.append(self._goal)
            self._goal = None

    def _safe_loop(self):
        try:
            self._loop()
        except Exception as exc:
            import traceback
            self.get_logger().error('explorer error: {}\n{}'.format(exc, traceback.format_exc()))
            if self._phase != 'idle':
                self._finish(False, 'internal error: {}'.format(exc))

    def _track_path(self):
        p = self._pose.pose()
        if p is not None and (not self._driven or
                              math.hypot(p[0] - self._driven[-1][0], p[1] - self._driven[-1][1]) > 0.1):
            self._driven.append((p[0], p[1]))

    def _loop(self):
        if self._phase == 'idle':
            return
        self._track_path()
        if self._now() > self._deadline:
            self._finish(True, 'time limit reached')
            return
        if not self._fc_ok:
            self._task.feedback('running', 'waiting for FC: GUIDED + armed')
            return
        if self._phase == 'warmup':
            v = self.get_parameter('warmup_speed').value
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = v / self.get_parameter('warmup_radius').value
            self._cmd.publish(cmd)
            self._warmup_left -= 0.1
            self._task.feedback('running', 'warm-up lap', progress=round(self._known_area(), 1))
            if self._warmup_left <= 0:
                self._cmd.publish(Twist())
                self._phase = 'frontier'
            return
        # frontier phase
        if self._goal is not None:
            self._task.feedback('running', 'exploring frontier ({:.1f}, {:.1f})'.format(*self._goal),
                                progress=round(self._known_area(), 1))
            return
        gm, pose = self._map.map, self._pose.pose()
        if gm is None or pose is None:
            return
        gm = mark_driven(gm, self._driven + [pose[:2]], self.get_parameter('driven_radius').value)
        rb = self.get_parameter('blacklist_radius').value
        dmin = self.get_parameter('min_goal_distance').value
        rr = self.get_parameter('robot_radius').value
        # Only frontiers reachable through known free space (no islands beyond walls).
        fronts = [f for f in frontiers(gm, self.get_parameter('min_frontier_size').value, rr,
                                       start_xy=pose[:2])
                  if all(math.hypot(f[0] - b[0], f[1] - b[1]) > rb for b in self._blacklist)]
        reach = reachable_free(gm, pose[:2], rr) if fronts else None
        cands = []
        for fx, fy, size in fronts:
            goal = approach_point(gm, (fx, fy), reach, self.get_parameter('standoff').value,
                                  self.get_parameter('approach_max').value)
            if goal is None:
                self._blacklist.append((fx, fy))
                continue
            d = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
            if d < dmin:
                # we're already where we'd park to see it: it won't get better
                self._blacklist.append((fx, fy))
                continue
            cands.append((goal, (fx, fy), size, d))
        if not cands:
            self._finish(True, 'no frontiers left')
            return
        # big and close is best
        goal, front, size, _ = max(cands, key=lambda c: c[2] / (1.0 + c[3]))
        self._goal = front           # blacklisted by frontier position once done
        self._goal_xy = goal
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = goal
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)
        self.get_logger().info('map_room: frontier ({:.2f}, {:.2f}) size {}, parking at ({:.2f}, {:.2f}) '
                               '- {} candidates'.format(front[0], front[1], size, goal[0], goal[1],
                                                        len(cands)))



def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
