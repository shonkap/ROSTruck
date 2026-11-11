"""Navigator module: drives to points on the SLAM map. Handles task type "goto".

    task: {"type": "goto", "waypoints": [[x, y] or "place name", ...], "speed": 0.3}

Also takes single goals from other modules (the explorer) on nav/goal
(geometry_msgs/PoseStamped, map frame) and reports on nav/state
(String JSON {"state": "idle|driving|succeeded|failed", "message"}).
nav/cancel (std_msgs/Empty) stops it.

Plans with A* on /map (walls inflated by the truck's radius) and follows the
path with pure pursuit. This truck is car-like (front + rear steer, no skid
steering, can't turn on the spot), so the turn rate is always capped at
speed / min_turn_radius; the radius is raised automatically if fc_bridge's
steering monitor learns that the truck can't turn as tight as configured.
"""
import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, String

from truck_tasks.planning import (ThreePointTurn, heading_error, plan, pure_pursuit,
                                  speed_for_curvature)
from truck_tasks.places_node import normalize, resolve_waypoints
from truck_tasks.ros_common import LATCHED, MapCache, ModuleAnnouncer, PoseTracker, jmsg, jparse


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        p = self.declare_parameter
        p('speed', 0.3)                 # m/s in curves (and the base speed)
        p('max_speed', 0.5)             # m/s on straight path sections
        p('lookahead', 0.45)            # m
        p('goal_tolerance', 0.25)       # m
        p('robot_radius', 0.20)         # m, half the truck's width + margin
        p('min_turn_radius', 0.35)      # m, both directions (fc_bridge may raise it)
        p('replan_period', 2.0)         # s
        p('stuck_timeout', 12.0)        # s without getting closer -> back up + replan, then fail
        p('backup_distance', 0.5)       # m to reverse when stuck (car-like: can't pivot out)
        p('backup_speed', 0.2)          # m/s
        p('lost_timeout', 30.0)         # s without a pose (VO lost) before the task fails
        p('turnaround_start', 1.75)     # rad (100 deg): path behind us -> 3-point turn
        p('turnaround_done', 1.0)       # rad (57 deg): back to normal path following
        p('turnaround_max_s', 25.0)     # give up turning after this long (stuck logic takes over)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._speed = g('speed')
        self._lookahead = g('lookahead')
        self._tol = g('goal_tolerance')
        self._radius = g('robot_radius')
        self._min_r = {'left': g('min_turn_radius'), 'right': g('min_turn_radius')}
        self._replan = g('replan_period')
        self._stuck_timeout = g('stuck_timeout')

        self._map = MapCache(self)
        self._pose = PoseTracker(self)
        self._cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self._path_pub = self.create_publisher(Path, 'nav/path', 10)
        self._state_pub = self.create_publisher(String, 'nav/state', 10)
        self.create_subscription(PoseStamped, 'nav/goal', self._on_goal, 10)
        self.create_subscription(Empty, 'nav/cancel', lambda m: self._stop('canceled'), 10)
        self.create_subscription(String, 'fc/status', self._on_fc_status, 10)
        self.create_subscription(String, 'fc/steering_report', self._on_steering, 10)
        # nearest obstacle ahead from the collision guard (>= 50 = nothing seen)
        self._front = None
        self.create_subscription(Float32, 'safety/obstacle_distance', self._on_front, 10)
        # behind the rear bumper, from the lidar guard (>= 50 = nothing / no lidar)
        self._rear = None
        self.create_subscription(Float32, 'safety/rear_distance', self._on_rear, 10)
        self._task = ModuleAnnouncer(self, 'navigator', ['goto'], self._on_task,
                                     lambda: self._stop('aborted'))

        self._places = {}
        self.create_subscription(String, 'places', self._on_places, LATCHED)
        self._goals = []          # remaining waypoints [(x, y)]
        self._from_task = False
        self._path = None
        self._last_plan = -math.inf
        self._best_dist = math.inf
        self._best_time = 0.0
        self._replans_stuck = 0
        self._fc_ok = False
        self._fc_status = ''
        self._backup_until = 0.0
        self._lost_since = None
        self._last_lost_backup = -math.inf
        self._turning_since = None
        self.create_timer(0.1, self._safe_loop)

    # ------------------------------------------------------------- inputs
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_rear(self, msg):
        self._rear = None if msg.data >= 50.0 else float(msg.data)

    def _on_front(self, msg):
        self._front = None if msg.data >= 50.0 else float(msg.data)

    def _on_fc_status(self, msg):
        self._fc_status = msg.data
        self._fc_ok = 'relaying_cmd_vel=True' in msg.data

    def _on_steering(self, msg):
        try:
            rep = json.loads(msg.data)
        except ValueError:
            return
        for side in ('left', 'right'):
            r = (rep.get(side) or {}).get('min_radius')
            if r and r > self._min_r[side]:
                self.get_logger().info('{} turns limited to radius {:.2f} m (learned by fc_bridge)'.format(side, r))
                self._min_r[side] = r

    def _on_places(self, msg):
        d = jparse(msg)
        if isinstance(d, list):
            self._places = {normalize(p['name']): p for p in d if 'name' in p}

    def _on_task(self, task):
        pts = task.get('waypoints') or []
        try:
            self._goals = resolve_waypoints(pts, self._places)
        except ValueError as exc:
            self._task.feedback('failed', str(exc))
            return
        if not self._goals:
            self._task.feedback('failed', 'no waypoints')
            return
        self._speed = float(task.get('speed', self.get_parameter('speed').value))
        self._from_task = True
        self._start_leg()

    def _on_goal(self, msg):
        self._goals = [(msg.pose.position.x, msg.pose.position.y)]
        self._from_task = False
        self._speed = self.get_parameter('speed').value
        self._start_leg()

    def _start_leg(self):
        self._path = None
        self._last_plan = -math.inf
        self._best_dist = math.inf
        self._best_time = self._now()
        self._replans_stuck = 0
        self._state('driving', 'to ({:.2f}, {:.2f})'.format(*self._goals[0]))

    # --------------------------------------------------------------- loop
    def _state(self, state, message=''):
        self._state_pub.publish(jmsg({'state': state, 'message': message,
                                      'goals': self._goals}))

    def _stop(self, why, ok=False):
        if self._goals:
            self._cmd.publish(Twist())
        self._goals = []
        self._path = None
        self._state('succeeded' if ok else 'failed' if why not in ('canceled', 'aborted') else 'idle', why)
        if self._from_task:
            self._task.feedback('succeeded' if ok else 'failed', why)
        self._from_task = False

    def _safe_loop(self):
        # A bug must fail the current task, not kill the module.
        try:
            self._loop()
        except Exception as exc:
            import traceback
            self.get_logger().error('navigator error: {}\n{}'.format(exc, traceback.format_exc()))
            self._stop('internal error: {}'.format(exc))

    def _loop(self):
        if not self._goals:
            return
        if not self._fc_ok:
            # FC not GUIDED+armed: nothing would move. Don't count this as stuck.
            self._best_time = self._now()
            self._task.feedback('running', 'waiting for FC: GUIDED + armed ({})'.format(self._fc_status))
            return
        pose = self._pose.pose()
        gm = self._map.map
        if gm is None:
            self._best_time = self._now()
            self._task.feedback('running', 'waiting for a map (is SLAM running?)')
            return
        if pose is None:
            self._handle_lost()
            return
        self._lost_since = None
        gx, gy = self._goals[0]
        dist = math.hypot(gx - pose[0], gy - pose[1])
        if dist < self._tol:
            self._goals.pop(0)
            if not self._goals:
                self._stop('arrived', ok=True)
                return
            self._task.feedback('running', 'waypoint reached, {} left'.format(len(self._goals)))
            self._start_leg()
            return

        now = self._now()
        if dist < self._best_dist - 0.05:
            self._best_dist, self._best_time = dist, now
        elif now - self._best_time > self._stuck_timeout:
            self._replans_stuck += 1
            if self._replans_stuck > 2:
                self._stop('stuck: no progress for {:.0f} s'.format(3 * self._stuck_timeout))
                return
            # Probably nosed into something: reverse straight back, then replan.
            bs = self.get_parameter('backup_speed').value
            self._backup_until = now + self.get_parameter('backup_distance').value / bs
            self.get_logger().warn('no progress: backing up, then replanning')
            self._best_time, self._last_plan = now, -math.inf
            self._path = None
        if now < self._backup_until:
            cmd = Twist()
            cmd.linear.x = -self.get_parameter('backup_speed').value
            self._cmd.publish(cmd)
            self._best_time = now
            return

        if self._path is None or now - self._last_plan > self._replan:
            path = plan(gm, pose[:2], (gx, gy), self._radius)
            self._last_plan = now
            if path is None:
                if self._path is None:
                    self._stop('no path to ({:.2f}, {:.2f})'.format(gx, gy))
                    return
            else:
                self._path = path
                self._publish_path(path)

        min_r = max(self._min_r.values())
        v, w, idx = pure_pursuit(pose, self._path, self._lookahead, self._speed, min_r)

        # Path leads behind us: a car can't pivot, so do a three-point turn.
        err = heading_error(pose, self._path[idx])
        g = self.get_parameter
        if self._turning_since is None and abs(err) > g('turnaround_start').value and dist > self._tol:
            self._turning_since = now
            self._turn = ThreePointTurn()
            self.get_logger().info('target behind ({:.0f} deg): three-point turn'.format(math.degrees(err)))
        if self._turning_since is not None:
            if abs(err) < g('turnaround_done').value or now - self._turning_since > g('turnaround_max_s').value:
                self._turning_since = None
            else:
                tv, tw = self._turn.step(now, pose, self._path[idx], gm, min_r, self._front, self._rear)
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = tv, tw
                self._cmd.publish(cmd)
                if not self._turn.boxed_in:
                    self._best_time = now  # turning in place isn't "no progress" (unless boxed in)
                self._task.feedback('running', 'three-point turn ({:.0f} deg to go, {}){}'.format(
                    math.degrees(abs(err)), 'reversing' if tv < 0 else 'forward',
                    ' - boxed in' if self._turn.boxed_in else ''))
                return
        # Work in curvature (1/m): the path shape doesn't depend on speed.
        k = w / v if v else 0.0
        side = 'left' if k > 0 else 'right'
        k = max(-1.0 / self._min_r[side], min(1.0 / self._min_r[side], k))   # per-side limit
        v = speed_for_curvature(k, self._speed, max(self._speed, g('max_speed').value))
        v *= max(0.4, min(1.0, dist / 0.6))            # slow down for the last bit
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, k * v
        self._cmd.publish(cmd)
        self._task.feedback('running', '{:.1f} m to waypoint, {} left'.format(dist, len(self._goals)),
                            progress=round(dist, 2))

    def _handle_lost(self):
        """No fresh pose: visual odometry lost tracking. Usually the cameras are
        too close to something (stereo can't see closer than ~0.3 m), so back
        away slowly - that normally brings tracking back."""
        now = self._now()
        self._best_time = now
        if self._lost_since is None:
            self._lost_since = now
            self.get_logger().warn('localization lost (no fresh map->base_link pose)')
        if now - self._lost_since > self.get_parameter('lost_timeout').value:
            self._stop('localization lost for {:.0f} s - check the cameras / SLAM'.format(
                now - self._lost_since))
            return
        cmd = Twist()
        if now - self._lost_since > 2.0 and now - self._last_lost_backup < 3.0:
            cmd.linear.x = -0.15
        elif now - self._last_lost_backup > 8.0 and now - self._lost_since > 2.0:
            self._last_lost_backup = now
            cmd.linear.x = -0.15
        self._cmd.publish(cmd)
        self._task.feedback('running', 'localization lost - backing up to recover')

    def _publish_path(self, pts):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._path_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
