"""LLM planner plugin: answers natural-language task requests.

    export ANTHROPIC_API_KEY=...          # on the PC running this
    ros2 run truck_planner planner_node
    ros2 run truck_planner planner_node --ros-args -p auto_submit:=true

Listens on tasks/requests (what you type into "Ask the planner" on the web
page), gives Claude the robot's situation (pose, map summary, frontiers,
landmarks, FC state, task queue) and publishes a report + proposed task list
on tasks/reports. By default the tasks are only PROPOSED - the web page shows
a "Run these tasks" button. With auto_submit they're queued right away
(source "llm"). Every task is validated first (known type, waypoints on free
map space).

object_landmarks (std_msgs/String JSON {"label", "x", "y", ...} or a list of them):
named places, e.g. from a camera object detector, so "go to the couch" works.

Without credentials it falls back to a small keyword planner and says so.
"""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from truck_planner.llm import DEFAULT_MODEL, PlannerError, make_client, plan_with_claude
from truck_planner.planner_core import build_context, rule_based_plan, summarize_map, validate_plan
from truck_tasks.planning import approach_point, frontiers, mark_driven, reachable_free
from truck_tasks.ros_common import LATCHED, MapCache, ModuleAnnouncer, PoseTracker, jmsg, jparse


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner')
        self.declare_parameter('model', DEFAULT_MODEL)
        self.declare_parameter('effort', 'medium')
        self.declare_parameter('auto_submit', False)
        self._map = MapCache(self)
        self._pose = PoseTracker(self, max_age=5.0)
        self._fc_status = ''
        self._tasks = {}
        self._landmarks = {}
        self._places = []
        self._client = make_client()
        self._busy = threading.Lock()
        self._report_pub = self.create_publisher(String, 'tasks/reports', 10)
        # exactly what the planner gives the LLM (inspect with: ros2 topic echo /planner/context)
        self._ctx_pub = self.create_publisher(String, 'planner/context', 10)
        self._submit_pub = self.create_publisher(String, 'tasks/submit', 10)
        self.create_subscription(String, 'tasks/requests', self._on_request, 10)
        self.create_subscription(String, 'tasks/status', self._on_tasks, LATCHED)
        self.create_subscription(String, 'fc/status', self._on_fc, 10)
        self.create_subscription(String, 'object_landmarks', self._on_landmark, 10)
        self.create_subscription(String, 'places', self._on_places, LATCHED)
        # Announce as the planner (shown on the web page). The manager never
        # dispatches "request" tasks, it forwards them on tasks/requests.
        ModuleAnnouncer(self, 'planner', ['request'], lambda t: None, lambda: None)
        self.get_logger().info('planner ready: {}'.format(
            'Claude {}'.format(self.get_parameter('model').value) if self._client
            else 'NO Claude credentials -> keyword planner (set ANTHROPIC_API_KEY)'))

    def _on_tasks(self, msg):
        self._tasks = jparse(msg) or {}

    def _on_fc(self, msg):
        self._fc_status = msg.data

    def _on_places(self, msg):
        d = jparse(msg)
        if isinstance(d, list):
            self._places = d

    def _on_landmark(self, msg):
        d = jparse(msg)
        for lm in d if isinstance(d, list) else [d]:
            if isinstance(lm, dict) and lm.get('label') and 'x' in lm and 'y' in lm:
                lm = dict(lm, seen=time.strftime('%H:%M:%S'))
                self._landmarks[str(lm['label'])] = lm

    def _snapper(self, pose):
        """Waypoint -> nearest spot the truck can reach with clearance (or None)."""
        gm = self._map.map
        if gm is None or pose is None:
            return None            # no map: can't check, accept as given
        gm = mark_driven(gm, [pose[:2]], 0.35)
        reach = reachable_free(gm, pose[:2], 0.2)
        return lambda x, y: approach_point(gm, (x, y), reach, clearance=0.35, max_dist=1.0)

    def _on_request(self, msg):
        req = jparse(msg)
        if not req or not req.get('text'):
            return
        # Claude can take a while: never block the ROS executor.
        threading.Thread(target=self._handle, args=(req,), daemon=True).start()

    def _handle(self, req):
        with self._busy:          # one plan at a time
            text = req['text']
            pose = self._pose.pose()
            gm = self._map.map
            landmarks = list(self._landmarks.values())
            summary = summarize_map(gm, pose, lambda g, p: frontiers(g, 10, 0.2, start_xy=p[:2]))
            places = list(self._places)
            ctx = build_context(text, pose, summary, self._fc_status, self._tasks, landmarks, places)
            self._ctx_pub.publish(jmsg(ctx))
            t0 = time.time()
            source = 'keyword planner'
            try:
                if self._client is None:
                    plan = rule_based_plan(text, pose, landmarks, places)
                else:
                    plan = plan_with_claude(self._client, ctx, self.get_parameter('model').value,
                                            self.get_parameter('effort').value)
                    source = plan.get('model', 'claude')
            except PlannerError as exc:
                self.get_logger().error('planner: {}'.format(exc))
                if exc.fallback_ok:
                    plan = rule_based_plan(text, pose, landmarks, places)
                    plan['report'] = 'Claude unavailable ({}). '.format(exc) + plan['report']
                else:
                    plan = {'title': 'Planner error', 'report': str(exc), 'tasks': []}
            tasks, problems = validate_plan(plan, self._snapper(pose))
            report = plan.get('report', '')
            if problems:
                report += '\n\nValidation:\n- ' + '\n- '.join(problems)
            auto = self.get_parameter('auto_submit').value and bool(tasks)
            out = {'request_id': req.get('id'), 'request': text, 'title': plan.get('title', ''),
                   'text': report, 'tasks': tasks, 'submitted': auto, 'planner': source,
                   'seconds': round(time.time() - t0, 1)}
            self._report_pub.publish(jmsg(out))
            self.get_logger().info('plan for "{}": {} task(s) via {} in {:.1f}s{}'.format(
                text, len(tasks), source, out['seconds'], ' - submitted' if auto else ''))
            if auto:
                for t in tasks:
                    t = dict(t, source='llm')
                    self._submit_pub.publish(String(data=json.dumps(t)))


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
