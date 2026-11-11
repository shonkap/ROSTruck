"""Task manager node: one queue for everything the truck should do.

Sources (web page, LLM planner, object detector, CLI) publish tasks on
tasks/submit; executor modules (navigator, explorer, future plugins)
register the types they handle on tasks/modules and get work on
tasks/dispatch. See ros_common.py for the full bus description.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from truck_tasks.ros_common import LATCHED, jmsg, jparse
from truck_tasks.task_core import TaskBoard


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.board = TaskBoard()
        self._status_pub = self.create_publisher(String, 'tasks/status', LATCHED)
        self._dispatch_pub = self.create_publisher(String, 'tasks/dispatch', 10)
        self._abort_pub = self.create_publisher(String, 'tasks/abort', 10)
        self._request_pub = self.create_publisher(String, 'tasks/requests', 10)
        self.create_subscription(String, 'tasks/submit', self._on_submit, 10)
        self.create_subscription(String, 'tasks/cancel', self._on_cancel, 10)
        self.create_subscription(String, 'tasks/feedback', self._on_feedback, 10)
        self.create_subscription(String, 'tasks/modules', self._on_module, 10)
        self.create_timer(1.0, self._tick)

    def _emit(self, events):
        for kind, t in events:
            if kind == 'dispatch':
                self.get_logger().info('task {} {} -> {}'.format(t['id'], t['type'], t['handler']))
                self._dispatch_pub.publish(jmsg(t))
            elif kind == 'cancel':
                self._abort_pub.publish(jmsg({'id': t['id']}))
            elif kind == 'request':
                self.get_logger().info('request {} forwarded: {}'.format(t['id'], t.get('text', '')))
                self._request_pub.publish(jmsg(t))
            elif kind == 'rejected':
                self.get_logger().warn('rejected: {}'.format(t.get('message') or t.get('error')))
        self._publish_status()

    def _on_submit(self, msg):
        task = jparse(msg)
        t, events = self.board.submit(task)
        if t is not None:
            self.get_logger().info('task {} submitted by {}: {}'.format(
                t['id'], t['source'], t['type']))
        self._emit(events)

    def _on_cancel(self, msg):
        arg = msg.data.strip()
        if arg == 'all':
            self._emit(self.board.cancel_all('canceled by user'))
        else:
            try:
                self._emit(self.board.cancel(int(arg)))
            except ValueError:
                self.get_logger().warn('tasks/cancel wants "all" or a task id, got ' + arg)

    def _on_feedback(self, msg):
        d = jparse(msg)
        if not d or 'id' not in d:
            return
        events = self.board.feedback(d['id'], d.get('state', ''), d.get('message', ''),
                                     d.get('progress'))
        if d.get('state') in ('succeeded', 'failed', 'canceled'):
            self.get_logger().info('task {} {}: {}'.format(d['id'], d['state'], d.get('message', '')))
        self._emit(events)

    def _on_module(self, msg):
        d = jparse(msg)
        if d and d.get('name'):
            new = d['name'] not in self.board.modules
            self.board.register(d['name'], d.get('handles', []))
            if new:
                self.get_logger().info('module {} handles {}'.format(d['name'], d.get('handles')))

    def _tick(self):
        self._emit(self.board.tick())

    def _publish_status(self):
        self._status_pub.publish(jmsg(self.board.status()))


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
