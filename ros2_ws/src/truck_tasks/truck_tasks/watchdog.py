"""Watchdog module: turns bad conditions into tasks. Python 3.6.

    ros2 run truck_tasks watchdog --ros-args -p low_voltage:=7.0 -p critical_voltage:=6.6

  * battery below low_voltage for hold_s while a task runs  -> return home
    (priority 9: preempts normal tasks; "home" place if saved, else map origin)
  * battery below critical_voltage for hold_s               -> stop everything
  * FC link lost for link_timeout_s while a task runs       -> stop everything
    (nothing can drive anyway; leaves a clean state for when the link is back)

Voltage thresholds default to 0 = off (set them for your pack, e.g. 2S LiPo:
low 7.0 V, critical 6.6 V). A voltage of 0 / NaN means "no battery measured"
and never triggers. Status on watchdog/status.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from truck_tasks.places_node import normalize
from truck_tasks.ros_common import LATCHED, jparse


class WatchdogLogic(object):
    def __init__(self, low_voltage=0.0, critical_voltage=0.0, hold_s=10.0, link_timeout_s=3.0):
        self.low, self.crit = low_voltage, critical_voltage
        self.hold, self.link_timeout = hold_s, link_timeout_s
        self._low_since = self._crit_since = None
        self._link_lost_since = None
        self.returned_home = False
        self.stopped_for = None

    def update(self, t, voltage, link_ok, active_task):
        """Returns a list of actions: ('return_home', reason) / ('stop', reason)."""
        actions = []
        measured = voltage is not None and not math.isnan(voltage) and voltage > 0.5
        crit = measured and self.crit > 0 and voltage < self.crit
        low = measured and self.low > 0 and voltage < self.low
        self._crit_since = (self._crit_since or t) if crit else None
        self._low_since = (self._low_since or t) if low else None
        if self._crit_since is not None and t - self._crit_since >= self.hold and self.stopped_for != 'battery':
            self.stopped_for = 'battery'
            actions.append(('stop', 'battery critical: {:.2f} V'.format(voltage)))
        elif (self._low_since is not None and t - self._low_since >= self.hold and active_task
              and not self.returned_home and active_task.get('source') != 'watchdog'):
            self.returned_home = True
            actions.append(('return_home', 'battery low: {:.2f} V'.format(voltage)))
        if not low and not crit and measured:
            self.returned_home = False
            if self.stopped_for == 'battery':
                self.stopped_for = None

        self._link_lost_since = None if link_ok else (self._link_lost_since or t)
        if (self._link_lost_since is not None and t - self._link_lost_since >= self.link_timeout
                and active_task and self.stopped_for != 'link'):
            self.stopped_for = 'link'
            actions.append(('stop', 'FC link lost for {:.0f} s'.format(t - self._link_lost_since)))
        if link_ok and self.stopped_for == 'link':
            self.stopped_for = None
        return actions


class WatchdogNode(Node):
    def __init__(self):
        super().__init__('watchdog')
        p = self.declare_parameter
        p('low_voltage', 0.0)
        p('critical_voltage', 0.0)
        p('hold_s', 10.0)
        p('link_timeout_s', 3.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.logic = WatchdogLogic(g('low_voltage'), g('critical_voltage'), g('hold_s'), g('link_timeout_s'))
        self._voltage = None
        self._fc_status = ''
        self._fc_time = -1e9
        self._tasks = {}
        self._places = {}
        self._submit = self.create_publisher(String, 'tasks/submit', 10)
        self._status = self.create_publisher(String, 'watchdog/status', 10)
        self.create_subscription(BatteryState, 'fc/battery', self._on_batt, 10)
        self.create_subscription(String, 'fc/status', self._on_fc, 10)
        self.create_subscription(String, 'tasks/status', lambda m: setattr(self, '_tasks', jparse(m) or {}), LATCHED)
        self.create_subscription(String, 'places', self._on_places, LATCHED)
        self.create_timer(1.0, self._tick)
        self.get_logger().info('watchdog: low {} V, critical {} V (0 = off), link timeout {} s'.format(
            g('low_voltage'), g('critical_voltage'), g('link_timeout_s')))

    def _on_batt(self, msg):
        self._voltage = msg.voltage

    def _on_fc(self, msg):
        self._fc_status = msg.data
        self._fc_time = time.time()

    def _on_places(self, msg):
        d = jparse(msg)
        if isinstance(d, list):
            self._places = {normalize(p['name']): p for p in d if 'name' in p}

    def _tick(self):
        link_ok = self._fc_status.startswith('connected') and time.time() - self._fc_time < 3.0
        active = self._tasks.get('active')
        for kind, reason in self.logic.update(time.time(), self._voltage, link_ok, active):
            self.get_logger().warn('watchdog: {} -> {}'.format(reason, kind))
            if kind == 'stop':
                task = {'type': 'stop', 'priority': 10, 'source': 'watchdog', 'note': reason}
            else:
                home = 'home' if 'home' in self._places else [0.0, 0.0]
                task = {'type': 'goto', 'waypoints': [home], 'priority': 9, 'source': 'watchdog',
                        'note': reason}
            self._submit.publish(String(data=json.dumps(task)))
        self._status.publish(String(data=json.dumps({
            'voltage': self._voltage, 'link_ok': link_ok, 'stopped_for': self.logic.stopped_for,
            'returned_home': self.logic.returned_home})))


def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
