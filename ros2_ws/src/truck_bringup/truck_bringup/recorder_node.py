"""Record runs to rosbag2 from the web page (or any node). Python 3.6.

    ros2 run truck_bringup recorder_node --ros-args -p dir:=~/.ros/bags

In:  record/command  std_msgs/String JSON {"action": "start", "preset": "slam", "seconds": 120}
                                          {"action": "stop"}
Out: record/status   std_msgs/String JSON {recording, name, preset, seconds, size_mb, free_gb, message}

Presets:
  slam   stereo images + camera_info + FC IMU + TF + odom: replay later to tune
         SLAM/VO offline (ros2 bag play <dir> + slam.launch.py). ~25 MB/s raw.
  light  everything except images (telemetry, map, tasks, detections). Small.

Not a task: recording runs alongside whatever the truck is doing. Stops on
"stop", after `seconds` (capped by max_seconds), or when disk space drops
below min_free_gb.
"""
import json
import os
import shutil
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from truck_bringup.ros_qos import LATCHED

PRESETS = {
    'slam': ['/stereo/left/image_raw', '/stereo/left/camera_info', '/stereo/right/image_raw',
             '/stereo/right/camera_info', '/fc/imu', '/tf', '/tf_static', '/odom', '/cmd_vel',
             '/fc/status'],
    'light': ['/fc/imu', '/fc/status', '/fc/battery', '/fc/steering_report', '/tf', '/tf_static',
              '/odom', '/map', '/cmd_vel', '/tasks/status', '/safety/obstacle_distance',
              '/detections', '/object_landmarks', '/nav/path'],
}


def bag_name(preset, t=None):
    return time.strftime('%Y-%m-%d_%H%M%S', time.localtime(t)) + '_' + preset


def dir_size_mb(path):
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total / 1e6


class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder')
        self.declare_parameter('dir', '~/.ros/bags')
        self.declare_parameter('max_seconds', 600.0)
        self.declare_parameter('min_free_gb', 2.0)
        self._dir = os.path.expanduser(self.get_parameter('dir').value)
        if not os.path.isdir(self._dir):
            os.makedirs(self._dir)
        self._proc = None
        self._name = self._preset = None
        self._start = 0.0
        self._limit = 0.0
        self._message = 'idle'
        self._status = self.create_publisher(String, 'record/status', LATCHED)
        self.create_subscription(String, 'record/command', self._on_command, 10)
        self.create_timer(1.0, self._tick)

    def _free_gb(self):
        return shutil.disk_usage(self._dir).free / 1e9

    def _on_command(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if d.get('action') == 'start':
            self._start_rec(d.get('preset', 'slam'), float(d.get('seconds', 120)))
        elif d.get('action') == 'stop':
            self._stop_rec('stopped')

    def _start_rec(self, preset, seconds):
        if self._proc is not None:
            self._message = 'already recording ' + self._name
            return
        if preset not in PRESETS:
            self._message = 'unknown preset {} (have: {})'.format(preset, ', '.join(PRESETS))
            return
        if self._free_gb() < self.get_parameter('min_free_gb').value:
            self._message = 'not enough disk space ({:.1f} GB free)'.format(self._free_gb())
            return
        self._name, self._preset = bag_name(preset), preset
        self._limit = min(max(5.0, seconds), self.get_parameter('max_seconds').value)
        out = os.path.join(self._dir, self._name)
        self._proc = subprocess.Popen(['ros2', 'bag', 'record', '-o', out] + PRESETS[preset],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                      preexec_fn=os.setsid)
        self._start = time.time()
        self._message = 'recording'
        self.get_logger().info('recording {} ({}) for up to {:.0f} s -> {}'.format(
            self._name, preset, self._limit, out))

    def _stop_rec(self, why):
        if self._proc is None:
            return
        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)   # lets rosbag2 close the file cleanly
            self._proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
            except Exception:
                pass
        size = dir_size_mb(os.path.join(self._dir, self._name))
        self._message = '{}: {} ({:.0f} MB, {:.0f} s)'.format(why, self._name, size, time.time() - self._start)
        self.get_logger().info(self._message)
        self._proc = None

    def _tick(self):
        rec = self._proc is not None
        if rec and self._proc.poll() is not None:
            self._proc = None
            self._message = 'ros2 bag record exited unexpectedly'
            rec = False
        if rec and time.time() - self._start >= self._limit:
            self._stop_rec('time limit reached')
            rec = False
        if rec and self._free_gb() < self.get_parameter('min_free_gb').value / 2.0:
            self._stop_rec('stopped: disk almost full')
            rec = False
        st = {'recording': rec, 'name': self._name, 'preset': self._preset,
              'seconds': round(time.time() - self._start) if rec else None,
              'limit': self._limit, 'size_mb': round(dir_size_mb(os.path.join(self._dir, self._name)))
              if rec else None, 'free_gb': round(self._free_gb(), 1), 'message': self._message,
              'dir': self._dir, 'presets': sorted(PRESETS)}
        self._status.publish(String(data=json.dumps(st)))

    def destroy_node(self):
        self._stop_rec('stopped (shutdown)')
        Node.destroy_node(self)


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
