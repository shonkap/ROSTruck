"""Web control page for the truck: live map, click-to-set waypoints, tasks.

A plugin like any other module: it only talks to the rest of the system over
ROS topics (the task bus in truck_tasks/ros_common.py). Standard library HTTP
server, no extra dependencies, Python 3.6 - runs on the Jetson or the PC.

    ros2 run truck_web web_node                      # http://<host>:8080
    ros2 run truck_web web_node --ros-args -p port:=8080 -p token:=secret

HTTP API (also handy for scripts):
    GET  /api/state      pose, path, FC status, tasks, planner reports, map info
    GET  /api/map.png    current occupancy map (row 0 = top = max y)
    GET  /api/camera.jpg latest left camera frame (downscaled, ~5 Hz)
    POST /api/task       a task JSON, e.g. {"type": "goto", "waypoints": [[1, 2]]}
    POST /api/cancel     {"id": 3} or {"id": "all"}
    POST /api/place      {"name", "x", "y"} | {"name", "here": true} | {"name", "delete": true}
    POST /api/record     {"action": "start", "preset": "slam"|"light", "seconds": 120} | {"action": "stop"}
    POST /api/save_map   save the current map as <map_dir>/map.png + map.yaml (nav2 format)
    POST /api/teleop     {"v": m/s, "w": rad/s} - hold-to-drive: send >= 5 Hz; stops
                         0.3 s after the last one. Refused while a task is running.

!! Anyone who can reach this port can send the truck somewhere. Keep it on a
private network, or set `token` (then add ?token=... to the URL).
"""
import json
import os
import socketserver
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import Float32, String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def map_to_png(data, width, height):
    g = np.asarray(data, dtype=np.int16).reshape(height, width)
    img = np.full((height, width, 3), (60, 56, 52), np.uint8)     # unknown (BGR)
    img[(g >= 0) & (g < 25)] = (240, 240, 236)                    # free
    img[(g >= 25) & (g < 65)] = (170, 170, 170)                   # uncertain
    img[g >= 65] = (40, 40, 40)                                   # occupied
    ok, buf = cv2.imencode('.png', np.flipud(img))
    return buf.tobytes() if ok else b''


class WebState(object):
    """Everything the page shows; written by ROS callbacks, read by HTTP threads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.map_png = b''
        self.map_info = None
        self.map_version = 0
        self.pose = None
        self.path = []
        self.fc_status = ''
        self.battery = None
        self.tasks = {}
        self.reports = []
        self.steering = {}
        self.nav = {}
        self.camera_jpg = b''
        self.camera_time = 0.0
        self.obstacle = None
        self.rear = None
        self.lidar_front = None
        self.places = []
        self.record = {}


class WebNode(Node):
    def __init__(self):
        super().__init__('web_node')
        self.declare_parameter('port', 8080)
        self.declare_parameter('bind', '0.0.0.0')
        self.declare_parameter('token', '')
        self.declare_parameter('camera_topic', 'stereo/left/image_raw')
        self.declare_parameter('camera_hz', 5.0)
        self.declare_parameter('camera_width', 480)
        self.declare_parameter('map_dir', '')                # where "Save map" writes
        self.declare_parameter('teleop_max_speed', 0.5)     # m/s
        self.declare_parameter('teleop_max_turn', 1.0)      # rad/s
        self.state = WebState()
        self._last_map = None
        self.token = self.get_parameter('token').value
        self._submit = self.create_publisher(String, 'tasks/submit', 10)
        self._place_pub = self.create_publisher(String, 'places/set', 10)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._teleop = None             # (v, w, time) from the last /api/teleop
        self._teleop_lock = threading.Lock()
        self.create_timer(0.1, self._teleop_tick)
        self.create_subscription(String, 'places', self._on_places, LATCHED)
        self._rec_pub = self.create_publisher(String, 'record/command', 10)
        self.create_subscription(String, 'record/status', self._on_record, LATCHED)
        self._cancel = self.create_publisher(String, 'tasks/cancel', 10)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, LATCHED)
        self.create_subscription(Path, 'nav/path', self._on_path, 10)
        self.create_subscription(String, 'tasks/status', self._on_tasks, LATCHED)
        self.create_subscription(String, 'tasks/reports', self._on_report, 10)
        self.create_subscription(String, 'fc/status', self._on_fc, 10)
        self.create_subscription(String, 'fc/steering_report', self._on_steering, 10)
        self.create_subscription(String, 'nav/state', self._on_nav, 10)
        self.create_subscription(BatteryState, 'fc/battery', self._on_battery, 10)
        self.create_subscription(Image, self.get_parameter('camera_topic').value, self._on_camera,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32, 'safety/obstacle_distance', self._on_obstacle, 10)
        self.create_subscription(Float32, 'safety/rear_distance', self._on_rear, 10)
        self.create_subscription(Float32, 'safety/lidar/front_distance', self._on_lidar_front, 10)
        self._cam_last = 0.0
        import tf2_ros
        self._tf = tf2_ros.Buffer()
        self._tfl = tf2_ros.TransformListener(self._tf, self)
        self.create_timer(0.2, self._update_pose)
        self.static_dir = os.path.join(get_package_share_directory('truck_web'), 'static')

    # ---------------------------------------------------------- ROS inputs
    def _on_map(self, msg):
        self._last_map = msg
        i = msg.info
        if i.width == 0:
            return
        png = map_to_png(msg.data, i.width, i.height)
        with self.state.lock:
            self.state.map_png = png
            self.state.map_version += 1
            self.state.map_info = {'width': i.width, 'height': i.height, 'resolution': i.resolution,
                                   'origin': [i.origin.position.x, i.origin.position.y],
                                   'version': self.state.map_version}

    def save_map(self):
        """Write the latest /map as map.png/.yaml into map_dir. Returns (ok, message)."""
        d = os.path.expanduser(self.get_parameter('map_dir').value or '~/maps/current')
        msg = self._last_map
        if msg is None:
            return False, 'no map received yet'
        i = msg.info
        g = np.asarray(msg.data, dtype=np.int16).reshape(i.height, i.width)
        img = np.full(g.shape, 205, np.uint8)                       # nav2 trinary map
        img[(g >= 0) & (g < 25)] = 254
        img[g >= 65] = 0
        if not os.path.isdir(d):
            os.makedirs(d)
        cv2.imwrite(os.path.join(d, 'map.png'), np.flipud(img))
        with open(os.path.join(d, 'map.yaml'), 'w') as f:
            f.write('image: map.png\nmode: trinary\nresolution: {}\norigin: [{}, {}, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'.format(
                        i.resolution, i.origin.position.x, i.origin.position.y))
        known = float((g >= 0).sum()) * i.resolution ** 2
        return True, 'saved {}/map.png ({:.1f} m^2 known)'.format(d, known)

    def _on_path(self, msg):
        with self.state.lock:
            self.state.path = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]

    def _json(self, msg):
        try:
            return json.loads(msg.data)
        except ValueError:
            return None

    def _on_tasks(self, msg):
        d = self._json(msg)
        if d is not None:
            with self.state.lock:
                self.state.tasks = d

    def _on_report(self, msg):
        d = self._json(msg) or {'text': msg.data}
        d['received'] = time.strftime('%H:%M:%S')
        with self.state.lock:
            self.state.reports.insert(0, d)
            del self.state.reports[20:]

    def _on_fc(self, msg):
        with self.state.lock:
            self.state.fc_status = msg.data

    def _on_steering(self, msg):
        d = self._json(msg)
        if d is not None:
            with self.state.lock:
                self.state.steering = d

    def _on_nav(self, msg):
        d = self._json(msg)
        if d is not None:
            with self.state.lock:
                self.state.nav = d

    def _on_battery(self, msg):
        with self.state.lock:
            self.state.battery = {'voltage': msg.voltage if msg.voltage == msg.voltage else None,
                                  'percentage': msg.percentage if msg.percentage == msg.percentage else None}

    def _on_camera(self, msg):
        now = time.time()
        if now - self._cam_last < 1.0 / self.get_parameter('camera_hz').value:
            return
        self._cam_last = now
        try:
            buf = np.frombuffer(msg.data, np.uint8)
            if msg.encoding in ('rgb8', 'bgr8'):
                img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
                if msg.encoding == 'rgb8':
                    img = img[:, :, ::-1]
            elif msg.encoding in ('mono8', '8UC1'):
                img = buf.reshape(msg.height, msg.step)[:, :msg.width]
            else:
                return
            w = self.get_parameter('camera_width').value
            if img.shape[1] > w:
                img = cv2.resize(img, (w, int(img.shape[0] * w / img.shape[1])), interpolation=cv2.INTER_AREA)
            ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
        except ValueError:
            return
        if ok:
            with self.state.lock:
                self.state.camera_jpg = jpg.tobytes()
                self.state.camera_time = now

    def teleop(self, v, w):
        """Returns an error string, or None if accepted."""
        with self.state.lock:
            active = (self.state.tasks or {}).get('active')
        if active:
            return 'a task is running (#{} {}) - cancel it first'.format(active.get('id'), active.get('type'))
        vmax = self.get_parameter('teleop_max_speed').value
        wmax = self.get_parameter('teleop_max_turn').value
        v = max(-vmax, min(vmax, float(v)))
        w = max(-wmax, min(wmax, float(w)))
        with self._teleop_lock:
            self._teleop = (v, w, time.time())
        return None

    def _teleop_tick(self):
        with self._teleop_lock:
            t = self._teleop
        if t is None:
            return
        cmd = Twist()
        if time.time() - t[2] < 0.3:
            cmd.linear.x, cmd.angular.z = t[0], t[1]
            self._cmd_pub.publish(cmd)
        else:                           # released / connection lost: one stop, then quiet
            self._cmd_pub.publish(cmd)
            with self._teleop_lock:
                if self._teleop is t:
                    self._teleop = None

    def _on_record(self, msg):
        d = self._json(msg)
        if isinstance(d, dict):
            with self.state.lock:
                self.state.record = d

    def _on_places(self, msg):
        d = self._json(msg)
        if isinstance(d, list):
            with self.state.lock:
                self.state.places = d

    def _on_rear(self, msg):
        with self.state.lock:
            self.state.rear = None if msg.data >= 50.0 else round(float(msg.data), 2)

    def _on_lidar_front(self, msg):
        with self.state.lock:
            self.state.lidar_front = None if msg.data >= 50.0 else round(float(msg.data), 2)

    def _on_obstacle(self, msg):
        with self.state.lock:
            self.state.obstacle = None if msg.data >= 50.0 else round(float(msg.data), 2)

    def _update_pose(self):
        import math
        from rclpy.time import Time
        try:
            t = self._tf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self.state.lock:
            self.state.pose = [t.transform.translation.x, t.transform.translation.y, yaw]

    # ---------------------------------------------------------- HTTP side
    def snapshot(self):
        s = self.state
        with s.lock:
            return {'pose': s.pose, 'path': s.path, 'fc_status': s.fc_status, 'battery': s.battery,
                    'tasks': s.tasks, 'reports': s.reports, 'steering': s.steering, 'nav': s.nav,
                    'map': s.map_info, 'time': time.time(), 'obstacle': s.obstacle,
                    'places': s.places, 'record': s.record, 'rear': s.rear,
                    'lidar_front': s.lidar_front,
                    'camera_age': (time.time() - s.camera_time) if s.camera_time else None}

    def submit(self, task):
        task = dict(task)
        task.setdefault('source', 'web')
        self._submit.publish(String(data=json.dumps(task)))

    def cancel(self, task_id):
        self._cancel.publish(String(data=str(task_id)))


def make_handler(node):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):   # keep the ROS log clean
            pass

        def _authorized(self):
            if not node.token:
                return True
            q = parse_qs(urlparse(self.path).query)
            return q.get('token', [''])[0] == node.token or \
                self.headers.get('X-Token', '') == node.token

        def _send(self, code, body, ctype='application/json'):
            if isinstance(body, str):
                body = body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if not self._authorized():
                return self._send(403, '{"error": "bad token"}')
            path = urlparse(self.path).path
            if path in ('/', '/index.html'):
                with open(os.path.join(node.static_dir, 'index.html'), 'rb') as f:
                    return self._send(200, f.read(), 'text/html; charset=utf-8')
            if path == '/api/state':
                return self._send(200, json.dumps(node.snapshot(), default=str))
            if path == '/api/camera.jpg':
                with node.state.lock:
                    jpg = node.state.camera_jpg
                return self._send(200, jpg, 'image/jpeg') if jpg else self._send(404, '{}')
            if path == '/api/map.png':
                with node.state.lock:
                    png = node.state.map_png
                return self._send(200, png, 'image/png') if png else self._send(404, '{}')
            return self._send(404, '{"error": "not found"}')

        def do_POST(self):
            if not self._authorized():
                return self._send(403, '{"error": "bad token"}')
            n = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(n).decode('utf-8') or '{}')
            except ValueError:
                return self._send(400, '{"error": "invalid JSON"}')
            path = urlparse(self.path).path
            if path == '/api/task':
                if not isinstance(body, dict) or 'type' not in body:
                    return self._send(400, '{"error": "task needs a type"}')
                node.submit(body)
                return self._send(200, '{"ok": true}')
            if path == '/api/save_map':
                ok, message = node.save_map()
                return self._send(200 if ok else 409, json.dumps({'ok': ok, 'message': message}))
            if path == '/api/record':
                if body.get('action') not in ('start', 'stop'):
                    return self._send(400, '{"error": "action must be start or stop"}')
                node._rec_pub.publish(String(data=json.dumps(body)))
                return self._send(200, '{"ok": true}')
            if path == '/api/teleop':
                try:
                    err = node.teleop(body.get('v', 0.0), body.get('w', 0.0))
                except (TypeError, ValueError):
                    return self._send(400, '{"error": "v and w must be numbers"}')
                if err:
                    return self._send(409, json.dumps({'error': err}))
                return self._send(200, '{"ok": true}')
            if path == '/api/place':
                if not isinstance(body, dict) or not str(body.get('name', '')).strip():
                    return self._send(400, '{"error": "place needs a name"}')
                node._place_pub.publish(String(data=json.dumps(body)))
                return self._send(200, '{"ok": true}')
            if path == '/api/cancel':
                node.cancel(body.get('id', 'all'))
                return self._send(200, '{"ok": true}')
            return self._send(404, '{"error": "not found"}')
    return Handler


class ThreadingHTTPServer(socketserver.ThreadingMixIn, HTTPServer):   # py3.6 has no http.server one
    daemon_threads = True
    allow_reuse_address = True


def main(args=None):
    rclpy.init(args=args)
    node = WebNode()
    port, bind = node.get_parameter('port').value, node.get_parameter('bind').value
    server = ThreadingHTTPServer((bind, port), make_handler(node))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info('truck web page on http://{}:{}/{}'.format(
        bind, port, '?token=...' if node.token else ''))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
