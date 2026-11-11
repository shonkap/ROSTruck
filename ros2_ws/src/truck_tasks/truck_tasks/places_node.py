"""Named places: "kitchen door", "charging spot", ... stored per map.

    ros2 run truck_tasks places_node --ros-args -p file:=~/.ros/places_living_room.json

In:   places/set   std_msgs/String JSON {"name": "kitchen", "x": 1.2, "y": 0.4}
                   {"name": "kitchen", "here": true}     -> the truck's current position
                   {"name": "kitchen", "delete": true}
Out:  places       std_msgs/String JSON [{"name", "x", "y"}, ...]  (latched)

The navigator accepts place names as goto waypoints ({"type": "goto",
"waypoints": ["kitchen"]}), the LLM planner gets them as context, and the web
page lists/draws them. Saved to `file` on every change. Python 3.6.
"""
import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from truck_tasks.ros_common import LATCHED, PoseTracker, jparse


def normalize(name):
    return ' '.join(str(name).strip().lower().replace('_', ' ').split())


def resolve_waypoints(points, places):
    """[[x, y] | "place name", ...] -> [(x, y), ...]. places: {normalized name: {x, y}}.
    Raises ValueError naming the problem."""
    out = []
    for p in points:
        if isinstance(p, str):
            pl = places.get(normalize(p))
            if pl is None:
                raise ValueError('unknown place "{}" (known: {})'.format(
                    p, ', '.join(sorted(v['name'] for v in places.values())) or 'none'))
            out.append((float(pl['x']), float(pl['y'])))
        else:
            try:
                x, y = p
                out.append((float(x), float(y)))
            except (TypeError, ValueError):
                raise ValueError('waypoint {!r} is neither [x, y] nor a place name'.format(p))
    return out


class PlaceBook(object):
    def __init__(self, path=None):
        self.path = path
        self.places = {}
        if path and os.path.exists(path):
            with open(path) as f:
                for p in json.load(f):
                    self.places[normalize(p['name'])] = {'name': p['name'], 'x': float(p['x']),
                                                         'y': float(p['y'])}

    def set(self, name, x, y):
        self.places[normalize(name)] = {'name': str(name).strip(), 'x': round(float(x), 2),
                                        'y': round(float(y), 2)}
        self.save()

    def delete(self, name):
        removed = self.places.pop(normalize(name), None) is not None
        self.save()
        return removed

    def get(self, name):
        return self.places.get(normalize(name))

    def list(self):
        return sorted(self.places.values(), key=lambda p: p['name'].lower())

    def save(self):
        if not self.path:
            return
        d = os.path.dirname(self.path)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(self.list(), f, indent=1)
        os.replace(tmp, self.path)


class PlacesNode(Node):
    def __init__(self):
        super().__init__('places')
        self.declare_parameter('file', '~/.ros/places.json')
        self.book = PlaceBook(os.path.expanduser(self.get_parameter('file').value))
        self._pose = PoseTracker(self, max_age=2.0)
        self._pub = self.create_publisher(String, 'places', LATCHED)
        self.create_subscription(String, 'places/set', self._on_set, 10)
        self._publish()
        self.get_logger().info('{} place(s) from {}'.format(len(self.book.places), self.book.path))

    def _on_set(self, msg):
        d = jparse(msg)
        if not isinstance(d, dict) or not str(d.get('name', '')).strip():
            self.get_logger().warn('places/set needs {"name": ..., "x", "y"} / "here" / "delete"')
            return
        name = d['name']
        if d.get('delete'):
            self.get_logger().info('place "{}" {}'.format(name, 'deleted' if self.book.delete(name) else 'not found'))
        elif d.get('here'):
            pose = self._pose.pose()
            if pose is None:
                self.get_logger().warn('no current pose (is SLAM running?) - "{}" not saved'.format(name))
                return
            self.book.set(name, pose[0], pose[1])
            self.get_logger().info('place "{}" = here ({:.2f}, {:.2f})'.format(name, pose[0], pose[1]))
        else:
            try:
                self.book.set(name, d['x'], d['y'])
            except (KeyError, TypeError, ValueError):
                self.get_logger().warn('place "{}": bad x/y'.format(name))
                return
            self.get_logger().info('place "{}" = ({:.2f}, {:.2f})'.format(name, float(d['x']), float(d['y'])))
        self._publish()

    def _publish(self):
        self._pub.publish(String(data=json.dumps(self.book.list())))


def main(args=None):
    rclpy.init(args=args)
    node = PlacesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
