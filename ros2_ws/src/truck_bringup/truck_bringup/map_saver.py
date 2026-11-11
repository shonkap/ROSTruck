"""Save RTAB-Map's 2D occupancy grid (/map) as PNG + YAML (nav2 map format).

    ros2 run truck_bringup map_saver --ros-args -p output:=~/maps/living_room

Writes <output>.png (white = free, black = occupied, grey = unknown) and
<output>.yaml, and prints how much of the map is known. Python 3.6 compatible.
"""
import os

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def grid_to_image(data, width, height):
    """OccupancyGrid data (row-major, origin bottom-left) -> uint8 image (top row first)."""
    g = np.asarray(data, dtype=np.int16).reshape(height, width)
    img = np.full((height, width), 205, np.uint8)      # unknown
    img[(g >= 0) & (g < 25)] = 254                     # free
    img[g >= 65] = 0                                   # occupied
    return np.flipud(img)


def map_stats(data):
    g = np.asarray(data, dtype=np.int16)
    known = int(np.count_nonzero(g >= 0))
    return {'known': known, 'free': int(np.count_nonzero((g >= 0) & (g < 25))),
            'occupied': int(np.count_nonzero(g >= 65)), 'total': int(g.size)}


class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.declare_parameter('output', '~/maps/map')
        self.declare_parameter('topic', 'map')
        self._out = os.path.expanduser(self.get_parameter('output').value)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self.get_parameter('topic').value, self._on_map, qos)
        self.done = False

    def _on_map(self, msg):
        info = msg.info
        if info.width == 0 or info.height == 0:
            self.get_logger().warn('map is empty, waiting...')
            return
        d = os.path.dirname(self._out)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        cv2.imwrite(self._out + '.png', grid_to_image(msg.data, info.width, info.height))
        q = info.origin.orientation
        yaw = 2.0 * np.arctan2(q.z, q.w)
        with open(self._out + '.yaml', 'w') as f:
            f.write('image: {}\nmode: trinary\nresolution: {}\norigin: [{}, {}, {}]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'.format(
                        os.path.basename(self._out) + '.png', info.resolution,
                        info.origin.position.x, info.origin.position.y, float(yaw)))
        s = map_stats(msg.data)
        cell = info.resolution ** 2
        self.get_logger().info(
            'saved {0}.png/.yaml: {1}x{2} cells @ {3:.3f} m, known {4:.1f} m^2 '
            '(free {5:.1f} m^2, occupied {6:.1f} m^2)'.format(
                self._out, info.width, info.height, info.resolution, s['known'] * cell,
                s['free'] * cell, s['occupied'] * cell))
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
