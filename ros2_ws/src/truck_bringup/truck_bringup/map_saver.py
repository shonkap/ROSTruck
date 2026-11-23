"""Save RTAB-Map's 2D occupancy grid (/map) as PNG + YAML (nav2 map format).

    ros2 run truck_bringup map_saver --ros-args -p output:=~/maps/living_room

Writes <output>.png (white = free, black = occupied, grey = unknown) and
<output>.yaml, and prints how much of the map is known. Python 3.6 compatible.
"""
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from truck_common.maps import save_nav2_map
from truck_common.ros import LATCHED, run_node


class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.declare_parameter('output', '~/maps/map')
        self.declare_parameter('topic', 'map')
        self.create_subscription(OccupancyGrid, self.get_parameter('topic').value, self._on_map, LATCHED)
        self.done = False

    def _on_map(self, msg):
        if msg.info.width == 0 or msg.info.height == 0:
            self.get_logger().warn('map is empty, waiting...')
            return
        self.get_logger().info(save_nav2_map(msg, self.get_parameter('output').value))
        self.done = True


def main(args=None):
    return run_node(MapSaver, args, until=lambda n: n.done)
