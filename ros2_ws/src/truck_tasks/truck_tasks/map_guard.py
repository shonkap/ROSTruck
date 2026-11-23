"""Map collision guard: the SLAM map as a third bumper. Python 3.6.

    ros2 run truck_tasks map_guard

The lidar sees only its scan plane (260 mm up) and the stereo guard only what
is in front of the cameras right now. The map remembers everything both have
seen, including obstacles 5 cm (2 in) and taller that RTAB-Map projected from
the stereo depth - so a shoe the cameras saw on the way in still blocks the
truck when it backs up or turns towards it later.

Occupied cells near the truck are moved into the truck's frame and checked
against the same lane as the lidar guard; published on safety/map/* (see
truck_common/guards.py). Without a map or pose (SLAM lost) it publishes "no
limit" and the live sensor guards carry on. Its accuracy is the
localization's, so it stops a little closer than the lidar guard.
"""
import math

import numpy as np
from rclpy.node import Node

from truck_common.guards import GuardPublisher, corridor_distances
from truck_common.maps import OCCUPIED_MIN
from truck_common.ros import PoseTracker, run_node
from truck_tasks.ros_common import MapCache


def occupied_points_near(grid, x, y, radius):
    """World (x, y) centres of occupied cells within a square of +-radius."""
    r0, c0 = grid.to_cell(x - radius, y - radius)
    r1, c1 = grid.to_cell(x + radius, y + radius)
    r0, c0 = max(r0, 0), max(c0, 0)
    r1, c1 = min(r1 + 1, grid.h), min(c1 + 1, grid.w)
    if r0 >= r1 or c0 >= c1:
        return np.zeros((0, 2))
    rows, cols = np.nonzero(grid.grid[r0:r1, c0:c1] >= OCCUPIED_MIN)
    return np.c_[grid.x0 + (cols + c0 + 0.5) * grid.res, grid.y0 + (rows + r0 + 0.5) * grid.res]


def to_base(points, pose):
    """World points -> truck frame (x forward, y left)."""
    if len(points) == 0:
        return points
    dx, dy = points[:, 0] - pose[0], points[:, 1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.c_[c * dx + s * dy, -s * dx + c * dy]


def map_distances(grid, pose, front_x, rear_x, half_width, look=1.0):
    pts = occupied_points_near(grid, pose[0], pose[1], max(front_x, -rear_x) + look)
    return corridor_distances(to_base(pts, pose), front_x, rear_x, half_width)


class MapGuard(Node):
    def __init__(self):
        super().__init__('map_guard')
        p = self.declare_parameter
        p('front_x', 0.25)          # m, bumpers relative to base_link
        p('rear_x', -0.25)
        p('half_width', 0.18)       # a bit narrower than the lidar's: map cells are 5 cm
        p('stop_distance', 0.12)    # m from the bumper
        p('slow_distance', 0.50)
        p('max_speed', 1.0)
        p('rate_hz', 10.0)
        self._g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._map = MapCache(self)
        self._pose = PoseTracker(self, max_age=1.0)
        self._out = GuardPublisher(self, 'map')
        self.create_timer(1.0 / self._g('rate_hz'), self._tick)

    def _tick(self):
        g = self._g
        grid, pose = self._map.map, self._pose.pose()
        front = rear = None
        if grid is not None and pose is not None:
            front, rear = map_distances(grid, pose, g('front_x'), g('rear_x'), g('half_width'),
                                        g('slow_distance') + 0.3)
        self._out.publish(front, rear, g('stop_distance'), g('slow_distance'), g('max_speed'))


def main(args=None):
    return run_node(MapGuard, args)
