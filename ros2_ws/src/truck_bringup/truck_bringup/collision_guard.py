"""Stereo collision guard: anything 2 in (5 cm) or taller ahead. Python 3.6.

    ros2 run truck_bringup collision_guard              # sim (raw = rectified)
    ros2 run truck_bringup collision_guard --ros-args -p left_topic:=stereo/left/image_rect_color \\
        -p right_topic:=stereo/right/image_rect          # real cameras

Block matching on a downscaled stereo pair (cheap enough for the Nano's CPU)
gives a 3D point per pixel; with the camera's height and pitch each point
gets a height above the floor. Points 5 cm to 0.6 m tall in the truck's lane
are obstacles - the floor drops out, and low things below the lidar's scan
plane are caught. Publishes safety/stereo/forward_limit and front_distance
(from the front bumper, see truck_common/guards.py). It only looks forward;
reversing is covered by the lidar and map guards.
"""
import math

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from truck_bringup.stereo_pair import Disparity, StereoPair, intrinsics
from truck_common.clouds import thin, xy_to_cloud
from truck_common.guards import GuardPublisher, corridor_distances
from truck_common.images import gray, image_to_bgr
from truck_common.ros import run_node


def obstacle_points(disp, fx, fy, cx, cy, baseline, cam_height, pitch, cam_x=0.0,
                    min_height=0.05, max_height=0.37, max_range=1.5, step=2):
    """Obstacle points (Nx2, truck frame: x forward from base_link, y left) from
    a disparity map, selected by HEIGHT above the floor.

    Every valid disparity pixel becomes a 3D point; with the camera's height
    and downward pitch it gets a height above the floor. Points between
    min_height (5 cm = 2 in) and max_height (the truck's own height + margin)
    up to max_range ahead are obstacles - the floor drops out, low things are
    kept, and a table top the truck fits under is ignored.
    """
    d = disp[::step, ::step]
    vs, us = np.nonzero(d > 1.0)
    if len(vs) == 0 or fx <= 0 or baseline <= 0:
        return np.zeros((0, 2))
    z = fx * baseline / d[vs, us]                  # optical depth
    x = (us * step - cx) * z / fx                  # right
    y = (vs * step - cy) * z / fy                  # down
    cp, sp = math.cos(pitch), math.sin(pitch)
    fwd = z * cp - y * sp                          # level-frame forward
    h = cam_height - (y * cp + z * sp)             # height above the floor
    keep = (h >= min_height) & (h <= max_height) & (fwd > 0) & (fwd <= max_range)
    return np.c_[cam_x + fwd[keep], -x[keep]]


class CollisionGuard(Node):
    def __init__(self):
        super().__init__('collision_guard')
        p = self.declare_parameter
        p('left_topic', 'stereo/left/image_raw')
        p('right_topic', 'stereo/right/image_raw')
        p('stop_distance', 0.30)        # m from the front bumper
        p('slow_distance', 1.10)
        p('max_speed', 1.0)             # m/s at slow_distance (ramp top)
        p('cam_x', 0.16)                # m, cameras ahead of base_link
        p('front_x', 0.25)              # m, front bumper ahead of base_link
        p('rate_hz', 8.0)
        p('scale', 0.5)                 # downscale before matching (CPU)
        p('half_width', 0.20)           # m, corridor half width (truck + margin)
        p('cam_height', 0.25)           # m above the floor
        p('cam_pitch', 0.10)            # rad, down
        p('max_range', 1.5)             # m (stereo depth error grows fast beyond this)
        p('min_height', 0.05)           # m: 2 in - lower than this is "floor"
        p('max_height', 0.37)           # m: the truck's height + margin - it fits under anything higher
        p('min_points', 40)             # obstacle pixels needed (noise rejection)
        g = self._g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._out = GuardPublisher(self, 'stereo', rear=False)
        self._points = self.create_publisher(PointCloud2, 'safety/stereo/points', 10)
        self._disparity = Disparity()
        self._logged = False
        StereoPair(self, g('left_topic'), g('right_topic'), self._process, g('rate_hz'))

    def _process(self, lmsg, rmsg, linfo, rinfo):
        g = self._g
        s = g('scale')
        gl, gr = gray(image_to_bgr(lmsg), s), gray(image_to_bgr(rmsg), s)
        k = intrinsics(linfo, rinfo, s)
        if not self._logged:
            self._logged = True
            self.get_logger().info(
                'guard: obstacles {:.2f}-{:.2f} m tall within {:.2f} m of the centre line, '
                'up to {:.1f} m ahead ({}x{}, fx {:.0f}, baseline {:.3f} m)'.format(
                    g('min_height'), g('max_height'), g('half_width'), g('max_range'),
                    gl.shape[1], gl.shape[0], k.fx, k.baseline))
        pts = obstacle_points(self._disparity(gl, gr), k.fx, k.fy, k.cx, k.cy, k.baseline,
                              g('cam_height'), g('cam_pitch'), g('cam_x'), g('min_height'),
                              g('max_height'), g('max_range'))
        dist, _ = corridor_distances(pts, g('front_x'), -1e9, g('half_width'), g('min_points'))
        self._points.publish(xy_to_cloud(thin(pts, cell=0.01, min_hits=2), 'base_link', lmsg.header.stamp))
        self._out.publish(dist, None, g('stop_distance'), g('slow_distance'), g('max_speed'))


def main(args=None):
    return run_node(CollisionGuard, args)
