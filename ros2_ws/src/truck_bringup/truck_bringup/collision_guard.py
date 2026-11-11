"""Collision guard: stereo depth -> forward speed limit. Python 3.6.

A safety layer under every other module: whatever publishes cmd_vel (web
waypoints, explorer, an LLM plan, teleop), fc_bridge caps the forward speed
at safety/speed_limit, which this node lowers as obstacles get close in front:

    > slow_distance        full speed (no limit published as a big number)
    stop..slow_distance    linear ramp
    < stop_distance        0 forward (reversing and turning still allowed,
                           so the navigator's back-up / 3-point turn work)

Depth comes from block matching on a downscaled stereo pair (cheap enough
for the Nano's CPU), in the image band the truck will drive into. Also
publishes safety/obstacle_distance (m) for the web page and other modules.

    ros2 run truck_bringup collision_guard              # sim (raw = rectified)
    ros2 run truck_bringup collision_guard --ros-args -p left_topic:=stereo/left/image_rect_color \\
        -p right_topic:=stereo/right/image_rect          # real cameras
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32

from truck_bringup.camera_info_yaml import baseline_m
from truck_bringup.stereo_check import image_to_bgr

NO_LIMIT = 99.0


def nearest_obstacle(disp, fx, baseline, roi, min_pixels=150, percentile=95.0):
    """Distance (m) to the nearest substantial thing in roi of a disparity map
    (float, px; <= 0 = no match). roi = (x0, y0, x1, y1). None if nothing seen.

    Uses a high percentile of disparity (= near), not the max, so a few bad
    matches don't trigger a stop; needs min_pixels valid pixels."""
    x0, y0, x1, y1 = roi
    d = disp[y0:y1, x0:x1]
    valid = d[d > 0.5]
    if valid.size < min_pixels or fx <= 0 or baseline <= 0:
        return None
    near = float(np.percentile(valid, percentile))
    return fx * baseline / near


def speed_limit(distance, stop_distance, slow_distance, max_speed):
    if distance is None:
        return NO_LIMIT
    if distance <= stop_distance:
        return 0.0
    if distance >= slow_distance:
        return NO_LIMIT
    return max_speed * (distance - stop_distance) / (slow_distance - stop_distance)


def path_roi(width, height, fx, cx, cy, cam_height, half_width, pitch, max_range):
    """Image region covering the truck's corridor (half_width either side)
    from the floor up to ~body height, out to max_range. Pixel box."""
    # columns: corridor edges at max_range (widest in image near the camera
    # anyway, so use a fixed angular width at a mid distance)
    mid = max(0.6, max_range / 2.0)
    half_px = int(fx * half_width / mid)
    x0, x1 = max(0, int(cx) - half_px), min(width, int(cx) + half_px)
    # rows: from the horizon-ish (a bit above, things taller than the truck)
    # down to where the floor is ~stop range away; exclude the floor itself
    floor_row = cy + fx * math.tan(pitch + math.atan2(cam_height, max_range))
    y0 = max(0, int(cy - fx * math.tan(0.15) + fx * math.tan(pitch)))
    y1 = min(height, int(floor_row))
    if y1 - y0 < 8:
        y0, y1 = int(height * 0.3), int(height * 0.7)
    return x0, y0, x1, y1


class CollisionGuard(Node):
    def __init__(self):
        super().__init__('collision_guard')
        p = self.declare_parameter
        p('left_topic', 'stereo/left/image_raw')
        p('right_topic', 'stereo/right/image_raw')
        p('stop_distance', 0.40)        # m, from the cameras
        p('slow_distance', 1.20)
        p('max_speed', 1.0)             # m/s at slow_distance (ramp top)
        p('rate_hz', 8.0)
        p('scale', 0.5)                 # downscale before matching (CPU)
        p('half_width', 0.20)           # m, corridor half width (truck + margin)
        p('cam_height', 0.25)           # m above the floor
        p('cam_pitch', 0.10)            # rad, down
        p('max_range', 2.0)             # m
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._g = g
        self._info = {'left': None, 'right': None}
        self._img = {'left': None, 'right': None}
        self._last_t = 0.0
        self._pub_limit = self.create_publisher(Float32, 'safety/speed_limit', 10)
        self._pub_dist = self.create_publisher(Float32, 'safety/obstacle_distance', 10)
        for side in ('left', 'right'):
            topic = g(side + '_topic')
            info_topic = topic.rsplit('/', 1)[0] + '/camera_info'
            self.create_subscription(CameraInfo, info_topic,
                                     lambda m, s=side: self._info.__setitem__(s, m), qos_profile_sensor_data)
            self.create_subscription(Image, topic, lambda m, s=side: self._on_img(s, m),
                                     qos_profile_sensor_data)
        self._matcher = None
        self._roi = None
        self._last_dist = None
        self.create_timer(2.0, self._report)

    def _on_img(self, side, msg):
        self._img[side] = msg
        l, r = self._img['left'], self._img['right']
        if l is None or r is None or self._info['left'] is None or self._info['right'] is None:
            return
        if (l.header.stamp.sec, l.header.stamp.nanosec) != (r.header.stamp.sec, r.header.stamp.nanosec):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_t < 1.0 / self._g('rate_hz'):
            return
        self._last_t = now
        self._img = {'left': None, 'right': None}
        self._process(l, r)

    def _process(self, lmsg, rmsg):
        g = self._g
        s = g('scale')
        gl = cv2.cvtColor(image_to_bgr(lmsg), cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(image_to_bgr(rmsg), cv2.COLOR_BGR2GRAY)
        if s != 1.0:
            gl = cv2.resize(gl, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
            gr = cv2.resize(gr, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        il, ir = self._info['left'], self._info['right']
        fx, cx, cy = il.p[0] * s, il.p[2] * s, il.p[6] * s
        B = baseline_m(ir.p)
        if self._matcher is None:
            self._matcher = cv2.StereoBM_create(numDisparities=64, blockSize=11)
            self._matcher.setUniquenessRatio(12)
            self._matcher.setTextureThreshold(10)
            self._roi = path_roi(gl.shape[1], gl.shape[0], fx, cx, cy, g('cam_height'),
                                 g('half_width'), g('cam_pitch'), g('max_range'))
            self.get_logger().info('guard ROI {} on {}x{} (fx {:.0f}, baseline {:.3f} m)'.format(
                self._roi, gl.shape[1], gl.shape[0], fx, B))
        disp = self._matcher.compute(gl, gr).astype(np.float32) / 16.0
        dist = nearest_obstacle(disp, fx, B, self._roi)
        self._last_dist = dist
        lim = speed_limit(dist, g('stop_distance'), g('slow_distance'), g('max_speed'))
        self._pub_limit.publish(Float32(data=float(lim)))
        self._pub_dist.publish(Float32(data=float(dist) if dist is not None else NO_LIMIT))

    def _report(self):
        if self._last_dist is not None and self._last_dist < self._g('slow_distance'):
            self.get_logger().info('obstacle {:.2f} m ahead'.format(self._last_dist),
                                   throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
