"""Shared stereo input for the guard, detector and stereo_check. Python 3.6.

StereoPair subscribes to a left/right image pair and their camera_info,
waits for frames with identical stamps (the camera node and the sim relay
stamp both images of a pair the same) and hands them on at most rate_hz.
"""
import time
from collections import namedtuple

import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from truck_bringup.camera_info_yaml import baseline_m

Intrinsics = namedtuple('Intrinsics', 'fx fy cx cy baseline')


def intrinsics(left_info, right_info, scale=1.0):
    """Rectified intrinsics of the left camera (scaled) + the baseline (m)."""
    p = left_info.p
    return Intrinsics(p[0] * scale, p[5] * scale, p[2] * scale, p[6] * scale, baseline_m(right_info.p))


class StereoPair(object):
    """callback(left_img, right_img, left_info, right_info) per matched pair."""

    def __init__(self, node, left_topic, right_topic, callback, rate_hz=None):
        self._cb, self._rate = callback, rate_hz
        self._info = {'left': None, 'right': None}
        self._img = {'left': None, 'right': None}
        self._last = 0.0
        for side, topic in (('left', left_topic), ('right', right_topic)):
            node.create_subscription(CameraInfo, topic.rsplit('/', 1)[0] + '/camera_info',
                                     lambda m, s=side: self._info.__setitem__(s, m), qos_profile_sensor_data)
            node.create_subscription(Image, topic, lambda m, s=side: self._on_img(s, m),
                                     qos_profile_sensor_data)

    def _on_img(self, side, msg):
        self._img[side] = msg
        l, r = self._img['left'], self._img['right']
        if l is None or r is None or None in self._info.values():
            return
        if (l.header.stamp.sec, l.header.stamp.nanosec) != (r.header.stamp.sec, r.header.stamp.nanosec):
            return
        now = time.time()
        if self._rate and now - self._last < 1.0 / self._rate:
            return
        self._last = now
        self._img = {'left': None, 'right': None}
        self._cb(l, r, self._info['left'], self._info['right'])


class Disparity(object):
    """Block matching tuned for the Nano's CPU (run on downscaled grey images)."""

    def __init__(self, num_disparities=64, block_size=11):
        self._bm = cv2.StereoBM_create(numDisparities=num_disparities, blockSize=block_size)
        self._bm.setUniquenessRatio(12)
        self._bm.setTextureThreshold(10)

    def __call__(self, left_gray, right_gray):
        """Disparity in pixels (float32; <= 0 = no match)."""
        return self._bm.compute(left_gray, right_gray).astype(np.float32) / 16.0
