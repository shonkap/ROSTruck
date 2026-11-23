"""sensor_msgs/Image helpers (no cv_bridge, which isn't in every image). Python 3.6."""
import cv2
import numpy as np


def image_to_bgr(msg):
    """bgr8 / rgb8 / mono8 Image -> BGR numpy array (copy)."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('bgr8', 'rgb8'):
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img.copy()
    if msg.encoding in ('mono8', '8UC1'):
        g = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    raise ValueError('unsupported encoding ' + msg.encoding)


def gray(img, scale=1.0):
    """BGR -> grey, optionally downscaled (INTER_AREA)."""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if scale != 1.0:
        g = cv2.resize(g, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return g
