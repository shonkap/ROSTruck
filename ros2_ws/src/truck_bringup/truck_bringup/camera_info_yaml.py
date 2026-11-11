"""Read/write camera calibration YAML in the ROS camera_calibration format.

Pure Python (no rclpy) so it is unit-testable and shared by the camera node,
the calibration wizard and the stereo checker. Python 3.6 compatible.
"""
import yaml


def _matrix(rows, cols, data):
    return {'rows': rows, 'cols': cols, 'data': [float(v) for v in data]}


def load(path):
    """Returns a dict with width, height, name, K(9), D(n), R(9), P(12), model."""
    with open(path) as f:
        y = yaml.safe_load(f)
    return {
        'width': int(y['image_width']),
        'height': int(y['image_height']),
        'name': y.get('camera_name', ''),
        'K': [float(v) for v in y['camera_matrix']['data']],
        'D': [float(v) for v in y['distortion_coefficients']['data']],
        'R': [float(v) for v in y['rectification_matrix']['data']],
        'P': [float(v) for v in y['projection_matrix']['data']],
        'model': y.get('distortion_model', 'plumb_bob'),
    }


def save(path, width, height, name, K, D, R, P, model='plumb_bob'):
    data = {
        'image_width': int(width),
        'image_height': int(height),
        'camera_name': name,
        'camera_matrix': _matrix(3, 3, K),
        'distortion_model': model,
        'distortion_coefficients': _matrix(1, len(D), D),
        'rectification_matrix': _matrix(3, 3, R),
        'projection_matrix': _matrix(3, 4, P),
    }
    with open(path, 'w') as f:
        try:
            yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False)
        except TypeError:   # PyYAML < 5.1 (Ubuntu 18.04 / Jetson) has no sort_keys
            f.seek(0)
            f.truncate()
            yaml.safe_dump(data, f, default_flow_style=None)


def baseline_m(right_P):
    """Stereo baseline from the right camera's projection matrix (P[3] = -fx * B)."""
    fx = right_P[0]
    if fx == 0:
        return 0.0
    return -right_P[3] / fx


def fill_camera_info(msg, calib):
    """Copy a loaded calibration into a sensor_msgs/CameraInfo."""
    msg.width = calib['width']
    msg.height = calib['height']
    msg.distortion_model = calib['model']
    msg.k = calib['K']
    msg.d = calib['D']
    msg.r = calib['R']
    msg.p = calib['P']
    return msg
