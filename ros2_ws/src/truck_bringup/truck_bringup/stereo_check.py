"""Check a stereo calibration on live images. Python 3.6 compatible.

    ros2 run truck_bringup stereo_check                  # uses stereo/*/camera_info
    ros2 run truck_bringup stereo_check --ros-args -p board:=9x6 -p square:=0.025
    ros2 run truck_bringup stereo_check --ros-args -p preview:=/tmp/stereo_check.png

What it measures (on N image pairs):
  * epipolar error: after rectification a point must be on the SAME pixel row
    in both images. Mean |row_left - row_right| of matched features (ORB on
    any textured scene, or checkerboard corners if a board is visible).
       < 0.5 px  good    0.5 - 1.0 px  usable    > 1.0 px  recalibrate
  * baseline (from the right camera's P matrix) vs. what you measured
  * disparity coverage: % of pixels where block matching finds depth
  * with a checkerboard: distance to the board from stereo vs. from its size
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from truck_bringup.camera_info_yaml import baseline_m


def image_to_bgr(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('bgr8', 'rgb8'):
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img.copy()
    if msg.encoding in ('mono8', '8UC1'):
        g = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    raise ValueError('unsupported encoding ' + msg.encoding)


def rectify_maps(info):
    K = np.array(info.k, dtype=np.float64).reshape(3, 3)
    D = np.array(info.d, dtype=np.float64)
    R = np.array(info.r, dtype=np.float64).reshape(3, 3)
    P = np.array(info.p, dtype=np.float64).reshape(3, 4)
    if not R.any():
        R = np.eye(3)
    return cv2.initUndistortRectifyMap(K, D, R, P[:, :3], (info.width, info.height), cv2.CV_32FC1)


def row_error_features(left_gray, right_gray):
    """Median/mean |dy| of ORB matches that look like true stereo matches."""
    orb = cv2.ORB_create(1500)
    kl, dl = orb.detectAndCompute(left_gray, None)
    kr, dr = orb.detectAndCompute(right_gray, None)
    if dl is None or dr is None:
        return None
    matches = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(dl, dr)
    dys = []
    for m in matches:
        (xl, yl), (xr, yr) = kl[m.queryIdx].pt, kr[m.trainIdx].pt
        # true stereo matches: right point is left of the left point, and close in row
        if 0.5 < xl - xr < 200 and abs(yl - yr) < 8 and m.distance < 40:
            dys.append(abs(yl - yr))
    if len(dys) < 20:
        return None
    return float(np.mean(dys)), float(np.median(dys)), len(dys)


def row_error_board(left_gray, right_gray, board):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    okl, cl = cv2.findChessboardCorners(left_gray, board, flags=flags)
    okr, cr = cv2.findChessboardCorners(right_gray, board, flags=flags)
    if not (okl and okr):
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    cl = cv2.cornerSubPix(left_gray, cl, (5, 5), (-1, -1), crit).reshape(-1, 2)
    cr = cv2.cornerSubPix(right_gray, cr, (5, 5), (-1, -1), crit).reshape(-1, 2)
    return cl, cr


class StereoCheck(Node):
    def __init__(self):
        super().__init__('stereo_check')
        self.declare_parameter('pairs', 20)
        self.declare_parameter('board', '')          # e.g. "9x6" inner corners
        self.declare_parameter('square', 0.0)        # m, checkerboard square size
        self.declare_parameter('expected_baseline', 0.0)  # m, what you measured
        self.declare_parameter('preview', '')        # PNG path for a visual check
        self._n = int(self.get_parameter('pairs').value)
        b = self.get_parameter('board').value
        self._board = tuple(int(v) for v in b.lower().split('x')) if b else None
        self._square = float(self.get_parameter('square').value)
        self._expected_b = float(self.get_parameter('expected_baseline').value)
        self._preview = self.get_parameter('preview').value
        self._info = {'left': None, 'right': None}
        self._img = {'left': None, 'right': None}
        self._maps = None
        self._feat, self._board_err, self._board_dist, self._cover = [], [], [], []
        self.done = False
        for s in ('left', 'right'):
            self.create_subscription(CameraInfo, 'stereo/{}/camera_info'.format(s),
                                     lambda m, s=s: self._on_info(s, m), qos_profile_sensor_data)
            self.create_subscription(Image, 'stereo/{}/image_raw'.format(s),
                                     lambda m, s=s: self._on_img(s, m), qos_profile_sensor_data)
        self.get_logger().info('collecting {} stereo pairs...'.format(self._n))

    def _on_info(self, side, msg):
        self._info[side] = msg

    def _on_img(self, side, msg):
        self._img[side] = msg
        l, r = self._img['left'], self._img['right']
        if l is None or r is None or self._info['left'] is None or self._info['right'] is None:
            return
        if (l.header.stamp.sec, l.header.stamp.nanosec) != (r.header.stamp.sec, r.header.stamp.nanosec):
            return
        self._img = {'left': None, 'right': None}
        self._process(image_to_bgr(l), image_to_bgr(r))

    def _process(self, left, right):
        il, ir = self._info['left'], self._info['right']
        if not any(il.k):
            self.get_logger().error('camera_info has no calibration (K = 0). Calibrate first.')
            self.done = True
            return
        if self._maps is None:
            self._maps = (rectify_maps(il), rectify_maps(ir))
        (lx, ly), (rx, ry) = self._maps
        rl = cv2.remap(left, lx, ly, cv2.INTER_LINEAR)
        rr = cv2.remap(right, rx, ry, cv2.INTER_LINEAR)
        gl, gr = cv2.cvtColor(rl, cv2.COLOR_BGR2GRAY), cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)

        fe = row_error_features(gl, gr)
        if fe is not None:
            self._feat.append(fe)
        fx, B = il.p[0], baseline_m(ir.p)
        if self._board:
            res = row_error_board(gl, gr, self._board)
            if res is not None:
                cl, cr = res
                self._board_err.append(float(np.mean(np.abs(cl[:, 1] - cr[:, 1]))))
                disp = float(np.mean(cl[:, 0] - cr[:, 0]))
                if disp > 0 and B > 0:
                    z_stereo = fx * B / disp
                    if self._square > 0:   # size-based distance from corner spacing
                        span = np.linalg.norm(cl[0] - cl[self._board[0] - 1])
                        z_size = fx * self._square * (self._board[0] - 1) / max(span, 1e-6)
                        self._board_dist.append((z_stereo, z_size))
        sgbm = cv2.StereoSGBM_create(0, 128, 7, uniquenessRatio=10, speckleWindowSize=50,
                                     speckleRange=2)
        disp = sgbm.compute(gl, gr).astype(np.float32) / 16.0
        self._cover.append(float(np.count_nonzero(disp > 0.5)) / disp.size)

        if self._preview and len(self._cover) == 1:
            sbs = np.hstack([rl, rr])
            for y in range(0, sbs.shape[0], 32):
                cv2.line(sbs, (0, y), (sbs.shape[1], y), (0, 255, 0), 1)
            dvis = cv2.applyColorMap(cv2.convertScaleAbs(disp, alpha=2.0), cv2.COLORMAP_JET)
            dvis = cv2.resize(dvis, (sbs.shape[1] // 2, sbs.shape[0] // 2))
            pad = np.zeros((dvis.shape[0], sbs.shape[1] - dvis.shape[1], 3), np.uint8)
            cv2.imwrite(self._preview, np.vstack([sbs, np.hstack([dvis, pad])]))
            self.get_logger().info('preview (rectified pair + disparity) -> ' + self._preview)

        if len(self._cover) >= self._n:
            self._summary(il, ir)

    def _summary(self, il, ir):
        log = self.get_logger().info
        fx, B = il.p[0], baseline_m(ir.p)
        hfov = 2 * math.degrees(math.atan(il.width / (2.0 * fx))) if fx else 0
        log('image {}x{}, fx {:.1f} px, horizontal FOV {:.0f} deg'.format(il.width, il.height, fx, hfov))
        line = 'baseline {:.4f} m'.format(B)
        if self._expected_b > 0:
            line += ' (you measured {:.4f} m, {:+.1f}%)'.format(
                self._expected_b, 100.0 * (B - self._expected_b) / self._expected_b)
        log(line)
        if B > 0:
            log('depth resolution: {:.1f} cm at 1 m, {:.1f} cm at 3 m (1 px disparity)'.format(
                100.0 / (fx * B), 900.0 / (fx * B)))
        verdict = 'n/a'
        errs = self._board_err or [f[0] for f in self._feat]
        if errs:
            e = float(np.mean(errs))
            verdict = 'GOOD' if e < 0.5 else ('USABLE' if e < 1.0 else 'RECALIBRATE')
            src = 'checkerboard corners' if self._board_err else 'ORB features'
            log('epipolar (row) error {:.2f} px from {} over {} pairs -> {}'.format(
                e, src, len(errs), verdict))
        else:
            log('epipolar error: not enough texture/matches - point the cameras at a busy scene')
        if self._board_dist:
            zs, zz = np.mean([d[0] for d in self._board_dist]), np.mean([d[1] for d in self._board_dist])
            log('board distance: stereo {:.3f} m vs board-size {:.3f} m ({:+.1f}%)'.format(
                zs, zz, 100.0 * (zs - zz) / zz))
        log('disparity coverage {:.0f}% of pixels'.format(100.0 * float(np.mean(self._cover))))
        print('STEREO_CHECK_RESULT {}'.format(verdict))
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = StereoCheck()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
