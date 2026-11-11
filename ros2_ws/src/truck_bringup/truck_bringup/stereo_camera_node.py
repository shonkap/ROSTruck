"""Two cameras -> one ROS stereo pair (Jetson side). Python 3.6 compatible.

Publishes (same topics as the simulator, so SLAM launch files are shared):
    stereo/left/image_raw, stereo/left/camera_info
    stereo/right/image_raw, stereo/right/camera_info
    services stereo/{left,right}/set_camera_info  (the calibrator's COMMIT button)

Both images of a pair carry the SAME timestamp. Two separate USB/CSI cameras
are not hardware-synchronised: each runs on its own clock with a random,
fixed phase offset (0 .. one frame period). So each camera is read in its own
thread into a short buffer, and the CLOSEST-in-time frames are paired; a pair
is published only if they were captured within max_skew_ms (default: half a
frame period, the best free-running cameras can guarantee). At 0.3 m/s,
10 ms of skew = 3 mm of motion between the two views - fine for mapping at
walking pace; for faster driving use a hardware-synchronised stereo camera.

Camera sources (parameters left_source / right_source):
    "0", "1", "/dev/video0"   V4L2 (USB webcam), MJPEG requested
    "csi:0", "csi:1"          Jetson CSI camera (IMX219 etc.) via nvarguscamerasrc
    "gst:<pipeline>"          any GStreamer pipeline ending in appsink
Single-device stereo cameras (one USB device giving one side-by-side frame,
e.g. ELP / Arducam stereo modules): set mode:=side_by_side and left_source only.
"""
import os
import threading
import time
from collections import deque

import cv2
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from sensor_msgs.srv import SetCameraInfo

from truck_bringup import camera_info_yaml


def csi_pipeline(sensor_id, width, height, fps, flip=0):
    return ('nvarguscamerasrc sensor-id={sid} ! '
            'video/x-raw(memory:NVMM),width={w},height={h},framerate={fps}/1,format=NV12 ! '
            'nvvidconv flip-method={flip} ! video/x-raw,format=BGRx ! '
            'videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false'
            ).format(sid=sensor_id, w=width, h=height, fps=fps, flip=flip)


def open_capture(source, width, height, fps, flip=0):
    if source.startswith('csi:'):
        return cv2.VideoCapture(csi_pipeline(int(source[4:]), width, height, fps, flip),
                                cv2.CAP_GSTREAMER)
    if source.startswith('gst:'):
        return cv2.VideoCapture(source[4:], cv2.CAP_GSTREAMER)
    dev = int(source) if source.isdigit() else source
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    # MJPEG: two YUYV 640x480@30 streams don't fit on one USB2 bus; MJPEG does.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def best_pair(left, right, last_left_seq, last_right_seq, max_skew):
    """left/right: [(stamp, seq), ...]. Returns (i_left, i_right, skew) of the
    best unpublished pair - newest left frame that has a right frame within
    max_skew, closest match first - or None."""
    best = None
    for i in sorted(range(len(left)), key=lambda k: -left[k][1]):     # newest left first
        tl, sl = left[i]
        if sl <= last_left_seq:
            break
        for j, (tr, sr) in enumerate(right):
            if sr <= last_right_seq:
                continue
            skew = abs(tl - tr)
            if skew <= max_skew and (best is None or skew < best[2]):
                best = (i, j, skew)
        if best is not None:
            return best
    return None


class Grabber(threading.Thread):
    """Reads one camera as fast as it delivers; keeps the last few frames."""

    def __init__(self, name, cap):
        threading.Thread.__init__(self, name=name)
        self.daemon = True
        self.cap = cap
        self.lock = threading.Lock()
        self.frame = None
        self.stamp = 0.0     # time.time() right after the frame was grabbed
        self.seq = 0
        self.running = True
        self.failures = 0
        self.buf = deque(maxlen=4)   # (frame, stamp, seq)

    def run(self):
        while self.running:
            ok = self.cap.grab()
            t = time.time()
            if not ok:
                self.failures += 1
                time.sleep(0.05)
                continue
            ok, frame = self.cap.retrieve()
            if not ok:
                self.failures += 1
                continue
            with self.lock:
                self.frame, self.stamp = frame, t
                self.seq += 1
                self.buf.append((frame, t, self.seq))

    def latest(self):
        with self.lock:
            return self.frame, self.stamp, self.seq

    def buffer(self):
        with self.lock:
            return list(self.buf)


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')
        p = self.declare_parameter
        p('mode', 'dual')                 # dual | side_by_side
        p('left_source', '0')
        p('right_source', '1')
        p('width', 640)                   # per camera
        p('height', 480)
        p('fps', 15)
        p('flip', 0)                      # CSI flip-method (2 = rotate 180)
        p('swap', False)                  # swap left/right if wired backwards
        p('max_skew_ms', 0.0)             # 0 = auto: half a frame period
        p('left_calibration', '')         # camera_calibration YAML paths
        p('right_calibration', '')
        p('publish_compressed', False)    # JPEG copies for streaming to a PC over WiFi
        p('jpeg_quality', 90)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self._mode = g('mode')
        self._w, self._h, self._fps = g('width'), g('height'), g('fps')
        self._max_skew = (g('max_skew_ms') or 500.0 / max(1, g('fps'))) / 1000.0
        self._calib_path = {'left': os.path.expanduser(g('left_calibration')),
                            'right': os.path.expanduser(g('right_calibration'))}

        self._info = {}
        for side in ('left', 'right'):
            info = CameraInfo()
            info.header.frame_id = '{}_camera_optical_frame'.format(side)
            info.width, info.height = self._w, self._h
            path = self._calib_path[side]
            if path and os.path.exists(path):
                camera_info_yaml.fill_camera_info(info, camera_info_yaml.load(path))
                self.get_logger().info('{} calibration: {}'.format(side, path))
            else:
                self.get_logger().warn(
                    '{} camera is UNCALIBRATED ({}): fine for calibrating, not for SLAM. '
                    'Run scripts/stereo_calibrate.py.'.format(side, path or 'no file set'))
            self._info[side] = info

        self._img_pub = {s: self.create_publisher(Image, 'stereo/{}/image_raw'.format(s),
                                                  qos_profile_sensor_data)
                         for s in ('left', 'right')}
        self._info_pub = {s: self.create_publisher(CameraInfo, 'stereo/{}/camera_info'.format(s),
                                                   qos_profile_sensor_data)
                          for s in ('left', 'right')}
        self._jpeg_quality = int(g('jpeg_quality'))
        self._cmp_pub = {}
        if g('publish_compressed'):
            self._cmp_pub = {s: self.create_publisher(
                CompressedImage, 'stereo/{}/image_raw/compressed'.format(s), qos_profile_sensor_data)
                for s in ('left', 'right')}
        for side in ('left', 'right'):
            self.create_service(SetCameraInfo, 'stereo/{}/set_camera_info'.format(side),
                                lambda req, resp, s=side: self._set_camera_info(s, req, resp))

        flip = g('flip')
        if self._mode == 'side_by_side':
            cap = open_capture(g('left_source'), self._w * 2, self._h, self._fps, flip)
            self._check_open(cap, g('left_source'))
            self._grabbers = [Grabber('stereo', cap)]
        else:
            sources = [g('left_source'), g('right_source')]
            if g('swap'):
                sources.reverse()
            caps = [open_capture(s, self._w, self._h, self._fps, flip) for s in sources]
            for cap, s in zip(caps, sources):
                self._check_open(cap, s)
            self._grabbers = [Grabber('left', caps[0]), Grabber('right', caps[1])]
        for gr in self._grabbers:
            gr.start()

        self._last_seq = [-1] * len(self._grabbers)
        self._last_miss = None
        self._stats = {'pairs': 0, 'skew_drop': 0, 'max_skew': 0.0}
        self.create_timer(1.0 / (2.0 * self._fps), self._tick)
        self.create_timer(10.0, self._report)

    def _check_open(self, cap, source):
        if not cap.isOpened():
            raise RuntimeError("can't open camera '{}' (try: v4l2-ctl --list-devices)".format(source))

    def _to_msg(self, frame, stamp, side):
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = '{}_camera_optical_frame'.format(side)
        img.height, img.width = frame.shape[0], frame.shape[1]
        img.encoding = 'bgr8'
        img.step = img.width * 3
        img.data = frame.tobytes()
        return img

    def _tick(self):
        if len(self._grabbers) == 2:
            lb, rb = self._grabbers[0].buffer(), self._grabbers[1].buffer()
            if not lb or not rb:
                return
            found = best_pair([(f[1], f[2]) for f in lb], [(f[1], f[2]) for f in rb],
                              self._last_seq[0], self._last_seq[1], self._max_skew)
            if found is None:
                # Both cameras moved on and nothing matched: count it once.
                newest = (lb[-1][2], rb[-1][2])
                if newest != self._last_miss and lb[0][2] > self._last_seq[0] and rb[0][2] > self._last_seq[1]:
                    self._stats['skew_drop'] += 1
                    self._last_miss = newest
                return
            i, j, skew = found
            self._stats['max_skew'] = max(self._stats['max_skew'], skew)
            left, right = lb[i][0], rb[j][0]
            t = (lb[i][1] + rb[j][1]) / 2.0
            seqs = [lb[i][2], rb[j][2]]
        else:
            frames = [gr.latest() for gr in self._grabbers]
            if frames[0][0] is None or frames[0][2] == self._last_seq[0]:
                return
            seqs = [frames[0][2]]
            sbs = frames[0][0]
            half = sbs.shape[1] // 2
            left, right = sbs[:, :half], sbs[:, half:]
            t = frames[0][1]
        self._last_seq = seqs
        # Use the capture time, not "now", so SLAM sees when the photons arrived.
        stamp = rclpy.time.Time(seconds=t).to_msg()
        for side, frame in (('left', left), ('right', right)):
            self._img_pub[side].publish(self._to_msg(frame, stamp, side))
            if self._cmp_pub:
                ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
                if ok:
                    cm = CompressedImage()
                    cm.header.stamp = stamp
                    cm.header.frame_id = '{}_camera_optical_frame'.format(side)
                    cm.format = 'jpeg'
                    cm.data = jpg.tobytes()
                    self._cmp_pub[side].publish(cm)
            info = self._info[side]
            info.header.stamp = stamp
            self._info_pub[side].publish(info)
        self._stats['pairs'] += 1

    def _report(self):
        s = self._stats
        fails = sum(gr.failures for gr in self._grabbers)
        self.get_logger().info(
            'stereo {:.1f} Hz, max skew {:.1f} ms, {} pairs dropped for skew, {} grab failures'.format(
                s['pairs'] / 10.0, s['max_skew'] * 1000.0, s['skew_drop'], fails))
        self._stats = {'pairs': 0, 'skew_drop': 0, 'max_skew': 0.0}

    def _set_camera_info(self, side, req, resp):
        ci = req.camera_info
        path = self._calib_path[side] or os.path.expanduser(
            '~/.ros/camera_info/{}.yaml'.format(side))
        try:
            d = os.path.dirname(path)
            if d and not os.path.isdir(d):
                os.makedirs(d)
            camera_info_yaml.save(path, ci.width, ci.height, side, list(ci.k), list(ci.d),
                                  list(ci.r), list(ci.p), ci.distortion_model or 'plumb_bob')
            ci.header.frame_id = self._info[side].header.frame_id
            self._info[side] = ci
            resp.success = True
            resp.status_message = 'saved ' + path
            self.get_logger().info('{} calibration saved to {}'.format(side, path))
        except Exception as exc:
            resp.success = False
            resp.status_message = str(exc)
            self.get_logger().error('saving {} calibration failed: {}'.format(side, exc))
        return resp

    def destroy_node(self):
        for gr in self._grabbers:
            gr.running = False
        for gr in self._grabbers:
            gr.join(timeout=1.0)
            gr.cap.release()
        Node.destroy_node(self)


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
