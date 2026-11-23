"""Object detection plugin: camera -> detections -> map landmarks (+ optional tasks).

    ros2 run truck_bringup detector_node                          # demo colour detector
    ros2 run truck_bringup detector_node --ros-args -p backend:=onnx \\
        -p model:=/models/yolov8n.onnx -p classes:="[person, chair, couch, ...]"

Pipeline (each frame, at rate_hz):
  1. detect objects in the left (rectified) image      -> Detection(label, conf, bbox)
  2. range them with stereo disparity inside the box  -> 3D point, optical frame
  3. transform to the SLAM map through TF             -> (x, y) on the map
  4. merge repeated sightings into landmarks          -> landmarks topic (1 Hz)
  5. rules: turn detections into tasks                -> tasks/submit

Topics out:
  detections  std_msgs/String JSON  [{label, confidence, bbox, distance, x, y}]  every frame
  object_landmarks  std_msgs/String JSON [{label, x, y, sightings, confidence}]  1 Hz
              (the LLM planner reads these: "go to the couch")
  tasks/submit when a rule fires, e.g. rules:='[{"label": "person", "max_distance": 1.0,
              "task": {"type": "stop", "priority": 10}, "cooldown_s": 10}]'

Your own model: write a class with detect(bgr) -> [Detection] in detection.py
and add it to make_detector() below. Python 3.6 compatible.
"""
import json
import time

from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

from truck_bringup.detection import (ColorBlobDetector, LandmarkMap, OnnxDetector, bbox_point,
                                     ground_point, quat_rotate, rule_matches)
from truck_bringup.stereo_pair import Disparity, StereoPair, intrinsics
from truck_common.images import gray, image_to_bgr
from truck_common.ros import Tf, jmsg, run_node


def make_detector(node):
    g = lambda n: node.get_parameter(n).value  # noqa: E731
    backend = g('backend')
    if backend == 'onnx':
        return OnnxDetector(g('model'), g('classes'), conf=g('min_confidence'))
    if backend == 'color':
        return ColorBlobDetector(label=g('color_label'))
    raise ValueError('unknown detector backend ' + backend)


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector')
        p = self.declare_parameter
        p('backend', 'color')                    # color | onnx
        p('model', '')                           # onnx model path
        p('classes', ['object'])                 # onnx class names
        p('min_confidence', 0.4)
        p('color_label', 'red_bin')              # label for the demo colour detector
        p('left_topic', 'stereo/left/image_raw')
        p('right_topic', 'stereo/right/image_raw')
        p('map_frame', 'map')
        p('rate_hz', 3.0)
        p('max_range', 4.0)                      # m, ignore far (noisy) stereo
        p('cam_height', 0.25)                    # m, for ranging by the floor contact point
        p('cam_pitch', 0.10)                     # rad, down
        p('rules', '[]')                         # JSON list, see module doc
        p('min_sightings', 6)                    # confirm a landmark after this many consistent sightings
        p('max_spread', 0.3)                     # m, sightings must agree this well
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._g = g
        self._detector = make_detector(self)
        self._rules = json.loads(g('rules') or '[]')
        self._rule_fired = {}
        self._landmarks = LandmarkMap(min_sightings=g('min_sightings'), max_spread=g('max_spread'))
        self._disparity = Disparity()
        self._tf = Tf(self)
        self._det_pub = self.create_publisher(String, 'detections', 10)
        # not 'landmarks': RTAB-Map already publishes a PoseArray there
        self._lm_pub = self.create_publisher(String, 'object_landmarks', 10)
        self._cand_pub = self.create_publisher(String, 'object_landmarks/candidates', 10)
        self._task_pub = self.create_publisher(String, 'tasks/submit', 10)
        StereoPair(self, g('left_topic'), g('right_topic'), self._process, g('rate_hz'))
        self.create_timer(1.0, self._publish_landmarks)
        self.get_logger().info('detector: backend {}, {} rule(s)'.format(g('backend'), len(self._rules)))

    def _process(self, lmsg, rmsg, linfo, rinfo):
        left = image_to_bgr(lmsg)
        dets = self._detector.detect(left)
        if not dets:
            self._det_pub.publish(String(data='[]'))
            return
        k = intrinsics(linfo, rinfo)
        s = 0.5
        disp = self._disparity(gray(left, s), gray(image_to_bgr(rmsg), s))
        # at the image's time; sim/real timing jitter: fall back to the latest
        frame = (self._g('map_frame'), lmsg.header.frame_id)
        tf = self._tf.lookup(*frame, stamp=Time.from_msg(lmsg.header.stamp)) or self._tf.lookup(*frame)
        out = []
        t = time.time()
        for d in dets:
            item = {'label': d.label, 'confidence': d.confidence, 'bbox': list(d.bbox),
                    'distance': None, 'x': None, 'y': None}
            pt = bbox_point(d.bbox, disp, k.fx, k.fy, k.cx, k.cy, k.baseline, scale=s)
            item['ranged_by'] = 'stereo'
            if pt is None:   # texture-less or at the image edge: use the floor contact point
                pt = ground_point(d.bbox, k.fx, k.fy, k.cx, k.cy, self._g('cam_height'), self._g('cam_pitch'))
                item['ranged_by'] = 'floor'
            if pt is not None and pt[2] <= self._g('max_range'):
                item['distance'] = round(pt[2], 2)
                if tf is not None:
                    tr, q = tf.transform.translation, tf.transform.rotation
                    rx, ry, _ = quat_rotate((q.x, q.y, q.z, q.w), pt)
                    item['x'], item['y'] = round(tr.x + rx, 2), round(tr.y + ry, 2)
                    self._landmarks.add(d.label, item['x'], item['y'], d.confidence, t)
            out.append(item)
            self._apply_rules(item, t)
        self._det_pub.publish(jmsg(out))

    def _apply_rules(self, det, t):
        for i, rule in enumerate(self._rules):
            if not rule_matches(rule, det['label'], det['distance']):
                continue
            if t - self._rule_fired.get(i, -1e9) < rule.get('cooldown_s', 10.0):
                continue
            self._rule_fired[i] = t
            task = dict(rule.get('task', {}), source='detector')
            task.setdefault('note', '{} at {} m'.format(det['label'], det['distance']))
            self._task_pub.publish(jmsg(task))
            self.get_logger().warn('rule fired: {} -> {}'.format(det['label'], task.get('type')))

    def _publish_landmarks(self):
        self._lm_pub.publish(jmsg(self._landmarks.confirmed()))
        self._cand_pub.publish(jmsg(self._landmarks.candidates()))


def main(args=None):
    return run_node(DetectorNode, args)
