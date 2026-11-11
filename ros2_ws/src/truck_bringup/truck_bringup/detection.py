"""Object detection -> map landmarks. ROS-free parts, Python 3.6.

Detectors return Detection(label, confidence, (x0, y0, x1, y1)) in pixels of
the LEFT rectified image. Swap in your own model by writing a class with a
`detect(bgr_image) -> [Detection]` method (see OnnxDetector for the shape).
"""
import math
from collections import namedtuple

import cv2
import numpy as np

Detection = namedtuple('Detection', 'label confidence bbox')


class ColorBlobDetector(object):
    """Demo detector: large, strongly saturated blobs of one hue range.

    Good for testing the whole pipeline with a coloured object (the sim room
    has a red bin). Not a real object detector - replace with your model."""

    def __init__(self, label='red_object', hue_ranges=((0, 6), (174, 180)), min_sat=190,
                 min_val=90, min_area=600):
        self.label, self.hue_ranges = label, hue_ranges
        self.min_sat, self.min_val, self.min_area = min_sat, min_val, min_area

    def detect(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.hue_ranges:
            mask |= cv2.inRange(hsv, (lo, self.min_sat, self.min_val), (hi, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        n, _, stats, _ = cv2.connectedComponentsWithStats(mask)
        out = []
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if area < self.min_area:
                continue
            fill = area / float(w * h)
            if fill < 0.4:           # thin streaks / texture scribbles
                continue
            out.append(Detection(self.label, round(min(1.0, fill), 2), (int(x), int(y), int(x + w), int(y + h))))
        return out


class OnnxDetector(object):
    """YOLO-style ONNX model through OpenCV DNN (CUDA on the Jetson if built in).

    Expects output rows [cx, cy, w, h, score_class_0, score_class_1, ...]
    (YOLOv5/v8 export, 640x640 input). Adjust parse() for other heads."""

    def __init__(self, model_path, class_names, input_size=640, conf=0.4, nms=0.45, use_cuda=True):
        self.net = cv2.dnn.readNetFromONNX(model_path)
        if use_cuda:
            try:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
            except Exception:
                pass
        self.names, self.size, self.conf, self.nms = list(class_names), input_size, conf, nms

    def detect(self, bgr):
        h, w = bgr.shape[:2]
        blob = cv2.dnn.blobFromImage(bgr, 1 / 255.0, (self.size, self.size), swapRB=True, crop=False)
        self.net.setInput(blob)
        out = self.net.forward()
        return self.parse(out, w, h)

    def parse(self, out, w, h):
        rows = out[0]
        if rows.shape[0] < rows.shape[1]:        # YOLOv8: (84, N) -> (N, 84)
            rows = rows.T
        boxes, scores, ids = [], [], []
        sx, sy = w / float(self.size), h / float(self.size)
        for r in rows:
            cls = r[4:] if rows.shape[1] == 4 + len(self.names) else r[5:] * r[4]
            k = int(np.argmax(cls))
            s = float(cls[k])
            if s < self.conf:
                continue
            cx, cy, bw, bh = r[:4]
            boxes.append([int((cx - bw / 2) * sx), int((cy - bh / 2) * sy), int(bw * sx), int(bh * sy)])
            scores.append(s)
            ids.append(k)
        keep = cv2.dnn.NMSBoxes(boxes, scores, self.conf, self.nms)
        keep = np.array(keep).flatten() if len(keep) else []
        return [Detection(self.names[ids[i]] if ids[i] < len(self.names) else str(ids[i]),
                          round(scores[i], 2),
                          (boxes[i][0], boxes[i][1], boxes[i][0] + boxes[i][2], boxes[i][1] + boxes[i][3]))
                for i in keep]


def bbox_point(bbox, disp, fx, fy, cx, cy, baseline, scale=1.0):
    """3D point (optical frame: x right, y down, z forward) of a detection,
    from the median valid disparity inside its (shrunk) box. None if no depth.
    disp may be computed on an image downscaled by `scale`."""
    x0, y0, x1, y1 = bbox
    mx, my = (x1 - x0) // 4, (y1 - y0) // 4
    sx0, sy0 = int((x0 + mx) * scale), int((y0 + my) * scale)
    sx1, sy1 = int((x1 - mx) * scale) + 1, int((y1 - my) * scale) + 1
    patch = disp[max(0, sy0):sy1, max(0, sx0):sx1]
    valid = patch[patch > 0.5]
    if valid.size < 5:
        return None
    d = float(np.median(valid)) / scale          # back to full-resolution pixels
    z = fx * baseline / d
    u, v = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    return ((u - cx) * z / fx, (v - cy) * z / fy, z)


def ground_point(bbox, fx, fy, cx, cy, cam_height, pitch):
    """3D point (optical frame) where the bottom-centre of the box meets the
    floor, for a camera cam_height above a flat floor, tilted down by pitch.
    Works on texture-less objects stereo can't range. None if the ray doesn't
    hit the floor (bottom of the box at/above the horizon)."""
    x0, y0, x1, y1 = bbox
    u, v = (x0 + x1) / 2.0, float(y1)
    rx, ry, rz = (u - cx) / fx, (v - cy) / fy, 1.0
    # optical (y down, z forward) -> level frame rotated by the pitch:
    y_level = ry * math.cos(pitch) + rz * math.sin(pitch)
    if y_level <= 0.02:
        return None
    t = cam_height / y_level
    return (rx * t, ry * t, rz * t)


def quat_rotate(q, p):
    """Rotate point p by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    px, py, pz = p
    # t = 2 * cross(q.xyz, p); p' = p + w * t + cross(q.xyz, t)
    tx, ty, tz = 2 * (y * pz - z * py), 2 * (z * px - x * pz), 2 * (x * py - y * px)
    return (px + w * tx + (y * tz - z * ty),
            py + w * ty + (z * tx - x * tz),
            pz + w * tz + (x * ty - y * tx))


class LandmarkMap(object):
    """Merges repeated sightings into stable landmarks (running average).

    A landmark is only confirmed when it was seen often enough AND the
    sightings agree (spread <= max_spread): a real object lands in the same
    place from every viewpoint, a false detection (e.g. a red shape painted on
    a wall, ranged as if it stood on the floor) jumps around."""

    def __init__(self, merge_radius=0.6, min_sightings=3, max_spread=0.3):
        self.items = []          # dicts: label, x, y, n, confidence, last, recent
        self.merge_radius, self.min_sightings = merge_radius, min_sightings
        self.max_spread = max_spread

    @staticmethod
    def spread(lm):
        pts = lm['recent']
        if len(pts) < 2:
            return 0.0
        mx = sum(p[0] for p in pts) / len(pts)
        my = sum(p[1] for p in pts) / len(pts)
        return math.sqrt(sum((p[0] - mx) ** 2 + (p[1] - my) ** 2 for p in pts) / len(pts))

    def add(self, label, x, y, confidence, t):
        for lm in self.items:
            if lm['label'] == label and math.hypot(lm['x'] - x, lm['y'] - y) < self.merge_radius:
                n = min(lm['n'], 20)
                lm['x'] = (lm['x'] * n + x) / (n + 1)
                lm['y'] = (lm['y'] * n + y) / (n + 1)
                lm['n'] += 1
                lm['confidence'] = max(lm['confidence'], confidence)
                lm['last'] = t
                lm['recent'] = (lm['recent'] + [(x, y)])[-20:]
                return lm
        lm = {'label': label, 'x': x, 'y': y, 'n': 1, 'confidence': confidence, 'last': t,
              'recent': [(x, y)]}
        self.items.append(lm)
        return lm

    def candidates(self):
        """Every tracked landmark, confirmed or not (for tuning / debugging)."""
        return [{'label': lm['label'], 'x': round(lm['x'], 2), 'y': round(lm['y'], 2),
                 'sightings': lm['n'], 'spread_m': round(self.spread(lm), 2)}
                for lm in sorted(self.items, key=lambda l: -l['n'])]

    def confirmed(self):
        """Landmarks seen often enough to trust, labelled uniquely (couch, couch_2, ...)."""
        out, counts = [], {}
        for lm in sorted(self.items, key=lambda l: -l['n']):
            if lm['n'] < self.min_sightings or self.spread(lm) > self.max_spread:
                continue
            k = counts.get(lm['label'], 0) + 1
            counts[lm['label']] = k
            name = lm['label'] if k == 1 else '{}_{}'.format(lm['label'], k)
            out.append({'label': name, 'x': round(lm['x'], 2), 'y': round(lm['y'], 2),
                        'sightings': lm['n'], 'confidence': lm['confidence'],
                        'spread_m': round(self.spread(lm), 2)})
        return out


def rule_matches(rule, det_label, distance):
    """rule: {"label": "person", "max_distance": 1.0, "task": {...}}"""
    if rule.get('label') not in (det_label, '*'):
        return False
    md = rule.get('max_distance')
    return md is None or (distance is not None and distance <= md)
