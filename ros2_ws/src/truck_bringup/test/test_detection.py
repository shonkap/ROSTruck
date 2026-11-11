import json
import math

import numpy as np
import pytest

from truck_bringup.detection import (ColorBlobDetector, LandmarkMap, bbox_point, ground_point,
                                     quat_rotate, rule_matches)


def test_color_blob_finds_red_square_not_scribbles():
    img = np.full((240, 320, 3), 120, np.uint8)
    img[100:160, 140:200] = (0, 0, 230)           # solid red square (BGR)
    img[20:22, 10:300] = (0, 0, 230)              # thin red line (texture-like)
    dets = ColorBlobDetector(min_area=300).detect(img)
    assert len(dets) == 1
    x0, y0, x1, y1 = dets[0].bbox
    assert abs(x0 - 140) <= 3 and abs(y1 - 160) <= 3


def test_bbox_point_depth_and_bearing():
    disp = np.zeros((240, 320), np.float32)
    disp[50:90, 200:240] = 19.0                    # half-res disparity
    # full-res: fx 380, B 0.1, disparity 38 px -> z = 1.0 m
    p = bbox_point((400, 100, 480, 180), disp, 380.0, 380.0, 320.0, 240.0, 0.1, scale=0.5)
    assert p[2] == pytest.approx(1.0, rel=0.01)
    assert p[0] == pytest.approx((440 - 320) / 380.0, rel=0.01)   # right of centre
    assert bbox_point((0, 0, 40, 40), disp, 380, 380, 320, 240, 0.1, 0.5) is None


def test_quat_rotate_yaw_90():
    s = math.sqrt(0.5)
    x, y, z = quat_rotate((0.0, 0.0, s, s), (1.0, 0.0, 0.0))
    assert (x, y, z) == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)


def test_landmarks_merge_and_confirm():
    lm = LandmarkMap(min_sightings=3)
    for i in range(4):
        lm.add('couch', 2.0 + 0.05 * i, 1.0, 0.8, i)
    lm.add('couch', 5.0, 1.0, 0.9, 5)              # a second couch, seen once
    lm.add('ball', 0.0, 0.0, 0.5, 6)
    c = lm.confirmed()
    assert [x['label'] for x in c] == ['couch']
    assert c[0]['x'] == pytest.approx(2.07, abs=0.02)
    json.dumps(c)


def test_rules():
    assert rule_matches({'label': 'person', 'max_distance': 1.0}, 'person', 0.8)
    assert not rule_matches({'label': 'person', 'max_distance': 1.0}, 'person', 1.5)
    assert not rule_matches({'label': 'person'}, 'dog', 0.5)
    assert rule_matches({'label': '*'}, 'dog', None)


def test_ground_point_level_and_tilted_camera():
    # level camera 0.25 m up: box bottom 95 px below centre at fx 380 -> floor 1.0 m ahead
    p = ground_point((300, 200, 340, 335), 380.0, 380.0, 320.0, 240.0, 0.25, 0.0)
    assert p[2] == pytest.approx(1.0, rel=0.01) and p[1] == pytest.approx(0.25, rel=0.01)
    # tilted down: the image centre row hits the floor at h / tan(pitch)
    p = ground_point((300, 200, 340, 240), 380.0, 380.0, 320.0, 240.0, 0.25, 0.1)
    assert math.hypot(p[2], p[1]) == pytest.approx(0.25 / math.sin(0.1), rel=0.01)
    # bottom above the horizon: no floor contact
    assert ground_point((300, 100, 340, 150), 380.0, 380.0, 320.0, 240.0, 0.25, 0.0) is None


def test_inconsistent_sightings_are_not_confirmed():
    lm = LandmarkMap(min_sightings=3, max_spread=0.3)
    for i in range(10):                            # real object: tight cluster
        lm.add('bin', 1.0 + 0.02 * (i % 3), 2.0, 0.9, i)
    for i, x in enumerate([5.0, 5.4, 4.65, 5.35, 4.7, 5.4, 4.65, 5.35]):   # false: jitters ~0.35 m
        lm.add('bin', x, 0.0, 0.9, 20 + i)
    c = lm.confirmed()
    assert [x['label'] for x in c] == ['bin'] and c[0]['x'] < 1.1
    assert [x['sightings'] for x in lm.candidates()] == [10, 8]      # still visible for tuning
    assert c[0]['spread_m'] < 0.05
