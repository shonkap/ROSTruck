import math
import os
from types import SimpleNamespace as NS

import numpy as np
import pytest

from truck_common.guards import NO_LIMIT, corridor_distances, ramp
from truck_common.maps import save_nav2_map, trinary_image
from truck_common.ros import jmsg, jparse, rpy_from_quat, yaw_from_quat


def quat_yaw(yaw):
    return NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def test_yaw_and_rpy():
    assert yaw_from_quat(quat_yaw(1.0)) == pytest.approx(1.0)
    assert rpy_from_quat(quat_yaw(-2.0)) == pytest.approx((0.0, 0.0, -2.0))


def test_json_roundtrip():
    assert jparse(jmsg({'a': 1})) == {'a': 1}
    assert jparse(NS(data='not json')) is None


def test_ramp():
    assert ramp(None, 0.2, 0.7, 1.0) == NO_LIMIT
    assert ramp(0.1, 0.2, 0.7, 1.0) == 0.0
    assert ramp(0.45, 0.2, 0.7, 1.0) == pytest.approx(0.5)
    assert ramp(0.8, 0.2, 0.7, 1.0) == NO_LIMIT


def test_corridor():
    pts = np.array([[0.5, 0.0], [0.6, 0.05], [0.4, 0.5], [-0.5, 0.0], [-0.55, 0.1], [0.1, 0.0]])
    front, rear = corridor_distances(pts, 0.25, -0.25, 0.2)
    assert front == pytest.approx(0.35) and rear == pytest.approx(0.3)
    assert corridor_distances(pts[:1], 0.25, -0.25, 0.2) == (None, None)


def test_save_map(tmp_path):
    data = [-1, 0, 100, 50]
    msg = NS(data=data, info=NS(width=2, height=2, resolution=0.05,
                                origin=NS(position=NS(x=-1.0, y=-2.0), orientation=quat_yaw(0.0))))
    line = save_nav2_map(msg, str(tmp_path / 'sub' / 'room'))
    assert 'known' in line
    yaml = open(str(tmp_path / 'sub' / 'room.yaml')).read()
    assert 'origin: [-1.0, -2.0, 0.0]' in yaml and 'image: room.png' in yaml
    img = trinary_image(np.array(data, np.int16).reshape(2, 2))
    assert img.tolist() == [[0, 205], [205, 254]]     # flipped: row 0 = top
    assert os.path.exists(str(tmp_path / 'sub' / 'room.png'))


def test_solid_objects_drops_only_few_mm_specks():
    from truck_common.clouds import solid_objects
    pen = np.array([[1.0 + dx, 2.0] for dx in np.arange(0, 0.02, 0.005)])    # 2 cm
    cat = np.array([[2.0 + dx, 1.0 + dy] for dx in np.arange(0, 0.3, 0.01) for dy in (0.0, 0.01)])
    specks = np.array([[3.0, 3.0], [0.5, 0.5]])                                 # lone returns
    kept = solid_objects(np.vstack([pen, cat, specks]))
    assert len(kept) == len(pen) + len(cat)
