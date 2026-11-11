import os

import numpy as np
import pytest

from truck_bringup import camera_info_yaml
from truck_bringup.map_saver import grid_to_image, map_stats


def test_calibration_yaml_round_trip(tmp_path):
    K = [500.0, 0, 320, 0, 500, 240, 0, 0, 1]
    D = [0.1, -0.2, 0.001, 0.002, 0.0]
    R = [1.0, 0, 0, 0, 1, 0, 0, 0, 1]
    P = [480.0, 0, 320, -48.0, 0, 480, 240, 0, 0, 0, 1, 0]
    path = os.path.join(str(tmp_path), 'right.yaml')
    camera_info_yaml.save(path, 640, 480, 'right', K, D, R, P)
    c = camera_info_yaml.load(path)
    assert (c['width'], c['height'], c['name'], c['model']) == (640, 480, 'right', 'plumb_bob')
    assert c['K'] == pytest.approx(K)
    assert c['D'] == pytest.approx(D)
    assert c['P'] == pytest.approx(P)


def test_reads_camera_calibration_tool_output(tmp_path):
    # Format written by ROS camera_calibration (cameracalibrator SAVE / COMMIT).
    text = """image_width: 640
image_height: 480
camera_name: narrow_stereo/left
camera_matrix:
  rows: 3
  cols: 3
  data: [502.1, 0., 318.2, 0., 501.7, 243.9, 0., 0., 1.]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.31, 0.09, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1., 0., 0., 0., 1., 0., 0., 0., 1.]
projection_matrix:
  rows: 3
  cols: 4
  data: [470., 0., 320., 0., 0., 470., 240., 0., 0., 0., 1., 0.]
"""
    path = os.path.join(str(tmp_path), 'left.yaml')
    with open(path, 'w') as f:
        f.write(text)
    c = camera_info_yaml.load(path)
    assert c['K'][0] == pytest.approx(502.1)
    assert len(c['D']) == 5 and len(c['P']) == 12


def test_baseline_from_projection():
    # P[3] = -fx * B  ->  B = 0.1 m
    assert camera_info_yaml.baseline_m([480.0, 0, 320, -48.0, 0, 480, 240, 0, 0, 0, 1, 0]) \
        == pytest.approx(0.10)
    assert camera_info_yaml.baseline_m([0.0] * 12) == 0.0


def test_grid_to_image_flips_rows_and_maps_values():
    # 2x3 grid, row 0 = bottom of the map: [free, occupied, unknown]
    data = [0, 100, -1,
            50, 0, 0]
    img = grid_to_image(data, 3, 2)
    assert img.shape == (2, 3)
    # bottom map row ends up as the LAST image row
    assert list(img[1]) == [254, 0, 205]
    # 50 is neither free (<25) nor occupied (>=65) -> unknown grey
    assert list(img[0]) == [205, 254, 254]
    s = map_stats(data)
    assert s == {'known': 5, 'free': 3, 'occupied': 1, 'total': 6}
    assert img.dtype == np.uint8


def test_best_pair_matches_closest_frames_across_phase_offset():
    from truck_bringup.stereo_camera_node import best_pair
    # two free-running 15 fps cameras, right lags left by 25 ms
    left = [(0.000 + k / 15.0, k + 1) for k in range(4)]
    right = [(0.025 + k / 15.0, k + 1) for k in range(4)]
    i, j, skew = best_pair(left, right, -1, -1, 0.034)
    assert (left[i][1], right[j][1]) == (4, 4) and abs(skew - 0.025) < 1e-9
    # nothing new on the right since the last published pair -> wait
    assert best_pair(left, right, 3, 4, 0.034) is None
    # 15 ms limit: these cameras would never pair
    assert best_pair(left, right, -1, -1, 0.015) is None
