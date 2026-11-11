import numpy as np
import pytest

from truck_bringup.collision_guard import NO_LIMIT, nearest_obstacle, path_roi, speed_limit


def test_nearest_obstacle_from_disparity():
    disp = np.zeros((240, 320), np.float32)
    disp[100:140, 140:180] = 38.0           # fx*B/d = 190*0.1/38 = 0.5 m
    d = nearest_obstacle(disp, 190.0, 0.1, (120, 80, 200, 160))
    assert d == pytest.approx(0.5, rel=0.01)


def test_too_few_pixels_is_nothing():
    disp = np.zeros((240, 320), np.float32)
    disp[120:125, 150:155] = 60.0           # 25 px of noise
    assert nearest_obstacle(disp, 190.0, 0.1, (120, 80, 200, 160)) is None


def test_speed_limit_ramp():
    assert speed_limit(None, 0.4, 1.2, 1.0) == NO_LIMIT
    assert speed_limit(0.3, 0.4, 1.2, 1.0) == 0.0
    assert speed_limit(0.8, 0.4, 1.2, 1.0) == pytest.approx(0.5)
    assert speed_limit(2.0, 0.4, 1.2, 1.0) == NO_LIMIT


def test_path_roi_is_central_and_inside_image():
    x0, y0, x1, y1 = path_roi(320, 240, 190.0, 160.0, 120.0, 0.25, 0.2, 0.1, 2.0)
    assert 0 <= x0 < 160 < x1 <= 320
    assert 0 <= y0 < y1 <= 240
