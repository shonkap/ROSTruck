import math

import pytest

from fc_bridge.vision import VisionFeed, enu_to_ned_pose


def test_enu_to_ned_axes():
    n, e, d, r, p, y = enu_to_ned_pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.0)
    assert (n, e, d) == (2.0, 1.0, -3.0)
    assert r == 0.1 and p == -0.2
    assert y == pytest.approx(math.pi / 2)        # facing ENU +x (east) = NED yaw 90


def test_facing_north_is_ned_yaw_zero():
    assert enu_to_ned_pose(0, 0, 0, 0, 0, math.pi / 2)[5] == pytest.approx(0.0)


def test_velocity_from_motion_in_ned():
    f = VisionFeed()
    f.update(0.0, 0, 0, 0, 0, 0, 0)
    for i in range(1, 20):
        out = f.update(0.1 * i, 0.05 * i, 0, 0, 0, 0, 0)   # 0.5 m/s east
    assert out['ve'] == pytest.approx(0.5, abs=0.01)
    assert abs(out['vn']) < 1e-6
    assert out['reset_counter'] == 0


def test_jump_bumps_reset_counter():
    f = VisionFeed()
    f.update(0.0, 0, 0, 0, 0, 0, 0)
    f.update(0.1, 0.03, 0, 0, 0, 0, 0)
    out = f.update(0.2, 1.5, 0, 0, 0, 0, 0)               # loop closure jump
    assert out['reset_counter'] == 1 and out['ve'] == 0.0


def test_covariance_diagonal():
    out = VisionFeed().update(0.0, 0, 0, 0, 0, 0, 0, pos_std=0.1, yaw_std=0.2)
    c = out['covariance']
    assert len(c) == 21
    assert c[0] == c[6] == c[11] == pytest.approx(0.01)
    assert c[15] == c[18] == c[20] == pytest.approx(0.04)
    assert c[1] == 0.0
