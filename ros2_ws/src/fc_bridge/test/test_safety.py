import pytest

from fc_bridge.safety import FORWARD, REVERSE, SpeedCaps


def test_most_restrictive_fresh_cap_wins_and_curvature_kept():
    s = SpeedCaps(max_age=1.0)
    s.update('stereo', FORWARD, 0.4, 0.0)
    s.update('lidar', FORWARD, 0.2, 0.0)
    lin, ang, by = s.apply(0.5, 0.5, 0.5)
    assert (lin, by) == (0.2, FORWARD) and ang == pytest.approx(0.2)   # curvature 1.0 kept


def test_reverse_cap_only_from_lidar():
    s = SpeedCaps()
    s.update('stereo', FORWARD, 0.0, 0.0)          # wall ahead
    assert s.apply(-0.2, 0.3, 0.1) == (-0.2, 0.3, None)   # reversing away is fine
    s.update('lidar_rear', REVERSE, 0.0, 0.0)      # ...unless something is behind too
    lin, ang, by = s.apply(-0.2, 0.3, 0.1)
    assert lin == 0.0 and by == REVERSE


def test_stale_guard_is_ignored_and_reported():
    s = SpeedCaps(max_age=1.0)
    s.update('stereo', FORWARD, 0.0, 0.0)
    assert s.apply(0.3, 0.0, 0.5)[0] == 0.0
    assert s.apply(0.3, 0.0, 2.0)[0] == 0.3
    assert s.stale_sources(2.0) == ['stereo']
