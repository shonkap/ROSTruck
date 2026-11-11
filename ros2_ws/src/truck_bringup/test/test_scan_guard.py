import math

import pytest

from truck_bringup.scan_guard import NO_LIMIT, corridor_distances, limit_for, scan_to_points


def scan_with(hits, n=360, far=5.0):
    """360-ray scan (angle_min -pi), all `far` except {deg: range}."""
    r = [far] * n
    for deg, rng in hits.items():
        r[int(round((math.radians(deg) + math.pi) / (2 * math.pi / n))) % n] = rng
    return r, -math.pi, 2 * math.pi / n


def test_points_in_base_link_with_mount_offset_and_yaw():
    r, a0, inc = scan_with({0: 1.0}, far=float('inf'))
    pts = scan_to_points(r, a0, inc, 0.15, 12.0, (0.12, 0.0, 0.0))
    assert pts.shape == (1, 2) and pts[0] == pytest.approx([1.12, 0.0], abs=1e-6)
    pts = scan_to_points(r, a0, inc, 0.15, 12.0, (0.0, 0.0, math.pi))   # mounted backwards
    assert pts[0] == pytest.approx([-1.0, 0.0], abs=1e-6)


def test_front_and_rear_corridor_and_self_hits_ignored():
    # wall 0.6 m ahead of the laser, something 0.5 m behind the truck, own mast at 0.05 m
    r, a0, inc = scan_with({0: 0.6, 1: 0.6, 180: 0.9, 179: 0.9, 90: 0.3})
    pts = scan_to_points(r, a0, inc, 0.15, 12.0, (0.12, 0.0, 0.0))
    front, rear = corridor_distances(pts, 0.25, -0.25, 0.2)
    assert front == pytest.approx(0.12 + 0.6 - 0.25, abs=0.02)    # bumper to wall
    assert rear == pytest.approx(0.9 - 0.12 - 0.25, abs=0.02)
    # the hit at 90 deg (0.3 m to the side) is outside the corridor: not counted


def test_single_noisy_return_is_ignored():
    r, a0, inc = scan_with({0: 0.3})
    pts = scan_to_points(r, a0, inc, 0.15, 12.0, (0.0, 0.0, 0.0))
    front, _ = corridor_distances(pts, 0.25, -0.25, 0.2, min_points=2)
    assert front is None or front > 1.0


def test_limits():
    assert limit_for(None, 0.2, 0.7, 1.0) == NO_LIMIT
    assert limit_for(0.1, 0.2, 0.7, 1.0) == 0.0
    assert limit_for(0.45, 0.2, 0.7, 1.0) == pytest.approx(0.5)
    assert limit_for(2.0, 0.2, 0.7, 1.0) == NO_LIMIT
