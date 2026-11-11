import math
import random

import pytest

from fc_bridge.conversions import (
    frd_to_flu, gps_fix_type_to_navsat_status, ned_attitude_to_enu_quaternion,
    twist_to_rover_attitude_target, wrap_pi,
)


def _rot_ned(roll, pitch, yaw):
    """Body FRD -> world NED rotation matrix, ZYX Euler (ArduPilot convention)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matvec(m, v):
    return [sum(m[i][j] * v[j] for j in range(3)) for i in range(3)]


def _quat_rotate(q, v):
    x, y, z, w = q
    m = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    return _matvec(m, v)


def _ned_to_enu(v):
    return [v[1], v[0], -v[2]]


def test_attitude_conversion_matches_full_frame_change():
    rng = random.Random(0)
    for _ in range(200):
        roll, pitch, yaw = (rng.uniform(-math.pi, math.pi), rng.uniform(-1.4, 1.4),
                            rng.uniform(-math.pi, math.pi))
        v_frd = [rng.uniform(-1, 1) for _ in range(3)]
        expected = _ned_to_enu(_matvec(_rot_ned(roll, pitch, yaw), v_frd))
        q = ned_attitude_to_enu_quaternion(roll, pitch, yaw)
        got = _quat_rotate(q, list(frd_to_flu(*v_frd)))
        assert got == pytest.approx(expected, abs=1e-9)


def test_facing_north_is_plus_90_deg_enu_yaw():
    x, y, z, w = ned_attitude_to_enu_quaternion(0.0, 0.0, 0.0)
    assert 2 * math.atan2(z, w) == pytest.approx(math.pi / 2)


def test_accel_at_rest_points_up_in_flu():
    # ArduPilot at rest: zacc ~ -1 g in FRD.
    assert frd_to_flu(0.0, 0.0, -9.81)[2] == pytest.approx(9.81)


def test_wrap_pi():
    assert wrap_pi(3 * math.pi / 2) == pytest.approx(-math.pi / 2)


def test_cmd_vel_left_turn_is_negative_frd_yaw_rate():
    thrust, yaw_rate = twist_to_rover_attitude_target(1.0, 0.5, 2.0, math.radians(90))
    assert thrust == pytest.approx(0.5)
    assert yaw_rate == pytest.approx(-0.5)


def test_cmd_vel_clamps_and_allows_reverse():
    thrust, yaw_rate = twist_to_rover_attitude_target(-10.0, -10.0, 2.0, 1.0)
    assert thrust == -1.0
    assert yaw_rate == 1.0


def test_gps_status_mapping():
    assert gps_fix_type_to_navsat_status(0) == -1
    assert gps_fix_type_to_navsat_status(1) == -1
    assert gps_fix_type_to_navsat_status(3) == 0
    assert gps_fix_type_to_navsat_status(4) == 1
    assert gps_fix_type_to_navsat_status(6) == 2
