import pytest

from fc_bridge.steering_monitor import LEFT, RIGHT, SteeringMonitor, normalized_steering

PARAMS = {'ATC_STR_RAT_FF': 0.9, 'TURN_RADIUS': 0.3,
          'SERVO1_TRIM': 1500, 'SERVO1_MIN': 1000, 'SERVO1_MAX': 2000,
          'SERVO2_TRIM': 1500, 'SERVO2_MIN': 1000, 'SERVO2_MAX': 2000}


def drive(mon, cmd, meas, steer, seconds=6.0, speed=0.35, pwm=(1500, 1500), t0=0.0):
    """Feed `seconds` of steady 20 Hz samples; returns end time."""
    t = t0
    while t < t0 + seconds:
        mon.add_sample(t, cmd, meas, speed, steer, pwm[0], pwm[1])
        t += 0.05
    return t


def test_normalized_steering():
    assert normalized_steering(2000, 1000, 1500, 2000, False) == 1.0
    assert normalized_steering(1250, 1000, 1500, 2000, False) == -0.5
    assert normalized_steering(1250, 1000, 1500, 2000, True) == 0.5
    assert normalized_steering(0, 1000, 1500, 2000, False) is None


def test_samples_need_a_steady_command_and_speed():
    m = SteeringMonitor()
    assert not m.add_sample(0.0, 0.35, 0.3, 0.35, -0.2, 1400, 1600)   # first sample
    assert not m.add_sample(0.5, 0.35, 0.3, 0.35, -0.2, 1400, 1600)   # not steady yet
    assert m.add_sample(1.2, 0.35, 0.3, 0.35, -0.2, 1400, 1600)
    assert not m.add_sample(1.3, 0.35, 0.3, 0.05, -0.2, 1400, 1600)   # too slow


def test_good_tracking_changes_nothing():
    m = SteeringMonitor()
    t = drive(m, 0.35, 0.35, -0.3)
    drive(m, -0.35, -0.35, 0.3, t0=t + 1)
    changes, notes = m.recommend(PARAMS)
    assert changes == {}


def test_under_turning_raises_ff_step_limited():
    m = SteeringMonitor()
    t = drive(m, 0.35, 0.15, -0.2)          # 43 % of the requested rate
    drive(m, -0.35, -0.15, 0.2, t0=t + 1)
    changes, _ = m.recommend(PARAMS)
    # 1/0.43 = 2.3x, limited to 1.5x per step
    assert changes['ATC_STR_RAT_FF'] == pytest.approx(0.9 * 1.5, abs=1e-3)


def test_over_turning_lowers_ff():
    m = SteeringMonitor()
    t = drive(m, 0.35, 0.45, -0.4)
    drive(m, -0.35, -0.45, 0.4, t0=t + 1)
    changes, _ = m.recommend(PARAMS)
    assert changes['ATC_STR_RAT_FF'] < 0.9


def test_off_centre_trim_is_fixed_first():
    m = SteeringMonitor()
    # straight line, but the FC holds the front servo at 1540 and the rear at 1460
    t = drive(m, 0.0, 0.0, 0.08, pwm=(1540, 1460))
    drive(m, 0.35, 0.15, -0.2, t0=t + 1)    # also under-turning - must wait for re-centring
    changes, notes = m.recommend(PARAMS)
    assert changes == {'SERVO1_TRIM': 1525, 'SERVO2_TRIM': 1475}   # 25 us step limit
    assert 'ATC_STR_RAT_FF' not in changes


def test_end_stop_on_one_side_learns_per_direction_radius_and_clamps():
    m = SteeringMonitor()
    # left: fine.  right: steering pinned at +1 and only 0.5 rad/s at 0.35 m/s (r = 0.7 m)
    t = drive(m, 0.35, 0.35, -0.3)
    drive(m, -1.2, -0.5, 1.0, t0=t + 1)
    changes, notes = m.recommend(PARAMS)
    assert m.min_radius[RIGHT] == pytest.approx(0.7)
    assert m.min_radius[LEFT] is None
    assert changes['TURN_RADIUS'] == pytest.approx(0.7)
    # right turns are capped at v / r, left turns untouched
    assert m.clamp_yaw_rate(-1.2, 0.35) == pytest.approx(-0.5)
    assert m.clamp_yaw_rate(1.2, 0.35) == 1.2
    assert any('end stop' in n for n in notes)


def test_bounds_hold():
    m = SteeringMonitor()
    p = dict(PARAMS, ATC_STR_RAT_FF=2.9)
    t = drive(m, 0.35, 0.05, -0.5)
    drive(m, -0.35, -0.05, 0.5, t0=t + 1)
    changes, _ = m.recommend(p)
    assert changes['ATC_STR_RAT_FF'] <= SteeringMonitor.FF_BOUNDS[1]
    assert LEFT in m.summary() and 'straight_samples' in m.summary()


def test_straight_line_drift_moves_front_trim():
    m = SteeringMonitor()
    p = dict(PARAMS, SERVO1_TRIM=1560)
    # FC holds the servo at its (wrong) trim, truck curves right
    t = drive(m, 0.0, -0.058, 0.0, pwm=(1560, 1500))
    # turning samples teach the sensitivity: steer -0.3 -> +0.35 rad/s
    drive(m, 0.35, 0.35, -0.3, t0=t + 1)
    changes, notes = m.recommend(p)
    # -drift/k = 0.058 / -1.167 = -0.05 steer -> -0.05 * 500 us = -25 us (step limit)
    assert changes == {'SERVO1_TRIM': 1535}
    assert 'curves' in notes[0]


def test_drift_without_turn_data_waits():
    m = SteeringMonitor()
    drive(m, 0.0, -0.058, 0.0, pwm=(1560, 1500))
    changes, _ = m.recommend(dict(PARAMS, SERVO1_TRIM=1560))
    assert changes == {}


def test_small_residual_drift_still_corrected():
    m = SteeringMonitor()
    p = dict(PARAMS, SERVO1_TRIM=1535)
    t = drive(m, 0.0, -0.034, 0.0, pwm=(1535, 1500))
    drive(m, 0.35, 0.35, -0.3, t0=t + 1)
    changes, _ = m.recommend(p)
    # 0.034 / 1.167 * 500 us = 14.6 us left
    assert changes == {'SERVO1_TRIM': 1520}
