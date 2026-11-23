import pytest

from truck_bringup.truck_keys import CarControls


def test_speed_and_steering_hold_and_combine():
    c = CarControls()
    for ch in 'www':
        c.key(ch)
    c.key('a')
    t = c.twist()
    assert t.linear.x == pytest.approx(0.3) and t.angular.z == pytest.approx(0.15)   # k=0.5
    c.key('w')                                  # faster: same curvature, more yaw rate
    assert c.twist().angular.z == pytest.approx(0.2)


def test_limits_reverse_center_stop_quit():
    c = CarControls()
    for _ in range(20):
        c.key('a')
        c.key('s')
    assert c.k == pytest.approx(1 / 0.35, abs=1e-6) and c.v == pytest.approx(-0.3)
    c.key('c')
    assert c.k == 0.0 and c.v == pytest.approx(-0.3)
    c.key(' ')
    assert (c.v, c.k) == (0.0, 0.0)
    assert c.key('q') is False and c.twist().linear.x == 0.0


def test_no_turning_while_stopped():
    c = CarControls()
    c.key('a')
    assert c.twist().angular.z == 0.0          # a car can't turn in place
