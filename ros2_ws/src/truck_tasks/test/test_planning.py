import math

import numpy as np
import pytest

from truck_tasks.planning import (GridMap, astar, cost_map, dilate, frontiers, plan,
                                  pure_pursuit, reachable_free, approach_point, mark_driven,
                                  heading_error, ThreePointTurn, speed_for_curvature)


def room(w=60, h=40, res=0.1, unknown_right=False):
    """Walled room 6 x 4 m, wall down the middle with a gap at the top."""
    g = np.zeros((h, w), np.int16)
    g[0, :] = g[-1, :] = 100
    g[:, 0] = g[:, -1] = 100
    g[:30, 30] = 100            # divider, gap in rows 30..38
    if unknown_right:
        g[1:-1, 45:-1] = -1
    return GridMap(g.ravel().tolist(), w, h, res, 0.0, 0.0)


def test_cell_world_round_trip():
    gm = room()
    r, c = gm.to_cell(1.23, 2.07)
    x, y = gm.to_world(r, c)
    assert abs(x - 1.25) < 1e-9 and abs(y - 2.05) < 1e-9


def test_dilate_disk():
    m = np.zeros((9, 9), bool)
    m[4, 4] = True
    d = dilate(m, 2.0)
    assert d[4, 6] and d[6, 4] and not d[6, 6]


def test_plan_goes_through_the_gap():
    gm = room()
    path = plan(gm, (1.0, 1.0), (5.0, 1.0), robot_radius=0.15)
    assert path is not None
    # must cross x=3.0 (the divider) above y=3.0 where the gap is
    crossing = [p for p in path if abs(p[0] - 3.05) < 0.1]
    assert crossing and all(p[1] > 3.0 for p in crossing)
    assert math.hypot(path[-1][0] - 5.0, path[-1][1] - 1.0) < 0.2


def test_blocked_goal_returns_none():
    gm = room()
    g = gm.grid.copy()
    g[30:39, 30] = 100          # close the gap
    gm2 = GridMap(g.ravel().tolist(), gm.w, gm.h, gm.res, 0, 0)
    assert plan(gm2, (1.0, 1.0), (5.0, 1.0), robot_radius=0.15) is None


def test_inflation_keeps_robot_off_walls():
    gm = room()
    cost = cost_map(gm, robot_radius=0.2)
    assert not np.isfinite(cost[1, 10])        # 0.1 m from the bottom wall
    assert np.isfinite(cost[10, 10])


def test_astar_trivial():
    cost = np.ones((5, 5))
    assert astar(cost, (0, 0), (0, 3)) == [(0, 0), (0, 1), (0, 2), (0, 3)]


def test_frontiers_found_at_unknown_border():
    gm = room(unknown_right=True)
    fr = frontiers(gm, min_size=5)
    assert fr, 'expected a frontier along the unknown region'
    x, y, size = fr[0]
    assert 4.3 < x < 4.6


def test_no_frontiers_in_fully_known_room():
    assert frontiers(room(), min_size=5) == []


def test_pure_pursuit_turns_toward_target_and_respects_radius():
    path = [(0.0, 0.0), (1.0, 1.0)]
    v, w, _ = pure_pursuit((0.0, 0.0, 0.0), path, 0.5, 0.3, 0.4)
    assert v == 0.3 and w > 0                       # target is to the left
    assert w <= 0.3 / 0.4 + 1e-9
    v, w, _ = pure_pursuit((0.0, 0.0, 0.0), [(0, 0), (-1.0, -0.2)], 0.5, 0.3, 0.4)
    assert w == pytest.approx(-0.3 / 0.4)           # behind-right: max lock right


def test_frontier_islands_beyond_walls_are_ignored():
    # room 0..30 cols walled; outside the right wall a free "island" next to unknown
    g = np.full((40, 60), -1, np.int16)
    g[1:39, 1:30] = 0
    g[0, :31] = g[39, :31] = 100
    g[:, 0] = g[:, 30] = 100
    g[10:20, 35:40] = 0             # stereo-noise island outside the room
    g[10:20, 25:29] = -1            # a real unknown patch inside the room
    gm = GridMap(g.ravel().tolist(), 60, 40, 0.1, 0.0, 0.0)
    all_f = frontiers(gm, min_size=5)
    assert any(f[0] > 3.4 for f in all_f)                  # island frontier exists...
    reach = frontiers(gm, min_size=5, start_xy=(1.0, 1.0))
    assert reach and all(f[0] < 3.0 for f in reach)        # ...but isn't reachable
    assert not reachable_free(gm, (1.0, 1.0), 0.15)[15, 37]


def test_standoff_drops_frontiers_next_to_walls():
    gm = room(unknown_right=True)
    near = frontiers(gm, min_size=3, robot_radius=0.15)
    far = frontiers(gm, min_size=3, robot_radius=0.15, standoff=0.5)
    assert sum(f[2] for f in far) < sum(f[2] for f in near)


def test_approach_point_keeps_clearance_from_a_wall_hugging_frontier():
    gm = room()
    reach = reachable_free(gm, (1.0, 1.0), 0.15)
    # a target right against the bottom wall
    p = approach_point(gm, (1.5, 0.15), reach, clearance=0.45, max_dist=1.2)
    assert p is not None
    assert p[1] >= 0.1 + 0.45 - 0.05          # >= clearance from the wall at y~0.05
    assert math.hypot(p[0] - 1.5, p[1] - 0.15) <= 1.2
    assert approach_point(gm, (1.5, 0.15), reach, clearance=0.45, max_dist=0.2) is None


def test_truck_standing_in_its_own_unknown_hole_still_reaches_the_room():
    g = np.zeros((40, 60), np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    g[17:24, 17:24] = -1            # unseen floor under/around the truck
    g[30:39, 40:59] = -1            # real unexplored area
    gm = GridMap(g.ravel().tolist(), 60, 40, 0.1, 0.0, 0.0)
    reach = reachable_free(gm, (2.05, 2.05), 0.15)
    assert reach[5, 5] and reach[25, 50]
    assert frontiers(gm, 5, 0.15, start_xy=(2.05, 2.05))


def test_small_unknown_gaps_are_bridged_but_walls_are_not():
    g = np.zeros((40, 60), np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    g[:, 20] = -1                   # 1-cell unknown seam (sparse stereo grid)
    g[:, 40] = 100                  # a real wall
    gm = GridMap(g.ravel().tolist(), 60, 40, 0.1, 0.0, 0.0)
    reach = reachable_free(gm, (1.0, 2.0), 0.15)
    assert reach[20, 30]            # across the seam
    assert not reach[20, 50]        # not through the wall


def test_driven_path_reconnects_the_unseen_bubble():
    g = np.zeros((40, 60), np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    g[10:30, 10:30] = -1            # big unseen area the truck drove through
    g[30:39, 40:59] = -1
    gm = GridMap(g.ravel().tolist(), 60, 40, 0.1, 0.0, 0.0)
    start = (2.0, 2.0)
    assert not reachable_free(gm, start, 0.15)[5, 5]
    path = [(2.0 - 0.1 * i, 2.0) for i in range(12)]      # drove out to x = 0.9
    gm2 = mark_driven(gm, path, 0.35)
    assert reachable_free(gm2, start, 0.15)[5, 5]
    assert gm.grid[20, 20] == -1    # original untouched


def test_heading_error_signs():
    assert heading_error((0, 0, 0), (1, 1)) == pytest.approx(math.pi / 4)
    assert heading_error((0, 0, 0), (-1, -0.01)) < -3.0       # behind, slightly right


def test_three_point_turn_alternates_and_respects_blocking():
    gm = room()
    tp = ThreePointTurn(leg_s=3.0, speed=0.15)
    # open space: starts reversing, turning toward a target behind-left (+ yaw)
    v, w = tp.step(0.0, (2.0, 2.0, 0.0), (1.0, 2.5), gm, 0.4)
    assert v < 0 and w > 0
    # after leg_s it goes forward, still turning the same way
    v, w = tp.step(3.5, (2.0, 2.0, 0.0), (1.0, 2.5), gm, 0.4)
    assert v > 0 and w > 0
    # guard sees something close ahead -> back to reversing at once
    v, w = tp.step(3.6, (2.0, 2.0, 0.0), (1.0, 2.5), gm, 0.4, front_dist=0.3)
    assert v < 0


def test_three_point_turn_starts_forward_with_wall_behind_and_boxed_in_waits():
    gm = room()
    tp = ThreePointTurn()
    # truck at x=0.35 facing +x: the left wall (x~0.05) is right behind it
    v, w = tp.step(0.0, (0.35, 2.0, 0.0), (0.2, 1.0), gm, 0.4)
    assert v > 0 and w < 0
    # wall behind AND obstacle ahead: stop and wait
    v, w = tp.step(0.1, (0.35, 2.0, 0.0), (0.2, 1.0), gm, 0.4, front_dist=0.3)
    assert (v, w) == (0.0, 0.0) and tp.boxed_in

def test_three_point_turn_outside_the_map_does_not_crash():
    g = np.zeros((20, 30), np.int16)
    small = GridMap(g.ravel().tolist(), 30, 20, 0.1, 0.0, 0.0)
    v, w = ThreePointTurn().step(0.0, (5.5, 3.5, 0.0), (4.0, 3.5), small, 0.4)
    assert v != 0

def test_speed_for_curvature():
    assert speed_for_curvature(0.0, 0.3, 0.5) == 0.5
    assert speed_for_curvature(-2.0, 0.3, 0.5) == 0.3
    assert 0.3 < speed_for_curvature(0.9, 0.3, 0.5) < 0.5
    assert speed_for_curvature(0.0, 0.3, 0.2) == 0.3     # max below base: never faster than base


def test_three_point_turn_uses_lidar_rear_distance():
    gm = room()
    tp = ThreePointTurn()
    # open map behind, but the lidar sees something 0.1 m behind the bumper -> go forward
    v, w = tp.step(0.0, (2.0, 2.0, 0.0), (1.0, 2.5), gm, 0.4, front_dist=None, rear_dist=0.1)
    assert v > 0
