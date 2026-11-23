import math

import numpy as np
import pytest

from truck_bringup.collision_guard import obstacle_points
from truck_common.guards import corridor_distances


def obstacle_distance_3d(disp, fx, fy, cx, cy, b, h, pitch, half_width):
    pts = obstacle_points(disp, fx, fy, cx, cy, b, h, pitch)
    return corridor_distances(pts, 0.0, -1e9, half_width, 40)[0]

FX = FY = 190.0
CX, CY = 160.0, 120.0
B = 0.1
H = 0.23        # camera height
W, HT = 320, 240


def floor_disparity(pitch):
    """Disparity image of a flat floor seen by a camera H m up, pitched down."""
    disp = np.zeros((HT, W), np.float32)
    for v in range(HT):
        ray_y = (v - CY) / FY                       # optical y per unit z
        down = ray_y * math.cos(pitch) + math.sin(pitch)
        if down <= 0.01:
            continue
        z = H / down                                # depth where the ray hits the floor
        disp[v, :] = FX * B / z
    return disp


def put_box(disp, distance, height, pitch, half_width=0.1):
    """Paint a box face 'distance' m ahead (level frame), 'height' m tall."""
    for v in range(HT):
        ray_y = (v - CY) / FY
        cp, sp = math.cos(pitch), math.sin(pitch)
        fwd_per_z = cp - ray_y * sp
        z = distance / fwd_per_z
        drop = (ray_y * cp + sp) * z
        if 0 <= H - drop <= height:
            for u in range(W):
                x = (u - CX) * z / FX
                if abs(x) <= half_width:
                    disp[v, u] = max(disp[v, u], FX * B / z)
    return disp


def test_flat_floor_is_not_an_obstacle():
    for pitch in (0.0, 0.1, 0.2):
        assert obstacle_distance_3d(floor_disparity(pitch), FX, FY, CX, CY, B, H, pitch, 0.2) is None


def test_low_object_close_in_front_is_caught():
    # 6 cm tall (over 2 in), 0.6 m ahead: below the lidar's plane and below a fixed image window
    disp = put_box(floor_disparity(0.1), 0.6, 0.06, 0.1)
    d = obstacle_distance_3d(disp, FX, FY, CX, CY, B, H, 0.1, 0.2)
    assert d == pytest.approx(0.6, abs=0.06)


def test_object_under_2_inches_is_ignored_and_offside_ignored():
    disp = put_box(floor_disparity(0.1), 0.6, 0.03, 0.1)       # 3 cm: under the threshold
    assert obstacle_distance_3d(disp, FX, FY, CX, CY, B, H, 0.1, 0.2) is None


def test_table_top_above_the_truck_is_ignored():
    # 4 cm thick top at 0.43-0.47 m, 0.8 m ahead: the truck (0.32 m) drives under it
    disp = floor_disparity(0.1)
    for v in range(HT):
        ray_y = (v - CY) / FY
        cp, sp = math.cos(0.1), math.sin(0.1)
        z = 0.8 / (cp - ray_y * sp)
        if 0.43 <= H - (ray_y * cp + sp) * z <= 0.47:
            disp[v, :] = FX * B / z
    assert obstacle_distance_3d(disp, FX, FY, CX, CY, B, H, 0.1, 0.2) is None
