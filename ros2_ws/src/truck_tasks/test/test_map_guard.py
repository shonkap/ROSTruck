import math

import numpy as np
import pytest

from truck_tasks.map_guard import map_distances, to_base
from truck_tasks.planning import GridMap


def room(obstacles, res=0.05, size=4.0):
    n = int(size / res)
    data = np.zeros((n, n), np.int16)
    g = GridMap(data.ravel(), n, n, res, -size / 2, -size / 2)
    for x, y in obstacles:
        r, c = g.to_cell(x, y)
        g.grid[r, c] = 100
    return g


def test_to_base_rotation():
    p = to_base(np.array([[1.0, 1.0]]), (1.0, 0.0, math.pi / 2))
    assert p[0] == pytest.approx([1.0, 0.0])


def test_obstacle_ahead_and_behind():
    g = room([(0.7, 0.01), (0.7, 0.07), (-0.6, 0.01), (-0.6, -0.07)])
    front, rear = map_distances(g, (0.0, 0.0, 0.0), 0.25, -0.25, 0.18)
    assert front == pytest.approx(0.45, abs=0.05)
    assert rear == pytest.approx(0.35, abs=0.05)
    # turned around: front and rear swap
    front, rear = map_distances(g, (0.0, 0.0, math.pi), 0.25, -0.25, 0.18)
    assert front == pytest.approx(0.35, abs=0.05) and rear == pytest.approx(0.45, abs=0.05)


def test_offside_and_single_cell_ignored():
    g = room([(0.6, 0.51), (0.6, 0.57), (0.6, 0.01)])     # side cells + one lone cell ahead
    assert map_distances(g, (0.0, 0.0, 0.0), 0.25, -0.25, 0.18) == (None, None)
