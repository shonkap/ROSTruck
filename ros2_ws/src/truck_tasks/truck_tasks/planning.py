"""Grid planning for a car-like truck. ROS-free, numpy only, Python 3.6.

Conventions: the grid is a nav_msgs/OccupancyGrid laid out as a numpy array
grid[row, col] with row 0 at the map origin (bottom), values -1 unknown,
0..100 occupancy. World <-> cell uses origin (x0, y0) and resolution; the
map's origin yaw is assumed 0 (RTAB-Map publishes it that way).
"""
import heapq
import math

import numpy as np

UNKNOWN, FREE_MAX, OCCUPIED_MIN = -1, 25, 65


class GridMap(object):
    def __init__(self, data, width, height, resolution, origin_x, origin_y):
        self.grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.w, self.h = width, height
        self.res = float(resolution)
        self.x0, self.y0 = float(origin_x), float(origin_y)

    def to_cell(self, x, y):
        return int(math.floor((y - self.y0) / self.res)), int(math.floor((x - self.x0) / self.res))

    def to_world(self, r, c):
        return self.x0 + (c + 0.5) * self.res, self.y0 + (r + 0.5) * self.res

    def inside(self, r, c):
        return 0 <= r < self.h and 0 <= c < self.w


def _disk(radius_cells):
    r = int(math.ceil(radius_cells))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return (xx * xx + yy * yy) <= radius_cells * radius_cells


def dilate(mask, radius_cells):
    """Binary dilation with a disk (no scipy on the Jetson)."""
    if radius_cells <= 0:
        return mask.copy()
    k = _disk(radius_cells)
    r = k.shape[0] // 2
    padded = np.pad(mask, r, mode='constant')
    out = np.zeros_like(mask)
    for dy, dx in zip(*np.nonzero(k)):
        out |= padded[dy:dy + mask.shape[0], dx:dx + mask.shape[1]]
    return out


def cost_map(gm, robot_radius, unknown_cost=8.0, soft_margin=0.15):
    """Per-cell traversal cost; inf = blocked.

    occupied cells are inflated by the robot radius (lethal) and by a soft
    margin (expensive) so paths keep away from walls; unknown is allowed but
    costs more (needed for exploring)."""
    occ = gm.grid >= OCCUPIED_MIN
    lethal = dilate(occ, robot_radius / gm.res)
    near = dilate(occ, (robot_radius + soft_margin) / gm.res)
    cost = np.ones(gm.grid.shape, dtype=np.float64)
    cost[gm.grid == UNKNOWN] = unknown_cost
    cost[near] += 4.0
    cost[lethal] = np.inf
    return cost


def astar(cost, start, goal):
    """8-connected A* on a cost grid. start/goal = (row, col). Returns [(r, c)] or None."""
    h, w = cost.shape
    if not (0 <= goal[0] < h and 0 <= goal[1] < w) or not np.isfinite(cost[goal]):
        return None
    if not (0 <= start[0] < h and 0 <= start[1] < w):
        return None
    moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    g = {start: 0.0}
    parent = {start: None}
    heur = lambda p: math.hypot(p[0] - goal[0], p[1] - goal[1])  # noqa: E731
    openq = [(heur(start), start)]
    closed = set()
    while openq:
        _, cur = heapq.heappop(openq)
        if cur in closed:
            continue
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]
        closed.add(cur)
        for dr, dc, step in moves:
            nxt = (cur[0] + dr, cur[1] + dc)
            if not (0 <= nxt[0] < h and 0 <= nxt[1] < w) or nxt in closed:
                continue
            c = cost[nxt]
            if not np.isfinite(c):
                continue
            ng = g[cur] + step * c
            if ng < g.get(nxt, math.inf):
                g[nxt] = ng
                parent[nxt] = cur
                heapq.heappush(openq, (ng + heur(nxt), nxt))
    return None


def nearest_free(cost, cell, max_radius=20):
    """Closest finite-cost cell to `cell` (start/goal may sit in the inflation zone)."""
    h, w = cost.shape
    if 0 <= cell[0] < h and 0 <= cell[1] < w and np.isfinite(cost[cell]):
        return cell
    for rad in range(1, max_radius + 1):
        best = None
        for dr in range(-rad, rad + 1):
            for dc in (-rad, rad) if abs(dr) != rad else range(-rad, rad + 1):
                r, c = cell[0] + dr, cell[1] + dc
                if 0 <= r < h and 0 <= c < w and np.isfinite(cost[r, c]):
                    d = dr * dr + dc * dc
                    if best is None or d < best[0]:
                        best = (d, (r, c))
        if best:
            return best[1]
    return None


def plan(gm, start_xy, goal_xy, robot_radius):
    """World-frame path [(x, y)] from start to goal, or None."""
    cost = cost_map(gm, robot_radius)
    # The robot's own cell is never lethal for itself (it is standing there).
    s = nearest_free(cost, gm.to_cell(*start_xy))
    g = nearest_free(cost, gm.to_cell(*goal_xy), max_radius=int(0.5 / gm.res))
    if s is None or g is None:
        return None
    cells = astar(cost, s, g)
    if cells is None:
        return None
    return simplify([gm.to_world(r, c) for r, c in cells], spacing=0.15)


def simplify(path, spacing):
    """Resample a dense path to roughly `spacing` metres between points."""
    if not path:
        return path
    out = [path[0]]
    for p in path[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= spacing:
            out.append(p)
    if out[-1] != path[-1]:
        out.append(path[-1])
    return out


def mark_driven(gm, points, radius):
    """Copy of the map with the floor the truck has driven over marked free.

    A forward-looking stereo pair never sees the floor beside/behind the
    truck, so its own path stays 'unknown' - but it's provably traversable.
    Occupied cells are left alone."""
    g = gm.grid.copy()
    k = _disk(radius / gm.res)
    rad = k.shape[0] // 2
    ks = list(zip(*np.nonzero(k)))
    for x, y in points:
        r0, c0 = gm.to_cell(x, y)
        for dr, dc in ks:
            r, c = r0 + dr - rad, c0 + dc - rad
            if 0 <= r < gm.h and 0 <= c < gm.w and g[r, c] == UNKNOWN:
                g[r, c] = 0
    out = object.__new__(GridMap)
    out.grid, out.w, out.h, out.res, out.x0, out.y0 = g, gm.w, gm.h, gm.res, gm.x0, gm.y0
    return out


def reachable_free(gm, start_xy, robot_radius, gap=0.15, footprint=0.4):
    """Cells the truck can reach from start_xy without crossing walls or real
    unknown space (4-connected flood fill on the inflated grid).

    Two allowances for how a forward-looking stereo map really looks:
      * the truck's own footprint counts as free - the cameras never see the
        floor right under/around the truck, so it sits in an unknown hole;
      * unknown gaps up to `gap` m wide next to free cells are bridged - the
        stereo grid is patchy.
    Walls stay lethal, so free "islands" beyond them are still excluded."""
    free = (gm.grid >= 0) & (gm.grid < FREE_MAX)
    lethal = dilate(gm.grid >= OCCUPIED_MIN, robot_radius / gm.res)
    bridge = (gm.grid == UNKNOWN) & dilate(free, gap / gm.res)
    passable = (free | bridge) & ~lethal
    r0, c0 = gm.to_cell(*start_xy)
    k = _disk(footprint / gm.res)
    rad = k.shape[0] // 2
    for dr, dc in zip(*np.nonzero(k)):
        r, c = r0 + dr - rad, c0 + dc - rad
        if gm.inside(r, c) and gm.grid[r, c] < OCCUPIED_MIN:
            passable[r, c] = True
    out = np.zeros(gm.grid.shape, bool)
    if not gm.inside(r0, c0) or not passable[r0, c0]:
        cost = np.where(passable, 1.0, np.inf)
        start = nearest_free(cost, (r0, c0), max_radius=int(0.6 / gm.res))
        if start is None:
            return out
    else:
        start = (r0, c0)
    stack = [start]
    out[start] = True
    while stack:
        r, c = stack.pop()
        for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= nr < gm.h and 0 <= nc < gm.w and passable[nr, nc] and not out[nr, nc]:
                out[nr, nc] = True
                stack.append((nr, nc))
    return out


def frontiers(gm, min_size=8, robot_radius=0.17, start_xy=None, standoff=None):
    """Clusters of known-free cells bordering unknown space, as world (x, y, size),
    largest first. These are where the map can still grow.

    With start_xy, only frontiers connected to the truck through known free
    space count - stereo noise leaves free "islands" beyond walls whose
    frontiers can't actually be reached. `standoff` (>= robot_radius) keeps
    frontier goals that far from obstacles."""
    free = (gm.grid >= 0) & (gm.grid < FREE_MAX)
    unknown = gm.grid == UNKNOWN
    border = free & dilate(unknown, 1.0)
    # don't chase frontiers that are inside a wall's inflation zone
    border &= ~dilate(gm.grid >= OCCUPIED_MIN, max(robot_radius, standoff or 0.0) / gm.res)
    if start_xy is not None:
        border &= reachable_free(gm, start_xy, robot_radius)
    seen = np.zeros_like(border)
    out = []
    for r0, c0 in zip(*np.nonzero(border)):
        if seen[r0, c0]:
            continue
        stack, cells = [(r0, c0)], []
        seen[r0, c0] = True
        while stack:
            r, c = stack.pop()
            cells.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < gm.h and 0 <= nc < gm.w and border[nr, nc] and not seen[nr, nc]:
                        seen[nr, nc] = True
                        stack.append((nr, nc))
        if len(cells) >= min_size:
            # centroid snapped to an actual frontier cell
            mr = sum(c[0] for c in cells) / float(len(cells))
            mc = sum(c[1] for c in cells) / float(len(cells))
            r, c = min(cells, key=lambda p: (p[0] - mr) ** 2 + (p[1] - mc) ** 2)
            x, y = gm.to_world(r, c)
            out.append((x, y, len(cells)))
    out.sort(key=lambda f: -f[2])
    return out


def approach_point(gm, target_xy, reachable, clearance, max_dist):
    """Where to park to look at target_xy: the reachable cell nearest to it that
    is at least `clearance` m from any obstacle, within max_dist. None if none.
    (Frontiers often hug walls/furniture; driving right onto them would put the
    cameras closer than stereo can see.)"""
    ok = reachable & ~dilate(gm.grid >= OCCUPIED_MIN, clearance / gm.res)
    cost = np.where(ok, 1.0, np.inf)
    cell = nearest_free(cost, gm.to_cell(*target_xy), max_radius=int(math.ceil(max_dist / gm.res)))
    return None if cell is None else gm.to_world(*cell)


def heading_error(pose, target):
    """Angle from the truck's nose to target, in (-pi, pi]; + = target on the left."""
    x, y, yaw = pose
    a = math.atan2(target[1] - y, target[0] - x) - yaw
    return math.atan2(math.sin(a), math.cos(a))


def _occupied_along(gm, pose, distances, sign, half_width=0.12):
    """Any occupied map cell (actual obstacle, not the safety margin) at the
    given distances ahead (sign +1) or behind (-1), across the truck's width."""
    x, y, yaw = pose
    c, s_ = math.cos(yaw), math.sin(yaw)
    for d in distances:
        for lat in (-half_width, 0.0, half_width):
            px = x + sign * d * c - lat * s_
            py = y + sign * d * s_ + lat * c
            r, cc = gm.to_cell(px, py)
            if gm.inside(r, cc) and gm.grid[r, cc] >= OCCUPIED_MIN:
                return True
    return False


class ThreePointTurn(object):
    """Turn a car that can't pivot to face a target behind it.

    Alternates reverse and forward legs on full lock. ArduRover keeps yaw-rate
    semantics in reverse (+ = CCW either way), so both legs rotate the truck
    the same way. A leg ends after leg_s or as soon as that direction is
    blocked: an obstacle in the map just behind, or the collision guard (or
    the map) reporting something close ahead. Starts in reverse when there's
    room behind - usually the reason we got here is something in front."""

    def __init__(self, leg_s=3.0, speed=0.15, front_stop=0.5, rear_stop=0.15):
        self.leg_s, self.speed, self.front_stop = leg_s, speed, front_stop
        self.rear_stop = rear_stop      # m from the rear bumper (lidar)
        self.direction = None
        self.leg_start = 0.0
        self.boxed_in = False

    def blocked(self, direction, pose, gm, front_dist, rear_dist=None):
        if direction < 0:
            if rear_dist is not None and rear_dist < self.rear_stop:   # lidar sees behind
                return True
            return _occupied_along(gm, pose, (0.25, 0.35), -1)
        if front_dist is not None and front_dist < self.front_stop:
            return True
        return _occupied_along(gm, pose, (0.3, 0.4), +1)

    def step(self, t, pose, target, gm, min_turn_radius, front_dist=None, rear_dist=None):
        err = heading_error(pose, target)
        b = lambda d: self.blocked(d, pose, gm, front_dist, rear_dist)  # noqa: E731
        if self.direction is None:
            self.direction = -1 if not b(-1) else 1
            self.leg_start = t
        if t - self.leg_start > self.leg_s or b(self.direction):
            other = -self.direction
            if not b(other):
                self.direction, self.leg_start = other, t
            elif b(self.direction):
                self.boxed_in = True          # both ways blocked: wait (stuck logic takes over)
                return 0.0, 0.0
        self.boxed_in = False
        v = self.direction * self.speed
        return v, math.copysign(self.speed / min_turn_radius, err)


def speed_for_curvature(curvature, base_speed, max_speed, straight=0.4, tight=1.5):
    """Faster on straight path sections, base speed in curves (curvature in 1/m)."""
    k = abs(curvature)
    if max_speed <= base_speed or k >= tight:
        return base_speed
    if k <= straight:
        return max_speed
    f = (tight - k) / (tight - straight)
    return base_speed + f * (max_speed - base_speed)


def pure_pursuit(pose, path, lookahead, speed, min_turn_radius):
    """Car-like path following. pose = (x, y, yaw). Returns (v, w, target_index).

    The truck can't turn on the spot, so the turn rate is capped at
    v / min_turn_radius; a target behind the truck gives a max-lock turn."""
    x, y, yaw = pose
    # first point at least `lookahead` away, searching from the closest point onward
    dists = [math.hypot(px - x, py - y) for px, py in path]
    i0 = min(range(len(path)), key=lambda i: dists[i])
    idx = len(path) - 1
    for i in range(i0, len(path)):
        if dists[i] >= lookahead:
            idx = i
            break
    tx, ty = path[idx]
    dx, dy = tx - x, ty - y
    # target in the truck frame
    lx = math.cos(yaw) * dx + math.sin(yaw) * dy
    ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
    ld = max(1e-3, math.hypot(lx, ly))
    curvature = 2.0 * ly / (ld * ld)
    w = speed * curvature
    w_max = speed / min_turn_radius
    if lx < 0:   # target behind: turn as hard as possible towards it
        w = math.copysign(w_max, ly if abs(ly) > 1e-3 else 1.0)
    w = max(-w_max, min(w_max, w))
    return speed, w, idx
