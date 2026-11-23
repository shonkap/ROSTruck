"""Collision guards: shared maths and topics. Python 3.6.

Every guard (stereo, lidar, map) publishes std_msgs/Float32 on

    safety/<source>/forward_limit   m/s cap while driving forward (99 = none)
    safety/<source>/reverse_limit   m/s cap while reversing      (99 = none)
    safety/<source>/front_distance  m ahead of the front bumper  (99 = clear)
    safety/<source>/rear_distance   m behind the rear bumper     (99 = clear)

and the lidar and stereo guards also publish what they see right now as
safety/<source>/points (PointCloud2, base_link) for LiveObstacles.

fc_bridge applies the strictest fresh limit to every cmd_vel; the navigator
and the web page use the nearest fresh distance (Obstacles).
"""
import math
import time

import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32

from truck_common.clouds import cloud_to_xy, solid_objects, thin

NO_LIMIT = 99.0
CLEAR = 50.0            # any distance >= this means "nothing seen"
SOURCES = ('stereo', 'lidar', 'map')
LIMITS = ('forward_limit', 'reverse_limit')
DISTANCES = ('front_distance', 'rear_distance')


def topic(source, name):
    return 'safety/{}/{}'.format(source, name)


def ramp(distance, stop, slow, max_speed):
    """Speed cap: 0 inside `stop`, linear up to max_speed at `slow`, none beyond."""
    if distance is None or distance >= slow:
        return NO_LIMIT
    if distance <= stop:
        return 0.0
    return max_speed * (distance - stop) / (slow - stop)


def corridor_distances(points, front_x, rear_x, half_width, min_points=2):
    """Nearest obstacle ahead of the front bumper (x > front_x) and behind the
    rear bumper (x < rear_x) within |y| <= half_width, for Nx2 points in the
    truck frame. Points inside the footprint (the truck itself, its mast,
    localization error) are ignored; min_points rejects single noisy hits.
    Returns (front, rear), each None when clear."""
    if len(points) == 0:
        return None, None
    x, y = points[:, 0], points[:, 1]
    lane = np.abs(y) <= half_width
    ahead = np.sort(x[lane & (x > front_x)] - front_x)
    behind = np.sort(rear_x - x[lane & (x < rear_x)])
    front = float(ahead[min_points - 1]) if len(ahead) >= min_points else None
    rear = float(behind[min_points - 1]) if len(behind) >= min_points else None
    return front, rear


class GuardPublisher(object):
    """Publishes one guard's limits and distances on the standard topics."""

    def __init__(self, node, source, rear=True):
        names = LIMITS + DISTANCES if rear else (LIMITS[0], DISTANCES[0])
        self._pubs = {n: node.create_publisher(Float32, topic(source, n), 10) for n in names}

    def publish(self, front, rear, stop, slow, max_speed):
        vals = {'forward_limit': ramp(front, stop, slow, max_speed),
                'reverse_limit': ramp(rear, stop, slow, max_speed),
                'front_distance': NO_LIMIT if front is None else front,
                'rear_distance': NO_LIMIT if rear is None else rear}
        for n, pub in self._pubs.items():
            pub.publish(Float32(data=float(vals[n])))


class Obstacles(object):
    """Nearest obstacle ahead / behind over all guards' fresh distances.
    Thread-safe enough for reading from HTTP threads (plain float reads)."""

    def __init__(self, node, max_age=1.0):
        self._max_age = max_age
        self._d = {}            # (source, name) -> (distance, time)
        for s in SOURCES:
            for n in DISTANCES:
                node.create_subscription(Float32, topic(s, n),
                                         lambda m, k=(s, n): self._d.__setitem__(k, (m.data, time.time())), 10)

    def _nearest(self, name, sources=SOURCES):
        now = time.time()
        vals = [d for (s, n), (d, t) in list(self._d.items())
                if n == name and s in sources and now - t <= self._max_age and d < CLEAR]
        return min(vals) if vals else None

    def front(self, sources=SOURCES):
        return self._nearest('front_distance', sources)

    def rear(self, sources=SOURCES):
        return self._nearest('rear_distance', sources)

    def seen(self):
        """Sources heard from recently."""
        now = time.time()
        return sorted(set(s for (s, _), (_, t) in list(self._d.items()) if now - t <= self._max_age))


def to_world(points, pose):
    """Truck-frame points -> map frame, pose = (x, y, yaw)."""
    if len(points) == 0:
        return points
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.c_[pose[0] + c * points[:, 0] - s * points[:, 1], pose[1] + s * points[:, 0] + c * points[:, 1]]


class LiveObstacles(object):
    """What the lidar and stereo guards saw in the last `memory` seconds, in
    the map frame - for planning around things that aren't in the SLAM map
    (yet): a cat walks in front, the path goes around it. Kept a few seconds
    so an obstacle isn't forgotten the moment it leaves the cameras' view.
    Things only a few mm in size are dropped as noise (solid_objects); the
    stop guards still react to anything close."""

    def __init__(self, node, pose_fn, memory=3.0, max_range=2.5, min_range=0.3, sources=('lidar', 'stereo')):
        self._pose_fn, self._memory, self._range, self._min = pose_fn, memory, max_range, min_range
        self._seen = []           # (time, Nx2 map points)
        self._cache = (-1e9, np.zeros((0, 2)))
        for s in sources:
            node.create_subscription(PointCloud2, topic(s, 'points'), self._on_points, 10)

    def _on_points(self, msg):
        pose = self._pose_fn()
        if pose is None:
            return
        pts = cloud_to_xy(msg)
        if len(pts):    # not too far (noisy), not on the truck itself (mast, own body)
            r = np.hypot(pts[:, 0], pts[:, 1])
            pts = pts[(r <= self._range) & (r >= self._min)]
        now = time.time()
        self._seen = [(t, p) for t, p in self._seen if now - t <= self._memory]
        self._seen.append((now, to_world(pts, pose)))

    def points(self, max_age=0.5):
        """Recent obstacle points (map frame). Recomputed at most every max_age s:
        the clustering is plain Python, too slow for every control-loop tick."""
        now = time.time()
        if now - self._cache[0] > max_age:
            fresh = [p for t, p in self._seen if now - t <= self._memory and len(p)]
            # merge repeated hits first (1 cm), then drop few-mm specks
            pts = solid_objects(thin(np.vstack(fresh), cell=0.01)) if fresh else np.zeros((0, 2))
            self._cache = (now, pts)
        return self._cache[1]
