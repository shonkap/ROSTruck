"""Speed caps from collision guards, applied to every cmd_vel. ROS-free, py3.6.

Each guard publishes its own caps (stereo: forward; lidar, map: both).
The most restrictive FRESH cap wins in each direction; a stale guard (no
message for max_age s) is ignored, so a dead guard can't freeze the truck -
fc_bridge warns about it instead. When a cap slows the truck down, the turn
rate is scaled with it so the truck keeps the same path (curvature).
"""
FORWARD, REVERSE = 'forward', 'reverse'


class SpeedCaps(object):
    def __init__(self, max_age=1.0):
        self.max_age = max_age
        self._caps = {}          # (source, direction) -> (cap m/s, time)

    def update(self, source, direction, cap, t):
        self._caps[(source, direction)] = (max(0.0, float(cap)), t)

    def stale_sources(self, t):
        return sorted(set(s for (s, _), (_, ts) in self._caps.items() if t - ts > self.max_age))

    def cap(self, direction, t):
        """(strictest fresh cap, its source) for a direction, or (None, None)."""
        caps = [(c, s) for (s, d), (c, ts) in self._caps.items() if d == direction and t - ts <= self.max_age]
        return min(caps) if caps else (None, None)

    def apply(self, lin, ang, t):
        """Returns (lin, ang, limited_by) - limited_by is None or the source that capped it."""
        if lin == 0:
            return lin, ang, None
        c, source = self.cap(FORWARD if lin > 0 else REVERSE, t)
        if c is None or abs(lin) <= c:
            return lin, ang, None
        scale = c / abs(lin)
        return lin * scale, ang * scale, source
