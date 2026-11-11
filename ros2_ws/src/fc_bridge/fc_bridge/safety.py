"""Speed caps from collision guards, applied to every cmd_vel. ROS-free, py3.6.

Each guard publishes its own cap (stereo: forward; lidar: forward + reverse).
The most restrictive FRESH cap wins in each direction; a stale guard (no
message for max_age s) is ignored, so a dead guard can't freeze the truck -
fc_bridge warns about it instead. When a cap slows the truck down, the turn
rate is scaled with it so the truck keeps the same path (curvature).
"""
FORWARD, REVERSE = 'forward', 'reverse'


class SpeedCaps(object):
    def __init__(self, max_age=1.0):
        self.max_age = max_age
        self._caps = {}          # source -> (direction, cap m/s, time)

    def update(self, source, direction, cap, t):
        self._caps[source] = (direction, max(0.0, float(cap)), t)

    def stale_sources(self, t):
        return sorted(s for s, (_, _, ts) in self._caps.items() if t - ts > self.max_age)

    def cap(self, direction, t):
        caps = [c for d, c, ts in self._caps.values() if d == direction and t - ts <= self.max_age]
        return min(caps) if caps else None

    def apply(self, lin, ang, t):
        """Returns (lin, ang, limited_by) - limited_by is None or 'forward'/'reverse'."""
        if lin > 0:
            c = self.cap(FORWARD, t)
            if c is not None and lin > c:
                return c, ang * c / lin, FORWARD
        elif lin < 0:
            c = self.cap(REVERSE, t)
            if c is not None and -lin > c:
                return -c, ang * c / -lin, REVERSE
        return lin, ang, None
