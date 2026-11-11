"""SLAM pose -> ArduPilot external navigation (VISION_POSITION_ESTIMATE). ROS-free, py3.6.

Why: ArduRover's GUIDED mode needs a position estimate. Indoors there is no
GPS fix, so without this the FC refuses GUIDED and the Jetson can never
drive. With VISO_TYPE=1 and EK3_SRC1_POSXY/VELXY/YAW=6 the FC's EKF uses the
SLAM pose instead of GPS.

Frames: the SLAM map frame is ROS ENU (x east-ish, y north-ish, z up, yaw
CCW from x). ArduPilot wants NED (x north, y east, z down, yaw CW from
north). The map's absolute heading doesn't matter: the EKF takes yaw from
the same source, so it's self-consistent.
"""
import math


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def enu_to_ned_pose(x, y, z, roll, pitch, yaw):
    """ENU position + FLU body RPY -> NED position + FRD body RPY (MAVROS convention)."""
    return y, x, -z, roll, -pitch, wrap_pi(math.pi / 2.0 - yaw)


class VisionFeed(object):
    """Turns a stream of map-frame poses into VISION_POSITION/SPEED_ESTIMATE fields.

    * velocity by finite difference (smoothed), NED world frame
    * reset_counter bumps when SLAM jumps (loop closure / relocalisation), so
      the EKF re-anchors instead of believing the truck teleported
    * covariance grows when tracking quality drops (caller passes it)
    """

    JUMP_M = 0.25          # position jump that counts as a SLAM reset
    JUMP_RAD = 0.35
    VEL_ALPHA = 0.5        # low-pass on finite-difference velocity

    def __init__(self):
        self.reset_counter = 0
        self._last = None      # (t, n, e, d, yaw)
        self._vel = (0.0, 0.0, 0.0)

    def update(self, t, x, y, z, roll, pitch, yaw, pos_std=0.05, yaw_std=0.05):
        """t seconds; ENU pose. Returns dict with ned position/attitude, velocity,
        covariance (21-element upper triangle) and reset_counter."""
        n, e, d, r, p, yw = enu_to_ned_pose(x, y, z, roll, pitch, yaw)
        if self._last is not None:
            dt = t - self._last[0]
            dn, de, dd = n - self._last[1], e - self._last[2], d - self._last[3]
            dyaw = abs(wrap_pi(yw - self._last[4]))
            if math.hypot(dn, de) > self.JUMP_M + 2.0 * max(dt, 0.0) or dyaw > self.JUMP_RAD + 3.0 * max(dt, 0.0):
                self.reset_counter = (self.reset_counter + 1) % 256
                self._vel = (0.0, 0.0, 0.0)
            elif dt > 1e-3:
                a = self.VEL_ALPHA
                self._vel = tuple(a * (v_new / dt) + (1.0 - a) * v_old
                                  for v_new, v_old in zip((dn, de, dd), self._vel))
        self._last = (t, n, e, d, yw)
        pv, av = pos_std ** 2, yaw_std ** 2
        # upper-triangular 6x6 (x, y, z, roll, pitch, yaw), row-major: 21 values
        cov = [0.0] * 21
        for i, idx in enumerate((0, 6, 11, 15, 18, 20)):
            cov[idx] = pv if i < 3 else av
        return {'n': n, 'e': e, 'd': d, 'roll': r, 'pitch': p, 'yaw': yw,
                'vn': self._vel[0], 've': self._vel[1], 'vd': self._vel[2],
                'covariance': cov, 'reset_counter': self.reset_counter}
