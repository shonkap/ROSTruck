"""Learns how the truck really steers and corrects the FC config. ROS-free, Python 3.6.

The truck is NOT skid steer: one throttle drives all four wheels the same
direction, and it turns only with the front + rear steering servos
(SERVO1 = front, SERVO2 = rear reversed, both GroundSteering). So the only
levers are: how much steering the FC applies per requested turn rate
(ATC_STR_RAT_FF), where "straight" is (SERVO1/2_TRIM), and how tight the
truck can physically turn (TURN_RADIUS) - which can differ left vs right
(bent linkage, servo hitting a stop early, trim off).

Fed with samples while the FC drives in GUIDED, it detects:

  1. GAIN      measured/commanded yaw rate consistently != 1 (unsaturated)
               -> ATC_STR_RAT_FF *= 1/ratio
  2. TRIM      told to go straight but either the servo sits off its trim
               (the FC's integrator is holding a correction) or the truck
               still curves (drift) -> TRIM moved to the PWM that actually
               drives straight (drift converted to PWM with the steering
               sensitivity learned from turns)
  3. LIMIT     steering pinned at its end stop and still under-turning
               -> learned minimum turn radius PER DIRECTION; the bridge clamps
               cmd_vel to it and TURN_RADIUS gets the larger (worse) side

Every recommendation is step-limited and bounded; the bridge applies it only
with auto_apply:=true and logs old -> new values for rollback.
"""
from collections import deque

LEFT, RIGHT = 'left', 'right'


def _median(vals):
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return None
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def normalized_steering(pwm, pwm_min, pwm_trim, pwm_max, reversed_):
    """Servo PWM -> ArduPilot steering output in [-1, 1] (+ = steer right)."""
    if pwm <= 0:
        return None
    if pwm >= pwm_trim:
        out = (pwm - pwm_trim) / float(max(1, pwm_max - pwm_trim))
    else:
        out = (pwm - pwm_trim) / float(max(1, pwm_trim - pwm_min))
    out = max(-1.0, min(1.0, out))
    return -out if reversed_ else out


class SteeringMonitor(object):
    # sample selection
    MIN_SPEED = 0.15            # m/s, below this yaw rate says little
    MIN_TURN = 0.10             # rad/s commanded to count as "turning"
    MAX_STRAIGHT = 0.02         # rad/s commanded to count as "straight"
    STEADY_S = 1.0              # command must be unchanged this long
    SATURATED = 0.95            # |steering| above this = at the end stop
    # decision thresholds
    MIN_SAMPLES = 60            # ~3 s of steady data at 20 Hz per bin
    GAIN_TOL = 0.12             # |ratio - 1| tolerated
    TRIM_TOL = 0.04             # steering offset tolerated when straight (4%)
    DRIFT_TOL = 0.02            # rad/s of yaw tolerated when commanded straight
    # step limits / bounds
    FF_STEP = (0.67, 1.5)       # max factor per change
    FF_BOUNDS = (0.05, 3.0)
    TRIM_STEP_US = 25           # max PWM change per trim update
    TRIM_BOUNDS = (1300, 1700)
    RADIUS_BOUNDS = (0.15, 5.0)

    def __init__(self, window=200):
        self._turn = {LEFT: deque(maxlen=window), RIGHT: deque(maxlen=window)}
        self._straight = deque(maxlen=window)
        self._last_cmd = None
        self._cmd_since = None
        self.min_radius = {LEFT: None, RIGHT: None}

    def reset(self):
        for d in self._turn.values():
            d.clear()
        self._straight.clear()

    def add_sample(self, t, cmd_yaw, meas_yaw, speed, steer_front, pwm_front, pwm_rear):
        """cmd_yaw/meas_yaw: rad/s, ROS convention (+ = left). speed: m/s (>0 fwd).
        steer_front: normalized front steering [-1,1] (+ = right).
        Returns True if the sample was used."""
        if self._last_cmd is None or abs(cmd_yaw - self._last_cmd) > 0.02:
            self._last_cmd, self._cmd_since = cmd_yaw, t
            return False
        if t - self._cmd_since < self.STEADY_S or speed < self.MIN_SPEED or steer_front is None:
            return False
        if abs(cmd_yaw) >= self.MIN_TURN:
            side = LEFT if cmd_yaw > 0 else RIGHT
            self._turn[side].append((cmd_yaw, meas_yaw, speed, steer_front))
            return True
        if abs(cmd_yaw) <= self.MAX_STRAIGHT:
            self._straight.append((pwm_front, pwm_rear, meas_yaw, steer_front))
            return True
        return False

    # ------------------------------------------------------------- analysis

    def side_stats(self, side):
        s = self._turn[side]
        if len(s) < self.MIN_SAMPLES:
            return None
        unsat = [m / c for c, m, v, st in s if abs(st) < self.SATURATED]
        sat = [(c, m, v) for c, m, v, st in s if abs(st) >= self.SATURATED]
        return {
            'n': len(s),
            'ratio': _median(unsat) if len(unsat) >= self.MIN_SAMPLES // 2 else None,
            'saturated_frac': len(sat) / float(len(s)),
            # tightest radius actually achieved while pinned at the stop
            'sat_radius': _median([v / abs(m) for c, m, v in sat if abs(m) > 1e-3]) if sat else None,
            'sat_ratio': _median([m / c for c, m, v in sat]) if sat else None,
        }

    def steer_sensitivity(self):
        """Median yaw rate per unit of normalized steering (negative: + steering
        = right = negative ROS yaw rate), from unsaturated turns."""
        vals = []
        for side in (LEFT, RIGHT):
            vals += [m / st for c, m, v, st in self._turn[side]
                     if 0.05 < abs(st) < self.SATURATED]
        return _median(vals) if len(vals) >= self.MIN_SAMPLES // 2 else None

    def recommend(self, params):
        """params: dict of current FC values (ATC_STR_RAT_FF, SERVO1/2_TRIM,
        SERVO1/2_MIN/MAX, TURN_RADIUS). Returns (changes dict, notes list)."""
        changes, notes = {}, []
        stats = {s: self.side_stats(s) for s in (LEFT, RIGHT)}

        # 2. TRIM first: an off-centre trim also shows up as left/right asymmetry.
        if len(self._straight) >= self.MIN_SAMPLES:
            drift = _median([row[2] for row in self._straight])
            k = self.steer_sensitivity()
            for idx, ch in ((0, 'SERVO1'), (1, 'SERVO2')):
                trim = params.get(ch + '_TRIM')
                lo, hi = params.get(ch + '_MIN'), params.get(ch + '_MAX')
                if trim is None or lo is None or hi is None:
                    continue
                actual = _median([row[idx] for row in self._straight if row[idx] > 0])
                if actual is None:
                    continue
                span = (hi - lo) / 2.0
                target = actual
                tol_us = self.TRIM_TOL * span
                why = '{} sits at {:.0f} us when driving straight'.format(ch, actual)
                # Remaining drift: only the FRONT axle is corrected (yaw rate alone
                # can't tell front from rear; front is what you'd trim by hand).
                if ch == 'SERVO1' and k and abs(drift) > self.DRIFT_TOL:
                    d_steer = -drift / k                      # normalized, + = right
                    sign = -1.0 if params.get('SERVO1_REVERSED', 0) > 0.5 else 1.0
                    target = actual + sign * d_steer * span
                    why = 'truck curves {:+.3f} rad/s when told to go straight'.format(drift)
                    tol_us = 3.0   # the drift itself already passed DRIFT_TOL
                if abs(target - trim) > tol_us:
                    step = max(-self.TRIM_STEP_US, min(self.TRIM_STEP_US, target - trim))
                    new = int(round(max(self.TRIM_BOUNDS[0], min(self.TRIM_BOUNDS[1], trim + step))))
                    if new != int(trim):
                        changes[ch + '_TRIM'] = new
                        notes.append('{} (trim {:.0f}) -> {}_TRIM {}'.format(why, trim, ch, new))
            if changes:
                # re-centre first, then judge gain/limits on fresh data
                return changes, notes

        # 3. LIMITS per direction
        for side in (LEFT, RIGHT):
            st = stats[side]
            if st and st['saturated_frac'] > 0.3 and st['sat_radius'] and \
                    st['sat_ratio'] is not None and st['sat_ratio'] < 1.0 - self.GAIN_TOL:
                r = max(self.RADIUS_BOUNDS[0], min(self.RADIUS_BOUNDS[1], st['sat_radius']))
                self.min_radius[side] = r
                notes.append('{}: steering at its end stop, tightest turn radius {:.2f} m '
                             '(cmd_vel will be limited to that)'.format(side, r))
        known = [r for r in self.min_radius.values() if r]
        if known:
            worst = round(max(known), 2)
            cur = params.get('TURN_RADIUS')
            if cur is None or abs(cur - worst) > 0.05:
                changes['TURN_RADIUS'] = worst
            if len(known) == 2 and abs(self.min_radius[LEFT] - self.min_radius[RIGHT]) > 0.15:
                notes.append('asymmetric: left {:.2f} m vs right {:.2f} m minimum radius - check '
                             'steering linkage / servo end points (SERVOn_MIN/MAX)'.format(
                                 self.min_radius[LEFT], self.min_radius[RIGHT]))

        # 1. GAIN from unsaturated turning, both sides
        ratios = [stats[s]['ratio'] for s in (LEFT, RIGHT) if stats[s] and stats[s]['ratio']]
        ff = params.get('ATC_STR_RAT_FF')
        if ratios and ff:
            r = sum(ratios) / len(ratios)
            if abs(r - 1.0) > self.GAIN_TOL and r > 0.05:
                factor = max(self.FF_STEP[0], min(self.FF_STEP[1], 1.0 / r))
                new = round(max(self.FF_BOUNDS[0], min(self.FF_BOUNDS[1], ff * factor)), 3)
                if abs(new - ff) > 1e-3:
                    changes['ATC_STR_RAT_FF'] = new
                    notes.append('truck turns at {:.0f}% of the requested rate -> '
                                 'ATC_STR_RAT_FF {:.3f} -> {:.3f}'.format(100 * r, ff, new))
            if len(ratios) == 2 and abs(ratios[0] - ratios[1]) > 0.2:
                notes.append('left/right response differs ({:.0f}% vs {:.0f}%) - usually trim'.format(
                    100 * stats[LEFT]['ratio'], 100 * stats[RIGHT]['ratio']))
        return changes, notes

    def clamp_yaw_rate(self, yaw_rate, speed):
        """Limit a requested yaw rate (+ = left) to what the truck can do at `speed`."""
        side = LEFT if yaw_rate > 0 else RIGHT
        r = self.min_radius.get(side)
        if not r or abs(speed) < 1e-3:
            return yaw_rate
        limit = abs(speed) / r
        return max(-limit, min(limit, yaw_rate))

    def summary(self):
        out = {}
        for side in (LEFT, RIGHT):
            st = self.side_stats(side)
            out[side] = {'samples': len(self._turn[side]),
                         'ratio': None if not st or st['ratio'] is None else round(st['ratio'], 3),
                         'saturated': None if not st else round(st['saturated_frac'], 2),
                         'min_radius': self.min_radius[side]}
        out['straight_samples'] = len(self._straight)
        return out

