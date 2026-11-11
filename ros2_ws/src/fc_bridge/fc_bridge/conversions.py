"""Pure (ROS-free) conversions between MAVLink/ArduPilot conventions and ROS.

ArduPilot reports everything in aerospace conventions:
  - body frame FRD (x Forward, y Right, z Down)
  - world frame NED (x North, y East, z Down), yaw 0 = North, + = clockwise

ROS (REP-103 / REP-105) expects:
  - body frame FLU (x Forward, y Left, z Up)
  - world frame ENU (x East, y North, z Up), yaw 0 = East, + = counter-clockwise

Getting this wrong doesn't crash anything - it just silently makes SLAM /
robot_localization fuse an IMU that thinks the truck is upside down and
turning the wrong way - so it lives here, where it is unit-tested.
"""
import math

# MAVLink SCALED_IMU*/RAW_IMU (as ArduPilot fills it) and ATTITUDE units -> SI.
MILLIGRAVITY_TO_MPS2 = 0.001 * 9.80665
MILLIRAD_PER_S_TO_RAD_PER_S = 0.001

# ArduRover custom_mode numbers (ArduPilot Rover/mode.h).
ROVER_MODE_MANUAL = 0
ROVER_MODE_HOLD = 4
ROVER_MODE_GUIDED = 15


def wrap_pi(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def euler_to_quaternion(roll, pitch, yaw):
    """ZYX (yaw-pitch-roll) Euler angles in radians -> quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def frd_to_flu(x, y, z):
    """Body-frame vector FRD -> FLU (gyro rates, accelerations)."""
    return (x, -y, -z)


def ned_attitude_to_enu_quaternion(roll, pitch, yaw):
    """ArduPilot ATTITUDE (FRD body in NED world) -> ROS orientation
    (FLU body in ENU world), as quaternion (x, y, z, w).

    Same mapping MAVROS uses: roll unchanged, pitch negated, yaw -> pi/2 - yaw.
    """
    return euler_to_quaternion(roll, -pitch, wrap_pi(math.pi / 2.0 - yaw))


def gps_fix_type_to_navsat_status(fix_type):
    """MAVLink GPS_FIX_TYPE -> sensor_msgs/NavSatStatus.status value."""
    if fix_type <= 1:      # NO_GPS / NO_FIX
        return -1          # STATUS_NO_FIX
    if fix_type == 4:      # DGPS
        return 1           # STATUS_SBAS_FIX
    if fix_type >= 5:      # RTK float / fixed (and PPP)
        return 2           # STATUS_GBAS_FIX
    return 0               # 2D / 3D -> STATUS_FIX


def twist_to_rover_attitude_target(linear_x, angular_z, wp_speed, max_yaw_rate):
    """ROS cmd_vel -> (thrust, body_yaw_rate) for ArduRover SET_ATTITUDE_TARGET.

    In GUIDED mode ArduRover interprets SET_ATTITUDE_TARGET as
    "drive at speed = thrust * WP_SPEED while turning at body_yaw_rate", with
    thrust in [-1, 1] (negative = reverse). The FC's own steering mixer then
    works out the front/rear steering servo angles, so the companion computer
    never has to know this chassis has rear steer.

    Sign flip: ROS angular.z is + = turn left (CCW from above); MAVLink body
    yaw rate is FRD, + = turn right (CW from above).
    """
    if wp_speed <= 0.0:
        raise ValueError('wp_speed must be > 0')
    thrust = max(-1.0, min(1.0, linear_x / wp_speed))
    yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, -angular_z))
    return thrust, yaw_rate
