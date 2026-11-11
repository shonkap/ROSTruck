"""MAVLink <-> ROS2 bridge for the SEQURE H743 running ArduRover.

One node owns the one MAVLink link (USB serial on the truck, UDP in sim).
It has to be one node: two processes opening the same /dev/ttyACM* each get
a random subset of the incoming bytes and both see corrupted MAVLink.

Telemetry in (FC -> ROS), all converted to ROS conventions (FLU/ENU, SI):
    fc/imu        sensor_msgs/Imu          ATTITUDE + RAW_IMU
    fc/gps/fix    sensor_msgs/NavSatFix    GPS_RAW_INT
    fc/battery    sensor_msgs/BatteryState SYS_STATUS
    fc/mode       std_msgs/String          HEARTBEAT (e.g. "GUIDED")
    fc/armed      std_msgs/Bool            HEARTBEAT
    fc/status     std_msgs/String          human-readable summary

Commands out (ROS -> FC):
    cmd_vel       geometry_msgs/Twist      linear.x [m/s], angular.z [rad/s]

cmd_vel is relayed as SET_ATTITUDE_TARGET (speed + turn rate), which ArduRover
only acts on in GUIDED mode. Steering geometry stays on the FC: this truck's
rear steer is set up in ArduRover as a second, reversed GroundSteering output
(SERVO2_FUNCTION=26, SERVO2_REVERSED=1), so the FC mixes front/rear steering
itself and this node only asks for "how fast" and "how hard to turn".

Safety: this node never arms the FC and never changes its mode. Switching to
GUIDED and arming stays with the pilot on the RC transmitter. If cmd_vel stops
arriving for cmd_timeout_s, the node commands a stop.

Python 3.6 compatible on purpose: that is what ROS2 Humble runs on in the
dustynv L4T R32.7.1 image for the original Jetson Nano.
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, String

from pymavlink import mavutil

from fc_bridge.steering_monitor import SteeringMonitor, normalized_steering
from fc_bridge.safety import FORWARD, REVERSE, SpeedCaps
from fc_bridge.vision import VisionFeed
from fc_bridge.conversions import (
    MILLIGRAVITY_TO_MPS2,
    ROVER_MODE_GUIDED,
    frd_to_flu,
    gps_fix_type_to_navsat_status,
    ned_attitude_to_enu_quaternion,
    twist_to_rover_attitude_target,
)

MAV = mavutil.mavlink

# SET_ATTITUDE_TARGET type_mask bits: use yaw rate + thrust, ignore the rest.
_ATT_IGNORE_ROLL_RATE = 1
_ATT_IGNORE_PITCH_RATE = 2
_ATT_IGNORE_ATTITUDE = 128
ATTITUDE_TARGET_MASK = _ATT_IGNORE_ROLL_RATE | _ATT_IGNORE_PITCH_RATE | _ATT_IGNORE_ATTITUDE

# Rough 1-sigma noise for the ArduPilot EKF attitude and the raw IMU, used to
# fill the covariances robot_localization / RTAB-Map need. Tune if needed.
ORIENTATION_VAR_RP = 0.02 ** 2   # rad^2, roll/pitch
ORIENTATION_VAR_YAW = 0.05 ** 2  # rad^2, yaw (compass-dependent)
ANGULAR_VEL_VAR = 0.01 ** 2      # (rad/s)^2
LINEAR_ACCEL_VAR = 0.1 ** 2      # (m/s^2)^2


class MavlinkBridgeNode(Node):
    def __init__(self):
        super().__init__('mavlink_bridge_node')

        p = self.declare_parameter
        p('connection', '/dev/fc0')
        p('baud', 115200)
        p('source_system', 1)
        p('source_component', MAV.MAV_COMP_ID_ONBOARD_COMPUTER)
        p('poll_rate_hz', 100.0)
        p('imu_rate_hz', 20.0)
        p('gps_rate_hz', 5.0)
        p('battery_rate_hz', 2.0)
        p('link_timeout_s', 3.0)
        p('reconnect_period_s', 2.0)
        p('frame_id', 'fc_link')
        p('gps_frame_id', 'gps_link')
        p('command_rate_hz', 10.0)
        p('cmd_timeout_s', 0.5)
        p('max_yaw_rate_dps', 90.0)
        p('default_wp_speed', 2.0)
        # Steering self-check (see steering_monitor.py). Report-only unless auto_apply.
        p('steering_monitor', True)
        p('auto_apply', False)
        p('monitor_period_s', 10.0)
        p('limits_file', '~/.ros/steering_limits.yaml')
        p('param_log', '~/.ros/fc_param_changes.log')
        # SLAM pose -> FC (VISION_POSITION_ESTIMATE) so GUIDED works without GPS.
        # FC needs VISO_TYPE=1, EK3_SRC1_POSXY=6, EK3_SRC1_VELXY=6, EK3_SRC1_YAW=6.
        p('vision_pose', False)
        p('vision_rate_hz', 15.0)
        p('vision_map_frame', 'map')
        p('vision_base_frame', 'base_link')
        p('vision_max_age_s', 0.5)      # never send a pose older than this
        p('vision_pos_std', 0.05)       # m, reported to the EKF
        p('vision_yaw_std', 0.05)       # rad
        # EKF origin to set when there's no GPS (any fixed point works indoors)
        p('origin_lat', 47.6062)
        p('origin_lon', -122.3321)
        p('origin_alt', 50.0)

        g = self._param
        self._connection = g('connection')
        self._baud = g('baud')
        self._source_system = g('source_system')
        self._source_component = g('source_component')
        self._link_timeout = g('link_timeout_s')
        self._reconnect_period = g('reconnect_period_s')
        self._frame_id = g('frame_id')
        self._gps_frame_id = g('gps_frame_id')
        self._cmd_timeout = g('cmd_timeout_s')
        self._max_yaw_rate = math.radians(g('max_yaw_rate_dps'))
        self._wp_speed = g('default_wp_speed')
        self._stream_rates = {
            MAV.MAVLINK_MSG_ID_ATTITUDE: g('imu_rate_hz'),
            MAV.MAVLINK_MSG_ID_RAW_IMU: g('imu_rate_hz'),
            MAV.MAVLINK_MSG_ID_GPS_RAW_INT: g('gps_rate_hz'),
            MAV.MAVLINK_MSG_ID_SYS_STATUS: g('battery_rate_hz'),
            # for the steering monitor: servo PWMs + FC speed estimate
            MAV.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: g('imu_rate_hz'),
            MAV.MAVLINK_MSG_ID_VFR_HUD: 5.0,
        }

        self._imu_pub = self.create_publisher(Imu, 'fc/imu', qos_profile_sensor_data)
        self._gps_pub = self.create_publisher(NavSatFix, 'fc/gps/fix', qos_profile_sensor_data)
        self._battery_pub = self.create_publisher(BatteryState, 'fc/battery', 10)
        self._mode_pub = self.create_publisher(String, 'fc/mode', 10)
        self._armed_pub = self.create_publisher(Bool, 'fc/armed', 10)
        self._status_pub = self.create_publisher(String, 'fc/status', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        # Speed caps from the collision guards (stereo: forward; lidar: both ways).
        self._caps = SpeedCaps(max_age=1.0)
        for topic, source, direction in (('safety/speed_limit', 'stereo guard', FORWARD),
                                         ('safety/lidar/forward_limit', 'lidar guard', FORWARD),
                                         ('safety/lidar/reverse_limit', 'lidar guard (rear)', REVERSE)):
            self.create_subscription(
                Float32, topic,
                lambda m, s=source, d=direction: self._caps.update(s, d, m.data, self._now()), 10)
        self._limit_note = 0.0

        # Link state.
        self._mav = None
        self._last_connect_attempt = -math.inf
        self._fc_sysid = None
        self._fc_compid = None
        self._last_fc_heartbeat = -math.inf
        self._fc_mode = None
        self._fc_mode_name = 'UNKNOWN'
        self._fc_armed = False
        self._boot = time.monotonic()

        # Latest telemetry pieces merged into one Imu message.
        self._quat = (0.0, 0.0, 0.0, 1.0)
        self._ang_vel = (0.0, 0.0, 0.0)
        self._lin_acc = (0.0, 0.0, 0.0)

        # Command relay state.
        self._last_cmd = None
        self._last_cmd_time = -math.inf
        self._sent_yaw = 0.0       # yaw rate actually requested (ROS, + = left), after clamping

        # Steering monitor state.
        self._fc_params = {}
        self._servo_pwm = (0, 0)
        self._groundspeed = 0.0
        self._monitor = SteeringMonitor() if g('steering_monitor') else None
        self._auto_apply = g('auto_apply')
        self._limits_file = os.path.expanduser(g('limits_file'))
        self._param_log = os.path.expanduser(g('param_log'))
        self._pending_sets = {}
        self._report_pub = self.create_publisher(String, 'fc/steering_report', 10)
        if self._monitor is not None:
            self._load_limits()
            self.create_timer(g('monitor_period_s'), self._monitor_tick)

        self._vision = None
        if g('vision_pose'):
            import tf2_ros
            self._vision = VisionFeed()
            self._tf_buf = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)
            self._vision_sent = 0
            self._vision_stale = 0
            self._origin_sent = -math.inf
            self.create_timer(1.0 / g('vision_rate_hz'), self._send_vision)
            self.create_timer(10.0, self._vision_report)
            self.get_logger().info('vision_pose: feeding {} -> {} to the FC as external navigation'.format(
                g('vision_map_frame'), g('vision_base_frame')))
        self._relaying = False

        self.create_timer(1.0 / g('poll_rate_hz'), self._poll)
        self.create_timer(1.0, self._housekeeping)
        self.create_timer(1.0 / g('command_rate_hz'), self._send_command)
        self._housekeeping()

    def _param(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ link

    def _now(self):
        return time.monotonic()

    def _link_up(self):
        return (self._mav is not None and self._fc_sysid is not None
                and self._now() - self._last_fc_heartbeat < self._link_timeout)

    def _connect(self):
        self._last_connect_attempt = self._now()
        try:
            self._mav = mavutil.mavlink_connection(
                self._connection, baud=self._baud,
                source_system=self._source_system,
                source_component=self._source_component,
                autoreconnect=False,
            )
        except Exception as exc:  # serial.SerialException, OSError, ...
            self._mav = None
            self.get_logger().warn(
                "Can't open '{}': {} (retrying every {:.0f}s)".format(
                    self._connection, exc, self._reconnect_period),
                throttle_duration_sec=30.0)
            return
        self._fc_sysid = None
        self.get_logger().info(
            "Opened '{}', waiting for FC heartbeat...".format(self._connection))

    def _drop_link(self, reason):
        self.get_logger().error('MAVLink link lost: {}'.format(reason))
        try:
            if self._mav is not None:
                self._mav.close()
        except Exception:
            pass
        self._mav = None
        self._fc_sysid = None
        self._relaying = False
        self._publish_status()

    def _housekeeping(self):
        """1 Hz: (re)connect, send our companion heartbeat, watch link health."""
        if self._mav is None:
            if self._now() - self._last_connect_attempt >= self._reconnect_period:
                self._connect()
            return

        try:
            self._mav.mav.heartbeat_send(
                MAV.MAV_TYPE_ONBOARD_CONTROLLER, MAV.MAV_AUTOPILOT_INVALID, 0, 0,
                MAV.MAV_STATE_ACTIVE)
        except Exception as exc:
            self._drop_link(exc)
            return

        if self._fc_sysid is not None and not self._link_up():
            self.get_logger().warn(
                'No FC heartbeat for {:.1f}s'.format(self._now() - self._last_fc_heartbeat),
                throttle_duration_sec=5.0)
            self._relaying = False
        self._publish_status()

    def _on_fc_connected(self):
        self.get_logger().info('FC heartbeat: system {} component {}.'.format(
            self._fc_sysid, self._fc_compid))
        # Ask for exactly the messages we use, at the rates we want. This is
        # the modern replacement for REQUEST_DATA_STREAM and isn't overridden
        # by the FC's SRx_* stream-rate params.
        for msg_id, rate_hz in self._stream_rates.items():
            interval_us = int(1e6 / rate_hz) if rate_hz > 0 else -1
            self._mav.mav.command_long_send(
                self._fc_sysid, self._fc_compid,
                MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, interval_us, 0, 0, 0, 0, 0)
        # ArduRover scales SET_ATTITUDE_TARGET thrust by WP_SPEED (falling
        # back to CRUISE_SPEED when WP_SPEED is 0); fetch both.
        for name in self.WATCHED_PARAMS:
            self._mav.mav.param_request_read_send(
                self._fc_sysid, self._fc_compid, name.encode('ascii'), -1)

    def _poll(self):
        if self._mav is None:
            return
        try:
            while True:
                msg = self._mav.recv_match(blocking=False)
                if msg is None:
                    return
                self._handle_message(msg)
        except Exception as exc:  # USB unplugged, etc.
            self._drop_link(exc)

    # ------------------------------------------------------------- telemetry

    def _handle_message(self, msg):
        msg_type = msg.get_type()
        if msg_type == 'BAD_DATA':
            return

        if msg_type == 'HEARTBEAT':
            # Only the autopilot's heartbeat counts (not GCSs, gimbals, or our own echo).
            if (msg.autopilot == MAV.MAV_AUTOPILOT_INVALID
                    or msg.get_srcComponent() != MAV.MAV_COMP_ID_AUTOPILOT1):
                return
            first = self._fc_sysid is None
            self._fc_sysid = msg.get_srcSystem()
            self._fc_compid = msg.get_srcComponent()
            self._mav.target_system = self._fc_sysid
            self._mav.target_component = self._fc_compid
            self._last_fc_heartbeat = self._now()
            self._fc_mode = msg.custom_mode
            self._fc_armed = bool(msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)
            if first:
                self._on_fc_connected()
            self._publish_mode(msg)
            return

        if self._fc_sysid is None or msg.get_srcSystem() != self._fc_sysid:
            return

        if msg_type == 'ATTITUDE':
            self._quat = ned_attitude_to_enu_quaternion(msg.roll, msg.pitch, msg.yaw)
            self._ang_vel = frd_to_flu(msg.rollspeed, msg.pitchspeed, msg.yawspeed)
            self._publish_imu()
        elif msg_type in ('RAW_IMU', 'SCALED_IMU', 'SCALED_IMU2', 'SCALED_IMU3'):
            # ArduPilot fills RAW_IMU accel in mG like SCALED_IMU* (zacc ~ -1000
            # at rest = specific force pointing up in FRD). After FRD->FLU this
            # becomes +9.8 m/s^2 on z, which is what ROS consumers expect.
            self._lin_acc = frd_to_flu(
                msg.xacc * MILLIGRAVITY_TO_MPS2,
                msg.yacc * MILLIGRAVITY_TO_MPS2,
                msg.zacc * MILLIGRAVITY_TO_MPS2)
        elif msg_type == 'GPS_RAW_INT':
            self._publish_gps(msg)
        elif msg_type == 'SYS_STATUS':
            self._publish_battery(msg)
        elif msg_type == 'PARAM_VALUE':
            self._handle_param(msg)
        elif msg_type == 'SERVO_OUTPUT_RAW':
            self._servo_pwm = (msg.servo1_raw, msg.servo2_raw)
            self._monitor_sample()
        elif msg_type == 'VFR_HUD':
            self._groundspeed = msg.groundspeed
        elif msg_type == 'STATUSTEXT':
            self.get_logger().info('FC: {}'.format(msg.text))

    def _handle_param(self, msg):
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii', 'ignore')
        name = name.rstrip('\x00')
        if name in self.WATCHED_PARAMS:
            self._fc_params[name] = msg.param_value
            want = self._pending_sets.pop(name, None)
            if want is not None:
                ok = abs(msg.param_value - want) < 1e-3
                self.get_logger().info('FC {} = {:g} ({})'.format(
                    name, msg.param_value, 'applied' if ok else 'NOT applied, wanted {:g}'.format(want)))
        if name == 'WP_SPEED' and msg.param_value > 0.0:
            self._wp_speed = msg.param_value
            self.get_logger().info('FC WP_SPEED = {:.2f} m/s (cmd_vel scaling)'.format(
                self._wp_speed))
        elif name == 'CRUISE_SPEED' and msg.param_value > 0.0 and self._wp_speed <= 0.0:
            self._wp_speed = msg.param_value

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _publish_imu(self):
        imu = Imu()
        imu.header.stamp = self._stamp()
        imu.header.frame_id = self._frame_id
        o = imu.orientation
        o.x, o.y, o.z, o.w = self._quat
        w = imu.angular_velocity
        w.x, w.y, w.z = self._ang_vel
        a = imu.linear_acceleration
        a.x, a.y, a.z = self._lin_acc
        imu.orientation_covariance = [
            ORIENTATION_VAR_RP, 0.0, 0.0,
            0.0, ORIENTATION_VAR_RP, 0.0,
            0.0, 0.0, ORIENTATION_VAR_YAW]
        imu.angular_velocity_covariance = [
            ANGULAR_VEL_VAR, 0.0, 0.0, 0.0, ANGULAR_VEL_VAR, 0.0, 0.0, 0.0, ANGULAR_VEL_VAR]
        imu.linear_acceleration_covariance = [
            LINEAR_ACCEL_VAR, 0.0, 0.0, 0.0, LINEAR_ACCEL_VAR, 0.0, 0.0, 0.0, LINEAR_ACCEL_VAR]
        self._imu_pub.publish(imu)

    def _publish_gps(self, msg):
        fix = NavSatFix()
        fix.header.stamp = self._stamp()
        fix.header.frame_id = self._gps_frame_id
        fix.status.status = gps_fix_type_to_navsat_status(msg.fix_type)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = msg.lat * 1e-7
        fix.longitude = msg.lon * 1e-7
        # NavSatFix wants height above the WGS84 ellipsoid; use it when the FC
        # provides it (MAVLink2 extension), otherwise fall back to AMSL.
        alt_ellipsoid = getattr(msg, 'alt_ellipsoid', 0)
        fix.altitude = (alt_ellipsoid if alt_ellipsoid else msg.alt) * 1e-3

        h_acc = getattr(msg, 'h_acc', 0)
        v_acc = getattr(msg, 'v_acc', 0)
        if h_acc > 0 and v_acc > 0:
            h_var = (h_acc * 1e-3) ** 2
            fix.position_covariance = [h_var, 0.0, 0.0, 0.0, h_var, 0.0,
                                       0.0, 0.0, (v_acc * 1e-3) ** 2]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        elif msg.eph != 65535:
            # HDOP * ~UERE as a rough horizontal sigma.
            h_var = (msg.eph * 1e-2 * 3.0) ** 2
            fix.position_covariance = [h_var, 0.0, 0.0, 0.0, h_var, 0.0,
                                       0.0, 0.0, 4.0 * h_var]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._gps_pub.publish(fix)

    def _publish_battery(self, msg):
        nan = float('nan')
        battery = BatteryState()
        battery.header.stamp = self._stamp()
        battery.present = True
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        battery.voltage = msg.voltage_battery * 1e-3 if msg.voltage_battery != 65535 else nan
        # MAVLink: cA, + = discharging. ROS: A, - = discharging.
        battery.current = -msg.current_battery * 1e-2 if msg.current_battery != -1 else nan
        battery.percentage = msg.battery_remaining / 100.0 if msg.battery_remaining != -1 else nan
        battery.charge = battery.capacity = battery.design_capacity = nan
        battery.temperature = nan
        self._battery_pub.publish(battery)

    def _publish_mode(self, msg):
        self._fc_mode_name = mavutil.mode_string_v10(msg)
        self._mode_pub.publish(String(data=self._fc_mode_name))
        self._armed_pub.publish(Bool(data=self._fc_armed))

    def _publish_status(self):
        if self._mav is None:
            text = 'disconnected'
        elif self._fc_sysid is None:
            text = 'waiting_for_heartbeat'
        elif not self._link_up():
            text = 'link_timeout'
        else:
            text = 'connected mode={} armed={} relaying_cmd_vel={}'.format(
                self._fc_mode_name, self._fc_armed, self._relaying)
        self._status_pub.publish(String(data=text))

    # -------------------------------------------------------------- commands

    def _on_cmd_vel(self, msg):
        self._last_cmd = msg
        self._last_cmd_time = self._now()

    def _send_command(self):
        active = self._link_up() and self._fc_armed and self._fc_mode == ROVER_MODE_GUIDED
        if active != self._relaying:
            self._relaying = active
            if active:
                self.get_logger().info('FC is GUIDED + armed: relaying cmd_vel.')
            else:
                self.get_logger().info('Not relaying cmd_vel (FC must be GUIDED and armed).')
        if not active:
            return

        fresh = self._last_cmd is not None and \
            self._now() - self._last_cmd_time <= self._cmd_timeout
        lin = self._last_cmd.linear.x if fresh else 0.0
        ang = self._last_cmd.angular.z if fresh else 0.0
        # Collision guards: most restrictive fresh cap per direction.
        want = lin
        lin, ang, by = self._caps.apply(lin, ang, self._now())
        if by and self._now() - self._limit_note > 5.0:
            self.get_logger().warn('collision guard: {} speed {:.2f} -> {:.2f} m/s'.format(
                by, abs(want), abs(lin)))
            self._limit_note = self._now()
        stale = self._caps.stale_sources(self._now())
        if stale and self._now() - self._limit_note > 10.0:
            self.get_logger().warn('{} stopped publishing - its speed cap is not active'.format(
                ', '.join(stale)))
            self._limit_note = self._now()
        if self._monitor is not None:
            # never ask for a tighter turn than this side of the truck can do
            ang = self._monitor.clamp_yaw_rate(ang, lin)
        self._sent_yaw = ang
        thrust, yaw_rate = twist_to_rover_attitude_target(
            lin, ang, self._wp_speed, self._max_yaw_rate)
        try:
            self._mav.mav.set_attitude_target_send(
                int((self._now() - self._boot) * 1000) & 0xFFFFFFFF,
                self._fc_sysid, self._fc_compid,
                ATTITUDE_TARGET_MASK,
                [1.0, 0.0, 0.0, 0.0],  # ignored, but must be a valid quaternion
                0.0, 0.0, yaw_rate, thrust)
        except Exception as exc:
            self._drop_link(exc)

    # --------------------------------------------------- vision pose -> FC

    def _send_vision(self):
        if not self._link_up():
            return
        from rclpy.time import Time
        g = self._param
        try:
            t = self._tf_buf.lookup_transform(g('vision_map_frame'), g('vision_base_frame'), Time())
        except Exception:
            self._vision_stale += 1
            return
        stamp = Time.from_msg(t.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if age > g('vision_max_age_s'):
            self._vision_stale += 1      # SLAM/VO lost: send nothing, the EKF times out ExtNav
            return
        tr, q = t.transform.translation, t.transform.rotation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        v = self._vision.update(stamp.nanoseconds * 1e-9, tr.x, tr.y, tr.z, roll, pitch, yaw,
                                g('vision_pos_std'), g('vision_yaw_std'))
        usec = int(time.time() * 1e6)
        try:
            # EKF origin: required for position modes when there is no GPS.
            # Without GPS the FC also doesn't know the time: tell it.
            if self._now() - self._origin_sent > 30.0:
                self._mav.mav.system_time_send(usec, 0)
                self._mav.mav.set_gps_global_origin_send(
                    self._fc_sysid, int(g('origin_lat') * 1e7), int(g('origin_lon') * 1e7),
                    int(g('origin_alt') * 1000))
                self._origin_sent = self._now()
            self._mav.mav.vision_position_estimate_send(
                usec, v['n'], v['e'], v['d'], v['roll'], v['pitch'], v['yaw'],
                v['covariance'], v['reset_counter'])
            vv = g('vision_pos_std') ** 2
            self._mav.mav.vision_speed_estimate_send(
                usec, v['vn'], v['ve'], v['vd'], [vv, 0, 0, 0, vv, 0, 0, 0, vv], v['reset_counter'])
            self._vision_sent += 1
        except Exception as exc:
            self._drop_link(exc)

    def _vision_report(self):
        self.get_logger().info('vision_pose: {} sent, {} skipped (no fresh SLAM pose) in 10 s, resets={}'.format(
            self._vision_sent, self._vision_stale, self._vision.reset_counter))
        self._vision_sent = self._vision_stale = 0

    # ------------------------------------------------------ steering monitor

    WATCHED_PARAMS = ('WP_SPEED', 'CRUISE_SPEED', 'ATC_STR_RAT_FF', 'TURN_RADIUS',
                      'SERVO1_MIN', 'SERVO1_TRIM', 'SERVO1_MAX', 'SERVO1_REVERSED',
                      'SERVO2_MIN', 'SERVO2_TRIM', 'SERVO2_MAX', 'SERVO2_REVERSED')

    def _monitor_sample(self):
        if self._monitor is None or not self._relaying or self._last_cmd is None:
            return
        fp = self._fc_params
        if any(k not in fp for k in ('SERVO1_MIN', 'SERVO1_TRIM', 'SERVO1_MAX')):
            return
        lin = self._last_cmd.linear.x
        if lin <= 0.0:
            return   # reversing flips the steering geometry; learn from forward driving only
        steer = normalized_steering(self._servo_pwm[0], fp['SERVO1_MIN'], fp['SERVO1_TRIM'],
                                    fp['SERVO1_MAX'], fp.get('SERVO1_REVERSED', 0) > 0.5)
        self._monitor.add_sample(self._now(), self._sent_yaw, self._ang_vel[2],
                                 self._groundspeed, steer, self._servo_pwm[0], self._servo_pwm[1])

    def _monitor_tick(self):
        changes, notes = self._monitor.recommend(self._fc_params)
        report = self._monitor.summary()
        report['recommend'] = changes
        report['notes'] = notes
        report['auto_apply'] = self._auto_apply
        self._report_pub.publish(String(data=json.dumps(report)))
        for n in notes:
            self.get_logger().info('steering: ' + n)
        self._save_limits()
        if not changes:
            return
        if not (self._auto_apply and self._link_up()):
            self.get_logger().warn(
                'steering: recommended FC changes {} (not applied: auto_apply is false)'.format(changes),
                throttle_duration_sec=30.0)
            return
        for name, value in changes.items():
            old = self._fc_params.get(name)
            self._mav.mav.param_set_send(self._fc_sysid, self._fc_compid, name.encode('ascii'),
                                         float(value), MAV.MAV_PARAM_TYPE_REAL32)
            self._pending_sets[name] = float(value)
            self._log_param_change(name, old, value)
            self.get_logger().warn('steering: set {} {} -> {}'.format(name, old, value))
        # judge the new settings on fresh data only
        self._monitor.reset()

    def _log_param_change(self, name, old, new):
        try:
            d = os.path.dirname(self._param_log)
            if d and not os.path.isdir(d):
                os.makedirs(d)
            with open(self._param_log, 'a') as f:
                f.write('{} {} {} -> {}\n'.format(time.strftime('%Y-%m-%d %H:%M:%S'), name, old, new))
        except OSError as exc:
            self.get_logger().error('cannot write {}: {}'.format(self._param_log, exc))

    def _load_limits(self):
        try:
            import yaml
            with open(self._limits_file) as f:
                data = yaml.safe_load(f) or {}
            for side in ('left', 'right'):
                if data.get(side):
                    self._monitor.min_radius[side] = float(data[side])
            self.get_logger().info('learned min turn radius: {}'.format(self._monitor.min_radius))
        except (OSError, IOError, ValueError):
            pass

    def _save_limits(self):
        r = self._monitor.min_radius
        if not any(r.values()):
            return
        try:
            d = os.path.dirname(self._limits_file)
            if d and not os.path.isdir(d):
                os.makedirs(d)
            with open(self._limits_file, 'w') as f:
                f.write('# learned by fc_bridge steering monitor (m); delete to re-learn\n')
                for side in ('left', 'right'):
                    f.write('{}: {}\n'.format(side, r[side] if r[side] else 'null'))
        except OSError as exc:
            self.get_logger().error('cannot write {}: {}'.format(self._limits_file, exc))


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
