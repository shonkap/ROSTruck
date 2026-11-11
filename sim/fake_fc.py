#!/usr/bin/env python3
"""Fake flight controller standing in for the SEQURE H743 (ArduRover 4.7) so
the ROS2 side (fc_bridge) can be developed without hardware.

Speaks real MAVLink over UDP:
  - Streams HEARTBEAT, ATTITUDE, RAW_IMU, GPS_RAW_INT, SYS_STATUS - the same
    messages the real FC streams (it sends RAW_IMU, not SCALED_IMU2).
  - Reports GUIDED + armed by default, and honours SET_MODE /
    MAV_CMD_DO_SET_MODE / MAV_CMD_COMPONENT_ARM_DISARM.
  - Acts on SET_ATTITUDE_TARGET like ArduRover GUIDED does
    (speed = thrust * WP_SPEED, turn at body_yaw_rate) with a trivial
    kinematic model, so lat/lon/heading move when cmd_vel is published.

Not a physics sim; ArduPilot SITL is the next step up from this.
"""
import argparse
import math
import time

from pymavlink import mavutil

MAV = mavutil.mavlink
RATE_HZ = 20
WP_SPEED = 2.0               # m/s, matches the real FC's WP_SPEED
CMD_TIMEOUT_S = 3.0          # ArduRover GUIDED stops after ~3 s without targets
MODE_MANUAL, MODE_HOLD, MODE_GUIDED = 0, 4, 15
EARTH_R = 6378137.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--connect', default='udpout:dev:14551',
        help="Where to push MAVLink (default: %(default)s - the docker-compose "
             "'dev' service; the bridge listens with connection:=udpin:0.0.0.0:14551).")
    parser.add_argument('--mode', type=int, default=MODE_GUIDED,
                        help='Initial ArduRover custom_mode (15=GUIDED, 4=HOLD, 0=MANUAL).')
    parser.add_argument('--disarmed', action='store_true', help='Start disarmed.')
    args = parser.parse_args()

    print('fake_fc: pushing telemetry to {}'.format(args.connect), flush=True)
    mav = mavutil.mavlink_connection(
        args.connect, source_system=1, source_component=MAV.MAV_COMP_ID_AUTOPILOT1)

    mode, armed = args.mode, not args.disarmed
    lat0, lon0 = math.radians(47.6062), math.radians(-122.3321)
    north = east = 0.0
    yaw = 0.0                 # NED, rad
    speed = yaw_rate = 0.0
    last_cmd = -math.inf
    voltage_mv = 12600
    t0 = time.monotonic()
    tick = 0

    def ack(command, result=MAV.MAV_RESULT_ACCEPTED):
        mav.mav.command_ack_send(command, result)

    try:
        while True:
            now = time.monotonic()
            dt = 1.0 / RATE_HZ
            boot_ms = int((now - t0) * 1000)

            # ---- inbound
            while True:
                msg = mav.recv_match(blocking=False)
                if msg is None:
                    break
                mtype = msg.get_type()
                if mtype in ('HEARTBEAT', 'BAD_DATA'):
                    continue
                if mtype == 'SET_ATTITUDE_TARGET':
                    speed = max(-1.0, min(1.0, msg.thrust)) * WP_SPEED
                    yaw_rate = msg.body_yaw_rate
                    last_cmd = now
                    if tick % RATE_HZ == 0:
                        print('fake_fc: target speed={:+.2f} m/s yaw_rate={:+.2f} rad/s (FRD)'
                              .format(speed, yaw_rate), flush=True)
                    continue
                if mtype == 'SET_MODE':
                    mode = msg.custom_mode
                elif mtype == 'COMMAND_LONG':
                    if msg.command == MAV.MAV_CMD_DO_SET_MODE:
                        mode = int(msg.param2)
                    elif msg.command == MAV.MAV_CMD_COMPONENT_ARM_DISARM:
                        armed = msg.param1 > 0.5
                    ack(msg.command)
                print('fake_fc: received {}: {}'.format(mtype, msg.to_dict()), flush=True)

            # ---- "physics"
            if not (armed and mode == MODE_GUIDED) or now - last_cmd > CMD_TIMEOUT_S:
                speed = yaw_rate = 0.0
            yaw = (yaw + yaw_rate * dt) % (2 * math.pi)
            north += speed * math.cos(yaw) * dt
            east += speed * math.sin(yaw) * dt
            lat = lat0 + north / EARTH_R
            lon = lon0 + east / (EARTH_R * math.cos(lat0))

            # ---- outbound
            roll = 0.02 * math.sin(now * 0.5)
            pitch = 0.01 * math.sin(now * 0.3)
            mav.mav.attitude_send(boot_ms, roll, pitch, yaw, 0.0, 0.0, yaw_rate)
            mav.mav.raw_imu_send(
                boot_ms * 1000,
                0, 0, -1000,                       # mG, ~-1 g on Z at rest (FRD)
                0, 0, int(yaw_rate * 1000),        # mrad/s
                0, 0, 0)
            if tick % (RATE_HZ // 5) == 0:
                mav.mav.gps_raw_int_send(
                    boot_ms * 1000, 3,
                    int(math.degrees(lat) * 1e7), int(math.degrees(lon) * 1e7),
                    50000, 120, 200, int(abs(speed) * 100),
                    int(math.degrees(yaw) * 100) % 36000, 12)
            if tick % (RATE_HZ // 2) == 0:
                voltage_mv = max(10500, voltage_mv - 1)
                mav.mav.sys_status_send(
                    0, 0, 0, 0, voltage_mv, 150,
                    max(0, int((voltage_mv - 10500) / (12600 - 10500) * 100)),
                    0, 0, 0, 0, 0, 0)
            if tick % RATE_HZ == 0:
                base_mode = MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                if armed:
                    base_mode |= MAV.MAV_MODE_FLAG_SAFETY_ARMED
                mav.mav.heartbeat_send(
                    MAV.MAV_TYPE_GROUND_ROVER, MAV.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    base_mode, mode, MAV.MAV_STATE_ACTIVE)

            tick += 1
            time.sleep(dt)
    except KeyboardInterrupt:
        print('fake_fc: stopping.')


if __name__ == '__main__':
    main()
