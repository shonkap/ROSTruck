#!/usr/bin/env python3
"""Read-only health check of the MAVLink link to the FC (real truck or SITL).

Sends nothing that moves the truck: only a companion heartbeat, message
interval requests, AUTOPILOT_VERSION and parameter reads.

    .venv/bin/python scripts/fc_check.py                       # /dev/fc0 or /dev/ttyACM0
    .venv/bin/python scripts/fc_check.py --conn tcp:127.0.0.1:5762   # SITL
    .venv/bin/python scripts/fc_check.py --save-params fc.parm # dump all params

Exit code 0 = link healthy, 1 = something failed (see the FAIL lines).
Python 3.6 compatible, so it also runs on the Jetson.
"""
import argparse
import os
import sys
import time
from collections import Counter

from pymavlink import mavutil

MAV = mavutil.mavlink

# Params that matter for this truck, with what we expect (None = just show it).
KEY_PARAMS = [
    ('SERVO1_FUNCTION', 26, 'front steering (GroundSteering)'),
    ('SERVO2_FUNCTION', 26, 'rear steering (GroundSteering)'),
    ('SERVO2_REVERSED', 1, 'rear steer counter-steers'),
    ('SERVO3_FUNCTION', 70, 'throttle'),
    ('FRAME_CLASS', None, '1 = rover'),
    ('ARMING_REQUIRE', 1, '0 = always armed (unsafe for autonomy)'),
    ('FS_GCS_ENABLE', None, 'GCS/companion failsafe'),
    ('MODE_CH', None, 'mode switch channel'),
    ('MODE1', None, ''), ('MODE2', None, ''), ('MODE3', None, ''),
    ('MODE4', None, ''), ('MODE5', None, ''), ('MODE6', None, ''),
    ('WP_SPEED', None, 'cmd_vel speed scale'),
    ('CRUISE_SPEED', None, ''), ('CRUISE_THROTTLE', None, ''),
    ('ATC_STR_RAT_MAX', None, 'max turn rate deg/s'),
    ('AHRS_EKF_TYPE', None, ''), ('EK3_SRC1_POSXY', None, '3 = GPS, 6 = ExtNav'),
    ('VISO_TYPE', None, '1 = MAVLink vision (for SLAM -> FC later)'),
    ('SERIAL0_PROTOCOL', None, 'USB'),
]
ROVER_MODES = {0: 'MANUAL', 1: 'ACRO', 3: 'STEERING', 4: 'HOLD', 5: 'LOITER',
               6: 'FOLLOW', 7: 'SIMPLE', 8: 'DOCK', 9: 'CIRCLE', 10: 'AUTO',
               11: 'RTL', 12: 'SMART_RTL', 15: 'GUIDED', 16: 'INITIALISING'}

failures = []


def ok(text):
    print('  OK    ' + text)


def fail(text):
    failures.append(text)
    print('  FAIL  ' + text)


def info(text):
    print('        ' + text)


def default_conn():
    for path in ('/dev/fc0', '/dev/ttyACM0'):
        if os.path.exists(path):
            return path
    return '/dev/ttyACM0'


def wait_heartbeat(m, timeout):
    end = time.time() + timeout
    while time.time() < end:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if hb is not None and hb.autopilot != MAV.MAV_AUTOPILOT_INVALID \
                and hb.get_srcComponent() == MAV.MAV_COMP_ID_AUTOPILOT1:
            return hb
    return None


def fetch_all_params(m, timeout=30.0):
    m.mav.param_request_list_send(m.target_system, m.target_component)
    params, total = {}, None
    end = last_rx = time.time()
    end += timeout
    while time.time() < end and time.time() - last_rx < 3.0:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg is None:
            continue
        last_rx = time.time()
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii', 'ignore')
        params[name.rstrip('\x00')] = msg.param_value
        total = msg.param_count
        if total and len(params) >= total:
            break
    return params, total


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--conn', default=default_conn(),
                    help='pymavlink connection string (default: %(default)s)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--seconds', type=float, default=5.0, help='rate measurement window')
    ap.add_argument('--save-params', metavar='FILE', help='write all FC params to FILE')
    args = ap.parse_args()

    print('Connecting to {} ...'.format(args.conn))
    try:
        m = mavutil.mavlink_connection(args.conn, baud=args.baud, source_system=255,
                                       source_component=MAV.MAV_COMP_ID_ONBOARD_COMPUTER + 1)
    except Exception as exc:
        fail("can't open {}: {}".format(args.conn, exc))
        return 1

    print('\n[1] Heartbeat')
    hb = wait_heartbeat(m, 10.0)
    if hb is None:
        fail('no autopilot heartbeat in 10 s (wrong port? another program holding it?)')
        return 1
    m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
    armed = bool(hb.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)
    ok('system {} comp {}, type={} ({}), mode={} ({}), armed={}'.format(
        m.target_system, m.target_component, hb.type,
        'GROUND_ROVER' if hb.type == MAV.MAV_TYPE_GROUND_ROVER else 'NOT A ROVER?',
        hb.custom_mode, ROVER_MODES.get(hb.custom_mode, '?'), armed))
    if hb.type != MAV.MAV_TYPE_GROUND_ROVER:
        fail('FC is not reporting MAV_TYPE_GROUND_ROVER')
    info('MAVLink {}'.format('2' if m.WIRE_PROTOCOL_VERSION == '2.0' else '1'))

    print('\n[2] Firmware')
    m.mav.command_long_send(m.target_system, m.target_component,
                            MAV.MAV_CMD_REQUEST_MESSAGE, 0,
                            MAV.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
    ver = m.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3.0)
    if ver is None:
        fail('no AUTOPILOT_VERSION reply')
    else:
        v = ver.flight_sw_version
        ok('ArduPilot {}.{}.{} (git {})'.format(
            (v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF,
            bytes(ver.flight_custom_version).decode('ascii', 'ignore').rstrip('\x00')))

    print('\n[3] Telemetry rates (same requests fc_bridge makes)')
    wanted = {'ATTITUDE': 20.0, 'RAW_IMU': 20.0, 'GPS_RAW_INT': 5.0, 'SYS_STATUS': 2.0}
    for name, hz in wanted.items():
        m.mav.command_long_send(m.target_system, m.target_component,
                                MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                getattr(MAV, 'MAVLINK_MSG_ID_' + name), int(1e6 / hz),
                                0, 0, 0, 0, 0)
    time.sleep(1.0)  # let the new intervals take effect
    counts = Counter()
    bad = 0
    last_hb = None
    end = time.time() + args.seconds
    next_hb = 0.0
    while time.time() < end:
        if time.time() >= next_hb:
            m.mav.heartbeat_send(MAV.MAV_TYPE_ONBOARD_CONTROLLER, MAV.MAV_AUTOPILOT_INVALID,
                                 0, 0, MAV.MAV_STATE_ACTIVE)
            next_hb = time.time() + 1.0
        msg = m.recv_match(blocking=True, timeout=0.1)
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'BAD_DATA':
            bad += 1
            continue
        if msg.get_srcSystem() != m.target_system:
            continue
        counts[t] += 1
        if t == 'HEARTBEAT':
            last_hb = msg
        elif t == 'STATUSTEXT':
            info('FC says: ' + msg.text)
    for name, hz in sorted(wanted.items()):
        got = counts[name] / args.seconds
        line = '{:<12s} {:5.1f} Hz (asked {:.0f})'.format(name, got, hz)
        (ok if got >= 0.7 * hz else fail)(line)
    hb_hz = counts['HEARTBEAT'] / args.seconds
    (ok if hb_hz >= 0.7 else fail)('HEARTBEAT    {:5.1f} Hz'.format(hb_hz))
    (ok if bad == 0 else fail)('{} corrupt/unparsed packets'.format(bad))
    others = ', '.join('{} {}'.format(k, v) for k, v in counts.most_common()
                       if k not in wanted and k != 'HEARTBEAT')
    info('also seen: ' + (others or '-'))

    print('\n[4] Sensor snapshot')
    att = m.messages.get('ATTITUDE')
    imu = m.messages.get('RAW_IMU')
    gps = m.messages.get('GPS_RAW_INT')
    sys_s = m.messages.get('SYS_STATUS')
    if att:
        import math
        info('attitude roll {:+.1f} pitch {:+.1f} yaw {:.1f} deg'.format(
            math.degrees(att.roll), math.degrees(att.pitch), math.degrees(att.yaw) % 360))
    if imu:
        g = (imu.xacc ** 2 + imu.yacc ** 2 + imu.zacc ** 2) ** 0.5 / 1000.0
        (ok if 0.9 < g < 1.1 else fail)(
            'accel |a| = {:.2f} g (should be ~1.00 at rest)'.format(g))
    if gps:
        info('GPS fix_type {} sats {} (0/1 = no fix, expected indoors)'.format(
            gps.fix_type, gps.satellites_visible))
    if sys_s:
        info('battery {:.2f} V'.format(sys_s.voltage_battery / 1000.0)
             if sys_s.voltage_battery != 65535 else 'battery voltage not reported')
    if last_hb is not None:
        info('mode now {} armed={}'.format(
            ROVER_MODES.get(last_hb.custom_mode, last_hb.custom_mode),
            bool(last_hb.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)))

    print('\n[5] Parameters (round trip FC <-> computer)')
    t0 = time.time()
    m.mav.param_request_read_send(m.target_system, m.target_component, b'SERVO1_FUNCTION', -1)
    pv = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=3.0)
    if pv is None:
        fail('no PARAM_VALUE reply - commands are NOT reaching the FC')
    else:
        ok('param read round trip {:.0f} ms'.format((time.time() - t0) * 1000))
    params, total = fetch_all_params(m)
    (ok if total and len(params) >= total else fail)(
        'downloaded {}/{} params'.format(len(params), total))
    for name, expect, note in KEY_PARAMS:
        if name not in params:
            info('{:<16s} (not present)'.format(name))
            continue
        val = params[name]
        sval = '{:g}'.format(val)
        if name.startswith('MODE') and name[4:].isdigit():
            sval += ' ' + ROVER_MODES.get(int(val), '?')
        line = '{:<16s} {:<12s} {}'.format(name, sval, note)
        if expect is None:
            info(line)
        elif abs(val - expect) < 1e-6:
            ok(line)
        else:
            fail(line + '  (expected {})'.format(expect))
    modes = [int(params.get('MODE%d' % i, -1)) for i in range(1, 7)]
    if 15 not in modes:
        fail('GUIDED (15) is not on the mode switch: the Jetson can never drive')
    print('\n[6] Indoor (no GPS) readiness - SLAM pose as external navigation')
    indoor = [('GPS1_TYPE', 0), ('VISO_TYPE', 1), ('EK3_SRC1_POSXY', 6), ('EK3_SRC1_VELXY', 6),
              ('EK3_SRC1_POSZ', 1), ('EK3_SRC1_YAW', 6), ('COMPASS_USE', 0)]
    todo = [(n, params.get(n), want) for n, want in indoor
            if n in params and abs(params[n] - want) > 1e-6]
    if not todo:
        ok('configured for indoor ExtNav (run fc_bridge with vision_pose: true + SLAM)')
    else:
        info('not set up for indoor driving yet (fine outdoors with GPS). To enable it set:')
        for n, have, want in todo:
            info('  {:<16s} {:g} -> {:g}'.format(n, have, want))
        info('  (see README "Indoor driving (no GPS)"; GUIDED is refused indoors until then)')

    if args.save_params:
        with open(args.save_params, 'w') as f:
            for k in sorted(params):
                f.write('{},{:g}\n'.format(k, params[k]))
        info('wrote {} params to {}'.format(len(params), args.save_params))

    print('\nRESULT: {}'.format('ALL GOOD' if not failures else
                                '{} problem(s):\n  - '.format(len(failures)) +
                                '\n  - '.join(failures)))
    return 0 if not failures else 1


if __name__ == '__main__':
    sys.exit(main())
