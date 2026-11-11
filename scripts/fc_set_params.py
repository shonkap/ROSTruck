#!/usr/bin/env python3
"""Change FC parameters safely: full backup first, set, read back, log.

    .venv/bin/python scripts/fc_set_params.py --profile safety          # arming + mode switch
    .venv/bin/python scripts/fc_set_params.py --profile indoor          # no-GPS ExtNav (see README)
    .venv/bin/python scripts/fc_set_params.py WP_SPEED=1.5 CRUISE_SPEED=1.5
    .venv/bin/python scripts/fc_set_params.py --restore backups/fc_params_2026-11-07_2301.parm
    .venv/bin/python scripts/fc_set_params.py --profile safety --dry-run

Every run writes backups/fc_params_<timestamp>.parm (ALL params, before any
change) and appends old -> new to backups/fc_param_changes.log. Python 3.6 ok.
"""
import argparse
import os
import sys
import time

from pymavlink import mavutil

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BACKUPS = os.path.join(REPO, 'backups')

PROFILES = {
    # Deliberate arming + a 3-position mode switch on channel 8:
    # low (<1360) MANUAL, middle (1361-1620) HOLD, high (>1620) GUIDED (Jetson drives).
    'safety': [('ARMING_REQUIRE', 1), ('MODE_CH', 8),
               ('MODE1', 0), ('MODE2', 0), ('MODE3', 4), ('MODE4', 4), ('MODE5', 15), ('MODE6', 15)],
    # Indoor: EKF navigates on the SLAM pose from fc_bridge (vision_pose: true). Reboot after.
    'indoor': [('GPS1_TYPE', 0), ('VISO_TYPE', 1), ('VISO_DELAY_MS', 50),
               ('EK3_SRC1_POSXY', 6), ('EK3_SRC1_VELXY', 6), ('EK3_SRC1_POSZ', 1),
               ('EK3_SRC1_VELZ', 0), ('EK3_SRC1_YAW', 6), ('COMPASS_USE', 0)],
}
REBOOT_NEEDED = {'GPS1_TYPE', 'VISO_TYPE', 'EK3_SRC1_POSXY', 'EK3_SRC1_VELXY', 'EK3_SRC1_POSZ',
                 'EK3_SRC1_VELZ', 'EK3_SRC1_YAW', 'COMPASS_USE'}


def default_conn():
    for path in ('/dev/fc0', '/dev/ttyACM0'):
        if os.path.exists(path):
            return path
    return '/dev/ttyACM0'


def name_of(msg):
    n = msg.param_id
    if isinstance(n, bytes):
        n = n.decode('ascii', 'ignore')
    return n.rstrip('\x00')


def fetch_all(m, timeout=40.0):
    m.mav.param_request_list_send(m.target_system, m.target_component)
    params, total, last = {}, None, time.time()
    end = time.time() + timeout
    while time.time() < end and time.time() - last < 3.0:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg is None:
            continue
        last = time.time()
        params[name_of(msg)] = (msg.param_value, msg.param_type)
        total = msg.param_count
        if total and len(params) >= total:
            break
    return params, total


def set_param(m, name, value, ptype, tries=3):
    for _ in range(tries):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode('ascii'),
                             float(value), ptype)
        end = time.time() + 3.0
        while time.time() < end:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if msg is not None and name_of(msg) == name:
                if abs(msg.param_value - float(value)) < 1e-4:
                    return msg.param_value
                break
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('changes', nargs='*', help='NAME=VALUE ...')
    ap.add_argument('--profile', choices=sorted(PROFILES))
    ap.add_argument('--restore', metavar='PARM_FILE', help='set every param that differs from this backup')
    ap.add_argument('--conn', default=default_conn())
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    wanted = []
    if args.profile:
        wanted += PROFILES[args.profile]
    for c in args.changes:
        n, v = c.split('=', 1)
        wanted.append((n.strip().upper(), float(v)))
    if args.restore:
        with open(args.restore) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    n, v = line.replace('\t', ',').replace(' ', ',').split(',')[:2]
                    wanted.append((n, float(v)))
    if not wanted:
        ap.error('nothing to change (give NAME=VALUE, --profile or --restore)')

    m = mavutil.mavlink_connection(args.conn, baud=115200, source_system=255)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        print('no heartbeat on ' + args.conn)
        return 1
    m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
    print('connected to system {} on {}; downloading all params for the backup...'.format(
        m.target_system, args.conn))
    params, total = fetch_all(m)
    if not total or len(params) < total:
        print('only got {}/{} params - refusing to change anything without a full backup'.format(
            len(params), total))
        return 1
    os.makedirs(BACKUPS, exist_ok=True)
    stamp = time.strftime('%Y-%m-%d_%H%M%S')
    backup = os.path.join(BACKUPS, 'fc_params_{}.parm'.format(stamp))
    with open(backup, 'w') as f:
        f.write('# full FC parameter backup before fc_set_params.py, {}\n'.format(stamp))
        for k in sorted(params):
            f.write('{},{:g}\n'.format(k, params[k][0]))
    print('backup: {} ({} params)'.format(os.path.relpath(backup, REPO), len(params)))

    todo = []
    for n, v in wanted:
        if n not in params:
            print('  SKIP  {} - no such parameter on this FC'.format(n))
            continue
        if abs(params[n][0] - v) < 1e-4:
            continue
        todo.append((n, params[n][0], v, params[n][1]))
    if not todo:
        print('nothing to do - already set.')
        return 0
    for n, old, new, _ in todo:
        print('  {:<16s} {:g} -> {:g}'.format(n, old, new))
    if args.dry_run:
        print('dry run: nothing written.')
        return 0

    failed, reboot = [], False
    with open(os.path.join(BACKUPS, 'fc_param_changes.log'), 'a') as log:
        for n, old, new, ptype in todo:
            got = set_param(m, n, new, ptype)
            ok = got is not None
            log.write('{} {} {:g} -> {:g} {}\n'.format(stamp, n, old, new, 'ok' if ok else 'FAILED'))
            print('  {}  {} = {:g}'.format('OK  ' if ok else 'FAIL', n, got if ok else old))
            if not ok:
                failed.append(n)
            reboot |= n in REBOOT_NEEDED
    if reboot:
        print('\nReboot the FC for these to take effect (power cycle, or MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN).')
    print('undo with: scripts/fc_set_params.py --restore {}'.format(os.path.relpath(backup, REPO)))
    return 0 if not failed else 1


if __name__ == '__main__':
    sys.exit(main())
