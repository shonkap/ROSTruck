#!/usr/bin/env python3
"""End-to-end regression test in the simulator. Run INSIDE the sim container:

    python3 scripts/sim_smoke_test.py                 # GPS sim
    python3 scripts/sim_smoke_test.py --indoor        # no GPS: FC navigates on SLAM
    python3 scripts/sim_smoke_test.py --keep          # leave the sim running afterwards

Launches the full stack headless, plays pilot (GUIDED + arm), then through
the web API (same as the buttons): Map room -> must map most of the room and
finish; goto home -> must arrive. Prints PASS/FAIL per check, exit code 0 if
all passed. Takes ~10 minutes.
"""
import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB = 'http://127.0.0.1:8080'
LOG = '/tmp/smoke_launch.log'
results = []


def check(name, ok, detail=''):
    results.append((name, ok))
    print('  {}  {}{}'.format('PASS' if ok else 'FAIL', name, ' - ' + detail if detail else ''), flush=True)
    return ok


def api(path, body=None, timeout=5):
    req = urllib.request.Request(WEB + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def wait_for(fn, timeout, period=2.0):
    end = time.time() + timeout
    while time.time() < end:
        try:
            v = fn()
            if v:
                return v
        except Exception:
            pass
        time.sleep(period)
    return None


def log_has(text):
    try:
        with open(LOG, 'rb') as f:
            return text.encode() in f.read()
    except OSError:
        return False


def run_task(task, timeout):
    """Submit via the web API, wait until it's finished; returns the history entry."""
    before = {h['id'] for h in api('/api/state')['tasks'].get('history', [])}
    api('/api/task', task)

    def done():
        st = api('/api/state')['tasks']
        for h in st.get('history', []):
            if h['id'] not in before and h['type'] == task['type']:
                return h
        return None
    return wait_for(done, timeout, 5.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--indoor', action='store_true')
    ap.add_argument('--keep', action='store_true')
    ap.add_argument('--map-minutes', type=float, default=7.0)
    ap.add_argument('--min-area', type=float, default=24.0, help='m^2 that must get mapped (room = 30)')
    args = ap.parse_args()

    stop = os.path.join(REPO, 'sim', 'stop_sim.sh')
    subprocess.call([stop], stdout=subprocess.DEVNULL)
    print('launching sim ({})...'.format('indoor, no GPS' if args.indoor else 'GPS'), flush=True)
    with open(LOG, 'w') as log:
        subprocess.Popen(['ros2', 'launch', 'truck_sim', 'sim.launch.py', 'gui:=false', 'rviz:=false',
                          'indoor:={}'.format(str(args.indoor).lower())],
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    t0 = time.time()
    try:
        ready = 'is using external nav' if args.indoor else 'origin set'
        check('FC EKF ready', wait_for(lambda: log_has(ready), 180), ready)
        check('web page up', wait_for(lambda: api('/api/state'), 60))
        mods = wait_for(lambda: set(api('/api/state')['tasks'].get('modules', {})) >=
                        {'navigator', 'explorer'}, 60)
        check('task modules registered', mods)

        armed = False
        for _ in range(4):
            out = subprocess.run(['ros2', 'run', 'truck_sim', 'sim_ctl', 'guided', 'arm'],
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=90).stdout.decode()
            if 'armed=True' in out:
                armed = True
                break
            time.sleep(5)
        if not check('GUIDED + armed', armed):
            return 1

        h = run_task({'type': 'map_room', 'max_minutes': args.map_minutes}, args.map_minutes * 60 + 120)
        area = (h or {}).get('progress') or 0
        if h and not area:
            try:
                area = float(h['message'].split(' - ')[1].split()[0])
            except (IndexError, ValueError):
                area = 0
        check('map_room finished', h is not None and h['state'] == 'succeeded', h and h['message'])
        check('mapped >= {:.0f} m^2'.format(args.min_area), area >= args.min_area, '{} m^2'.format(area))

        h = run_task({'type': 'goto', 'waypoints': [[0.0, 0.0]]}, 240)
        check('goto home arrived', h is not None and h['state'] == 'succeeded', h and h['message'])
        st = api('/api/state')
        pose = st.get('pose') or [99, 99]
        dist = (pose[0] ** 2 + pose[1] ** 2) ** 0.5
        check('ends within 0.35 m of home', dist < 0.35, '{:.2f} m'.format(dist))
        crashed = log_has('process has died') or log_has('navigator error') or log_has('explorer error')
        check('no node crashed / internal errors', not crashed)
    finally:
        if not args.keep:
            subprocess.call([stop], stdout=subprocess.DEVNULL)
    failed = [n for n, ok in results if not ok]
    print('\nSMOKE TEST {} ({} checks, {:.0f} min){}'.format(
        'PASSED' if not failed else 'FAILED', len(results), (time.time() - t0) / 60,
        '' if not failed else ': ' + ', '.join(failed)))
    return 0 if not failed else 1


if __name__ == '__main__':
    sys.exit(main())
