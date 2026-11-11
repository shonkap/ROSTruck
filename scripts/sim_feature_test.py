#!/usr/bin/env python3
"""Feature test in the simulator (run INSIDE the sim container, ~10 min):

  session A (mapping, map_name=feature_test):
    map_room -> map saved; save places by position and by coordinates;
    goto a place BY NAME; manual drive (/api/teleop) moves the truck and is
    refused while a task runs; unknown place names fail cleanly
  session B (new launch, localization:=true, same map):
    map + places loaded at start; truck relocalizes in the old map;
    goto a saved place by name without exploring first

    python3 scripts/sim_feature_test.py [--indoor]
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB = 'http://127.0.0.1:8080'
LOG = '/tmp/feature_launch.log'
MAP = 'feature_test'
results = []


def check(name, ok, detail=''):
    results.append((name, bool(ok)))
    print('  {}  {}{}'.format('PASS' if ok else 'FAIL', name, ' - ' + str(detail) if detail != '' else ''),
          flush=True)
    return ok


def api(path, body=None, timeout=5):
    req = urllib.request.Request(WEB + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode() or '{}')


def state():
    return api('/api/state')[1]


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
    before = {h['id'] for h in state()['tasks'].get('history', [])}
    api('/api/task', task)

    def done():
        for h in state()['tasks'].get('history', []):
            if h['id'] not in before and h['type'] == task['type']:
                return h
    return wait_for(done, timeout, 4.0)


def launch(extra, indoor):
    subprocess.call([os.path.join(REPO, 'sim', 'stop_sim.sh')], stdout=subprocess.DEVNULL)
    with open(LOG, 'w') as log:
        subprocess.Popen(['ros2', 'launch', 'truck_sim', 'sim.launch.py', 'gui:=false', 'rviz:=false',
                          'indoor:={}'.format(str(indoor).lower()), 'map_name:=' + MAP] + extra,
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    ready = 'is using external nav' if indoor else 'origin set'
    ok = wait_for(lambda: log_has(ready), 180) and wait_for(state, 60)
    wait_for(lambda: {'navigator', 'explorer'} <= set(state()['tasks'].get('modules', {})), 60)
    return ok


def arm():
    for _ in range(4):
        out = subprocess.run(['ros2', 'run', 'truck_sim', 'sim_ctl', 'guided', 'arm'], stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, timeout=90).stdout.decode()
        if 'armed=True' in out:
            return True
        time.sleep(5)
    return False


def known_area():
    m = state().get('map') or {}
    return m.get('width', 0) * m.get('height', 0) * m.get('resolution', 0) ** 2


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--indoor', action='store_true')
    args = ap.parse_args()
    shutil.rmtree(os.path.join(REPO, 'maps', MAP), ignore_errors=True)
    t0 = time.time()
    try:
        print('session A: mapping', flush=True)
        check('A launched', launch([], args.indoor))
        check('A armed', arm())
        h = run_task({'type': 'map_room', 'max_minutes': 7}, 600)
        check('A map_room', h and h['state'] == 'succeeded', h and h['message'])
        check('A map files saved', os.path.exists(os.path.join(REPO, 'maps', MAP, 'map.png')))
        api('/api/place', {'name': 'Here Spot', 'here': True})
        api('/api/place', {'name': 'kitchen door', 'x': 0.6, 'y': 0.4})
        pl = wait_for(lambda: len(state().get('places') or []) >= 2 and state()['places'], 15)
        check('A places saved (here + coordinates)', pl and len(pl) == 2, [p['name'] for p in pl or []])
        h = run_task({'type': 'goto', 'waypoints': ['kitchen door']}, 240)
        pose = state()['pose']
        d = math.hypot(pose[0] - 0.6, pose[1] - 0.4) if pose else 99
        check('A goto by place name', h and h['state'] == 'succeeded' and d < 0.4, '{:.2f} m from it'.format(d))
        h = run_task({'type': 'goto', 'waypoints': ['garage']}, 30)
        check('A unknown place rejected cleanly', h and h['state'] == 'failed' and 'unknown place' in h['message'],
              h and h['message'])
        # manual drive: moves when idle
        p0 = state()['pose']
        end = time.time() + 3.0
        while time.time() < end:
            api('/api/teleop', {'v': 0.3, 'w': 0.0})
            time.sleep(0.1)
        time.sleep(1.5)
        p1 = state()['pose']
        moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        check('A teleop drives (hold 3 s)', moved > 0.2, '{:.2f} m'.format(moved))
        time.sleep(2.0)
        p2 = state()['pose']
        check('A teleop stops when released', math.hypot(p2[0] - p1[0], p2[1] - p1[1]) < 0.1)
        api('/api/task', {'type': 'goto', 'waypoints': ['Here Spot']})
        time.sleep(3)
        code, body = api('/api/teleop', {'v': 0.3, 'w': 0.0})
        check('A teleop refused while a task runs', code == 409, body.get('error', ''))
        wait_for(lambda: not state()['tasks'].get('active'), 240)
        crashed = log_has('process has died') or log_has('navigator error') or log_has('explorer error')
        check('A no crashes', not crashed)

        print('session B: relocalize in the saved map', flush=True)
        check('B launched (localization)', launch(['localization:=true'], args.indoor))
        area = wait_for(lambda: known_area() > 20 and known_area(), 60)
        check('B saved map loaded at start', area, '{:.0f} m^2 map extent'.format(area or 0))
        pl = wait_for(lambda: state().get('places'), 15)
        check('B places loaded', pl and len(pl) == 2)
        pose = wait_for(lambda: state().get('pose'), 30)
        d0 = math.hypot(pose[0], pose[1]) if pose else 99
        check('B starts at the map origin (start_at:=origin)', d0 < 0.3, '{:.2f} m'.format(d0))
        check('B armed', arm())
        h = run_task({'type': 'goto', 'waypoints': ['kitchen door', 'Here Spot']}, 300)
        check('B goto saved places without exploring', h and h['state'] == 'succeeded', h and h['message'])
        crashed = log_has('process has died') or log_has('navigator error')
        check('B no crashes', not crashed)
    finally:
        subprocess.call([os.path.join(REPO, 'sim', 'stop_sim.sh')], stdout=subprocess.DEVNULL)
    failed = [n for n, ok in results if not ok]
    print('\nFEATURE TEST {} ({} checks, {:.0f} min){}'.format(
        'PASSED' if not failed else 'FAILED', len(results), (time.time() - t0) / 60,
        '' if not failed else ': ' + ', '.join(failed)))
    return 0 if not failed else 1


if __name__ == '__main__':
    sys.exit(main())
