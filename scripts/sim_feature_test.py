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
import math
import os
import shutil
import sys
import time

from simtest import REPO, SimTest, api, arm, run_task, state, stop_sim, wait_for

MAP = 'feature_test'


def known_area():
    m = state().get('map') or {}
    return m.get('width', 0) * m.get('height', 0) * m.get('resolution', 0) ** 2


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--indoor', action='store_true')
    args = ap.parse_args()
    shutil.rmtree(os.path.join(REPO, 'maps', MAP), ignore_errors=True)
    t = SimTest('FEATURE TEST', '/tmp/feature_launch.log')
    try:
        print('session A: mapping', flush=True)
        t.check('A launched', t.launch(args.indoor, ['map_name:=' + MAP]))
        t.check('A armed', arm())
        h = run_task({'type': 'map_room', 'max_minutes': 7}, 600)
        t.check('A map_room', h and h['state'] == 'succeeded', h and h['message'])
        t.check('A map files saved', os.path.exists(os.path.join(REPO, 'maps', MAP, 'map.png')))
        api('/api/place', {'name': 'Here Spot', 'here': True})
        api('/api/place', {'name': 'kitchen door', 'x': 0.6, 'y': 0.4})
        pl = wait_for(lambda: len(state().get('places') or []) >= 2 and state()['places'], 15)
        t.check('A places saved (here + coordinates)', pl and len(pl) == 2, [p['name'] for p in pl or []])
        h = run_task({'type': 'goto', 'waypoints': ['kitchen door']}, 240)
        pose = state()['pose']
        d = math.hypot(pose[0] - 0.6, pose[1] - 0.4) if pose else 99
        t.check('A goto by place name', h and h['state'] == 'succeeded' and d < 0.4, '{:.2f} m from it'.format(d))
        h = run_task({'type': 'goto', 'waypoints': ['garage']}, 30)
        t.check('A unknown place rejected cleanly', h and h['state'] == 'failed' and 'unknown place' in h['message'],
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
        t.check('A teleop drives (hold 3 s)', moved > 0.2, '{:.2f} m'.format(moved))
        time.sleep(2.0)
        p2 = state()['pose']
        t.check('A teleop stops when released', math.hypot(p2[0] - p1[0], p2[1] - p1[1]) < 0.1)
        api('/api/task', {'type': 'goto', 'waypoints': ['Here Spot']})
        time.sleep(3)
        code, body = api('/api/teleop', {'v': 0.3, 'w': 0.0})
        t.check('A teleop refused while a task runs', code == 409, body.get('error', ''))
        wait_for(lambda: not state()['tasks'].get('active'), 240)
        t.check('A no crashes', not t.crashed())

        print('session B: relocalize in the saved map', flush=True)
        t.check('B launched (localization)', t.launch(args.indoor, ['map_name:=' + MAP, 'localization:=true']))
        area = wait_for(lambda: known_area() > 20 and known_area(), 60)
        t.check('B saved map loaded at start', area, '{:.0f} m^2 map extent'.format(area or 0))
        pl = wait_for(lambda: state().get('places'), 15)
        t.check('B places loaded', pl and len(pl) == 2)
        pose = wait_for(lambda: state().get('pose'), 30)
        d0 = math.hypot(pose[0], pose[1]) if pose else 99
        t.check('B starts at the map origin (start_at:=origin)', d0 < 0.3, '{:.2f} m'.format(d0))
        t.check('B armed', arm())
        h = run_task({'type': 'goto', 'waypoints': ['kitchen door', 'Here Spot']}, 300)
        t.check('B goto saved places without exploring', h and h['state'] == 'succeeded', h and h['message'])
        t.check('B no crashes', not t.crashed())
    finally:
        stop_sim()
    return t.summary()


if __name__ == '__main__':
    sys.exit(main())
