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
import sys

from simtest import SimTest, arm, run_task, state, stop_sim


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--indoor', action='store_true')
    ap.add_argument('--keep', action='store_true')
    ap.add_argument('--map-minutes', type=float, default=7.0)
    ap.add_argument('--min-area', type=float, default=24.0, help='m^2 that must get mapped (room = 30)')
    args = ap.parse_args()

    t = SimTest('SMOKE TEST', '/tmp/smoke_launch.log')
    try:
        t.launch(args.indoor)
        if not t.check('GUIDED + armed', arm()):
            return t.summary()
        h = run_task({'type': 'map_room', 'max_minutes': args.map_minutes}, args.map_minutes * 60 + 120)
        area = (h or {}).get('progress') or 0
        t.check('map_room finished', h is not None and h['state'] == 'succeeded', h and h['message'])
        t.check('mapped >= {:.0f} m^2'.format(args.min_area), area >= args.min_area, '{} m^2'.format(area))

        h = run_task({'type': 'goto', 'waypoints': [[0.0, 0.0]]}, 240)
        t.check('goto home arrived', h is not None and h['state'] == 'succeeded', h and h['message'])
        pose = state().get('pose') or [99, 99]
        dist = (pose[0] ** 2 + pose[1] ** 2) ** 0.5
        t.check('ends within 0.35 m of home', dist < 0.35, '{:.2f} m'.format(dist))
        t.check('no node crashed / internal errors', not t.crashed())
    finally:
        if not args.keep:
            stop_sim()
    return t.summary()


if __name__ == '__main__':
    sys.exit(main())
