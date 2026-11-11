#!/usr/bin/env python3
"""Relay the FC's USB serial port to UDP, both directions.

Why: Docker Desktop for Linux runs containers inside a VM, so `devices:` /
`--device=/dev/ttyACM0` can't hand the FC's USB port to the dev container.
Run this on the host instead; the dev container publishes UDP 14550 and the
bridge listens on it:

    # host
    .venv/bin/python scripts/fc_udp_forward.py            # /dev/fc0 or /dev/ttyACM0
    # dev container
    ros2 launch fc_bridge fc_bridge.launch.py connection:=udpin:0.0.0.0:14550

On the Jetson (native Docker Engine) none of this is needed: the container gets
/dev/fc0 directly (see docker/docker-compose.jetson.yml).
"""
import argparse
import os
import sys
import time

from pymavlink import mavutil


def default_serial():
    for path in ('/dev/fc0', '/dev/ttyACM0'):
        if os.path.exists(path):
            return path
    return '/dev/ttyACM0'


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--serial', default=default_serial())
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--udp', default='udpout:127.0.0.1:14550',
                    help='UDP endpoint to forward to (default: %(default)s)')
    args = ap.parse_args()

    fc = mavutil.mavlink_connection(args.serial, baud=args.baud)
    udp = mavutil.mavlink_connection(args.udp)
    print('Forwarding {} <-> {} (Ctrl+C to stop)'.format(args.serial, args.udp), flush=True)

    to_udp = to_fc = 0
    last_report = time.monotonic()
    try:
        while True:
            busy = False
            msg = fc.recv_msg()
            if msg is not None and msg.get_type() != 'BAD_DATA':
                udp.write(msg.get_msgbuf())
                to_udp += 1
                busy = True
            msg = udp.recv_msg()
            if msg is not None and msg.get_type() != 'BAD_DATA':
                fc.write(msg.get_msgbuf())
                to_fc += 1
                busy = True
            if time.monotonic() - last_report > 5.0:
                print('  FC->UDP {:6d} msgs   UDP->FC {:6d} msgs'.format(to_udp, to_fc),
                      flush=True)
                last_report = time.monotonic()
            if not busy:
                time.sleep(0.001)
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    sys.exit(main())
