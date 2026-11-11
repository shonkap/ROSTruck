"""Stand-in for the RC transmitter in simulation: change mode, arm, disarm.

fc_bridge deliberately never arms or changes mode (that stays with the pilot
on the real truck), so in SITL this does the pilot's part over SITL's spare
MAVLink port (SERIAL2 = tcp:5763):

    ros2 run truck_sim sim_ctl guided arm     # hand control to ROS (cmd_vel)
    ros2 run truck_sim sim_ctl hold           # stop
    ros2 run truck_sim sim_ctl manual disarm
    ros2 run truck_sim sim_ctl status
    ros2 run truck_sim sim_ctl SERVO1_TRIM=1560   # set a param (e.g. inject a fault)
"""
import argparse
import sys
import time

from pymavlink import mavutil

MAV = mavutil.mavlink
MODES = {'manual': 0, 'acro': 1, 'hold': 4, 'auto': 10, 'rtl': 11, 'guided': 15}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('actions', nargs='+',
                    help='{} | arm | disarm | status | NAME=VALUE'.format(' | '.join(sorted(MODES))))
    ap.add_argument('--conn', default='tcp:127.0.0.1:5763')
    ap.add_argument('--timeout', type=float, default=60.0,
                    help='how long to wait for SITL/the EKF to be ready')
    args = ap.parse_args(argv)

    m = mavutil.mavlink_connection(args.conn, source_system=254)
    hb = m.wait_heartbeat(timeout=args.timeout)
    if hb is None:
        print('no heartbeat on ' + args.conn)
        return 1

    def status():
        # SITL routes every MAVLink peer's heartbeat to every port (e.g.
        # fc_bridge's), so only trust the autopilot's own.
        end = time.time() + 3
        while time.time() < end:
            hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb is not None and hb.get_srcComponent() == MAV.MAV_COMP_ID_AUTOPILOT1 \
                    and hb.autopilot != MAV.MAV_AUTOPILOT_INVALID:
                return (mavutil.mode_string_v10(hb),
                        bool(hb.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED))
        return '?', None

    for action in args.actions:
        if '=' in action:
            name, value = action.split('=', 1)
            m.mav.param_set_send(m.target_system, m.target_component, name.encode('ascii'),
                                 float(value), MAV.MAV_PARAM_TYPE_REAL32)
            pv = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
            print('{} = {}'.format(name, pv.param_value if pv else 'no reply'))
            continue
        if action not in MODES and action not in ('arm', 'disarm', 'status'):
            print('unknown action ' + action)
            return 2
        deadline = time.time() + args.timeout
        while True:
            if action in MODES:
                m.mav.command_long_send(m.target_system, m.target_component,
                                        MAV.MAV_CMD_DO_SET_MODE, 0,
                                        MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                        MODES[action], 0, 0, 0, 0, 0)
            elif action in ('arm', 'disarm'):
                m.mav.command_long_send(m.target_system, m.target_component,
                                        MAV.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                        1 if action == 'arm' else 0, 0, 0, 0, 0, 0, 0)
            else:
                break
            ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
            if ack is not None and ack.result == MAV.MAV_RESULT_ACCEPTED:
                print('{}: accepted'.format(action))
                break
            # Not ready yet (EKF still converging, prearm checks): show why and retry.
            while True:
                txt = m.recv_match(type='STATUSTEXT', blocking=False)
                if txt is None:
                    break
                print('  FC: ' + txt.text)
            if time.time() > deadline:
                print('{}: refused ({})'.format(
                    action, 'no ack' if ack is None else 'result {}'.format(ack.result)))
                return 1
            time.sleep(2.0)
    mode, armed = status()
    print('mode={} armed={}'.format(mode, armed))
    return 0


if __name__ == '__main__':
    sys.exit(main())
