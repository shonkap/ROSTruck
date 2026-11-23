"""MAVLink helpers shared by fc_check.py and fc_set_params.py."""
import os
import time

from pymavlink import mavutil

MAV = mavutil.mavlink


def default_conn():
    for path in ('/dev/fc0', '/dev/ttyACM0'):
        if os.path.exists(path):
            return path
    return '/dev/ttyACM0'


def wait_autopilot(m, timeout=10.0):
    """The autopilot's own heartbeat (not a GCS's / fc_bridge's routed through
    it); also points m.target_* at it. None on timeout."""
    end = time.time() + timeout
    while time.time() < end:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if hb is not None and hb.autopilot != MAV.MAV_AUTOPILOT_INVALID \
                and hb.get_srcComponent() == MAV.MAV_COMP_ID_AUTOPILOT1:
            m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
            return hb
    return None


def param_name(msg):
    n = msg.param_id
    if isinstance(n, bytes):
        n = n.decode('ascii', 'ignore')
    return n.rstrip('\x00')


def fetch_params(m, timeout=40.0):
    """All params: ({name: (value, type)}, count the FC reported)."""
    m.mav.param_request_list_send(m.target_system, m.target_component)
    params, total, last = {}, None, time.time()
    end = time.time() + timeout
    while time.time() < end and time.time() - last < 3.0:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg is None:
            continue
        last = time.time()
        params[param_name(msg)] = (msg.param_value, msg.param_type)
        total = msg.param_count
        if total and len(params) >= total:
            break
    return params, total


def set_param(m, name, value, ptype, tries=3):
    """Set and read back; returns the FC's value, or None if it didn't take."""
    for _ in range(tries):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode('ascii'), float(value), ptype)
        end = time.time() + 3.0
        while time.time() < end:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if msg is not None and param_name(msg) == name:
                if abs(msg.param_value - float(value)) < 1e-4:
                    return msg.param_value
                break
    return None


def write_parm(path, params, header=None):
    """Mission Planner style NAME,VALUE file."""
    with open(path, 'w') as f:
        if header:
            f.write('# {}\n'.format(header))
        for k in sorted(params):
            v = params[k]
            f.write('{},{:g}\n'.format(k, v[0] if isinstance(v, tuple) else v))
