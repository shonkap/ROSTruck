"""Helpers for the end-to-end sim tests (sim_smoke_test.py, sim_feature_test.py).

They drive the simulator like a user would: launch it headless, play pilot
with sim_ctl, then use the web API (the same calls the page's buttons make).
"""
import json
import os
import subprocess
import time
import urllib.error
import urllib.request

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB = 'http://127.0.0.1:8080'
STOP = os.path.join(REPO, 'sim', 'stop_sim.sh')


def wait_for(fn, timeout, period=2.0):
    """Poll fn() until it returns something truthy (exceptions = not yet)."""
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


def api(path, body=None, timeout=5):
    """(HTTP status, JSON body) of a web API call."""
    req = urllib.request.Request(WEB + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode() or '{}')


def state():
    return api('/api/state')[1]


def run_task(task, timeout):
    """Submit a task, wait until it's finished; returns its history entry (or None)."""
    before = {h['id'] for h in state()['tasks'].get('history', [])}
    api('/api/task', task)

    def done():
        for h in state()['tasks'].get('history', []):
            if h['id'] not in before and h['type'] == task['type']:
                return h
    return wait_for(done, timeout, 4.0)


def stop_sim():
    subprocess.call([STOP], stdout=subprocess.DEVNULL)


def arm():
    """GUIDED + arm through sim_ctl (the pilot's job on the real truck)."""
    for _ in range(4):
        out = subprocess.run(['ros2', 'run', 'truck_sim', 'sim_ctl', 'guided', 'arm'], stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, timeout=90).stdout.decode()
        if 'armed=True' in out:
            return True
        time.sleep(5)
    return False


class SimTest(object):
    """Check bookkeeping + one launched sim whose output goes to `log`."""

    def __init__(self, title, log):
        self.title, self._log_base, self.log = title, log, log
        self._launches = 0
        self.results = []
        self.t0 = time.time()

    def check(self, name, ok, detail=''):
        self.results.append((name, bool(ok)))
        print('  {}  {}{}'.format('PASS' if ok else 'FAIL', name, ' - ' + str(detail) if detail != '' else ''),
              flush=True)
        return ok

    def log_has(self, text):
        try:
            with open(self.log, 'rb') as f:
                return text.encode() in f.read()
        except OSError:
            return False

    def crashed(self):
        return any(self.log_has(t) for t in ('process has died', 'navigator error', 'explorer error'))

    def launch(self, indoor, extra=()):
        """(Re)start the sim headless; True once the FC's EKF and the web page are up."""
        # Anything else driving the truck (a teleop left open in another terminal)
        # silently fights the navigator and makes every result meaningless.
        stray = subprocess.run(['pgrep', '-af', 'truck_keys|teleop_twist|explore_room|turn_test'],
                               stdout=subprocess.PIPE).stdout.decode().strip()
        stray = [l for l in stray.splitlines() if 'pgrep' not in l]
        if not self.check('nothing else publishing cmd_vel', not stray, '; '.join(stray)):
            return False
        stop_sim()
        self._launches += 1         # one log per launch, so a later session can't hide an earlier failure
        self.log = self._log_base.replace('.log', '_{}.log'.format(self._launches))
        print('launching sim ({}), log {}'.format('indoor, no GPS' if indoor else 'GPS', self.log), flush=True)
        with open(self.log, 'w') as log:
            subprocess.Popen(['ros2', 'launch', 'truck_sim', 'sim.launch.py', 'gui:=false', 'rviz:=false',
                              'indoor:={}'.format(str(indoor).lower())] + list(extra),
                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        ready = 'is using external nav' if indoor else 'origin set'
        ok = self.check('FC EKF ready', wait_for(lambda: self.log_has(ready), 180), ready)
        ok = self.check('web page up', wait_for(state, 60)) and ok
        mods = wait_for(lambda: {'navigator', 'explorer'} <= set(state()['tasks'].get('modules', {})), 60)
        return self.check('task modules registered', mods) and ok

    def summary(self):
        failed = [n for n, ok in self.results if not ok]
        print('\n{} {} ({} checks, {:.0f} min){}'.format(
            self.title, 'PASSED' if not failed else 'FAILED', len(self.results), (time.time() - self.t0) / 60,
            '' if not failed else ': ' + ', '.join(failed)))
        return 0 if not failed else 1
