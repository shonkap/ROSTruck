"""Task queue / arbitration, ROS-free. Python 3.6.

A task is a JSON-able dict:
    {"type": "goto", "waypoints": [[x, y], ...]}      drive to map points in order
    {"type": "map_room", "max_minutes": 10}           explore until the map stops growing
    {"type": "stop"}                                  cancel everything now
    {"type": "request", "text": "..."}                natural-language request for
                                                      planner plugins (e.g. an LLM)
plus optional "priority" (0-10, default 5; higher preempts), "source"
("web", "llm", "detector", ...) and "note".

Who executes what is NOT hard-coded: executor modules announce the task
types they handle (see task_manager.py), so a new plugin can add a type.
"""
import itertools
import time

DEFAULT_PRIORITY = 5
TERMINAL = ('succeeded', 'failed', 'canceled')
# handled by the manager itself
BUILTIN = ('stop', 'request')


class TaskBoard(object):
    def __init__(self, ack_timeout=5.0, history=50, clock=time.time):
        self._ids = itertools.count(1)
        self.queue = []
        self.active = None
        self.history = []
        self._history_len = history
        self._ack_timeout = ack_timeout
        self._clock = clock
        self.modules = {}          # name -> {"handles": [...], "seen": t}

    # ----------------------------------------------------------- modules
    def register(self, name, handles):
        self.modules[name] = {'handles': list(handles), 'seen': self._clock()}

    def handlers(self, task_type, max_age=10.0):
        now = self._clock()
        return [n for n, m in self.modules.items()
                if task_type in m['handles'] and now - m['seen'] < max_age]

    def known_types(self):
        types = set(BUILTIN)
        for m in self.modules.values():
            types.update(m['handles'])
        return sorted(types)

    # ------------------------------------------------------------- tasks
    def submit(self, task):
        """Returns (task_or_None, events). events: list of (kind, task) the
        caller turns into ROS messages: 'dispatch', 'cancel', 'request', 'rejected'."""
        events = []
        if not isinstance(task, dict) or 'type' not in task:
            return None, [('rejected', {'error': 'task must be a JSON object with "type"'})]
        t = dict(task)
        t['id'] = next(self._ids)
        t.setdefault('priority', DEFAULT_PRIORITY)
        t.setdefault('source', 'unknown')
        t['created'] = self._clock()
        t['state'] = 'queued'
        t['message'] = ''
        ttype = t['type']

        if ttype == 'stop':
            events += self.cancel_all('stop requested by ' + t['source'])
            self._finish(t, 'succeeded', 'all tasks stopped')
            return t, events
        if ttype == 'request':
            # Not executed here: published for planner plugins (LLM) to handle.
            self._finish(t, 'succeeded', 'forwarded to planner plugins')
            return t, [('request', t)]
        if not self.handlers(ttype):
            self._finish(t, 'failed', 'no module handles "{}" (known: {})'.format(
                ttype, ', '.join(self.known_types())))
            return t, [('rejected', t)]

        if self.active is not None and t['priority'] > self.active['priority']:
            pre = self.active
            pre['state'] = 'queued'
            pre['message'] = 'preempted by task {}'.format(t['id'])
            events.append(('cancel', pre))
            self.queue.insert(0, pre)
            self.active = None
        self.queue.append(t)
        # stable sort: higher priority first, FIFO within a priority
        self.queue.sort(key=lambda x: -x['priority'])
        events += self._start_next()
        return t, events

    def cancel(self, task_id):
        if self.active is not None and self.active['id'] == task_id:
            t = self.active
            self.active = None
            self._finish(t, 'canceled', 'canceled')
            return [('cancel', t)] + self._start_next()
        for t in list(self.queue):
            if t['id'] == task_id:
                self.queue.remove(t)
                self._finish(t, 'canceled', 'canceled')
        return []

    def cancel_all(self, reason='canceled'):
        events = []
        if self.active is not None:
            events.append(('cancel', self.active))
            self._finish(self.active, 'canceled', reason)
            self.active = None
        for t in self.queue:
            self._finish(t, 'canceled', reason)
        self.queue = []
        return events

    def feedback(self, task_id, state, message='', progress=None):
        """Executor reports on the active task. Returns events."""
        t = self.active
        if t is None or t['id'] != task_id:
            return []
        if state == 'running':
            t['state'] = 'running'
            t['acked'] = True
        t['message'] = message or t['message']
        if progress is not None:
            t['progress'] = progress
        if state in TERMINAL:
            self.active = None
            self._finish(t, state, message)
            return self._start_next()
        return []

    def tick(self):
        """Call periodically: fails a dispatched task nobody picked up."""
        t = self.active
        if t is not None and not t.get('acked') and \
                self._clock() - t.get('dispatched', self._clock()) > self._ack_timeout:
            self.active = None
            self._finish(t, 'failed', 'no executor responded')
            return self._start_next()
        return []

    def _start_next(self):
        if self.active is not None or not self.queue:
            return []
        t = self.queue.pop(0)
        t['state'] = 'dispatched'
        t['dispatched'] = self._clock()
        t['acked'] = False
        t['handler'] = (self.handlers(t['type']) or ['?'])[0]
        self.active = t
        return [('dispatch', t)]

    def _finish(self, t, state, message):
        t['state'] = state
        t['message'] = message
        t['finished'] = self._clock()
        self.history.insert(0, t)
        del self.history[self._history_len:]

    def status(self):
        return {'active': self.active, 'queue': self.queue, 'history': self.history[:15],
                'modules': {n: m['handles'] for n, m in self.modules.items()},
                'types': self.known_types()}
