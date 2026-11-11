from truck_tasks.task_core import TaskBoard


class Clock(object):
    t = 0.0

    def __call__(self):
        return self.t


def board():
    c = Clock()
    b = TaskBoard(ack_timeout=5.0, clock=c)
    b.register('navigator', ['goto'])
    b.register('explorer', ['map_room'])
    return b, c


def kinds(events):
    return [k for k, _ in events]


def test_dispatch_and_complete_in_order():
    b, _ = board()
    t1, ev = b.submit({'type': 'goto', 'waypoints': [[1, 1]]})
    assert kinds(ev) == ['dispatch'] and b.active['id'] == t1['id']
    t2, ev = b.submit({'type': 'map_room'})
    assert ev == [] and b.queue[0]['id'] == t2['id']
    b.feedback(t1['id'], 'running')
    ev = b.feedback(t1['id'], 'succeeded', 'arrived')
    assert kinds(ev) == ['dispatch'] and b.active['id'] == t2['id']
    assert b.history[0]['state'] == 'succeeded'


def test_unknown_type_rejected_with_known_list():
    b, _ = board()
    t, ev = b.submit({'type': 'dance'})
    assert kinds(ev) == ['rejected'] and t['state'] == 'failed'
    assert 'goto' in t['message']


def test_plugin_can_add_a_type():
    b, _ = board()
    b.register('detector', ['follow_object'])
    t, ev = b.submit({'type': 'follow_object', 'label': 'ball'})
    assert kinds(ev) == ['dispatch'] and t['handler'] == 'detector'


def test_higher_priority_preempts_and_resumes_later():
    b, _ = board()
    low, _ = b.submit({'type': 'map_room'})
    b.feedback(low['id'], 'running')
    high, ev = b.submit({'type': 'goto', 'waypoints': [[0, 0]], 'priority': 9, 'source': 'detector'})
    assert kinds(ev) == ['cancel', 'dispatch']
    assert b.active['id'] == high['id'] and b.queue[0]['id'] == low['id']
    ev = b.feedback(high['id'], 'succeeded')
    assert b.active['id'] == low['id']


def test_stop_cancels_everything():
    b, _ = board()
    a, _ = b.submit({'type': 'map_room'})
    b.submit({'type': 'goto', 'waypoints': [[1, 1]]})
    t, ev = b.submit({'type': 'stop', 'source': 'web'})
    assert kinds(ev) == ['cancel'] and b.active is None and b.queue == []
    assert t['state'] == 'succeeded'


def test_request_is_forwarded_not_executed():
    b, _ = board()
    t, ev = b.submit({'type': 'request', 'text': 'map the kitchen'})
    assert kinds(ev) == ['request'] and b.active is None


def test_unacknowledged_dispatch_times_out():
    b, c = board()
    t, _ = b.submit({'type': 'goto', 'waypoints': [[1, 1]]})
    c.t = 6.0
    b.tick()
    assert b.active is None and b.history[0]['message'] == 'no executor responded'


def test_stale_module_no_longer_handles():
    b, c = board()
    c.t = 30.0
    t, ev = b.submit({'type': 'goto', 'waypoints': [[1, 1]]})
    assert kinds(ev) == ['rejected']


def test_bad_input():
    b, _ = board()
    t, ev = b.submit('not a dict')
    assert t is None and kinds(ev) == ['rejected']
