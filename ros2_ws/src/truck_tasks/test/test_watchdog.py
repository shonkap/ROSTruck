from truck_tasks.watchdog import WatchdogLogic

TASK = {'id': 1, 'type': 'map_room', 'source': 'web'}


def run(w, seconds, voltage, link_ok=True, task=TASK, t0=0.0):
    acts = []
    for i in range(int(seconds)):
        acts += w.update(t0 + i, voltage, link_ok, task)
    return acts


def test_off_by_default_and_no_battery_never_triggers():
    assert run(WatchdogLogic(), 30, 5.0) == []
    assert run(WatchdogLogic(7.0, 6.6), 30, 0.0) == []          # 0 V = not measured
    assert run(WatchdogLogic(7.0, 6.6), 30, float('nan')) == []


def test_low_battery_returns_home_once_after_hold():
    w = WatchdogLogic(7.0, 6.6, hold_s=10)
    acts = run(w, 9, 6.9)
    assert acts == []
    acts = run(w, 20, 6.9, t0=9)
    assert [a[0] for a in acts] == ['return_home']
    # no task running -> nothing to return from
    assert run(WatchdogLogic(7.0, 6.6, hold_s=1), 5, 6.9, task=None) == []


def test_critical_battery_stops():
    w = WatchdogLogic(7.0, 6.6, hold_s=2)
    acts = run(w, 5, 6.5)
    assert ('stop' in [a[0] for a in acts])


def test_link_loss_stops_running_task_once():
    w = WatchdogLogic(link_timeout_s=3)
    acts = run(w, 10, None, link_ok=False)
    assert [a[0] for a in acts] == ['stop']
    run(w, 2, None, link_ok=True, t0=10)                        # link back: re-armed
    assert [a[0] for a in run(w, 10, None, link_ok=False, t0=20)] == ['stop']
