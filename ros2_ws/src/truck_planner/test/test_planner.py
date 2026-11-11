import json
from types import SimpleNamespace

import numpy as np
import pytest

from truck_planner.llm import PlannerError, plan_with_claude
from truck_planner.planner_core import (PLAN_SCHEMA, build_context, rule_based_plan,
                                        summarize_map, validate_plan)
from truck_tasks.planning import GridMap


def grid():
    g = np.zeros((40, 60), np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    g[30:39, 40:59] = -1
    return GridMap(g.ravel().tolist(), 60, 40, 0.1, 0.0, 0.0)


def test_schema_is_strict_everywhere():
    assert PLAN_SCHEMA['additionalProperties'] is False
    item = PLAN_SCHEMA['properties']['tasks']['items']
    assert item['additionalProperties'] is False
    assert set(item['required']) == set(item['properties'])


def test_map_summary_and_context():
    s = summarize_map(grid(), (1.0, 1.0, 0.0), lambda g, p: [(4.4, 3.5, 30)])
    assert s['available'] and s['free_area_m2'] > 15
    assert s['unexplored_frontiers'] == [[4.4, 3.5]]
    ctx = build_context('map it', (1.0, 1.0, 0.0), s, 'connected', {'queue': [{'type': 'goto'}]}, [])
    assert ctx['truck_pose'] == {'x': 1.0, 'y': 1.0, 'heading_deg': 0}
    assert ctx['queued_tasks'] == ['goto']
    json.dumps(ctx)      # must be serialisable for the prompt


def test_validation_drops_bad_tasks_and_blocked_waypoints():
    gm = grid()
    free = lambda x, y: (x, y) if 0 <= gm.grid[gm.to_cell(x, y)] < 65 else None  # noqa: E731
    plan = {'tasks': [
        {'type': 'goto', 'waypoints': [[1, 1], [0.0, 0.0], ['a', 2]], 'max_minutes': 0, 'note': ''},
        {'type': 'dance', 'waypoints': [], 'max_minutes': 0, 'note': ''},
        {'type': 'map_room', 'waypoints': [], 'max_minutes': 999, 'note': ''},
        {'type': 'goto', 'waypoints': [[0.0, 0.0]], 'max_minutes': 0, 'note': ''},
    ]}
    tasks, problems = validate_plan(plan, free)
    assert tasks[0] == {'type': 'goto', 'waypoints': [[1.0, 1.0]], 'note': ''}   # (0,0) is wall
    assert tasks[1]['type'] == 'map_room' and tasks[1]['max_minutes'] == 30.0
    assert len(tasks) == 2
    assert any('dance' in p for p in problems) and any('no reachable' in p for p in problems)


def test_rule_based_fallback():
    assert rule_based_plan('please STOP now')['tasks'][0]['type'] == 'stop'
    assert rule_based_plan('go to 1.5, -2')['tasks'][0]['waypoints'] == [[1.5, -2.0]]
    assert rule_based_plan('map the kitchen')['tasks'][0]['type'] == 'map_room'
    assert rule_based_plan('drive home')['tasks'][0]['waypoints'] == [[0.0, 0.0]]
    lm = [{'label': 'couch', 'x': 2.0, 'y': 1.0}]
    assert rule_based_plan('go to the couch', landmarks=lm)['tasks'][0]['waypoints'] == [[2.0, 1.0]]
    lm2 = [{'label': 'red_bin', 'x': 0.2, 'y': -2.5}]
    assert rule_based_plan('drive to the red bin', landmarks=lm2)['tasks'][0]['waypoints'] == [[0.2, -2.5]]
    assert rule_based_plan('make me a sandwich')['tasks'] == []


class FakeClient(object):
    """Stands in for anthropic.Anthropic(): records the request, returns a canned message."""

    def __init__(self, text, stop_reason='end_turn'):
        self.sent = None
        resp = SimpleNamespace(stop_reason=stop_reason, model='claude-opus-5-5',
                               content=[SimpleNamespace(type='thinking', thinking=''),
                                        SimpleNamespace(type='text', text=text)])

        def create(**kw):
            self.sent = kw
            return resp
        self.beta = SimpleNamespace(messages=SimpleNamespace(create=create))


def test_claude_request_shape_and_parse():
    pytest.importorskip('anthropic')
    plan = {'title': 'Map', 'report': 'ok', 'tasks': [
        {'type': 'map_room', 'waypoints': [], 'max_minutes': 5, 'note': 'explore'}]}
    c = FakeClient(json.dumps(plan))
    out = plan_with_claude(c, {'request': 'map'})
    assert out['tasks'][0]['type'] == 'map_room' and out['model'] == 'claude-opus-5-5'
    assert c.sent['model'] == 'claude-opus-5-5'
    assert c.sent['output_config']['format']['schema'] is PLAN_SCHEMA
    assert c.sent['fallbacks'] == 'default'
    assert 'server-side-fallback-2026-07-01' in c.sent['betas']
    assert 'thinking' not in c.sent          # Opus 5.5: adaptive by default


def test_claude_refusal_and_truncation_raise():
    pytest.importorskip('anthropic')
    with pytest.raises(PlannerError):
        plan_with_claude(FakeClient('', stop_reason='refusal'), {})
    with pytest.raises(PlannerError):
        plan_with_claude(FakeClient('{"tit', stop_reason='max_tokens'), {})


def test_no_credentials_means_no_client(monkeypatch, tmp_path):
    from truck_planner import llm
    for k in ('ANTHROPIC_API_KEY', 'ANTHROPIC_AUTH_TOKEN', 'ANTHROPIC_PROFILE',
              'ANTHROPIC_FEDERATION_RULE_ID'):
        monkeypatch.delenv(k, raising=False)
    monkeypatch.setenv('HOME', str(tmp_path))
    assert llm.have_credentials() is False and llm.make_client() is None
    monkeypatch.setenv('ANTHROPIC_API_KEY', 'x')
    assert llm.have_credentials() is True


def test_waypoint_on_an_obstacle_is_moved_next_to_it():
    plan = {'tasks': [{'type': 'goto', 'waypoints': [[2.0, 1.0]], 'max_minutes': 0, 'note': ''}]}
    tasks, problems = validate_plan(plan, lambda x, y: (x + 0.5, y))
    assert tasks[0]['waypoints'] == [[2.5, 1.0]] and 'next to it' in problems[0]


def test_keyword_planner_uses_places_longest_name_first():
    places = [{'name': 'door', 'x': 1.0, 'y': 1.0}, {'name': 'kitchen door', 'x': 3.0, 'y': 0.5}]
    plan = rule_based_plan('please go to the kitchen door', places=places)
    assert plan['tasks'][0]['waypoints'] == [[3.0, 0.5]]
