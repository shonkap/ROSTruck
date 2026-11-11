"""Planner logic, ROS-free: robot context for the LLM, the plan schema, plan
validation, and a rule-based fallback when no LLM is configured.

A plan is what the planner answers a natural-language request with:
    {"title": str, "report": str, "tasks": [task, ...]}
where each task is one the task manager understands, e.g.
    {"type": "goto", "waypoints": [[x, y], ...]}
    {"type": "map_room", "max_minutes": 8}
    {"type": "stop"}
"""
import math
import re

# Task types the planner may propose (it must never invent others).
PLANNABLE = ('goto', 'map_room', 'stop')
MAX_WAYPOINTS = 20

# JSON schema for Claude's structured output (strict: every field required,
# no extras; "not applicable" is an empty list / 0 / "").
PLAN_SCHEMA = {
    'type': 'object',
    'properties': {
        'title': {'type': 'string', 'description': 'Short title of the plan (under 60 chars).'},
        'report': {'type': 'string', 'description': (
            'Plain-language report for the operator: what you understood, the plan, '
            'assumptions, risks, and anything the truck cannot do.')},
        'tasks': {
            'type': 'array',
            'description': 'Tasks to queue, in order. Empty if nothing should be executed.',
            'items': {
                'type': 'object',
                'properties': {
                    'type': {'type': 'string', 'enum': list(PLANNABLE)},
                    'waypoints': {
                        'type': 'array',
                        'description': 'goto only: [x, y] map points in metres, else empty.',
                        'items': {'type': 'array', 'items': {'type': 'number'}},
                    },
                    'max_minutes': {'type': 'number',
                                    'description': 'map_room only: time limit in minutes, else 0.'},
                    'note': {'type': 'string', 'description': 'Why this task (one line).'},
                },
                'required': ['type', 'waypoints', 'max_minutes', 'note'],
                'additionalProperties': False,
            },
        },
    },
    'required': ['title', 'report', 'tasks'],
    'additionalProperties': False,
}

SYSTEM_PROMPT = """You plan missions for a small autonomous RC truck (Redcat Ground Pounder) driving indoors.

How the truck works:
- It is car-like: front and rear steering, one motor, it cannot turn in place (minimum turn radius about 0.35 m) and needs about 0.5 m clearance from obstacles.
- It localises with stereo-camera SLAM on a 2D occupancy map (metres, map frame; x/y as given in the context). Unknown areas have not been seen yet.
- It can only do these tasks: "goto" (drive through map waypoints in order), "map_room" (explore until the map stops growing), "stop" (cancel everything).
- Landmarks, if listed, are objects a camera detector has seen, with their map position.
- Places, if listed, are named map positions saved by the operator; goto waypoints may be given as [x, y] using their coordinates.
- A human pilot holds the RC transmitter and must switch the truck to GUIDED for any task to move it.

Plan with the tasks above only. Put waypoints on known free space, away from obstacles; if the request needs a place that isn't mapped or a landmark that isn't known, say so in the report and propose mapping first instead of guessing coordinates. If the request is unsafe or impossible, return no tasks and explain why. Keep the report short and concrete."""


def summarize_map(gm, pose=None, frontier_fn=None):
    """Compact text-friendly map summary for the prompt."""
    if gm is None:
        return {'available': False}
    import numpy as np
    known = gm.grid >= 0
    free = (gm.grid >= 0) & (gm.grid < 25)
    occ = gm.grid >= 65
    out = {'available': True, 'resolution_m': round(gm.res, 3),
           'known_area_m2': round(float(known.sum()) * gm.res * gm.res, 1),
           'free_area_m2': round(float(free.sum()) * gm.res * gm.res, 1),
           'occupied_area_m2': round(float(occ.sum()) * gm.res * gm.res, 1)}
    if free.any():
        rows, cols = np.nonzero(free)
        x0, y0 = gm.to_world(rows.min(), cols.min())
        x1, y1 = gm.to_world(rows.max(), cols.max())
        out['free_space_bounds'] = {'x_min': round(x0, 2), 'x_max': round(x1, 2),
                                    'y_min': round(y0, 2), 'y_max': round(y1, 2)}
        out['free_space_centroid'] = [round(float(gm.x0 + (cols.mean() + 0.5) * gm.res), 2),
                                      round(float(gm.y0 + (rows.mean() + 0.5) * gm.res), 2)]
    if frontier_fn is not None and pose is not None:
        try:
            fr = frontier_fn(gm, pose)
            out['unexplored_frontiers'] = [[round(f[0], 2), round(f[1], 2)] for f in fr[:8]]
        except Exception:
            pass
    return out


def build_context(request_text, pose, map_summary, fc_status, task_status, landmarks, places=None):
    tk = task_status or {}
    active = tk.get('active')
    return {
        'request': request_text,
        'truck_pose': None if pose is None else {'x': round(pose[0], 2), 'y': round(pose[1], 2),
                                                 'heading_deg': round(math.degrees(pose[2]))},
        'fc_status': fc_status,
        'map': map_summary,
        'landmarks': landmarks,
        'places': places or [],
        'active_task': None if not active else {'type': active.get('type'),
                                                'message': active.get('message')},
        'queued_tasks': [t.get('type') for t in tk.get('queue', [])],
    }


def validate_plan(plan, snap=None):
    """Keep only well-formed, executable tasks. Returns (tasks, problems).

    snap(x, y) -> (x, y) or None: moves a waypoint to reachable free space with
    clearance (e.g. a landmark's own position is an obstacle - park next to
    it), or None if there's nowhere to stand nearby."""
    tasks, problems = [], []
    for i, t in enumerate((plan or {}).get('tasks') or []):
        kind = t.get('type')
        if kind not in PLANNABLE:
            problems.append('task {}: unknown type {!r} dropped'.format(i + 1, kind))
            continue
        if kind == 'goto':
            pts = []
            for p in (t.get('waypoints') or [])[:MAX_WAYPOINTS]:
                try:
                    x, y = float(p[0]), float(p[1])
                except (TypeError, ValueError, IndexError):
                    problems.append('task {}: bad waypoint {!r} dropped'.format(i + 1, p))
                    continue
                if not (math.isfinite(x) and math.isfinite(y)):
                    continue
                if snap is not None:
                    s = snap(x, y)
                    if s is None:
                        problems.append('task {}: no reachable free space near ({:.2f}, {:.2f}) - '
                                        'dropped'.format(i + 1, x, y))
                        continue
                    if math.hypot(s[0] - x, s[1] - y) > 0.1:
                        problems.append('task {}: ({:.2f}, {:.2f}) is not drivable, using ({:.2f}, '
                                        '{:.2f}) next to it'.format(i + 1, x, y, s[0], s[1]))
                    x, y = s
                pts.append([round(x, 2), round(y, 2)])
            if not pts:
                problems.append('task {}: goto without usable waypoints dropped'.format(i + 1))
                continue
            tasks.append({'type': 'goto', 'waypoints': pts, 'note': t.get('note', '')})
        elif kind == 'map_room':
            mm = t.get('max_minutes') or 10
            tasks.append({'type': 'map_room', 'max_minutes': max(1.0, min(30.0, float(mm))),
                          'note': t.get('note', '')})
        else:
            tasks.append({'type': 'stop', 'priority': 10, 'note': t.get('note', '')})
    return tasks, problems


_NUM = r'(-?\d+(?:\.\d+)?)'


def rule_based_plan(text, pose=None, landmarks=None, places=None):
    """Tiny keyword planner used when no LLM is configured."""
    t = text.lower()
    note = ('(No LLM configured - this is the built-in keyword planner. Set '
            'ANTHROPIC_API_KEY for the planner node to get real plans.)')
    if re.search(r'\b(stop|halt|abort|cancel)\b', t):
        return {'title': 'Stop', 'report': 'Stopping all tasks. ' + note, 'tasks': [
            {'type': 'stop', 'waypoints': [], 'max_minutes': 0, 'note': 'requested stop'}]}
    pts = [[float(a), float(b)] for a, b in re.findall(_NUM + r'\s*,\s*' + _NUM, t)]
    if pts and re.search(r'\b(go|drive|move|goto|visit)\b', t):
        return {'title': 'Drive to {} point(s)'.format(len(pts)),
                'report': 'Driving to {}. {}'.format(pts, note), 'tasks': [
                    {'type': 'goto', 'waypoints': pts, 'max_minutes': 0, 'note': 'from request'}]}
    if re.search(r'\b(home|start|origin)\b', t):
        return {'title': 'Return to start', 'report': 'Driving back to the map origin. ' + note,
                'tasks': [{'type': 'goto', 'waypoints': [[0.0, 0.0]], 'max_minutes': 0,
                           'note': 'map origin'}]}
    named = [{'label': p['name'], 'x': p['x'], 'y': p['y']} for p in places or []]
    for lm in sorted(named + list(landmarks or []), key=lambda l: -len(l.get('label', ''))):
        name = lm.get('label', '').lower()
        if name and (name in t or name.replace('_', ' ') in t):
            return {'title': 'Go to {}'.format(lm['label']),
                    'report': 'Driving to the {} seen at ({:.2f}, {:.2f}). {}'.format(
                        lm['label'], lm['x'], lm['y'], note),
                    'tasks': [{'type': 'goto', 'waypoints': [[lm['x'], lm['y']]], 'max_minutes': 0,
                               'note': 'landmark'}]}
    if re.search(r'\b(map|explore|scan|survey)\b', t):
        return {'title': 'Map the room', 'report': 'Exploring until the map stops growing. ' + note,
                'tasks': [{'type': 'map_room', 'waypoints': [], 'max_minutes': 10,
                           'note': 'from request'}]}
    return {'title': 'Not understood', 'report': "I can only stop, map/explore, go home, or go to "
            "'x, y' coordinates or known landmarks. " + note, 'tasks': []}
