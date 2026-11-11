"""Claude call for the planner: robot context in, structured plan out.

Uses the official `anthropic` SDK, so this runs on the PC (Python 3.10+), not
on the Jetson's Python 3.6 - which is fine: it's a task-bus plugin and talks
to the truck over ROS like everything else.

Credentials: ANTHROPIC_API_KEY (or an `ant auth login` profile) in the
environment of the planner node.
"""
import json
import os

from truck_planner.planner_core import PLAN_SCHEMA, SYSTEM_PROMPT

DEFAULT_MODEL = 'claude-opus-5-5'


class PlannerError(Exception):
    def __init__(self, msg, fallback_ok=False):
        Exception.__init__(self, msg)
        self.fallback_ok = fallback_ok   # credentials/network: keyword planner may answer instead


def have_credentials():
    """Any of the credential sources the SDK resolves (it only fails at request time)."""
    env = os.environ
    if env.get('ANTHROPIC_API_KEY') or env.get('ANTHROPIC_AUTH_TOKEN') or env.get('ANTHROPIC_PROFILE'):
        return True
    if env.get('ANTHROPIC_FEDERATION_RULE_ID') and (env.get('ANTHROPIC_IDENTITY_TOKEN') or
                                                     env.get('ANTHROPIC_IDENTITY_TOKEN_FILE')):
        return True
    return os.path.isdir(os.path.expanduser('~/.config/anthropic'))   # `ant auth login` profile


def make_client():
    """Anthropic client, or None if the SDK or credentials are missing."""
    if not have_credentials():
        return None
    try:
        import anthropic
        return anthropic.Anthropic()
    except Exception:
        return None


def plan_with_claude(client, context, model=DEFAULT_MODEL, effort='medium'):
    """Returns a plan dict matching PLAN_SCHEMA. Raises PlannerError."""
    import anthropic
    try:
        response = client.beta.messages.create(
            model=model,
            max_tokens=16000,
            system=SYSTEM_PROMPT,
            messages=[{
                'role': 'user',
                'content': 'Robot context (JSON):\n' + json.dumps(context, indent=1) +
                           '\n\nPlan for the request above.',
            }],
            output_config={
                'effort': effort,
                'format': {'type': 'json_schema', 'schema': PLAN_SCHEMA},
            },
            # On a safety decline, re-run on Anthropic's recommended fallback model.
            betas=['server-side-fallback-2026-07-01'],
            fallbacks='default',
        )
    except anthropic.AuthenticationError as exc:
        raise PlannerError('Claude rejected the credentials: {}'.format(exc), fallback_ok=True)
    except anthropic.RateLimitError as exc:
        raise PlannerError('Claude rate limit, try again shortly: {}'.format(exc))
    except anthropic.APIStatusError as exc:
        raise PlannerError('Claude API error {}: {}'.format(exc.status_code, exc))
    except anthropic.APIConnectionError as exc:
        raise PlannerError('cannot reach the Claude API (network?): {}'.format(exc), fallback_ok=True)

    if response.stop_reason == 'refusal':
        raise PlannerError('Claude declined this request.')
    if response.stop_reason == 'max_tokens':
        raise PlannerError('plan was cut off (max_tokens).')
    text = next((b.text for b in response.content if b.type == 'text'), None)
    if not text:
        raise PlannerError('no plan in the response.')
    try:
        plan = json.loads(text)
    except ValueError as exc:
        raise PlannerError('plan was not valid JSON: {}'.format(exc))
    plan['model'] = response.model
    return plan
