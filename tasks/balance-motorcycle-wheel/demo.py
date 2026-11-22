"""Play the fixture press and use overhead RGB to select each wheel turn."""

from collections import deque
from dataclasses import replace
from pathlib import Path

import numpy as np

from demo_actions import actions
from scorer import MARKS_DEG, offset_deg
from wheel_index_rgb import WheelIndexReader


HERE = Path(__file__).resolve().parent
PARK_STEPS = 20
VISIBILITY_WAIT_STEPS = 12
RELEASE_PAUSE = 110
STAGING_MARK_DEG = 315.


def park_applicators(press_plan):
    """Retrace the calibrated approach, then open above the original trays."""
    by_phase = {row['phase']: row for row in press_plan['waypoints']}
    return {'waypoints': [
        by_phase['release_watch'],
        {**by_phase['line_up'], 'phase': 'clear_rim', 'steps': 10},
        {**by_phase['lift'], 'phase': 'over_trays'},
        {**by_phase['close'], 'phase': 'lower_applicators', 'steps': 24},
        {**by_phase['surround'], 'phase': 'park_applicators', 'steps': 20},
        {**by_phase['lift'], 'phase': 'withdraw_empty_jaws', 'finger_m': by_phase['surround']['finger_m']},
    ]}


class WheelDemoMotion:
    """Use fixed setup calibration and the captured index angle at each release."""

    def __init__(self, press_plan, release_routes):
        self.reader = WheelIndexReader()
        self.pending = deque(command for _, command in actions(press_plan))
        self.pending.extend(command for _, command in actions(park_applicators(press_plan)))
        self.marks = deque((STAGING_MARK_DEG, *MARKS_DEG))
        self.travel_deg = release_routes['travel_deg']
        self.approach = release_routes['approach']
        self.commands = release_routes['commands']
        self.contact_controls = int(release_routes['contact_controls'])
        self.last = None
        self.waited = 0
        self.active_mark = None

    def _queue_approach(self):
        start = self.approach[0]
        for step in range(1, PARK_STEPS + 1):
            t = step / PARK_STEPS
            blend = t * t * (3. - 2. * t)
            self.pending.append(self.last + (start - self.last) * blend)
        self.pending.extend(self.commands[0, :self.contact_controls])

    def _queue_turn(self, visible_deg):
        travel = offset_deg(self.active_mark, visible_deg)
        route = int(np.argmin(np.abs(self.travel_deg - travel)))
        if abs(self.travel_deg[route] - travel) > .35:
            return False
        self.pending.extend(self.commands[route, self.contact_controls:])
        rest = VISIBILITY_WAIT_STEPS if self.active_mark == STAGING_MARK_DEG else RELEASE_PAUSE
        self.pending.extend([self.commands[route, -1]] * rest)
        return True

    def next_action(self, overhead_rgb):
        if not self.pending:
            if self.active_mark is None:
                if not self.marks:
                    return self.last, 'route_finished'
                self.active_mark = self.marks.popleft()
                self._queue_approach()
            else:
                visible = self.reader.route_angle_deg(overhead_rgb)
                if visible is None:
                    self.waited += 1
                    reason = 'index_unreadable' if self.waited >= VISIBILITY_WAIT_STEPS else None
                    return self.last, reason
                self.waited = 0
                if not self._queue_turn(visible):
                    return self.last, 'index_outside_screened_arc'
                self.active_mark = None
        self.last = self.pending.popleft()
        return self.last, None


def register_policy():
    import json

    from inspect_robots.registry import register
    from inspect_robots.types import Action, ActionChunk
    from yam_arena.scripted import YamScriptedPolicy

    class WheelDemoPolicy(YamScriptedPolicy):
        def __init__(self):
            super().__init__()
            self.info = replace(self.info, name='wheel_balance_demo')

        def reset(self, scene):
            press_plan = json.loads((HERE / 'robot-press-shoe-plan.json').read_text())
            with np.load(HERE / 'release-routes.npz') as routes:
                self.motion = WheelDemoMotion(press_plan, routes)

        def act(self, observation):
            action, reason = self.motion.next_action(observation.images['top_cam'])
            meta = {'request_stop': True, 'demo_stop_reason': reason} if reason else {}
            return ActionChunk(actions=[Action(data=action, meta=meta)], control_hz=20.)

    register('policy', 'wheel_balance_demo')(WheelDemoPolicy)
    return 'wheel_balance_demo'
