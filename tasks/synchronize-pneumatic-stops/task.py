"""Restore the timing of two guarded pneumatic stops."""
import json
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as m
import pneumatic_fixture as g
from diagnostic_cycle import TestCycle
from paired_waypoints import demo
from timing_scorer import Frame, TimingScorer
from yam_arena.constants import GRIPPER


SCENE = "station"
INSTRUCTION = "Run the test cycle. Adjust and lock the slow stop's speed control, then show that the two stops finish together twice."
FIXTURE = "paired_fixture"
GRIPPER["effort"] = 40.
OBJECTS = [g.station_model()]
scorer = TimingScorer()
cycle = TestCycle()
SWEEP_MIN = np.array((-.161, -.023, .052))
SWEEP_MAX = np.array((-.060, .023, .117))


@m.on_reset
def reset(environment):
    cycle.reset()
    scorer.reset()
    base = environment.unwrapped
    fixture = base.scene[FIXTURE]
    positions = fixture.data.default_joint_pos.torch.clone()
    velocities = fixture.data.default_joint_vel.torch.clone().zero_()
    fixture.write_joint_state_to_sim_index(position=positions, velocity=velocities)
    fixture.set_joint_position_target(positions)
    base.sim.forward()
    cameras = [base.scene[name] for name in ("top_cam", "left_cam", "right_cam")]
    for camera in cameras:
        camera.reset()
    for _ in range(24):
        base.sim.render()
    for camera in cameras:
        camera.update(0., force_recompute=True)


def is_safe(part, points):
    tips = part.to_local(points)
    safe = np.isfinite(tips).all() and part.tilt_deg() <= 15.
    for side in (-.109, .109):
        relative = tips - (0., side, 0.)
        inside = ((relative >= SWEEP_MIN) & (relative <= SWEEP_MAX)).all(axis=1)
        safe &= not inside.any()
    return bool(safe)


@m.on_step
def observe(environment):
    part = m.state(environment, FIXTURE)
    distances = (-part.joint("reference"), -part.joint("adjusted"))
    pressed = part.joint("test_button") <= -.0028
    cap_lift = part.joint("cap_lift")
    locked = abs(cap_lift) <= .00025
    dial = part.deg("flow_knob")
    safe = is_safe(part, m.fingertips(environment)) and np.isfinite(dial)
    frame = Frame(*distances, pressed, locked, True, bool(safe))
    value = scorer.update(frame)
    for name, target in zip(("reference", "adjusted"), cycle.update(pressed, locked, dial, distances), strict=True):
        m.set_target(environment, FIXTURE, name, -target)
    trace = os.getenv("PAIRED_STOPS_TRACE")
    if trace:
        row = dict(vars(frame), dial_deg=dial, cap_lift_m=cap_lift, score=value)
        with open(trace, "a", encoding="utf-8") as output:
            output.write(json.dumps(row, allow_nan=False) + "\n")


def score(_objects):
    return scorer.score
