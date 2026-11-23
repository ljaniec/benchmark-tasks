"""Inspect task with a physical thumb screw, feed button, and two stock blanks."""
import json
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import feeder_geometry as f
from feed_scorer import FeedScorer, Frame
from feeder_waypoints import demo
from mounted_feeder_fixture import ORIGIN, START_GAP, parts
from yam_arena.constants import GRIPPER


SCENE = "station"
INSTRUCTION = "Adjust the separator to feed one blank at a time. Press the test button and confirm that both blanks feed through separately."
GRIPPER["effort"] = 40.
OBJECTS = list(parts())
PROOF_STEPS = 120
scorer = FeedScorer()
remaining = 0
was_pressed = False


def set_feed_speed(base, speed):
    assembly = base.scene[f.MACHINE]
    names = [f"feed_roller_{i}_joint" for i in range(1, len(f.ROLLER_X) + 1)]
    ids = assembly.find_joints(names)[0]
    commanded = assembly.data.default_joint_vel.torch[:, ids].clone().fill_(speed)
    assembly.set_joint_velocity_target(commanded, joint_ids=ids)
    return assembly, ids


@f.m.on_reset
def reset(env):
    global remaining, was_pressed
    scorer.reset()
    remaining = 0
    was_pressed = False
    base = env.unwrapped
    assembly = base.scene[f.MACHINE]
    positions = assembly.data.default_joint_pos.torch.clone()
    velocities = assembly.data.default_joint_vel.torch.clone().zero_()
    assembly.write_joint_state_to_sim_index(position=positions, velocity=velocities)
    assembly.set_joint_position_target(positions)
    set_feed_speed(base, 0.)
    for name in f.STOCK:
        card = base.scene[name]
        pose = card.data.default_root_pose.torch.clone()
        pose[:, :3] += base.scene.env_origins
        card.write_root_pose_to_sim_index(root_pose=pose)
        card.write_root_velocity_to_sim_index(root_velocity=card.data.default_root_vel.torch.clone().zero_())
    base.sim.forward()
    cameras = [base.scene[name] for name in ("top_cam", "left_cam", "right_cam")]
    for camera in cameras:
        camera.reset()
    for _ in range(24):
        base.sim.render()
    for camera in cameras:
        camera.update(0., force_recompute=True)


def location(card):
    return np.asarray(card.pos) - ORIGIN


@f.m.on_step
def observe(env):
    global remaining, was_pressed
    fixture = f.m.state(env, f.MACHINE)
    lower, upper = [f.m.state(env, name) for name in f.STOCK]
    a, b = location(lower), location(upper)
    pressed = fixture.joint("proof_button") < -.0022
    starting = pressed and not was_pressed and remaining == 0
    was_pressed = pressed
    if starting:
        remaining = PROOF_STEPS
    running = remaining > 0
    assembly, roller_ids = set_feed_speed(env.unwrapped, f.DRIVE_RAD_S if running else 0.)
    tips = np.asarray(f.m.fingertips(env)) - ORIGIN
    over_stock = (np.abs(tips[:, 1]) < .048) & (-.127 < tips[:, 0]) & (tips[:, 0] < .235)
    in_pinch = over_stock & (.019 < tips[:, 2]) & (tips[:, 2] < .054)
    aligned = bool(np.all(np.abs(a[:2] - b[:2]) < (.0025, .0025))
                   and abs(a[0] + .063) < .004 and abs(a[1]) < .0015
                   and abs(b[2] - a[2] - f.THICKNESS) < .0007)
    stable = all(card.tilt_deg() < 10. and np.linalg.norm(card.lin_vel) < .015 for card in (lower, upper))
    safe = (np.isfinite(tips).all() and not in_pinch.any() and fixture.tilt_deg() <= 15.
            and all(-.125 <= xyz[0] <= .235 and abs(xyz[1]) <= .050 and .017 <= xyz[2] <= .058
                    for xyz in (a, b)))
    frame = Frame(
        pressed, running, aligned, bool(safe), a[0] + f.LENGTH * .5, a[0] - f.LENGTH * .5,
        b[0] + f.LENGTH * .5, bool(stable and b[0] - f.LENGTH * .5 >= .012),
    )
    score = scorer.update(frame)
    destination = os.getenv("CARTON_FEEDER_TRACE")
    if destination:
        sample = dict(vars(frame), score=score, cycles_remaining=remaining,
                      gap_m=START_GAP + fixture.joint("separator"),
                      knob_deg=fixture.deg("adjuster"), button_m=-fixture.joint("proof_button"),
                      roller_rad_s=assembly.data.joint_vel.torch[0, roller_ids].tolist(),
                      roller_rad=assembly.data.joint_pos.torch[0, roller_ids].tolist(),
                      blank_centers=[a.tolist(), b.tolist()])
        with open(destination, "a", encoding="utf-8") as output:
            output.write(json.dumps(sample, allow_nan=False) + "\n")
    remaining = max(0, remaining - 1)


def score(_objects):
    return scorer.result
