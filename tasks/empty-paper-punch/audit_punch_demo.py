"""Save delivered RGB, native RGB, and a matched-seed reset check."""
import json
import os
from pathlib import Path
import sys

import numpy as np
from inspect_robots.cli import main
from yam_arena import inspect_embodiment


CAMERAS = ("top_cam", "left_cam", "right_cam")
ARTICULATIONS = ("left_robot", "right_robot", "paper_punch")
STEPS = (65, 105, 130, 170, 200, 225, 255, 300, 350, 395, 425, 460, 490, 510)
output = Path(os.environ["PUNCH_AUDIT_DIR"])
camera_output = output / "camera-audit"
camera_output.mkdir()
make_arena = inspect_embodiment.yam_arena_embodiment


def as_array(value):
    return getattr(value, "torch", value).detach().cpu().numpy().copy()


def snapshot(arena, observation):
    task = arena.task.module
    names = (*ARTICULATIONS, *task.scene.SCRAP_NAMES)
    state = {}
    for name in names:
        data = arena._base.scene[name].data
        for field in ("root_link_pose_w", "root_vel_w"):
            state[f"{name}/{field}"] = as_array(getattr(data, field))
        if name in ARTICULATIONS:
            for field in ("joint_pos", "joint_vel"):
                state[f"{name}/{field}"] = as_array(getattr(data, field))
    info = {
        "score": vars(task.scorer).copy(), "cover_ticks": task._cover_ticks,
        "cover_verified": task._cover_verified, "has_previous_scraps": task._previous_scraps is not None,
        "image_times": observation.image_times, "state_time": observation.state_time,
        "physics_hz": round(1. / arena._base.sim.cfg.dt), "control_hz": round(1. / arena._base.step_dt),
    }
    return state, info, {name: frame.copy() for name, frame in observation.images.items()}


def record(arena, label, observation):
    assert set(observation.images) == set(CAMERAS)
    for name in CAMERAS:
        img = observation.images[name]
        frame = as_array(arena._base.scene[name].data.output["rgb"])[0, ..., :3]
        assert img.shape == (176, 320, 3) and frame.shape == (360, 640, 3)
        np.save(camera_output / f"{label}-delivered_{name}.npy", img)
        np.save(camera_output / f"{label}_{name}.npy", frame)


def gradient_alignment(before, after):
    vectors = []
    for frame in (before, after):
        gray = frame.astype(np.float32).mean(axis=2)
        gradient = np.concatenate([np.diff(gray, axis=axis).ravel() for axis in (0, 1)])
        vectors.append(gradient)
    norms = np.linalg.norm(vectors[0]) * np.linalg.norm(vectors[1])
    return float(vectors[0] @ vectors[1] / norms) if norms > 0. else 0.


def audit_factory(**kwargs):
    arena = make_arena(**kwargs)
    reset_arena, step_arena, close_arena = arena.reset, arena.step, arena.close
    initial = {}
    steps = 0

    def reset(scene, *, seed=None):
        obs = reset_arena(scene, seed=seed)
        initial.update(scene=scene, seed=seed, snapshot=snapshot(arena, obs))
        record(arena, "start", obs)
        return obs

    def step(action):
        nonlocal steps
        result = step_arena(action)
        steps += 1
        if steps in STEPS:
            record(arena, f"t{steps:04d}", result.observation)
        return result

    def close():
        if not initial or not steps:
            close_arena()
            return
        before, counters, frames = initial["snapshot"]
        terminal = vars(arena.task.module.scorer).copy()
        trace_path = os.environ.pop("PUNCH_TRACE", None)
        try:
            obs = reset_arena(initial["scene"], seed=initial["seed"])
            after, after_counters, after_frames = snapshot(arena, obs)
            record(arena, "reset", obs)
        finally:
            if trace_path is not None:
                os.environ["PUNCH_TRACE"] = trace_path
            close_arena()
        errors = {name: float(np.abs(value - after[name]).max()) for name, value in before.items()}
        rgb = {}
        for name in CAMERAS:
            delta = np.abs(frames[name].astype(np.float32) - after_frames[name])
            rgb[name] = {
                "mean_delta": float(delta.mean()),
                "fraction_over_10": float((delta > 10.).mean()),
                "gradient_alignment": gradient_alignment(frames[name], after_frames[name]),
            }
        checks = {
            "physical_state": max(errors.values()) <= 2.e-4,
            "counters_and_timestamps": counters == after_counters,
            "rates": after_counters["physics_hz"] == 480 and after_counters["control_hz"] == 20,
            "cameras": all(row["mean_delta"] < 3. and row["gradient_alignment"] >= .95 for row in rgb.values()),
        }
        result = dict(checks=checks, steps=steps, terminal=terminal, reset=after_counters, max_state_errors=errors, rgb=rgb)
        (output / "reset-audit.json").write_text(json.dumps(result, indent=2) + "\n")
        assert all(checks.values()), "Post-trial punch reset audit failed"

    arena.reset, arena.step, arena.close = reset, step, close
    return arena


inspect_embodiment.yam_arena_embodiment = audit_factory
sys.exit(main())
