"""Independent native-state and RGB reset audit for the revised towel task."""

from dataclasses import asdict
import json
import os
from pathlib import Path
import sys

import numpy as np
from inspect_robots.cli import main
from yam_arena import inspect_embodiment


CAMERAS = ("top_cam", "left_cam", "right_cam")
ASSETS = ("left_robot", "right_robot", "dispenser", "feed_rollers", "towel_web")
FIELDS = ("body_link_pose_w", "body_link_vel_w", "joint_pos", "joint_vel")
FRAMES = (1, 118, 155, 205, 238, 260, 300, 316, 320, 334, 350, 450, 550, 650,
          750, 830, 890, 930)
DELIVERED_SNAPSHOTS = (238, 260, 316, 320, 334)
AUDIT = Path(os.environ["TOWEL_AUDIT_DIR"]) / "native-reset"
AUDIT.mkdir()
native_factory = inspect_embodiment.yam_arena_embodiment


def array(value):
    return getattr(value, "torch", value).detach().cpu().numpy().copy()


def cameras(arena, obs):
    assert set(obs.images) == set(CAMERAS)
    frames = {}
    for name in CAMERAS:
        raw = array(arena._base.scene[name].data.output["rgb"])[0, ..., :3]
        assert raw.shape == (360, 640, 3), (name, raw.shape)
        assert obs.images[name].shape == (176, 320, 3), (name, obs.images[name].shape)
        frames[name] = {"native": raw, "delivered": obs.images[name].copy()}
    return frames


def write_frames(frames, tag):
    for name, views in frames.items():
        for size, rgb in views.items():
            np.save(AUDIT / f"{tag}_{size}_{name}.npy", rgb)


def physical_state(arena):
    scene = arena._base.scene
    states = {f"{name}/{field}": array(getattr(scene[name].data, field))
              for name in ASSETS for field in FIELDS}
    states["free_web_edges"] = arena.task.module._paper_edges(arena._base).copy()
    return states


def max_errors(expected, actual):
    return {key: float(np.max(np.abs(value - actual[key]))) for key, value in expected.items()}


def snapshot(arena, obs):
    module = arena.task.module
    counters = {
        "feed": asdict(module._feed), "ratchet_depth": module._ratchet.depth,
        "commanded_feed_speed": module._roller_speed,
        "state_time": float(obs.state_time),
        "image_times": {k: float(v) for k, v in obs.image_times.items()},
    }
    return physical_state(arena), counters, cameras(arena, obs)


def audited_factory(**kwargs):
    arena = native_factory(**kwargs)
    reset_native, step_native, close_native = arena.reset, arena.step, arena.close
    episode = {}
    steps = 0

    def reset(scene, *, seed=None):
        nonlocal steps
        obs = reset_native(scene, seed=seed)
        steps = 0
        episode.update(scene=scene, seed=seed, initial=snapshot(arena, obs))
        write_frames(episode["initial"][2], "start")
        return obs

    def step(action):
        nonlocal steps
        result = step_native(action)
        steps += 1
        if steps == 1:
            episode.update(first_action=action, first_step=physical_state(arena))
        if steps in FRAMES:
            for name in CAMERAS:
                rgb = array(arena._base.scene[name].data.output["rgb"])[0, ..., :3]
                np.save(AUDIT / f"step-{steps:04}_native_{name}.npy", rgb)
        if steps in DELIVERED_SNAPSHOTS:
            for name, delivered in arena._observation().images.items():
                assert delivered.shape == (176, 320, 3)
                np.save(AUDIT / f"step-{steps:04}_delivered_{name}.npy", delivered)
        return result

    def close():
        if not episode or not steps:
            close_native()
            return
        try:
            terminal = snapshot(arena, arena._observation())
            write_frames(terminal[2], "terminal")
            trace = os.environ.pop("TOWEL_TRACE", None)
            try:
                after = snapshot(arena, reset_native(episode["scene"], seed=episode["seed"]))
                step_native(episode["first_action"])
                first_step_errors = max_errors(episode["first_step"], physical_state(arena))
            finally:
                if trace is not None:
                    os.environ["TOWEL_TRACE"] = trace
            write_frames(after[2], "reset")
            initial = episode["initial"]
            errors = max_errors(initial[0], after[0])
            visual = {}
            for name in CAMERAS:
                for size in ("native", "delivered"):
                    fresh = after[2][name][size].astype(float)
                    baseline = initial[2][name][size].astype(float)
                    before_reset = terminal[2][name][size].astype(float)
                    visual[f"{name}/{size}"] = {
                        "shape": list(fresh.shape),
                        "reset_to_start": float(np.abs(fresh - baseline).mean()),
                        "reset_to_terminal": float(np.abs(fresh - before_reset).mean()),
                    }
            criteria = {
                "physical": all(np.isfinite(v) and v < 2e-4 for v in errors.values()),
                "counters_and_timestamps": after[1] == initial[1]
                and after[1]["state_time"] == 0. and all(v == 0. for v in after[1]["image_times"].values()),
                "camera_buffers": all(v["reset_to_start"] < 3.
                                      and (v["reset_to_terminal"] <= 10.
                                           or v["reset_to_start"] < .15 * v["reset_to_terminal"])
                                      for v in visual.values()),
            }
            (AUDIT / "report.json").write_text(json.dumps({
                "max_state_errors": errors, "first_step_errors": first_step_errors,
                "visual": visual, "checks": criteria,
                "initial": initial[1], "after_reset": after[1], "terminal": terminal[1], "steps": steps,
            }, indent=2) + "\n")
        finally:
            close_native()

    arena.reset, arena.step, arena.close = reset, step, close
    return arena


inspect_embodiment.yam_arena_embodiment = audited_factory
sys.exit(main())
