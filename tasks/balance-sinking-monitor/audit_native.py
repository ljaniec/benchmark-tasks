"""Capture native and delivered views and compare a same-process episode reset."""

import json
import os
from pathlib import Path
import sys

import numpy as np
from inspect_robots.cli import main
from yam_arena import inspect_embodiment


CAMERAS = ("top_cam", "left_cam", "right_cam")
ASSETS = ("left_robot", "right_robot", "monitor_arm")
FIELDS = ("body_link_pose_w", "body_link_vel_w", "joint_pos", "joint_vel")
FRAME_STEPS = {45, 80, 118, 150, 168, 193, 211, 240, 298, 351, 398, 484, 568}
REPLAY_STEPS = {1, 20, 50}
AUDIT = Path("native-reset")
native_factory = inspect_embodiment.yam_arena_embodiment


def array(value):
    return getattr(value, "torch", value).detach().cpu().numpy().copy()


def state(arena):
    scene = arena._base.scene
    bodies = {f"{name}/{field}": array(getattr(scene[name].data, field)) for name in ASSETS for field in FIELDS}
    for field in ("root_link_pose_w", "root_link_vel_w"):
        bodies[f"hex_key/{field}"] = array(getattr(scene["hex_key"].data, field))
    return bodies


def snapshot(arena, observation, label):
    scene = arena._base.scene
    physical = state(arena)
    module = arena.task.module
    counters = {"void": module._release.fault, "dwell": module._release.steps, "score": module._release.value,
                "steps": module._step, "holds": sorted(module._release.held_heights),
                "t": float(observation.state_time),
                "camera_times": {name: float(t) for name, t in observation.image_times.items()}}
    frames = {}
    for name in CAMERAS:
        native = array(scene[name].data.output["rgb"])[0, ..., :3]
        delivered = observation.images[name].copy()
        assert native.shape == (360, 640, 3)
        assert delivered.shape == (176, 320, 3)
        frames[name + "/native"] = native
        frames[name + "/delivered"] = delivered
    for name, frame in frames.items():
        np.save(AUDIT / f"{label}_{name.replace('/', '_')}.npy", frame)
    return physical, counters, frames


def errors(a, b):
    return {name: float(np.max(np.abs(value - b[name]))) for name, value in a.items()}


def camera_errors(initial, repeated, terminal):
    initial = initial.astype(np.float32)
    delta = np.abs(repeated.astype(np.float32) - initial)
    mean = float(delta.mean())
    changed_fraction = float((np.max(delta, axis=-1) > 255. / 8.).mean())
    terminal_mean = float(np.abs(terminal.astype(np.float32) - initial).mean())
    return {
        "mean_rgb_delta": mean,
        "large_change_pixel_fraction": changed_fraction,
        "terminal_rgb_delta": terminal_mean,
        "restored": mean <= .02 * 255. and changed_fraction <= .01
        and (terminal_mean <= 10. or mean <= .15 * terminal_mean),
    }


def audited_factory(**kwargs):
    arena = native_factory(**kwargs)
    reset_native, step_native, close_native = arena.reset, arena.step, arena.close
    run = {}

    def reset(scene, *, seed=None):
        obs = reset_native(scene, seed=seed)
        run.update(scene=scene, seed=seed, steps=0, initial=snapshot(arena, obs, "initial"),
                   early_actions=[], early_states={})
        return obs

    def step(action):
        result = step_native(action)
        run["steps"] += 1
        if run["steps"] <= max(REPLAY_STEPS):
            run["early_actions"].append(action)
        if run["steps"] in REPLAY_STEPS:
            run["early_states"][run["steps"]] = state(arena)
        if run["steps"] in FRAME_STEPS:
            for name in CAMERAS:
                rgb = array(arena._base.scene[name].data.output["rgb"])[0, ..., :3]
                np.save(AUDIT / f"step-{run['steps']:04d}_{name}_native.npy", rgb)
        return result

    def close():
        try:
            if not run.get("steps"):
                return
            terminal = snapshot(arena, arena._observation(), "terminal")
            output = os.environ.pop("MONITOR_TRACE", None)
            try:
                obs = reset_native(run["scene"], seed=run["seed"])
                repeated = snapshot(arena, obs, "repeated")
                replays = {}
                for count, action in enumerate(run["early_actions"], 1):
                    step_native(action)
                    if count in run["early_states"]:
                        replays[count] = errors(run["early_states"][count], state(arena))
            finally:
                if output is not None:
                    os.environ["MONITOR_TRACE"] = output
            initial = run["initial"]
            pixels = {name: camera_errors(initial[2][name], image, terminal[2][name])
                      for name, image in repeated[2].items()}
            reset_errors = errors(initial[0], repeated[0])
            checks = {
                "physical": all(np.isfinite(x) and x < 2e-4 for x in reset_errors.values()),
                "first_action": all(np.isfinite(x) and x < 2e-4 for x in replays[1].values()),
                "counters_and_timestamps": repeated[1] == initial[1] and repeated[1]["t"] == 0.
                and all(t == 0. for t in repeated[1]["camera_times"].values()),
                "camera_buffers": all(result["restored"] for result in pixels.values()),
            }
            (AUDIT / "report.json").write_text(json.dumps({
                "steps": run["steps"], "checks": checks, "reset_errors": reset_errors,
                "replay_errors": replays[1], "later_replay_errors": {k: v for k, v in replays.items() if k != 1},
                "pixel_reset_to_initial_and_terminal": pixels,
            }, indent=2) + "\n")
        finally:
            close_native()

    arena.reset, arena.step, arena.close = reset, step, close
    return arena


if __name__ == "__main__":
    AUDIT.mkdir()
    inspect_embodiment.yam_arena_embodiment = audited_factory
    sys.exit(main())
