"""Record the physical scene and the exact gate camera inputs during a smoke run."""

from dataclasses import asdict
import json
import os
from pathlib import Path
import sys

import numpy as np
from inspect_robots.cli import main
from yam_arena import inspect_embodiment


FRAMES = (1, 45, 79, 110, 141, 168, 186, 219, 237, 265, 310, 355, 410, 510)
CAMERAS = ("top_cam", "left_cam", "right_cam")
BODIES = ("left_robot", "right_robot", "pedestrian_gate", "hydraulic_closer",
          "gate_holder", "gate_latch", "hex_driver")
OUTPUT = Path(os.environ["GATE_AUDIT_DIR"])
OUTPUT.mkdir(parents=True, exist_ok=False)
original_factory = inspect_embodiment.yam_arena_embodiment
contrast = {}


def array(value):
    return getattr(value, "torch", value).detach().cpu().numpy().copy()


def snapshot(arena, obs, label):
    assert set(obs.images) == set(CAMERAS)
    frames = {}
    for name in CAMERAS:
        raw = array(arena._base.scene[name].data.output["rgb"])[0, ..., :3]
        delivered = obs.images[name].copy()
        assert raw.shape == (360, 640, 3), (label, name, raw.shape)
        assert delivered.shape == (176, 320, 3) and delivered.dtype == np.uint8, (label, name, delivered.shape, delivered.dtype)
        contrast[f"{label}/{name}"] = float(delivered.std())
        for kind, image in (("native", raw), ("delivered", delivered)):
            frames[f"{name}/{kind}"] = image
            np.save(OUTPUT / f"{label}-{name}-{kind}.npy", image)
    return frames


def physics_state(arena):
    state = {}
    for name in BODIES:
        data = arena._base.scene[name].data
        fields = ["root_link_pose_w", "root_vel_w"]
        if name != "hex_driver":
            fields += ["joint_pos", "joint_vel"]
        for field in fields:
            state[f"{name}/{field}"] = array(getattr(data, field))
    return state


def task_state(arena):
    trial = arena._base.gate_trial
    return asdict(trial.counter), trial.swing_contact, trial.peak_pawl_deg, arena.task.module.score_value


def instrument(**kwargs):
    arena = original_factory(**kwargs)
    original_reset, original_step, original_close = arena.reset, arena.step, arena.close
    episode = {}
    steps = 0

    def reset(scene, *, seed=None):
        nonlocal steps
        obs = original_reset(scene, seed=seed)
        steps = 0
        episode.update(scene=scene, seed=seed, state=physics_state(arena),
                       counters=task_state(arena),
                       images=snapshot(arena, obs, "initial"),
                       times=(obs.state_time, dict(obs.image_times)))
        return obs

    def step(action):
        nonlocal steps
        result = original_step(action)
        steps += 1
        if steps in FRAMES:
            snapshot(arena, result.observation, f"step-{steps:04d}")
        return result

    def close():
        if not episode or steps == 0:
            original_close()
            return
        terminal = arena._observation()
        snapshot(arena, terminal, "terminal")
        saved_trace = os.environ.pop("GATE_TRACE", None)
        try:
            obs = original_reset(episode["scene"], seed=episode["seed"])
            repeated = physics_state(arena)
            images = snapshot(arena, obs, "repeated")
            drift = {name: float(np.max(np.abs(value - repeated[name])))
                     for name, value in episode["state"].items()}
            image_delta = {key: float(np.mean(np.abs(image.astype(float) - images[key])))
                           for key, image in episode["images"].items()}
            result = {
                "steps": steps,
                "max_physical_delta": drift,
                "mean_rgb_delta": image_delta,
                "timestamps_repeat": (obs.state_time, dict(obs.image_times)) == episode["times"],
                "counters_repeat": task_state(arena) == episode["counters"],
                "cameras_refreshed": all(value < 3. for value in image_delta.values()),
                "physics_repeats": all(np.isfinite(value) and value < 2e-4 for value in drift.values()),
                "low_contrast_samples": {name: value for name, value in contrast.items() if value < 2.},
                "delivered_rgb_std": contrast,
            }
            (OUTPUT / "reset.json").write_text(json.dumps(result, indent=2) + "\n")
        finally:
            if saved_trace is not None:
                os.environ["GATE_TRACE"] = saved_trace
            original_close()

    arena.reset, arena.step, arena.close = reset, step, close
    return arena


inspect_embodiment.yam_arena_embodiment = instrument
sys.exit(main())
