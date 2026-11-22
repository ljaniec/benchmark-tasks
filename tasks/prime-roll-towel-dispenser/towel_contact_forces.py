"""Private audit using the installed Isaac Lab PhysX contact tensor view."""

from dataclasses import replace
import json
import os
from pathlib import Path

import numpy as np

from towel_contact_pairs import _with_contact_api


def instrument(mechkit):
    native = mechkit.mechanism

    def mechanism(*args, **kwargs):
        part = native(*args, **kwargs)
        return replace(part, build=lambda: _with_contact_api(part.build()))

    mechkit.mechanism = mechanism
    views = {}
    report = None
    step = 0

    @mechkit.on_reset
    def start(env):
        from isaaclab_physx.physics import PhysxManager

        nonlocal report, step
        views.clear()
        step = 0
        trace = os.environ.get("TOWEL_TRACE")
        report = Path(trace).with_name("contact-forces.jsonl") if trace else None
        if report is None:
            return
        sim_view = PhysxManager.get_physics_sim_view()

        def bodies(name):
            return list(env.unwrapped.scene[name].root_view.link_paths[0])

        left = bodies("left_robot")
        right = bodies("right_robot")
        fixture = bodies("dispenser")
        rollers = bodies("feed_rollers")
        web_group = bodies("towel_web")
        wrap = [p for p in web_group if p.rsplit("/", 1)[-1] == "roll"]
        paper = [p for p in web_group if p.rsplit("/", 1)[-1].startswith("web_")]
        pairs = {"fixture_on_hand": (fixture + rollers, left),
                 "loading_arm_on_right": ([p for p in rollers if p.endswith("/loading_arm")], right),
                 "housing_on_right": (fixture, right),
                 "idler_on_housing": ([p for p in rollers if p.endswith("/idler")], fixture),
                 "paper_on_hand": (paper + wrap, left),
                 "housing_on_paper": (paper, fixture),
                 "roll_on_paper": (paper, wrap),
                 "rollers_on_paper": (paper, rollers)}
        for name, (sensors, filters) in pairs.items():
            if not sensors or not filters:
                raise ValueError(f"Missing sensor or filter prims for {name}: {sensors}, {filters}")
            view = sim_view.create_rigid_contact_view(sensors, filter_patterns=[filters] * len(sensors),
                                                      max_contact_data_count=32 * len(sensors))
            if not view.filter_count:
                raise ValueError(f"No contact filters matched for {name}")
            views[name] = sensors, filters, view
        report.write_text("")

    @mechkit.on_step
    def save(env):
        nonlocal step
        if report is None:
            return
        dt = float(env.unwrapped.cfg.sim.dt)

        def array(tensor):
            if hasattr(tensor, "detach"):
                tensor = tensor.detach().cpu()
            return tensor.numpy()

        results = {}
        for name, (sensors, filters, view) in views.items():
            magnitudes = np.linalg.norm(array(view.get_contact_force_matrix(dt=dt)), axis=-1)
            if magnitudes.shape[0] != len(sensors):
                raise ValueError(f"Unexpected body dimensions for {name}: {magnitudes.shape}")
            points = counts = starts = None
            if name == "fixture_on_hand" and np.any(magnitudes >= .05):
                _, raw_points, _, _, raw_counts, raw_starts = view.get_contact_data(dt=dt)
                points, counts, starts = map(array, (raw_points, raw_counts, raw_starts))
            records = []
            for i, j in np.argwhere(magnitudes >= .05):
                counterparty = filters[j] if magnitudes.shape[1] == len(filters) else str(j)
                point = None
                if points is not None and counts[i, j]:
                    point = points[starts[i, j]].tolist()
                records.append((sensors[i].rsplit("/", 1)[-1], counterparty.rsplit("/", 1)[-1],
                                float(magnitudes[i, j]), point))
            results[name] = sorted(records, key=lambda row: -row[2])[:10]
        with report.open("a") as out:
            out.write(json.dumps({"step": step, "forces_n": results}) + "\n")
        step += 1
