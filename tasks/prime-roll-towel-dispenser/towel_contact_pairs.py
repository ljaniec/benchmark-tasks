"""Private PhysX collision-pair audit for the towel development trials."""

from collections import defaultdict
from dataclasses import replace
from functools import lru_cache
import json
import math
import os
from pathlib import Path
import re


def _with_contact_api(text):
    text, n = re.subn(
        r'(prepend apiSchemas = \["PhysicsRigidBodyAPI", "PhysicsMassAPI"[^\]\n]*)\]',
        r'\1, "PhysxContactReportAPI"]',
        text,
    )
    text, attributes = re.subn(
        r'(?m)^(        float physics:mass =)',
        r'        float physxContactReport:threshold = 0.002\n\1',
        text,
    )
    if not n or n != attributes:
        raise ValueError("Could not match each rigid body to its contact report schema")
    return text


def instrument(mechkit):
    native_mechanism = mechkit.mechanism

    def mechanism(*args, **kwargs):
        part = native_mechanism(*args, **kwargs)
        return replace(part, build=lambda: _with_contact_api(part.build()))

    mechkit.mechanism = mechanism
    pairs = defaultdict(float)
    event_tick = 0
    report = None
    observer = None
    step = 0

    @mechkit.on_reset
    def start(_env):
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools

        nonlocal event_tick, observer, report, step
        pairs.clear()
        event_tick = step = 0
        report = Path(os.environ["TOWEL_TRACE"]).with_name("contacts.jsonl")
        report.write_text("")

        @lru_cache(maxsize=1024)
        def path(handle):
            return str(PhysicsSchemaTools.intToSdfPath(handle))

        def collect(headers, data):
            nonlocal event_tick
            event_tick += 1
            for header in headers:
                if not header.num_contact_data:
                    continue
                actors = (path(header.actor0), path(header.actor1))
                colliders = (path(header.collider0), path(header.collider1))
                pair = tuple(sorted(zip(actors, colliders, strict=True)))
                start_index = header.contact_data_offset
                for i in range(start_index, start_index + header.num_contact_data):
                    pairs[pair] += math.hypot(*data[i].impulse)

        observer = get_physx_simulation_interface().subscribe_contact_report_events(collect)

    @mechkit.on_step
    def save(_env):
        nonlocal event_tick, step
        records = sorted(pairs.items(), key=lambda pair: -pair[1])
        if report is not None:
            with report.open("a") as out:
                out.write(json.dumps({"step": step, "event_ticks": event_tick, "pairs": records}) + "\n")
        pairs.clear()
        event_tick = 0
        step += 1
