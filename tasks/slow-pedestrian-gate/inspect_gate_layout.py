"""Inspect scene and fixed reset waypoints for the first robotic gate smoke."""

from dataclasses import replace
from math import pi, radians
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m
from closer_hardware import CAP_Z, SPEED_CENTRE, TIP_TO_GRIP_M, hex_driver, hydraulic_closer
from gate_latches import HOLDER_GRIP_CENTRE, HOLDER_PIVOT, holder_hook, closing_latch
from installed_gate import installed_gate
from yam_arena import ik, station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, GRIPPER, READY
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix


SCENE = "station"
INSTRUCTION = ("Slow the gate's main closing speed, release the hold-open hook, "
               "and let the gate swing into its latch on its own.")
DRIVER_RESET = tuple(json.loads((HERE / "driver-pickup-screen.json").read_text())[
    "best_route"]["driver_reset_tip_m"])
RIGHT_BASE = (0.0, -0.305, 0.875)
RIGHT_ROLL = pi / 2.0
LOOK = {"support": m.Look(color=m.rgb(.25, .34, .37), roughness=.61)}
GRIPPER["effort"] = 40.0

# Preserve the rear camera stand and remove the tabletop from the gate sweep.
station.TABLE_CENTER = (-.221, 0.0, station.TABLE_TOP - .030)
station.TABLE_SIZE = (.205, 1.20, .060)
station.BASES["right"] = RIGHT_BASE


if not getattr(station.top_camera, "gate_view", False):
    standard_camera = station.top_camera

    def gate_camera(variant="calibrated"):
        camera = standard_camera(variant)
        tilt = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), radians(10.0))
        orientation = quat_xyzw_to_matrix(camera.rot_xyzw) @ tilt
        return replace(camera, rot_xyzw=tuple(ik.matrix_to_quat_xyzw(orientation)))

    gate_camera.gate_view = True
    station.top_camera = gate_camera


def supports():
    floor = station.FLOOR_Z
    sizes = [
        (f"{side}_base_pedestal", (.174, .146, height - floor),
         (x, y, (height + floor) / 2))
        for side, (x, y, height) in station.BASES.items()
    ]
    x, y, bottom = DRIVER_RESET
    sizes.append(("rack_post", (.050, .052, bottom - floor),
                  (x, y, (bottom + floor) / 2)))
    outer, hole = .036, .0078
    rail = (outer - hole) / 2
    offset = (outer + hole) / 4
    for i, z in enumerate((.789, .826)):
        sizes.extend([
            (f"rack_{i}_left", (rail, outer, .008), (x - offset, y, z)),
            (f"rack_{i}_right", (rail, outer, .008), (x + offset, y, z)),
            (f"rack_{i}_front", (hole, rail, .008), (x, y - offset, z)),
            (f"rack_{i}_back", (hole, rail, .008), (x, y + offset, z)),
            (f"rack_{i}_back_bracket", (.018, .028, z - bottom),
             (x + .023, y, (z + bottom) / 2)),
        ])
    parts = [(name, m.box(size, center), "support") for name, size, center in sizes]
    collision = [m.col_box(name + "_shape", size, center).replace(
        "contactOffset = 0.004", "contactOffset = 0.00020")
        for name, size, center in sizes]
    return m.fixed("station_supports", (0.0, 0.0, 0.0), parts, collision, LOOK)


def sensed_gate():
    part = installed_gate()
    header = """    def Xform "gate" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]
    )
    {"""
    measured = header.replace('"PhysxRigidBodyAPI"]', '"PhysxRigidBodyAPI", "PhysxContactReportAPI"]')
    measured += "\n        float physxContactReport:threshold = 0"

    def stage_source():
        source = part.build()
        if source.count(header) != 1:
            raise ValueError("Could not set contact reporting on the moving gate leaf")
        return source.replace(header, measured)

    return replace(part, build=stage_source)


OBJECTS = [
    supports(),
    sensed_gate(),
    hydraulic_closer(),
    holder_hook(),
    closing_latch(),
    replace(hex_driver(), pos=DRIVER_RESET, contact_offset=.00012, torsional_patch=None),
]


class FixedRotation(float):
    """A waypoint attitude given directly as a world-frame rotation."""

    def __new__(cls, rotation):
        value = float.__new__(cls, 0.0)
        value.rotation = rotation
        return value


def demo():
    """Fixed calibrated motion for checking tool and hook interaction."""
    from yam_arena import demo as motion

    if not getattr(motion.grip_rotation, "accepts_fixed_rotation", False):
        plain = motion.grip_rotation

        def grip_rotation(yaw=0.):
            return yaw.rotation if isinstance(yaw, FixedRotation) else plain(yaw)

        grip_rotation.accepts_fixed_rotation = True
        motion.grip_rotation = grip_rotation
    Waypoint = motion.Waypoint

    x, y, rest = DRIVER_RESET
    tool = (x, y, rest + TIP_TO_GRIP_M)
    tool_hover = (x, y, tool[2] + .030)
    carry = (x, y, 1.065)
    valve_hover = (*SPEED_CENTRE[:2], 1.065)
    seated = (*SPEED_CENTRE[:2], CAP_Z + TIP_TO_GRIP_M - .0035)
    grasp = FINGER_CLOSED
    right = [
        Waypoint(1.6, tool_hover, FINGER_OPEN, "approach driver rack", RIGHT_ROLL),
        Waypoint(.7, tool, FINGER_OPEN, "line up on driver grip", RIGHT_ROLL),
        Waypoint(.7, tool, grasp, "grip the driver", RIGHT_ROLL),
        Waypoint(1.0, carry, grasp, "lift driver clear of both bushings", RIGHT_ROLL),
        Waypoint(1.4, valve_hover, grasp, "move above the regulating socket", RIGHT_ROLL),
        Waypoint(1.3, seated, grasp, "seat the 3 mm tip", RIGHT_ROLL),
        Waypoint(.4, seated, grasp, "settle the driver", RIGHT_ROLL),
    ]
    for degrees in range(30, 181, 30):
        right.append(Waypoint(.4, seated, grasp, "slow the main sweep",
                              RIGHT_ROLL - radians(degrees)))
    right.extend([
        Waypoint(1.0, valve_hover, grasp, "withdraw the driver", -RIGHT_ROLL),
        Waypoint(1.4, carry, grasp, "hold the withdrawn driver clear", -RIGHT_ROLL),
    ])

    pivot = np.asarray(HOLDER_PIVOT)
    offset = np.asarray(HOLDER_GRIP_CENTRE) - pivot
    lift_axis = np.array([0., 1., 0.])
    hook_grasp = tuple(pivot + axis_angle_matrix(lift_axis, radians(-3.4)) @ offset)
    hook_lift = tuple(pivot + axis_angle_matrix(lift_axis, radians(-24.)) @ offset)
    hx, hy, hz = hook_grasp
    left_hover = (hx, hy, hz + .030)
    clear = (hx - .015, hy, hz + .074)
    _, ready_rotation = ik.tcp_pose(READY, station.BASES["left"])
    crossing = (.195, .255, 1.013)
    approach_height = (*crossing[:2], .925)
    ready = FixedRotation(ready_rotation)
    cross = Waypoint(1.3, crossing, FINGER_OPEN, "cross above the top rail", ready)
    descend = [
        Waypoint(.8, approach_height, FINGER_OPEN, "lower on the post side", ready),
        Waypoint(.9, left_hover, FINGER_OPEN, "turn toward the hook grip", 0.0),
    ]
    waiting = sum(w.duration for w in right) - cross.duration - sum(w.duration for w in descend)
    left = [
        cross,
        Waypoint(waiting, crossing, FINGER_OPEN, "wait above the rail for adjustment", ready),
        *descend,
        Waypoint(.8, hook_grasp, FINGER_OPEN, "surround the flat-sided hook grip", 0.0),
        Waypoint(.65, hook_grasp, FINGER_CLOSED, "grip the holdback", 0.0),
        Waypoint(.95, hook_lift, FINGER_CLOSED, "lift the loaded hook away from the keeper", 0.0),
        Waypoint(.85, hook_lift, FINGER_CLOSED, "hold the hook until the gate clears", 0.0),
        Waypoint(.9, clear, FINGER_OPEN, "release the hook and clear the swing", 0.0),
        Waypoint(9.5, clear, FINGER_OPEN, "observe passive gate swing", 0.0),
    ]
    return {"left": left, "right": right}
