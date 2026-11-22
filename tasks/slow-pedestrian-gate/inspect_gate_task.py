"""Adjust the modeled valve and observe an actual passive gate closure."""

import json
from math import cos, degrees, hypot, pi, radians
import os
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from inspect_gate_layout import INSTRUCTION, OBJECTS, SCENE, demo, m
from closer_hardware import SPEED_CENTRE, TIP_TO_GRIP_M
from closing_model import hydraulic_torque, MAX_VALVE_ADJUSTMENT_TURNS
from gate_latches import HOLDER_GRIP_CENTRE, HOLDER_PIVOT, HOLDER_RELEASE_DEG
from scorer import ClosingScore


RECREATE_SCENE_ON_RESET = True
SUCCESS_HOLD_S = 0.05
SCORE_OVER = "end"
CONTACT_THRESHOLD_NS = 0.00025
TOOL_LOW = (-.25, -.65, .65)
TOOL_HIGH = (1.25, .65, 1.245)
score_value = 0.0


class GateTrial:
    """Apply closer torque at physics frequency and track the free return."""

    def __init__(self, base):
        import warp as wp
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysxSchema, PhysicsSchemaTools
        from isaaclab.sim.utils.stage import get_current_stage

        self.base = base
        self.original_step = base.sim.step
        self.gate, self.gate_joint = m._joint(base, "pedestrian_gate", "gate")
        self.speed, self.speed_joint = m._joint(base, "hydraulic_closer", "speed")
        self.holder, self.hook_joint = m._joint(base, "gate_holder", "hook")
        self.latch, self.latch_joint = m._joint(base, "gate_latch", "pawl")
        count = len(self.gate.joint_names)
        self.efforts = np.zeros((1, count), dtype=np.float32)
        self.wrench = wp.zeros((1, count), dtype=wp.float32, device=base.device)
        self.index = wp.array([0], dtype=wp.int32, device=base.device)
        self.gate_position = 0.0
        self.gate_speed = 0.0
        self.swing_contact = False
        self.peak_pawl_deg = 0.0
        self.counter = ClosingScore()
        self.contact_paths = {}

        stage = get_current_stage()
        prim = stage.GetPrimAtPath(f"{m.ENV_PATH}/pedestrian_gate/gate")
        if not prim.IsValid():
            raise RuntimeError("Gate contact witness is missing")
        if not prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
            raise RuntimeError("Gate leaf must report contact")
        self.gate_body_path = str(prim.GetPath())
        self.resolve_path = PhysicsSchemaTools.intToSdfPath
        self.listener = get_physx_simulation_interface().subscribe_contact_report_events(self.record_contact)

    def contact_path(self, handle):
        if handle not in self.contact_paths:
            self.contact_paths[handle] = str(self.resolve_path(handle))
        return self.contact_paths[handle]

    def record_contact(self, headers, data):
        for header in headers:
            if not header.num_contact_data:
                continue
            bodies = [self.contact_path(header.actor0), self.contact_path(header.actor1)]
            if self.gate_body_path not in bodies:
                continue
            other = bodies[1] if bodies[0] == self.gate_body_path else bodies[0]
            if not any(f"/{name}/" in other + "/" for name in ("left_robot", "right_robot", "hex_driver")):
                continue
            start = header.contact_data_offset
            impulse = sum(hypot(*data[i].impulse)
                          for i in range(start, start + header.num_contact_data))
            self.swing_contact |= impulse >= CONTACT_THRESHOLD_NS

    @staticmethod
    def sample(asset, joint, field):
        data = getattr(asset.data, field)
        return float(getattr(data, "torch", data)[0, joint])

    def step(self, *args, **kwargs):
        self.gate_position = self.sample(self.gate, self.gate_joint, "joint_pos")
        self.gate_speed = self.sample(self.gate, self.gate_joint, "joint_vel")
        valve = self.sample(self.speed, self.speed_joint, "joint_pos")
        setting = float(np.clip(-valve / (2 * pi), 0.0, MAX_VALVE_ADJUSTMENT_TURNS))
        self.efforts[0, self.gate_joint] = np.clip(
            hydraulic_torque(self.gate_position, self.gate_speed, setting), -16.0, 16.0
        )
        self.wrench.assign(self.efforts)
        self.gate.root_view.set_dof_actuation_forces(self.wrench, self.index)
        return self.original_step(*args, **kwargs)

    def read_score(self):
        angle = degrees(self.sample(self.gate, self.gate_joint, "joint_pos"))
        hook = degrees(self.sample(self.holder, self.hook_joint, "joint_pos"))
        pawl = degrees(self.sample(self.latch, self.latch_joint, "joint_pos"))
        if hook > HOLDER_RELEASE_DEG and angle > 89.5:
            self.peak_pawl_deg = 0.0
        else:
            self.peak_pawl_deg = max(self.peak_pawl_deg, pawl)
        upright = True
        for side in ("left", "right"):
            data = self.base.scene[f"{side}_robot"].data.root_link_pose_w
            pose = getattr(data, "torch", data)[0, 3:7].cpu().numpy()
            upright &= bool(m.quat_matrix(pose)[2, 2] >= cos(radians(15)))
        tool = self.base.scene["hex_driver"]
        tool_pose = m._np(tool.data.root_link_pose_w)[0]
        tip = tool_pose[:3]
        tool_inside = bool((np.asarray(TOOL_LOW) <= tip).all() and (tip <= TOOL_HIGH).all())
        score = self.counter.tick(
            angle_deg=angle,
            holder_engaged=hook > HOLDER_RELEASE_DEG and 89.5 <= angle <= 90.5,
            sweep_clear=not self.swing_contact,
            latch_engaged=pawl < 2.0 and self.peak_pawl_deg >= 5.0,
            fixture_intact=upright and tool_inside and -.5 <= angle <= 92.5,
        )
        trace_path = os.getenv("GATE_TRACE")
        if trace_path:
            from yam_arena.runtime import tcp_world
            from yam_arena.urdf import axis_angle_matrix

            valve_deg = degrees(self.sample(self.speed, self.speed_joint, "joint_pos"))
            shaft = m.quat_matrix(tool_pose[3:7])[:, 2]
            grip = tip + TIP_TO_GRIP_M * shaft
            right_tcp = np.asarray(tcp_world(self.base.scene["right_robot"]))
            left_tcp = np.asarray(tcp_world(self.base.scene["left_robot"]))
            tip_offset = tip - np.asarray(SPEED_CENTRE)
            pivot = np.asarray(HOLDER_PIVOT)
            hook_grip = pivot + axis_angle_matrix(np.array([0., 1., 0.]), radians(hook)) @ (
                np.asarray(HOLDER_GRIP_CENTRE) - pivot
            )
            data = {"step": self.counter.step, "gate_deg": angle, "hook_deg": hook,
                    "pawl_deg": pawl, "assisted": self.swing_contact, "score": score,
                    "speed_valve_deg": valve_deg, "gate_speed_rad_s": self.gate_speed,
                    "driver_tip_m": tip.tolist(), "driver_insertion_mm": -1000. * tip_offset[2],
                    "driver_lateral_error_mm": 1000. * float(np.linalg.norm(tip_offset[:2])),
                    "driver_axis_deg": degrees(np.arccos(np.clip(shaft[2], -1., 1.))),
                    "driver_grip_offset_mm": 1000. * float(np.linalg.norm(grip - right_tcp)),
                    "hook_grip_offset_mm": 1000. * float(np.linalg.norm(hook_grip - left_tcp)),
                    "driver_inside": tool_inside, "invalid": self.counter.invalid}
            with Path(trace_path).open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(data, allow_nan=False) + "\n")
        self.swing_contact = False
        return score


@m.on_before_reset
def remove_gate_runtime(env):
    base = env.unwrapped
    if hasattr(base, "gate_trial"):
        base.sim.step = base.gate_trial.original_step
        base.gate_trial.listener = None
        del base.gate_trial


@m.on_reset
def begin_gate_trial(env):
    global score_value
    score_value = 0.0
    base = env.unwrapped
    # Write the authored native start after settling, then refresh the
    # cameras before returning the first policy observation.
    for name in ("pedestrian_gate", "hydraulic_closer", "gate_holder", "gate_latch"):
        asset = base.scene[name]
        position = asset.data.default_joint_pos.torch.clone()
        velocity = asset.data.default_joint_vel.torch.clone()
        asset.write_joint_state_to_sim_index(position=position, velocity=velocity)
    base.sim.forward()
    for _ in range(3):
        base.sim.render()
    for camera in ("top_cam", "left_cam", "right_cam"):
        base.scene[camera].update(0., force_recompute=True)
    base.gate_trial = GateTrial(base)
    trial = base.gate_trial
    angle = degrees(trial.sample(trial.gate, trial.gate_joint, "joint_pos"))
    if abs(angle - 90.0) > .5:
        raise RuntimeError(f"Expected a gate starting at 90 degrees, got {angle:.3f}")
    trace_path = os.getenv("GATE_TRACE")
    if trace_path:
        Path(trace_path).write_text("", encoding="utf-8")
    base.sim.step = base.gate_trial.step


@m.on_step
def read_gate_trial(env):
    global score_value
    score_value = env.unwrapped.gate_trial.read_score()


def score(_objects):
    return score_value


def success(_objects):
    return score_value == 1.0
