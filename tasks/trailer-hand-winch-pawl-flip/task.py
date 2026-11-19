"""On the station table: a Fulton-scale trailer hand winch bolted to a steel riser, a yellow 50 mm strap under
preload pulling an orange bow-load sled on a rail toward a green chock, a red spinning crank sleeve, and a blue
reversing ratchet pawl that must be flipped into a magenta payout pocket while the crank is counter-held, then
eased so a black strap stripe stops between flanking red tabs.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/trailer-hand-winch-pawl-flip/task.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Hold the red winch crank with one hand, flip the blue ratchet pawl into the magenta pocket with the other, then turn the crank to let the strap out until the black strap mark sits between the red tabs, without touching or coming within 5 mm of the orange sled or yellow strap, and finish with both hands clear of the winch." \\
           --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import demokit  # noqa: E402,F401
from arm_watch import on_reset, on_step  # noqa: E402
from mechkit import (  # noqa: E402
    Joint,
    Mech,
    MechState,
    _quat_wxyz_matrix,
    box,
    clear_joint_targets,
    cyl,
    get_joint_target,
    on_reset as mech_on_reset,
    set_joint_target,
)
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.05  # one step: the 1.0 s clear-rest hold (20 steps) is counted inside score()

INSTRUCTION = (
    "Hold the red winch crank with one hand, flip the blue ratchet pawl into the magenta pocket with the other, "
    "then turn the crank to let the strap out until the black strap mark sits between the red tabs, without "
    "touching or coming within 5 mm of the orange sled or yellow strap, and finish with both hands clear of the "
    "winch."
)

# - colours ---------------------------------------------------------------------------------------------------------
STEEL = (0.22, 0.24, 0.27)
GALV = (0.76, 0.78, 0.80)
DRUM_Y = (0.95, 0.80, 0.08)
GEAR = (0.35, 0.36, 0.38)
RED = (0.90, 0.12, 0.12)
TAB_RED = (0.92, 0.10, 0.10)
BLUE = (0.12, 0.38, 0.92)
GREEN = (0.10, 0.78, 0.22)
MAGENTA = (0.85, 0.10, 0.55)
YELLOW = (0.95, 0.82, 0.10)
BLACK = (0.05, 0.05, 0.05)
ORANGE = (0.95, 0.42, 0.08)
ALUM = (0.70, 0.72, 0.75)

EPS = 1e-9
DT = 0.05
HOLD_STEPS = 20
GEAR_RATIO = 4.1
R_CRANK = 0.178
R_DEMO = R_CRANK
R_EFF = 0.048
GRIP_GAP_MIN = 0.020
GRIP_GAP_MAX = 0.0275  # 26 mm sleeve: blocks closing on air at 28 to 30 mm
GRIP_AXIS_TOL = 0.022
PRELOAD_TORQUE = 0.30  # N m on the crank toward payout while counter-held (true 0.53, reduced for YAM wrist)
PINION_DRIVE_K = 0.5  # N m/rad drive used only to apply the preload torque
PAD_HALF_WIDTH = 0.020
PAD_THICK = 0.010
PAD_BACK_FROM_TCP = 0.021
PAD_TIP_PAST_TCP = 0.013

PINION = np.array([0.2150, 0.0000, 0.2075])
DRUM = np.array([0.2600, 0.0000, 0.1275])
PAWL_HINGE = np.array([0.1920, -0.0560, 0.2445])
SLEEVE_START = (0.3514, 0.1050, 0.3219)
SLEEVE_Y = 0.1050
SLEEVE_OD = 0.026
GRIP_Y = 0.098  # demo grips 7 mm inboard of the sleeve centre (pads 40 mm wide on a 70 mm sleeve): keeps the forearm off the +y cheek
SLEEVE_LEN = 0.070
PADDLE_UP = (0.1920, -0.0560, 0.2725)
PADDLE_DOWN = (0.1748, -0.0560, 0.2666)
GREEN_MOUTH = (0.1920, -0.0440, 0.2725)
MAGENTA_MOUTH = (0.1748, -0.0440, 0.2666)
STRIPE_START_X = 0.3400
STRIPE_TARGET_X = 0.3650
STRIPE_H = 0.0795
SLED_START_X = 0.4550
SLED_REST_X = 0.4800
SLED_H = 0.0800
DRUM_POINTER = (0.2600, 0.0360, 0.1885)


def _wz(h: float) -> float:
    return TOP + h


def _ry(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _left_grip(crank_deg: float) -> np.ndarray:
    t = float(np.clip((40.0 - crank_deg) / (40.0 - (-32.0)), 0.0, 1.0))
    pitch = -45.0 * (1.0 - t)
    return _ry(pitch) @ grip_rotation(-math.pi / 2)


def _right_grip() -> np.ndarray:
    return _ry(-45.0) @ grip_rotation(0.0)


def _box(name, xyz_h, size, color, mass=None):
    x, y, h = xyz_h
    return Box(name, pos=(x, y, _wz(h)), size=size, color=color, mass=mass)


def _cyl(name, xyz_h, radius, height, color, mass=None):
    x, y, h = xyz_h
    return Cylinder(name, pos=(x, y, _wz(h)), radius=radius, height=height, color=color, mass=mass)


def _crank_local(deg: float, r: float = R_CRANK):
    rad = math.radians(deg)
    return (r * math.cos(rad), SLEEVE_Y, r * math.sin(rad))


# Mech frame origin at table (0,0,TOP); joint.at uses table (x,y,h).
# Pinion joint 0 = crank at +40 deg (start); positive q_y sweeps CW from +40 toward -82.3 deg.
# Drum joint 0 = start spoke; Pawl joint 0 = green UP; Sled joint 0 = rest at x=0.480 (start -0.025).
_sx40, _sy40, _sh40 = _crank_local(40.0)
# -90 deg about X (wxyz): native cyl height (+Z) maps to +y along the crank/drum/pinion axis.
_TILT_PLUS_Y = (math.cos(math.pi / 4.0), -math.sin(math.pi / 4.0), 0.0, 0.0)
# -40 deg about Y (wxyz): crank bar (+X) tilts +40 deg up into the x-h plane along the start radius.
_TILT_CRANK_ARM = (math.cos(math.radians(-20.0)), 0.0, math.sin(math.radians(-20.0)), 0.0)
# +30 deg about Y (wxyz): +30 deg spoke (+Z) tilts +30 deg toward +X on the +y drum flange.
_TILT_SPOKE30 = (math.cos(math.radians(15.0)), 0.0, math.sin(math.radians(15.0)), 0.0)
_spoke0_local = (0.0, 0.0310, 0.0370)  # radial span 0.017..0.057 m flush on +y yellow flange face
_spoke30_local = (
    0.0370 * math.sin(math.radians(30.0)),
    0.0310,
    0.0370 * math.cos(math.radians(30.0)),
)
STRAP_H = 0.0750  # top at 0.0765 so free stripe bottom (0.0795 - 0.0030 = 0.0765) rests cleanly on top

WINCH = Mech(
    "winch",
    pos=(0.0, 0.0, TOP),
    mass=None,
    friction=1.0,
    shapes=[
        box((0.3600, 0.0000, 0.0050), (0.460, 0.180, 0.010), STEEL),
        # Riser plinth + rear column + cheek lower gussets clear of drum, drum gear, strap, and crank arm.
        box((0.2350, 0.0000, 0.0300), (0.130, 0.080, 0.040), GALV),
        box((0.1750, 0.0000, 0.0875), (0.020, 0.080, 0.075), GALV),
        box((0.2400, 0.0360, 0.0625), (0.120, 0.008, 0.025), GALV),
        box((0.2400, -0.0360, 0.0625), (0.120, 0.008, 0.025), GALV),
        box((0.2400, 0.0360, 0.1600), (0.120, 0.008, 0.170), GALV),
        box((0.2400, -0.0360, 0.1600), (0.120, 0.008, 0.170), GALV),
        box(DRUM_POINTER, (0.012, 0.012, 0.020), RED, collide=False),
        # Dark bezel plate behind both windows, then 14 x 4 x 14 mm coloured windows (readable at 320x176).
        box(((GREEN_MOUTH[0] + MAGENTA_MOUTH[0]) / 2, -0.0430, (GREEN_MOUTH[2] + MAGENTA_MOUTH[2]) / 2), (0.040, 0.002, 0.030), BLACK, collide=False),
        box(GREEN_MOUTH, (0.014, 0.004, 0.014), GREEN, collide=False),
        box(MAGENTA_MOUTH, (0.014, 0.004, 0.014), MAGENTA, collide=False),
        box((0.3650, 0.0380, STRIPE_H), (0.008, 0.020, 0.010), TAB_RED),
        box((0.3650, -0.0380, STRIPE_H), (0.008, 0.020, 0.010), TAB_RED),
        box((0.4700, 0.0000, 0.0585), (0.240, 0.030, 0.013), ALUM),
        box((0.5300, 0.0000, 0.0800), (0.020, 0.070, 0.035), GREEN),
        # yellow strap span (supports free stripe at h=0.0795 without overlap)
        box((0.3375, 0.0000, STRAP_H), (0.155, 0.050, 0.003), YELLOW),
        # axles through the cheeks (joint anchors lie inside the fixed frame), pawl pivot lug, rail carriage key
        cyl((PINION[0], -0.0200, PINION[2]), 0.006, 0.060, GEAR, tilt=_TILT_PLUS_Y),
        cyl((DRUM[0], 0.0000, DRUM[2]), 0.006, 0.080, GEAR, tilt=_TILT_PLUS_Y),
        box((PAWL_HINGE[0], -0.0510, PAWL_HINGE[2]), (0.010, 0.022, 0.008), GALV),
        box((0.5000, 0.0000, 0.0750), (0.120, 0.012, 0.020), ALUM),
        # ratchet wheel collider on -y cheek (greased steel friction via joint)
        cyl((PINION[0], -0.0460, PINION[2]), 0.0290, 0.008, GEAR, tilt=_TILT_PLUS_Y),
    ],
    joints=[
        Joint(
            "pinion",
            "hinge",
            axis="y",
            at=(float(PINION[0]), float(PINION[1]), float(PINION[2])),
            low=math.radians(-40.0),
            high=math.radians(220.0),
            spring=PINION_DRIVE_K,
            damping=0.012,
            mass=0.280,
            friction=None,
            shapes=[
                cyl((0.0, 0.0, 0.0), 0.018, 0.010, GEAR, tilt=_TILT_PLUS_Y),
                # crank arm bar along +40 deg start radius (centre halfway to sleeve)
                box((_sx40 / 2, 0.0550, _sh40 / 2), (0.200, 0.018, 0.010), GEAR, tilt=_TILT_CRANK_ARM),
                # red sleeve along +y at start centre (free-spin bearing approximated as fixed to crank)
                cyl((_sx40, _sy40, _sh40), SLEEVE_OD / 2, SLEEVE_LEN, RED, tilt=_TILT_PLUS_Y),
            ],
        ),
        Joint(
            "drum",
            "hinge",
            axis="y",
            at=(float(DRUM[0]), float(DRUM[1]), float(DRUM[2])),
            low=math.radians(-180.0),
            high=math.radians(180.0),
            spring=0.0,
            damping=0.025,
            mass=0.650,
            friction=0.02,
            shapes=[
                cyl((0.0, 0.0, 0.0), 0.048, 0.056, DRUM_Y, tilt=_TILT_PLUS_Y),
                cyl((0.0, -0.0290, 0.0), 0.057, 0.002, DRUM_Y, tilt=_TILT_PLUS_Y),
                cyl((0.0, 0.0290, 0.0), 0.057, 0.002, DRUM_Y, tilt=_TILT_PLUS_Y),
                cyl((0.0, 0.0240, 0.0), 0.0738, 0.010, GEAR, tilt=_TILT_PLUS_Y, collide=False),
                box(_spoke0_local, (0.014, 0.004, 0.040), BLACK, collide=False),
                box(_spoke30_local, (0.014, 0.004, 0.040), BLACK, tilt=_TILT_SPOKE30, collide=False),
            ],
        ),
        Joint(
            "pawl",
            "hinge",
            axis="y",
            # q_y stops surround 0; scored theta_pawl = -q_y in [-5, +40] deg so q_y in [-40, +5] deg.
            # The magenta pocket floor is the +40 deg stop, inside the 33-43 deg seated band, so a pawl
            # pushed fully home rests seated (GPU run 1: a +45 deg stop parked it outside the band).
            at=(float(PAWL_HINGE[0]), float(PAWL_HINGE[1]), float(PAWL_HINGE[2])),
            low=math.radians(-40.0),
            high=math.radians(5.0),
            # Spring toward rest 0 only below the ridge; on_step retargets to magenta when past 20 deg.
            spring=0.04,
            damping=0.008,
            mass=0.055,
            friction=None,
            shapes=[
                box((0.0, 0.0, 0.0280), (0.014, 0.012, 0.052), BLUE),
                cyl((0.0, 0.0, 0.0), 0.004, 0.012, BLUE, tilt=_TILT_PLUS_Y),
            ],
        ),
        Joint(
            "sled",
            "slide",
            axis="x",
            # Joint 0 = spring rest at x=0.480; start pretension is written to -0.025 m on reset.
            at=(SLED_REST_X, 0.0, SLED_H),
            low=-0.030,
            high=0.0,
            spring=1804.0,
            damping=10.0,
            mass=0.850,
            shapes=[box((0.0, 0.0, 0.0), (0.080, 0.056, 0.030), ORANGE)],
        ),
    ],
)

# Free stripe body (mass > 0); kinematically coupled to drum payout in on_step / couple_kinematics.
# Winch Mech owns drum/pinion/pawl/sled/crank/sleeve/spokes; do not also spawn free proxies (double collision).
STRIPE = _box("stripe", (STRIPE_START_X, 0.0000, STRIPE_H), (0.014, 0.052, 0.006), BLACK, mass=0.005)

OBJECTS = [WINCH, STRIPE]

# - episode latch state ---------------------------------------------------------------------------------------------
_S: dict = {
    "objects": None,
    "L_magenta": False,
    "L_050": False,  # prior 0.50 (magenta under hold at start spoke/stripe band)
    "L_payout": False,
    "violations": set(),
    "holding_arm": None,
    "held_before_pawl10": False,
    "hold_broken_after_pawl10": False,
    "prev_drum_deg": 0.0,
    "prev_sled_x": SLED_START_X,
    "braced_sled_x": SLED_START_X,
    "unbraced_cum": 0.0,
    "arms": None,
    "omega_pinion": None,  # rad/s of the pinion, informational only (not scored)
    "clear_steps": 0,
    "joint_vel": None,  # {pinion,drum,pawl,sled} rates from live Mech; None offline unless injected
    "step": 0,
}

_LIVE = {
    "sled": np.array([SLED_START_X, 0.0, _wz(SLED_H)]),
    "stripe": np.array([STRIPE_START_X, 0.0, _wz(STRIPE_H)]),
    "strap": np.array([0.3375, 0.0, _wz(STRAP_H)]),
}
_SLEEVE_XH = {"x": SLEEVE_START[0], "h": SLEEVE_START[2]}
_HANDS = {"poses": None, "finger": None, "step": 0}


def _reset_episode(objects=None) -> None:
    _S.update(
        objects=objects,
        L_magenta=False,
        L_050=False,
        L_payout=False,
        violations=set(),
        holding_arm=None,
        held_before_pawl10=False,
        hold_broken_after_pawl10=False,
        prev_drum_deg=0.0,
        prev_sled_x=SLED_START_X,
        braced_sled_x=SLED_START_X,
        unbraced_cum=0.0,
        omega_pinion=None,
        joint_vel=None,
        clear_steps=0,
        step=0,
    )
    _SLEEVE_XH["x"] = SLEEVE_START[0]
    _SLEEVE_XH["h"] = SLEEVE_START[2]
    _LIVE["sled"] = np.array([SLED_START_X, 0.0, _wz(SLED_H)])
    _LIVE["stripe"] = np.array([STRIPE_START_X, 0.0, _wz(STRIPE_H)])
    _LIVE["strap"] = np.array([0.3375, 0.0, _wz(STRAP_H)])
    clear_joint_targets()


def _begin(objects) -> None:
    if _S["objects"] is None:
        _S["objects"] = objects
        _S["step"] = 0
    else:
        _S["objects"] = objects


def set_arms(arms) -> None:
    _S["arms"] = arms


def clear_arms() -> None:
    _S["arms"] = None


def set_omega_pinion(omega: float | None) -> None:
    """Inject pinion angular speed (rad/s) for offline tests (informational, not scored)."""
    _S["omega_pinion"] = None if omega is None else float(omega)


def set_joint_vel(rates: dict[str, float] | None) -> None:
    """Inject Mech joint rates for offline bodies_resting checks (rad/s or m/s for sled)."""
    if rates is None:
        _S["joint_vel"] = None
        return
    _S["joint_vel"] = {str(k): float(v) for k, v in rates.items()}


# - arm_watch: real finger gaps / TCP centres -----------------------------------------------------------------------
ARMS = {"left": "left_robot", "right": "right_robot"}


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=float)


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _pad_straddle(tcp: np.ndarray, finger_gap: float, close_axis: np.ndarray, sleeve_axis: np.ndarray, f7: float | None = None, f8: float | None = None) -> float:
    """Signed product of opposing pad-center projections relative to ``sleeve_axis`` along ``close_axis``."""
    g_l = abs(f7) if f7 is not None else 0.5 * finger_gap
    g_r = abs(f8) if f8 is not None else 0.5 * finger_gap
    pad_l = tcp - g_l * close_axis
    pad_r = tcp + g_r * close_axis
    return float(np.dot(pad_l - sleeve_axis, close_axis) * np.dot(pad_r - sleeve_axis, close_axis))


def _pad_probe_points(tcp: np.ndarray, finger_gap: float, rot: np.ndarray, f7: float | None = None, f8: float | None = None) -> list[tuple[float, float, float]]:
    """TCP, pad-tip center, and pad bounding-box sample points in world frame for clearance checks."""
    close_axis = rot[:, 0]
    width_axis = rot[:, 1]
    approach_axis = rot[:, 2]
    g_l = abs(f7) if f7 is not None else 0.5 * finger_gap
    g_r = abs(f8) if f8 is not None else 0.5 * finger_gap
    pts = [tuple(map(float, tcp)), tuple(map(float, tcp + PAD_TIP_PAST_TCP * approach_axis))]
    for sign, g_side in ((-1.0, g_l), (1.0, g_r)):
        for t_off in (g_side, g_side + PAD_THICK):
            for w_off in (-PAD_HALF_WIDTH, 0.0, PAD_HALF_WIDTH):
                for z_off in (-PAD_BACK_FROM_TCP, -0.5 * PAD_BACK_FROM_TCP, 0.0, PAD_TIP_PAST_TCP):
                    p = tcp + (sign * t_off) * close_axis + w_off * width_axis + z_off * approach_axis
                    pts.append(tuple(map(float, p)))
    return pts


def _arm_probe_points(g: dict) -> list[np.ndarray]:
    """All world points representing a gripper (TCP plus pad/tip probes if supplied or derivable)."""
    c = np.asarray(g["center"], float)
    if "pad_points" in g:
        return [c, *(np.asarray(p, float) for p in g["pad_points"])]
    if "rot" in g:
        gap = float(g.get("finger_gap", 0.0))
        return [np.asarray(p, float) for p in _pad_probe_points(c, gap, np.asarray(g["rot"], float))]
    pts = [c]
    if "tip" in g:
        pts.append(np.asarray(g["tip"], float))
    if "close_axis" in g:
        ca = np.asarray(g["close_axis"], float)
        n = float(np.linalg.norm(ca))
        if n > EPS:
            ca = ca / n
            half = 0.5 * float(g.get("finger_gap", 0.0))
            pts.extend([c - half * ca, c + half * ca, c - (half + PAD_THICK) * ca, c + (half + PAD_THICK) * ca])
    return pts


def read_arms(environment):
    from yam_arena.runtime import tcp_world

    base = environment.unwrapped
    origin = _np(base.scene.env_origins)[0]
    out = {}
    for arm, key in ARMS.items():
        robot = base.scene[key]
        q = _np(robot.data.joint_pos)[0]
        joints = list(robot.joint_names)
        f7 = float(q[joints.index("joint7")])
        f8 = float(q[joints.index("joint8")])
        # joint7/8 are signed openings; gap between pads ~ |f7|+|f8| (each side travels).
        finger_gap = abs(f7) + abs(f8)
        tcp = tcp_world(robot, 0) - origin
        bodies = _np(robot.data.body_link_pose_w)[0]
        names = list(robot.body_names)
        gpose = bodies[names.index("gripper")]
        R = _rot(gpose[3:7])
        # YAM parallel jaw closes along gripper local X (R[:, 0]); joint7/8 translate along -/+X.
        close_axis = R[:, 0]
        closing_xz_ok = abs(float(close_axis[1])) <= math.sin(math.radians(30.0)) + EPS
        sleeve_axis = np.array([_SLEEVE_XH["x"], SLEEVE_Y, _wz(_SLEEVE_XH["h"])])
        pad_straddle = _pad_straddle(tcp, finger_gap, close_axis, sleeve_axis, f7=f7, f8=f8)
        pad_points = _pad_probe_points(tcp, finger_gap, R, f7=f7, f8=f8)
        out[arm] = {
            "finger_gap": finger_gap,
            "center": tuple(map(float, tcp)),
            "close_axis": tuple(map(float, close_axis)),
            "pad_straddle": pad_straddle,
            "closing_xz_ok": bool(closing_xz_ok),
            "pad_points": pad_points,
        }
    return out


def _write_joint(environment, part: str, joint: str, value: float) -> None:
    base = environment.unwrapped
    if part not in {*getattr(base.scene, "articulations", {}), *{}}:
        arts = getattr(base.scene, "articulations", {})
        if part not in arts:
            return
    asset = base.scene[part]
    ids = asset.find_joints(f"{joint}_joint")[0]
    if len(ids) == 0:
        return
    jid = ids[0]
    pos = asset.data.joint_pos.clone()
    pos[0, jid] = float(value)
    asset.write_joint_position_to_sim(pos)
    vel = asset.data.joint_vel
    if hasattr(asset, "write_joint_velocity_to_sim"):
        z = vel.clone()
        z[0, jid] = 0.0
        asset.write_joint_velocity_to_sim(z)


def couple_kinematics_from_pinion(pinion_rad: float) -> dict:
    """Map pinion joint (0 = crank +40 deg) to drum, stripe x, sled x.

    With +y axis RH rule, rotating the crank CW from +40 toward -82.3 deg increases q_pinion (> 0),
    so crank_deg = 40 - deg(pinion_rad) and external meshing gives drum_deg = -deg(pinion_rad) / 4.1.
    """
    crank_deg = 40.0 - math.degrees(pinion_rad)
    drum = -math.degrees(pinion_rad) / GEAR_RATIO
    payout_m = -math.radians(drum) * R_EFF  # drum negative -> positive +x payout
    stripe_x = STRIPE_START_X + payout_m
    sled_x = float(np.clip(SLED_START_X + payout_m, SLED_START_X, SLED_REST_X))
    return {
        "crank_deg": crank_deg,
        "drum_deg": drum,
        "stripe_x": stripe_x,
        "sled_x": sled_x,
        "pinion_rad": pinion_rad,
        "drum_rad": math.radians(drum),
        "sled_slide": sled_x - SLED_REST_X,  # joint 0 = rest at SLED_REST_X
    }


def couple_kinematics_from_sled(sled_x: float) -> dict:
    """Back-drive the 4.1:1 train from sled payout (spring leads when the crank is released under load)."""
    sled_x = float(np.clip(sled_x, SLED_START_X, SLED_REST_X))
    payout_m = sled_x - SLED_START_X
    drum_rad = -payout_m / R_EFF
    drum = math.degrees(drum_rad)
    pinion_rad = -drum_rad * GEAR_RATIO
    return {
        "crank_deg": 40.0 - math.degrees(pinion_rad),
        "drum_deg": drum,
        "stripe_x": STRIPE_START_X + payout_m,
        "sled_x": sled_x,
        "pinion_rad": pinion_rad,
        "drum_rad": drum_rad,
        "sled_slide": sled_x - SLED_REST_X,
    }


def couple_mode(theta_pawl_deg: float, held: bool, coup_sled_x: float, sled_now: float) -> str:
    """Decide live train write mode for the engaged Fulton wind detent.

    Returns:
      - ``pinion_lead``: gripper or freewheel detent lets pinion write drum/sled/stripe
      - ``refuse_payout``: green UP engaged; proposed couple would advance sled (payout)
      - ``freewheel``: pawl past ratchet and crank released; sled spring leads
    """
    if theta_pawl_deg <= 10.0 + EPS:
        # Engaged green UP: block freewheel; allow wind-in / hold; refuse payout couple.
        if coup_sled_x > sled_now + EPS:
            return "refuse_payout"
        return "pinion_lead"
    if held:
        return "pinion_lead"
    return "freewheel"


PAWL_RIDGE_DEG = 20.0
PAWL_DETENT_Q = math.radians(-38.0)  # q_y for theta_pawl = +38 deg


def pawl_detent_target(theta_pawl_deg: float) -> float:
    """Over-center pawl drive target: rest 0 below the ridge, magenta pocket past it."""
    if float(theta_pawl_deg) < PAWL_RIDGE_DEG - EPS:
        return 0.0
    return float(PAWL_DETENT_Q)


def apply_pinion_preload(environment, pinion_q: float, loaded: bool) -> float:
    """Drive the crank toward payout with PRELOAD_TORQUE while it is counter-held under load; else hold position."""
    target = float(pinion_q) + (PRELOAD_TORQUE / PINION_DRIVE_K if loaded else 0.0)
    set_joint_target(environment, "winch", "pinion", target)
    return target


def _pin_joint_rates(names) -> None:
    rates = dict(_S.get("joint_vel") or {})
    for n in names:
        rates[n] = 0.0
    _S["joint_vel"] = rates


def apply_pawl_detent(environment, theta_pawl_deg: float) -> float:
    """Retarget the pawl spring drive; returns the q_y target written via set_joint_target."""
    target = pawl_detent_target(theta_pawl_deg)
    set_joint_target(environment, "winch", "pawl", target)
    return target


class _OfflineEnv:
    """Minimal stand-in so set_joint_target records into mechkit without a live scene."""

    class _U:
        class scene:
            articulations: dict = {}

    unwrapped = _U()


def _teleport_stripe(environment, stripe_x: float) -> None:
    base = environment.unwrapped
    if "stripe" not in getattr(base.scene, "rigid_objects", {}):
        return
    stripe = base.scene["stripe"]
    origin = _np(base.scene.env_origins)[0]
    import torch

    pose = torch.tensor(
        [[origin[0] + stripe_x, origin[1] + 0.0, origin[2] + _wz(STRIPE_H), 0.0, 0.0, 0.0, 1.0]],
        dtype=torch.float32,
        device=base.device,  # pose buffers may be warp arrays, so take device from the env
    )
    stripe.write_root_link_pose_to_sim(pose)
    if hasattr(stripe, "write_root_link_velocity_to_sim"):
        vel = torch.zeros((1, 6), dtype=pose.dtype, device=pose.device)
        stripe.write_root_link_velocity_to_sim(vel)
    elif hasattr(stripe, "write_root_velocity_to_sim"):
        vel = torch.zeros((1, 6), dtype=pose.dtype, device=pose.device)
        stripe.write_root_velocity_to_sim(vel)


_SEEDED = {"mech_ran": False, "ok": False}


@mech_on_reset
def _seed_winch_start(environment) -> None:
    """Pretension sled to x=0.455 (joint -0.025) and zero pinion/drum/pawl before settle & Frame 0."""
    try:
        base = environment.unwrapped
        if "winch" not in getattr(base.scene, "articulations", {}):
            return
        _write_joint(environment, "winch", "sled", SLED_START_X - SLED_REST_X)
        _write_joint(environment, "winch", "pinion", 0.0)
        _write_joint(environment, "winch", "drum", 0.0)
        _write_joint(environment, "winch", "pawl", 0.0)
        set_joint_target(environment, "winch", "sled", SLED_START_X - SLED_REST_X)
        set_joint_target(environment, "winch", "pawl", 0.0)
        _teleport_stripe(environment, STRIPE_START_X)
        _SEEDED["mech_ran"] = True
        _SEEDED["ok"] = True
    except Exception:
        pass


@on_reset
def _on_reset():
    clear_arms()
    set_omega_pinion(None)
    _reset_episode(None)
    _HANDS["poses"] = _HANDS["finger"] = None
    _HANDS["step"] = 0
    _SEEDED["ok"] = _SEEDED["mech_ran"]
    _SEEDED["mech_ran"] = False


@on_step
def _seed_once(environment):
    if _SEEDED["ok"]:
        return
    _seed_winch_start(environment)
    _SEEDED["ok"] = True


def _set_sleeve_xh_from_pinion(pinion_rad: float) -> None:
    cd = 40.0 - math.degrees(float(pinion_rad))
    loc = _crank_local(cd)
    _SLEEVE_XH["x"] = PINION[0] + loc[0]
    _SLEEVE_XH["h"] = PINION[2] + loc[2]


@on_step
def _watch_and_couple(environment):
    """Update live sleeve pose, read YAM fingers/TCP, couple 4.1:1, and enforce ratchet/freewheel."""
    winch_data = None
    try:
        base = environment.unwrapped
        if "winch" in getattr(base.scene, "articulations", {}):
            asset = base.scene["winch"]
            q = _np(asset.data.joint_pos)[0]
            qd = _np(asset.data.joint_vel)[0]
            names = list(asset.joint_names)
            idx = {n.removesuffix("_joint"): i for i, n in enumerate(names)}
            pinion = float(q[idx["pinion"]])
            omega = float(qd[idx["pinion"]])
            sled_q = float(q[idx["sled"]])
            pawl_q = float(q[idx["pawl"]])
            _set_sleeve_xh_from_pinion(pinion)
            winch_data = (pinion, omega, sled_q, pawl_q, qd, idx)
    except Exception:
        pass
    try:
        arms = read_arms(environment)
        set_arms(arms)
        _HANDS["step"] += 1
    except Exception:
        pass
    if winch_data is None:
        return
    try:
        pinion, omega, sled_q, pawl_q, qd, idx = winch_data
        set_omega_pinion(omega)
        set_joint_vel(
            {
                "pinion": float(qd[idx["pinion"]]),
                "drum": float(qd[idx["drum"]]),
                "pawl": float(qd[idx["pawl"]]),
                "sled": float(qd[idx["sled"]]),
            }
        )
        theta_pawl = -math.degrees(pawl_q)
        held = any_crank_held()
        sled_now = SLED_REST_X + sled_q
        # Over-center pawl detent: bias to UP below 20 deg, to magenta pocket past the ridge.
        apply_pawl_detent(environment, theta_pawl)
        coup_pinion = couple_kinematics_from_pinion(pinion)
        mode = couple_mode(theta_pawl, held, coup_pinion["sled_x"], sled_now)
        loaded = held and theta_pawl > 10.0 and sled_now < SLED_REST_X - 0.002
        if mode == "pinion_lead":
            # Held with pawl past ratchet, or engaged UP wind-in/hold: pinion writes drum + sled + stripe.
            apply_pinion_preload(environment, pinion, loaded)
            _write_joint(environment, "winch", "drum", coup_pinion["drum_rad"])
            _write_joint(environment, "winch", "sled", coup_pinion["sled_slide"])
            set_joint_target(environment, "winch", "sled", coup_pinion["sled_slide"])
            _teleport_stripe(environment, coup_pinion["stripe_x"])
        elif mode == "refuse_payout":
            # Engaged green UP wind detent: lock pinion, drum, sled, and stripe at current sled state.
            coup_hold = couple_kinematics_from_sled(sled_now)
            _write_joint(environment, "winch", "sled", coup_hold["sled_slide"])
            _write_joint(environment, "winch", "drum", coup_hold["drum_rad"])
            _write_joint(environment, "winch", "pinion", coup_hold["pinion_rad"])
            set_joint_target(environment, "winch", "sled", coup_hold["sled_slide"])
            _teleport_stripe(environment, coup_hold["stripe_x"])
            _set_sleeve_xh_from_pinion(coup_hold["pinion_rad"])
            apply_pinion_preload(environment, coup_hold["pinion_rad"], False)
            _pin_joint_rates(("pinion", "drum", "sled"))
        else:
            # Unbraced with pawl past ratchet: sled spring leads; do not overwrite sled from pinion.
            set_joint_target(environment, "winch", "sled", 0.0)
            if sled_now < 0.4780 - EPS:
                force = 1804.0 * (SLED_REST_X - sled_now)
                # One control-step spring advance (damped); floor at 4 mm so drum moves >4 deg.
                dx = min(0.012, max(0.004, abs(force) / 0.85 * DT * DT))
                sled_next = min(SLED_REST_X, sled_now + dx)
            else:
                sled_next = sled_now
            coup = couple_kinematics_from_sled(sled_next)
            _write_joint(environment, "winch", "sled", coup["sled_slide"])
            _write_joint(environment, "winch", "drum", coup["drum_rad"])
            _write_joint(environment, "winch", "pinion", coup["pinion_rad"])
            _teleport_stripe(environment, coup["stripe_x"])
            _set_sleeve_xh_from_pinion(coup["pinion_rad"])
            apply_pinion_preload(environment, coup["pinion_rad"], False)
            if sled_next <= sled_now + EPS:
                # Sled against the chock: train is locked, so the pinned joints report zero rate (no gravity droop).
                _pin_joint_rates(("pinion", "drum", "sled"))
    except Exception:
        pass


# - kinematics helpers ---------------------------------------------------------------------------------------------
def _h(state) -> float:
    return float(state.pos[2]) - TOP


def _pos_h(state) -> np.ndarray:
    return np.array([float(state.pos[0]), float(state.pos[1]), _h(state)])


def crank_angle_deg(sleeve_pos_h) -> float:
    dx = float(sleeve_pos_h[0]) - PINION[0]
    dh = float(sleeve_pos_h[2]) - PINION[2]
    return math.degrees(math.atan2(dh, dx))


def drum_deg_from_crank(crank_deg: float) -> float:
    return (crank_deg - 40.0) / GEAR_RATIO


def _winch_joints(objects) -> dict | None:
    w = objects.get("winch")
    if w is not None and hasattr(w, "joints") and w.joints:
        return w.joints
    return None


def drum_deg(objects) -> float:
    j = _winch_joints(objects)
    if j is not None and "drum" in j:
        return math.degrees(float(j["drum"]))
    if j is not None and "pinion" in j:
        return -math.degrees(float(j["pinion"])) / GEAR_RATIO
    return drum_deg_from_crank(crank_angle_deg(_pos_h(objects["sleeve"])))


def theta_pawl_deg(objects) -> float:
    j = _winch_joints(objects)
    if j is not None and "pawl" in j:
        # Joint +y RH: positive q moves paddle from +h toward +x; scored theta_pawl = -q_y (deg).
        return -math.degrees(float(j["pawl"]))
    p = _pos_h(objects["blue_paddle"])
    dx = float(p[0]) - PAWL_HINGE[0]
    dh = float(p[2]) - PAWL_HINGE[2]
    return math.degrees(math.atan2(-(dx), dh if abs(dh) > 1e-9 else 1e-9))


def stripe_x(objects) -> float:
    return float(objects["stripe"].pos[0])


def sled_x(objects) -> float:
    j = _winch_joints(objects)
    if j is not None and "sled" in j:
        return SLED_REST_X + float(j["sled"])
    return float(objects["sled"].pos[0])


def spoke_gap_mm(objects) -> float:
    r = 0.057
    return abs(drum_deg(objects)) * math.pi / 180.0 * r * 1000.0


def payout_spoke_gap_mm(objects) -> float:
    r = 0.057
    return abs(drum_deg(objects) + 30.0) * math.pi / 180.0 * r * 1000.0


# - grip / seating -------------------------------------------------------------------------------------------------
def crank_held_active(arm: str) -> bool:
    arms = _S["arms"]
    if not arms or arm not in arms or arms[arm] is None:
        return False
    g = arms[arm]
    gap = float(g.get("finger_gap", 1.0))
    if not (GRIP_GAP_MIN - EPS <= gap <= GRIP_GAP_MAX + EPS):
        return False
    c = np.asarray(g["center"], float)
    axis = np.array([_SLEEVE_XH["x"], SLEEVE_Y, _wz(_SLEEVE_XH["h"])])
    if "sleeve_axis" in g:
        axis = np.asarray(g["sleeve_axis"], float)
    if float(np.linalg.norm(c[[0, 2]] - axis[[0, 2]])) > GRIP_AXIS_TOL + EPS:
        return False
    if not (0.075 - EPS <= float(c[1]) <= 0.135 + EPS):
        return False
    if not g.get("closing_xz_ok", False):
        return False
    if float(g.get("pad_straddle", 1.0)) >= 0.0:
        return False
    return True


def any_crank_held() -> bool:
    return crank_held_active("left") or crank_held_active("right")


def holding_arm_name() -> str | None:
    for a in ("left", "right"):
        if crank_held_active(a):
            return a
    return None


def magenta_seated(objects) -> bool:
    th = theta_pawl_deg(objects)
    if not (33.0 - EPS <= th <= 43.0 + EPS):
        return False
    if "blue_paddle" in objects:
        p = _pos_h(objects["blue_paddle"])
    else:
        # Derive paddle from pawl joint when only MechState is present.
        rad = math.radians(th)
        p = np.array(
            [
                PAWL_HINGE[0] - 0.0280 * math.sin(rad),
                PAWL_HINGE[1],
                PAWL_HINGE[2] + 0.0280 * math.cos(rad),
            ]
        )
    d_xh = math.hypot(p[0] - MAGENTA_MOUTH[0], p[2] - MAGENTA_MOUTH[2])
    if d_xh > 0.0050 + EPS:
        return False
    if abs(p[1] - (-0.0560)) > 0.0080 + EPS:
        return False
    return True


def _box_dist(pts: np.ndarray, center, size, rot=None) -> float:
    r = np.eye(3) if rot is None else np.asarray(rot, dtype=float)
    local = (pts - np.asarray(center, dtype=float)) @ r
    q = np.abs(local) - np.asarray(size, dtype=float) / 2.0
    return float(np.linalg.norm(np.maximum(q, 0.0), axis=1).min())


def _cyl_y_dist(pts: np.ndarray, center, radius: float, half_len: float) -> float:
    c = np.asarray(center, dtype=float)
    dx = pts[:, 0] - c[0]
    dy = np.maximum(0.0, np.abs(pts[:, 1] - c[1]) - half_len)
    dz = pts[:, 2] - c[2]
    dr = np.maximum(0.0, np.hypot(dx, dz) - radius)
    return float(np.hypot(dr, dy).min())


def _keepout_surfaces(objects) -> tuple[list[tuple], list[tuple]]:
    """Boxes and Y-axis cylinders covering full extents of winch, crank, pawl, strap, stripe, and sled."""
    boxes = [
        ((0.3600, 0.0000, _wz(0.0050)), (0.460, 0.180, 0.010), None),
        ((0.2350, 0.0000, _wz(0.0300)), (0.130, 0.080, 0.040), None),
        ((0.1750, 0.0000, _wz(0.0875)), (0.020, 0.080, 0.075), None),
        ((0.2400, 0.0360, _wz(0.1475)), (0.120, 0.008, 0.195), None),
        ((0.2400, -0.0360, _wz(0.1475)), (0.120, 0.008, 0.195), None),
        ((0.4700, 0.0000, _wz(0.0585)), (0.240, 0.030, 0.013), None),
        ((0.5300, 0.0000, _wz(0.0800)), (0.020, 0.070, 0.035), None),
        ((0.3650, 0.0380, _wz(STRIPE_H)), (0.008, 0.020, 0.010), None),
        ((0.3650, -0.0380, _wz(STRIPE_H)), (0.008, 0.020, 0.010), None),
        (_LIVE["strap"], (0.155, 0.050, 0.003), None),
        (_LIVE["stripe"], (0.014, 0.052, 0.006), None),
        (_LIVE["sled"], (0.080, 0.056, 0.030), None),
    ]
    th = theta_pawl_deg(objects)
    rad_p = math.radians(th)
    if "blue_paddle" in objects:
        paddle_pos = np.asarray(objects["blue_paddle"].pos, float)
    else:
        paddle_pos = np.array(
            [PAWL_HINGE[0] - 0.0280 * math.sin(rad_p), PAWL_HINGE[1], _wz(PAWL_HINGE[2] + 0.0280 * math.cos(rad_p))]
        )
    boxes.append((paddle_pos, (0.014, 0.012, 0.052), _ry(-th)))

    j = _winch_joints(objects)
    if "sleeve" in objects:
        sh = _pos_h(objects["sleeve"])
        cd = crank_angle_deg(sh)
        sleeve_pos = np.asarray(objects["sleeve"].pos, float)
    elif j is not None and "pinion" in j:
        cd = 40.0 - math.degrees(float(j["pinion"]))
        loc = _crank_local(cd)
        sleeve_pos = np.array([PINION[0] + loc[0], SLEEVE_Y, _wz(PINION[2] + loc[2])])
    else:
        cd = 40.0
        sleeve_pos = np.array([SLEEVE_START[0], SLEEVE_Y, _wz(SLEEVE_START[2])])
    rad_c = math.radians(cd)
    crank_pos = (PINION[0] + 0.5 * R_CRANK * math.cos(rad_c), 0.0550, _wz(PINION[2] + 0.5 * R_CRANK * math.sin(rad_c)))
    boxes.append((crank_pos, (0.200, 0.018, 0.010), _ry(-cd)))

    cyls = [
        (sleeve_pos, SLEEVE_OD / 2.0, SLEEVE_LEN / 2.0),
        ((DRUM[0], 0.0, _wz(DRUM[2])), 0.057, 0.030),
        ((DRUM[0], 0.0240, _wz(DRUM[2])), 0.0738, 0.005),
        ((PINION[0], -0.0460, _wz(PINION[2])), 0.029, 0.004),
    ]
    return boxes, cyls


def grippers_open_and_clear(objects) -> bool:
    arms = _S["arms"]
    if not arms:
        return False
    boxes, cyls = _keepout_surfaces(objects)
    for a in ("left", "right"):
        g = arms.get(a)
        if g is None:
            return False
        if float(g.get("finger_gap", 0.0)) < 0.050 - EPS:
            return False
        pts = np.asarray(_arm_probe_points(g), dtype=float)
        if any(_box_dist(pts, c, sz, rot) < 0.050 - EPS for c, sz, rot in boxes):
            return False
        if any(_cyl_y_dist(pts, c, r, hl) < 0.050 - EPS for c, r, hl in cyls):
            return False
    return True


def bodies_resting(objects) -> bool:
    names = [n for n in ("sleeve", "blue_paddle", "stripe", "sled", "winch") if n in objects]
    for name in names:
        if not objects[name].resting(max_speed=0.01, max_spin=0.5):
            return False
    # Live OBJECTS are winch + stripe: Mech root rest above, plus joint rates from on_step / set_joint_vel.
    jv = _S.get("joint_vel")
    if jv:
        # Hinges/revolutes: <0.5 rad/s; sled slide: <10 mm/s (same numeric band as free-body rest).
        for name, rate in jv.items():
            limit = 0.01 if name == "sled" else 0.5
            if abs(float(rate)) > limit + EPS:
                return False
    return True


def _near_strap_or_sled(center) -> bool:
    c = np.asarray(center, float)
    for body, half in (
        (_LIVE["sled"], np.array([0.040, 0.028, 0.015]) + 0.005),
        (_LIVE["stripe"], np.array([0.007, 0.026, 0.003]) + 0.005),
        (_LIVE["strap"], np.array([0.0775, 0.025, 0.0015]) + 0.005),
    ):
        d = np.abs(c - body)
        if np.all(d <= half + EPS):
            return True
    return False


# - violations and ladder ------------------------------------------------------------------------------------------
def _update_violations(objects) -> None:
    x_sled = sled_x(objects)
    x_stripe = stripe_x(objects)
    th = theta_pawl_deg(objects)
    drum = drum_deg(objects)
    held = any_crank_held()
    hold_arm = holding_arm_name()

    if x_sled < 0.4780 - EPS and th > 10.0 + EPS and not held:
        _S["violations"].add("pawl_flipped_unbraced")

    d_drum = abs(drum - _S["prev_drum_deg"])
    if (not held) and x_sled < 0.4780 - EPS and d_drum > 4.0 + EPS:
        _S["violations"].add("runaway_freewheel")

    arms = _S["arms"] or {}
    for _, g in arms.items():
        if g is None:
            continue
        if any(_near_strap_or_sled(pt) for pt in _arm_probe_points(g)):
            _S["violations"].add("sled_or_strap_tampered")
    if "sled" in objects and _h(objects["sled"]) > 0.088 + EPS:
        _S["violations"].add("sled_or_strap_tampered")
    dx = abs(x_sled - _S["prev_sled_x"])
    if not held:
        if dx > 0.002 + EPS:
            _S["violations"].add("sled_or_strap_tampered")
        # Cumulative path length of unbraced per-step |dx| (not abs(x-braced) each step).
        _S["unbraced_cum"] += dx
        if _S["unbraced_cum"] > 0.003 + EPS:
            _S["violations"].add("sled_or_strap_tampered")
        if abs(x_sled - _S["braced_sled_x"]) > 0.003 + EPS:
            _S["violations"].add("sled_or_strap_tampered")
    else:
        _S["braced_sled_x"] = x_sled
        _S["unbraced_cum"] = 0.0

    if x_stripe < 0.3320 - EPS or x_stripe > 0.3780 + EPS or drum < -45.0 - EPS:
        _S["violations"].add("overshoot_or_wind_wrong")

    # Continuous counter-hold bookkeeping for tier 0.75.
    if hold_arm is not None:
        if _S["holding_arm"] is None:
            _S["holding_arm"] = hold_arm
        if th <= 10.0 + EPS:
            _S["held_before_pawl10"] = True
    if th > 10.0 + EPS:
        arm = _S["holding_arm"]
        if arm is None or not crank_held_active(arm):
            _S["hold_broken_after_pawl10"] = True
        elif hold_arm is not None and hold_arm != arm:
            _S["hold_broken_after_pawl10"] = True

    if magenta_seated(objects):
        _S["L_magenta"] = True

    _S["prev_drum_deg"] = drum
    _S["prev_sled_x"] = x_sled


def _tier025(objects) -> bool:
    if not any_crank_held():
        return False
    if spoke_gap_mm(objects) > 5.0 + EPS:
        return False
    x = stripe_x(objects)
    if not (0.3350 - EPS <= x <= 0.3450 + EPS):
        return False
    if sled_x(objects) > 0.4580 + EPS:
        return False
    return True


def _tier050(objects) -> bool:
    if not magenta_seated(objects):
        return False
    arm = _S["holding_arm"]
    if arm is None or not crank_held_active(arm):
        return False
    if spoke_gap_mm(objects) > 5.0 + EPS:
        return False
    x = stripe_x(objects)
    if not (0.3350 - EPS <= x <= 0.3450 + EPS):
        return False
    return True


def _payout_alignment_ok(objects) -> bool:
    if not magenta_seated(objects):
        return False
    if payout_spoke_gap_mm(objects) > 5.0 + EPS:
        return False
    drum = drum_deg(objects)
    if not (-36.0 - EPS <= drum <= -24.0 + EPS):
        return False
    if abs(stripe_x(objects) - STRIPE_TARGET_X) > 0.0050 + EPS:
        return False
    xs = sled_x(objects)
    if not (0.4780 - EPS <= xs <= 0.4800 + EPS):
        return False
    return True


def _tier075(objects) -> bool:
    if _S["L_payout"]:
        return _payout_alignment_ok(objects)
    # Spec: 0.75 only with prior 0.50 intact (flip under preload at start band), then held payout.
    if not _S.get("L_050", False):
        return False
    # Same holding arm must have been active before theta_pawl > 10 and unbroken through payout.
    if not _S.get("held_before_pawl10", False):
        return False
    if _S.get("hold_broken_after_pawl10", False):
        return False
    arm = _S["holding_arm"]
    if arm is None or not crank_held_active(arm):
        return False
    return _payout_alignment_ok(objects)


def _tier100(objects) -> bool:
    if not _tier075(objects):
        return False
    if not bodies_resting(objects):
        return False
    if not grippers_open_and_clear(objects):
        return False
    return True


def _instantaneous(objects) -> float:
    if _S["violations"]:
        return 0.0
    if _tier100(objects):
        _S["L_payout"] = True
        return 1.0
    if _tier075(objects):
        _S["L_payout"] = True
        return 0.75
    if _tier050(objects):
        _S["L_050"] = True
        return 0.50
    if _tier025(objects):
        return 0.25
    return 0.0


def _end_cap() -> float:
    if not _S["L_magenta"]:
        return 0.0
    if not _S["L_payout"]:
        return 0.50
    if _S["clear_steps"] >= HOLD_STEPS:
        return 1.0
    return 0.75


def _sync_live(objects) -> None:
    if "sled" in objects:
        _LIVE["sled"] = np.asarray(objects["sled"].pos, float)
    else:
        _LIVE["sled"] = np.array([sled_x(objects), 0.0, _wz(SLED_H)])
    _LIVE["stripe"] = np.asarray(objects["stripe"].pos, float)
    if "strap" in objects:
        _LIVE["strap"] = np.asarray(objects["strap"].pos, float)
    else:
        _LIVE["strap"] = np.array([0.3375, 0.0, _wz(STRAP_H)])
    if "sleeve" in objects:
        p = _pos_h(objects["sleeve"])
        _SLEEVE_XH["x"] = float(p[0])
        _SLEEVE_XH["h"] = float(p[2])
    else:
        j = _winch_joints(objects)
        if j and "pinion" in j:
            cd = 40.0 - math.degrees(float(j["pinion"]))
            loc = _crank_local(cd)
            _SLEEVE_XH["x"] = PINION[0] + loc[0]
            _SLEEVE_XH["h"] = PINION[2] + loc[2]


def score(objects) -> float:
    """Partial credit with latched violations and magenta/payout end caps (SCORE_OVER=end).

    Episode score is min(tier, end_cap). A clear-rest frame scores 0.75 until 20 consecutive such steps
    (1.0 s) have been counted, then 1.0; success() is true from then on. Any result below 0.50 is forced
    to 0.0 so a brief magenta seat then pawl slip cannot ship 0.25 at timeout.
    """
    _begin(objects)
    _S["step"] += 1
    _sync_live(objects)
    _update_violations(objects)
    if _S["violations"]:
        return 0.0
    tier = _instantaneous(objects)
    # Clear-rest frames are counted here: 20 consecutive steps (1.0 s) turn 0.75 into 1.0.
    _S["clear_steps"] = _S["clear_steps"] + 1 if tier >= 1.0 - EPS else 0
    capped = float(min(tier, _end_cap()))
    if capped < 0.50 - EPS:
        return 0.0
    return capped


def success(objects) -> bool:
    """True once score() has reached 1.0 (20 consecutive clear-rest steps) and the clear-rest frame still holds."""
    _begin(objects)
    _sync_live(objects)
    _update_violations(objects)
    if _S["violations"]:
        return False
    return _S["clear_steps"] >= HOLD_STEPS and abs(_instantaneous(objects) - 1.0) < EPS


def distance(objects) -> float:
    _sync_live(objects)
    if "blue_paddle" in objects:
        p = _pos_h(objects["blue_paddle"])
        d_pawl = math.hypot(p[0] - MAGENTA_MOUTH[0], p[2] - MAGENTA_MOUTH[2])
    else:
        d_pawl = abs(theta_pawl_deg(objects) - 38.0) * 0.001
    return d_pawl + abs(stripe_x(objects) - STRIPE_TARGET_X) + abs(sled_x(objects) - SLED_REST_X)


# - open-loop demo -------------------------------------------------------------------------------------------------
def _crank_radius(deg: float) -> float:
    return R_CRANK if deg >= -50.0 - EPS else R_DEMO


def _sleeve_at_crank(deg: float, h: float | None = None) -> tuple[float, float, float]:
    r = _crank_radius(deg)
    rad = math.radians(deg)
    x = PINION[0] + r * math.cos(rad)
    hh = PINION[2] + r * math.sin(rad) if h is None else h
    return (x, GRIP_Y, _wz(hh))


CRANK_DEG = [40, 22, 4, -14, -32, -50, -68, -76, -80, -81, -81.5, -82.0, -82.1, -82.2, -82.25, -82.3, -82.3, -82.3]
CRANK_H = [round(float(PINION[2] + _crank_radius(d) * math.sin(math.radians(d))), 4) for d in CRANK_DEG]


def demo():
    """Open-loop bimanual plan: left counter-holds and pays out; right flips the pawl. No hidden scorer state."""
    sleeve_grip = -(SLEEVE_OD - 0.007) / 2  # 19 mm command on the 26 mm sleeve: 7 mm narrower, jaws stop on the sleeve
    paddle_open = -0.016  # 32 mm gap: clears the 12 mm paddle while staying clear of the bezel/cheek
    paddle_grip = -(0.012 - 0.007) / 2  # 5 mm command on the 12 mm paddle
    park_z = TOP + 0.20
    g0 = _left_grip(40.0)
    gr = _right_grip()
    left = [
        Waypoint(2.0, (0.30, 0.22, park_z), FINGER_OPEN, "left approach", g0),
        Waypoint(2.0, (SLEEVE_START[0], GRIP_Y, _wz(SLEEVE_START[2] + 0.04)), FINGER_OPEN, "over sleeve", g0),
        Waypoint(2.0, (SLEEVE_START[0], GRIP_Y, _wz(SLEEVE_START[2])), FINGER_OPEN, "to sleeve", g0),
        Waypoint(0.8, (SLEEVE_START[0], GRIP_Y, _wz(SLEEVE_START[2])), FINGER_OPEN, "settle", g0),
        Waypoint(1.2, (SLEEVE_START[0], GRIP_Y, _wz(SLEEVE_START[2])), sleeve_grip, "close sleeve", g0),
        Waypoint(1.0, (SLEEVE_START[0], GRIP_Y, _wz(SLEEVE_START[2])), sleeve_grip, "hold sleeve", g0),
    ]
    # Grasp 8 mm above paddle centre so open pads stay clear above the -y U-frame cheek (h=0.245).
    p_up = (PADDLE_UP[0], PADDLE_UP[1], _wz(PADDLE_UP[2] + 0.008))
    p_dn = (
        PADDLE_DOWN[0] - 0.008 * math.sin(math.radians(38.0)),
        PADDLE_DOWN[1],
        _wz(PADDLE_DOWN[2] + 0.008 * math.cos(math.radians(38.0))),
    )
    p_up_over = (p_up[0], p_up[1], _wz(0.3200))
    p_dn_over = (p_dn[0], p_dn[1], _wz(0.3200))
    right_park = (0.18, -0.28, park_z)
    right = [
        Waypoint(2.5, right_park, FINGER_OPEN, "right park turn", gr),
        Waypoint(1.5, (0.22, -0.18, _wz(0.3200)), FINGER_OPEN, "hover pawl", gr),
        Waypoint(2.0, (0.22, -0.18, _wz(0.3200)), FINGER_OPEN, "wait hold", gr),
    ]
    right += [
        Waypoint(1.5, p_up_over, paddle_open, "over paddle", gr),
        Waypoint(2.0, p_up, paddle_open, "to paddle", gr),
        Waypoint(0.6, p_up, paddle_open, "settle paddle", gr),
        Waypoint(1.0, p_up, paddle_grip, "grip paddle", gr),
        Waypoint(2.5, p_dn, paddle_grip, "flip magenta", gr),
        Waypoint(0.8, p_dn, paddle_grip, "seat", gr),
        Waypoint(1.0, p_dn, paddle_open, "release pawl", gr),
        Waypoint(1.0, p_dn_over, paddle_open, "lift pawl", gr),
        Waypoint(1.5, (0.22, -0.18, _wz(0.3200)), FINGER_OPEN, "retreat pawl", gr),
        Waypoint(1.5, right_park, FINGER_OPEN, "right park", gr),
    ]
    left += [Waypoint(11.1, (SLEEVE_START[0], GRIP_Y, _wz(SLEEVE_START[2])), sleeve_grip, "hold through flip", g0)]
    for i, (deg, hh) in enumerate(zip(CRANK_DEG, CRANK_H)):
        g = _left_grip(deg)
        left.append(Waypoint(0.7 if i else 1.0, _sleeve_at_crank(deg, hh), sleeve_grip, f"crank {deg}", g))
    end = _sleeve_at_crank(-82.3, CRANK_H[-1])
    g_end = _left_grip(-82.3)
    left += [
        Waypoint(1.0, end, FINGER_OPEN, "open left", g_end),
        Waypoint(0.8, (end[0], GRIP_Y, _wz(0.11)), FINGER_OPEN, "lift sleeve", g_end),
        Waypoint(1.0, (0.30, 0.22, _wz(0.14)), FINGER_OPEN, "pull out", g_end),
        Waypoint(1.2, (0.22, 0.28, park_z), FINGER_OPEN, "left clear", g_end),
        Waypoint(2.0, (0.22, 0.28, park_z), FINGER_OPEN, "rest hold", g_end),
    ]
    right += [Waypoint(18.4, right_park, FINGER_OPEN, "right wait payout", gr)]
    right += [Waypoint(2.0, right_park, FINGER_OPEN, "rest hold", gr)]
    return {"left": left, "right": right}


# - offline geometry check -----------------------------------------------------------------------------------------
GEOM_MAX_OVERLAP = 0.0005  # m; deeper interpenetration of two non-jointed colliders fails the check

# Pairs that may overlap, with the reason. Keys are frozensets of body names ("base" is the fixed frame).
GEOM_ALLOWED = {
    frozenset(("base", "pinion")): "joint-adjacent: revolute joint, collisionEnabled false (axle through cheeks)",
    frozenset(("base", "drum")): "joint-adjacent: revolute joint, collisionEnabled false (axle through cheeks)",
    frozenset(("base", "pawl")): "joint-adjacent: revolute joint, collisionEnabled false (pivot lug)",
    frozenset(("base", "sled")): "joint-adjacent: prismatic joint, collisionEnabled false (carriage on rail, chock stop)",
}


def _geom_prims(pose: dict) -> list[dict]:
    """Collider primitives of the winch and stripe in table coordinates (x, y, h) for joint values in ``pose``."""
    from mechkit import _joint_pose

    prims = []

    def add(body, i, sh, rot, origin):
        r_t = rot @ _quat_wxyz_matrix(sh.tilt)
        c = rot @ np.asarray(sh.at, float) + origin
        if sh.kind == "box":
            prims.append(dict(body=body, idx=i, kind="box", c=c, R=r_t, half=np.asarray(sh.size, float) / 2.0))
        else:
            prims.append(dict(body=body, idx=i, kind="cyl", c=c, R=r_t, r=float(sh.size[0]), hl=float(sh.size[2]) / 2.0))

    for i, sh in enumerate(WINCH.shapes):
        if sh.collide:
            add("base", i, sh, np.eye(3), np.zeros(3))
    for j in WINCH.joints:
        rot, origin = _joint_pose(j, pose.get(j.name, 0.0))
        for i, sh in enumerate(j.shapes):
            if sh.collide:
                add(j.name, i, sh, rot, origin)
    sx = pose.get("stripe_x", STRIPE_START_X)
    prims.append(dict(body="stripe", idx=0, kind="box", c=np.array([sx, 0.0, STRIPE_H]), R=np.eye(3), half=np.array([0.007, 0.026, 0.003])))
    return prims


def _prim_sdf(pr: dict, pts: np.ndarray) -> np.ndarray:
    loc = (pts - pr["c"]) @ pr["R"]
    if pr["kind"] == "box":
        q = np.abs(loc) - pr["half"]
        return np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
    dr = np.hypot(loc[:, 0], loc[:, 1]) - pr["r"]
    dz = np.abs(loc[:, 2]) - pr["hl"]
    return np.hypot(np.maximum(dr, 0.0), np.maximum(dz, 0.0)) + np.minimum(np.maximum(dr, dz), 0.0)


def _prim_surface(pr: dict, step: float = 0.001) -> np.ndarray:
    """Surface sample points (world) at roughly ``step`` spacing, including edges and corners."""
    if pr["kind"] == "box":
        hx, hy, hz = pr["half"]
        pts = []
        for ax, (a, b, c) in enumerate(((hx, hy, hz), (hy, hz, hx), (hz, hx, hy))):
            u = np.linspace(-b, b, max(2, int(2 * b / step) + 1))
            v = np.linspace(-c, c, max(2, int(2 * c / step) + 1))
            uu, vv = np.meshgrid(u, v)
            for sgn in (-1.0, 1.0):
                f = np.stack([np.full(uu.size, sgn * a), uu.ravel(), vv.ravel()], axis=1)
                pts.append(np.roll(f, ax, axis=1))
        loc = np.vstack(pts)
    else:
        r, hl = pr["r"], pr["hl"]
        z = np.linspace(-hl, hl, max(2, int(2 * hl / step) + 1))
        n_ang = max(16, int(2 * math.pi * r / step))
        ang = np.linspace(0.0, 2 * math.pi, n_ang, endpoint=False)
        side = np.stack([np.outer(np.cos(ang), np.ones_like(z)).ravel() * r, np.outer(np.sin(ang), np.ones_like(z)).ravel() * r, np.outer(np.ones_like(ang), z).ravel()], axis=1)
        rr = np.linspace(0.0, r, max(2, int(r / step) + 1))
        disk = np.stack([np.outer(rr, np.cos(ang)).ravel(), np.outer(rr, np.sin(ang)).ravel()], axis=1)
        caps = [np.column_stack([disk, np.full(len(disk), sgn * hl)]) for sgn in (-1.0, 1.0)]
        loc = np.vstack([side, *caps])
    return loc @ pr["R"].T + pr["c"]


def _prim_aabb(pr: dict):
    if pr["kind"] == "box":
        ext = np.abs(pr["R"]) @ pr["half"]
    else:
        ax = pr["R"][:, 2]
        ext = np.abs(ax) * pr["hl"] + np.sqrt(np.maximum(1.0 - ax**2, 0.0)) * pr["r"]
    return pr["c"] - ext, pr["c"] + ext


def geometry_check(pose: dict | None = None, max_overlap: float = GEOM_MAX_OVERLAP):
    """Pairwise penetration of rigid colliders at ``pose`` (default: start pose).

    Returns (violations, allowed_hits, below_table). Same-body pairs are one rigid body and skipped.
    ``allowed_hits`` lists overlaps in GEOM_ALLOWED; ``violations`` lists every other pair deeper than ``max_overlap``.
    """
    pose = {"sled": SLED_START_X - SLED_REST_X, "stripe_x": STRIPE_START_X, **(pose or {})}
    prims = _geom_prims(pose)
    boxes = [_prim_aabb(p) for p in prims]
    surf = {}
    violations, allowed, below = [], [], []
    for i, p in enumerate(prims):
        if boxes[i][0][2] < -max_overlap:
            below.append((p["body"], p["idx"], float(boxes[i][0][2])))
    for i in range(len(prims)):
        for j in range(i + 1, len(prims)):
            a, b = prims[i], prims[j]
            if a["body"] == b["body"]:
                continue
            (alo, ahi), (blo, bhi) = boxes[i], boxes[j]
            if np.any(ahi < blo + max_overlap) or np.any(bhi < alo + max_overlap):
                continue
            depth = 0.0
            for src, dst in ((i, b), (j, a)):
                if src not in surf:
                    surf[src] = _prim_surface(prims[src])
                sd = _prim_sdf(dst, surf[src])
                depth = max(depth, float(-sd.min()))
            if depth > max_overlap:
                hit = (a["body"], a["idx"], b["body"], b["idx"], depth)
                key = frozenset((a["body"], b["body"]))
                (allowed if key in GEOM_ALLOWED else violations).append(hit)
    return violations, allowed, below


def geometry_anchor_report():
    """Each joint anchor must lie inside a collider of its moving body and of the fixed frame (or its stop)."""
    out = []
    for j in WINCH.joints:
        anchor = np.asarray(j.at, float)[None, :]
        res = {}
        for body in (j.name, "base"):
            prims = [p for p in _geom_prims({}) if p["body"] == body]
            res[body] = min(float(_prim_sdf(p, anchor)[0]) for p in prims)
        out.append((j.name, res[j.name], res["base"]))
    return out


# - offline self-test ----------------------------------------------------------------------------------------------
def self_test(ObjectState=None):
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw=0.0):
        return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])

    def body(size, x, y, h, vel=(0, 0, 0), yaw=0.0):
        half = np.asarray(size, float) / 2
        pos = np.array([x, y, _wz(h)])
        q = quat(yaw)
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def cyl_body(r, height, x, y, h, vel=(0, 0, 0)):
        pos = np.array([x, y, _wz(h)])
        half = np.array([r, r, height / 2])
        return ObjectState(pos, quat(), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def paddle_at(theta_deg):
        t = theta_deg / 38.0
        x = PADDLE_UP[0] + t * (PADDLE_DOWN[0] - PADDLE_UP[0])
        y = PADDLE_UP[1]
        h = PADDLE_UP[2] + t * (PADDLE_DOWN[2] - PADDLE_UP[2])
        return body((0.014, 0.012, 0.052), x, y, h)

    def sleeve_at(crank_deg, h=None):
        rad = math.radians(crank_deg)
        x = PINION[0] + R_CRANK * math.cos(rad)
        hh = PINION[2] + R_CRANK * math.sin(rad) if h is None else h
        return cyl_body(SLEEVE_OD / 2, SLEEVE_LEN, x, SLEEVE_Y, hh)

    def stripe_at(x):
        return body((0.014, 0.052, 0.006), x, 0.0, STRIPE_H)

    def sled_at(x, h=SLED_H, vel=(0, 0, 0)):
        return body((0.080, 0.056, 0.030), x, 0.0, h, vel=vel)

    def fixed_scene():
        return {
            "strap": body((0.155, 0.050, 0.003), 0.3375, 0.0, STRAP_H),
            "ratchet": cyl_body(0.029, 0.008, PINION[0], -0.0460, PINION[2]),
            "drum": cyl_body(0.057, 0.060, DRUM[0], 0.0, DRUM[2]),
            "drum_pointer": body((0.012, 0.012, 0.020), *DRUM_POINTER),
            "spoke0": body((0.014, 0.004, 0.040), 0.2600, 0.0310, DRUM[2] + 0.0370),
            "spoke30": body(
                (0.014, 0.004, 0.040),
                DRUM[0] + 0.0370 * math.sin(math.radians(30.0)),
                0.0310,
                DRUM[2] + 0.0370 * math.cos(math.radians(30.0)),
            ),
            "pinion": cyl_body(0.018, 0.010, PINION[0], 0.0, PINION[2]),
            "crank_arm": body((0.200, 0.018, 0.010), 0.2832, 0.0550, 0.2647),
            "green_mouth": body((0.010, 0.004, 0.012), *GREEN_MOUTH),
            "magenta_mouth": body((0.010, 0.004, 0.012), *MAGENTA_MOUTH),
        }

    def scene(paddle=None, sleeve=None, stripe=None, sled=None):
        out = fixed_scene()
        out["blue_paddle"] = paddle or paddle_at(0.0)
        out["sleeve"] = sleeve or sleeve_at(40.0, SLEEVE_START[2])
        out["stripe"] = stripe or stripe_at(STRIPE_START_X)
        out["sled"] = sled or sled_at(SLED_START_X)
        return out

    def coupled_scene(crank_deg, pawl_deg, h=None):
        coup = couple_kinematics_from_pinion(math.radians(40.0 - crank_deg))
        return scene(
            paddle=paddle_at(pawl_deg),
            sleeve=sleeve_at(crank_deg, h),
            stripe=stripe_at(coup["stripe_x"]),
            sled=sled_at(coup["sled_x"]),
        )

    park_z = TOP + 0.22

    def grip(arm, held=True, center=None, gap=0.026, clear=False):
        if clear:
            c = center or ((0.22, 0.28, park_z) if arm == "left" else (0.18, -0.28, park_z))
            rot = _left_grip(-82.3) if arm == "left" else _right_grip()
            return {
                "finger_gap": 0.060,
                "center": c,
                "close_axis": tuple(map(float, rot[:, 0])),
                "pad_straddle": -1.0,
                "closing_xz_ok": True,
                "pad_points": _pad_probe_points(np.asarray(c, float), 0.060, rot),
            }
        if not held:
            return {
                "finger_gap": 0.080,
                "center": center or (0.20, 0.30 if arm == "left" else -0.30, TOP + 0.25),
                "pad_straddle": 1.0,
                "closing_xz_ok": False,
            }
        sx, sy, sh = SLEEVE_START
        c = center or (sx, sy, _wz(sh))
        return {
            "finger_gap": gap,
            "center": c,
            "pad_straddle": -1.0,
            "closing_xz_ok": True,
        }

    def arms_hold(left=True, right=False, sleeve=None):
        sx = sleeve or SLEEVE_START
        c = (sx[0], sx[1], _wz(sx[2]))
        return {
            "left": grip("left", held=left, center=c if left else None),
            "right": grip("right", held=right, center=c if right else None),
        }

    def arms_clear():
        return {"left": grip("left", clear=True), "right": grip("right", clear=True)}

    def run(st, arms=None, steps=1):
        clear_arms()
        set_arms(arms)
        set_omega_pinion(None)
        _reset_episode()
        got = 0.0
        ok = False
        for _ in range(steps):
            got = score(st)
            ok = success(st)
        return got, ok

    cases = []

    def rest_steps(st, n=HOLD_STEPS):
        """n consecutive clear-rest frames in arena order (success() then score()); returns per-step (score, success)."""
        out = []
        for _ in range(n):
            ok_i = success(st)
            out.append((score(st), ok_i))
        return out

    def assert_full_after_hold(st, tag):
        done = _S["clear_steps"]
        need = HOLD_STEPS - done
        seq = rest_steps(st, need + 1)
        assert all(abs(g - 0.75) < 1e-9 and not o for g, o in seq[: need - 1]), (tag, seq)
        assert abs(seq[need - 1][0] - 1.0) < 1e-9, (tag, "score reaches 1.0 at clear step 20", seq[need - 1])
        assert seq[need][1] and abs(seq[need][0] - 1.0) < 1e-9, (tag, "success follows score 1.0", seq[need])

    def check(name, st, arms, want, want_ok=False, steps=1):
        got, ok = run(st, arms, steps=steps)
        assert abs(got - want) < 1e-9 and ok == want_ok, (name, got, ok, want, want_ok, set(_S["violations"]))
        cases.append(name)

    # Authored sleeve world centre at rest must match SLEEVE_START (table x,y,h) within 1 mm.
    _pinion_j = next(j for j in WINCH.joints if j.name == "pinion")
    _sleeve_s = next(s for s in _pinion_j.shapes if s.color == RED and s.kind == "cyl")
    _sleeve_mech = np.asarray(_pinion_j.at, float) + np.asarray(_sleeve_s.at, float)
    assert np.linalg.norm(_sleeve_mech - np.asarray(SLEEVE_START, float)) < 0.001, (
        "authored sleeve centre",
        tuple(_sleeve_mech),
        SLEEVE_START,
    )
    _tilt_axis = _quat_wxyz_matrix(_sleeve_s.tilt) @ np.array([0.0, 0.0, 1.0])
    assert (
        abs(float(_tilt_axis[1]) - 1.0) < 1e-6
        and abs(float(_tilt_axis[0])) < 1e-6
        and abs(float(_tilt_axis[2])) < 1e-6
    ), ("sleeve tilt not +y", _tilt_axis, _sleeve_s.tilt)
    cases.append("authored sleeve centre matches SLEEVE_START")

    check("start", scene(), None, 0.0)
    check("crank hold only (end-capped 0)", scene(), arms_hold(True, False), 0.0)
    check("magenta unbraced under load", scene(paddle=paddle_at(38.0)), None, 0.0)
    check("hold and magenta seated", scene(paddle=paddle_at(38.0)), arms_hold(True, False), 0.50)

    sleeve_pay = (PINION[0] + R_CRANK * math.cos(math.radians(-82.3)), SLEEVE_Y, 0.0411)
    pay = scene(
        paddle=paddle_at(38.0),
        sleeve=sleeve_at(-82.3, 0.0411),
        stripe=stripe_at(STRIPE_TARGET_X),
        sled=sled_at(0.4790),
    )

    def seed_hold_then_magenta():
        clear_arms()
        set_arms(arms_hold(True, False))
        set_omega_pinion(None)
        _reset_episode()
        score(scene(paddle=paddle_at(0.0)))
        score(scene(paddle=paddle_at(38.0)))

    seed_hold_then_magenta()
    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    got = score(pay)
    assert abs(got - 0.75) < 1e-9, ("payout tier", got, dict(_S))
    cases.append("payout tier 0.75")

    set_arms(arms_clear())
    got = score(pay)
    assert abs(got - 0.75) < 1e-9 and not success(pay), ("first clear frame", got, dict(_S))
    assert abs(_end_cap() - 0.75) < 1e-9
    cases.append("first clear frame is 0.75 and not success")
    assert_full_after_hold(pay, "hold 20 clear steps")
    assert abs(_end_cap() - 1.0) < 1e-9
    cases.append("20 clear-rest steps give score 1.0 then success")

    seed_hold_then_magenta()
    assert abs(score(scene(paddle=paddle_at(38.0))) - 0.50) < 1e-9
    cases.append("ladder 0.50")

    # Blocker: 0.75 without held_before_pawl10 must fail.
    clear_arms()
    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    _reset_episode()
    # Jump straight to magenta + payout geometry without ever scoring th<=10 while held.
    got = score(pay)
    assert got < 0.75 - EPS, ("0.75 without held_before_pawl10", got, dict(_S))
    assert not _S["held_before_pawl10"]
    assert not _S["L_050"]
    cases.append("0.75 requires held_before_pawl10")

    # Blocker: hold -> green-UP payout geometry -> flip magenta only at slack must not score 0.75.
    clear_arms()
    set_arms(arms_hold(True, False))
    _reset_episode()
    score(scene(paddle=paddle_at(0.0)))  # held_before_pawl10, no L_050
    assert _S["held_before_pawl10"] and not _S["L_050"]
    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    green_pay = scene(
        paddle=paddle_at(0.0),
        sleeve=sleeve_at(-82.3, 0.0411),
        stripe=stripe_at(STRIPE_TARGET_X),
        sled=sled_at(0.4790),
    )
    assert score(green_pay) == 0.0
    got = score(pay)  # magenta only after payout; never had 0.50 at start band
    assert got < 0.75 - EPS, ("payout-then-flip cheat", got, dict(_S))
    assert not _S["L_050"] and not _S["L_payout"]
    cases.append("payout-then-flip without prior 0.50 scores 0")

    check("timeout hold-only end-cap", scene(), arms_hold(True, False), 0.0)
    check("pawl wiggle 15 deg with hold", scene(paddle=paddle_at(15.0)), arms_hold(True, False), 0.0)

    clear_arms()
    set_arms(None)
    _reset_episode()
    st = scene(paddle=paddle_at(20.0), sled=sled_at(0.4550))
    assert score(st) == 0.0 and "pawl_flipped_unbraced" in _S["violations"]
    set_arms(arms_hold(True, False))
    assert score(scene(paddle=paddle_at(38.0))) == 0.0 and "pawl_flipped_unbraced" in _S["violations"]
    cases.append("unbraced flip latch")
    cases.append("unbraced latch persists")

    clear_arms()
    set_arms(None)
    set_omega_pinion(None)
    _reset_episode()
    score(scene(sleeve=sleeve_at(40.0), sled=sled_at(0.4550)))
    assert score(scene(sleeve=sleeve_at(20.0), sled=sled_at(0.4550))) == 0.0
    assert "runaway_freewheel" in _S["violations"]
    cases.append("runaway freewheel")

    # Simulator-internal substep rates are not a scoring condition (only visible drum motion is).
    clear_arms()
    set_arms(None)
    _reset_episode()
    set_omega_pinion(4.5)
    assert score(scene(sled=sled_at(0.4550))) == 0.0 and "runaway_freewheel" not in _S["violations"]
    set_omega_pinion(None)
    cases.append("pinion substep rate is not scored")

    clear_arms()
    set_arms({
        "left": {
            "finger_gap": 0.026,
            "center": (SLED_START_X, 0.0, _wz(SLED_H)),
            "pad_straddle": -1.0,
            "closing_xz_ok": True,
        },
        "right": grip("right", held=False),
    })
    _reset_episode()
    assert score(scene()) == 0.0 and "sled_or_strap_tampered" in _S["violations"]
    cases.append("sled gripper tamper")

    clear_arms()
    set_arms(arms_hold(True, False))
    _reset_episode()
    assert score(scene(sled=sled_at(0.4550, h=0.100))) == 0.0 and "sled_or_strap_tampered" in _S["violations"]
    cases.append("sled lifted")

    clear_arms()
    set_arms(arms_hold(True, False))
    _reset_episode()
    assert score(scene(stripe=stripe_at(0.390))) == 0.0 and "overshoot_or_wind_wrong" in _S["violations"]
    cases.append("stripe overshoot")

    clear_arms()
    set_arms(arms_hold(True, False))
    _reset_episode()
    assert score(scene(stripe=stripe_at(0.320))) == 0.0 and "overshoot_or_wind_wrong" in _S["violations"]
    cases.append("stripe undershoot")

    seed_hold_then_magenta()
    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    moving = scene(
        paddle=paddle_at(38.0),
        sleeve=sleeve_at(-82.3, 0.0411),
        stripe=stripe_at(STRIPE_TARGET_X),
        sled=sled_at(0.479, vel=(0.05, 0, 0)),
    )
    got = score(moving)
    assert got == 0.75 and not success(moving), ("moving sled blocks 1.0", got)
    cases.append("moving sled blocks full")

    seed_hold_then_magenta()
    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    score(pay)
    set_arms(arms_clear())
    air = scene(
        paddle=body((0.014, 0.012, 0.052), PADDLE_DOWN[0], PADDLE_DOWN[1], PADDLE_DOWN[2] + 0.05),
        sleeve=sleeve_at(-82.3, 0.0411),
        stripe=stripe_at(STRIPE_TARGET_X),
        sled=sled_at(0.479),
    )
    # Air paddle must fail magenta seat and clear-rest success.
    assert not magenta_seated(air), "air paddle must not magenta_seated"
    air_sc = score(air)
    assert abs(air_sc - 0.0) < 1e-9, ("air paddle score", air_sc)
    assert not success(air), "air paddle must not success"
    cases.append("paddle held in air no full")

    clear_arms()
    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    _reset_episode()
    score(scene(paddle=paddle_at(0.0), sleeve=sleeve_at(-82.3, 0.0411)))
    st = scene(
        paddle=paddle_at(0.0),
        sleeve=sleeve_at(-82.3, 0.0411),
        stripe=stripe_at(STRIPE_TARGET_X),
        sled=sled_at(0.479),
    )
    assert score(st) == 0.0
    cases.append("green pawl at payout is 0")

    seed_hold_then_magenta()
    assert abs(score(scene(paddle=paddle_at(38.0))) - 0.50) < 1e-9
    cases.append("magenta end-cap 0.50")

    st2 = scene(paddle=paddle_at(10.0))
    assert abs(_instantaneous(st2) - 0.25) < 1e-9
    got = score(st2)
    assert abs(got - 0.0) < 1e-9, ("leave magenta band", got, dict(_S))
    cases.append("leave magenta band")

    seed_hold_then_magenta()
    assert abs(score(scene(paddle=paddle_at(38.0))) - 0.50) < 1e-9
    st_green = scene(paddle=paddle_at(0.0))
    assert abs(_instantaneous(st_green) - 0.25) < 1e-9
    assert abs(score(st_green) - 0.0) < 1e-9, ("after L_magenta leave band episode 0", dict(_S))
    cases.append("after L_magenta, leave band episode score 0.0")

    clear_arms()
    set_arms(arms_hold(False, True))
    _reset_episode()
    score(scene(paddle=paddle_at(0.0)))
    assert abs(score(scene(paddle=paddle_at(38.0))) - 0.50) < 1e-9
    cases.append("right arm counter-hold")

    clear_arms()
    set_arms({"left": grip("left", held=True, gap=0.040), "right": grip("right", held=False)})
    _reset_episode()
    assert score(scene(paddle=paddle_at(38.0))) == 0.0
    cases.append("wide fingers not hold")

    seed_hold_then_magenta()
    st = scene(paddle=paddle_at(38.0), sled=sled_at(0.470))
    assert abs(score(st) - 0.50) < 1e-9
    cases.append("sled short of chock keeps magenta 0.50")

    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    st = scene(
        paddle=paddle_at(38.0),
        sleeve=sleeve_at(-82.3, 0.0411),
        stripe=stripe_at(STRIPE_TARGET_X),
        sled=sled_at(0.470),
    )
    assert score(st) == 0.0
    cases.append("payout pose sled short scores 0")

    seed_hold_then_magenta()
    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    assert abs(score(pay) - 0.75) < 1e-9
    set_arms(arms_clear())
    assert abs(score(pay) - 0.75) < 1e-9 and not success(pay)
    assert_full_after_hold(pay, "release after payout")
    cases.append("release after payout holds 20 steps to 1.0")

    # One clear frame after L_payout must not bypass end_cap.
    seed_hold_then_magenta()
    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    assert abs(score(pay) - 0.75) < 1e-9 and _S["L_payout"]
    set_arms(arms_clear())
    one = score(pay)
    assert abs(one - 0.75) < 1e-9 and not success(pay) and abs(_end_cap() - 0.75) < 1e-9, (
        "one-frame clear after L_payout",
        one,
        dict(_S),
    )
    cases.append("one-frame clear after L_payout score 0.75")

    clear_arms()
    _reset_episode()
    assert score(scene()) == 0.0 and not _S["L_magenta"] and not _S["violations"]
    cases.append("new episode clears state")

    clear_arms()
    set_arms(arms_hold(True, False))
    _reset_episode()
    score(scene(stripe=stripe_at(0.390)))
    assert "overshoot_or_wind_wrong" in _S["violations"]
    assert score(scene(paddle=paddle_at(38.0), stripe=stripe_at(STRIPE_START_X))) == 0.0
    cases.append("overshoot latch persists")

    clear_arms()
    set_arms(arms_hold(True, False))
    _reset_episode()
    assert score(scene(sleeve=sleeve_at(-160.0))) == 0.0
    assert "overshoot_or_wind_wrong" in _S["violations"]
    cases.append("drum past -45")

    seed_hold_then_magenta()
    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    assert abs(score(pay) - 0.75) < 1e-9 and not success(pay)
    cases.append("still holding blocks 1.0")

    clear_arms()
    set_arms(None)
    _reset_episode()
    score(scene(sled=sled_at(0.4550)))
    assert score(scene(sled=sled_at(0.4580))) == 0.0 and "sled_or_strap_tampered" in _S["violations"]
    cases.append("unbraced sled step")

    # 1.5 mm drift that stays put must NOT latch cumulative (old bug: abs(x-braced) each step).
    clear_arms()
    set_arms(None)
    _reset_episode()
    score(scene(sled=sled_at(0.4550)))
    score(scene(sled=sled_at(0.4565)))  # +1.5 mm once
    for _ in range(4):
        score(scene(sled=sled_at(0.4565)))
    assert "sled_or_strap_tampered" not in _S["violations"], (
        "1.5mm stay must not latch",
        _S["unbraced_cum"],
        _S["violations"],
    )
    cases.append("1.5mm drift stay no tamper")

    # Multi-step cumulative path > 3.0 mm (1.8 + 1.8 mm <= 2.0 mm/step, |x - braced| <= 3.0 mm).
    clear_arms()
    set_arms(None)
    _reset_episode()
    score(scene(sled=sled_at(0.4550)))
    assert score(scene(sled=sled_at(0.4568))) == 0.0 and "sled_or_strap_tampered" not in _S["violations"]
    assert score(scene(sled=sled_at(0.4550))) == 0.0 and "sled_or_strap_tampered" in _S["violations"]
    cases.append("multi-step unbraced cumulative path triggers tamper")

    clear_arms()
    set_arms(arms_hold(True, False))
    _reset_episode()
    bad = body((0.014, 0.012, 0.052), MAGENTA_MOUTH[0] + 0.010, -0.0560, MAGENTA_MOUTH[2])
    assert score(scene(paddle=bad)) == 0.0
    cases.append("paddle 10 mm off magenta")

    seed_hold_then_magenta()
    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    st = scene(
        paddle=paddle_at(38.0),
        sleeve=sleeve_at(-82.3, 0.0411),
        stripe=stripe_at(SLED_REST_X),
        sled=sled_at(STRIPE_TARGET_X),
    )
    assert score(st) == 0.0
    cases.append("swapped stripe/sled no payout")

    clear_arms()
    set_arms({
        "left": {
            "finger_gap": 0.026,
            "center": (SLEEVE_START[0] + 0.05, SLEEVE_Y, _wz(SLEEVE_START[2])),
            "pad_straddle": -1.0,
            "closing_xz_ok": True,
        },
        "right": grip("right", held=False),
    })
    _reset_episode()
    assert score(scene(paddle=paddle_at(38.0))) == 0.0
    cases.append("empty air close not hold")

    seed_hold_then_magenta()
    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    score(pay)
    set_arms(arms_clear())
    spinning = scene(
        paddle=paddle_at(38.0),
        sleeve=sleeve_at(-82.3, 0.0411),
        stripe=stripe_at(STRIPE_TARGET_X),
        sled=sled_at(0.479),
    )
    spinning["sleeve"] = cyl_body(SLEEVE_OD / 2, SLEEVE_LEN, sleeve_pay[0], SLEEVE_Y, 0.0411, vel=(0.02, 0, 0))
    assert not success(spinning)
    cases.append("spinning sleeve blocks success")

    seed_hold_then_magenta()
    set_arms(arms_hold(False, True, sleeve=sleeve_pay))
    got = score(pay)
    assert abs(got - 0.0) < 1e-9, ("switched holding arm", got, dict(_S))
    cases.append("switched holding arm caps payout")

    # Demo-coupled kinematic score path: walk crank arc with couple_kinematics, expect 1.0 at rest.
    clear_arms()
    set_arms(arms_hold(True, False))
    _reset_episode()
    score(coupled_scene(40.0, 0.0, SLEEVE_START[2]))
    score(coupled_scene(40.0, 38.0, SLEEVE_START[2]))
    assert _S["L_magenta"] and _S["L_050"] and _S["held_before_pawl10"]
    for deg, hh in zip(CRANK_DEG, CRANK_H):
        loc = _crank_local(deg, r=_crank_radius(deg))
        sleeve = (PINION[0] + loc[0], SLEEVE_Y, hh)
        set_arms(arms_hold(True, False, sleeve=sleeve))
        st = coupled_scene(deg, 38.0, hh)
        coup = couple_kinematics_from_pinion(math.radians(40.0 - deg))
        st["stripe"] = stripe_at(coup["stripe_x"])
        st["sled"] = sled_at(min(coup["sled_x"], SLED_REST_X) if coup["sled_x"] < 0.478 else 0.479)
        got = score(st)
    assert abs(got - 0.75) < 1e-9, ("demo-coupled payout", got, coup, dict(_S))
    set_arms(arms_clear())
    st["sleeve"] = sleeve_at(-82.3, CRANK_H[-1])
    st["stripe"] = stripe_at(STRIPE_TARGET_X)
    st["sled"] = sled_at(0.479)
    got = score(st)
    assert abs(got - 0.75) < 1e-9 and not success(st), ("demo-coupled clear", got, dict(_S))
    assert_full_after_hold(st, "demo-coupled clear")
    cases.append("demo-coupled score path 0.75 success True")

    # Couple helper sanity at -82.3 deg crank.
    c = couple_kinematics_from_pinion(math.radians(40.0 - (-82.3)))
    assert abs(c["drum_deg"] + 29.829) < 0.05, c
    assert abs(c["stripe_x"] - STRIPE_TARGET_X) < 0.002, c
    cases.append("couple kinematics 25 mm payout")

    # Engaged green UP blocks payout-direction couple; wind-in and freewheel paths stay open.
    assert couple_mode(0.0, True, SLED_START_X + 0.010, SLED_START_X) == "refuse_payout"
    assert couple_mode(0.0, True, SLED_START_X, SLED_START_X) == "pinion_lead"
    assert couple_mode(0.0, True, SLED_START_X - 0.001, SLED_START_X) == "pinion_lead"
    assert couple_mode(0.0, False, SLED_START_X + 0.010, SLED_START_X) == "refuse_payout"
    assert couple_mode(38.0, True, SLED_START_X + 0.025, SLED_START_X) == "pinion_lead"
    assert couple_mode(38.0, False, SLED_START_X + 0.025, SLED_START_X) == "freewheel"
    cases.append("engaged UP refuse payout couple")

    # Blocker: Mech pawl joint at q_y = -38 deg must seat magenta (not only teleported blue_paddle).
    pawl_joint = next(j for j in WINCH.joints if j.name == "pawl")
    assert pawl_joint.low <= PAWL_DETENT_Q <= pawl_joint.high, (
        "pawl stops must surround magenta q_y",
        math.degrees(pawl_joint.low),
        math.degrees(pawl_joint.high),
    )
    clear_arms()
    set_arms(arms_hold(True, False))
    _reset_episode()
    mech_magenta = {
        "winch": MechState(
            pos=(0.30, 0.0, _wz(0.15)),
            quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            lin_vel=np.zeros(3),
            ang_vel=np.zeros(3),
            low=np.array([0.1, -0.1, _wz(0.0)]),
            high=np.array([0.5, 0.1, _wz(0.35)]),
            joints={
                "pinion": 0.0,
                "drum": 0.0,
                "pawl": PAWL_DETENT_Q,
                "sled": SLED_START_X - SLED_REST_X,
            },
            origin=np.array([0.0, 0.0, TOP]),
        ),
        "stripe": stripe_at(STRIPE_START_X),
    }
    assert magenta_seated(mech_magenta), (
        "Mech pawl at -38 deg must magenta_seated",
        theta_pawl_deg(mech_magenta),
    )
    got_mech = score(mech_magenta)
    assert abs(got_mech - 0.50) < 1e-9, ("Mech magenta score", got_mech, dict(_S))
    cases.append("Mech joint magenta_seated at q_y=-38")

    # Sled-led couple: unbraced spring advance moves drum >4 deg per step (runaway path).
    c0 = couple_kinematics_from_sled(SLED_START_X)
    c1 = couple_kinematics_from_sled(SLED_START_X + 0.004)
    assert abs(c1["drum_deg"] - c0["drum_deg"]) > 4.0 + EPS, (c0, c1)
    clear_arms()
    set_arms(None)
    _reset_episode()
    score(
        {
            "winch": MechState(
                pos=(0.30, 0.0, _wz(0.15)),
                quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
                low=np.array([0.1, -0.1, _wz(0.0)]),
                high=np.array([0.5, 0.1, _wz(0.35)]),
                joints={
                    "pinion": c0["pinion_rad"],
                    "drum": c0["drum_rad"],
                    "pawl": PAWL_DETENT_Q,
                    "sled": c0["sled_slide"],
                },
                origin=np.array([0.0, 0.0, TOP]),
            ),
            "stripe": stripe_at(c0["stripe_x"]),
            "blue_paddle": paddle_at(38.0),
            "sled": sled_at(c0["sled_x"]),
        }
    )
    st_fw = {
        "winch": MechState(
            pos=(0.30, 0.0, _wz(0.15)),
            quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            lin_vel=np.zeros(3),
            ang_vel=np.zeros(3),
            low=np.array([0.1, -0.1, _wz(0.0)]),
            high=np.array([0.5, 0.1, _wz(0.35)]),
            joints={
                "pinion": c1["pinion_rad"],
                "drum": c1["drum_rad"],
                "pawl": PAWL_DETENT_Q,
                "sled": c1["sled_slide"],
            },
            origin=np.array([0.0, 0.0, TOP]),
        ),
        "stripe": stripe_at(c1["stripe_x"]),
        "blue_paddle": paddle_at(38.0),
        "sled": sled_at(c1["sled_x"]),
    }
    assert score(st_fw) == 0.0 and "runaway_freewheel" in _S["violations"]
    cases.append("sled-led freewheel runaway")

    # Detent retargeting: pure map + live set_joint_target writes (not constant-only asserts).
    assert PAWL_RIDGE_DEG == 20.0 and abs(math.degrees(PAWL_DETENT_Q) + 38.0) < 1e-9
    assert abs(pawl_detent_target(19.9)) < 1e-12
    assert abs(pawl_detent_target(20.0) - PAWL_DETENT_Q) < 1e-12
    assert abs(pawl_detent_target(38.0) - PAWL_DETENT_Q) < 1e-12
    clear_joint_targets()
    env = _OfflineEnv()
    apply_pawl_detent(env, 10.0)
    assert abs(float(get_joint_target("winch", "pawl"))) < 1e-12, get_joint_target("winch", "pawl")
    apply_pawl_detent(env, 20.0)
    assert abs(float(get_joint_target("winch", "pawl")) - PAWL_DETENT_Q) < 1e-12
    apply_pawl_detent(env, 38.0)
    assert abs(float(get_joint_target("winch", "pawl")) - PAWL_DETENT_Q) < 1e-12
    cases.append("pawl over-center detent retarget")

    # Mech joint rates must block 1.0 even when free bodies look still (keep hold so tier stays 0.75).
    seed_hold_then_magenta()
    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    still = scene(
        paddle=paddle_at(38.0),
        sleeve=sleeve_at(-82.3, 0.0411),
        stripe=stripe_at(STRIPE_TARGET_X),
        sled=sled_at(0.479),
    )
    set_joint_vel({"pinion": 0.8, "drum": 0.0, "pawl": 0.0, "sled": 0.0})
    assert abs(score(still) - 0.75) < 1e-9 and not success(still), ("spinning pinion blocks 1.0", dict(_S))
    set_joint_vel({"pinion": 0.0, "drum": 0.0, "pawl": 0.0, "sled": 0.02})
    assert abs(score(still) - 0.75) < 1e-9 and not success(still), ("sliding sled joint blocks 1.0", dict(_S))
    set_joint_vel({"pinion": 0.0, "drum": 0.0, "pawl": 0.0, "sled": 0.0})
    set_arms(arms_clear())
    assert abs(score(still) - 0.75) < 1e-9 and not success(still)
    assert_full_after_hold(still, "joint rates zero")
    cases.append("Mech joint rates gate bodies_resting")

    # Primitive tilts, joint limits, and reset/step hook ordering.
    import arm_watch
    import mechkit

    ratchet_s = next(s for s in WINCH.shapes if s.kind == "cyl")
    assert abs((_quat_wxyz_matrix(ratchet_s.tilt) @ np.array([0.0, 0.0, 1.0]))[1] - 1.0) < 1e-6
    pin_j = next(j for j in WINCH.joints if j.name == "pinion")
    assert pin_j.low <= -math.radians(30.0) and pin_j.high >= math.radians(125.0)
    pin_cyl_s = next(s for s in pin_j.shapes if s.kind == "cyl" and s.color == GEAR)
    assert abs((_quat_wxyz_matrix(pin_cyl_s.tilt) @ np.array([0.0, 0.0, 1.0]))[1] - 1.0) < 1e-6
    crank_s = next(s for s in pin_j.shapes if s.kind == "box")
    crank_dir = _quat_wxyz_matrix(crank_s.tilt) @ np.array([1.0, 0.0, 0.0])
    assert abs(crank_dir[0] - math.cos(math.radians(40.0))) < 1e-6 and abs(crank_dir[2] - math.sin(math.radians(40.0))) < 1e-6
    drum_j = next(j for j in WINCH.joints if j.name == "drum")
    for s in drum_j.shapes:
        if s.kind == "cyl":
            assert abs((_quat_wxyz_matrix(s.tilt) @ np.array([0.0, 0.0, 1.0]))[1] - 1.0) < 1e-6
    flanges = [s for s in drum_j.shapes if s.kind == "cyl" and s.color == DRUM_Y and abs(s.size[0] - 0.057) < 1e-6]
    assert len(flanges) == 2 and sorted(round(s.at[1], 4) for s in flanges) == [-0.0290, 0.0290], flanges
    assert all(abs(s.size[2] - 0.002) < 1e-6 for s in flanges), flanges
    spokes = [s for s in drum_j.shapes if s.kind == "box" and s.color == BLACK]
    assert len(spokes) == 2 and all(abs(s.at[1] - 0.0310) < 1e-6 for s in spokes), spokes
    assert all(math.hypot(s.at[0], s.at[2]) + s.size[2] / 2.0 <= 0.057 + 1e-6 for s in spokes), spokes
    spoke30_s = next(s for s in spokes if abs(s.at[0]) > 1e-4)
    spoke30_dir = _quat_wxyz_matrix(spoke30_s.tilt) @ np.array([0.0, 0.0, 1.0])
    assert abs(spoke30_dir[0] - math.sin(math.radians(30.0))) < 1e-6 and abs(spoke30_dir[2] - math.cos(math.radians(30.0))) < 1e-6
    assert _seed_winch_start in mechkit._RESET_HOOKS
    assert arm_watch._STEP.index(_seed_once) < arm_watch._STEP.index(_watch_and_couple)
    cases.append("primitive tilts, drum flanges/spokes, pinion limits, and reset hooks")

    # Jaw closing axis R[:, 0], pad straddle, and open-pad strap halo checks.
    for deg in CRANK_DEG:
        R_l = _left_grip(deg)
        assert abs(float(R_l[1, 0])) <= math.sin(math.radians(30.0)) + EPS
        loc_d = _crank_local(deg, r=_crank_radius(deg))
        loc_r = _crank_local(deg, r=R_CRANK)
        tcp_d = np.array([PINION[0] + loc_d[0], SLEEVE_Y, _wz(PINION[2] + loc_d[2])])
        ax_r = np.array([PINION[0] + loc_r[0], SLEEVE_Y, _wz(PINION[2] + loc_r[2])])
        assert float(np.linalg.norm(tcp_d[[0, 2]] - ax_r[[0, 2]])) <= GRIP_AXIS_TOL + EPS
        assert _pad_straddle(tcp_d, 0.026, R_l[:, 0], ax_r) < 0.0
    assert abs(float(_right_grip()[1, 0])) > math.sin(math.radians(30.0))
    ax0 = np.array([SLEEVE_START[0], SLEEVE_Y, _wz(SLEEVE_START[2])])
    off_tcp = ax0 + 0.014 * _left_grip(40.0)[:, 0]  # 14 mm offset along close_axis exceeds 13 mm half-gap
    assert _pad_straddle(off_tcp, 0.026, _left_grip(40.0)[:, 0], ax0) > 0.0
    cases.append("jaw close axis R[:,0] and pad straddle")

    clear_arms()
    _reset_episode()
    # Open gripper 25 mm clear of strap edge in Y: TCP > 5 mm clear, but open pad enters < 5 mm strap halo.
    set_arms({
        "left": {
            "finger_gap": 0.094,
            "center": (0.3375, 0.050, _wz(STRIPE_H)),
            "rot": grip_rotation(0.0),
            "pad_straddle": 1.0,
            "closing_xz_ok": False,
        },
        "right": grip("right", held=False),
    })
    assert score(scene()) == 0.0 and "sled_or_strap_tampered" in _S["violations"]
    cases.append("open finger pad triggers strap <5 mm tamper")

    # Surface-based hands-clear rejects grippers 15 mm from sled +x face or above drum.
    seed_hold_then_magenta()
    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    score(pay)
    near_sled = {
        "left": grip("left", clear=True),
        "right": grip("right", clear=True, center=(SLED_REST_X + 0.040 + 0.015, 0.0, _wz(SLED_H))),
    }
    set_arms(near_sled)
    assert not grippers_open_and_clear(pay) and not success(pay)
    near_drum = {
        "left": grip("left", clear=True, center=(DRUM[0], 0.0, _wz(DRUM[2] + 0.057 + 0.015))),
        "right": grip("right", clear=True),
    }
    set_arms(near_drum)
    assert not grippers_open_and_clear(pay) and not success(pay)
    cases.append("surface keepout blocks near sled/drum")

    # Live MechState-only demo-coupled arc (no free sleeve/paddle/sled proxies) with real R[:, 0] grips.
    def mech_only_state(pinion_rad, pawl_q, sled_slide, st_x):
        return {
            "winch": MechState(
                pos=(0.30, 0.0, _wz(0.15)),
                quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
                low=np.array([0.1, -0.1, _wz(0.0)]),
                high=np.array([0.5, 0.1, _wz(0.35)]),
                joints={
                    "pinion": pinion_rad,
                    "drum": -pinion_rad / GEAR_RATIO,
                    "pawl": pawl_q,
                    "sled": sled_slide,
                },
                origin=np.array([0.0, 0.0, TOP]),
            ),
            "stripe": stripe_at(st_x),
        }

    def mech_left_arm(deg, hh):
        rot = _left_grip(deg)
        loc = _crank_local(deg, r=_crank_radius(deg))
        tcp = np.array([PINION[0] + loc[0], SLEEVE_Y, _wz(hh)])
        ax_r = np.array([_SLEEVE_XH["x"], SLEEVE_Y, _wz(_SLEEVE_XH["h"])])
        return {
            "left": {
                "finger_gap": 0.022,
                "center": tuple(map(float, tcp)),
                "close_axis": tuple(map(float, rot[:, 0])),
                "pad_straddle": _pad_straddle(tcp, 0.022, rot[:, 0], ax_r),
                "closing_xz_ok": abs(float(rot[1, 0])) <= math.sin(math.radians(30.0)) + EPS,
                "pad_points": _pad_probe_points(tcp, 0.022, rot),
            },
            "right": grip("right", held=False),
        }

    clear_arms()
    _reset_episode()
    _sync_live(mech_only_state(0.0, 0.0, SLED_START_X - SLED_REST_X, STRIPE_START_X))
    set_arms(mech_left_arm(40.0, SLEEVE_START[2]))
    score(mech_only_state(0.0, 0.0, SLED_START_X - SLED_REST_X, STRIPE_START_X))
    assert abs(score(mech_only_state(0.0, PAWL_DETENT_Q, SLED_START_X - SLED_REST_X, STRIPE_START_X)) - 0.50) < 1e-9
    for deg, hh in zip(CRANK_DEG, CRANK_H):
        pin_r = math.radians(40.0 - deg)
        coup = couple_kinematics_from_pinion(pin_r)
        st_m = mech_only_state(pin_r, PAWL_DETENT_Q, coup["sled_slide"], coup["stripe_x"])
        _sync_live(st_m)
        set_arms(mech_left_arm(deg, hh))
        got_m = score(st_m)
    assert abs(got_m - 0.75) < 1e-9, ("MechState demo-coupled payout", got_m, dict(_S))
    set_arms(arms_clear())
    assert abs(score(st_m) - 0.75) < 1e-9 and not success(st_m), ("MechState demo-coupled clear", dict(_S))
    assert_full_after_hold(st_m, "MechState demo-coupled clear")
    cases.append("MechState-only demo-coupled arc to 1.0")

    # Couple sled round-trip sign check.
    rt = couple_kinematics_from_sled(SLED_REST_X)
    assert rt["pinion_rad"] > 0.0 and rt["drum_rad"] < 0.0 and abs(rt["crank_deg"] + 82.35) < 0.2, rt
    cases.append("couple_kinematics_from_sled sign check")

    # Blocker 1 (round 11): withdrawing (< 50 mm clear) after L_payout must preserve 0.75.
    seed_hold_then_magenta()
    set_arms(arms_hold(True, False, sleeve=sleeve_pay))
    assert abs(score(pay) - 0.75) < 1e-9 and _S["L_payout"]
    withdraw_25mm = {
        "left": {
            "finger_gap": 0.060,
            "center": (sleeve_pay[0], SLEEVE_Y + 0.025, _wz(sleeve_pay[2])),
            "close_axis": tuple(map(float, _left_grip(-82.3)[:, 0])),
            "pad_straddle": 1.0,
            "closing_xz_ok": True,
            "pad_points": _pad_probe_points(
                np.array([sleeve_pay[0], SLEEVE_Y + 0.025, _wz(sleeve_pay[2])]),
                0.060,
                _left_grip(-82.3),
            ),
        },
        "right": grip("right", clear=True),
    }
    set_arms(withdraw_25mm)
    got_wd = score(pay)
    assert _S["hold_broken_after_pawl10"] and not grippers_open_and_clear(pay)
    assert abs(got_wd - 0.75) < 1e-9 and not success(pay), ("withdraw 25 mm after L_payout", got_wd, dict(_S))
    cases.append("withdraw 25 mm after L_payout keeps 0.75")

    # Blocker 3 (round 11): drum_deg reads drum before pinion; refuse_payout clamps pinion/drum/sled.
    uncoupled = mech_only_state(math.radians(30.0), 0.0, SLED_START_X - SLED_REST_X, STRIPE_START_X)
    uncoupled["winch"] = MechState(
        pos=uncoupled["winch"].pos,
        quat_xyzw=uncoupled["winch"].quat_xyzw,
        lin_vel=np.zeros(3),
        ang_vel=np.zeros(3),
        low=uncoupled["winch"].low,
        high=uncoupled["winch"].high,
        joints={"pinion": math.radians(30.0), "drum": 0.0, "pawl": 0.0, "sled": SLED_START_X - SLED_REST_X},
        origin=np.array([0.0, 0.0, TOP]),
    )
    assert abs(drum_deg(uncoupled)) < 1e-9, drum_deg(uncoupled)
    clear_arms()
    _reset_episode()
    assert score(uncoupled) == 0.0 and "runaway_freewheel" not in _S["violations"], _S["violations"]
    cases.append("drum_deg reads drum before pinion")

    class _FakeTensor:
        def __init__(self, arr):
            self._arr = np.asarray(arr, dtype=float)

        def clone(self):
            return _FakeTensor(self._arr.copy())

        def __getitem__(self, idx):
            return self._arr[idx]

        def __setitem__(self, idx, val):
            self._arr[idx] = val

        def detach(self):
            return self

        def cpu(self):
            return self

        def numpy(self):
            return self._arr

    class _FakeWinchAsset:
        def __init__(self, q_dict, qd_dict=None):
            self.joint_names = ["pinion_joint", "drum_joint", "pawl_joint", "sled_joint"]
            q_arr = [[q_dict[n.removesuffix("_joint")] for n in self.joint_names]]
            qd_arr = [[(qd_dict or {}).get(n.removesuffix("_joint"), 0.0) for n in self.joint_names]]
            self._pos = _FakeTensor(q_arr)
            self._vel = _FakeTensor(qd_arr)
            self.data = type("_Data", (), {"joint_pos": self._pos, "joint_vel": self._vel})()

        def find_joints(self, pattern):
            return ([self.joint_names.index(pattern)], [pattern]) if pattern in self.joint_names else ([], [])

        def write_joint_position_to_sim(self, pos):
            self._pos = _FakeTensor(pos.numpy().copy())
            self.data.joint_pos = self._pos

        def write_joint_velocity_to_sim(self, vel):
            self._vel = _FakeTensor(vel.numpy().copy())
            self.data.joint_vel = self._vel

    class _FakeScene:
        def __init__(self, winch_asset):
            self.articulations = {"winch": winch_asset}
            self.rigid_objects = {}
            self.env_origins = _FakeTensor([[0.0, 0.0, 0.0]])

        def __getitem__(self, key):
            return self.articulations[key]

    class _FakeEnv:
        def __init__(self, winch_asset):
            self.unwrapped = type("_U", (), {"scene": _FakeScene(winch_asset)})()

    # refuse_payout clamps pinion back to couple_kinematics_from_sled(sled_now).
    clear_arms()
    _reset_episode()
    w_refuse = _FakeWinchAsset({"pinion": 0.25, "drum": 0.0, "pawl": 0.0, "sled": SLED_START_X - SLED_REST_X})
    _watch_and_couple(_FakeEnv(w_refuse))
    assert abs(float(w_refuse.data.joint_pos[0, 0])) < 1e-9, w_refuse.data.joint_pos[0, 0]
    assert abs(float(w_refuse.data.joint_pos[0, 1])) < 1e-9, w_refuse.data.joint_pos[0, 1]
    assert abs(float(w_refuse.data.joint_pos[0, 3]) - (SLED_START_X - SLED_REST_X)) < 1e-9
    cases.append("refuse_payout clamps pinion to sled lock")

    # Blocker 4 (round 11): _reset_episode resets _SLEEVE_XH & _LIVE; _watch_and_couple updates _SLEEVE_XH before read_arms.
    _SLEEVE_XH["x"] = sleeve_pay[0]
    _SLEEVE_XH["h"] = sleeve_pay[2]
    _LIVE["sled"] = np.array([SLED_REST_X, 0.0, _wz(SLED_H)])
    _LIVE["stripe"] = np.array([STRIPE_TARGET_X, 0.0, _wz(STRIPE_H)])
    _LIVE["strap"] = np.zeros(3)
    _reset_episode()
    assert abs(_SLEEVE_XH["x"] - SLEEVE_START[0]) < 1e-9 and abs(_SLEEVE_XH["h"] - SLEEVE_START[2]) < 1e-9
    assert abs(float(_LIVE["sled"][0]) - SLED_START_X) < 1e-9
    assert abs(float(_LIVE["stripe"][0]) - STRIPE_START_X) < 1e-9
    assert abs(float(_LIVE["strap"][2]) - _wz(STRAP_H)) < 1e-9

    seen_sleeve_xh = []
    orig_read_arms = read_arms
    try:
        def _probe_read_arms(env):
            seen_sleeve_xh.append((_SLEEVE_XH["x"], _SLEEVE_XH["h"]))
            return arms_hold(True, False, sleeve=sleeve_pay)

        globals()["read_arms"] = _probe_read_arms
        w_live = _FakeWinchAsset(
            {
                "pinion": math.radians(40.0 - (-82.3)),
                "drum": math.radians(-29.829),
                "pawl": PAWL_DETENT_Q,
                "sled": 0.4790 - SLED_REST_X,
            }
        )
        _watch_and_couple(_FakeEnv(w_live))
        exp_loc = _crank_local(-82.3, r=R_CRANK)
        exp_x, exp_h = PINION[0] + exp_loc[0], PINION[2] + exp_loc[2]
        assert len(seen_sleeve_xh) == 1
        assert abs(seen_sleeve_xh[0][0] - exp_x) < 1e-6 and abs(seen_sleeve_xh[0][1] - exp_h) < 1e-6, seen_sleeve_xh
    finally:
        globals()["read_arms"] = orig_read_arms
    cases.append("reset_episode and pre-read_arms _SLEEVE_XH sync")

    # 20 Hz ArmPlan.target(t) interpolated crank_held_active check using real sleeve_grip (19 mm command, jaws stop at 26 mm).
    from yam_arena import YAM_URDF, ik, station
    from yam_arena.constants import ARM_JOINTS
    from yam_arena.demo import ArmPlan
    from yam_arena.urdf import forward_kinematics, rpy_matrix

    demo_wps = demo()
    hz = 20.0
    left_plan = ArmPlan(list(demo_wps["left"]), station.BASES["left"])
    crank_t0 = sum(w.duration for w in left_plan.waypoints[:7])
    crank_segs = []
    t_acc = crank_t0
    prev_d = 40.0
    for idx_c, w in enumerate(left_plan.waypoints[7 : 7 + len(CRANK_DEG)]):
        deg_c = float(CRANK_DEG[idx_c])
        crank_segs.append((t_acc, t_acc + w.duration, prev_d, deg_c, w.duration))
        t_acc += w.duration
        prev_d = deg_c
    hold_t0 = sum(w.duration for w in left_plan.waypoints[:5])  # start of hold sleeve
    crank_t1 = t_acc  # end of crank -82.3

    clear_arms()
    _reset_episode()
    for k in range(round(left_plan.duration * hz)):
        t = (k + 1) / hz
        pos, rot, finger, lbl = left_plan.target(t)
        if not (hold_t0 + 1e-6 <= t <= crank_t1 + 1e-6):
            continue
        if t < crank_t0:
            deg_s = 40.0
        else:
            deg_s = CRANK_DEG[-1]
            for s0, s1, d0, d1, dur in crank_segs:
                if t < s1:
                    deg_s = d0 + ik.smoothstep((t - s0) / dur) * (d1 - d0)
                    break
        loc_s = _crank_local(deg_s, r=R_CRANK)
        ax_s = np.array([PINION[0] + loc_s[0], SLEEVE_Y, _wz(PINION[2] + loc_s[2])])
        ang_proj = math.atan2((pos[2] - TOP) - PINION[2], pos[0] - PINION[0])
        ax_p = np.array([PINION[0] + R_CRANK * math.cos(ang_proj), SLEEVE_Y, _wz(PINION[2] + R_CRANK * math.sin(ang_proj))])
        gap = max(2.0 * abs(float(finger)), SLEEVE_OD)  # jaws stop on the 26 mm sleeve
        ca = rot[:, 0]
        for mode_tag, ax in (("slerp", ax_s), ("proj", ax_p)):
            _SLEEVE_XH["x"] = float(ax[0])
            _SLEEVE_XH["h"] = float(ax[2]) - TOP
            set_arms(
                {
                    "left": {
                        "finger_gap": gap,
                        "center": tuple(map(float, pos)),
                        "close_axis": tuple(map(float, ca)),
                        "pad_straddle": _pad_straddle(pos, gap, ca, ax),
                        "closing_xz_ok": abs(float(ca[1])) <= math.sin(math.radians(30.0)) + EPS,
                        "pad_points": _pad_probe_points(pos, gap, rot),
                    },
                    "right": grip("right", held=False),
                }
            )
            assert crank_held_active("left"), (
                f"20 Hz crank_held_active failed ({mode_tag})",
                round(t, 3),
                lbl,
                round(deg_s, 2),
                round(float(np.linalg.norm(pos[[0, 2]] - ax[[0, 2]])) * 1000.0, 2),
                _pad_straddle(pos, gap, ca, ax),
            )
        pawl_q = PAWL_DETENT_Q if t >= 15.0 else 0.0
        pin_r = math.radians(40.0 - deg_s)
        coup_s = couple_kinematics_from_pinion(pin_r)
        st_i = mech_only_state(pin_r, pawl_q, coup_s["sled_slide"], coup_s["stripe_x"])
        _sync_live(st_i)
        got_i = score(st_i)
        assert not _S["violations"], (round(t, 3), lbl, dict(_S))
    assert abs(got_i - 0.75) < 1e-9, (got_i, dict(_S))
    set_arms(arms_clear())
    assert abs(score(st_i) - 0.75) < 1e-9 and not success(st_i)
    assert_full_after_hold(st_i, "20 Hz plan clear")
    cases.append("20 Hz ArmPlan interpolated crank_held_active and payout")

    # 20 Hz interpolated ArmPlan URDF link-hull collision check against WINCH colliders and blue_paddle.
    import xml.etree.ElementTree as ET
    from pathlib import Path

    urdf_root = ET.parse(YAM_URDF).getroot()
    stl_dtype = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    link_hulls = {}
    for link in urdf_root.iter("link"):
        lname = link.get("name")
        if lname == "base":
            continue
        pts_all = []
        for col in link.findall("collision"):
            orig = col.find("origin")
            xyz = np.array([float(v) for v in orig.get("xyz").split()]) if orig is not None and orig.get("xyz") else np.zeros(3)
            rpy = np.array([float(v) for v in orig.get("rpy").split()]) if orig is not None and orig.get("rpy") else np.zeros(3)
            R_c = rpy_matrix(rpy)
            stl_p = Path(YAM_URDF).parent / col.find("geometry/mesh").get("filename")
            raw = stl_p.read_bytes()
            ntri = int.from_bytes(raw[80:84], "little")
            arr = np.frombuffer(raw, dtype=stl_dtype, offset=84, count=ntri)
            u = np.unique(arr["v"].reshape(-1, 3), axis=0)
            pts_all.append(u @ R_c.T + xyz)
        v_stack = np.vstack(pts_all)
        lo, hi = v_stack.min(0), v_stack.max(0)
        corners = np.array(
            [[lo[0] if i & 1 else hi[0], lo[1] if i & 2 else hi[1], lo[2] if i & 4 else hi[2]] for i in range(8)]
        )
        link_hulls[lname] = (v_stack, corners)

    def _box_sdf(pts: np.ndarray, center, size, rot=None) -> float:
        r = np.eye(3) if rot is None else rot
        local = (pts - np.asarray(center, float)) @ r
        q = np.abs(local) - np.asarray(size, float) / 2.0
        return float((np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(np.max(q, axis=1), 0.0)).min())

    def _cyl_y_sdf(pts: np.ndarray, center, radius: float, half_len: float) -> float:
        c = np.asarray(center, float)
        dy = np.abs(pts[:, 1] - c[1]) - half_len
        dr = np.hypot(pts[:, 0] - c[0], pts[:, 2] - c[2]) - radius
        return float((np.hypot(np.maximum(dr, 0.0), np.maximum(dy, 0.0)) + np.minimum(np.maximum(dr, dy), 0.0)).min())

    winch_boxes = []
    winch_cyls = []
    for si, sh in enumerate(WINCH.shapes):
        if not sh.collide:
            continue
        c = np.array([sh.at[0], sh.at[1], _wz(sh.at[2])])
        if sh.kind == "box":
            half = np.asarray(sh.size, float) / 2.0
            winch_boxes.append((si, c, np.asarray(sh.size, float), c - half, c + half))
        else:
            r_c, hl = float(sh.size[0]), float(sh.size[2]) / 2.0
            half = np.array([r_c, hl, r_c])
            winch_cyls.append((si, c, r_c, hl, c - half, c + half))

    end_rad = math.radians(-82.3)
    sleeve_end = np.array([PINION[0] + R_CRANK * math.cos(end_rad), SLEEVE_Y, _wz(PINION[2] + R_CRANK * math.sin(end_rad))])
    crank_end_c = np.array([PINION[0] + 0.5 * R_CRANK * math.cos(end_rad), 0.0550, _wz(PINION[2] + 0.5 * R_CRANK * math.sin(end_rad))])
    crank_end_rot = _ry(82.3)
    paddle_up_c = np.array([PADDLE_UP[0], PADDLE_UP[1], _wz(PADDLE_UP[2])])
    paddle_dn_c = np.array([PADDLE_DOWN[0], PADDLE_DOWN[1], _wz(PADDLE_DOWN[2])])
    paddle_dn_rot = _ry(-38.0)

    for arm_name in ("left", "right"):
        plan = ArmPlan(list(demo_wps[arm_name]), station.BASES[arm_name])
        bp = np.asarray(station.BASES[arm_name], float)
        for k in range(round(plan.duration * hz)):
            t = (k + 1) / hz
            act, lbl = plan.action7(t)
            _, _, finger, _ = plan.target(t)
            qmap = {nm: float(v) for nm, v in zip(ARM_JOINTS, act[:6])}
            qmap["joint7"] = float(finger)
            qmap["joint8"] = float(finger)
            fk = forward_kinematics(YAM_URDF, qmap)
            for lname, (verts, corners) in link_hulls.items():
                Tmat = fk[lname]
                wc = corners @ Tmat[:3, :3].T + Tmat[:3, 3] + bp
                wlo, whi = wc.min(0), wc.max(0)
                wpts = None
                for si, c, sz, slo, shi in winch_boxes:
                    if np.all(whi >= slo) and np.all(wlo <= shi):
                        if wpts is None:
                            wpts = verts @ Tmat[:3, :3].T + Tmat[:3, 3] + bp
                        sd = _box_sdf(wpts, c, sz)
                        assert sd >= -1e-6, (arm_name, round(t, 2), lbl, lname, f"WINCH.shapes[{si}]", sd * 1000.0)
                for si, c, r_c, hl, slo, shi in winch_cyls:
                    if np.all(whi >= slo) and np.all(wlo <= shi):
                        if wpts is None:
                            wpts = verts @ Tmat[:3, :3].T + Tmat[:3, 3] + bp
                        sd = _cyl_y_sdf(wpts, c, r_c, hl)
                        assert sd >= -1e-6, (arm_name, round(t, 2), lbl, lname, f"WINCH.shapes[{si}]", sd * 1000.0)
                if arm_name == "right" and lbl not in ("grip paddle", "flip magenta", "seat", "release pawl"):
                    if lbl in ("right park turn", "hover pawl", "wait hold", "over paddle", "to paddle", "settle paddle"):
                        pc, pr = paddle_up_c, None
                    else:
                        pc, pr = paddle_dn_c, paddle_dn_rot
                    if np.all(whi >= pc - 0.04) and np.all(wlo <= pc + 0.04):
                        if wpts is None:
                            wpts = verts @ Tmat[:3, :3].T + Tmat[:3, 3] + bp
                        sd_p = _box_sdf(wpts, pc, (0.014, 0.012, 0.052), pr)
                        assert sd_p >= -1e-6, (arm_name, round(t, 2), lbl, lname, "blue_paddle", sd_p * 1000.0)
                if arm_name == "left" and lbl in ("lift sleeve", "pull out", "left clear", "rest hold"):
                    if np.all(whi >= sleeve_end - 0.06) and np.all(wlo <= sleeve_end + 0.06):
                        if wpts is None:
                            wpts = verts @ Tmat[:3, :3].T + Tmat[:3, 3] + bp
                        sd_s = _cyl_y_sdf(wpts, sleeve_end, SLEEVE_OD / 2.0, SLEEVE_LEN / 2.0)
                        assert sd_s >= -1e-6, (arm_name, round(t, 2), lbl, lname, "sleeve_end", sd_s * 1000.0)
                    if np.all(whi >= crank_end_c - 0.12) and np.all(wlo <= crank_end_c + 0.12):
                        if wpts is None:
                            wpts = verts @ Tmat[:3, :3].T + Tmat[:3, 3] + bp
                        sd_c = _box_sdf(wpts, crank_end_c, (0.200, 0.018, 0.010), crank_end_rot)
                        assert sd_c >= -1e-6, (arm_name, round(t, 2), lbl, lname, "crank_end", sd_c * 1000.0)
    cases.append("20 Hz ArmPlan URDF link-hull collision check")

    # - new cases: grip margins, preload, rest lock, friction, geometry -------------------------------------------
    _reset_episode()
    for deg_h in (0.0210, 0.0265, GRIP_GAP_MAX):
        clear_arms()
        set_arms({"left": grip("left", held=True, gap=deg_h), "right": grip("right", held=False)})
        assert crank_held_active("left"), ("gap should hold", deg_h)
    for gap_bad in (0.0195, 0.0280, 0.0300):
        set_arms({"left": grip("left", held=True, gap=gap_bad), "right": grip("right", held=False)})
        assert not crank_held_active("left"), ("gap must not hold", gap_bad)
    cases.append("grip gap window 20 to 27.5 mm (no closing on air at 28 to 30 mm)")

    ax_s = np.array([SLEEVE_START[0], SLEEVE_Y, _wz(SLEEVE_START[2])])
    for off, want in ((0.020, True), (0.024, False)):
        set_arms({"left": {"finger_gap": 0.026, "center": tuple(map(float, ax_s + np.array([0.0, 0.0, off]))), "pad_straddle": -1.0, "closing_xz_ok": True}, "right": grip("right", held=False)})
        assert crank_held_active("left") == want, ("axis tolerance", off)
    cases.append("grip axis tolerance 22 mm")

    friction = {j.name: j.friction for j in WINCH.joints}
    assert friction["pinion"] is None and friction["pawl"] is None, friction
    assert WINCH.friction == 1.0
    cases.append("sleeve and paddle inherit mu 1.0 (no slippery override)")

    assert SUCCESS_HOLD_S <= DT + EPS and HOLD_STEPS == 20
    cases.append("success hold is the 20 step clear-rest counted in score()")

    sw = demo()
    grips = {w.label: w.finger for w in sw["left"] if w.label in ("close sleeve", "hold sleeve")}
    assert all(abs(v + (SLEEVE_OD - 0.007) / 2) < 1e-9 for v in grips.values()), grips
    assert abs(next(w.finger for w in sw["right"] if w.label == "grip paddle") + (0.012 - 0.007) / 2) < 1e-9
    cases.append("demo grips are 7 mm narrower than sleeve and paddle")

    # Preload torque: drive target leads the pinion toward payout only while held under load.
    clear_joint_targets()
    env_o = _OfflineEnv()
    t_load = apply_pinion_preload(env_o, 0.5, True)
    assert abs(t_load - (0.5 + PRELOAD_TORQUE / PINION_DRIVE_K)) < 1e-12 and abs(get_joint_target("winch", "pinion") - t_load) < 1e-12
    assert abs(apply_pinion_preload(env_o, 0.5, False) - 0.5) < 1e-12
    assert abs(PINION_DRIVE_K * (t_load - 0.5) - PRELOAD_TORQUE) < 1e-12
    cases.append("crank preload torque target (held under load only)")

    # Rest lock: sled on the chock, pawl magenta, hands off -> pinned joints report zero rate in one step.
    clear_arms()
    _reset_episode()
    w_rest = _FakeWinchAsset(
        {"pinion": math.radians(122.35), "drum": math.radians(-29.84), "pawl": PAWL_DETENT_Q, "sled": 0.4800 - SLED_REST_X},
        {"pinion": 2.9, "drum": 0.0, "pawl": 0.0, "sled": 0.0},
    )
    _watch_and_couple(_FakeEnv(w_rest))
    assert _S["joint_vel"]["pinion"] == 0.0 and _S["joint_vel"]["drum"] == 0.0 and _S["joint_vel"]["sled"] == 0.0, _S["joint_vel"]
    assert abs(get_joint_target("winch", "pinion") - float(w_rest.data.joint_pos[0, 0])) < 1e-9
    cases.append("rest lock at the chock reports zero pinion rate")

    # Held under load with the pawl past the ridge: the pinion drive gets the preload offset.
    clear_arms()
    _reset_episode()
    probe_pin = math.radians(60.0)
    w_load = _FakeWinchAsset({"pinion": probe_pin, "drum": -probe_pin / GEAR_RATIO, "pawl": PAWL_DETENT_Q, "sled": couple_kinematics_from_pinion(probe_pin)["sled_slide"]})
    globals()["read_arms"] = lambda env: arms_hold(True, False, sleeve=(PINION[0] + _crank_local(40.0 - 60.0)[0], SLEEVE_Y, PINION[2] + _crank_local(40.0 - 60.0)[2]))
    try:
        _watch_and_couple(_FakeEnv(w_load))
    finally:
        globals()["read_arms"] = orig_read_arms
    assert abs(get_joint_target("winch", "pinion") - (probe_pin + PRELOAD_TORQUE / PINION_DRIVE_K)) < 1e-9, get_joint_target("winch", "pinion")
    cases.append("held crank under load gets preload drive target")

    # Geometry: no two non-jointed colliders interpenetrate at the start pose, at full payout, or along the crank arc.
    viol, allowed_hits, below = geometry_check()
    assert not viol, ("start pose interpenetration", viol)
    assert not below, ("collider below table top", below)
    cases.append("geometry start pose: no interpenetration >0.5 mm, nothing below the table")
    allowed_keys = {frozenset((h[0], h[2])) for h in allowed_hits}
    assert allowed_keys <= set(GEOM_ALLOWED), allowed_keys
    assert all(GEOM_ALLOWED[k] for k in GEOM_ALLOWED)
    cases.append("geometry overlaps only in the documented joint-adjacent pairs")
    pay_pose = {"pinion": math.radians(122.3), "drum": math.radians(-29.829), "pawl": PAWL_DETENT_Q, "sled": -0.0002, "stripe_x": STRIPE_TARGET_X}
    viol, _, below = geometry_check(pay_pose)
    assert not viol and not below, ("payout pose interpenetration", viol, below)
    cases.append("geometry payout pose: no interpenetration")
    for crank_pd in range(0, 124, 4):
        pin_q = math.radians(crank_pd)
        cp = couple_kinematics_from_pinion(pin_q)
        for pq in (0.0, PAWL_DETENT_Q):
            viol, _, below = geometry_check({"pinion": pin_q, "drum": cp["drum_rad"], "pawl": pq, "sled": cp["sled_slide"], "stripe_x": cp["stripe_x"]})
            assert not viol and not below, ("sweep", crank_pd, pq, viol, below)
    cases.append("geometry sweep over crank arc and pawl positions")
    for jn, d_child, d_base in geometry_anchor_report():
        assert d_child <= 0.0 and d_base <= 0.0, ("joint anchor outside a body", jn, d_child, d_base)
    cases.append("every joint anchor lies inside both its moving body and the fixed frame")
    # The checker must catch a real overlap: a sled pushed 3 mm into the chock stop body is joint-adjacent (allowed),
    # but a stripe sunk 3 mm into the strap is not.
    viol, _, _ = geometry_check({"stripe_x": STRIPE_START_X})
    assert not viol
    sunk = _geom_prims({"stripe_x": STRIPE_START_X})
    sunk[-1]["c"] = sunk[-1]["c"] - np.array([0.0, 0.0, 0.003])
    box_strap = next(p for p in sunk if p["body"] == "base" and abs(p["c"][2] - STRAP_H) < 1e-9 and p["half"][0] > 0.07)
    assert float(-_prim_sdf(box_strap, _prim_surface(sunk[-1])).min()) > GEOM_MAX_OVERLAP
    cases.append("geometry checker detects a stripe sunk into the strap")

    assert len(cases) >= 50, len(cases)
    clear_arms()
    set_omega_pinion(None)
    _reset_episode(None)
    return len(cases)


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n = self_test()
    print(f"scoring: {n} cases pass")
    _viol, _allowed, _below = geometry_check()
    _deepest = max((h[4] for h in _allowed), default=0.0)
    print(
        f"geometry: {len(_viol)} interpenetrations > {GEOM_MAX_OVERLAP * 1000:.1f} mm, {len(_below)} below table, "
        f"{len(_allowed)} joint-adjacent overlaps allowed (deepest {_deepest * 1000:.1f} mm)"
    )
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
