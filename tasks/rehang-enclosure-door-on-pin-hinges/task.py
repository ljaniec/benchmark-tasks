"""On the station table: a grey 250 x 200 x 150 mm 16 gauge steel wall-mount electrical enclosure screwed to a plywood
backboard, its door taken off. Two removable-pin hinges stay on the enclosure's left side wall (two fixed knuckles
each, 5.1 mm bore). The grey door (0.78 kg, one 14.5 mm knuckle per hinge in a 16 mm gap) stands upright on a white
acetal (POM) setting block between two short blue acetal (POM) guide strips in front of the hinges, opened 90 degrees.
The block's 17 mm top is 2.5 mm below the height that centres the door knuckles in their gaps, so the door has to be
lifted to hang it. Two hinge pins (4.8 mm stainless shank, orange 11 mm head) stand in a blue pin
block on the left. Hang the door: line its knuckles up between the fixed knuckles, drop both pins fully down,
close the door against the enclosure and let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/rehang-enclosure-door-on-pin-hinges/task.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Pick up the orange-headed hinge pins and push them fully down through both hinges of the grey enclosure door, then close the door and let go." \\
           --scorer task_score --max-steps 3000 --seed 0
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import demokit  # noqa: E402,F401
from arm_watch import on_reset, on_step  # noqa: E402
from mechkit import Mech, MechState, _quat_xyzw_matrix, box, cyl, sphere  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.05  # one step: the 1.0 s hands-clear hold (20 steps) is counted inside score()

INSTRUCTION = (
    'Pick up the orange-headed hinge pins and push them fully down through both hinges of the grey enclosure door, then close the door and let go.'
)

# - colours -----------------------------------------------------------------------------------------------------------
STEEL_GREY = (0.66, 0.68, 0.70)  # ANSI-61 grey powder coat
ZINC = (0.80, 0.81, 0.83)
STAINLESS = (0.55, 0.57, 0.60)
ORANGE = (0.98, 0.45, 0.05)
FOAM_BLUE = (0.15, 0.35, 0.85)
PLYWOOD = (0.80, 0.66, 0.46)
BLACK = (0.06, 0.06, 0.07)
WHITE = (0.93, 0.93, 0.92)

EPS = 1e-9
DT = 0.05
HOLD_STEPS = 20

# - enclosure (table frame: x forward, y left, h above the table top; metres) -----------------------------------------
W = 0.2000  # nVent Hoffman INLINE LHC252015: 250 x 200 x 150 mm wall-mount enclosure, 16 gauge steel
H = 0.2500
D = 0.1500
T_WALL = 0.0015  # 16 gauge steel
X_F = 0.3066  # front plane of the body (door inner face rests on it)
X_A = X_F - 0.0066  # hinge axis x
Y_A = 0.0750  # hinge axis y (5 mm outboard of the left side wall)
Y_C = Y_A - 0.0050 - W / 2  # body centre y
BODY_H0 = 0.015  # body bottom above the table
HINGE_HC = (0.055, 0.225)  # (lower, upper) hinge centres on the body: 40 mm from the body ends; the door knuckles sit 35.5 mm and 44.5 mm from the door ends because the door rides 4.5 mm above the body bottom
R_IN = 0.00255  # bore 5.1 mm
R_OUT = 0.0050  # knuckle 10 mm OD
N_SEG = 16
SEG_W = 0.0021  # tangential width of one wall segment (covers the outer gaps of a 16-gon)
FK_LEN = 0.015  # fixed knuckle length
FK_IN = 0.008  # fixed knuckle inner face from the hinge centre (16 mm gap)
DK_LEN = 0.0145  # door knuckle length: 0.75 mm axial play per side in the 16 mm gap
DOOR_YE = -0.0066  # door's hinge-side plate edge in the door frame; at 90 deg open and coaxial it sits 13.2 mm short of
                   # the side wall's front edge (nothing stops the slide; the cue is visual)

PIN_R = 0.0024
PIN_L = 0.052  # shank under the head, ball tip included
HEAD_R = 0.0055
HEAD_T = 0.0140  # 6 mm head under a 14 mm tall orange vinyl cap
PIN_MASS = 0.012
HEAD_FRICTION = 0.9  # orange vinyl cap on the head
PIN_FRICTION = 0.3  # stainless shank in zinc-plated steel knuckles
DOOR_FRICTION = 0.7  # powder-coated steel
BLOCK_FRICTION = 0.08  # acetal (POM) setting block and guide strips; PhysX averages with the door's 0.7, so the effective pair is 0.39

DOOR_MASS = 0.78
B_NOM = 0.0195  # door bottom when its knuckles are centred in the gaps
B_REST = B_NOM - (2 * FK_IN - DK_LEN) / 2  # door bottom when its knuckles sit on the lower fixed knuckles
BLOCK_DROP = 0.0025  # the acetal support block's top is this far BELOW the height that puts the knuckles in the gaps
BLOCK_H = B_NOM - BLOCK_DROP  # support block top: the hand must lift the door by more than BLOCK_DROP - play to enter the gaps
FLANGE_D = 0.012
FLANGE_INSET = 0.0045  # side flanges sit 4.5 mm in from the door edges, inside the body walls
FLANGE_INSET_TOP = 0.0075  # top flange clears the body top wall by 1.5 mm with the knuckles centred, 0.75 mm at the top of the play
BOTTOM_FLANGE_Z = 0.0  # bottom flange flush with the door's bottom edge: the door stands on it
DK_Z = tuple(hc - B_NOM for hc in HINGE_HC)  # door knuckle centres in the door frame
DOOR_START = (X_A - 0.040, Y_A, BLOCK_H)  # door frame origin at the start (yaw -90 deg: opened 90 deg)
DOOR_START_YAW = -math.pi / 2

PIN_START = ((0.2200, 0.1950), (0.2200, 0.2250))
PIN_HOLE_FLOOR = 0.010
PIN_BLOCK_H = 0.040

# scoring tolerances
SEAT_HIGH_TOL = 0.0010  # head bottom at most 1 mm above the top knuckle face
SEAT_LOW_TOL = 0.0005
TIP_BELOW = 0.0030  # tip at least 3 mm below the bottom knuckle
AXIS_TOL = 0.0010  # pin and door knuckle within 1 mm of the hinge axis
PIN_TILT_MAX = math.radians(3.0)
CLOSED_GAP = 0.0030
CLEAR_DIST = 0.050
OPEN_GAP = 0.040
DOOR_TILT_MAX = math.radians(45.0)


def _wz(h: float) -> float:
    return TOP + h


def _qz(phi: float) -> tuple[float, float, float, float]:
    return (math.cos(phi / 2), 0.0, 0.0, math.sin(phi / 2))


def _ring(cx: float, cy: float, h0: float, h1: float, color) -> list:
    """Hollow knuckle: N_SEG flat wall segments around a 5.1 mm bore (a 16-gon, inner apothem R_IN)."""
    rc = (R_IN + R_OUT) / 2
    out = []
    for k in range(N_SEG):
        phi = 2 * math.pi * k / N_SEG
        out.append(
            box(
                (cx + rc * math.cos(phi), cy + rc * math.sin(phi), (h0 + h1) / 2),
                (R_OUT - R_IN, SEG_W, h1 - h0),
                color,
                tilt=_qz(phi),
            )
        )
    return out


def _span(a0, a1, b0, b1, c0, c1, color, collide=True):
    return box(((a0 + a1) / 2, (b0 + b1) / 2, (c0 + c1) / 2), (a1 - a0, b1 - b0, c1 - c0), color, collide=collide)


def fixed_knuckles(i: int) -> tuple[tuple[float, float], tuple[float, float]]:
    hc = HINGE_HC[i]
    return (hc - FK_IN - FK_LEN, hc - FK_IN), (hc + FK_IN, hc + FK_IN + FK_LEN)


def _enclosure_shapes() -> list:
    y0, y1 = Y_C - W / 2, Y_C + W / 2
    x1 = X_F + D
    s = [
        _span(x1, x1 + 0.018, Y_C - 0.25, Y_C + 0.25, 0.0, 0.40, PLYWOOD),  # backboard
        _span(X_F + 0.005, x1, y0, y1, 0.0, BODY_H0, PLYWOOD),  # mounting spacer
        _span(x1 - T_WALL, x1, y0, y1, BODY_H0, BODY_H0 + H, STEEL_GREY),  # back
        _span(X_F, x1 - T_WALL, y0, y1, BODY_H0, BODY_H0 + T_WALL, STEEL_GREY),  # bottom
        _span(X_F, x1 - T_WALL, y0, y1, BODY_H0 + H - T_WALL, BODY_H0 + H, STEEL_GREY),  # top
        _span(X_F, x1 - T_WALL, y0, y0 + T_WALL, BODY_H0 + T_WALL, BODY_H0 + H - T_WALL, STEEL_GREY),  # right side
        _span(X_F, x1 - T_WALL, y1 - T_WALL, y1, BODY_H0 + T_WALL, BODY_H0 + H - T_WALL, STEEL_GREY),  # left side
        # back panel with a DIN rail and terminal blocks (what the door is taken off for); visual only
        _span(x1 - 0.0046, x1 - T_WALL, y0 + 0.012, y1 - 0.012, BODY_H0 + 0.012, BODY_H0 + H - 0.012, WHITE, collide=False),
        _span(x1 - 0.0120, x1 - 0.0046, y0 + 0.020, y1 - 0.020, BODY_H0 + 0.110, BODY_H0 + 0.145, ZINC, collide=False),
    ]
    for k in range(8):
        yk = y0 + 0.035 + k * 0.0172
        s.append(_span(x1 - 0.050, x1 - 0.012, yk - 0.003, yk + 0.003, BODY_H0 + 0.100, BODY_H0 + 0.155, BLACK, collide=False))
    for i in range(2):
        for h0, h1 in fixed_knuckles(i):
            s += _ring(X_A, Y_A, h0, h1, ZINC)
            s.append(_span(X_A + 0.0030, X_F + 0.020, Y_A - 0.0047, Y_A - 0.0032, h0, h1, ZINC))  # leaf on the side wall
    return s


ENCLOSURE = Mech("enclosure", pos=(0.0, 0.0, TOP), mass=None, friction=0.5, shapes=_enclosure_shapes())


def _door_shapes() -> list:
    """Door frame: origin on the hinge axis at the door's bottom edge, closed orientation (x toward the body)."""
    xo, xi = 0.0050, 0.0066  # outer and inner face of the plate; plate collider 1.6 mm (16 gauge sheet is 1.52 mm nominal)
    ye, yl = DOOR_YE, DOOR_YE - W  # hinge edge, latch edge
    fi, ft, fb = FLANGE_INSET, FLANGE_INSET_TOP, BOTTOM_FLANGE_Z
    s = [
        _span(xo, xi, yl, ye, 0.0, H, STEEL_GREY),  # plate
        _span(xi, xi + FLANGE_D, yl + fi, ye - fi, H - ft - T_WALL, H - ft, STEEL_GREY),  # top flange
        _span(xi, xi + FLANGE_D, yl + fi, ye - fi, fb, fb + T_WALL, STEEL_GREY),  # bottom flange
        _span(xi, xi + FLANGE_D, ye - fi - T_WALL, ye - fi, fb + T_WALL, H - ft - T_WALL, STEEL_GREY),  # hinge side
        _span(xi, xi + FLANGE_D, yl + fi, yl + fi + T_WALL, fb + T_WALL, H - ft - T_WALL, STEEL_GREY),  # latch side
        # quarter-turn latch: black wing knob on the outer face, 25 mm from the latch edge
        _span(xo - 0.020, xo, yl + 0.019, yl + 0.031, H / 2 - 0.015, H / 2 + 0.015, BLACK),
    ]
    for zc in DK_Z:
        s += _ring(0.0, 0.0, zc - DK_LEN / 2, zc + DK_LEN / 2, ZINC)
        s.append(_span(0.0035, xo, -0.027, -0.0030, zc - DK_LEN / 2, zc + DK_LEN / 2, ZINC))  # door leaf
    return s


DOOR = Mech(
    "door",
    pos=(DOOR_START[0], DOOR_START[1], _wz(DOOR_START[2])),
    yaw=DOOR_START_YAW,
    mass=DOOR_MASS,
    friction=DOOR_FRICTION,
    shapes=_door_shapes(),
)


def _pin_shapes() -> list:
    """Pin frame: origin on the axis at the underside of the head."""
    return [
        cyl((0.0, 0.0, HEAD_T / 2), HEAD_R, HEAD_T, ORANGE, friction=HEAD_FRICTION),
        cyl((0.0, 0.0, -(PIN_L - PIN_R) / 2), PIN_R, PIN_L - PIN_R, STAINLESS),
        sphere((0.0, 0.0, -(PIN_L - PIN_R)), PIN_R, STAINLESS),
    ]


PINS = tuple(
    Mech(f"pin_{tag}", pos=(x, y, _wz(PIN_HOLE_FLOOR + PIN_L)), mass=PIN_MASS, friction=PIN_FRICTION, shapes=_pin_shapes())
    for tag, (x, y) in zip("ab", PIN_START)
)


def _rest_shapes() -> list:
    """Door support block (17 mm tall white acetal (POM) block the door stands and slides on; its top is BLOCK_DROP = 2.5 mm
    below the height that centres the door knuckles in the gaps, so the hand must lift the door 1.75 to 3.25 mm off it to
    slide the knuckles in) and two blue acetal (POM) guide strips either side of the door's latch end at the start."""
    x0 = DOOR_START[0] + DOOR_YE - W  # latch edge of the door at the start
    s = [
        box(((X_A - 0.012 + x0 - 0.015) / 2, Y_A - 0.0135, BLOCK_H / 2), (X_A - 0.012 - x0 + 0.015, 0.063, BLOCK_H), WHITE, friction=BLOCK_FRICTION),
    ]
    # two acetal (POM) guide strips, 30 mm tall, either side of the plate at the door's latch end: the outer strip is 1.5 mm
    # from the plate's outer face, the inner strip 13.5 mm from the plate's inner face. The door stands on its 13.6 mm deep
    # base (plate and bottom flange) and the strips catch its latch end if it tips. They end 4 mm past the latch edge, so
    # the door leaves them after its first 4 mm of travel and a slight yaw from the grasp cannot wedge it
    for y0, y1 in ((Y_A - 0.0035, Y_A + 0.0165), (Y_A - 0.0401, Y_A - 0.0201)):
        s.append(box((x0 - 0.004, (y0 + y1) / 2, BLOCK_H + 0.015), (0.016, y1 - y0, 0.030), FOAM_BLUE, friction=BLOCK_FRICTION))
    return s


DOOR_REST = Mech("door_rest", pos=(0.0, 0.0, TOP), mass=None, friction=0.8, shapes=_rest_shapes())


def _pin_block_shapes() -> list:
    """Blue foam pin block 60 x 75 x 40 mm with two 6 mm square holes 30 mm deep."""
    xs, (ya, yb) = PIN_START[0][0], (PIN_START[0][1], PIN_START[1][1])
    h = 0.003
    x0, x1, y0, y1 = xs - 0.030, xs + 0.030, ya - 0.020, yb + 0.025
    f, t = PIN_HOLE_FLOOR, PIN_BLOCK_H
    return [
        _span(x0, x1, y0, y1, 0.0, f, FOAM_BLUE),
        _span(x0, xs - h, y0, y1, f, t, FOAM_BLUE),
        _span(xs + h, x1, y0, y1, f, t, FOAM_BLUE),
        _span(xs - h, xs + h, y0, ya - h, f, t, FOAM_BLUE),
        _span(xs - h, xs + h, ya + h, yb - h, f, t, FOAM_BLUE),
        _span(xs - h, xs + h, yb + h, y1, f, t, FOAM_BLUE),
    ]


PIN_BLOCK = Mech("pin_block", pos=(0.0, 0.0, TOP), mass=None, friction=0.8, shapes=_pin_block_shapes())

OBJECTS = [ENCLOSURE, DOOR, *PINS, DOOR_REST, PIN_BLOCK]
PIN_NAMES = tuple(p.name for p in PINS)

# - episode state -------------------------------------------------------------------------------------------------------
_S: dict = {"violations": set(), "clear_steps": 0, "arms": None, "step": 0, "eval_step": -1, "cache": None}


def _reset_episode() -> None:
    _S.update(violations=set(), clear_steps=0, arms=None, step=0, eval_step=-1, cache=None)


def set_arms(arms) -> None:
    _S["arms"] = arms


def tick() -> None:
    """Advance one control step (the simulator does this in on_step)."""
    _S["step"] += 1


# - reading the arms (finger gap, TCP and pad probe points) --------------------------------------------------------------
ARMS = {"left": "left_robot", "right": "right_robot"}
PAD_HALF_WIDTH = 0.020
PAD_THICK = 0.010
PAD_BACK_FROM_TCP = 0.021
PAD_TIP_PAST_TCP = 0.013


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=float)


def pad_probe_points(tcp, finger_gap: float, rot) -> np.ndarray:
    tcp, rot = np.asarray(tcp, float), np.asarray(rot, float)
    ca, wa, aa = rot[:, 0], rot[:, 1], rot[:, 2]
    pts = [tcp, tcp + PAD_TIP_PAST_TCP * aa]
    for sign in (-1.0, 1.0):
        for t_off in (finger_gap / 2, finger_gap / 2 + PAD_THICK):
            for w_off in (-PAD_HALF_WIDTH, 0.0, PAD_HALF_WIDTH):
                for z_off in (-PAD_BACK_FROM_TCP, 0.0, PAD_TIP_PAST_TCP):
                    pts.append(tcp + sign * t_off * ca + w_off * wa + z_off * aa)
    return np.array(pts)


def read_arms(environment) -> dict:
    from yam_arena.runtime import tcp_world

    base = environment.unwrapped
    origin = _np(base.scene.env_origins)[0]
    out = {}
    for arm, key in ARMS.items():
        robot = base.scene[key]
        q = _np(robot.data.joint_pos)[0]
        names = list(robot.joint_names)
        gap = abs(float(q[names.index("joint7")])) + abs(float(q[names.index("joint8")]))
        tcp = _np(tcp_world(robot, 0)) - origin
        bodies = _np(robot.data.body_link_pose_w)[0]
        g = bodies[list(robot.body_names).index("gripper")]
        rot = _quat_xyzw_matrix(g[3:7])
        out[arm] = {"finger_gap": gap, "center": tcp, "pad_points": pad_probe_points(tcp, gap, rot)}
    return out


@on_step
def _watch(environment) -> None:
    try:
        _S["arms"] = read_arms(environment)
        _S["live"] = True
        if _S["step"] % 20 == 0:
            print("[arms] step=%d " % _S["step"] + " ".join(
                f"{a}: tcp=({g['center'][0]:.4f},{g['center'][1]:.4f},{g['center'][2] - TOP:.4f}) gap={g['finger_gap'] * 1000:.1f}"
                for a, g in _S["arms"].items()), flush=True)
    except Exception as exc:  # keep the trial running; clear check then fails safe (arms unknown = not clear)
        _S["arms"] = None
        if _S["step"] % 200 == 0:
            print(f"[task] read_arms failed: {exc!r}")
    tick()


@on_reset
def _on_reset() -> None:
    _reset_episode()


# - geometry of the parts in the world ---------------------------------------------------------------------------------
def _pose(state) -> tuple[np.ndarray, np.ndarray]:
    o = np.asarray(getattr(state, "origin", state.pos), float)
    return _quat_xyzw_matrix(state.quat_xyzw), o


def hinge_axis(i: int) -> np.ndarray:
    return np.array([X_A, Y_A, _wz(HINGE_HC[i])])


def pin_ends(state) -> tuple[np.ndarray, np.ndarray, float]:
    """(head underside centre, tip, tilt from vertical in rad)."""
    r, o = _pose(state)
    ax = r[:, 2]
    tilt = math.acos(max(-1.0, min(1.0, float(ax[2]))))
    return o, o - PIN_L * ax, tilt


def door_point(state, local) -> np.ndarray:
    r, o = _pose(state)
    return r @ np.asarray(local, float) + o


def door_knuckle_centre(state, i: int) -> np.ndarray:
    return door_point(state, (0.0, 0.0, DK_Z[i]))


def _radial(p, i: int) -> float:
    a = hinge_axis(i)
    return float(math.hypot(p[0] - a[0], p[1] - a[1]))


def pin_home(objects, pin: str, i: int) -> bool:
    """The pin passes through all three knuckles of hinge i: head seated on the top knuckle, tip below the bottom one."""
    head, tip, tilt = pin_ends(objects[pin])
    (lo0, _lo1), (_up0, up1) = fixed_knuckles(i)
    if tilt > PIN_TILT_MAX or _radial(head, i) > AXIS_TOL or _radial(tip, i) > AXIS_TOL + PIN_L * math.sin(PIN_TILT_MAX):
        return False
    dz = head[2] - _wz(up1)
    return -SEAT_LOW_TOL <= dz <= SEAT_HIGH_TOL and tip[2] <= _wz(lo0) - TIP_BELOW


def door_knuckle_engaged(objects, i: int) -> bool:
    """The door's own knuckle of hinge i sits coaxially in the gap between the two fixed knuckles."""
    c = door_knuckle_centre(objects["door"], i)
    r, _o = _pose(objects["door"])
    if math.acos(max(-1.0, min(1.0, float(r[2, 2])))) > math.radians(5.0) or _radial(c, i) > AXIS_TOL:
        return False
    (_a, lo1), (up0, _b) = fixed_knuckles(i)
    return _wz(lo1) - 0.0005 <= c[2] - DK_LEN / 2 and c[2] + DK_LEN / 2 <= _wz(up0) + 0.0005


def hinge_pinned(objects, i: int) -> bool:
    return door_knuckle_engaged(objects, i) and any(pin_home(objects, p, i) for p in PIN_NAMES)


def latch_gaps(objects) -> tuple[float, float]:
    """Gap between the door's inner face and the body front at the latch edge, bottom and top corners (m)."""
    yl = DOOR_YE - W
    return tuple(float(X_F - door_point(objects["door"], (0.0066, yl, z))[0]) for z in (0.0, H))


def door_closed(objects) -> bool:
    return all(-0.0015 <= g <= CLOSED_GAP for g in latch_gaps(objects))


def _door_boxes(state) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """(centre, half size, rotation) of the door's big colliders (plate, flanges, knob)."""
    r, o = _pose(state)
    out = []
    for s in DOOR.shapes[:6]:
        out.append((r @ np.array(s.at) + o, np.array(s.size) / 2, r))
    for zc in DK_Z:
        out.append((r @ np.array((0.0, -0.010, zc)) + o, np.array((R_OUT, R_OUT + 0.017, DK_LEN / 2)), r))
    return out


def _box_dist(pts: np.ndarray, c, half, rot) -> np.ndarray:
    q = np.abs((pts - c) @ rot) - half
    return np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)


def _segment_dist(pts: np.ndarray, a, b, radius: float) -> np.ndarray:
    ab = b - a
    t = np.clip(((pts - a) @ ab) / max(float(ab @ ab), EPS), 0.0, 1.0)
    return np.linalg.norm(pts - (a + t[:, None] * ab), axis=1) - radius


def door_supported_by_hinges_only(objects) -> bool:
    """Not resting on the bench or the acetal setting block and guide strips (it hangs from the hinges and touches the body front)."""
    st = objects["door"]
    corners = [door_point(st, (x, y, z)) for x in (0.005, 0.0186) for y in (DOOR_YE, DOOR_YE - W) for z in (0.0, H)]
    if min(c[2] for c in corners) < TOP + 0.002:
        return False
    rest = []
    for s in DOOR_REST.shapes:
        c = np.array((s.at[0], s.at[1], _wz(s.at[2])))
        rest.append((c, np.array(s.size) / 2 + 0.001))
    for c_d, half_d, rot_d in _door_boxes(st):
        for c_r, half_r in rest:
            ext = np.abs(rot_d) @ half_d
            if np.all(np.abs(c_d - c_r) <= ext + half_r):
                return False
    return True


def grippers_open_and_clear(objects) -> bool:
    arms = _S["arms"]
    if not arms or set(arms) != {"left", "right"}:
        return False
    boxes = _door_boxes(objects["door"])
    for g in arms.values():
        if float(g["finger_gap"]) < OPEN_GAP - EPS:
            return False
        pts = np.asarray(g["pad_points"], float)
        for c, half, rot in boxes:
            if float(_box_dist(pts, c, half, rot).min()) < CLEAR_DIST:
                return False
        for p in PIN_NAMES:
            head, tip, _t = pin_ends(objects[p])
            if float(_segment_dist(pts, tip, head + np.array((0, 0, HEAD_T)), HEAD_R).min()) < CLEAR_DIST:
                return False
    return True


def _update_violations(objects) -> None:
    v = _S["violations"]
    r, o = _pose(objects["door"])
    if math.acos(max(-1.0, min(1.0, float(r[2, 2])))) > DOOR_TILT_MAX:
        v.add("door_dropped")
    if o[2] < TOP - 0.05 or abs(o[0] - 0.36) > 0.55 or abs(o[1]) > 0.65:
        v.add("door_off_bench")
    for p in PIN_NAMES:
        head, _tip, _t = pin_ends(objects[p])
        if head[2] < TOP - 0.03 or abs(head[0] - 0.36) > 0.55 or abs(head[1]) > 0.65:
            v.add(f"{p}_off_bench")


def _rung(objects) -> float:
    n = sum(hinge_pinned(objects, i) for i in range(2))
    if n == 0:
        return 0.0
    if n == 1:
        return 0.25
    return 0.75 if door_closed(objects) else 0.5


def _evaluate(objects) -> dict:
    if _S["eval_step"] == _S["step"] and _S["cache"] is not None:
        return _S["cache"]
    _update_violations(objects)
    rung = 0.0 if _S["violations"] else _rung(objects)
    full_ok = rung >= 0.75 and door_supported_by_hinges_only(objects) and grippers_open_and_clear(objects)
    _S["clear_steps"] = _S["clear_steps"] + 1 if full_ok else 0
    value = 1.0 if _S["clear_steps"] >= HOLD_STEPS else rung
    res = {"score": value, "rung": rung, "success": value >= 1.0}
    _S["eval_step"], _S["cache"] = _S["step"], res
    if _S["step"] % 20 == 0 and _S.get("live"):
        _log(objects, res)
    return res


def _log(objects, res) -> None:
    st = objects["door"]
    r, o = _pose(st)
    yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
    dk = [door_knuckle_centre(st, i) for i in range(2)]
    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))
    tx, ty = math.degrees(math.atan2(-r[1, 2], r[2, 2])), math.degrees(math.atan2(r[0, 2], r[2, 2]))
    dz = [(dk[i][2] - hinge_axis(i)[2]) * 1000 for i in range(2)]
    pins = []
    for p in PIN_NAMES:
        head, _tip, tilt = pin_ends(objects[p])
        pins.append(f"{p}=({head[0]:.4f},{head[1]:.4f},{head[2] - TOP:.4f}) tilt={math.degrees(tilt):.1f}")
    print(
        f"[task] step={_S['step']} score={res['score']:.2f} door o=({o[0]:.4f},{o[1]:.4f},{o[2] - TOP:.4f}) yaw={yaw:.1f} "
        f"tilt={tilt_deg:.2f} txy=({tx:.2f},{ty:.2f}) dk_dz=({dz[0]:.2f},{dz[1]:.2f}) dk_r=({_radial(dk[0], 0) * 1000:.2f},{_radial(dk[1], 1) * 1000:.2f})mm gaps={[round(g * 1000, 1) for g in latch_gaps(objects)]} "
        + " ".join(pins)
        + f" viol={sorted(_S['violations'])}",
        flush=True,
    )


def score(objects) -> float:
    return float(_evaluate(objects)["score"])


def success(objects) -> bool:
    return bool(_evaluate(objects)["success"])


def distance(objects) -> float:
    d = 0.0
    for i, p in zip((1, 0), PIN_NAMES):
        head, _tip, _t = pin_ends(objects[p])
        d += float(np.linalg.norm(head - np.array((X_A, Y_A, _wz(fixed_knuckles(i)[1][1])))))
    return d


# - demo: open-loop joint plan built from fixed Cartesian waypoints -------------------------------------------------------
def _rx(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


RIGHT_PITCH = -60.0
LEFT_LEAN = 50.0
PUSH_LEAN = 35.0
PUSH_Y_SHIFT = 0.004  # closed fingertips press the head 4 mm toward +y of its centre, clear of the door plate
GRIP_LOCAL = np.array((0.0118, DOOR_YE - W / 2, H - 0.004))  # pinch centre on the door's top edge above its centre of mass, door frame
HEAD_CONTACT_S = 0.006  # head centre this far along the approach axis from the TCP (fingertip grip)
HEAD_WIDTH_SHIFT = 0.010  # TCP shifted along the finger width so the fingers stay off the door and side wall
LEFT_PARK = (0.20, 0.19, TOP + 0.16)
SWING_DEG = 60.0
Y_COMP = 0.0005  # hand y offset after the grip (tuned on the sim: the squeezed pinch shifts the door toward -y)
PRE_LIFT_GAP = 0.012  # the door is lifted when its knuckles are this far (12 mm) in front of the hinge axis, clear of the frame knuckles
DOOR_PRESS = 0.0015  # once the pins are in, the hand aims 1.5 mm low so the door rests on the lower knuckles
LOAD_TILT = (7.65, -6.7)  # world-frame rotation (deg about x, about y) added to the hand's orientation while it carries the door: the YAM's tilt under the load
LOAD_COMP = np.array((0.009, -0.0111, 0.0405))  # hand target offset while it carries the door: the YAM's measured sag under the load (open-loop tuning)
PUSH_R = 0.110  # the closing push presses the door's outer face this far from the hinge axis
SLIDE_OVERSHOOT = 0.006  # slide along the block: hand target 6 mm past the lift point, compensates the arm's measured lag behind block friction in the sim (open-loop tuning)
PIN_RELEASE_H = 0.012  # the pin is let go 12 mm above the seat; the pin may rest on the door knuckle until the door comes into line


def right_rot(phi_deg: float = 0.0) -> np.ndarray:
    return _rz(phi_deg) @ _ry(RIGHT_PITCH) @ grip_rotation(0.0)


def carry_rot(phi_deg: float = 0.0) -> np.ndarray:
    return _rx(LOAD_TILT[0]) @ _ry(LOAD_TILT[1]) @ right_rot(phi_deg)


def left_rot(lean: float = LEFT_LEAN) -> np.ndarray:
    return _rx(-lean) @ grip_rotation(math.pi / 2)


def door_origin_for_grip(tcp, phi_deg: float) -> np.ndarray:
    """Door frame origin when the right TCP pinches GRIP_LOCAL with the door opened (90 - phi) deg."""
    yaw = math.radians(-90.0 + phi_deg)
    r = _rz(math.degrees(yaw))
    return np.asarray(tcp, float) - r @ GRIP_LOCAL


def grip_for_door(origin, phi_deg: float) -> np.ndarray:
    return np.asarray(origin, float) + _rz(-90.0 + phi_deg) @ GRIP_LOCAL


def tcp_for_head(head_centre, rot) -> np.ndarray:
    a, w = rot[:, 2], rot[:, 1]
    sign = 1.0 if w[1] > 0 else -1.0
    return np.asarray(head_centre, float) - HEAD_CONTACT_S * a + sign * HEAD_WIDTH_SHIFT * w


def _p(v) -> tuple[float, float, float]:
    return tuple(float(x) for x in v)


def push_tcp(open_deg: float, h: float, rot) -> np.ndarray:
    """Right TCP pressing the door's outer face PUSH_R from the axis with closed fingertips, door opened open_deg."""
    face = np.array((X_A, Y_A, 0.0)) + _rz(-open_deg) @ np.array((0.0050 - 0.0015, -PUSH_R, 0.0))
    face[2] = _wz(h)
    return face - PAD_TIP_PAST_TCP * rot[:, 2]


def push_rot(open_deg: float) -> np.ndarray:
    return _rz(-min(open_deg, 20.0)) @ _ry(-60.0) @ grip_rotation(0.0)


def demo():
    """Right hand carries the door onto its hinges and holds it; left hand drops both pins and pushes them home;
    right hand swings the door shut and lets go. Fixed waypoints only, no sim state."""
    o_start = np.array((DOOR_START[0], DOOR_START[1], _wz(BLOCK_H)))
    o_pre = np.array((X_A - PRE_LIFT_GAP + SLIDE_OVERSHOOT, Y_A + Y_COMP, _wz(BLOCK_H)))  # door pushed along the block to just short of the hinge
    o_lift = np.array((X_A - PRE_LIFT_GAP, Y_A + Y_COMP, _wz(B_NOM)))
    o_in = np.array((X_A - 0.0008, Y_A + Y_COMP, _wz(B_NOM)))  # lifted, the door slides freely, so the hand aims at the axis itself
    o_rest = o_in
    o_axis = np.array((X_A, Y_A + Y_COMP, _wz(B_NOM)))  # door coaxial with the fixed knuckles: swing about the axis
    r0 = right_rot(0.0)
    rc = carry_rot(0.0)
    g_start = grip_for_door(o_start, 0.0)
    door_open_cmd = -0.020  # 40 mm gap over the 13.6 mm door edge
    door_release_cmd = -0.011  # 22 mm: lets go of the edge without the fingertips reaching the enclosure
    door_grip_cmd = 0.0  # closed command on the 13.6 mm edge: 6.8 mm per side of squeeze, 34 N per finger
    right = [
        Waypoint(1.5, _p(g_start + np.array((0.022, -0.045, 0.022))), door_open_cmd, "R beside door", r0),
        Waypoint(1.5, _p(g_start + np.array((0.022, 0.0, 0.022))), door_open_cmd, "R above door edge", r0),
        Waypoint(1.0, _p(g_start + np.array((0.0, 0.0, 0.008))), door_open_cmd, "R over door edge", r0),
        Waypoint(1.0, _p(g_start), door_open_cmd, "R to door edge", r0),
        Waypoint(0.4, _p(g_start), door_open_cmd, "R settle", r0),
        Waypoint(1.0, _p(g_start), door_grip_cmd, "R grip door", r0),
        Waypoint(0.6, _p(g_start), door_grip_cmd, "R hold", r0),
        Waypoint(2.5, _p(grip_for_door(o_pre, 0.0)), door_grip_cmd, "R slide door along the block", r0),
        Waypoint(1.5, _p(grip_for_door(o_lift, 0.0) + LOAD_COMP), door_grip_cmd, "R lift door off the block", rc),
        Waypoint(1.5, _p(grip_for_door(o_in, 0.0) + LOAD_COMP), door_grip_cmd, "R slide knuckles in", rc),
        Waypoint(1.0, _p(grip_for_door(o_rest, 0.0) + LOAD_COMP), door_grip_cmd, "R settle door", rc),
    ]
    t_door_in = sum(w.duration for w in right)

    lr = left_rot(LEFT_LEAN)
    lp = left_rot(PUSH_LEAN)
    pin_open_cmd = -0.012  # 24 mm gap over the 11 mm head
    pin_grip_cmd = 0.0  # closed command on the 11 mm head: 5.5 mm per side of squeeze
    left = [Waypoint(2.0, (0.22, 0.25, _wz(0.20)), FINGER_OPEN, "L ready", lr)]
    for k, (i, (px, py)) in enumerate(zip((1, 0), PIN_START)):
        c_pick = np.array((px, py, _wz(PIN_HOLE_FLOOR + PIN_L + HEAD_T / 2)))
        seat_top = _wz(fixed_knuckles(i)[1][1])
        c_start = np.array((X_A, Y_A, seat_top + PIN_L + 0.006 + HEAD_T / 2))
        c_side = c_start + np.array((0.0, 0.040, 0.0))
        c_low = np.array((X_A, Y_A, seat_top + PIN_RELEASE_H + HEAD_T / 2))
        c_lift = c_pick + np.array((0.0, 0.0, 0.050))
        c_mid = np.array((0.5 * (px + X_A), 0.5 * (py + c_side[1]), max(c_lift[2], c_side[2] - 0.03)))
        left += [
            Waypoint(2.0, _p(tcp_for_head(c_pick + np.array((0, 0, 0.04)), lr)), pin_open_cmd, f"L over pin {k}", lr),
            Waypoint(1.5, _p(tcp_for_head(c_pick, lr)), pin_open_cmd, f"L to pin {k}", lr),
            Waypoint(0.4, _p(tcp_for_head(c_pick, lr)), pin_open_cmd, "L settle", lr),
            Waypoint(0.8, _p(tcp_for_head(c_pick, lr)), pin_grip_cmd, f"L grip pin {k}", lr),
            Waypoint(2.0, _p(tcp_for_head(c_lift, lr)), pin_grip_cmd, f"L lift pin {k}", lr),
            Waypoint(2.0, _p(tcp_for_head(c_mid, lr)), pin_grip_cmd, f"L carry pin {k}", lr),
            Waypoint(2.0, _p(tcp_for_head(c_side, lr)), pin_grip_cmd, f"L beside hinge {k}", lr),
            Waypoint(1.5, _p(tcp_for_head(c_start, lr)), pin_grip_cmd, f"L over hinge {k}", lr),
            Waypoint(0.5, _p(tcp_for_head(c_start, lr)), pin_grip_cmd, "L settle", lr),
            Waypoint(4.0, _p(tcp_for_head(c_low, lr)), pin_grip_cmd, f"L drop pin {k}", lr),
            Waypoint(0.6, _p(tcp_for_head(c_low, lr)), pin_open_cmd, f"L release pin {k}", lr),
            Waypoint(0.8, _p(tcp_for_head(c_low + np.array((0, 0.004, 0.020)), lr)), pin_open_cmd, "L up", lr),
        ]
        head_top = np.array((X_A, Y_A, seat_top + HEAD_T))
        press = head_top - PAD_TIP_PAST_TCP * lp[:, 2] + np.array((0.0, PUSH_Y_SHIFT, 0.0))
        left += [
            Waypoint(1.0, _p(press + np.array((0, 0.0, PIN_RELEASE_H + 0.006))), 0.0, f"L close over head {k}", lp),
            Waypoint(2.0, _p(press + np.array((0, 0.0, -0.002))), 0.0, f"L push pin {k} home", lp),
            Waypoint(0.5, _p(press + np.array((0, 0.0, -0.002))), 0.0, "L hold push", lp),
            Waypoint(1.0, _p(press + np.array((0, 0.03, 0.015))), 0.0, f"L off hinge {k}", lp),
        ]
        if k == 0:
            left.insert(1, Waypoint(max(0.1, t_door_in - 2.0), (0.22, 0.25, _wz(0.20)), FINGER_OPEN, "L wait door", lr))
    left += [Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "L park", lr)]
    t_pins_done = sum(w.duration for w in left)

    right.append(Waypoint(t_pins_done - t_door_in, _p(grip_for_door(o_rest, 0.0) + LOAD_COMP), door_grip_cmd, "R hold door", rc))
    o_hung = o_axis - np.array((0.0, 0.0, DOOR_PRESS))
    right.append(Waypoint(1.0, _p(grip_for_door(o_hung, 0.0)), door_grip_cmd, "R unload door onto the knuckles", r0))
    for phi in range(10, int(SWING_DEG) + 1, 10):
        right.append(Waypoint(0.8, _p(grip_for_door(o_hung, phi)), door_grip_cmd, f"R swing {phi}", right_rot(phi)))
    g_end = grip_for_door(o_hung, SWING_DEG)
    r_end = right_rot(SWING_DEG)
    open_deg = 90.0 - SWING_DEG
    right += [
        Waypoint(0.8, _p(g_end), door_release_cmd, "R release door", r_end),
        Waypoint(1.0, _p(g_end + np.array((-0.012, -0.004, 0.030))), door_release_cmd, "R lift off edge", r_end),
        Waypoint(1.5, _p(push_tcp(open_deg + 4.0, 0.13, push_rot(open_deg)) - 0.045 * push_rot(open_deg)[:, 2]), 0.0, "R to door face", push_rot(open_deg)),
    ]
    for a in (open_deg, 20.0, 12.0, 6.0, 2.0, 0.0, -0.8):
        right.append(Waypoint(0.7 if a > 0 else 1.0, _p(push_tcp(a, 0.13, push_rot(max(a, 0.0)))), 0.0, f"R push {a}", push_rot(max(a, 0.0))))
    pr = push_rot(0.0)
    p_last = push_tcp(0.0, 0.13, pr)
    right += [
        Waypoint(1.0, _p(p_last), 0.0, "R push ease to 0", pr),
        Waypoint(0.6, _p(p_last), 0.0, "R hold shut", pr),
        Waypoint(2.5, _p(p_last + np.array((-0.020, 0.0, 0.0))), 0.0, "R back off straight", pr),
        Waypoint(1.2, _p(p_last + np.array((-0.020, 0.0, 0.0)) - 0.05 * pr[:, 2]), 0.0, "R back off", pr),
        Waypoint(1.5, (0.16, -0.20, _wz(0.22)), FINGER_OPEN, "R park", pr),
        Waypoint(2.0, (0.16, -0.20, _wz(0.22)), FINGER_OPEN, "R rest", pr),
    ]
    t_right = sum(w.duration for w in right)
    left.append(Waypoint(max(0.1, t_right - t_pins_done), LEFT_PARK, FINGER_OPEN, "L rest", lr))
    return {"left": left, "right": right}


if __name__ == "__main__":
    from selftest import run_all

    run_all(sys.modules[__name__])
