"""On the station table: the opened system of a workstation tower laid on its side (modelled as a fixed board with a
CPU heatsink block and two grey DDR5 memory sticks; board layout approximate). A low-profile graphics card (black
shroud, a green tape band across its top edge and down its faces, full-height steel bracket) stands in the board's x16
slot, its face toward the arms. The slot's retention latch sits under the card's overhang at the tail-side end of the
gold fingers; its orange tab is beside the card's back face, between the card and the heatsink, and its hook sits in
the card's retention notch and springs back to locked when released. A grey antistatic tray lies on the mat on the left.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/pcie-card-latch-pull/task.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Put the green graphics card into the grey tray, pressing the orange latch tab to free it." \\
           --scorer task_score --max-steps 1600 --seed 0
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import demokit  # noqa: E402,F401
from arm_watch import on_reset, on_step  # noqa: E402
from mechkit import Joint, Mech, box, cyl, joint_poses, set_joint_target  # noqa: E402
from mechkit import on_reset as mech_on_reset  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.05  # the 1.0 s clear-rest hold (20 steps) is counted inside score()

INSTRUCTION = "Put the green graphics card into the grey tray, pressing the orange latch tab to free it."

DT = 0.05
HOLD_STEPS = 20
EPS = 1e-9

# - colours -----------------------------------------------------------------------------------------------------------
ALU = (0.42, 0.43, 0.46)
MAT_GREY = (0.86, 0.86, 0.87)
PCB_MB = (0.10, 0.34, 0.20)
PCB_CARD = (0.06, 0.09, 0.07)
SLOT_BLACK = (0.04, 0.04, 0.04)
SHROUD = (0.07, 0.07, 0.08)
GOLD = (0.85, 0.66, 0.22)
STEEL = (0.72, 0.73, 0.75)
GREEN = (0.10, 0.80, 0.22)
ORANGE = (0.98, 0.45, 0.05)
DIMM_GREY = (0.55, 0.56, 0.58)
COOLER = (0.05, 0.05, 0.05)
FIN = (0.45, 0.46, 0.48)
TRAY_GREY = (0.36, 0.40, 0.45)
STANDOFF = (0.80, 0.70, 0.30)


def _box(lo, hi, color=SLOT_BLACK, collide: bool = True):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return box(tuple((lo + hi) / 2), tuple(hi - lo), color, collide=collide)


# - card frame: origin at the tail end of the card, bottom of the gold fingers, PCB mid-plane ---------------------------
# x along the card toward the bracket, y across the card (shroud on +y), z up. At rest the card frame is world-aligned.
# The card and slot are built in the card frame and turned 90 deg about the vertical, so the card's broad face (shroud
# side) faces the arms and the top camera: card x (tail to bracket) = world +y, card y (shroud side) = world -x.
FRAME_YAW = math.pi / 2
FRAME_R = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
CARD_ORIGIN = np.array([0.180, -0.030, TOP + 0.0146])
CARD_LEN = 0.1559  # PCB length (NVIDIA T400 low profile)
CARD_H = 0.0689  # PCB height including the gold fingers
PCB_T = 0.00157
PCB_HALF = PCB_T / 2
FINGER_DEPTH = 0.0083  # gold fingers below the PCB edge
FINGER_X0, FINGER_X1 = 0.0525, 0.1415  # 89 mm x16 contact region
SLIT_X1 = 0.0575  # retention notch: a slit open toward the tail, between z = SLIT_Z0 and the PCB edge
SLIT_Z0 = 0.0040
SHROUD_Y1 = 0.0150
BRACKET_X = (CARD_LEN, CARD_LEN + 0.0010)
BRACKET_Y = (-0.0025, 0.0159)
BRACKET_Z = (-0.0060, 0.1140)  # full-height bracket, 120 mm
BAND_X = (0.095, 0.135)  # green tape band on the top edge, 40 mm wide, 95-135 mm from the tail end
CARD_MASS = 0.125

# Collider boxes of the card in its own frame: (name, lo, hi)
CARD_BOXES = [
    ("pcb_tail", (0.0, -PCB_HALF, FINGER_DEPTH), (FINGER_X0, PCB_HALF, CARD_H)),
    ("pcb_main", (FINGER_X0, -PCB_HALF, FINGER_DEPTH), (CARD_LEN, PCB_HALF, CARD_H)),
    ("notch_jaw", (FINGER_X0, -PCB_HALF, 0.0), (SLIT_X1, PCB_HALF, SLIT_Z0)),
    ("fingers", (SLIT_X1, -PCB_HALF, 0.0), (FINGER_X1, PCB_HALF, FINGER_DEPTH)),
    ("shroud", (0.012, PCB_HALF, 0.012), (0.150, SHROUD_Y1, CARD_H - 0.0004)),
    ("bracket", (BRACKET_X[0], BRACKET_Y[0], BRACKET_Z[0]), (BRACKET_X[1], BRACKET_Y[1], BRACKET_Z[1])),
]
FINGER_BOXES = ("notch_jaw", "fingers")

CARD = Mech(
    "card",
    pos=tuple(float(v) for v in CARD_ORIGIN),
    yaw=FRAME_YAW,
    mass=CARD_MASS,
    friction=1.0,
    color=SHROUD,
    shapes=[
        *[_box(lo, hi, {"pcb_tail": PCB_CARD, "pcb_main": PCB_CARD, "notch_jaw": GOLD, "fingers": GOLD,
                        "shroud": SHROUD, "bracket": STEEL}[n]) for n, lo, hi in CARD_BOXES],
        _box((BAND_X[0], -PCB_HALF - 0.0002, CARD_H - 0.0004), (BAND_X[1], SHROUD_Y1 + 0.0002, CARD_H + 0.0006), GREEN,
             collide=False),
        _box((BAND_X[0], SHROUD_Y1, 0.012), (BAND_X[1], SHROUD_Y1 + 0.0006, CARD_H - 0.0004), GREEN, collide=False),
        _box((BAND_X[0], -PCB_HALF - 0.0006, 0.030), (BAND_X[1], -PCB_HALF, CARD_H - 0.0004), GREEN, collide=False),
        cyl((0.080, SHROUD_Y1 + 0.0003, 0.040), 0.018, 0.0006, FIN, tilt=(math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0),
            collide=False),
    ],
)

# - slot: fixed articulation in the card frame (same origin) -----------------------------------------------------------
SLOT_TOP_Z = 0.0090  # top of the slot housing, card frame (11 mm above the board)
BOARD_TOP_Z = -0.0020  # motherboard top surface, card frame
SLOT_HALF_W = 0.0040
CHANNEL_HALF = PCB_HALF + 0.0005
PAD_Y_OUT = 0.0035
PAD_SPRING = 10250.0  # N/m; 1 mm preload -> 10.25 N per pad, mu 0.6 -> 6.15 N per pad
PAD_PRELOAD = 0.0010
PAD_TAIL_X = (0.0645, 0.0905)
PAD_FRONT_X = (0.1045, 0.1305)
PAD_Z = (0.0005, 0.0080)
PAD_FRICTION = 0.2  # combines (average) with the card's 1.0 to mu 0.6
LATCH_PIVOT = (0.0540, 0.0, -0.0015)
LATCH_OPEN = math.radians(-12.0)
LATCH_SPRING = 0.53  # N m/rad
LATCH_PRELOAD = 0.10  # rad past the locked stop
HOOK_SPRING = 30000.0  # N/m: latch hook flex; 1 mm = 30 N
FORCED_LIFT = 0.003  # tail 3 mm up with no hand on the pressed tab since seating = card forced past the latch
SEATED_LIFT = 0.002
TAB_UP_DEG = -5.0  # latch closer than 5 deg to its locked stop = tab not pressed
TAB_HAND_DIST = 0.002  # a finger pad within 2 mm of the orange tab-top surface at its current angle = a hand on the tab
TAB_X = (0.0205, 0.0305)  # under the card overhang, beside the PCB back face
TAB_Y = (-0.022, -0.003)
TAB_YC = 0.5 * (TAB_Y[0] + TAB_Y[1])
TAB_TOP_Z = 0.0115


def _pad(name, side, xr):
    sgn = 1.0 if side > 0 else -1.0
    y_in, y_out = sgn * PCB_HALF, sgn * PAD_Y_OUT
    lo = (xr[0], min(y_in, y_out), PAD_Z[0])
    hi = (xr[1], max(y_in, y_out), PAD_Z[1])
    at = tuple((np.array(lo) + np.array(hi)) / 2)
    rel_lo = tuple(np.array(lo) - np.array(at))
    rel_hi = tuple(np.array(hi) - np.array(at))
    low, high = (-0.0007, 0.0025) if sgn > 0 else (-0.0025, 0.0007)
    return Joint(name, "slide", axis="y", at=at, low=low, high=high, spring=PAD_SPRING, damping=20.0, mass=0.02,
                 friction=PAD_FRICTION, shapes=[_box(rel_lo, rel_hi, SLOT_BLACK)])


PADS = {
    "pad_tail_p": (+1, PAD_TAIL_X),
    "pad_tail_n": (-1, PAD_TAIL_X),
    "pad_front_p": (+1, PAD_FRONT_X),
    "pad_front_n": (-1, PAD_FRONT_X),
}


def _latch_shapes():
    px, _, pz = LATCH_PIVOT

    def rel(lo, hi, color=SLOT_BLACK, collide=True):
        return _box((lo[0] - px, lo[1], lo[2] - pz), (hi[0] - px, hi[1], hi[2] - pz), color, collide)

    return [
        rel((0.0455, -0.0030, 0.0045), (0.0535, 0.0030, 0.0075)),  # hook lip, 1 mm over the notch jaw edge (x 0.0200)
        rel((0.0455, -0.0030, -0.0010), (0.0520, 0.0030, 0.0045)),  # hook post
        rel((0.0305, -0.0040, 0.0040), (0.0455, 0.0040, 0.0055)),  # lever arm under the card's tail, 2.8 mm below the PCB
        rel((TAB_X[0], TAB_Y[0], 0.0060), (TAB_X[1], TAB_Y[1], TAB_TOP_Z)),  # press tab, 10 x 19 mm
        rel((TAB_X[0], TAB_Y[0], TAB_TOP_Z), (TAB_X[1], TAB_Y[1], TAB_TOP_Z + 0.0006), ORANGE, False),
    ]


SLOT = Mech(
    "slot",
    pos=tuple(float(v) for v in CARD_ORIGIN),
    yaw=FRAME_YAW,
    mass=None,
    friction=0.5,
    color=SLOT_BLACK,
    shapes=[
        _box((0.0525, -SLOT_HALF_W, BOARD_TOP_Z), (0.1465, SLOT_HALF_W, 0.0)),  # channel floor
        _box((0.1420, -SLOT_HALF_W, 0.0), (0.1465, SLOT_HALF_W, 0.0080)),  # front end wall, 0.5 mm end play
        *[_box((x0, s * CHANNEL_HALF if s > 0 else -SLOT_HALF_W, 0.0), (x1, SLOT_HALF_W if s > 0 else -CHANNEL_HALF, SLOT_TOP_Z))
          for (x0, x1) in ((0.0550, 0.0625), (0.0925, 0.1025), (0.1325, 0.1420)) for s in (1, -1)],
        *[_box((x0, s * PAD_Y_OUT if s > 0 else -SLOT_HALF_W, 0.0), (x1, SLOT_HALF_W if s > 0 else -PAD_Y_OUT, SLOT_TOP_Z))
          for (x0, x1) in (PAD_TAIL_X, PAD_FRONT_X) for s in (1, -1)],
        _box((0.0405, -SLOT_HALF_W, BOARD_TOP_Z), (0.0525, SLOT_HALF_W, -0.0012), collide=False),  # latch pocket floor
    ],
    joints=[
        *[_pad(n, s, xr) for n, (s, xr) in PADS.items()],
        Joint("hook", "slide", axis="z", at=LATCH_PIVOT, low=-0.0002, high=0.0030, spring=HOOK_SPRING, damping=30.0,
              mass=0.02, shapes=[_box((-0.0015, -0.0015, -0.0005), (0.0015, 0.0015, 0.0), SLOT_BLACK)]),
        Joint("latch", "hinge", axis="y", at=LATCH_PIVOT, low=LATCH_OPEN, high=0.0, spring=LATCH_SPRING, damping=0.02,
              mass=0.02, friction=0.4, parent="hook", shapes=_latch_shapes()),
    ],
)

# - static bench, board, cooler, memory (world coordinates relative to (0, 0, TOP)) -----------------------
# Boxes are laid out in the card frame (numbers below = card-frame position + (0.200, -0.060)) and turned with it.
_LAYOUT_SHIFT = np.array([0.200, -0.060, 0.0])


def _lay_pt(p) -> np.ndarray:
    p = np.asarray(p, float) - _LAYOUT_SHIFT
    return np.array([CARD_ORIGIN[0], CARD_ORIGIN[1], 0.0]) + FRAME_R @ p


def _lay(lo, hi) -> tuple[np.ndarray, np.ndarray]:
    a, b = _lay_pt(lo), _lay_pt(hi)
    return np.minimum(a, b), np.maximum(a, b)


MB_LO, MB_HI = _lay((0.110, -0.240, 0.0110), (0.354, 0.004, 0.0126))
COOLER_LO, COOLER_HI = _lay((0.230, -0.195, 0.0126), (0.325, -0.100, 0.0666))
_COOLER_FIN_HI = _lay((0.230, -0.195, 0.0126), (0.325, -0.100, 0.0426))[1]
DIMMS = [_lay((x - 0.0035, -0.233, 0.0126), (x + 0.0035, -0.100, 0.0126 + 0.0380)) for x in (0.125, 0.136)]
DIMM_SOCKETS = [_lay((x - 0.0050, -0.239, 0.0126), (x + 0.0050, -0.094, 0.0206)) for x in (0.125, 0.136)]
BENCH_LO, BENCH_HI = _lay((0.080, -0.245, 0.0030), (0.375, 0.005, 0.0050))
MAT_LO = np.array([0.060, -0.300, 0.0])
MAT_HI = np.array([0.500, 0.340, 0.0030])

_bench_shapes = [
    _box(MAT_LO, MAT_HI, MAT_GREY),
    _box(BENCH_LO, BENCH_HI, ALU),
    _box(MB_LO, MB_HI, PCB_MB),
    _box(COOLER_LO, _COOLER_FIN_HI, FIN),
    _box((COOLER_LO[0], COOLER_LO[1], _COOLER_FIN_HI[2]), COOLER_HI, COOLER),
]
for (lo, hi), (slo, shi) in zip(DIMMS, DIMM_SOCKETS):
    _bench_shapes.append(_box(lo, hi, DIMM_GREY))
    _bench_shapes.append(_box(slo, shi, SLOT_BLACK))
for sx in (0.120, 0.344):
    for sy in (-0.230, -0.006):
        _bench_shapes.append(cyl(tuple(_lay_pt((sx, sy, 0.0080))), 0.0025, 0.0060, STANDOFF, collide=False))

BENCH = Mech("bench", pos=(0.0, 0.0, TOP), mass=None, friction=0.8, color=ALU, shapes=_bench_shapes)

# - antistatic tray --------------------------------------------------------------------------------------------------
TRAY_LO = np.array([0.150, 0.145, 0.0030])
TRAY_HI = np.array([0.450, 0.325, 0.0180])
TRAY_WALL = 0.002
TRAY_FLOOR = 0.002
TRAY_IN_LO = TRAY_LO + np.array([TRAY_WALL, TRAY_WALL, TRAY_FLOOR])
TRAY_IN_HI = TRAY_HI - np.array([TRAY_WALL, TRAY_WALL, 0.0])
TRAY = Mech(
    "tray",
    pos=(0.0, 0.0, TOP),
    mass=None,
    friction=0.8,
    color=TRAY_GREY,
    shapes=[
        _box(TRAY_LO, (TRAY_HI[0], TRAY_HI[1], TRAY_LO[2] + TRAY_FLOOR), TRAY_GREY),
        _box(TRAY_LO, (TRAY_LO[0] + TRAY_WALL, TRAY_HI[1], TRAY_HI[2]), TRAY_GREY),
        _box((TRAY_HI[0] - TRAY_WALL, TRAY_LO[1], TRAY_LO[2]), TRAY_HI, TRAY_GREY),
        _box(TRAY_LO, (TRAY_HI[0], TRAY_LO[1] + TRAY_WALL, TRAY_HI[2]), TRAY_GREY),
        _box((TRAY_LO[0], TRAY_HI[1] - TRAY_WALL, TRAY_LO[2]), TRAY_HI, TRAY_GREY),
    ],
)

OBJECTS = [BENCH, TRAY, SLOT, CARD]


# - world-frame helpers ----------------------------------------------------------------------------------------------
def _w(rel) -> np.ndarray:
    """Bench-relative (z above TOP) to world."""
    return np.asarray(rel, float) + np.array([0.0, 0.0, TOP])


def _cfw(p) -> np.ndarray:
    """Card-frame point (slot at rest) to world."""
    return CARD_ORIGIN + FRAME_R @ np.asarray(p, float)


def _cfw_box(lo, hi) -> tuple[np.ndarray, np.ndarray]:
    a, b = _cfw(lo), _cfw(hi)
    return np.minimum(a, b), np.maximum(a, b)


SLOT_TOP_W = CARD_ORIGIN[2] + SLOT_TOP_Z
_foot = _cfw_box((0.0405, -SLOT_HALF_W, 0.0), (0.1465, SLOT_HALF_W, 0.0))
SLOT_FOOT_LO, SLOT_FOOT_HI = _foot[0][:2], _foot[1][:2]
TAIL_REF_LOCAL = np.array([0.0, 0.0, FINGER_DEPTH])  # bottom corner of the card's tail end
TAIL_REF_REST_Z = CARD_ORIGIN[2] + FINGER_DEPTH

STATIC_HAZARDS = [("cooler", _w(COOLER_LO), _w(COOLER_HI))]
STATIC_HAZARDS += [(f"dimm{i}", _w(np.minimum(lo, slo)), _w(np.maximum(hi, shi)))
                   for i, ((lo, hi), (slo, shi)) in enumerate(zip(DIMMS, DIMM_SOCKETS))]


def _box_grid(lo, hi, n=(5, 3, 5)) -> np.ndarray:
    axes = [np.linspace(lo[i], hi[i], n[i]) for i in range(3)]
    g = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
    return g


def _box_surface(lo, hi, step=0.004) -> np.ndarray:
    """Points on the six faces of a box, at most ``step`` apart."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    n = [max(2, int(math.ceil((hi[i] - lo[i]) / step)) + 1) for i in range(3)]
    g = _box_grid(lo, hi, n)
    on = np.isclose(g, lo).any(1) | np.isclose(g, hi).any(1)
    return g[on]


CARD_SAMPLES = {name: _box_surface(lo, hi) for name, lo, hi in CARD_BOXES}
FINGER_SAMPLES = np.vstack([_box_surface(lo, hi, 0.003) for n, lo, hi in CARD_BOXES if n in FINGER_BOXES])
ALL_CARD_SAMPLES = np.vstack(list(CARD_SAMPLES.values()))


def _quat_xyzw_rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def card_pose(card) -> tuple[np.ndarray, np.ndarray]:
    """(rotation, origin) of the card frame in the world."""
    return _quat_xyzw_rot(card.quat_xyzw), np.asarray(card.origin, float)


def card_points(card, local: np.ndarray) -> np.ndarray:
    rot, origin = card_pose(card)
    return np.asarray(local, float) @ rot.T + origin


def _in_aabb(pts: np.ndarray, lo, hi, margin: float = 0.0) -> np.ndarray:
    lo, hi = np.asarray(lo) - margin, np.asarray(hi) + margin
    return np.all((pts >= lo) & (pts <= hi), axis=1)


# - arm watch: finger gaps and gripper probe points -------------------------------------------------------------------
ARMS = {"left": "left_robot", "right": "right_robot"}
PAD_HALF_WIDTH = 0.013
PAD_BACK_FROM_TCP = 0.034
PAD_TIP_PAST_TCP = 0.013
_HANDS: dict = {"arms": None}


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=float)


def gripper_probe_points(tcp, rot, finger_gap) -> np.ndarray:
    """TCP, pad tips and pad box samples of one gripper (world)."""
    tcp, rot = np.asarray(tcp, float), np.asarray(rot, float)
    c, w, a = rot[:, 0], rot[:, 1], rot[:, 2]
    half = 0.5 * float(finger_gap)
    pts = [tcp, tcp + PAD_TIP_PAST_TCP * a]
    for sgn in (-1.0, 1.0):
        for t in (half, half + 0.0064):
            for wo in (-PAD_HALF_WIDTH, 0.0, PAD_HALF_WIDTH):
                for zo in (-PAD_BACK_FROM_TCP, -0.5 * PAD_BACK_FROM_TCP, 0.0, PAD_TIP_PAST_TCP):
                    pts.append(tcp + sgn * t * c + wo * w + zo * a)
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
        tcp = np.asarray(tcp_world(robot, 0), float) - origin
        bodies = _np(robot.data.body_link_pose_w)[0]
        g = bodies[list(robot.body_names).index("gripper")]
        rot = _quat_xyzw_rot(g[3:7])
        out[arm] = {"finger_gap": gap, "tcp": tcp, "rot": rot, "points": gripper_probe_points(tcp, rot, gap)}
    return out


def set_arms(arms) -> None:
    _HANDS["arms"] = arms


# - live slot drive: pad preload and latch spring preload ------------------------------------------------------------
_TARGETS = {
    "pad_tail_p": -PAD_PRELOAD,
    "pad_front_p": -PAD_PRELOAD,
    "pad_tail_n": PAD_PRELOAD,
    "pad_front_n": PAD_PRELOAD,
    "latch": LATCH_PRELOAD,
    "hook": 0.0,
}


@mech_on_reset
def _seed_slot(environment) -> None:
    _apply_slot_targets(environment)


@on_step
def _watch(environment) -> None:
    _apply_slot_targets(environment)
    try:
        set_arms(read_arms(environment))
    except Exception:
        pass


def _apply_slot_targets(environment) -> None:
    for joint, value in _TARGETS.items():
        set_joint_target(environment, "slot", joint, value)


# - scoring ------------------------------------------------------------------------------------------------------------
_S: dict = {}


def _reset_episode() -> None:
    _S.clear()
    _S.update(violation=None, clear_steps=0, step=0, best=0.0, last={}, pressed_in_rise=False, free_ref=None,
              cache_obj=None, cache_score=0.0)


_reset_episode()


@on_reset
def _on_reset() -> None:
    _reset_episode()
    _HANDS["arms"] = None


def tail_lift(card) -> float:
    return float(card_points(card, TAIL_REF_LOCAL[None])[0, 2] - TAIL_REF_REST_Z)


JAW_REF_LOCAL = np.array([[FINGER_X0, 0.0, 0.0], [SLIT_X1, 0.0, 0.0]])  # bottom of the notch jaw under the hook


def edge_rise(card) -> float:
    """Largest rise of the card's tail corner or of the notch jaw held by the latch hook (any way the tail end leaves)."""
    jaw = float(card_points(card, JAW_REF_LOCAL)[:, 2].max() - CARD_ORIGIN[2])
    return max(tail_lift(card), jaw)


def connector_out(card) -> bool:
    """Gold fingers at least 10 mm above the slot top, or wholly outside the slot footprint seen from above."""
    pts = card_points(card, FINGER_SAMPLES)
    if float(pts[:, 2].min()) >= SLOT_TOP_W + 0.010:
        return True
    over = np.all((pts[:, :2] >= SLOT_FOOT_LO - 0.002) & (pts[:, :2] <= SLOT_FOOT_HI + 0.002), axis=1)
    return not bool(over.any())


def card_tilt_deg(card) -> float:
    """Angle between the PCB plane and the tray floor (0 = lying flat)."""
    rot, _ = card_pose(card)
    normal = rot[:, 1]
    return math.degrees(math.acos(min(1.0, abs(float(normal[2])))))


def card_in_tray(card) -> bool:
    pts = card_points(card, ALL_CARD_SAMPLES)
    lo = _w(TRAY_IN_LO) - np.array([0.0, 0.0, 0.0035])
    hi = np.array([_w(TRAY_IN_HI)[0], _w(TRAY_IN_HI)[1], TOP + 0.060])
    return bool(np.all(_in_aabb(pts, lo, hi)))


def card_resting(card) -> bool:
    return bool(np.linalg.norm(card.lin_vel) < 0.01 and np.linalg.norm(card.ang_vel) < 0.5)


def hazard_contact(card) -> str | None:
    """A card part touching the cooler or memory, or its gold fingers touching the board outside the slot."""
    pts = card_points(card, ALL_CARD_SAMPLES)
    for name, lo, hi in STATIC_HAZARDS:
        if bool(_in_aabb(pts, lo, hi, 0.001).any()):
            return name
    fp = card_points(card, FINGER_SAMPLES)
    near_board = fp[:, 2] <= _w(MB_HI)[2] + 0.001
    in_board = np.all((fp[:, :2] >= _w(MB_LO)[:2]) & (fp[:, :2] <= _w(MB_HI)[:2]), axis=1)
    in_slot = np.all((fp[:, :2] >= SLOT_FOOT_LO - 0.003) & (fp[:, :2] <= SLOT_FOOT_HI + 0.003), axis=1)
    if bool((near_board & in_board & ~in_slot).any()):
        return "board"
    return None


HOLD_DIST = 0.008


def card_held(card, arms) -> bool:
    """A gripper is closed (pad gap under 30 mm) with a pad within 8 mm of the card."""
    if not arms:
        return False
    pts = card_points(card, ALL_CARD_SAMPLES)
    for g in arms.values():
        if float(g["finger_gap"]) >= OPEN_GAP:
            continue
        p = np.asarray(g["points"], float)
        if float(np.sqrt(((p[:, None, :] - pts[None, :, :]) ** 2).sum(-1)).min()) <= HOLD_DIST:
            return True
    return False


def tab_top_world(latch: float) -> np.ndarray:
    """Centre of the orange tab top at latch angle ``latch`` (rad, hinge about the card y axis at LATCH_PIVOT)."""
    c, sn = math.cos(latch), math.sin(latch)
    rel = np.array([0.5 * (TAB_X[0] + TAB_X[1]), TAB_YC, TAB_TOP_Z]) - np.array(LATCH_PIVOT)
    rot = np.array([[c, 0.0, sn], [0.0, 1.0, 0.0], [-sn, 0.0, c]])
    return _cfw(np.array(LATCH_PIVOT) + rot @ rel)


def _grid(lo, hi, step) -> np.ndarray:
    axes = [np.linspace(lo[i], hi[i], max(2, int(math.ceil((hi[i] - lo[i]) / step)) + 1)) for i in range(3)]
    return np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)


# Both finger pads as solid boxes in the gripper frame (closing axis, pad width, approach), sampled every 2 mm.
_PAD_GRID = _grid((0.0, -PAD_HALF_WIDTH, -PAD_BACK_FROM_TCP), (0.0064, PAD_HALF_WIDTH, PAD_TIP_PAST_TCP), 0.002)


def finger_surface_points(tcp, rot, finger_gap) -> np.ndarray:
    """Dense points (2 mm grid) filling both finger pads of one gripper, in the world frame."""
    tcp, rot = np.asarray(tcp, float), np.asarray(rot, float)
    half = 0.5 * float(finger_gap)
    pts = []
    for sgn in (-1.0, 1.0):
        local = _PAD_GRID.copy()
        local[:, 0] = sgn * (half + local[:, 0])
        pts.append(local @ rot.T + tcp)
    return np.vstack(pts)


def tab_surface_distance(points, latch: float) -> np.ndarray:
    """Distance of each point to the orange tab-top rectangle (TAB_X x TAB_Y at TAB_TOP_Z) at latch angle ``latch``."""
    c, sn = math.cos(latch), math.sin(latch)
    rot = np.array([[c, 0.0, sn], [0.0, 1.0, 0.0], [-sn, 0.0, c]])
    card = (np.asarray(points, float) - CARD_ORIGIN) @ FRAME_R  # world -> card frame
    local = (card - np.array(LATCH_PIVOT)) @ rot + np.array(LATCH_PIVOT)  # undo the latch rotation
    dx = np.maximum(np.maximum(TAB_X[0] - local[:, 0], local[:, 0] - TAB_X[1]), 0.0)
    dy = np.maximum(np.maximum(TAB_Y[0] - local[:, 1], local[:, 1] - TAB_Y[1]), 0.0)
    dz = local[:, 2] - TAB_TOP_Z
    return np.sqrt(dx * dx + dy * dy + dz * dz)


def hand_on_tab(arms, latch: float = 0.0) -> bool:
    """Any part of a finger pad within TAB_HAND_DIST of the orange tab-top surface at the current latch angle."""
    if not arms:
        return False
    for g in arms.values():
        pts = finger_surface_points(g["tcp"], g["rot"], g["finger_gap"])
        if float(tab_surface_distance(pts, latch).min()) <= TAB_HAND_DIST:
            return True
    return False


def card_low(card) -> float:
    return float(card_points(card, ALL_CARD_SAMPLES)[:, 2].min())


def card_off_bench(card) -> bool:
    pts = card_points(card, ALL_CARD_SAMPLES)
    lo, hi = _w(MAT_LO), _w(MAT_HI)
    return bool(pts[:, 2].min() < TOP - 0.02 or (pts[:, 0] < lo[0]).any() or (pts[:, 0] > hi[0]).any()
                or (pts[:, 1] < lo[1]).any() or (pts[:, 1] > hi[1]).any())


def _part_points() -> dict:
    """Points of the parts the hands must clear at the end, other than the card (slot, latch tab, tray rim)."""
    return {
        "slot": _box_surface(*_cfw_box((TAB_X[0], TAB_Y[0], -0.002), (0.1465, SLOT_HALF_W, TAB_TOP_Z)), 0.006),
        "tray": np.vstack([_box_surface(_w(lo), _w(hi), 0.006) for lo, hi in (
            (TRAY_LO, (TRAY_HI[0], TRAY_LO[1] + TRAY_WALL, TRAY_HI[2])),
            ((TRAY_LO[0], TRAY_HI[1] - TRAY_WALL, TRAY_LO[2]), TRAY_HI),
            (TRAY_LO, (TRAY_LO[0] + TRAY_WALL, TRAY_HI[1], TRAY_HI[2])),
            ((TRAY_HI[0] - TRAY_WALL, TRAY_LO[1], TRAY_LO[2]), TRAY_HI),
            (TRAY_LO, (TRAY_HI[0], TRAY_HI[1], TRAY_LO[2] + TRAY_FLOOR)))]),
    }


CLEAR_DIST = 0.050
OPEN_GAP = 0.030


PART_POINTS = _part_points()


def hands_clear(card, arms) -> tuple[bool, float]:
    """Both grippers open (pad gap >= 30 mm) and every probe at least 50 mm from the card, slot and tray."""
    if not arms:
        return False, 0.0
    targets = np.vstack([card_points(card, ALL_CARD_SAMPLES), *PART_POINTS.values()])
    worst = np.inf
    ok = True
    for g in arms.values():
        if float(g["finger_gap"]) < OPEN_GAP:
            ok = False
        p = np.asarray(g["points"], float)
        d = np.sqrt(((p[:, None, :] - targets[None, :, :]) ** 2).sum(-1)).min()
        worst = min(worst, float(d))
    return ok and worst >= CLEAR_DIST, worst


def _evaluate(objects) -> float:
    card, slot = objects["card"], objects["slot"]
    _S["step"] += 1
    hook = float(slot.joints.get("hook", 0.0))
    latch = float(slot.joints.get("latch", 0.0))
    lift = tail_lift(card)
    rise = edge_rise(card)
    out = connector_out(card)
    tab_down = math.degrees(latch) <= TAB_UP_DEG and hand_on_tab(_HANDS["arms"], latch)
    if rise < SEATED_LIFT and not out:
        _S["pressed_in_rise"] = tab_down  # card seated: the rise starts now
    elif tab_down:
        _S["pressed_in_rise"] = True
    held = card_held(card, _HANDS["arms"])
    low = card_low(card)
    if held:
        _S["free_ref"] = None
    elif _S["free_ref"] is None:
        _S["free_ref"] = low
    if _S["violation"] is None:
        if (rise >= FORCED_LIFT or out) and not _S["pressed_in_rise"]:
            _S["violation"] = "latch_forced"
        elif _S["free_ref"] is not None and _S["free_ref"] - low > 0.030:
            _S["violation"] = "card_dropped"
        elif card_off_bench(card):
            _S["violation"] = "card_off_bench"
        else:
            hit = hazard_contact(card)
            if hit is not None:
                _S["violation"] = f"card_hit_{hit}"
    flat = card_tilt_deg(card) <= 10.0
    in_tray = card_in_tray(card)
    rest = card_resting(card)
    clear, worst = hands_clear(card, _HANDS["arms"])
    done = in_tray and flat and rest and clear and _S["violation"] is None
    _S["clear_steps"] = _S["clear_steps"] + 1 if done else 0
    if _S["violation"] is not None:
        s = 0.0
    elif _S["clear_steps"] >= HOLD_STEPS:
        s = 1.0
    elif out:
        s = 0.5
    elif lift >= 0.005:
        s = 0.25
    else:
        s = 0.0
    _S["best"] = max(_S["best"], s)
    _S["last"] = dict(score=s, lift_mm=round(lift * 1000, 1), out=out, flat=flat, in_tray=in_tray, rest=rest,
                      clear=clear, hand_mm=round(worst * 1000, 1) if np.isfinite(worst) else None,
                      latch_deg=round(math.degrees(latch), 1), hook_mm=round(hook * 1000, 2),
                      violation=_S["violation"], clear_steps=_S["clear_steps"],
                      joints={k: round(float(v) * 1000, 2) for k, v in slot.joints.items()})
    if _S["step"] % 20 == 0 or s != _S.get("prev_s"):
        rot, origin = card_pose(card)
        print(f"[score] step={_S['step']} {_S['last']} card_origin={np.round(origin, 4).tolist()} "
              f"tilt={card_tilt_deg(card):.1f}", flush=True)
        arms = _HANDS["arms"] or {}
        for k, g in arms.items():
            print(f"[arms] step={_S['step']} {k} tcp={np.round(g['tcp'], 4).tolist()} gap_mm={g['finger_gap'] * 1000:.1f}",
                  flush=True)
    _S["prev_s"] = s
    return s


def _step(objects) -> float:
    """Evaluate each step once: the embodiment calls success() then score() with the same states dict."""
    if objects is not _S["cache_obj"]:
        _S["cache_obj"] = objects
        _S["cache_score"] = _evaluate(objects)
    return _S["cache_score"]


def score(objects) -> float:
    return _step(objects)


def success(objects) -> bool:
    if "card" in objects:
        _step(objects)
    return _S["clear_steps"] >= HOLD_STEPS and _S["violation"] is None


def distance(objects) -> float:
    _, origin = card_pose(objects["card"])
    target = (_w(TRAY_IN_LO) + _w(TRAY_IN_HI)) / 2
    return float(np.linalg.norm(origin[:2] - target[:2]))


# - demo: open-loop joint plan (no sim state, no observations) ------------------------------------------------------
def _rot_axis(axis, deg) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    t = math.radians(deg)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


PRESS_TILT_DEG = 15.0
R_PRESS = _rot_axis((1, 0, 0), 20.0) @ _rot_axis((0, 1, 0), PRESS_TILT_DEG) @ grip_rotation(math.pi / 2)  # jaws close along x; hand leans out toward the tail side, clear of the cooler
R_GRIP = grip_rotation(-math.pi / 2)  # jaws close across the card thickness (world x)
R_CARRY = grip_rotation(0.0)  # turned 90 deg: card long axis along world x (tail toward +x) over the tray
LAY_DEG = 75.0
R_LAY = _rot_axis((1, 0, 0), -LAY_DEG) @ R_CARRY
TIP_HALF_T = 0.0064
LAY_X = 0.264
LAY_Y = 0.260
CARRY_Z = TOP + 0.160  # card bottom about 85 mm above the table, 20 mm over the cooler top
PEEL_RISE = 0.090  # commanded rise; the arm lags by force / ~350 N/m, so this builds the peel force
LAY_Z = TOP + 0.048


def press_tcp(tip_low_world) -> np.ndarray:
    """TCP for the tilted closed gripper whose lowest fingertip corner sits at ``tip_low_world``."""
    a, c = R_PRESS[:, 2], R_PRESS[:, 0]
    low_side = -c if c[2] > 0 else c  # the closing-axis side that points down
    tip_center = np.asarray(tip_low_world, float) - TIP_HALF_T * low_side
    return tip_center - PAD_TIP_PAST_TCP * a


TAB_W = _cfw((0.5 * (TAB_X[0] + TAB_X[1]), TAB_YC, TAB_TOP_Z))
CARD_TOP_W = CARD_ORIGIN[2] + CARD_H
GRIP_W = _cfw((0.5 * (BAND_X[0] + BAND_X[1]), 0.5 * (SHROUD_Y1 - PCB_HALF), CARD_H - 0.002))
GRIP_X, GRIP_Y = float(GRIP_W[0]), float(GRIP_W[1])
GRIP_T = SHROUD_Y1 + PCB_HALF  # grip thickness across PCB + shroud at the top edge


def demo():
    """Open-loop bimanual plan: right presses the latch tab, left peels and lifts the card and lays it in the tray."""
    t_open = -0.016  # 32 mm pad gap
    t_grip = -(GRIP_T - 0.009) / 2  # 9 mm narrower than the card: pads stop on the card and squeeze
    grip_z = CARD_TOP_W - 0.002
    tab_up = press_tcp(TAB_W + np.array([0.0, 0.0, 0.05]))
    tab_touch = press_tcp(TAB_W + np.array([0.0, 0.0, 0.001]))
    tab_press = press_tcp(TAB_W + np.array([0.0, 0.0, -0.022]))  # 22 mm past the open stop: ~8 N on the tab
    right_park = (0.16, -0.24, TOP + 0.20)
    right = [
        Waypoint(2.0, tuple(tab_up), FINGER_CLOSED, "over tab", R_PRESS),
        Waypoint(1.5, tuple(tab_touch), FINGER_CLOSED, "touch tab", R_PRESS),
        Waypoint(1.5, tuple(tab_press), FINGER_CLOSED, "press tab", R_PRESS),
        Waypoint(9.0, tuple(tab_press), FINGER_CLOSED, "hold tab", R_PRESS),
        Waypoint(1.0, tuple(tab_up), FINGER_CLOSED, "release tab", R_PRESS),
        Waypoint(2.0, right_park, FINGER_OPEN, "right park", grip_rotation(0.0)),
        Waypoint(30.0, right_park, FINGER_OPEN, "right wait", grip_rotation(0.0)),
    ]
    over = (GRIP_X, GRIP_Y, grip_z + 0.050)
    at = (GRIP_X, GRIP_Y, grip_z)
    # pitch of the gripper about the card-frame y axis; the arm lags, so the card tail peels first
    r_peel = _rot_axis(FRAME_R @ np.array([0.0, 1.0, 0.0]), -6.0) @ R_GRIP
    peel = (GRIP_X, GRIP_Y, grip_z + PEEL_RISE)
    lifted = (GRIP_X, GRIP_Y, CARRY_Z)
    over_tray = (LAY_X, LAY_Y, CARRY_Z)
    lay_hi = (LAY_X, LAY_Y, TOP + 0.110)
    lay = (LAY_X, LAY_Y, LAY_Z)
    slide_out = tuple(np.array(lay) - 0.045 * R_LAY[:, 2])
    left = [
        Waypoint(2.0, over, t_open, "over band", R_GRIP),
        Waypoint(1.5, at, t_open, "to band", R_GRIP),
        Waypoint(1.0, at, t_open, "settle", R_GRIP),
        Waypoint(1.0, at, t_grip, "grip band", R_GRIP),
        Waypoint(0.5, at, t_grip, "hold grip", R_GRIP),
        Waypoint(4.0, peel, t_grip, "peel tail", r_peel),
        Waypoint(1.5, peel, t_grip, "hold peel", r_peel),
        Waypoint(2.5, lifted, t_grip, "lift out", R_GRIP),
        Waypoint(3.0, over_tray, t_grip, "carry and turn", R_CARRY),
        Waypoint(2.5, lay_hi, t_grip, "tilt", R_LAY),
        Waypoint(2.0, lay, t_grip, "set edge down", R_LAY),
        Waypoint(1.0, lay, t_grip, "settle edge", R_LAY),
        Waypoint(1.0, lay, -0.012, "release", R_LAY),
        Waypoint(0.5, lay, -0.012, "let card rest", R_LAY),
        Waypoint(2.0, slide_out, -0.012, "slide fingers out", R_LAY),
        Waypoint(1.5, (LAY_X, LAY_Y - 0.010, TOP + 0.120), -0.012, "back off", R_LAY),
        Waypoint(1.5, (0.20, 0.33, TOP + 0.20), FINGER_OPEN, "left park", R_CARRY),
        Waypoint(6.0, (0.20, 0.33, TOP + 0.20), FINGER_OPEN, "left rest", R_CARRY),
    ]
    return {"left": left, "right": right}


# - offline checks ------------------------------------------------------------------------------------------------------
def self_test() -> None:
    print("pcie-card-latch-pull offline check")
    print("objects:", [o.name for o in OBJECTS])
    import offline_check

    offline_check.run(sys.modules[__name__])


if __name__ == "__main__":
    self_test()
