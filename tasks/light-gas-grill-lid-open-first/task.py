"""A Weber Q 1000 portable propane grill on the station table: cast aluminium cookbox on a black frame, a hinged cast-aluminium lid with a front
handle bar, a push-and-turn burner control knob (red cover) on the front right of the cookbox (OFF, START/HI, LO) and a green 1 lb propane cylinder
on the right side. Turn the grill on.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/light-gas-grill-lid-open-first/task.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Turn on the grill with its red knob." --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit  # noqa: E402,F401
import mechkit as m  # noqa: E402
import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # score() counts its own 20-step clear hold; success() agrees

INSTRUCTION = "Turn on the grill with its red knob."

HOLD_STEPS = 20
LID_OPEN_DEG = 70.0  # white tape mark on the angle card
LID_STOP_DEG = 90.0  # lid open 59.7 cm high = hinge height 21.7 cm + lid depth 38 cm: the lid stands vertical on its stop
KNOB_OFF_DEG = 10.0  # pointer past this = knob has left OFF
KNOB_HI_DEG = 90.0  # START/HI, the lighting position (Copreci rotation diagram: OFF to max 90 deg)
KNOB_LIT_MIN = 80.0  # START/HI window used by the scorer: 80 to 100 deg
KNOB_LIT_MAX = 100.0
KNOB_STOP_DEG = 120.0  # LO end stop (Copreci rotation diagram: min at 120 deg)
PUSH_TRAVEL = 0.0035  # spindle stroke, Copreci CAL 20703 datasheet: "Stroke: 3,5 mm"
GATE_PUSH = 0.0030  # the push-to-turn latch releases after this much of the stroke (m)
PUSH_SPRING = 20.0 / PUSH_TRAVEL  # N/m: 20 N at the full stroke = datasheet "Max. pushing strenght: 20 N"
TURN_FRICTION = 0.2  # N m, datasheet "Max. turning torque of the spindle: 0.2 N m"
CLEAR_DIST = 0.050
GRIP_OPEN_MIN = 0.050  # pad gap for "gripper open"
REST_DEG_PER_STEP = 0.5
PART_FRICTION = (0.6, 0.5)  # static, dynamic: every grill part (cast aluminium, nylon handle and knob cover)
LATCH_FRICTION = (1.0, 0.8)  # internal latch stop (mechkit fixed-part default); never touched by the grippers

LOOKS = {
    "black": m.Look(color=m.rgb(0.07, 0.07, 0.08), roughness=0.55),
    "alu": m.Look(color=m.rgb(0.55, 0.56, 0.58), metallic=0.6, roughness=0.45),
    "lid": m.Look(color=m.rgb(0.62, 0.63, 0.66), metallic=0.5, roughness=0.4),
    "red": m.Look(color=m.rgb(0.95, 0.03, 0.03), roughness=0.35),
    "darkred": m.Look(color=m.rgb(0.55, 0.05, 0.05), roughness=0.5),
    "white": m.Look(color=m.rgb(0.95, 0.95, 0.93), roughness=0.5),
    "green": m.Look(color=m.rgb(0.10, 0.62, 0.22), roughness=0.45),
    "brass": m.Look(color=m.rgb(0.75, 0.60, 0.20), metallic=0.8, roughness=0.35),
    "grey": m.Look(color=m.rgb(0.45, 0.46, 0.48), metallic=0.6, roughness=0.4),
}

# - geometry (station frame, metres; z measured up from the table top) -------------------------------------------
# Weber Q 1000 published size: lid closed 36.9 H x 68.6 W x 42.0 D cm, lid open 59.7 H x 52.1 D cm (weber.com spec sheet).
XF = 0.22  # front face of cookbox and lid
XH = 0.60  # hinge axis x (rear edge): cookbox and lid 0.38 m deep
ZH = 0.217  # hinge height = cookbox rim (36.9 cm closed - 15.2 cm lid dome)
FRAME_H = 0.07  # black frame under the cookbox
BODY_HALF_W = 0.25  # cookbox and lid 0.50 m wide
FRAME_HALF_W = 0.30  # frame 0.60 m, plus side carry grips to 0.686 m
GRIP_HALF_W = 0.343
LID_HALF_W = 0.25
LID_H = 0.152
LID_MASS = 2.7  # replacement lid 60058, listed shipping weight 6 lb (2.7 kg)
LID_COM = (XH - 0.19, 0.0, ZH + 0.09)  # dome shell: the lid balances at about 65 deg and rests on its 90 deg stop above that
HB = (XF - 0.035, 0.0, ZH + 0.11)  # handle bar centre at lid angle 0
HB_R = 0.011  # 22 mm bar
HB_LEN = 0.30
HB_POST_Y = 0.12
KNOB = (XF, -0.21, 0.10)  # knob axis point on the cookbox front (axis along x), front right
FIN = (0.020, 0.012, 0.040)  # knob grip bar: 20 mm deep (18 mm proud of the cap), 12 mm across (y) at OFF, 40 mm tall
CAP_R = 0.028  # 56 mm knob cap
LUG_R = 0.020  # internal latch lug radius (inside the valve body, behind the panel)
LUG_X = XF + 0.006  # lug spans x XF+3 mm to XF+9 mm at rest
STOP_X0, STOP_X1 = XF + 0.001, LUG_X - 0.003 + GATE_PUSH  # latch stop: overlaps the lug by GATE_PUSH in depth at rest
CYL = (XF + 0.25, -0.40, 0.12)  # 1 lb cylinder lying along y on the right side, into the regulator
REG = (XF + 0.25, -0.27, 0.12)
CARD_Y = LID_HALF_W + 0.012  # printed angle card on the left of the hinge


def _box(parts, cols, name, size, center, look, collide=True):
    parts.append((name, m.box(size, center), look))
    if collide:
        cols.append(m.col_box(f"{name}_c", size, center))


def _hull(parts, cols, name, mesh, look):
    parts.append((name, mesh, look))
    cols.append(m.col_hull(f"{name}_c", mesh))


def _grill() -> m.Part:
    fp, fc = [], []
    d = XH - XF
    xc = XF + d / 2
    _box(fp, fc, "frame", (d, 2 * FRAME_HALF_W, FRAME_H), (xc, 0.0, FRAME_H / 2), "black")
    for s, n in ((1.0, "l"), (-1.0, "r")):
        _box(fp, fc, f"carry_grip_{n}", (0.20, GRIP_HALF_W - FRAME_HALF_W, 0.04), (xc, s * (FRAME_HALF_W + GRIP_HALF_W) / 2, 0.05), "black")
    _box(fp, fc, "cookbox", (d, 2 * BODY_HALF_W, ZH - FRAME_H), (xc, 0.0, FRAME_H + (ZH - FRAME_H) / 2), "alu")
    _box(fp, fc, "grate", (0.32, 0.43, 0.004), (xc, 0.0, ZH - 0.01), "grey", collide=False)
    kx, ky, kz = KNOB
    # OFF / 10 deg latch line / START-HI / LO printed dial sticker around the knob (visual only)
    for deg, look in ((0.0, "black"), (KNOB_OFF_DEG, "white"), (KNOB_LIT_MIN, "white"), (KNOB_HI_DEG, "black"), (KNOB_LIT_MAX, "white"), (KNOB_STOP_DEG, "black")):
        a = math.radians(deg)
        fp.append((f"tick{int(deg)}", m.box((0.0015, 0.008, 0.004), (XF - 0.0008, ky + 0.040 * math.sin(a), kz + 0.040 * math.cos(a))), look))
    # printed angle card (visual only) standing on the frame at the left of the hinge, white tape marks at 45 and 70 deg, 0.11 m from the axis
    fp.append(("angle_card", m.box((0.20, 0.003, ZH + 0.16 - FRAME_H), (XH - 0.08, CARD_Y, FRAME_H + (ZH + 0.16 - FRAME_H) / 2)), "black"))
    for deg in (45.0, LID_OPEN_DEG):
        a = math.radians(deg)
        fp.append((f"tape{int(deg)}", m.box((0.03, 0.004, 0.006), (XH - 0.11 * math.cos(a), CARD_Y + 0.002, ZH + 0.11 * math.sin(a))), "white"))
    _box(fp, fc, "regulator", (0.05, 0.04, 0.05), REG, "brass")

    # the lid: an inverted cast tray hinged on its rear lower edge, with a front handle bar on two posts
    lp, lc = [], []
    _box(lp, lc, "lid_top", (d, 2 * LID_HALF_W, 0.012), (xc, 0.0, ZH + LID_H - 0.006), "lid")
    _box(lp, lc, "lid_front", (0.012, 2 * LID_HALF_W, LID_H), (XF + 0.006, 0.0, ZH + LID_H / 2), "lid")
    _box(lp, lc, "lid_rear", (0.012, 2 * LID_HALF_W, LID_H), (XH - 0.006, 0.0, ZH + LID_H / 2), "lid")
    for s, n in ((1.0, "l"), (-1.0, "r")):
        _box(lp, lc, f"lid_side_{n}", (d, 0.012, LID_H), (xc, s * (LID_HALF_W - 0.006), ZH + LID_H / 2), "lid")
        _box(lp, lc, f"post_{n}", (XF - HB[0] + 0.004 + 0.002, 0.014, 0.016), (0.5 * (XF + 0.004 + HB[0]), s * HB_POST_Y, HB[2]), "black")
    bar = m.cylinder(HB_R, HB_LEN, HB, axis="y", segments=20)
    _hull(lp, lc, "handle", bar, "black")
    lid = m.hinge("lid", "Y", (XH, 0.0, ZH), 0.0, LID_STOP_DEG, lp, lc, mass=LID_MASS, damping=0.8, friction=0.02, gravity=True, com=LID_COM)

    # the knob: a push slide carrying a turn hinge; the push-to-turn latch is a lug on the turn link inside the valve body
    push = m.slider("push", "X", 0.0, PUSH_TRAVEL, [("shaft", m.cylinder(0.004, 0.020, (XF, ky, kz), axis="x"), "grey")],
                    [], at=(XF, ky, kz), mass=0.03, spring=PUSH_SPRING, damping=3.0, friction=0.0)
    tp, tc = [], []
    cap = m.cylinder(CAP_R, 0.014, (XF - 0.017, ky, kz), axis="x", segments=32)
    _hull(tp, tc, "cap", cap, "red")
    fin = m.box(FIN, (XF - 0.032, ky, kz))
    _hull(tp, tc, "fin", fin, "red")
    tp.append(("pointer", m.box((0.0015, 0.005, 0.026), (XF - 0.0428, ky, kz + 0.013)), "white"))
    lug = m.box((0.006, 0.006, 0.005), (LUG_X, ky + LUG_R, kz))
    _hull(tp, tc, "lug", lug, "darkred")
    turn = m.hinge("turn", "X", (XF, ky, kz), -KNOB_STOP_DEG, 0.0, tp, tc, mass=0.05, damping=0.05, friction=TURN_FRICTION, parent="push")
    return m.mechanism("grill", (0.0, 0.0, TOP), [lid, push, turn], LOOKS, frame_parts=fp, frame_colliders=fc, friction=PART_FRICTION)


def _latch() -> m.Part:
    """The valve's own push-to-turn latch, modelled as a fixed stop inside the cookbox front wall, below the lug (hidden from view)."""
    kx, ky, kz = KNOB
    size = (STOP_X1 - STOP_X0, 0.006, 0.004)
    center = (0.5 * (STOP_X0 + STOP_X1), ky + LUG_R, kz - 0.0035 - 0.002)
    return m.fixed("latch", (0.0, 0.0, TOP), [("stop", m.box(size, center), "darkred")], [m.col_box("stop_c", size, center)], LOOKS,
                   friction=LATCH_FRICTION)


def _cylinder() -> m.Part:
    tank = m.cylinder(0.0525, 0.20, CYL, axis="y", segments=32)
    return m.fixed("propane", (0.0, 0.0, TOP), [("tank", tank, "green")], [m.col_hull("tank_c", tank)], LOOKS, friction=PART_FRICTION)


OBJECTS = [_grill(), _latch(), _cylinder()]

# - reading the arms ---------------------------------------------------------------------------------------------
ARMS = {"left": "left_robot", "right": "right_robot"}
PAD_HALF_WIDTH, PAD_THICK, PAD_BACK, PAD_TIP = 0.020, 0.010, 0.021, 0.013


def _np(x):
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=float)


def _pad_points(tcp, gap, rot):
    close, width, appr = rot[:, 0], rot[:, 1], rot[:, 2]
    pts = [tuple(tcp), tuple(tcp + PAD_TIP * appr)]
    for sign in (-1.0, 1.0):
        for t in (gap / 2, gap / 2 + PAD_THICK):
            for w in (-PAD_HALF_WIDTH, 0.0, PAD_HALF_WIDTH):
                for z in (-PAD_BACK, -0.5 * PAD_BACK, 0.0, PAD_TIP):
                    pts.append(tuple(tcp + sign * t * close + w * width + z * appr))
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
        gap = abs(float(q[joints.index("joint7")])) + abs(float(q[joints.index("joint8")]))
        tcp = tcp_world(robot, 0) - origin
        bodies = _np(robot.data.body_link_pose_w)[0]
        gp = bodies[list(robot.body_names).index("gripper")]
        x, y, z, w = (float(v) for v in gp[3:7])
        rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        out[arm] = {"gap": gap, "points": _pad_points(tcp, gap, rot)}
    return out


# - clearance ------------------------------------------------------------------------------------------------------
def _box_dist(pts, center, size, rot=None):
    r = np.eye(3) if rot is None else rot
    local = (np.asarray(pts, float) - np.asarray(center, float)) @ r
    q = np.abs(local) - np.asarray(size, float) / 2.0
    return float((np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(np.max(q, axis=1), 0.0)).min())


def _ry(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _wz(h):
    return TOP + h


def min_clearance(points, lid_deg):
    """Smallest distance (m) from any gripper probe point to the frame, cookbox, lid, handle, knob zone, cylinder, regulator or angle card."""
    pts = np.asarray(points, float)
    d = XH - XF
    xc = XF + d / 2
    best = _box_dist(pts, (xc, 0.0, _wz(FRAME_H / 2)), (d, 2 * GRIP_HALF_W, FRAME_H))
    best = min(best, _box_dist(pts, (xc, 0.0, _wz((FRAME_H + ZH) / 2)), (d, 2 * BODY_HALF_W, ZH - FRAME_H)))
    hinge = np.array([XH, 0.0, _wz(ZH)])
    rot = _ry(lid_deg)
    lid_c = hinge + rot @ np.array([-d / 2, 0.0, LID_H / 2])
    best = min(best, _box_dist(pts, lid_c, (d, 2 * LID_HALF_W, LID_H), rot))
    hb = hinge + rot @ np.array([HB[0] - XH, 0.0, HB[2] - ZH])
    best = min(best, _box_dist(pts, hb, (2 * HB_R + 0.03, HB_LEN, 2 * HB_R), rot))
    kx, ky, kz = KNOB
    best = min(best, _box_dist(pts, (kx - 0.025, ky, _wz(kz)), (0.05, 2 * CAP_R + 0.004, 2 * CAP_R + 0.004)))
    best = min(best, _box_dist(pts, (CYL[0], CYL[1], _wz(CYL[2])), (0.105, 0.20, 0.105)))
    best = min(best, _box_dist(pts, (REG[0], REG[1], _wz(REG[2])), (0.05, 0.04, 0.05)))
    best = min(best, _box_dist(pts, (XH - 0.08, CARD_Y, _wz(FRAME_H + (ZH + 0.16 - FRAME_H) / 2)), (0.20, 0.003, ZH + 0.16 - FRAME_H)))
    return best


# - the trial ------------------------------------------------------------------------------------------------------
class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps = 0
        self.latched = False
        self.latch_reason = ""
        self.hold = 0
        self.prev = None
        self.lid = 0.0
        self.push = 0.0
        self.turn = 0.0
        self.score = 0.0
        self.full = False
        self.arms = None

    def update(self, lid_deg, push_m, turn_deg, arms=None):
        """One control step: joint values (deg, m, deg) and the arms ({'left': {gap, points}, ...} or None)."""
        self.steps += 1
        self.lid, self.push, self.turn, self.arms = lid_deg, push_m, turn_deg, arms
        if turn_deg > KNOB_OFF_DEG and lid_deg < LID_OPEN_DEG and not self.latched:
            self.latched, self.latch_reason = True, "gas on with the lid below the 70 deg mark"
        still = self.prev is not None and abs(lid_deg - self.prev[0]) < REST_DEG_PER_STEP and abs(turn_deg - self.prev[1]) < REST_DEG_PER_STEP
        self.prev = (lid_deg, turn_deg)
        clear = False
        if arms is not None:
            clear = all(a["gap"] >= GRIP_OPEN_MIN and min_clearance(a["points"], lid_deg) >= CLEAR_DIST for a in arms.values())
        ok = lid_deg >= LID_OPEN_DEG and KNOB_LIT_MIN <= turn_deg <= KNOB_LIT_MAX and clear and still and not self.latched
        self.hold = self.hold + 1 if ok else 0
        self.full = self.hold >= HOLD_STEPS
        self.score = self._grade()

    def _grade(self):
        if self.latched or self.lid < LID_OPEN_DEG:
            return 0.0
        if self.full:
            return 1.0
        if self.turn >= KNOB_LIT_MIN:
            return 0.5
        return 0.25

    def step(self, env):
        st = m.state(env, "grill")
        self.update(st.deg("lid"), st.joint("push"), -st.deg("turn"), read_arms(env))
        if self.steps % 5 == 0:
            clr = {k: round(min_clearance(v["points"], self.lid), 3) for k, v in self.arms.items()}
            gap = {k: round(v["gap"], 3) for k, v in self.arms.items()}
            print(f"[grill] step={self.steps} lid={self.lid:.1f} push={self.push * 1000:.1f}mm turn={self.turn:.1f} hold={self.hold} score={self.score} gap={gap} clear={clr}", flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


def distance(objects) -> float:
    return max(0.0, LID_OPEN_DEG - TRIAL.lid) / 100.0 + abs(KNOB_HI_DEG - TRIAL.turn) / 100.0


# - open-loop demo ------------------------------------------------------------------------------------------------
def _grip(pitch_deg, yaw, roll_deg=0.0):
    a = math.radians(roll_deg)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])
    return rx @ _ry(pitch_deg - 90.0) @ grip_rotation(yaw)


def _handle_at(deg):
    a = math.radians(deg)
    rx, rz = HB[0] - XH, HB[2] - ZH
    return (XH + rx * math.cos(a) + rz * math.sin(a), HB[1], _wz(ZH - rx * math.sin(a) + rz * math.cos(a)))


def _lift_pitch(deg):
    return float(np.interp(deg, (0.0, 16.0, 32.0, 40.0, 48.0), (0.0, 5.0, -5.0, -15.0, -25.0)))


LIFT_MAX = 48.0  # the handle leaves the YAM workspace above about 50 deg, so the right arm takes over on the lid side wall
PUSH_R = 0.26  # right arm pushes the bottom edge of the lid's right side wall, 0.26 m from the hinge axis
TURN_CMD = 92.0  # commanded wrist roll for START/HI; the YAM wrist reaches about 92 deg here with a few mm of IK error
PUSH_TO = 80.0  # past the 65 deg balance point; the lid then falls back onto its 90 deg stop


def _edge_at(deg, below=0.012):
    a = math.radians(deg)
    return (XH - PUSH_R * math.cos(a) - below * math.sin(a), -(LID_HALF_W - 0.006), _wz(ZH + PUSH_R * math.sin(a) - below * math.cos(a)))


def demo():
    """Open loop, deterministic. Left arm pinches the 22 mm handle bar and lifts the lid along its hinge arc to 48 deg and holds it there.
    The right arm puts its closed fingers under the bottom edge of the lid's right side wall, the left arm lets go, and the right arm pushes
    the lid past its 65 deg balance point to 80 deg; the lid settles on its 90 deg stop. The right arm then pinches the knob grip bar,
    pushes it in, turns it anticlockwise to START/HI (90 deg), lets go and backs off; both arms rest."""
    bar_close = -(2 * HB_R - 0.006) / 2
    g0 = _grip(_lift_pitch(0.0), -math.pi / 2)
    p0 = _handle_at(0.0)
    park_l = (0.18, 0.28, _wz(0.34))
    left = [
        Waypoint(2.5, (p0[0], 0.12, p0[2] + 0.04), FINGER_OPEN, "left approach", g0),
        Waypoint(1.0, (p0[0], 0.10, p0[2]), FINGER_OPEN, "beside handle", g0),
        Waypoint(1.5, p0, FINGER_OPEN, "to handle", g0),
        Waypoint(0.6, p0, FINGER_OPEN, "settle", g0),
        Waypoint(1.0, p0, bar_close, "close handle", g0),
    ]
    for deg in (8, 16, 24, 32, 40, LIFT_MAX):
        left.append(Waypoint(0.6, _handle_at(deg), bar_close, f"lift {deg:g}", _grip(_lift_pitch(deg), -math.pi / 2)))
    top = _handle_at(LIFT_MAX)
    gt = _grip(_lift_pitch(LIFT_MAX), -math.pi / 2)
    t_hold = sum(w.duration for w in left)
    left += [
        Waypoint(5.0, top, bar_close, "hold handle", gt),
        Waypoint(0.8, top, FINGER_OPEN, "release lid", gt),
        Waypoint(1.2, (top[0] - 0.10, top[1] + 0.06, top[2] - 0.03), FINGER_OPEN, "back off", gt),
        Waypoint(1.5, park_l, FINGER_OPEN, "left park", g0),
        Waypoint(3.0, park_l, FINGER_OPEN, "left rest", g0),
    ]
    park_r = (0.18, -0.28, _wz(0.30))
    gp = _grip(0.0, 0.0)
    gpark = _grip(10.0, 0.0)
    e0 = _edge_at(LIFT_MAX)
    right = [
        Waypoint(3.0, park_r, 0.0, "right park", gpark),
        Waypoint(t_hold - 3.0, park_r, 0.0, "right wait lid", gpark),
        Waypoint(2.0, (e0[0] - 0.06, e0[1], e0[2] - 0.06), 0.0, "below lid edge", gp),
        Waypoint(1.5, e0, 0.0, "under lid edge", gp),
        Waypoint(1.5, e0, 0.0, "take lid", gp),
        Waypoint(1.0, e0, 0.0, "hold lid", gp),
    ]
    for deg in (55, 62, 69, 75, PUSH_TO):
        right.append(Waypoint(0.7, _edge_at(deg), 0.0, f"push lid {deg:g}", gp))
    eN = _edge_at(PUSH_TO)
    right += [
        Waypoint(0.8, eN, 0.0, "lid over", gp),
        Waypoint(1.2, (eN[0] - 0.12, eN[1], eN[2] - 0.08), 0.0, "leave lid", gp),
    ]
    kx, ky, kz = KNOB
    fin_x = kx - 0.040
    kp = (fin_x, ky, _wz(kz))
    gk = _grip(10.0, 0.0)
    fin_close = -(FIN[1] - 0.006) / 2
    right += [
        Waypoint(1.8, (fin_x, ky, _wz(kz + 0.08)), FINGER_OPEN, "above knob", gk),
        Waypoint(1.2, (fin_x, ky, _wz(kz + 0.04)), FINGER_OPEN, "front of knob", gk),
        Waypoint(0.6, (fin_x, ky, _wz(kz + 0.02)), FINGER_OPEN, "aligned in front", gk),
        Waypoint(1.5, kp, FINGER_OPEN, "to fin", gk),
        Waypoint(0.6, kp, FINGER_OPEN, "settle", gk),
        Waypoint(1.0, kp, fin_close, "close fin", gk),
        Waypoint(0.8, (fin_x + 0.006, ky, _wz(kz)), fin_close, "push in", gk),
    ]
    for deg in (25, 50, 75, TURN_CMD):
        right.append(Waypoint(0.7, (fin_x + 0.006, ky, _wz(kz)), fin_close, f"turn {deg:g}", _grip(10.0, 0.0, -deg)))
    gt2 = _grip(10.0, 0.0, -TURN_CMD)
    right += [
        Waypoint(0.6, (fin_x + 0.006, ky, _wz(kz)), fin_close, "hold HI", gt2),
        Waypoint(0.8, (fin_x + 0.006, ky, _wz(kz)), FINGER_OPEN, "release knob", gt2),
        Waypoint(1.0, (fin_x - 0.10, ky, _wz(kz)), FINGER_OPEN, "back off", gt2),
        Waypoint(1.5, park_r, FINGER_OPEN, "right park", gk),
        Waypoint(3.0, park_r, FINGER_OPEN, "right rest", gk),
    ]
    return {"left": left, "right": right}


# - offline checks -------------------------------------------------------------------------------------------------
def _arms(gap=0.094, far=True):
    pts = [(0.0, 0.3, _wz(0.5))] if far else [(0.17, -0.21, _wz(0.10))]
    return {"left": {"gap": gap, "points": pts}, "right": {"gap": gap, "points": [(0.0, -0.3, _wz(0.5))]}}


def _check_scoring():
    T = Trial()
    cases = []

    def run(seq, arms=None):
        T.reset()
        for lid, push, turn, a in seq:
            T.update(lid, push, turn, a)
        return T.score

    clear = _arms()
    assert run([(0, 0, 0, clear)] * 5) == 0.0
    cases.append("nothing -> 0")
    assert run([(90, 0, 0, clear)] * 5) == 0.25
    assert run([(90, 0, 60, clear)] * 5) == 0.25
    cases.append("lid open, knob short of START/HI -> 0.25")
    assert run([(90, 0, 85, clear)] * 3) == 0.5
    cases.append("START/HI with lid open, no clear hold yet -> 0.5")
    assert run([(90, 0, 115, clear)] * 40) == 0.5
    cases.append("knob turned on past START/HI toward LO -> 0.5")
    seq = [(10, 0, 0, clear), (10, 0.003, 15, clear)] + [(90, 0, 90, clear)] * 30
    assert run(seq) == 0.0 and T.latched
    cases.append("gas on with lid shut then opened -> latched 0")
    seq = [(90, 0, 90, clear)] * 3 + [(40, 0, 90, clear)] + [(90, 0, 90, clear)] * 30
    assert run(seq) == 0.0
    cases.append("lid dropped while the knob is on -> latched 0")
    seq = [(90, 0, 90, clear)] * (HOLD_STEPS + 1)
    assert run(seq) == 1.0 and T.full
    cases.append("full 1.0 after clear hold")
    seq = [(90, 0, 90, clear)] * (HOLD_STEPS - 1)
    assert run(seq) == 0.5
    cases.append("hold of 19 steps -> 0.5")
    hand = {"left": {"gap": 0.094, "points": [(0.0, 0.3, _wz(0.5))]}, "right": {"gap": 0.014, "points": [(0.18, -0.21, _wz(0.10))]}}
    assert run([(90, 0, 90, hand)] * 40) == 0.5
    cases.append("gripper on knob -> 0.5")
    hand2 = {"left": {"gap": 0.094, "points": [(0.0, 0.3, _wz(0.5))]}, "right": {"gap": 0.094, "points": [(0.17, -0.21, _wz(0.10))]}}
    assert run([(90, 0, 90, hand2)] * 40) == 0.5
    cases.append("open gripper within 50 mm of knob -> 0.5")
    hp = _handle_at(90.0)
    handle = {"left": {"gap": 0.094, "points": [hp]}, "right": clear["right"]}
    assert run([(90, 0, 90, handle)] * 40) == 0.5
    cases.append("hand at lid handle -> 0.5")
    edge = {"left": clear["left"], "right": {"gap": 0.094, "points": [_edge_at(75.0, below=0.0)]}}
    assert run([(75, 0, 90, edge)] * 40) == 0.5
    cases.append("hand propping the lid at 75 deg -> 0.5")
    seq = [(90, 0, 80 + i, clear) for i in range(10)] + [(90, 0, 90, clear)] * 5
    assert run(seq) == 0.5
    cases.append("moving knob is not still -> 0.5")
    seq = [(90, 0, 90, clear)] * 3 + [(90, 0, 0, clear)] * 3
    assert run(seq) == 0.25
    cases.append("knob returned to OFF -> 0.25")
    assert distance(None) >= 0.0 and success(None) == T.full
    return cases


def _check_demo(hz=20):
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan

    plans = demo()
    out = []
    for arm, wps in plans.items():
        base = np.asarray(station.BASES[arm], float)
        plan = ArmPlan(list(wps), base)
        worst_end = worst_grip = 0.0
        ends = set()
        t = 0.0
        for w in wps:
            t += w.duration
            ends.add(round(t * hz))
        for k in range(1, round(plan.duration * hz) + 1):
            t = k / hz
            act, label = plan.action7(t)
            assert np.all(act[:6] >= ik.LOWER - 1e-6) and np.all(act[:6] <= ik.UPPER + 1e-6), (arm, t, label, act[:6])
            pos, rot, _, _ = plan.target(t)
            p, r = ik.tcp_pose(act[:6], base)
            e = float(np.linalg.norm(p - pos)) * 1000
            a = math.degrees(math.acos(np.clip((np.trace(r.T @ rot) - 1) / 2, -1, 1)))
            if k in ends and label in ("aligned in front", "to handle", "to fin", "settle", "close handle", "close fin", "push in", "hold handle", "hold HI", "under lid edge", "take lid") or (k in ends and label.startswith(("lift", "turn", "push lid"))):
                worst_end = max(worst_end, e)
                tol = 4.0 if label.startswith(("turn", "hold HI")) else 2.0  # the wrist roll nears its limit at 92 deg
                assert e < tol and a < 2.0, (arm, round(t, 2), label, e, a)
            if label.startswith(("lift", "turn", "close", "push", "settle", "hold", "take")):
                worst_grip = max(worst_grip, e)
                assert e < 3.0, (arm, round(t, 2), label, e)
        p, r = ik.tcp_pose(plan.action7(plan.duration)[0][:6], base)
        rest = min_clearance(_pad_points(p, 2 * abs(FINGER_OPEN), r), LID_STOP_DEG)
        assert rest >= CLEAR_DIST + 0.010, (arm, "rest pose clearance", rest)
        out.append(f"{arm}: {plan.duration:.1f} s plan, IK error at waypoint ends <= {worst_end:.2f} mm, during grip/lift/turn <= {worst_grip:.2f} mm, rest clearance {rest * 1000:.0f} mm")
    return out


def _check_geometry():
    """The internal latch lug must hit its stop at rest, stay blocked for any push under GATE_PUSH and clear it within the stroke."""
    kx, ky, kz = KNOB
    lug_x0, lug_x1 = LUG_X - 0.003, LUG_X + 0.003
    assert lug_x1 > STOP_X0 and lug_x0 < STOP_X1, "lug must overlap the stop at rest"
    assert abs((STOP_X1 - lug_x0) - GATE_PUSH) < 1e-9, "lug/stop overlap at rest equals GATE_PUSH"
    for short in (0.0, 0.0015, GATE_PUSH - 0.0001):
        assert lug_x0 + short < STOP_X1 - 1e-6, f"a push of {short * 1000:.1f} mm must stay blocked"
    assert lug_x0 + PUSH_TRAVEL > STOP_X1 + 0.0004, "a full stroke clears the latch with margin"
    assert STOP_X0 > XF, "latch stop is inside the cookbox front wall, hidden"
    assert kz - 0.0035 < kz - 0.0025, "stop below the lug at rest (the knob turns to negative angles)"
    assert abs(PUSH_SPRING * PUSH_TRAVEL - 20.0) < 1e-6 and TURN_FRICTION == 0.2
    assert abs(_handle_at(0.0)[0] - HB[0]) < 1e-9
    a = math.degrees(math.atan2(XH - LID_COM[0], LID_COM[2] - ZH))
    assert 60.0 < a < LID_OPEN_DEG < PUSH_TO, a
    return [f"latch: lug overlaps stop {GATE_PUSH * 1000:.1f} mm, pushes of 0, 1.5 and 2.9 mm stay blocked, the 3.5 mm stroke clears it; lid balances at {a:.1f} deg"]


if __name__ == "__main__":
    cases = _check_scoring()
    print(f"scoring: {len(cases)} cases pass")
    for c in cases:
        print("  ", c)
    for line in _check_geometry():
        print(line)
    for line in _check_demo():
        print(line)
    print("OK")
