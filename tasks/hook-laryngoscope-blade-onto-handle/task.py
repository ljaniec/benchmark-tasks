"""On a stainless instrument tray: a hook-on (ISO 7376 style) fibre-optic laryngoscope in two parts. An orange knurled
penlight handle (18.3 mm barrel, painted for visibility) lies in a closed printed cradle on the tray floor (a far rail, an
end stop, a near rail and a front post). Its head carries a 4 mm hinge pin standing upright between two cheeks, a stop face
with the lamp switch plunger in it, and a spring plunger (a 3 mm steel ball, 12 N at rest) in the upper cheek. A stainless
Macintosh 3 class blade (painted blue for visibility, simplified as straight boxes) lies on its side on a yellow carrier
card beside the handle; its heel ends in an open 4.6 mm hook slot with a small 1.5 mm lead-in chamfer.

Task: slide the blade's hook slot onto the hinge pin and swing the blade out 90 deg, then let go of the blade. On the way
the heel rides an ISO-style spring plunger (K 15000 N/m, ball r 1.5 mm) whose compression vs angle is applied as a hinge
spring torque on the pin (crest 80-84 deg, seat 90 deg; 1.1-4.4 N at a blade tip 100 mm from the pin). Past the crest the
torque snaps the heel onto the head's stop face, which presses the lamp switch in (the light at the blade's light outlet
comes on). A revolute on the pin, switched on once the slot is seated, keeps the pin from sliding out. The lamp switch's
spring (0.8 N at rest, 3.8 N pressed in) pushes the blade back; only the seat torque holds it.

Disclosed scoring check (never part of the robot's actions): once the locked state has held for 1 s with the hands
clear, the scorer pushes the blade 100 mm from the pin, horizontally and across the blade toward folded, with 0.8 N for 1 s,
ramped up over the first 0.25 s. 1.0 needs the locked state to last through it. The seat torque holds it (about 1.8 N at
the tip, no friction); with the plunger spring taken out, friction cannot, and the probe folds it back off the stop.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/hook-laryngoscope-blade-onto-handle/task.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<task.json instruction>" --scorer task_score --max-steps 1600 --seed 0
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (  # noqa: E402
    Body,
    BoxShape,
    CylShape,
    SphereShape,
    allow_matrix_grips,
    as_numpy,
    on_reset,
    on_step,
    pivot,
    rotation,
    set_pivot_drive,
    set_pivot_on,
    slider,
    state_from_pose,
)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

allow_matrix_grips()

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.05  # score() counts the 1 s hands-clear hold and the 1 s probe itself; success() fires when both pass
TOP = TABLE_TOP[SCENE]
MM = 0.001
DEBUG = False  # a status line every 10 control steps (stderr)

INSTRUCTION = ("Pick up the blue blade from the yellow card and hook it onto the orange laryngoscope handle until the "
               "light comes on, then let go.")

# - colours ------------------------------------------------------------------------------------------------------------
TRAY_C = (0.80, 0.81, 0.83)
ORANGE = (0.95, 0.42, 0.05)  # orange handle body (painted for visibility)
KNURL = (0.78, 0.30, 0.02)
STEEL = (0.62, 0.64, 0.68)
BLUE = (0.05, 0.25, 0.95)
DARK = (0.12, 0.12, 0.13)
GLOW = (1.00, 0.97, 0.60)
YELLOW = (1.00, 0.78, 0.00)

# - tray (fixed): 260 x 380 mm stainless instrument tray, 20 mm walls -------------------------------------------------
TRAY_XY = (0.250, -0.085)
TRAY_SIZE = (0.260, 0.380)
TRAY_WALL_H = 0.020
TRAY_T = 0.0015
TRAY_FRICTION = (0.25, 0.20)  # stainless on stainless, lightly oiled; the lower of two contacting frictions applies
FLOOR = TOP + TRAY_T  # the tray floor's top face

# - handle: frame origin on the hinge pin's axis at floor level, x along the barrel toward the head, z up ---------------
PIN = (0.300, -0.100)  # the pin's axis, world xy, at the start
BARREL_R = 9.15 * MM  # 18.3 mm OD
BARREL_Y = 16.5 * MM  # barrel axis beside the pin (the pin sits at the head's front corner)
BARREL_X = (-140.0 * MM, -12.0 * MM)
# the stop face the blade heel lands on: 0.4 mm farther from the pin than the heel's side at 90 deg with the pin centred in
# its slot, so the detent pushes the slot's far wall onto the pin before the heel lands (no slot play left to fold it back)
STOP_X = -8.4 * MM
HEAD_X = (STOP_X - 4.0 * MM, STOP_X)  # head block; its +x face is the stop face
HEAD_Y = (BARREL_Y - BARREL_R, BARREL_Y + BARREL_R)
LUG_Y = 4.5 * MM  # a stop lug between the cheeks carries the stop face on toward the pin, clear of the heel's sweep
CHEEK_X, CHEEK_Y = (STOP_X, 6.0 * MM), (-6.0 * MM, 18.0 * MM)
CHEEK_LO = (0.0, 2.0 * MM)  # lower cheek
CHEEK_HI = (17.5 * MM, 20.8 * MM)  # upper cheek: a 15.5 mm gap for the 11 mm heel
PIN_R = 2.0 * MM
PIN_Z = (CHEEK_LO[1], CHEEK_HI[0])
HANDLE_MASS = 0.180  # penlight barrel with two AA cells
LAMP_Y, LAMP_Z = 5.5 * MM, 8.5 * MM  # lamp switch plunger in the stop lug, 5.5 mm from the pin
LAMP_PROUD, LAMP_TRAVEL = 1.5 * MM, 1.9 * MM  # it stands 1.5 mm out of the stop face and goes 1.9 mm in
LAMP_K, LAMP_PRE = 2000.0, 0.4 * MM  # N/m: 0.8 N at rest, 3.8 N pressed in flush with the stop face (design value, see README)
LAMP_ON = 0.6 * MM  # pressed this far in: lamp on (the heel within 0.9 mm of the stop face)

# - blade: frame origin on the pin's axis when hooked (slot bottom on the pin), x along the blade toward its tip (u),
# y across it in the swing plane (t), z up from the floor. The blade lies on its side: its 18 mm web stands up. ------
HEEL_T = 8.0 * MM  # half width of the heel across the swing plane (16 mm heel: the grip)
HEEL_Z = (2.0 * MM, 14.0 * MM)  # the heel rests on the lower cheek (top at 2.0 mm): the detent spring closes inside the handle
HEEL_U = 40.0 * MM  # the heel block runs from the slot bottom to here; the spatula on to the tip
SLOT_HALF = 2.3 * MM  # 4.6 mm slot on the 4.0 mm pin
SLOT_BOTTOM = 2.3 * MM
PRONG_BACK = -12.0 * MM  # the heel's back end, where the slot opens
MOUTH_U, MOUTH_HALF = -9.0 * MM, 3.8 * MM  # the slot's small lead-in chamfer: 1.5 mm on each side over 3 mm
# - detent: a spring plunger (K 15000 N/m, ball r 1.5 mm) whose compression vs swing angle is applied as a hinge spring
# torque on the pin (derived from the plunger geometry: crest 80-84 deg, seat 90 deg). Visual ball only; no contact ball. --
BALL_R = 1.5 * MM
BALL_HX, BALL_HY = -5.5 * MM, 7.8 * MM  # the ball's axis in the handle frame (the pin is at 0, 0)
BALL_Z0 = 14.5 * MM  # ball centre at its rest stop (visual)
DET_K, DET_PRE, DET_TRAVEL = 15000.0, 0.8 * MM, 1.2 * MM  # N/m; 12 N at rest, 27 N on the crest (catalogue M4-M5 plunger)
CREST_DEG, SEAT_DEG = (80.0, 84.0), 90.0
HINGE_ON_DEG, HINGE_OFF_DEG, HINGE_U = 50.0, 45.0, 1.5 * MM  # revolute on once the pin is seated; off to unhook
DET_FOOT_DEG, DET_DAMP_DEG, SEAT_REG_DEG = 70.0, 0.002, 0.35  # chamfer foot; drive damping N m s/deg; regularized seat
_DET_K0 = 15000.0  # the table is built at this K; detent_tau scales with DET_K / _DET_K0
TOP_HI, GROOVE_Z, RAMP_LO = 14.0 * MM, 12.6 * MM, 12.9 * MM  # heel top, the seat's floor, the chamfer's foot
# the -t prong's top, across the blade (|t|, mm): 2.3-3.4 flat 14 | 3.4-5.2 seat floor 12.6 | 5.2-6.6 seat flank 12.6 -> 14
# (45 deg) | 6.6-7.0 crest 14 | 7.0-8.0 chamfer 14 -> 12.9. All edges run along the blade (u), so the ball's reaction is
# across the blade (t): it turns the blade about the pin and never pushes the slot off the pin.
GROOVE_T = (3.4 * MM, 5.2 * MM)  # the seat's floor
FLANK_T = (5.2 * MM, 6.6 * MM)  # the seat's outer flank
RAMP_T = (7.0 * MM, 8.0 * MM)
MOUTH_BAR = 1.0 * MM  # the slot's lead-in chamfer is a 1 mm wall on each prong
SPAT_T, SPAT_Z = 6.0 * MM, (0.0, 18.0 * MM)
BLADE_L = 130.0 * MM
BLADE_MASS = 0.075
OUTLET_U = 95.0 * MM  # the fibre-optic light outlet
BLADE_START = (0.300, -0.175)  # blade origin at the start, lying parallel to the handle (pointing -x)


def _quat_axis(axis, angle) -> tuple:
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    s = math.sin(angle / 2)
    return (float(axis[0] * s), float(axis[1] * s), float(axis[2] * s), math.cos(angle / 2))


RAIL_Y, RAIL_T, RAIL_H = BARREL_R + 2.0 * MM + 2.5 * MM, 5.0 * MM, 9.0 * MM  # rail centre 13.65 mm from the barrel axis
NEAR_X, NEAR_L = -82.0 * MM, 114.0 * MM  # blade-side rail: handle frame x -139 to -25 mm (holds the head against -y yaw)
FRONT_X, FRONT_Y, FRONT_W, FRONT_L, FRONT_H = 8.0 * MM, 19.5 * MM, 3.0 * MM, 9.0 * MM, 12.0 * MM  # front post, handle frame (x 6.5-9.5, y 15-24 mm)


def _box(lo, hi, color, collide=True, yaw=0.0) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, yaw=yaw, collide=collide)


def _tray() -> list:
    cx, cy = TRAY_XY
    sx, sy = TRAY_SIZE
    out = [Body.build("tray_floor", pos=(cx, cy, TOP + TRAY_T / 2), mass=1.0, kinematic=True, color=TRAY_C,
                      shapes=[BoxShape((0.0, 0.0, 0.0), (sx, sy, TRAY_T), TRAY_C)], friction=TRAY_FRICTION, slick=True,
                      contact_offset=0.0005)]
    zc = TOP + TRAY_WALL_H / 2
    out += [Box("tray_wall_n", pos=(cx + sx / 2 - TRAY_T / 2, cy, zc), size=(TRAY_T, sy, TRAY_WALL_H), color=TRAY_C),
            Box("tray_wall_s", pos=(cx - sx / 2 + TRAY_T / 2, cy, zc), size=(TRAY_T, sy, TRAY_WALL_H), color=TRAY_C),
            Box("tray_wall_e", pos=(cx, cy - sy / 2 + TRAY_T / 2, zc), size=(sx, TRAY_T, TRAY_WALL_H), color=TRAY_C),
            Box("tray_wall_w", pos=(cx, cy + sy / 2 - TRAY_T / 2, zc), size=(sx, TRAY_T, TRAY_WALL_H), color=TRAY_C)]
    xc = PIN[0] + (BARREL_X[0] + BARREL_X[1]) / 2
    yb = PIN[1] + BARREL_Y
    # the printed cradle: a low rail along the barrel on the far side and an end stop behind the end cap; the near rail
    # and the front post below close it. The handle can be lifted out
    out.append(Box("cradle_rail", pos=(xc, yb + RAIL_Y, FLOOR + RAIL_H / 2),
                   size=(BARREL_X[1] - BARREL_X[0] - 0.020, RAIL_T, RAIL_H), color=TRAY_C))
    out.append(Box("cradle_stop", pos=(PIN[0] + BARREL_X[0] - 0.006, yb, FLOOR + RAIL_H / 2), size=(RAIL_T, 2 * RAIL_Y, RAIL_H),
                   color=TRAY_C))
    # a short rail on the blade side at the barrel's rear end, clear of the blade's swing: with the far-side rail it stops the
    # handle turning in the cradle
    out.append(Box("cradle_rail_near", pos=(PIN[0] + NEAR_X, yb - RAIL_Y, FLOOR + RAIL_H / 2), size=(NEAR_L, RAIL_T, RAIL_H),
                   color=TRAY_C))
    # a front post that closes the cradle in +x (the handle cannot be pushed forward by the slide-on); it sits clear of the heel's sweep
    out.append(Box("cradle_front", pos=(PIN[0] + FRONT_X, PIN[1] + FRONT_Y, FLOOR + FRONT_H / 2), size=(FRONT_W, FRONT_L, FRONT_H),
                   color=TRAY_C))
    return out


def handle_shapes() -> list:
    bx0, bx1 = BARREL_X
    s = [CylShape(((bx0 + bx1) / 2, BARREL_Y, BARREL_R), BARREL_R, bx1 - bx0, ORANGE, axis="X")]
    for k in range(9):  # knurled bands (paint only)
        x = bx0 + 12 * MM + k * 11 * MM
        s.append(CylShape((x, BARREL_Y, BARREL_R), BARREL_R + 0.1 * MM, 6 * MM, KNURL, collide=False, axis="X"))
    s.append(CylShape((bx0 - 1.5 * MM, BARREL_Y, BARREL_R), 8.0 * MM, 3 * MM, DARK, axis="X"))  # end cap
    s.append(_box((HEAD_X[0], HEAD_Y[0], 0.0), (HEAD_X[1], HEAD_Y[1], 2 * BARREL_R), ORANGE))
    s.append(_box((HEAD_X[1] - 3.0 * MM, LUG_Y, CHEEK_LO[1]), (HEAD_X[1], HEAD_Y[0], CHEEK_HI[0]), ORANGE))  # the stop lug
    s.append(_box((CHEEK_X[0], CHEEK_Y[0], CHEEK_LO[0]), (CHEEK_X[1], CHEEK_Y[1], CHEEK_LO[1]), ORANGE))
    s.append(_box((CHEEK_X[0], CHEEK_Y[0], CHEEK_HI[0]), (CHEEK_X[1], CHEEK_Y[1], CHEEK_HI[1]), ORANGE))
    s.append(CylShape((0.0, 0.0, sum(PIN_Z) / 2), PIN_R, PIN_Z[1] - PIN_Z[0], STEEL))
    s.append(SphereShape((BALL_HX, BALL_HY, BALL_Z0), BALL_R, STEEL, collide=False))  # visual plunger ball; the detent is a hinge torque
    s.append(_box((HEAD_X[0] + 0.5 * MM, HEAD_Y[0] + 4 * MM, 2 * BARREL_R), (HEAD_X[1] - 0.5 * MM, HEAD_Y[1] - 4 * MM,
                  2 * BARREL_R + 0.2 * MM), DARK, collide=False))  # bulb window on the head
    return s


def _slope_bar(ta: float, za: float, tb: float, zb: float) -> BoxShape:
    """A 1 mm bar along the whole prong whose top face is the sloped line from (t ta, z za) to (t tb, z tb) across the blade
    (ta < tb); its underside is buried in the strips below."""
    phi, ln, thick = math.atan2(zb - za, tb - ta), math.hypot(zb - za, tb - ta), 1.0 * MM
    tc, zc = (ta + tb) / 2 + math.sin(phi) * thick / 2, (za + zb) / 2 - math.cos(phi) * thick / 2
    return BoxShape(((PRONG_BACK + SLOT_BOTTOM) / 2, tc, zc), (SLOT_BOTTOM - PRONG_BACK, ln, thick), BLUE,
                    quat_xyzw=(math.sin(phi / 2), 0.0, 0.0, math.cos(phi / 2)))


def _blade_prong(sign: float) -> list:
    """One prong of the heel behind the slot bottom (sign +1: the +t prong, flat top at 14 mm). The -t prong's top carries,
    across the blade, the seat (GROOVE_T floor, FLANK_T flank), the crest and the engage chamfer (RAMP_T), all running along
    the blade."""
    out = []
    z0, z1 = HEEL_Z
    t_in, t_mouth, t_out = sign * SLOT_HALF, sign * MOUTH_HALF, sign * HEEL_T
    # the slot's lead-in chamfer from (u -12, t 3.8) to (u -9, t 2.3): a 1 mm bar on the prong side of it
    p0, p1 = np.array([PRONG_BACK, t_mouth]), np.array([MOUTH_U, t_in])
    d = p1 - p0
    length = float(np.linalg.norm(d))
    n = np.array([-d[1], d[0]]) / length
    if n[1] * sign < 0:
        n = -n
    c = (p0 + p1) / 2 + n * MOUTH_BAR / 2
    out.append(BoxShape((float(c[0]), float(c[1]), (z0 + z1) / 2), (length, MOUTH_BAR, z1 - z0), BLUE,
                        yaw=math.atan2(d[1], d[0])))
    if sign > 0:
        out.append(_box((MOUTH_U, t_in, z0), (SLOT_BOTTOM, t_out, z1), BLUE))
        out.append(_box((PRONG_BACK, t_mouth, z0), (MOUTH_U, t_out, z1), BLUE))
        return out
    for ta, tb, top in ((-GROOVE_T[0], -SLOT_HALF, z1), (-FLANK_T[1], -GROOVE_T[0], GROOVE_Z), (-RAMP_T[0], -FLANK_T[1], z1),
                        (-HEEL_T, -RAMP_T[0], RAMP_LO)):
        out.append(_box((MOUTH_U, ta, z0), (SLOT_BOTTOM, tb, top), BLUE))
        if tb <= -MOUTH_HALF:  # behind the slot's chamfer the strip runs on to the heel's back end
            out.append(_box((PRONG_BACK, ta, z0), (MOUTH_U, tb, top), BLUE))
        elif ta < -MOUTH_HALF:
            out.append(_box((PRONG_BACK, ta, z0), (MOUTH_U, -MOUTH_HALF, top), BLUE))
    out.append(_slope_bar(-FLANK_T[1], z1, -FLANK_T[0], GROOVE_Z))  # the seat's outer flank, falling toward +t
    out.append(_slope_bar(-RAMP_T[1], RAMP_LO, -RAMP_T[0], z1))  # the chamfer, rising toward +t
    return out


def blade_shapes() -> list:
    z0, z1 = HEEL_Z
    s = [_box((SLOT_BOTTOM, -HEEL_T, z0), (HEEL_U, HEEL_T, z1), BLUE)]
    s += _blade_prong(1.0) + _blade_prong(-1.0)
    s.append(_box((HEEL_U, -SPAT_T, SPAT_Z[0]), (BLADE_L, SPAT_T, SPAT_Z[1]), BLUE))
    s.append(_box((14 * MM, -HEEL_T + 0.5 * MM, z1), (34 * MM, HEEL_T - 0.5 * MM, z1 + 0.2 * MM), BLUE, collide=False))
    s.append(_box((OUTLET_U - 4 * MM, -SPAT_T + 0.5 * MM, SPAT_Z[1]), (OUTLET_U + 4 * MM, SPAT_T - 0.5 * MM,
                  SPAT_Z[1] + 0.2 * MM), DARK, collide=False))  # light outlet (dark when off)
    return s


GRIP = dict(friction=(0.8, 0.7), contact_offset=0.0005, solver_iterations=32)
HANDLE_POS = (PIN[0], PIN[1], FLOOR + 0.00005)
HANDLE = Body.build("handle", pos=HANDLE_POS, mass=HANDLE_MASS, shapes=handle_shapes(), color=ORANGE,
                    com=(-0.076, BARREL_Y, BARREL_R), friction=(0.15, 0.12), slick=True, contact_offset=0.0005,
                    solver_iterations=32)
LAMP_POS = (PIN[0] + STOP_X + LAMP_PROUD - 1.0 * MM, PIN[1] + LAMP_Y, FLOOR + 0.00005 + LAMP_Z)
LAMP = Body.build("lamp_switch", pos=LAMP_POS, mass=0.003,
                  shapes=[BoxShape((0.0, 0.0, 0.0), (2.0 * MM, 2.0 * MM, 2.0 * MM), DARK)], color=DARK,
                  friction=(0.2, 0.15), slick=True, contact_offset=0.0004, solver_iterations=32,
                  ignore=("handle",))
BLADE = Body.build("blade", pos=(BLADE_START[0], BLADE_START[1], FLOOR + 0.00005), mass=BLADE_MASS,
                   shapes=blade_shapes(), yaw=math.pi, color=STEEL, com=(0.060, 0.0, 0.009), **GRIP)
HIDDEN = (0.45, 0.40, TOP - 0.20)
CARD = Body.build("blade_card", pos=(BLADE_START[0] - 0.050, BLADE_START[1], FLOOR + 0.0001), mass=0.01,
                  shapes=[BoxShape((0.0, 0.0, 0.0), (0.190, 0.050, 0.0004), YELLOW, collide=False)], kinematic=True,
                  color=YELLOW)  # the blade's printed carrier card: a marking on the tray floor, nothing collides with it
GLOW_BODY = Body.build("lamp_glow", pos=HIDDEN, mass=0.01,
                       shapes=[BoxShape((0.0, 0.0, 0.0), (9 * MM, 7 * MM, 1.0 * MM), GLOW, collide=False)],
                       kinematic=True, color=GLOW)
slider("lamp_travel", "handle", "lamp_switch", anchor=LAMP_POS, axis="X", limits=(-LAMP_TRAVEL, 0.0), stiffness=LAMP_K,
       damping=1.0, target=LAMP_PRE, max_force=30.0)
PIN_MID = sum(PIN_Z) / 2
pivot("blade_pin", "handle", "blade", anchor0=(0.0, 0.0, PIN_MID), anchor1=(0.0, 0.0, PIN_MID), zero_deg=270.0)
OBJECTS = [*_tray(), CARD, HANDLE, LAMP, BLADE, GLOW_BODY]

# - scoring thresholds -------------------------------------------------------------------------------------------------
HOOK_MIN_DEPTH = 2.0 * MM  # the pin at least this far into the slot past its back end
HOOK_T_TOL = 1.0 * MM
HOOK_TILT = math.radians(10.0)
SWUNG_DEG = 60.0
# crossed: while hooked on the pin with the hinge on, the blade turned through the crest (80-84 deg) inside CROSS_DEG;
# counted if a 20 Hz sample is in the crest or if the angle step between samples (opening swing) crosses that band
CROSS_DEG = (60.0, 89.0)
CROSS_EXT = 0.4 * MM  # crest compression is 1.0 mm; this is the minimum that counts as having ridden the ridge
OPEN_DEG = (85.0, 95.0)
FORCED_DEPTH = 1.0 * MM
HANDS_OFF = 0.030  # partial credit: no gripper centre within 30 mm of the blade
CLEAR = 0.050
OPEN_GAP = 0.030
HOLD_STEPS = 20
# still, over the hold before the probe: both parts under 1 cm/s, the handle under 0.5 rad/s, and the blade no more than
# STILL_MOVE from where it was, nor turned more than STILL_TURN, since the locked hold began (pose, not spin: the detent
# contact makes the blade's spin reading chatter at 0.5-0.9 rad/s while it sits at the stop, a motion no camera or judge can
# see). While the probe pushes, the locked state is judged without the stillness terms.
STILL_SPEED, STILL_MOVE, STILL_TURN = 0.01, 1.0 * MM, 1.0
# the probe after the hold (disclosed): PROBE_N at PROBE_AT (blade frame: 100 mm from the pin, heel mid-height), horizontal and
# across the blade toward folded, for PROBE_STEPS control steps, ramped up linearly over the first PROBE_RAMP of them
PROBE_N, PROBE_STEPS, PROBE_RAMP = 0.8, 20, 5
PROBE_U = 0.100
PROBE_AT = (PROBE_U, 0.0, sum(HEEL_Z) / 2)
HEEL_MID = sum(HEEL_Z) / 2
HANDLE_BOX = ((BARREL_X[0] - 3 * MM, CHEEK_Y[0], 0.0), (CHEEK_X[1], HEAD_Y[1], 2 * BARREL_R))
BLADE_BOX = ((PRONG_BACK, -HEEL_T, 0.0), (BLADE_L, HEEL_T, SPAT_Z[1]))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _p(state) -> np.ndarray:
    return np.asarray(state.pos, float)


def pin_depth(q) -> float:
    """How far (m) the pin (centre ``q`` in the blade frame) is pushed into the blade's material, 0 if it is in free
    space (outside the blade, or in the slot)."""
    u, t, z = (float(v) for v in q)
    if abs(z - HEEL_MID) >= (HEEL_Z[1] - HEEL_Z[0] + PIN_Z[1] - PIN_Z[0]) / 2:
        return 0.0
    half = HEEL_T if u <= HEEL_U else SPAT_T
    outside = min(half + PIN_R - abs(t), u - (PRONG_BACK - PIN_R), BLADE_L + PIN_R - u)
    if outside <= 0.0:
        return 0.0
    if u < PRONG_BACK:
        slot = MOUTH_HALF
    elif u < MOUTH_U:  # the chamfered lead-in narrows linearly
        slot = MOUTH_HALF + (SLOT_HALF - MOUTH_HALF) * (u - PRONG_BACK) / (MOUTH_U - PRONG_BACK)
    else:
        slot = SLOT_HALF
    in_slot = max(abs(t) - (slot - PIN_R), u - (SLOT_BOTTOM - PIN_R))
    return max(0.0, min(outside, in_slot))


def measure(objects) -> dict:
    """The laryngoscope as a judge sees it: blade angle, hook and lamp switch."""
    h, b = objects["handle"], objects["blade"]
    rh, rb = rotation(h.quat_xyzw), rotation(b.quat_xyzw)
    pin_w = _p(h) + rh @ np.array([0.0, 0.0, PIN_MID])
    q = rb.T @ (pin_w - _p(b))
    ub = rh.T @ rb[:, 0]
    theta = math.degrees(_wrap(math.atan2(ub[1], ub[0]) - math.pi))
    axes = math.acos(max(-1.0, min(1.0, float(rh[:, 2] @ rb[:, 2]))))
    hooked = (axes <= HOOK_TILT and MOUTH_U + HOOK_MIN_DEPTH <= q[0] <= SLOT_BOTTOM - PIN_R + 0.7 * MM
              and abs(q[1]) <= HOOK_T_TOL and abs(q[2] - HEEL_MID) <= 4 * MM)
    pressed = -float((rh.T @ (_p(objects["lamp_switch"]) - _p(h)))[0] - (LAMP_POS[0] - HANDLE_POS[0]))
    ext = plunger_ext(theta) if hooked else 0.0
    bu, bt = ball_in_blade_frame(theta)
    seated = bool(hooked and abs(theta - SEAT_DEG) <= 4.0)  # in the 90 deg hole, not still on the 80-84 crest
    catch = bool(hooked and theta >= HINGE_ON_DEG and abs(q[0]) <= HINGE_U)
    return {"theta": theta, "pin": q, "hooked": bool(hooked), "axes_deg": math.degrees(axes), "pressed": pressed,
            "lit": pressed >= LAMP_ON, "depth": pin_depth(q), "ball_u": float(bu), "ball_t": float(bt),
            "ball_ext": ext, "seated": seated, "catch": catch}


def _box_dist(point, state, box) -> float:
    r = rotation(state.quat_xyzw)
    local = r.T @ (np.asarray(point, float) - _p(state))
    lo, hi = np.asarray(box[0]), np.asarray(box[1])
    return float(np.linalg.norm(np.maximum(np.maximum(lo - local, local - hi), 0.0)))


def _off_bench(state) -> bool:
    x, y, z = (float(v) for v in state.pos)
    return z < TOP - 0.01 or not (-0.15 < x < 0.85) or abs(y) > 0.60


def _in_tray(state) -> bool:
    cx, cy = TRAY_XY
    sx, sy = TRAY_SIZE
    lo, hi = np.asarray(state.low), np.asarray(state.high)
    return bool(lo[0] >= cx - sx / 2 and hi[0] <= cx + sx / 2 and lo[1] >= cy - sy / 2 and hi[1] <= cy + sy / 2
                and lo[2] >= TOP - 0.002 and hi[2] <= FLOOR + 0.030)


# - episode state ------------------------------------------------------------------------------------------------------
def _crest_crossed_segment(prev: float, cur: float) -> bool:
    """True when an opening swing step crosses the 80-84 deg crest with enough plunger travel somewhere on that overlap."""
    if cur <= prev or prev >= CREST_DEG[1] or cur <= CREST_DEG[0]:
        return False
    seg_lo = max(CREST_DEG[0], prev)
    seg_hi = min(CREST_DEG[1], cur)
    for a in (seg_lo, seg_hi, (seg_lo + seg_hi) / 2):
        if plunger_ext(a) >= CROSS_EXT:
            return True
    return False


def _fresh() -> dict:
    return {"crossed": False, "tick": 0, "done_tick": -1, "arms": None, "violations": set(), "clear": 0, "last": 0.0, "m": None,
            "glow": False, "status": {}, "ext_max": 0.0, "anchor": None, "hinge": False, "hinge_sim": False, "prev_theta": None,
            "probe": {"state": "idle", "step": -1, "runs": 0, "fails": 0, "applied": 0.0, "fail_theta": None}}


_EP = _fresh()


@on_reset
def _new_episode() -> None:
    glow = _EP.get("glow", False)
    _EP.clear()
    _EP.update(_fresh())
    _EP["glow_reset"] = glow
    _EP["probe_reset"] = True  # write a zero probe force on the first step, whatever the reset left


def _arm_reads(env) -> dict:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    out = {}
    for name, key in (("left", "left_robot"), ("right", "right_robot")):
        robot = base.scene[key]
        gi = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][gi]
        tcp = g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET)
        q = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names) if hasattr(robot.data, "joint_names") else list(robot.joint_names)
        gap = -sum(float(q[names.index(j)]) for j in ("joint7", "joint8") if j in names)
        out[name] = {"tcp": tcp, "gap": gap}
    return out


def _place_glow(env, blade) -> None:
    """The lamp: a glowing patch at the blade's light outlet while the switch is pressed, out of sight otherwise. It
    is visual only (nothing collides with it); it is the bulb's light coming out of the blade's fibre bundle."""
    import torch

    base = env.unwrapped
    asset = base.scene["lamp_glow"]
    origin = as_numpy(base.scene.env_origins[0])
    if blade is None:
        p, q = np.array(HIDDEN), np.array([0.0, 0.0, 0.0, 1.0])
    else:
        p = _p(blade) + rotation(blade.quat_xyzw) @ np.array([OUTLET_U, 0.0, SPAT_Z[1] + 0.7 * MM])
        q = np.asarray(blade.quat_xyzw, float)
    pose = torch.tensor([[*(p + origin).tolist(), *q.tolist()]], dtype=torch.float32, device=asset.device)
    if hasattr(asset, "write_root_pose_to_sim"):
        asset.write_root_pose_to_sim(pose)
    else:
        asset.write_root_pose_to_sim_index(root_pose=pose)


def _apply_probe(env, blade, newton: float) -> None:
    """The scorer's probe (never part of the robot's actions): a horizontal force of ``newton`` on the blade at PROBE_AT, across
    the blade toward folded, held on the blade through the simulator's external-wrench buffer until changed (0: none). It is
    given as the same force at the centre of mass plus its moment about it."""
    import torch

    base = env.unwrapped
    asset = base.scene["blade"]
    f, tq = np.zeros(3), np.zeros(3)
    r = rotation(blade.quat_xyzw)
    if newton > 0.0:
        across = np.array([r[0, 1], r[1, 1], 0.0])
        f = -newton * across / np.linalg.norm(across)
        com = _p(blade) + r @ np.asarray(BLADE.com)
        if getattr(asset.data, "root_com_pos_w", None) is not None:
            com = as_numpy(asset.data.root_com_pos_w)[0] - as_numpy(base.scene.env_origins[0])
        tq = np.cross(_p(blade) + r @ np.asarray(PROBE_AT) - com, f)

    def tensor(v):
        return torch.tensor(np.asarray(v, float)[None, None, :], dtype=torch.float32, device=asset.device)

    try:
        asset.set_external_force_and_torque(tensor(f), tensor(tq), body_ids=[0], is_global=True)
    except TypeError:  # an Isaac Lab without is_global applies the wrench in the body frame
        asset.set_external_force_and_torque(tensor(r.T @ f), tensor(r.T @ tq), body_ids=[0])


def probe_newton() -> float:
    """The probe force (N) for the coming control step: from the step the full state has held HOLD_STEPS on, PROBE_STEPS steps
    ramped up over the first PROBE_RAMP; otherwise 0."""
    k = _EP["clear"] - HOLD_STEPS
    if _EP["violations"] or not 0 <= k < PROBE_STEPS:
        return 0.0
    return PROBE_N * min(1.0, (k + 1) / PROBE_RAMP)


@on_step
def _sim_step(env) -> None:
    from yam_arena.scene import object_states

    _EP["arms"] = _arm_reads(env)
    st = object_states(env, ["handle", "blade", "lamp_switch"])
    _EP["tick"] += 1
    _advance(st)  # this step's score, now (score() and success() return it), so the probe below follows it
    m = _EP["m"]
    want = bool(_EP["hinge"])
    if want != _EP["hinge_sim"]:
        set_pivot_on("blade_pin", want)
        _EP["hinge_sim"] = want
    if want:
        tgt, k, dmp, tmax = detent_drive(m["theta"])
        set_pivot_drive("blade_pin", tgt, k, dmp, tmax)
    else:
        set_pivot_drive("blade_pin", 0.0, 0.0, 0.0, 0.0)
    if m["lit"]:
        _place_glow(env, st["blade"])
        _EP["glow"] = True
    elif _EP["glow"] or _EP.pop("glow_reset", False):
        _place_glow(env, None)
        _EP["glow"] = False
    pr, n = _EP["probe"], probe_newton()
    if n > 0.0 or pr["applied"] > 0.0 or _EP.pop("probe_reset", False):
        _apply_probe(env, st["blade"], n)
        pr["applied"] = n
    if DEBUG and (_EP["tick"] % 10 == 0 or _EP["tick"] <= 5 or n > 0.0):
        a = _EP["arms"]
        print(f"[laryngo] t={_EP['tick'] / 20:5.1f}s theta={m['theta']:6.1f} pin_ut={np.round(m['pin'][:2] * 1000, 2).tolist()} "
              f"hooked={m['hooked']} axes={m['axes_deg']:4.1f} "
              f"ball_ext={m['ball_ext'] * 1000:5.2f} ext_max={_EP['ext_max'] * 1000:4.2f} "
              f"hinge={_EP['hinge']} crossed={_EP['crossed']} "
              f"pressed={m['pressed'] * 1000:5.2f} seated={m['seated']} lit={m['lit']} depth={m['depth'] * 1000:4.2f} "
              f"clear={_EP['clear']} probe={pr['state']}:{pr['step']} n={n:4.2f} runs={pr['runs']} fails={pr['fails']} "
              f"handle={np.round(_p(st['handle']) * 1000, 1).tolist()} blade={np.round(_p(st['blade']) * 1000, 1).tolist()} "
              f"L={np.round(a['left']['tcp'] * 1000).tolist()} Lgap={a['left']['gap'] * 1000:4.1f} "
              f"R={np.round(a['right']['tcp'] * 1000).tolist()} Rgap={a['right']['gap'] * 1000:4.1f} "
              f"vel_b={np.linalg.norm(st['blade'].lin_vel) * 1000:.1f}/{np.linalg.norm(st['blade'].ang_vel):.2f} "
              f"vel_h={np.linalg.norm(st['handle'].lin_vel) * 1000:.1f}/{np.linalg.norm(st['handle'].ang_vel):.2f} "
              f"status={_EP['status']} score={_EP['last']} viol={sorted(_EP['violations'])}", file=sys.stderr, flush=True)


# - scoring ------------------------------------------------------------------------------------------------------------
def _advance(objects) -> float:
    if _EP["done_tick"] == _EP["tick"]:
        return _EP["last"]
    _EP["done_tick"] = _EP["tick"]
    h, b = objects["handle"], objects["blade"]
    m = measure(objects)
    if _off_bench(h) or _off_bench(b):
        _EP["violations"].add("handle or blade off the bench")
    if m["depth"] > FORCED_DEPTH:
        _EP["violations"].add("blade forced onto the pin other than through the slot")
    if m["catch"]:
        _EP["hinge"] = True
    if (not m["hooked"]) or m["theta"] < HINGE_OFF_DEG:
        _EP["hinge"] = False
    prev_theta = _EP.get("prev_theta")
    hooked_swing = m["hooked"] and _EP["hinge"]
    swinging = hooked_swing and CROSS_DEG[0] <= m["theta"] <= CROSS_DEG[1]
    if hooked_swing and (
            swinging or (prev_theta is not None and CROSS_DEG[0] <= prev_theta <= CROSS_DEG[1])
            or (prev_theta is not None and prev_theta < m["theta"] and prev_theta < CREST_DEG[1]
                and m["theta"] > CREST_DEG[0])):
        _EP["ext_max"] = max(_EP["ext_max"], m["ball_ext"])
    crest = hooked_swing and (
        (CREST_DEG[0] <= m["theta"] <= CREST_DEG[1] and m["ball_ext"] >= CROSS_EXT)
        or (prev_theta is not None and _crest_crossed_segment(prev_theta, m["theta"])))
    if crest:
        _EP["crossed"] = True  # the plunger rode the crest (80-84 deg) while the blade was being swung on the pin
    if m["pin"][0] < PRONG_BACK - PIN_R:
        _EP["crossed"] = False  # the slot came off the pin: the next hook-on must swing over the crest again
    arms = _EP["arms"]
    hands_off = arms is None or all(_box_dist(a["tcp"], b, BLADE_BOX) >= HANDS_OFF for a in arms.values())
    clear = arms is None or all(a["gap"] >= OPEN_GAP and _box_dist(a["tcp"], b, BLADE_BOX) >= CLEAR
                                and _box_dist(a["tcp"], h, HANDLE_BOX) >= CLEAR for a in arms.values())
    handle_down = float(h.low[2]) <= FLOOR + 0.010  # the handle still lies on the tray floor, not lifted in a hand
    r25 = m["hooked"] and hands_off and handle_down
    r50 = r25 and m["theta"] >= SWUNG_DEG
    full = (r50 and OPEN_DEG[0] <= m["theta"] <= OPEN_DEG[1] and m["lit"] and m["seated"] and _EP["crossed"] and clear
            and _in_tray(h) and _in_tray(b))
    if full and _EP["clear"] < HOLD_STEPS:  # stillness is judged over the hold before the probe, not while it pushes
        full = h.resting() and float(np.linalg.norm(b.lin_vel)) < STILL_SPEED
        if full and _EP["clear"] > 0:  # the blade within STILL_MOVE and STILL_TURN of where the locked hold began
            a_pos, a_theta = _EP["anchor"]
            full = float(np.linalg.norm(_p(b) - a_pos)) <= STILL_MOVE and abs(m["theta"] - a_theta) <= STILL_TURN
    _EP["clear"] = _EP["clear"] + 1 if full else 0
    if _EP["clear"] == 1:
        _EP["anchor"] = (_p(b).copy(), m["theta"])
    pr, k = _EP["probe"], _EP["clear"] - HOLD_STEPS
    if k >= PROBE_STEPS:
        pr["state"], pr["step"] = "passed", PROBE_STEPS
    elif k >= 0:
        if pr["state"] != "loading":
            pr["runs"] += 1
        pr["state"], pr["step"] = "loading", k
    elif pr["state"] == "loading":  # the locked state broke under the probe
        pr["state"], pr["fails"], pr["fail_theta"] = "failed", pr["fails"] + 1, m["theta"]
    elif pr["state"] == "passed":
        pr["state"] = "idle"
    if _EP["violations"]:
        out = 0.0
    elif _EP["clear"] >= HOLD_STEPS + PROBE_STEPS:
        out = 1.0
    else:
        out = 0.5 if r50 else 0.25 if r25 else 0.0
    _EP["m"] = m
    _EP["prev_theta"] = m["theta"] if hooked_swing else None
    _EP["status"] = {"hands_off": hands_off, "clear": clear, "in_tray": (_in_tray(h), _in_tray(b)), "hinge": _EP["hinge"]}
    _EP["last"] = out
    return out


def score(objects) -> float:
    """1.0: the locked state for 2 s (40 control steps) on end, through the probe. Locked: the blade hooked on the pin, swung
    out to 85-95 deg from folded, the lamp switch pressed 0.6 mm or more (light on), seated past the crest (angle above 84 deg
    in the 90 deg seat),     the plunger seen on the crest (80-84 deg, compression 0.4 mm or more) while the blade turned through
    60-89 deg since it was last hooked on with the pin hinge engaged (counts if a control step lands in the crest or the angle
    change between steps crosses 80-84 deg on the opening swing), the handle and the blade inside the tray outline (each
    part's top no higher than 30 mm above the tray floor) and, over the first 20 locked steps (before the probe), still (both
    under 1 cm/s, the handle under 0.5 rad/s, the blade within 1.0 mm and 1 deg of where it was when the locked hold began),
    both grippers open (30 mm) and their centres 50 mm clear of both parts. The probe: from the 20th locked step on, 0.8 N at
    the blade 100 mm from the pin, horizontal and across the blade toward folded, for 20 steps (ramped up over the first 5);
    it starts again with the next locked hold if the locked state breaks. 0.5: hooked (pin 2 mm or more past the end of the
    lead-in, within 1 mm of the slot centreline, handle still on the tray floor), swung out 60 deg or more, no gripper centre
    within 30 mm of the blade. 0.25: hooked as above, no gripper centre within 30 mm of the blade. Else 0, and 0 for the rest
    of the episode once a part leaves the bench or the pin is pushed more than 1 mm into the blade steel instead of entering
    the slot."""
    return _advance(objects)


def success(objects) -> bool:
    return _advance(objects) >= 1.0


# - scripted demo (open loop: fixed TCP waypoints per arm, no observations, no sim state) ----------------------------
# Right hand only: takes the blade by its flange, turns it 40 deg out from the handle and sets it on the floor with the
# slot 45 mm behind the pin, lifts it 2 mm, slides it onto the pin, sets it down, then swings it out about the pin to the
# stop (the heel lands on the stop face and presses the lamp switch), lets go and backs
# away. The left hand only parks out of the way (the cradle holds the handle).
THETA0 = 40.0  # deg: the blade's angle from folded while it is slid onto the pin
HOOK_OUT = 0.045  # m: where the slot bottom starts, behind the pin along the blade
U_GRIP = 0.065  # m: the right pads' centre along the blade from the pin
Z_GRIP = FLOOR + 0.017  # the right TCP height on the heel: pad tips 4 mm above the floor
F_HEEL = -0.004  # finger target: an 8 mm gap on the 12 mm blade flange
L_GRIP_X = PIN[0] - 0.131  # the left pads on the barrel, 9 mm in from its end cap: 130+ mm from the right hand (arms block each other closer)
Z_LEFT = FLOOR + 0.016  # pad tips 3 mm above the floor
RIGHT_DX, RIGHT_DY = 0.0, 0.0
LIFT = 0.002  # the right hand carries the blade this much above the floor while it slides on
THETA_END = 92.0  # the arc is driven 2 deg past the stop (arm compliance presses the heel on it)
SWING_STEP, SWING_S = 4.0, 0.35
LEAD_S, LEAD_STEPS = 3.0, 15


def blade_dirs(theta_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """(u, t): the blade's long axis and its across-the-swing axis in the world, at ``theta_deg`` from folded (the
    handle as it starts)."""
    a = math.radians(theta_deg)
    return np.array([-math.cos(a), -math.sin(a), 0.0]), np.array([math.sin(a), -math.cos(a), 0.0])


def _pin3() -> np.ndarray:
    return np.array([PIN[0], PIN[1], 0.0])


def _grip_tcp(theta_deg: float, origin_xy) -> np.ndarray:
    u, _ = blade_dirs(theta_deg)
    p = np.array([origin_xy[0], origin_xy[1], 0.0]) + U_GRIP * u
    return np.array([p[0], p[1], Z_GRIP])


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.5), (0.8, 1.4), (0.5, 1.2), (-1.0, 0.0, 1.0), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=80)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, label: str) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), FINGER_OPEN, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(float(t), tuple(float(v) for v in pos), float(finger), label, np.asarray(rot, float))


def _duration(wps) -> float:
    return sum(w.duration for w in wps)


def _rot(theta_deg: float) -> np.ndarray:
    return grip_rotation(math.radians(theta_deg))  # fingers close across the blade


UP = np.array([0.0, 0.0, 1.0])


def _right() -> list[Waypoint]:
    g0 = _grip_tcp(0.0, BLADE_START)
    u0, _ = blade_dirs(THETA0)
    pre = _pin3()[:2] + HOOK_OUT * u0[:2]
    g1 = _grip_tcp(THETA0, pre) + np.array([RIGHT_DX, RIGHT_DY, 0.0])
    g2 = _grip_tcp(THETA0, _pin3()[:2] - 0.001 * u0[:2]) + np.array([RIGHT_DX, RIGHT_DY, 0.0])
    lift = LIFT * UP  # the blade is carried this far above the floor while it slides onto the pin
    out = _lead_in("right", g0 + 0.05 * UP, _rot(0.0), "over the blade's heel")
    out += [_w(1.2, g0, FINGER_OPEN, "down round the heel", _rot(0.0)),
            _w(0.6, g0, F_HEEL, "take the blade by the heel", _rot(0.0)),
            _w(0.3, g0, F_HEEL, "settle", _rot(0.0)),
            _w(1.0, g0 + 0.030 * UP, F_HEEL, "lift it", _rot(0.0)),
            _w(2.0, g1 + 0.030 * UP, F_HEEL, "turn it 40 deg out, slot behind the pin", _rot(THETA0)),
            _w(1.2, g1 - 0.0005 * UP, F_HEEL, "set it on the floor", _rot(THETA0)),
            _w(0.4, g1 + lift, F_HEEL, "lift it a little off the floor", _rot(THETA0)),
            _w(1.6, g2 + lift, F_HEEL, "slide the slot onto the pin", _rot(THETA0)),
            _w(0.5, g2 + lift, F_HEEL, "seated", _rot(THETA0)),
            _w(0.5, g2 - 0.0005 * UP, F_HEEL, "set it down on the floor", _rot(THETA0))]
    th = THETA0
    while th < THETA_END - 1e-9:
        th = min(THETA_END, th + SWING_STEP)
        g = _grip_tcp(th, _pin3()[:2] - 0.0005 * blade_dirs(th)[0][:2])
        out.append(_w(SWING_S, g - 0.0005 * UP, F_HEEL, "swing it out about the pin", _rot(th)))
    out.append(_w(0.6, out[-1].pos, F_HEEL, "press it on the stop", _rot(THETA_END)))
    g90 = _grip_tcp(90.0, _pin3()[:2])
    out += [_w(0.5, g90, F_HEEL, "ease back off the stop", _rot(90.0)),
            _w(0.5, g90, FINGER_OPEN, "let go", _rot(90.0)),
            _w(0.8, g90 + 0.060 * UP, FINGER_OPEN, "back up", _rot(90.0)),
            _w(1.5, (0.150, -0.300, TOP + 0.200), FINGER_OPEN, "out of the way", _rot(90.0))]
    return out


def _left(hold_until: float) -> list[Waypoint]:
    """The left hand stays out of the way (the right hand does the job; the cradle holds the handle)."""
    rot = grip_rotation(0.0)
    park = np.array([0.150, 0.300, TOP + 0.200])  # far left, away from the tray: the lamp stays visible in the cameras
    out = _lead_in("left", park, rot, "wait clear of the work")
    out.append(_w(max(0.5, hold_until - _duration(out)), park, FINGER_OPEN, "wait", rot))
    return out


END_WAIT = 3.0  # s: both hands wait clear at the end while the scorer's hold and probe run


def demo():
    right = _right()
    left = _left(_duration(right) + END_WAIT)
    right.append(_w(max(0.5, _duration(left) - _duration(right)), right[-1].pos, FINGER_OPEN, "wait", right[-1].yaw))
    return {"left": left, "right": right}


# - offline self-test: python task.py (needs only NumPy and yam_arena) -------------------------------------------------
def _yaw_q(yaw: float) -> tuple:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


_MOUTH_BAR_T = MOUTH_BAR * math.hypot(MOUTH_U - PRONG_BACK, MOUTH_HALF - SLOT_HALF) / (MOUTH_U - PRONG_BACK)  # its width across


def heel_top(u, t):
    """Height (m) of the heel's top face at the blade-frame points (u, t) (floats or arrays), NaN where there is no steel under
    it (outside the heel, in the slot, or in its lead-in chamfer between the 1 mm walls and the prongs' strips)."""
    u, t = np.broadcast_arrays(np.asarray(u, float), np.asarray(t, float))
    at = np.abs(t)
    flank = GROOVE_Z + (TOP_HI - GROOVE_Z) * (at - FLANK_T[0]) / (FLANK_T[1] - FLANK_T[0])
    ramp = RAMP_LO + (TOP_HI - RAMP_LO) * (RAMP_T[1] - at) / (RAMP_T[1] - RAMP_T[0])
    z = np.where(t > 0, TOP_HI, np.select([at <= GROOVE_T[0], at <= FLANK_T[0], at <= FLANK_T[1], at <= RAMP_T[0]],
                                         [TOP_HI, GROOVE_Z, flank, TOP_HI], ramp))
    behind = u < MOUTH_U
    line = np.where(behind, MOUTH_HALF + (SLOT_HALF - MOUTH_HALF) * (u - PRONG_BACK) / (MOUTH_U - PRONG_BACK), SLOT_HALF)
    bar = behind & (at >= line) & (at <= line + _MOUTH_BAR_T)
    z = np.where(bar, TOP_HI, z)
    z = np.where((at < line) | (behind & ~bar & (at < MOUTH_HALF)), np.nan, z)
    z = np.where(u >= SLOT_BOTTOM, TOP_HI, z)
    return np.where((u > HEEL_U) | (u < PRONG_BACK) | (at > HEEL_T), np.nan, z)


def ball_in_blade_frame(theta_deg: float, pin=(0.0, 0.0)) -> tuple:
    """Where the detent ball's axis is on the blade (u, t) when the blade stands at ``theta_deg`` from folded with the pin at
    ``pin`` (u, t) on the blade (default: on the blade's origin)."""
    a = math.radians(theta_deg)
    return (pin[0] - BALL_HX * math.cos(a) - BALL_HY * math.sin(a), pin[1] + BALL_HX * math.sin(a) - BALL_HY * math.cos(a))


def _build_detent_table():
    """Circumferential plunger track: a 1.5 mm ball on a crest plateau 80-84 deg and a hole seat at 90 deg, K 15000 N/m.
    Returns angle (deg), plunger travel (m), hinge torque on the blade (N m, positive opens toward 90)."""
    r_track = math.hypot(BALL_HX, BALL_HY)
    ext_crest = 1.0 * MM
    h_rest = BALL_R - ext_crest

    def s_of(th):
        return r_track * math.radians(th - SEAT_DEG)

    a_hole = abs(s_of(CREST_DEG[1]))
    z_foot, z_floor = -2.0 * MM, -3.0 * MM
    pts = []

    def add_line(s0, z0, s1, z1, n):
        for i in range(n + 1):
            t = i / n
            pts.append((s0 + (s1 - s0) * t, z0 + (z1 - z0) * t))

    add_line(s_of(40.0), z_foot, s_of(DET_FOOT_DEG), z_foot, 80)
    add_line(s_of(DET_FOOT_DEG), z_foot, s_of(CREST_DEG[0]), 0.0, 400)
    add_line(s_of(CREST_DEG[0]), 0.0, -a_hole, 0.0, 200)
    add_line(-a_hole, 0.0, -a_hole, z_floor, 80)
    add_line(-a_hole, z_floor, a_hole, z_floor, 80)
    add_line(a_hole, z_floor, a_hole, 0.0, 80)
    add_line(a_hole, 0.0, s_of(96.0), 0.0, 200)
    add_line(s_of(96.0), 0.0, s_of(110.0), 0.0, 80)
    ss, zz = np.array([p[0] for p in pts]), np.array([p[1] for p in pts])
    th = np.linspace(40.0, 110.0, 1401)
    ext = np.empty_like(th)
    for i, a in enumerate(th):
        s = s_of(float(a))
        d2 = (s - ss) ** 2
        msk = d2 < BALL_R ** 2
        h = float(np.max(zz[msk] + np.sqrt(np.maximum(BALL_R ** 2 - d2[msk], 0.0)))) if np.any(msk) else h_rest
        ext[i] = max(0.0, h - h_rest)
    force = _DET_K0 * (DET_PRE + ext)
    slope = np.gradient(ext, np.radians(th))
    tau = -force * slope  # Q_theta = -dU/dtheta, U = 0.5 K (pre+ext)^2
    return th, ext, tau


_DET_TH, _DET_EXT, _DET_TAU = _build_detent_table()


def plunger_ext(theta_deg: float, pin=(0.0, 0.0)) -> float:
    """Plunger travel (m) from the rest stop at ``theta_deg`` (the pin argument is kept for call-site compatibility)."""
    return float(np.interp(theta_deg, _DET_TH, _DET_EXT))


def detent_tau(theta_deg: float) -> float:
    """Hinge torque on the blade (N m, positive opens) from the plunger geometry, scaled with DET_K."""
    return float(np.interp(theta_deg, _DET_TH, _DET_TAU)) * (DET_K / _DET_K0)


def detent_drive(theta_deg: float, k: float | None = None) -> tuple:
    """PhysX angular drive (target deg from the 90 deg zero, stiffness N m/deg, damping N m s/deg, torque cap N m)
    that matches detent_tau at ``theta_deg``."""
    scale = (DET_K if k is None else k) / _DET_K0
    tau, phi = float(np.interp(theta_deg, _DET_TH, _DET_TAU)) * scale, theta_deg - SEAT_DEG
    if CREST_DEG[1] < theta_deg < 96.0:  # seat well, including the cusp at 90 where tau is 0
        tau_edge = abs(float(np.interp(SEAT_DEG + SEAT_REG_DEG, _DET_TH, _DET_TAU)) * scale)
        if abs(phi) < SEAT_REG_DEG:
            return 0.0, tau_edge / SEAT_REG_DEG, DET_DAMP_DEG, tau_edge
        return 0.0, abs(tau) / max(abs(phi), 0.05), DET_DAMP_DEG, max(abs(tau), tau_edge)
    if CREST_DEG[0] - 0.05 <= theta_deg <= CREST_DEG[1] + 0.05 or abs(tau) < 1e-6:
        return 0.0, 0.0, DET_DAMP_DEG, 0.0
    if DET_FOOT_DEG < theta_deg < CREST_DEG[0]:
        return DET_FOOT_DEG - SEAT_DEG, abs(tau) / max(theta_deg - DET_FOOT_DEG, 0.05), DET_DAMP_DEG, abs(tau)
    return 0.0, 0.0, DET_DAMP_DEG, 0.0


def swing_table(step: float = 0.5) -> list:
    """The detent along the swing: plunger travel, spring force, hinge torque and the same as a force at a blade tip 100 mm
    from the pin. tip_n positive opens (snap); negative is engage resistance."""
    rows = []
    for a in np.arange(50.0, 100.0 + 1e-9, step):
        ext = plunger_ext(float(a))
        tau = detent_tau(float(a))
        rows.append({"theta": float(a), "ext": ext, "force": DET_K * (DET_PRE + ext), "tau_nm": tau,
                     "tip_n": tau / PROBE_U, "tip_forward_n": -tau / PROBE_U})
    return rows


# - hold at 90 deg: a planar quasi-static model of the hooked blade with the slot's play ---------------------------------------
PLAY = SLOT_HALF - PIN_R  # the pin's clearance to each slot wall (0.3 mm)
PIN_HOME = SLOT_BOTTOM - PIN_R  # the pin's u on the blade when it sits in the slot bottom
BALL_SIDE_K = 7.0e4  # N/m: the plunger ball's sideways give in its bore (measured in the simulator: about 0.25 mm at 16 N)
SIM_PRESS_SHORT = 0.35 * MM  # the simulator pressed the lamp this much less than the model at 90 deg (measured)
_MU = (0.12, HANDLE.friction[0], TRAY_FRICTION[0])  # plunger-on-heel class, heel on lower cheek, blade on tray floor
_FACE_U = np.linspace(PRONG_BACK, SLOT_BOTTOM, 286)
_LAMP_YS = np.linspace(LAMP_Y - 1.0 * MM, LAMP_Y + 1.0 * MM, 9)


def hold_pose(theta: float, pu: float, pt: float, side: float = 0.0, probe: float = 0.0, k: float = DET_K):
    """The blade at ``theta`` deg with the pin at (pu, pt) on the blade and the detent ball pushed ``side`` m across the blade:
    (energy, state). Energy: the detent and lamp springs, the ball's side give and the probe's work (``probe`` N toward folded at
    the probe point). None where the heel would cut into the stop face or a spring would bottom out."""
    a = math.radians(theta)
    u, t = np.array([-math.cos(a), -math.sin(a)]), np.array([math.sin(a), -math.cos(a)])
    o = -(pu * u + pt * t)
    bu, bt = ball_in_blade_frame(theta, (pu, pt))
    ext = plunger_ext(theta)
    face = o + _FACE_U[:, None] * u - HEEL_T * t
    on = (face[:, 1] >= LUG_Y) & (face[:, 1] <= HEAD_Y[1])
    if on.any() and float(np.max(STOP_X - face[on, 0])) > 1e-7:
        return None
    along = (_LAMP_YS - o[1] + HEEL_T * t[1]) / u[1]
    x = (o[0] + along * u[0] - HEEL_T * t[0])[along >= PRONG_BACK]
    press = max(0.0, float(np.max(STOP_X + LAMP_PROUD - x))) if len(x) else 0.0
    if press > LAMP_TRAVEL:
        return None
    energy = (0.5 * k * (DET_PRE + ext) ** 2 + 0.5 * LAMP_K * (LAMP_PRE + press) ** 2 + 0.5 * BALL_SIDE_K * side ** 2
              + probe * float(o[0] + PROBE_U * u[0]))
    return energy, {"theta": theta, "pu": pu, "pt": pt, "side": side, "ext": ext, "press": press, "ball": np.array([bu, bt]),
                    "force": k * (DET_PRE + ext) if ext > 1e-9 else 0.0, "com": o + BLADE.com[0] * u}


def stop_angle(pt: float = PLAY) -> float:
    """The angle (deg) at which the heel lands on the stop face, the pin in the slot bottom ``pt`` across the slot."""
    lo, hi = 85.0, 95.0
    while hi - lo > 1e-4:
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if hold_pose(mid, PIN_HOME, pt) is not None else (lo, mid)
    return lo


BLADE_I = 3.9e-4  # kg m^2 about the pin (75 g blade, com 60 mm out)


def lamp_tau(th: float, h: float = 0.1) -> float:
    """Lamp-switch torque on the blade (N m, positive opens) at ``th`` deg."""
    stop = stop_angle(0.0)

    def U(a):
        r = hold_pose(min(a, stop - 0.05), PIN_HOME, 0.0)
        p = 0.0 if r is None else r[1]["press"]
        return 0.5 * LAMP_K * (LAMP_PRE + p) ** 2

    return -(U(th + h) - U(th - h)) / math.radians(2 * h)


def hinge_sim(theta0: float, probe: float = 0.0, k: float | None = None, t_end: float = 1.5,
              friction: bool = True) -> tuple:
    """1-DOF hinge at 240 Hz with the 20 Hz drive linearization used in the sim. Returns (theta_deg, lamp_press_m)."""
    k = DET_K if k is None else k
    stop = stop_angle(0.0)
    dt = 1.0 / 240.0
    th, w, press = float(theta0), 0.0, 0.0
    mu_n, k_stop = _MU[2] * BLADE_MASS * 9.81 * BLADE.com[0], 8.0
    tgt = kd = dmp = tmax = 0.0
    for i in range(int(t_end / dt)):
        if i % 12 == 0:
            tgt, kd, dmp, tmax = detent_drive(th, k=k)
        phi = th - SEAT_DEG
        tau_d = max(-tmax, min(tmax, kd * (tgt - phi) - dmp * math.degrees(w)))
        r = hold_pose(th, PIN_HOME, 0.0)
        if r is None:
            press, tau_stop = LAMP_TRAVEL, -k_stop * math.radians(th - stop) - 0.05 * w
        else:
            press, tau_stop = r[1]["press"], 0.0
        tau_f = -math.copysign(mu_n, w) if friction and abs(w) > 1e-4 else 0.0
        w += (tau_d + lamp_tau(min(th, stop - 0.05)) - probe * PROBE_U + tau_f + tau_stop) / BLADE_I * dt
        th += math.degrees(w) * dt
        th = min(100.0, max(50.0, th))
    return th, press


def hold_report() -> list:
    """1-DOF hinge: snap to the 90 deg seat, hold the disclosed 0.8 N probe, friction-alone (DET_K ~ 0) folds."""
    stop = stop_angle(0.0)
    th_snap, pr_snap = hinge_sim(86.0, probe=0.0)
    th_hold, pr_hold = hinge_sim(90.0, probe=PROBE_N, friction=False)
    th_fric, pr_fric = hinge_sim(90.0, probe=PROBE_N, k=1e-3)
    out = [f"hold: stop face reached at {stop:.2f} deg; 1-DOF snap from 86 deg -> {th_snap:.2f} deg, lamp {pr_snap / MM:.2f} mm",
           f"hold: probe {PROBE_N:.2f} N (frictionless) -> {th_hold:.2f} deg, lamp {pr_hold / MM:.2f} mm",
           f"hold: friction alone (DET_K 1e-3) under the probe -> {th_fric:.2f} deg, lamp {pr_fric / MM:.2f} mm"
           + (": folded" if th_fric < OPEN_DEG[0] or pr_fric < LAMP_ON else "")]
    assert 88.0 <= th_snap <= 92.0 and pr_snap >= LAMP_ON, (th_snap, pr_snap)
    assert OPEN_DEG[0] <= th_hold <= OPEN_DEG[1] and pr_hold >= LAMP_ON, (th_hold, pr_hold)
    assert th_fric < 80.0 or pr_fric < LAMP_ON, (th_fric, pr_fric)
    return out


def swing_report() -> list:
    """Tip-force vs angle from the plunger geometry: engage and snap inside 1.1-4.4 N; seat at 90; friction cannot hold."""
    rows = swing_table(0.05)
    eng = [r for r in rows if 55.0 <= r["theta"] <= CREST_DEG[0]]
    peak = max(eng, key=lambda r: r["tip_forward_n"])
    snap = max((r for r in rows if CREST_DEG[1] <= r["theta"] <= SEAT_DEG), key=lambda r: r["tip_n"])
    crest = max(r["ext"] for r in rows if CREST_DEG[0] <= r["theta"] <= CREST_DEG[1])
    seat = plunger_ext(SEAT_DEG)
    breakaway = snap["tau_nm"]
    lamp_nm = -lamp_tau(stop_angle(0.0) - 0.1)
    probe_nm = PROBE_N * PROBE_U
    alone_nm = _MU[2] * BLADE_MASS * 9.81 * BLADE.com[0]
    out = [f"detent: crest {CREST_DEG[0]:.0f}-{CREST_DEG[1]:.0f} deg, seat {SEAT_DEG:.0f} deg, ball r {BALL_R / MM:.1f} mm, "
           f"K {DET_K:.0f} N/m; rides {crest / MM:.2f} mm up, {seat / MM:.2f} mm in at the seat",
           f"detent: engage tip force {peak['tip_forward_n']:.2f} N at {peak['theta']:.1f} deg, snap {snap['tip_n']:.2f} N at "
           f"{snap['theta']:.2f} deg (instructions for use: 1.1-4.4 N)",
           f"detent: breakaway {breakaway * 1000:.0f} mN m against lamp {lamp_nm * 1000:.0f} + probe {probe_nm * 1000:.0f} mN m; "
           f"friction alone {alone_nm * 1000:.0f} mN m"]
    assert 1.1 <= peak["tip_forward_n"] <= 4.4, peak
    assert 1.1 <= snap["tip_n"] <= 4.4, snap
    assert crest >= CROSS_EXT and 0.0 < seat < crest, (crest, seat)
    assert breakaway > 1.3 * (lamp_nm + probe_nm), (breakaway, lamp_nm, probe_nm)
    assert alone_nm < 0.5 * (lamp_nm + probe_nm), (alone_nm, lamp_nm)
    return out


def pin_retention_report() -> list:
    """The pin hinge is a revolute: no translational play, so 5 N along the slot cannot slide the pin out."""
    mouth = abs(PRONG_BACK)
    out = [f"pin retention: revolute on the pin (no translational play); 5 N along the slot produces 0 slide "
           f"(mouth {mouth / MM:.0f} mm behind the pin, unhook only below {HINGE_OFF_DEG:.0f} deg)"]
    assert mouth >= 10.0 * MM
    return out


def _states(theta=0.0, origin=None, pressed=0.0, handle_dxy=(0.0, 0.0), lift=0.0, vel=0.0, lift_handle=0.0, ball_ext=None) -> dict:
    """Offline states: the handle where it starts (shifted ``handle_dxy``), the blade at ``theta`` deg from folded with its origin
    at ``origin`` (world xy; default: hooked, slot bottom on the pin), the lamp switch ``pressed`` m in, the plunger pushed in as
    the heel top pushes it at that angle (or ``ball_ext``)."""
    d = np.array([handle_dxy[0], handle_dxy[1], lift_handle])
    hp = np.asarray(HANDLE_POS) + d
    o = (np.array([PIN[0], PIN[1]]) + d[:2]) if origin is None else np.asarray(origin, float)
    bp = np.array([o[0], o[1], FLOOR + 0.00005 + lift + lift_handle])
    bq = _yaw_q(math.pi + math.radians(theta))
    return {"handle": state_from_pose(HANDLE, hp, (0, 0, 0, 1)),
            "blade": state_from_pose(BLADE, bp, bq, lin_vel=(vel, 0, 0)),
            "lamp_switch": state_from_pose(LAMP, np.asarray(LAMP_POS) + d - [pressed, 0, 0], (0, 0, 0, 1))}


FAR = {"left": {"tcp": np.array([0.20, 0.25, 0.95]), "gap": 0.09}, "right": {"tcp": np.array([0.20, -0.25, 0.95]),
                                                                              "gap": 0.09}}


def _step(objects, arms=FAR) -> float:
    _EP["arms"] = arms
    _EP["tick"] += 1
    s = score(objects)
    assert success(objects) == (s >= 1.0)
    assert score(objects) == s
    return s


def _press(theta: float) -> float:
    """How far the heel presses the lamp plunger (m) at ``theta`` deg with the pin on the blade's origin (hold_pose's lamp)."""
    r = hold_pose(theta, 0.0, 0.0)
    return LAMP_TRAVEL if r is None else r[1]["press"]


def _swing(theta_to: float, arms=FAR, start: float = 40.0) -> float:
    """Step the episode through a swing from ``start`` to ``theta_to`` deg with the plunger and the lamp where the heel puts them."""
    out = 0.0
    for th in np.arange(start, theta_to + 1e-9, 1.0):
        out = _step(_states(float(th), pressed=_press(float(th))), arms)
    return out


def _swing_step(step_deg: float, start: float, end: float, arms=FAR) -> float:
    """Coarse opening swing with fixed ``step_deg`` (mimics a fast sim step through the crest)."""
    out = _step(_states(start, pressed=_press(start)), arms)
    th = start
    while th + step_deg <= end + 1e-9:
        th += step_deg
        out = _step(_states(float(th), pressed=_press(float(th))), arms)
    return out


def _hold_one(final_fn) -> float:
    """Run the locked hold and probe to completion."""
    for _ in range(HOLD_STEPS - 2):
        _step(final_fn())
    _step(final_fn())
    for _ in range(PROBE_RAMP - 1):
        _step(final_fn())
    for _ in range(PROBE_STEPS - PROBE_RAMP):
        _step(final_fn())
    return _step(final_fn())


def _cases() -> int:
    n = 0

    def check(name, got, want):
        nonlocal n
        assert abs(got - want) < 1e-9, (name, got, want)
        n += 1

    def fresh():
        _new_episode()
        _EP.pop("glow_reset", None)

    def final(theta=90.0, **kw):
        return _states(theta, pressed=kw.pop("pressed", 0.9 * MM), **kw)

    fresh()
    check("start layout", _step(_states(0.0, BLADE_START)), 0.0)
    u40, _ = blade_dirs(40.0)
    fresh()
    check("blade laid with its back end short of the pin", _step(_states(40.0, np.array(PIN) + 0.017 * u40[:2])), 0.0)
    fresh()
    check("pin still in the lead-in chamfer", _step(_states(40.0, np.array(PIN) + 0.010 * u40[:2])), 0.0)
    fresh()
    check("pin 2.5 mm into the slot", _step(_states(40.0, np.array(PIN) + 0.0065 * u40[:2])), 0.25)
    fresh()
    check("hooked at 40 deg", _step(_states(40.0)), 0.25)
    held = {"left": FAR["left"], "right": {"tcp": _grip_tcp(40.0, PIN), "gap": 0.010}}
    fresh()
    check("hooked but still held", _step(_states(40.0), held), 0.0)
    fresh()
    check("hooked but the handle lifted 3 cm off the tray", _step(_states(61.0, lift_handle=0.03)), 0.0)
    fresh()
    check("hooked at 59 deg", _step(_states(59.0)), 0.25)
    fresh()
    check("hooked at 61 deg", _step(_states(61.0)), 0.5)
    # the full swing: the plunger is pushed in over the chamfer, the blade drops onto the step and the lamp comes on
    fresh()
    _swing(85.0)
    assert _EP["crossed"] and _EP["clear"] == 0
    n += 1
    check("swung through the chamfer, first clear step", _step(final()), 0.5)
    for _ in range(HOLD_STEPS - 2):
        _step(final())
    check("19 clear steps", _EP["last"], 0.5)
    assert probe_newton() == 0.0
    check("20 clear steps: the probe starts", _step(final()), 0.5)
    assert _EP["probe"]["state"] == "loading" and abs(probe_newton() - PROBE_N / PROBE_RAMP) < 1e-12
    for _ in range(PROBE_RAMP - 1):
        _step(final())
    assert abs(probe_newton() - PROBE_N) < 1e-12
    for _ in range(PROBE_STEPS - PROBE_RAMP):
        _step(final())
    assert _EP["clear"] == HOLD_STEPS + PROBE_STEPS - 1
    check("probe not yet complete (39 clear steps)", _EP["last"], 0.5)
    check("probe passed (40 clear steps): 1.0", _step(final()), 1.0)
    assert _EP["probe"]["state"] == "passed" and _EP["probe"]["runs"] == 1 and probe_newton() == 0.0
    n += 4
    check("still 1.0 after the probe", _step(final()), 1.0)
    near = {"left": {"tcp": np.array([L_GRIP_X, PIN[1] + BARREL_Y, Z_LEFT + 0.04]), "gap": 0.09}, "right": FAR["right"]}
    check("open hand 40 mm over the handle", _step(final(), near), 0.5)
    # a blade that never pushed the plunger in over the chamfer cannot score 1.0 (it was not locked on)
    fresh()
    for _ in range(25):
        _step(final())
    check("90 deg and lit but never crossed the chamfer", _EP["last"], 0.5)
    fresh()
    for th in np.arange(40.0, 88.0, 1.0):  # pin 4 mm out of the slot bottom: hinge never catches, crest does not count
        o = np.array(PIN) + 0.004 * blade_dirs(float(th))[0][:2]
        _step(_states(float(th), origin=o, pressed=_press(float(th))))
    for _ in range(HOLD_STEPS + PROBE_STEPS + 5):
        _step(final())
    check("swung with the pin short of the seat, hinge never on, never crossed", _EP["last"], 0.5)
    fresh()
    for th in np.arange(84.2, 89.5, 0.5):  # started past the crest: never rode 80-84
        _step(_states(float(th), pressed=_press(float(th))))
    assert not _EP["crossed"]
    n += 1
    # the probe: the locked state breaks under it, then a new hold re-arms it
    fresh()
    _swing(87.0)
    for _ in range(HOLD_STEPS + 8):
        _step(final())
    check("folded under the probe", _step(_states(84.0, pressed=0.0)), 0.5)
    assert _EP["probe"]["state"] == "failed" and _EP["probe"]["fails"] == 1 and probe_newton() == 0.0
    for _ in range(HOLD_STEPS + PROBE_STEPS - 1):
        _step(final())
    check("held again, probe 2 not yet complete", _EP["last"], 0.5)
    check("held again through probe 2: 1.0", _step(final()), 1.0)
    assert _EP["probe"]["runs"] == 2 and _EP["probe"]["state"] == "passed"
    n += 2
    fresh()
    _swing(87.0)
    for _ in range(3 * HOLD_STEPS):
        if _EP["clear"] == HOLD_STEPS:
            break
        _step(final())
    assert _EP["clear"] == HOLD_STEPS
    for k in range(PROBE_STEPS):  # jolted and turned 1.5 deg by the probe but still locked: stillness is not judged under it
        _step(final(88.5 if k % 2 else 90.0, pressed=0.8 * MM, vel=0.05))
    check("moving under the probe but locked throughout: 1.0", _EP["last"], 1.0)
    fresh()
    _swing(87.0)
    for _ in range(3 * HOLD_STEPS):
        if _EP["clear"] == HOLD_STEPS - 1:
            break
        _step(final())
    assert _EP["clear"] == HOLD_STEPS - 1
    check("moving on the last hold step before the probe", _step(final(vel=0.05)), 0.5)
    assert _EP["clear"] == 0
    fresh()
    assert probe_newton() == 0.0 and _EP["probe"]["state"] == "idle" and _EP["ext_max"] == 0.0
    _swing(87.0)
    assert _EP["ext_max"] >= CROSS_EXT
    for _ in range(HOLD_STEPS + 2):
        _step(final())
    _EP["violations"].add("offline")
    assert probe_newton() == 0.0
    n += 2
    # the slot taken off the pin clears the crossing: hooking it back at 90 deg is not a lock
    fresh()
    _swing(87.0)
    _step(_states(90.0, np.array(PIN) + 0.020 * blade_dirs(90.0)[0][:2]))
    assert not _EP["crossed"]
    for _ in range(HOLD_STEPS + PROBE_STEPS + 5):
        _step(final())
    check("slot taken off the pin and hooked back at 90 deg", _EP["last"], 0.5)
    # crossed, but resting on the flank short of the stop (not seated, lamp off)
    fresh()
    _swing(85.0)
    for _ in range(25):
        _step(_states(85.0, pressed=0.0))
    check("stopped on the flank at 85 deg, lamp off", _EP["last"], 0.5)
    fresh()
    _swing(87.0)
    for _ in range(25):
        _step(final(pressed=0.2 * MM))
    check("seated but the lamp not on (dark)", _EP["last"], 0.5)
    fresh()
    _swing(87.0)
    for _ in range(25):
        _step(final(97.0, pressed=1.0 * MM))
    check("97 deg", _EP["last"], 0.5)
    fresh()
    u90, _ = blade_dirs(90.0)
    for _ in range(25):  # heel pushed on the switch with the slot off the pin
        _step(_states(90.0, np.array(PIN) + 0.020 * u90[:2], pressed=1.0 * MM))
    check("heel pushed on the switch, not hooked", _EP["last"], 0.0)
    fresh()
    _swing(87.0)
    for _ in range(25):
        _step(final(vel=0.05))
    check("still moving", _EP["last"], 0.5)
    fresh()
    _swing(87.0)
    for k in range(45):  # creeping 0.1 deg per step inside the lit band: the hold restarts every time it passes 1 deg
        th = 86.0 + 0.1 * (k % 40)
        _step(final(th, pressed=_press(th)))
    check("blade creeping 0.1 deg per step", _EP["last"], 0.5)
    fresh()
    _swing(87.0)
    for _ in range(25):
        _step(final(), {"left": FAR["left"], "right": {"tcp": FAR["right"]["tcp"], "gap": 0.012}})
    check("a closed gripper far away", _EP["last"], 0.5)
    fresh()
    _swing(87.0)
    for _ in range(25):
        _step(final(handle_dxy=(-0.10, 0.0)))
    check("assembled but the handle 100 mm back, over the tray wall", _EP["last"], 0.5)
    # latched
    fresh()
    _swing(87.0)
    _step(final(origin=(0.3, 0.9)))
    for _ in range(25):
        _step(final())
    check("blade off the bench earlier", _EP["last"], 0.0)
    fresh()
    ut = np.array(PIN) - 0.004 * blade_dirs(60.0)[1][:2]  # pin 4 mm across into the heel's prong
    _step(_states(60.0, ut))
    _swing(87.0)
    for _ in range(25):
        _step(final())
    check("blade forced on across the slot", _EP["last"], 0.0)
    fresh()
    check("reset clears the latch", _step(_states(40.0)), 0.25)
    assert not _EP["crossed"]
    assert pin_depth(np.array([0.0, 0.0, HEEL_MID])) == 0.0
    assert pin_depth(np.array([-10 * MM, 0.3 * MM, HEEL_MID])) == 0.0  # in the lead-in chamfer
    assert pin_depth(np.array([-10 * MM, 5.0 * MM, HEEL_MID])) > 1.0 * MM  # pushed into the prong's steel
    assert pin_depth(np.array([0.0, 0.0, HEEL_MID + 0.015])) == 0.0
    n += 4
    # crest crossing: coarse steps must count the 80-84 deg band even when no sample lands inside it
    for step_deg in (4.0, 5.0, 5.6, 6.0, 8.0, 12.0):
        fresh()
        _swing(75.0, start=40.0)
        _swing_step(step_deg, 76.0, 91.0)
        assert _EP["crossed"], (step_deg, _EP.get("prev_theta"))
        n += 1
        if _EP["clear"] > 0 or _EP["m"]["theta"] >= OPEN_DEG[0]:
            _step(_states(84.0, pressed=_press(84.0)))
        assert _EP["clear"] == 0
        check(f"fast {step_deg} deg/step first clear", _step(final()), 0.5)
        check(f"fast {step_deg} deg/step through crest crossed", _hold_one(final), 1.0)
    fresh()
    _swing_step(6.0, 40.0, 78.0)
    assert not _EP["crossed"]
    for _ in range(HOLD_STEPS + PROBE_STEPS + 3):
        _step(_states(78.0, pressed=_press(78.0)))
    check("swing stopped before 80 deg, never crossed", _EP["last"], 0.5)
    n += 1
    # crest segment only between consecutive hooked swings with the hinge on (not slide-on with hinge off)
    for a, b in ((78.0, 86.0), (83.0, 85.5), (79.0, 88.0)):
        fresh()
        o_short = np.array(PIN) + 0.004 * blade_dirs(float(a))[0][:2]
        _step(_states(float(a), origin=o_short, pressed=_press(float(a))))
        assert not _EP["crossed"]
        _step(_states(float(b), pressed=_press(float(b))))
        assert not _EP["crossed"]
        for _ in range(HOLD_STEPS + PROBE_STEPS + 3):
            _step(final(float(b)))
        check(f"hinge-off then seat {a}->{b}, segment not crest-ridden", _EP["last"], 0.5)
        n += 1
        fresh()
        u, _ = blade_dirs(a)
        _step(_states(float(a), np.array(PIN) + 0.010 * u[:2]))  # unhooked: pin u = -10 mm in the slot
        _step(_states(float(a), origin=o_short, pressed=_press(float(a))))
        assert not _EP["crossed"]
        _step(_states(float(b), pressed=_press(float(b))))
        assert not _EP["crossed"]
        for _ in range(HOLD_STEPS + PROBE_STEPS + 3):
            _step(final(float(b)))
        check(f"unhooked lead-in then hinge-off {a}->{b}", _EP["last"], 0.5)
        n += 1
    return n


def _sweep_clearance() -> list[str]:
    """The blade's heel against the handle in plan (2D, z overlaps everywhere): along the demo's slide and swing,
    no blade collider may cut more than 0.2 mm into the handle's head block, barrel, pin or lamp switch."""
    hs = [s for s in handle_shapes() if s.collide and isinstance(s, BoxShape) and s.centre[2] > CHEEK_LO[1] + 1e-6
          and s.centre[2] < CHEEK_HI[0] - 1e-6] + [s for s in handle_shapes() if s.collide and isinstance(s, BoxShape)
                                                    and s.size[2] > 0.01]
    head = [(np.array(s.centre[:2]), np.array(s.size[:2]) / 2) for s in hs]
    head.append((np.array([FRONT_X, FRONT_Y]), np.array([FRONT_W / 2, FRONT_L / 2])))  # the cradle's front post
    head.append((np.array([NEAR_X, BARREL_Y - RAIL_Y]), np.array([NEAR_L / 2, RAIL_T / 2])))  # blade-side rail, holds the head
    lamp = (np.array([STOP_X + LAMP_PROUD - 1 * MM, LAMP_Y]), np.array([1 * MM, 1 * MM]))
    barrel = (np.array([(BARREL_X[0] + BARREL_X[1]) / 2, BARREL_Y]), np.array([(BARREL_X[1] - BARREL_X[0]) / 2,
                                                                               BARREL_R]))
    pts = []
    for s in blade_shapes():
        if not s.collide or s.centre[2] + s.size[2] / 2 < CHEEK_LO[1] or s.centre[2] - s.size[2] / 2 > CHEEK_HI[0]:
            continue
        r = rotation(s.quat())[:2, :2]
        hx, hy = s.size[0] / 2, s.size[1] / 2
        for a in np.linspace(-hx, hx, max(2, int(2 * hx / 0.0005) + 1)):
            for b in (-hy, hy):
                pts.append(np.array(s.centre[:2]) + r @ [a, b])
        for b in np.linspace(-hy, hy, max(2, int(2 * hy / 0.0005) + 1)):
            for a in (-hx, hx):
                pts.append(np.array(s.centre[:2]) + r @ [a, b])
    pts = np.array(pts)

    def worst(theta, out, with_lamp=False, across=0.0):
        u, t = blade_dirs(theta)
        o = out * u[:2] + across * t[:2]  # handle frame: the pin at the origin
        w = o + pts[:, :1] * u[:2] + pts[:, 1:2] * t[:2]
        deep = 0.0
        for c, h in head + [barrel] + ([lamp] if with_lamp else []):
            d = np.min(np.stack([h[0] - np.abs(w[:, 0] - c[0]), h[1] - np.abs(w[:, 1] - c[1])]), axis=0)
            deep = max(deep, float(d.max()))
        dp = PIN_R - np.hypot(w[:, 0], w[:, 1])
        return max(deep, float(dp.max()))

    play = (-(SLOT_HALF - PIN_R), 0.0, SLOT_HALF - PIN_R)
    slide = max(worst(THETA0, s, across=c) for s in np.linspace(HOOK_OUT + 0.004, 0.0, 33) for c in play)
    swing = max(worst(th, 0.0, across=c) for th in np.arange(THETA0, 89.01, 0.5) for c in play)
    assert slide <= 0.2 * MM and swing <= 0.2 * MM, (slide, swing)
    at90 = worst(90.0, 0.0, True)
    assert LAMP_ON + 0.3 * MM <= at90 <= LAMP_TRAVEL, at90  # the heel presses the switch about 0.9 mm
    return [f"heel vs handle in plan: slide worst {slide * 1000:.2f} mm, swing {THETA0:.0f}-89 deg worst "
            f"{swing * 1000:.2f} mm (pin anywhere in the slot's play), at 90 deg the heel presses the lamp switch "
            f"{at90 * 1000:.2f} mm (travel {LAMP_TRAVEL / MM:.1f})"]


if __name__ == "__main__":
    import json

    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    meta = json.loads((pathlib.Path(__file__).parent / "task.json").read_text())
    assert meta["instruction"] == INSTRUCTION, "task.json instruction differs from INSTRUCTION"
    print(f"scoring: {_cases()} cases pass")
    for line in _sweep_clearance() + swing_report() + hold_report() + pin_retention_report():
        print(line)
    task = load(__file__)
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / 20.0)[0] for k in range(round(plan.duration * 20) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.2, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan ({round(plan.duration * 20)} steps), worst IK {plan.worst_ik_mm:.3f} mm "
              f"/ {plan.worst_ik_deg:.3f} deg")
