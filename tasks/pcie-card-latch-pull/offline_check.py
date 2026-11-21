"""Offline checks for the card-latch-pull task: score ladder, latched violations, cheats, demo replay, collider overlap."""

from __future__ import annotations

import contextlib
import inspect
import io
import math
import time

import numpy as np

ZERO = np.zeros(3)
PRESSED = -0.15  # slot latch angle (rad) while the tab is pressed


# - synthetic states -------------------------------------------------------------------------------------------------
def rot_x(deg):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def matrix_to_xyzw(m):
    w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    if w > 1e-6:
        return np.array([(m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w), w])
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(m[i, i] - m[j, j] - m[k, k] + 1) * 2
    q = np.zeros(4)
    q[i] = s / 4
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    q[3] = (m[k, j] - m[j, k]) / s
    return q


def card_state(T, rot, origin, lin_vel=ZERO):
    from mechkit import MechState

    origin = np.asarray(origin, float)
    return MechState(pos=origin, quat_xyzw=matrix_to_xyzw(rot), lin_vel=np.asarray(lin_vel, float), ang_vel=ZERO.copy(),
                     low=origin - 0.1, high=origin + 0.1, joints={}, origin=origin)


def slot_state(T, hook=0.0, latch=0.0):
    from mechkit import MechState

    joints = {n: 0.0 for n in ("pad_tail_p", "pad_tail_n", "pad_front_p", "pad_front_n", "hook", "latch")}
    joints["hook"], joints["latch"] = hook, latch
    o = np.asarray(T.CARD_ORIGIN, float)
    return MechState(pos=o, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=ZERO.copy(), ang_vel=ZERO.copy(), low=o, high=o,
                     joints=joints, origin=o)


def hand(T, tcp, gap, rot=None):
    rot = np.eye(3) if rot is None else rot
    tcp = np.asarray(tcp, float)
    return {"finger_gap": gap, "tcp": tcp, "rot": rot, "points": T.gripper_probe_points(tcp, rot, gap)}


def parked(T, gap=0.09):
    return {"left": hand(T, (0.30, 0.55, T.TOP + 0.30), gap), "right": hand(T, (0.0, -0.50, T.TOP + 0.30), gap)}


def play(T, frames, hook=0.0, reset=True, credit=False):
    """Score one step per (card, latch, arms) frame, each on a fresh states dict; returns the scores.

    credit=True starts as if a hand had already pressed the tab since seating, for states away from the slot."""
    if reset:
        T._reset_episode()
    if credit:
        T._S["pressed_in_rise"] = True
    out = []
    with contextlib.redirect_stdout(io.StringIO()):
        for card, latch, arms in frames:
            T._HANDS["arms"] = arms
            out.append(T.score({"card": card, "slot": slot_state(T, hook, latch)}))
    return out


def run_steps(T, card, arms, n, hook=0.0, latch=PRESSED, reset=True):
    """Score n consecutive steps of a fixed state; returns the list of scores."""
    return play(T, [(card, latch, arms)] * n, hook, reset, credit=True)


def last(T, card, arms, n=1, **kw):
    return run_steps(T, card, arms, n, **kw)[-1]


# - poses -------------------------------------------------------------------------------------------------------------
def seated(T):
    return card_state(T, T.FRAME_R, T.CARD_ORIGIN)


def tail_raised(T, lift):
    """Seated card turned about the card-frame y axis through the front finger end until the tail is `lift` m up."""
    axis = T.FRAME_R @ np.array([0.0, 1.0, 0.0])
    pivot = T._cfw((T.FINGER_X1, 0.0, 0.0))
    tail = T.CARD_ORIGIN + T.FRAME_R @ T.TAIL_REF_LOCAL

    def height(deg):
        return float((T._rot_axis(axis, deg) @ (tail - pivot))[2] + pivot[2] - tail[2])

    sign = 1.0 if height(1.0) > 0 else -1.0
    lo, hi = 0.0, 10.0
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if height(sign * mid) < lift else (lo, mid)
    r = T._rot_axis(axis, sign * lo)
    return card_state(T, r @ T.FRAME_R, pivot + r @ (T.CARD_ORIGIN - pivot))


def pitched(T, deg):
    """Seated card turned nose-up by deg about the card-frame y axis through its tail-bottom corner (tail corner stays put)."""
    axis = T.FRAME_R @ np.array([0.0, 1.0, 0.0])
    pivot = T.CARD_ORIGIN + T.FRAME_R @ T.TAIL_REF_LOCAL
    far = T.FRAME_R @ np.array([T.CARD_LEN, 0.0, 0.0])
    sign = 1.0 if (T._rot_axis(axis, 1.0) @ far)[2] > far[2] else -1.0
    r = T._rot_axis(axis, sign * deg)
    return card_state(T, r @ T.FRAME_R, pivot + r @ (T.CARD_ORIGIN - pivot))


def straight_up(T, rise):
    return card_state(T, T.FRAME_R, T.CARD_ORIGIN + np.array([0, 0, rise]))


def flat_rot(T):
    """Card x along world x, PCB normal vertical (card y up), card z along world -y."""
    return np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])


def in_tray(T, rot=None, vel=ZERO, dx=0.0):
    """Card at rest on the tray floor, centred in the tray (shifted by dx along x), 0.5 mm above the floor."""
    rot = flat_rot(T) if rot is None else rot
    pts = T.card_points(card_state(T, rot, ZERO), T.ALL_CARD_SAMPLES)
    centre = (pts.min(0) + pts.max(0)) / 2
    target = (T.TRAY_LO + T.TRAY_HI) / 2 + np.array([dx, 0.0, 0.0])
    floor = T.TOP + T.TRAY_LO[2] + T.TRAY_FLOOR
    origin = np.array([target[0] - centre[0], target[1] - centre[1], floor + 0.0005 - pts[:, 2].min()])
    return card_state(T, rot, origin, vel)


def card_top(T, card):
    return T.card_points(card, T.ALL_CARD_SAMPLES)[:, 2].max()


def card_center(T, card):
    return T.card_points(card, T.ALL_CARD_SAMPLES).mean(0)


def tab_frame(T, latch, offset_xy=(0.0, 0.0)):
    """World point on the orange tab top and the tab-top normal at latch angle `latch`.

    offset_xy is from the tab-top centre along the 19 mm side (world x at rest) and the 10 mm side (world y at rest)."""
    c, s = math.cos(latch), math.sin(latch)
    rot = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    pivot = np.array(T.LATCH_PIVOT)
    local = np.array([0.5 * (T.TAB_X[0] + T.TAB_X[1]) + offset_xy[1], T.TAB_YC - offset_xy[0], T.TAB_TOP_Z])
    return T._cfw(pivot + rot @ (local - pivot)), T.FRAME_R @ rot @ np.array([0.0, 0.0, 1.0])


def on_tab(T, latch=PRESSED, offset_xy=(0.0, 0.0), lift=0.0):
    """Right gripper closed in the demo's press pose (R_PRESS), its lowest pad point at `offset_xy` on the tab top,
    `lift` m above the surface along its normal (the left arm stays parked)."""
    point, normal = tab_frame(T, latch, offset_xy)
    pads = T.finger_surface_points(ZERO, T.R_PRESS, 0.0)
    tcp = point - pads[int(np.argmin(pads @ normal))] + lift * normal
    arms = parked(T)
    arms["right"] = hand(T, tcp, 0.0, T.R_PRESS)
    return arms


def hovering(T, latch=PRESSED, h=0.020, offset_xy=(0.0, 0.0)):
    return on_tab(T, latch, offset_xy, h)


def hold(T, card, gap=0.004):
    """A closed gripper whose pads touch the highest point of the card."""
    pts = T.card_points(card, T.ALL_CARD_SAMPLES)
    p = pts[int(np.argmax(pts[:, 2]))]
    return hand(T, p + np.array([0.0, 0.0, 0.030]), gap)


# - checks ------------------------------------------------------------------------------------------------------------
def check_ladder(T):
    away = parked(T)
    pressing = on_tab(T)
    assert last(T, seated(T), away) == 0.0, "seated card must score 0"
    assert last(T, tail_raised(T, 0.006), pressing) == 0.25, "tail raised 6 mm with the tab pressed must score 0.25"
    assert last(T, tail_raised(T, 0.004), pressing) == 0.0, "tail raised 4 mm must score 0"

    lifted = card_state(T, T.FRAME_R, T.CARD_ORIGIN + np.array([0, 0, 0.030]))
    band = T.GRIP_W + np.array([0, 0, 0.030])
    holder = {"left": hand(T, band, 0.016), "right": pressing["right"]}
    assert T.card_held(lifted, holder), "gripper at the band must hold the lifted card"
    assert last(T, lifted, holder) == 0.5, "card lifted 30 mm must score 0.5"

    tray = in_tray(T)
    assert T.card_in_tray(tray) and T.card_tilt_deg(tray) < 1.0, "tray pose helper must be inside and flat"
    scores = run_steps(T, tray, away, 25)
    assert scores[:19] == [0.5] * 19, f"first 19 steps must be 0.5, got {scores[:19]}"
    assert scores[19:] == [1.0] * 6, f"from the 20th step must be 1.0, got {scores[19:]}"
    assert T.success({}), "success() must be True after the hold"

    c = card_center(T, tray)
    closed = {"left": hand(T, (0.30, 0.55, T.TOP + 0.30), 0.016), "right": away["right"]}
    assert max(run_steps(T, tray, closed, 30)) == 0.5, "one closed gripper must keep 0.5"
    near = {"left": hand(T, (c[0], c[1], card_top(T, tray) + 0.034 + 0.040), 0.09), "right": away["right"]}
    ok, worst = T.hands_clear(tray, near)
    assert not ok and worst < 0.050, f"gripper 40 mm from the card must not clear (worst {worst:.3f})"
    assert max(run_steps(T, tray, near, 30)) == 0.5, "gripper within 40 mm must keep 0.5"
    assert not T.success({}), "success must be False with a near gripper"

    tilted = in_tray(T, rot_x(15) @ flat_rot(T))
    assert T.card_in_tray(tilted), "tilted pose helper must stay inside the tray"
    assert max(run_steps(T, tilted, away, 30)) == 0.5, "card tilted 15 deg must stay at 0.5"

    moving = in_tray(T, vel=np.array([0.05, 0, 0]))
    assert max(run_steps(T, moving, away, 30)) < 1.0, "moving card must not reach 1.0"
    over_wall = in_tray(T, dx=0.09)
    assert not T.card_in_tray(over_wall)
    assert max(run_steps(T, over_wall, away, 30)) == 0.5, "card over the tray wall must stay at 0.5"
    standing = in_tray(T, rot_x(90) @ flat_rot(T))
    assert max(run_steps(T, standing, away, 30)) < 1.0, "card on its edge in the tray must not reach 1.0"
    return "ladder ok (0 / 0.25 / 0.5 / 1.0 after 20 steps)"


def check_violations(T):
    away = parked(T)
    good = in_tray(T)
    raised = tail_raised(T, 0.004)

    play(T, [(seated(T), 0.0, away), (raised, 0.0, away)])
    assert T._S["violation"] == "latch_forced", "tail up with no hand on the tab must violate"
    after = play(T, [(good, PRESSED, away)] * 30, reset=False)
    assert after == [0.0] * 30 and not T.success({}), "latch_forced must persist even in a perfect state"
    T._reset_episode()
    assert T._S["violation"] is None, "reset must clear the violation"

    pressing = on_tab(T)
    play(T, [(seated(T), PRESSED, pressing), (raised, PRESSED, pressing), (tail_raised(T, 0.006), PRESSED, pressing)])
    assert T._S["violation"] is None, "rising with a hand pressing the tab must not violate"
    up5 = tail_raised(T, 0.005)
    play(T, [(seated(T), PRESSED, pressing), (up5, PRESSED, pressing), (up5, 0.0, away)])
    assert T._S["violation"] is None, "latch springing back after the tail is already up must not violate"

    slow = [(tail_raised(T, 0.0005 * i), 0.0, away) for i in range(21)]
    play(T, slow)
    assert T._S["violation"] == "latch_forced", "slow tug (0.5 mm steps to 10 mm) without a hand must violate"
    play(T, [(seated(T), 0.0, away), (tail_raised(T, 0.004), 0.0, away)])
    assert T._S["violation"] == "latch_forced", "fast tug (4 mm in one step) without a hand must violate"

    ramp = [(tail_raised(T, 0.006 * i / 12), PRESSED, pressing) for i in range(13)]
    up6 = tail_raised(T, 0.006)
    scores = play(T, [(seated(T), PRESSED, pressing)] + ramp + [(up6, 0.0, away)] * 3)
    assert T._S["violation"] is None and scores[-1] == 0.25, "tug after a tab press must not violate and score 0.25"

    play(T, [(seated(T), 0.0, away)] + [(tail_raised(T, 0.001 * i), PRESSED, away) for i in range(1, 6)])
    assert T._S["violation"] == "latch_forced", "card tail pushing the latch open with no hand at the tab must violate"

    play(T, [(seated(T), PRESSED, pressing), (up5, PRESSED, pressing), (seated(T), 0.0, away),
             (seated(T), PRESSED, pressing), (up5, PRESSED, pressing), (up5, 0.0, away)])
    assert T._S["violation"] is None, "reseating then raising with a hand on the tab must not violate"
    play(T, [(seated(T), PRESSED, pressing), (up5, PRESSED, pressing), (seated(T), 0.0, away), (up5, 0.0, away)])
    assert T._S["violation"] == "latch_forced", "reseat must reset the pressed flag"

    up100 = card_state(T, T.FRAME_R, T.CARD_ORIGIN + np.array([0, 0, 0.100]))
    up50 = card_state(T, T.FRAME_R, T.CARD_ORIGIN + np.array([0, 0, 0.050]))
    held = {"left": hold(T, up100), "right": away["right"]}
    assert T.card_held(up100, held)
    scores = play(T, [(up100, PRESSED, held), (up100, PRESSED, away), (up50, PRESSED, away)], credit=True)
    assert T._S["violation"] == "card_dropped" and scores[-1] == 0.0, "50 mm fall after release must violate"
    play(T, [(good, PRESSED, away)] * 25, reset=False)
    assert T._S["violation"] == "card_dropped" and not T.success({}), "card_dropped must persist"

    edge = in_tray(T, rot_x(65) @ flat_rot(T))
    play(T, [(edge, PRESSED, {"left": hold(T, edge), "right": away["right"]}), (edge, PRESSED, away),
             (good, PRESSED, away), (good, PRESSED, away)], credit=True)
    assert T._S["violation"] is None, "release from a tilted edge onto the tray floor must not violate"

    c_mid = T._w((T.COOLER_LO + T.COOLER_HI) / 2)
    cooler = card_state(T, T.FRAME_R, (c_mid[0], c_mid[1] - 0.05, T.TOP + 0.03))  # card passes through the cooler
    assert last(T, cooler, away) == 0.0 and T._S["violation"] == "card_hit_cooler", T._S["violation"]

    origin = T.CARD_ORIGIN + T.FRAME_R @ np.array([0.0, 0.030, 0.0])
    board = card_state(T, T.FRAME_R, (origin[0], origin[1], T._w(T.MB_HI)[2]))
    assert last(T, board, away) == 0.0 and T._S["violation"] == "card_hit_board", T._S["violation"]

    off = card_state(T, T.FRAME_R, (0.60, 0.0, T.TOP + 0.03))
    assert last(T, off, away) == 0.0 and T._S["violation"] == "card_off_bench", T._S["violation"]

    steps = [5 * i for i in range(1, 8)]
    play(T, [(seated(T), 0.0, away)] + [(pitched(T, d), -0.15, away) for d in steps])
    assert T._S["violation"] == "latch_forced", "bracket-end-first pitch to 35 deg with no hand on the tab must violate"
    play(T, [(seated(T), PRESSED, pressing)] + [(pitched(T, d), PRESSED, pressing) for d in steps])
    assert T._S["violation"] is None, f"pitch with a hand on the pressed tab must not violate, got {T._S['violation']}"
    hover = hovering(T)
    play(T, [(seated(T), 0.0, away)] + [(pitched(T, d), -0.15, hover) for d in steps])
    assert T._S["violation"] == "latch_forced", "hovering 20 mm above the tab with the latch open must violate"
    assert not T.hand_on_tab(hover, -0.15) and T.hand_on_tab(pressing, -0.15), "hand_on_tab must need contact"
    play(T, [(seated(T), 0.0, away)] + [(straight_up(T, 0.001 * i), 0.0, away) for i in range(1, 13)])
    assert T._S["violation"] == "latch_forced", "straight 12 mm pull with no hand must violate"

    lifted = straight_up(T, 0.030)
    band = T.GRIP_W + np.array([0, 0, 0.030])
    holder = {"left": hand(T, band, 0.016), "right": pressing["right"]}
    over = card_state(T, flat_rot(T), in_tray(T).origin + np.array([0, 0, 0.020]))
    carry = {"left": hold(T, over), "right": away["right"]}
    good_run = ([(seated(T), PRESSED, pressing), (tail_raised(T, 0.006), PRESSED, pressing), (lifted, PRESSED, holder),
                 (over, 0.0, carry), (over, 0.0, carry)] + [(good, 0.0, away)] * 20)
    scores = play(T, good_run)
    assert T._S["violation"] is None and scores[-1] == 1.0, f"legal sequence: {T._S['violation']} {scores}"

    T._reset_episode()
    assert T._S["violation"] is None and T._S["free_ref"] is None and T._S["clear_steps"] == 0
    return ("violations ok (latch forced slow/fast/rocking/reseat/bracket-end pitch/straight pull, legal sequence, "
            "drop, cooler, board, off bench, latching, reset)")


def check_tab_contact(T):
    """Realistic downward press gripper (R_PRESS) touching or hovering over the orange tab while the tail rises 6 mm."""
    away = parked(T)
    rise = [tail_raised(T, 0.001 * i) for i in range(1, 7)]

    def violation(latch, arms):
        play(T, [(seated(T), 0.0, away)] + [(card, latch, arms) for card in rise])
        return T._S["violation"]

    def gap_mm(arms, latch):
        g = arms["right"]
        return 1000 * float(T.tab_surface_distance(T.finger_surface_points(g["tcp"], g["rot"], 0.0), latch).min())

    hx, hy = 1000 * 0.5 * (T.TAB_Y[1] - T.TAB_Y[0]), 1000 * 0.5 * (T.TAB_X[1] - T.TAB_X[0])  # 9.5 and 5.0 mm
    spots = [(0.0, 0.0)] + [(s * d, 0.0) for d in (3, 5, 7, 9) for s in (1, -1)]
    spots += [(0.0, s * d) for d in (3, 4) for s in (1, -1)]
    spots += [(sx * hx, sy * hy) for sx in (1, -1) for sy in (1, -1)]  # corners
    spots += [(s * hx, 0.0) for s in (1, -1)] + [(0.0, s * hy) for s in (1, -1)]  # edge midpoints
    for ox, oy in spots:
        arms = on_tab(T, PRESSED, (ox / 1000, oy / 1000))
        assert gap_mm(arms, PRESSED) < 1e-6, (ox, oy, gap_mm(arms, PRESSED))
        assert violation(PRESSED, arms) is None, f"press at ({ox:+.1f}, {oy:+.1f}) mm on the tab must not violate"
    for h in (0.003, 0.004, 0.010, 0.020):
        arms = hovering(T, PRESSED, h)
        assert abs(gap_mm(arms, PRESSED) - 1000 * h) < 1e-6
        assert violation(PRESSED, arms) == "latch_forced", f"hover {h * 1000:.0f} mm above the tab must violate"
    for deg in (-5.5, -8.6):
        latch = math.radians(deg)
        for h in (0.0031, 0.004):
            assert violation(latch, hovering(T, latch, h)) == "latch_forced", f"latch {deg} deg, hover {h * 1000} mm"
        touching = on_tab(T, latch)
        assert T.hand_on_tab(touching, latch) and violation(latch, touching) is None, f"latch {deg} deg, contact"
    small = math.radians(-3.5)
    assert violation(small, on_tab(T, small)) == "latch_forced", "contact with the latch only 3.5 deg open must violate"
    return (f"tab contact ok ({len(spots)} press spots incl. +-3/5/7/9 mm along the 19 mm side, +-3/4 mm across, "
            "corners and edges clean; hover 3/4/10/20 mm, hover 3.1/4 mm at -5.5/-8.6 deg and contact at -3.5 deg forced)")


def check_cheats(T):
    away = parked(T)
    upright = card_state(T, T.FRAME_R, (0.40, 0.22, T.TOP + 0.006))
    assert max(run_steps(T, upright, away, 30)) < 1.0, "upright card must not reach 1.0"

    base = in_tray(T)
    over = card_state(T, flat_rot(T), base.origin + np.array([0, 0, 0.020]))
    assert T.card_in_tray(over)
    held = {"left": hold(T, over), "right": away["right"]}
    assert T.card_held(over, held)
    scores = run_steps(T, over, held, 30)
    assert max(scores) == 0.5, f"card held flat over the tray must score 0.5, got {max(scores)}"

    centre = (T.TRAY_LO + T.TRAY_HI) / 2
    floor = T.TOP + T.TRAY_LO[2] + T.TRAY_FLOOR
    for height, expected in ((0.045, False), (0.060, True)):  # lowest probe point above the tray floor centre
        arm = {"left": hand(T, (centre[0], centre[1], floor + height + 0.034), 0.09), "right": away["right"]}
        ok, worst = T.hands_clear(seated(T), arm)
        assert ok == expected, f"gripper {height * 1000:.0f} mm above the tray floor: clear {ok}, worst {worst:.4f}"

    free_hands = run_steps(T, seated(T), away, 30)
    assert max(free_hands) == 0.0, "hands clear but card still in the slot must score 0"
    return "cheats ok (incl. hands-clear tray sampling)"


def check_step_cache(T):
    away = parked(T)
    T._reset_episode()
    T._HANDS["arms"] = away
    objects = {"card": seated(T), "slot": slot_state(T)}
    with contextlib.redirect_stdout(io.StringIO()):
        T.success(objects)
        T.score(objects)
    assert T._S["step"] == 1, f"success() then score() on one dict must count one step, got {T._S['step']}"

    T._reset_episode()
    T._S["pressed_in_rise"] = True
    good = in_tray(T)
    with contextlib.redirect_stdout(io.StringIO()):
        for i in range(25):
            objects = {"card": good, "slot": slot_state(T, 0.0, PRESSED)}
            ok = T.success(objects)
            s = T.score(objects)
            assert ok == (s == 1.0), f"success {ok} and score {s} disagree at step {i + 1}"
            assert ok == (i >= 19), f"hold must complete on step 20, got {ok} at step {i + 1}"
    assert T._S["step"] == 25
    return "step cache ok (one evaluation per dict, no one-step lag)"


def check_demo(T):
    from yam_arena import ik
    from yam_arena.demo import ArmPlan
    from yam_arena.station import BASES

    assert not inspect.signature(T.demo).parameters, "demo() must take no arguments"

    def replay():
        plans, acts = {}, {}
        for arm, wps in T.demo().items():
            plan = ArmPlan(wps, BASES[arm])
            steps = int(plan.duration / 0.05) + 1
            acts[arm] = np.array([plan.action7(i * 0.05)[0] for i in range(steps)])
            plans[arm] = plan
        return plans, acts

    plans, acts = replay()
    _, acts2 = replay()
    parts = []
    for arm, plan in plans.items():
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.5, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        q = acts[arm][:, :6]
        assert np.all(q >= ik.LOWER - 1e-9) and np.all(q <= ik.UPPER + 1e-9), f"{arm} joint limit exceeded"
        assert np.array_equal(acts[arm], acts2[arm]), f"{arm} plan not deterministic"
        parts.append(f"{arm} {plan.duration:.1f}s ik {plan.worst_ik_mm:.3f}mm/{plan.worst_ik_deg:.3f}deg")
    return "demo ok: " + ", ".join(parts)


def overlap_depth(lo1, hi1, lo2, hi2):
    """Penetration depth of two boxes: smallest per-axis overlap (<= 0 when apart)."""
    return float((np.minimum(hi1, hi2) - np.maximum(lo1, lo2)).min())


def shape_boxes(mech, offset, label, to_world):
    out = []
    for i, s in enumerate(mech.shapes):
        if s.kind == "box" and s.collide:
            at = np.asarray(s.at, float) + offset
            half = np.asarray(s.size, float) / 2
            lo, hi = to_world(at - half, at + half)
            out.append((f"{label}[{i}]", lo, hi))
    return out


def check_colliders(T):
    bench_up = np.array([0, 0, T.TOP])

    def ident_up(lo, hi):
        return lo + bench_up, hi + bench_up

    card = [(f"card.{n}", *T._cfw_box(lo, hi), n) for n, lo, hi in T.CARD_BOXES]
    static = shape_boxes(T.BENCH, ZERO, "bench", ident_up) + shape_boxes(T.TRAY, ZERO, "tray", ident_up)
    static += [("mat", T._w(T.MAT_LO), T._w(T.MAT_HI)), ("motherboard", T._w(T.MB_LO), T._w(T.MB_HI)),
               ("cooler", T._w(T.COOLER_LO), T._w(T.COOLER_HI)), ("bench_top", T._w(T.BENCH_LO), T._w(T.BENCH_HI))]
    static += [(f"dimm{i}", T._w(lo), T._w(hi)) for i, (lo, hi) in enumerate(T.DIMMS)]
    static += [(f"socket{i}", T._w(lo), T._w(hi)) for i, (lo, hi) in enumerate(T.DIMM_SOCKETS)]
    slot = shape_boxes(T.SLOT, ZERO, "slot", T._cfw_box)
    assert slot, "slot must expose collider boxes"
    for joint in T.SLOT.joints:
        slot += shape_boxes(joint, np.asarray(joint.at, float), f"slot.{joint.name}", T._cfw_box)
    assert any(n.startswith("slot.latch") for n, _, _ in slot), "latch shapes must be checked"

    worst = (-1.0, "")
    bad = []
    for name, lo, hi, kind in card:
        for other, olo, ohi in static + slot:
            d = overlap_depth(lo, hi, olo, ohi)
            if d <= 0:
                continue
            allowed = 0.0002 if (kind in T.FINGER_BOXES and other.startswith("slot")) else 0.0005
            if d > worst[0]:
                worst = (d, f"{name} vs {other}")
            if d > allowed:
                bad.append(f"{name} vs {other}: {d * 1000:.2f} mm")
    for name, lo, hi in slot:
        for other, olo, ohi in static:
            d = overlap_depth(lo, hi, olo, ohi)
            if d > 0:
                if d > worst[0]:
                    worst = (d, f"{name} vs {other}")
                if d > 0.0005:
                    bad.append(f"{name} vs {other}: {d * 1000:.2f} mm")
    assert not bad, "static collider overlap: " + "; ".join(bad)
    return f"colliders ok: deepest overlap {max(worst[0], 0) * 1000:.2f} mm {worst[1]}"


def run(T):
    start = time.time()
    for check in (check_ladder, check_violations, check_tab_contact, check_cheats, check_step_cache, check_demo, check_colliders):
        print(check(T))
    T._reset_episode()
    T._HANDS["arms"] = None
    print(f"offline_check passed in {time.time() - start:.1f} s")
