"""Offline checks for task.py (NumPy + yam_arena kinematics, no simulator). Run: python task.py

1. scoring ladder on hand-built states, cheats, latched violations, the 1 s hands-clear hold, reset;
2. collider interpenetration of every pair of separate bodies at the start, hung, pinned, closed poses and over the
   door's whole swing on its pins;
3. the demo's joint plan replayed at 20 Hz: IK error, URDF joint limits, and every arm link's collision mesh against
   the scene, the door and the pins at their scripted poses (fingertips may touch only the part they hold or push);
4. instruction text identical in task.py, task.json and README.md.
"""

from __future__ import annotations

import json
import math
import pathlib
import xml.etree.ElementTree as ET

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent


# - states -------------------------------------------------------------------------------------------------------------
def _quat_xyzw(rot: np.ndarray) -> np.ndarray:
    from yam_arena.ik import matrix_to_quat_xyzw

    return matrix_to_quat_xyzw(rot)


def _state(T, origin, rot):
    from mechkit import MechState

    o = np.asarray(origin, float)
    z = np.zeros(3)
    return MechState(pos=o, quat_xyzw=_quat_xyzw(rot), lin_vel=z, ang_vel=z, low=o, high=o, joints={}, origin=o)


def _fixed(T, spec):
    return _state(T, spec.pos, np.eye(3))


def door_state(T, origin, open_deg, tilt_deg=0.0):
    rot = T._rz(-open_deg) @ T._rx(tilt_deg)
    return _state(T, origin, rot)


def pin_state(T, head, tilt_deg=0.0, flip=False):
    rot = T._rx(tilt_deg) @ (T._rx(180.0) if flip else np.eye(3))
    return _state(T, head, rot)


def seat(T, i):
    return np.array((T.X_A, T.Y_A, T._wz(T.fixed_knuckles(i)[1][1])))


def scene(T, door=None, pa=None, pb=None):
    s = {
        "enclosure": _fixed(T, T.ENCLOSURE),
        "door_rest": _fixed(T, T.DOOR_REST),
        "pin_block": _fixed(T, T.PIN_BLOCK),
        "door": door or door_state(T, (T.DOOR_START[0], T.DOOR_START[1], T._wz(T.BLOCK_H)), 90.0),
        "pin_a": pa or pin_state(T, (*T.PIN_START[0], T._wz(T.PIN_HOLE_FLOOR + T.PIN_L))),
        "pin_b": pb or pin_state(T, (*T.PIN_START[1], T._wz(T.PIN_HOLE_FLOOR + T.PIN_L))),
    }
    return s


def hung(T, open_deg=90.0, b=None, dx=0.0, dy=0.0):
    b = T.B_REST if b is None else b
    return door_state(T, (T.X_A + dx, T.Y_A + dy, T._wz(b)), open_deg)


def arms(T, open_=True, far=True):
    gap = 0.09 if open_ else 0.0
    out = {}
    for arm, c in (("left", (0.20, 0.25, T._wz(0.25))), ("right", (0.12, -0.25, T._wz(0.25)))):
        if not far:
            c = (T.X_A - 0.02, T.Y_A + 0.03, T._wz(0.26))
        rot = T.grip_rotation(0.0)
        out[arm] = {"finger_gap": gap, "center": np.array(c), "pad_points": T.pad_probe_points(np.array(c), gap, rot)}
    return out


def run_steps(T, st, a, n=1):
    v = None
    for _ in range(n):
        T.tick()
        T.set_arms(a)
        v = T.score(st)
    return v


def test_scoring(T) -> int:
    cases = 0

    def check(name, st, want, a=None, n=1, reset=True):
        nonlocal cases
        if reset:
            T._reset_episode()
        got = run_steps(T, st, a if a is not None else arms(T), n)
        assert abs(got - want) < 1e-9, (name, got, want, sorted(T._S["violations"]))
        cases += 1

    up, lo = seat(T, 1), seat(T, 0)
    check("start", scene(T), 0.0)
    check("door hung, no pins", scene(T, door=hung(T)), 0.0)
    check("door shut, no pins (propped)", scene(T, door=hung(T, 0.0)), 0.0)
    check("upper pinned", scene(T, door=hung(T), pa=pin_state(T, up)), 0.25)
    check("lower pinned", scene(T, door=hung(T), pb=pin_state(T, lo)), 0.25)
    check("pin_b in upper hinge", scene(T, door=hung(T), pb=pin_state(T, up)), 0.25)
    check("both pinned, open", scene(T, door=hung(T), pa=pin_state(T, up), pb=pin_state(T, lo)), 0.5)
    check("both pinned, at 30 deg", scene(T, door=hung(T, 30.0), pa=pin_state(T, up), pb=pin_state(T, lo)), 0.5)
    both_shut = scene(T, door=hung(T, 0.0), pa=pin_state(T, up), pb=pin_state(T, lo))
    check("both pinned, shut, hands near", both_shut, 0.75, a=arms(T, far=False))
    check("both pinned, shut, fingers closed far", both_shut, 0.75, a=arms(T, open_=False), n=40)
    check("both pinned, shut, clear 19 steps", both_shut, 0.75, n=19)
    check("both pinned, shut, clear 20 steps", both_shut, 1.0, n=20)
    T._reset_episode()
    run_steps(T, both_shut, arms(T), 25)
    assert T.success(both_shut)
    cases += 1
    check("clear hold broken resets the count", both_shut, 0.75, a=arms(T, far=False), reset=False)
    check("clear count restarts", both_shut, 0.75, n=10, reset=False)
    check("arms unknown never clear", both_shut, 0.75, a={}, n=30)
    # pins not home
    check("pin 2 mm high", scene(T, door=hung(T), pa=pin_state(T, up + (0, 0, 0.002))), 0.0)
    check("pin 0.8 mm high ok", scene(T, door=hung(T), pa=pin_state(T, up + (0, 0, 0.0008))), 0.25)
    check("pin upside down", scene(T, door=hung(T), pa=pin_state(T, up - (0, 0, T.PIN_L), flip=True)), 0.0)
    check("pin tilted 5 deg", scene(T, door=hung(T), pa=pin_state(T, up, tilt_deg=5.0)), 0.0)
    check("pin beside hinge", scene(T, door=hung(T), pa=pin_state(T, up + (0.003, 0, 0))), 0.0)
    check("pins left in the block, door shut", scene(T, door=hung(T, 0.0)), 0.0)
    # door knuckle missing from the stack
    check("pins home, door still in the rest", scene(T, pa=pin_state(T, up), pb=pin_state(T, lo)), 0.0)
    check("pins home, door knuckle 2 mm off axis", scene(T, door=hung(T, dx=-0.002), pa=pin_state(T, up), pb=pin_state(T, lo)), 0.0)
    check("pins home, door lifted out of the gaps", scene(T, door=hung(T, b=T.B_REST + 0.006), pa=pin_state(T, up), pb=pin_state(T, lo)), 0.0)
    check("pins home, door left 2.5 mm low on the block", scene(T, door=door_state(T, (T.X_A, T.Y_A, T._wz(T.BLOCK_H)), 90.0), pa=pin_state(T, up), pb=pin_state(T, lo)), 0.0)
    check("pins home, door tilted 8 deg", scene(T, door=door_state(T, (T.X_A, T.Y_A, T._wz(T.B_REST)), 90.0, 8.0), pa=pin_state(T, up), pb=pin_state(T, lo)), 0.0)
    # knuckle play band: door bottom from B_REST (on the lower fixed knuckles) to B_NOM + play/2 (against the upper ones)
    play = (2 * T.FK_IN - T.DK_LEN) / 2
    pins_home = {"pa": pin_state(T, up), "pb": pin_state(T, lo)}
    b_top = T.B_NOM + play
    for label, b in (("rest", T.B_REST), ("centred", T.B_NOM), ("top of play", b_top), ("0.4 mm low", T.B_REST - 0.0004), ("0.4 mm high", b_top + 0.0004)):
        check(f"band {label}, open: both pinned", scene(T, door=hung(T, b=b), **pins_home), 0.5)
        check(f"band {label}, shut: both pinned and closed", scene(T, door=hung(T, 0.0, b=b), **pins_home), 0.75)
        check(f"band {label}, shut, clear 20 steps: full score", scene(T, door=hung(T, 0.0, b=b), **pins_home), 1.0, n=20)
        check(f"band {label}: one pin only", scene(T, door=hung(T, b=b), pa=pin_state(T, up)), 0.25)
    # overlap beyond the 0.5 mm tolerance in the fixed knuckles
    for label, b in (("1.2 mm into the lower knuckle", T.B_REST - 0.0012), ("2 mm above nominal", T.B_NOM + 0.0020), ("0.6 mm past the play", b_top + 0.0006 + 0.0005)):
        check(f"overlap {label}: pins home", scene(T, door=hung(T, b=b), **pins_home), 0.0)
        check(f"overlap {label}: shut, pins home", scene(T, door=hung(T, 0.0, b=b), **pins_home), 0.0)
        check(f"overlap {label}: shut, clear 30 steps", scene(T, door=hung(T, 0.0, b=b), **pins_home), 0.0, n=30)
    # door still on the support block, pins pushed home
    on_block = scene(T, door=door_state(T, (T.X_A, T.Y_A, T._wz(T.BLOCK_H)), 90.0), **pins_home)
    check("door on the block, pins home", on_block, 0.0)
    check("door on the block, pins home, clear 30 steps", on_block, 0.0, n=30)
    assert not T.door_supported_by_hinges_only(on_block)
    cases += 1
    shut_on_block_height = scene(T, door=hung(T, 0.0, b=T.BLOCK_H), **pins_home)
    check("door shut at block height, pins home", shut_on_block_height, 0.0, n=30)
    # door at table height or on the block cannot reach full score
    check("door shut on the table, pins home", scene(T, door=hung(T, 0.0, b=0.0), **pins_home), 0.0, n=30)
    assert not T.door_supported_by_hinges_only(scene(T, door=hung(T, 0.0, b=0.0), **pins_home))
    cases += 1
    check("door shut 1 mm off the table, pins home", scene(T, door=hung(T, 0.0, b=0.001), **pins_home), 0.0, n=30)
    check("door open on the block start pose, pins home", scene(T, door=hung(T, 90.0, b=T.BLOCK_H), **pins_home), 0.0, n=30)
    # a door hung correctly is supported by the hinges only, at both ends of the play band
    for b in (T.B_REST, T.B_NOM, b_top):
        assert T.door_supported_by_hinges_only(scene(T, door=hung(T, 0.0, b=b), **pins_home))
        assert T.door_supported_by_hinges_only(scene(T, door=hung(T, 90.0, b=b), **pins_home))
        cases += 2
    # door knuckle off axis while the pins are home
    for label, kw in (("2 mm in x", {"dx": -0.002}), ("2 mm in x outward", {"dx": 0.002}), ("2 mm in y", {"dy": 0.002}), ("2 mm in -y", {"dy": -0.002})):
        check(f"pins home, door knuckle {label}", scene(T, door=hung(T, 0.0, **kw), **pins_home), 0.0, n=30)
    check("pins home, door knuckle 0.8 mm off axis", scene(T, door=hung(T, 0.0, dx=-0.0008), **pins_home), 0.75)
    # closing gap
    for open_deg, want in ((0.5, 0.75), (1.5, 0.5), (3.0, 0.5)):
        gap = abs(T.DOOR_YE - T.W) * math.sin(math.radians(open_deg))
        check(f"door open {open_deg} deg (gap {gap * 1000:.1f} mm)", scene(T, door=hung(T, open_deg), pa=pin_state(T, up), pb=pin_state(T, lo)), want)
    # the reference full-score pose hangs the door with its knuckles resting on the lower fixed knuckles (bottom = B_REST)
    lo1 = T._wz(T.fixed_knuckles(0)[0][1])
    assert abs(both_shut["door"].pos[2] - T._wz(T.B_REST)) < 1e-12
    assert abs(T.door_knuckle_centre(both_shut["door"], 0)[2] - T.DK_LEN / 2 - lo1) < 1e-12
    cases += 2
    check("hung on the lower knuckles (B_REST), shut, clear 20 steps", both_shut, 1.0, n=20)
    check("held 0.75 mm high (top of the play), shut", scene(T, door=hung(T, 0.0, b=b_top), **pins_home), 0.75)
    check("held 0.75 mm high (top of the play), shut, clear 20 steps", scene(T, door=hung(T, 0.0, b=b_top), **pins_home), 1.0, n=20)
    # knuckle overlap: 0.45 mm is inside the 0.5 mm tolerance, 0.55 mm and 1.3 mm are not
    for label, b, want in (("0.45 mm into the lower knuckle", T.B_REST - 0.00045, 1.0), ("0.45 mm into the upper knuckle", b_top + 0.00045, 1.0),
                           ("0.55 mm into the lower knuckle", T.B_REST - 0.00055, 0.0), ("0.55 mm into the upper knuckle", b_top + 0.00055, 0.0),
                           ("1.3 mm too low", T.B_REST - 0.0013, 0.0), ("1.3 mm too high", b_top + 0.0013, 0.0)):
        check(f"door {label}, pins home, shut, clear 20 steps", scene(T, door=hung(T, 0.0, b=b), **pins_home), want, n=20)
        check(f"door {label}, pins home, open", scene(T, door=hung(T, b=b), **pins_home), 0.5 if want else 0.0)
    # door on the block: its knuckles are 1.75 mm into the lower fixed knuckles, so neither hinge counts
    assert not any(T.door_knuckle_engaged(on_block, i) for i in range(2))
    cases += 1
    # hand still on the hung, shut door: fingers pinching the top edge, open fingers on the edge, open hand 30 mm above
    grip_tcp = T.grip_for_door((T.X_A, T.Y_A, T._wz(T.B_REST)), 90.0)

    def right_at(tcp, gap, rot=None):
        a = arms(T)
        rot = T.right_rot(90.0) if rot is None else rot
        a["right"] = {"finger_gap": gap, "center": np.asarray(tcp, float), "pad_points": T.pad_probe_points(np.asarray(tcp, float), gap, rot)}
        return a

    check("hung and shut, right fingers pinching the top edge", both_shut, 0.75, a=right_at(grip_tcp, 0.0136), n=40)
    check("hung and shut, right fingers open on the top edge", both_shut, 0.75, a=right_at(grip_tcp, 0.09), n=40)
    check("hung and shut, open right hand 30 mm above the edge", both_shut, 0.75, a=right_at(grip_tcp + (0, 0, 0.030), 0.09), n=40)
    check("hung and shut, open right hand 100 mm above the edge", both_shut, 1.0, a=right_at(grip_tcp + (0, 0, 0.100), 0.09), n=20)
    a_pin = arms(T)
    head_c = up + (0, 0, T.HEAD_T / 2)
    a_pin["left"] = {"finger_gap": 0.011, "center": head_c, "pad_points": T.pad_probe_points(head_c, 0.011, T.left_rot())}
    check("hung and shut, left fingers still on the upper pin head", both_shut, 0.75, a=a_pin, n=40)
    # door over the block when open: hung at B_REST it is 1.75 mm above the block top, outside the 1 mm inflation
    for b in (T.B_REST, T.B_REST - 0.00045):
        hung_open = scene(T, door=hung(T, 90.0, b=b), **pins_home)
        assert T.door_supported_by_hinges_only(hung_open)
        cases += 1
        check(f"hung open over the block, bottom {(b - T.BLOCK_H) * 1000:.2f} mm above it, clear 30 steps", hung_open, 0.5, n=30)
    for deg in (90.0, 30.0, 0.0):
        near_block = scene(T, door=hung(T, deg, b=T.BLOCK_H + 0.0009), **pins_home)
        check(f"door 0.9 mm above the block top at {deg:.0f} deg open, pins home, clear 30 steps", near_block, 0.0, n=30)
    assert not T.door_supported_by_hinges_only(scene(T, door=hung(T, 90.0, b=T.BLOCK_H + 0.0009), **pins_home))
    cases += 1
    # one pin home, the other stopped high in the other hinge (on the door knuckle top: head 35.5 mm up; or 30 mm up)
    on_dk = T.door_knuckle_centre(hung(T), 0)[2] + T.DK_LEN / 2 + T.PIN_L
    assert abs(on_dk - lo[2] - 0.0355) < 1e-9
    cases += 1
    for label, pa, pb in (("upper home, lower pin on the door knuckle", pin_state(T, up), pin_state(T, (T.X_A, T.Y_A, on_dk))),
                          ("upper home, lower pin head 30 mm high", pin_state(T, up), pin_state(T, lo + (0, 0, 0.030))),
                          ("lower home, upper pin head 30 mm high", pin_state(T, up + (0, 0, 0.030)), pin_state(T, lo))):
        check(f"{label}, open", scene(T, door=hung(T), pa=pa, pb=pb), 0.25)
        check(f"{label}, shut, clear 30 steps", scene(T, door=hung(T, 0.0), pa=pa, pb=pb), 0.25, n=30)
    # supported only by the hinges
    low_door = scene(T, door=hung(T, 0.0, b=0.0015), pa=pin_state(T, up), pb=pin_state(T, lo))
    assert not T.door_supported_by_hinges_only(low_door)
    assert T.door_supported_by_hinges_only(both_shut)
    assert not T.door_supported_by_hinges_only(scene(T))
    cases += 3
    # latched violations
    tipped = scene(T, door=door_state(T, (0.15, 0.0, T._wz(0.01)), 90.0, 80.0))
    check("door dropped flat", tipped, 0.0)
    check("dropped door stays 0 after a perfect state", both_shut, 0.0, n=30, reset=False)
    assert "door_dropped" in T._S["violations"]
    cases += 1
    check("pin off the bench", scene(T, door=hung(T, 0.0), pa=pin_state(T, (0.3, 0.4, T.TOP - 0.2)), pb=pin_state(T, lo)), 0.0)
    check("pin off latch holds", both_shut, 0.0, n=25, reset=False)
    T._on_reset()
    assert not T._S["violations"] and T._S["clear_steps"] == 0
    check("after reset", both_shut, 1.0, n=20)
    # success and score agree within one step (memoised per step)
    T._reset_episode()
    for k in range(25):
        T.tick()
        T.set_arms(arms(T))
        s_ok = T.success(both_shut)
        sc = T.score(both_shut)
        assert s_ok == (sc >= 1.0), (k, s_ok, sc)
    cases += 1
    assert 0.0 < T.distance(scene(T)) and T.distance(both_shut) < 1e-9
    cases += 1
    T._reset_episode()
    return cases


# - collider geometry --------------------------------------------------------------------------------------------------
def prims(T, spec, origin, rot):
    out = []
    from mechkit import _quat_wxyz_matrix

    for s in spec.shapes:
        if not s.collide:
            continue
        r = rot @ _quat_wxyz_matrix(s.tilt)
        out.append({"kind": s.kind, "c": rot @ np.array(s.at) + np.asarray(origin, float), "rot": r, "size": np.array(s.size), "body": spec.name})
    return out


def sdf(pr, pts):
    loc = (pts - pr["c"]) @ pr["rot"]
    if pr["kind"] == "box":
        q = np.abs(loc) - pr["size"] / 2
        return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)
    if pr["kind"] == "cyl":
        dr = np.hypot(loc[:, 0], loc[:, 1]) - pr["size"][0]
        dz = np.abs(loc[:, 2]) - pr["size"][2] / 2
        return np.hypot(np.maximum(dr, 0), np.maximum(dz, 0)) + np.minimum(np.maximum(dr, dz), 0)
    return np.linalg.norm(loc, axis=1) - pr["size"][0]


def surface(pr, step=0.001):
    k = pr["kind"]
    if k == "box":
        hx, hy, hz = pr["size"] / 2
        pts = []
        for ax in range(3):
            o = [a for a in range(3) if a != ax]
            h = np.array((hx, hy, hz))
            n0, n1 = max(2, int(2 * h[o[0]] / step) + 1), max(2, int(2 * h[o[1]] / step) + 1)
            n0, n1 = min(n0, 160), min(n1, 160)
            u, v = np.meshgrid(np.linspace(-h[o[0]], h[o[0]], n0), np.linspace(-h[o[1]], h[o[1]], n1))
            for sgn in (-1, 1):
                p = np.zeros((u.size, 3))
                p[:, o[0]], p[:, o[1]], p[:, ax] = u.ravel(), v.ravel(), sgn * h[ax]
                pts.append(p)
        loc = np.vstack(pts)
    elif k == "cyl":
        r, hh = pr["size"][0], pr["size"][2] / 2
        th = np.linspace(0, 2 * np.pi, 64, endpoint=False)
        zz = np.linspace(-hh, hh, max(2, int(2 * hh / step) + 1))
        t2, z2 = np.meshgrid(th, zz)
        side = np.c_[r * np.cos(t2.ravel()), r * np.sin(t2.ravel()), z2.ravel()]
        rr = np.linspace(0, r, 5)
        r2, t3 = np.meshgrid(rr, th)
        cap = np.c_[r2.ravel() * np.cos(t3.ravel()), r2.ravel() * np.sin(t3.ravel())]
        loc = np.vstack([side, np.c_[cap, np.full(len(cap), hh)], np.c_[cap, np.full(len(cap), -hh)]])
    else:
        n = 300
        i = np.arange(n) + 0.5
        phi, th = np.arccos(1 - 2 * i / n), np.pi * (1 + 5**0.5) * i
        loc = pr["size"][0] * np.c_[np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)]
    return loc @ pr["rot"].T + pr["c"]


def aabb(pr):
    if pr["kind"] == "box":
        half = np.abs(pr["rot"]) @ (pr["size"] / 2)
    elif pr["kind"] == "cyl":
        half = np.abs(pr["rot"]) @ np.array((pr["size"][0], pr["size"][0], pr["size"][2] / 2))
    else:
        half = np.full(3, pr["size"][0])
    return pr["c"] - half, pr["c"] + half


def overlaps(groups, tol):
    """Deepest interpenetration between primitives of different groups, worse than tol (m)."""
    bad = []
    flat = [(g, p, *aabb(p)) for g, ps in groups.items() for p in ps]
    for i in range(len(flat)):
        gi, pi, lo_i, hi_i = flat[i]
        for j in range(i + 1, len(flat)):
            gj, pj, lo_j, hi_j = flat[j]
            if gi == gj or np.any(hi_i < lo_j) or np.any(hi_j < lo_i):
                continue
            d = min(float(sdf(pj, surface(pi)).min()), float(sdf(pi, surface(pj)).min()))
            if d < -tol:
                bad.append((gi, gj, round(d * 1000, 3)))
    return bad


def below_table(T, groups):
    out = []
    for g, ps in groups.items():
        for p in ps:
            if aabb(p)[0][2] < T.TOP - 1e-6:
                out.append(g)
    return out


def scene_prims(T, door_o, door_open, pin_heads):
    g = {
        "enclosure": prims(T, T.ENCLOSURE, T.ENCLOSURE.pos, np.eye(3)),
        "door_rest": prims(T, T.DOOR_REST, T.DOOR_REST.pos, np.eye(3)),
        "pin_block": prims(T, T.PIN_BLOCK, T.PIN_BLOCK.pos, np.eye(3)),
        "door": prims(T, T.DOOR, door_o, T._rz(-door_open)),
    }
    for p, h in zip(T.PINS, pin_heads):
        g[p.name] = prims(T, p, h, np.eye(3))
    return g


def test_geometry(T) -> int:
    cases = 0
    tol = 0.0002
    start_pins = [np.array((x, y, T._wz(T.PIN_HOLE_FLOOR + T.PIN_L))) for x, y in T.PIN_START]
    o_start = np.array((T.DOOR_START[0], T.DOOR_START[1], T._wz(T.BLOCK_H)))
    g = scene_prims(T, o_start, 90.0, start_pins)
    bad = overlaps(g, tol)
    assert not bad, ("start", bad)
    assert not below_table(T, g), below_table(T, g)
    cases += 2
    seats = [seat(T, 1), seat(T, 0)]
    for b in (T.B_NOM, T.B_NOM + 0.0007, T.B_NOM - 0.0007):
        bad = overlaps(scene_prims(T, np.array((T.X_A, T.Y_A, T._wz(b))), 90.0, start_pins), tol)
        assert not bad, ("hung without pins", b, bad)
        cases += 1
    for a in list(range(90, -1, -5)) + [2, 1]:
        bad = overlaps(scene_prims(T, np.array((T.X_A, T.Y_A, T._wz(T.B_NOM))), float(a), seats), tol)
        assert not bad, ("pinned door on the block at open angle", a, bad)
        cases += 1
    for a in list(range(10, -1, -5)) + [2, 1]:
        bad = overlaps(scene_prims(T, np.array((T.X_A, T.Y_A, T._wz(T.B_REST))), float(a), seats), tol)
        assert not bad, ("pinned door on the knuckles at open angle", a, bad)
        cases += 1
    # the door slides in 2 mm off its nominal height either way and still clears the fixed knuckles
    for dz in (0.0, 0.0007, -0.0007):
        for dx in (0.040, 0.020, 0.010, 0.0):
            bad = overlaps(scene_prims(T, np.array((T.X_A - dx, T.Y_A, T._wz(T.B_NOM + dz))), 90.0, start_pins), tol)
            assert not bad, ("slide in", dz, dx, bad)
            cases += 1
    # the checker itself catches a pin 0.5 mm off the bore axis and a door 1 mm into the body
    bad = overlaps(scene_prims(T, np.array((T.X_A, T.Y_A, T._wz(T.B_NOM))), 90.0, [seats[0] + (0.0005, 0, 0), seats[1]]), tol)
    assert bad
    bad = overlaps(scene_prims(T, np.array((T.X_A + 0.001, T.Y_A, T._wz(T.B_REST))), 0.0, seats), tol)
    assert bad
    cases += 2
    return cases


# - demo replay --------------------------------------------------------------------------------------------------------
def _stl_vertices(path: pathlib.Path) -> np.ndarray:
    raw = path.read_bytes()
    n = int.from_bytes(raw[80:84], "little")
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    arr = np.frombuffer(raw, dtype=dt, offset=84, count=n)
    return np.unique(arr["v"].reshape(-1, 3).astype(float), axis=0)


def link_clouds():
    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    root = ET.parse(YAM_URDF).getroot()
    out = {}
    for link in root.iter("link"):
        name = link.get("name")
        if name == "base":
            continue
        pts = []
        for col in link.findall("collision"):
            o = col.find("origin")
            xyz = np.array([float(v) for v in o.get("xyz").split()]) if o is not None and o.get("xyz") else np.zeros(3)
            rpy = np.array([float(v) for v in o.get("rpy").split()]) if o is not None and o.get("rpy") else np.zeros(3)
            pts.append(_stl_vertices(pathlib.Path(YAM_URDF).parent / col.find("geometry/mesh").get("filename")) @ rpy_matrix(rpy).T + xyz)
        if pts:
            out[name] = np.vstack(pts)
    return out


ON_BLOCK_LABELS = {"R grip door", "R hold", "R slide door along the block"}
CARRY_LABELS = {"R lift door off the block", "R slide knuckles in", "R settle door", "R hold door"}
R_CONTACT = {"R over door edge", "R to door edge", "R settle", "R grip door", "R hold", "R press door on block",
             "R slide door along the block", "R lift door off the block", "R slide knuckles in", "R settle door", "R hold door", "R unload door onto the knuckles", "R release door", "R to door face", "R hold shut", "R back off straight", "R back off"}


def test_demo(T) -> tuple[int, dict]:
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS
    from yam_arena.demo import ArmPlan
    from yam_arena.urdf import forward_kinematics
    from yam_arena import YAM_URDF

    hz = 20.0
    wps = T.demo()
    plans = {arm: ArmPlan(list(w), station.BASES[arm]) for arm, w in wps.items()}
    dur = {arm: p.duration for arm, p in plans.items()}
    assert abs(dur["left"] - dur["right"]) < 0.11, dur
    n_steps = round(max(dur.values()) * hz)
    assert n_steps <= 3000, n_steps
    clouds = link_clouds()
    tips = {"tip_left", "tip_right"}
    r0 = T.right_rot(0.0)
    hold_from = next(i for i, w in enumerate(wps["right"]) if w.label == "R grip door")
    hold_to = max(i for i, w in enumerate(wps["right"]) if w.label.startswith("R swing"))
    right_labels = [w.label for w in wps["right"]]
    left_labels = [w.label for w in wps["left"]]
    held = [(left_labels.index(f"L lift pin {n}"), left_labels.index(f"L drop pin {n}")) for n in range(2)]
    pushed = [left_labels.index(f"L push pin {n} home") for n in range(2)]
    door = {"o": np.array((T.DOOR_START[0], T.DOOR_START[1], T._wz(T.BLOCK_H))), "open": 90.0}
    pin_heads = [np.array((x, y, T._wz(T.PIN_HOLE_FLOOR + T.PIN_L))) for x, y in T.PIN_START]
    pin_home = [seat(T, 1), seat(T, 0)]
    static = {
        "enclosure": prims(T, T.ENCLOSURE, T.ENCLOSURE.pos, np.eye(3)),
        "door_rest": prims(T, T.DOOR_REST, T.DOOR_REST.pos, np.eye(3)),
        "pin_block": prims(T, T.PIN_BLOCK, T.PIN_BLOCK.pos, np.eye(3)),
    }
    min_clear: dict = {}
    worst_q = {}

    def idx_at(plan, t):
        acc = 0.0
        for i, w in enumerate(plan.waypoints):
            acc += w.duration
            if t < acc:
                return i
        return len(plan.waypoints) - 1

    for k in range(n_steps + 5):
        t = (k + 1) / hz
        q_arm, tcp = {}, {}
        for arm, plan in plans.items():
            act, lbl = plan.action7(t)
            pos, rot, finger, _ = plan.target(t)
            assert np.all(act[:6] >= ik.LOWER - 1e-9) and np.all(act[:6] <= ik.UPPER + 1e-9), (arm, t, lbl)
            q_arm[arm] = (act[:6], float(finger), lbl)
            tcp[arm] = (pos, rot)
        # door pose follows the right pinch while held, then the right fingertips while pushing
        ri = idx_at(plans["right"], t)
        pos, rot = tcp["right"]
        if hold_from < ri <= hold_to:
            phi = math.degrees(math.atan2((rot @ r0.T)[1, 0], (rot @ r0.T)[0, 0]))
            door["open"] = 90.0 - phi
            carried = right_labels[ri] in CARRY_LABELS
            door["o"] = T.door_origin_for_grip(pos - (T.LOAD_COMP if carried else 0.0), phi)
            door["o"][2] = max(door["o"][2], T._wz(T.B_REST))
            if right_labels[ri] in ON_BLOCK_LABELS:
                door["o"][2] = T._wz(T.BLOCK_H)
        elif right_labels[ri].startswith("R push") or right_labels[ri] == "R hold shut":
            tip = pos + T.PAD_TIP_PAST_TCP * rot[:, 2]
            v = tip[:2] - np.array((T.X_A, T.Y_A))
            ang = math.degrees(math.atan2(v[1], v[0]) - math.atan2(-T.PUSH_R, 0.0035))
            door["open"] = float(np.clip(ang, 0.0, door["open"]))
        lpos, lrot = tcp["left"]
        li = idx_at(plans["left"], t)
        for n in range(2):
            if held[n][0] <= li <= held[n][1]:
                pin_heads[n] = lpos - T.tcp_for_head(np.zeros(3), lrot) - np.array((0, 0, T.HEAD_T / 2))
            elif held[n][1] < li < pushed[n]:
                pin_heads[n] = pin_home[n] + np.array((0, 0, T.PIN_RELEASE_H))
            elif li >= pushed[n]:
                pin_heads[n] = pin_home[n].copy()
        groups = dict(static)
        groups["door"] = prims(T, T.DOOR, door["o"], T._rz(-door["open"]))
        for p, h in zip(T.PINS, pin_heads):
            groups[p.name] = prims(T, p, h, np.eye(3))
        for arm in ("left", "right"):
            qa, finger, lbl = q_arm[arm]
            hold_w = 0.0
            if arm == "right" and ("door" in lbl or "R swing" in lbl or lbl == "R hold") and finger > -0.006:
                hold_w = 0.0136
            if arm == "left" and finger > -0.004 and held[0][0] - 1 <= idx_at(plans["left"], t) <= held[1][1] + 1 and not lbl.startswith("L close") and "push" not in lbl and "off hinge" not in lbl:
                hold_w = 0.011
            fq = -max(abs(finger), hold_w / 2)
            qm = {n: float(v) for n, v in zip(ARM_JOINTS, qa)}
            qm["joint7"] = qm["joint8"] = fq
            fk = forward_kinematics(YAM_URDF, qm)
            base = np.asarray(station.BASES[arm], float)
            for lname, verts in clouds.items():
                tm = fk[lname]
                pts = verts @ tm[:3, :3].T + tm[:3, 3] + base
                lo, hi = pts.min(0) - 0.01, pts.max(0) + 0.01
                for gname, ps in groups.items():
                    r_contact = lbl in R_CONTACT or lbl.startswith(("R swing", "R push"))
                    if lname in tips and ((arm == "right" and gname == "door" and r_contact) or (arm == "left" and gname.startswith("pin_"))):
                        continue
                    for pr in ps:
                        plo, phi_ = aabb(pr)
                        if np.any(hi < plo) or np.any(phi_ < lo):
                            continue
                        d = float(sdf(pr, pts).min())
                        key = (arm, lname, gname)
                        if d < min_clear.get(key, (1.0,))[0]:
                            min_clear[key] = (d, round(t, 2), lbl + f" prim@{np.round((pr['c'] - [0, 0, T.TOP]) * 1000, 1).tolist()} size={np.round(pr['size'] * 1000, 1).tolist()}")
                        assert d > -0.0002, (arm, round(t, 2), lbl, lname, gname, round(d * 1000, 2))
    for arm, plan in plans.items():
        worst_q[arm] = (plan.worst_ik_mm, plan.worst_ik_deg)
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, worst_q[arm])
    return n_steps, {"min_clear": min_clear, "ik": worst_q, "dur": dur}


def test_text(T) -> int:
    n = 0
    tj = HERE / "task.json"
    if tj.exists():
        assert json.loads(tj.read_text())["instruction"] == T.INSTRUCTION
        n += 1
    rd = HERE / "README.md"
    if rd.exists():
        assert T.INSTRUCTION in rd.read_text().replace("\n", " ")
        n += 1
    for p in HERE.rglob("*"):
        if p.suffix in (".py", ".md", ".json", ".txt") and p.is_file():
            txt = p.read_text(errors="ignore")
            assert chr(0x2013) not in txt and chr(0x2014) not in txt, p
    return n


def run_all(T) -> None:
    n = test_scoring(T)
    print(f"scoring: {n} cases pass")
    g = test_geometry(T)
    print(f"geometry: {g} poses, no interpenetration > 0.2 mm between separate bodies, nothing below the table")
    steps, info = test_demo(T)
    tight = sorted(((round(v[0] * 1000, 1), k, v[1], v[2]) for k, v in info["min_clear"].items() if v[0] < 0.004))
    print(f"demo: {steps} steps ({info['dur']['left']:.1f} s), IK worst left {info['ik']['left'][0]:.3f} mm right {info['ik']['right'][0]:.3f} mm, no arm link penetrates the scene")
    for d, k, t, lbl in tight:
        print(f"  tight clearance {d} mm: {k} at {t} s ({lbl})")
    print(f"text: {test_text(T)} files consistent")
