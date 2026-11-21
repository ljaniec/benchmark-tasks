#!/usr/bin/env python3
"""Generate the two printable parts (binary STL, mm) for the Weber Q 1000 lid/knob task: the angle card and the red knob cover.
Run: python gen_cad.py"""

from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent

CAP_D, CAP_H = 56.0, 14.0  # knob cover: 56 mm cap, 14 mm deep, front face closed
KNOB_D = 50.0  # inner diameter: measure the stock knob and set this before printing
FRONT_WALL = 3.0
BAR = (40.0, 12.0, 20.0)  # grip bar across the cap (x, tall), width (y), depth along the knob axis (z)
BAR_SUNK = 2.0  # bar root sunk into the cap, so it stands 18 mm proud of the cap front face (as in task.py)
PLATE = (200.0, 307.0, 3.0)  # angle card: 0.20 m wide, 0.307 m tall, 3 mm thick (as in task.py)
RIB, RIB_H, RIB_R = (30.0, 6.0), 1.0, 110.0  # tape-guide ribs: size, height, distance from the hinge-axis mark
PLATE_HINGE = (80.0, -6.5)  # hinge-axis mark in the plate frame (from plate centre): 20 mm from the rear edge, 6.5 mm below centre
AXIS_MARK_R = 3.0


def box(cx, cy, cz, sx, sy, sz):
    h = np.array([sx, sy, sz]) / 2
    c = np.array([cx, cy, cz])
    v = np.array([[i, j, k] for k in (-1, 1) for j in (-1, 1) for i in (-1, 1)]) * h + c
    quads = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    return np.array([[v[a], v[b], v[d]] for a, b, c_, d in quads for a, b, d in ((a, b, c_), (a, c_, d))])


def cylinder(cx, cy, z0, r, h, n=48):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.stack([r * np.cos(t), r * np.sin(t)], 1)
    lo = np.c_[ring, np.zeros(n)]
    hi = np.c_[ring, np.full(n, h)]
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [[lo[i], lo[j], hi[j]], [lo[i], hi[j], hi[i]]]
        tris += [[[0, 0, 0], lo[j], lo[i]], [[0, 0, h], hi[i], hi[j]]]
    return np.array(tris, float) + np.array([cx, cy, z0])


def tube(r_out, r_in, h, n=48):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    tris = []
    for i in range(n):
        j = (i + 1) % n
        a, b = t[i], t[j]
        po = lambda ang, z: [r_out * np.cos(ang), r_out * np.sin(ang), z]
        pi_ = lambda ang, z: [r_in * np.cos(ang), r_in * np.sin(ang), z]
        tris += [[po(a, 0), po(b, 0), po(b, h)], [po(a, 0), po(b, h), po(a, h)]]
        tris += [[pi_(b, 0), pi_(a, 0), pi_(a, h)], [pi_(b, 0), pi_(a, h), pi_(b, h)]]
        tris += [[pi_(a, 0), pi_(b, 0), po(b, 0)], [pi_(a, 0), po(b, 0), po(a, 0)]]
        tris += [[pi_(b, h), pi_(a, h), po(a, h)], [pi_(b, h), po(a, h), po(b, h)]]
    return np.array(tris, float)


def write_stl(name, tris):
    tris = np.asarray(tris, np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tris
    with open(HERE / f"{name}.stl", "wb") as f:
        f.write(name.encode().ljust(80, b" "))
        f.write(np.uint32(len(tris)).tobytes())
        f.write(rec.tobytes())


def main():
    # knob cover, z = knob axis, z=0 open back, z=CAP_H closed front
    cover = np.concatenate([
        tube(CAP_D / 2, KNOB_D / 2, CAP_H - FRONT_WALL),
        cylinder(0, 0, CAP_H - FRONT_WALL, CAP_D / 2, FRONT_WALL),
        box(0, 0, CAP_H - BAR_SUNK + BAR[2] / 2, BAR[0], BAR[1], BAR[2]),
    ])
    write_stl("knob_cover_v1", cover)
    # angle card: u toward the rear of the grill, v up
    plate = [box(0, 0, PLATE[2] / 2, *PLATE), cylinder(PLATE_HINGE[0], PLATE_HINGE[1], PLATE[2], AXIS_MARK_R, RIB_H)]
    for deg in (45.0, 70.0):
        a = np.radians(deg)
        u, v = PLATE_HINGE[0] - RIB_R * np.cos(a), PLATE_HINGE[1] + RIB_R * np.sin(a)
        plate.append(box(u, v, PLATE[2] + RIB_H / 2, RIB[0], RIB[1], RIB_H))
    write_stl("angle_card_v1", np.concatenate(plate))
    print("wrote 2 STL files")


if __name__ == "__main__":
    main()
