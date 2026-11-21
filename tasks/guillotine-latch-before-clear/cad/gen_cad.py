#!/usr/bin/env python3
"""Generate printable parts (binary STL, mm) for the guillotine paper trimmer rig. Run: python gen_cad.py"""

from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent

BASE = (355.0, 450.0, 30.0)  # x (depth), y (length), z
BAR = (10.0, 305.0, 20.0)  # blade bar: thickness, length, height
HANDLE = (24.0, 24.0, 40.0)
PLATE = (36.0, 16.0, 5.0)  # slide latch plate: x length, y width, z thickness
KNOB = (12.0, 16.0, 30.0)  # thumb knob on the plate rear
POST = (45.0, 24.0, 50.0)  # guide block under the plate
HOUSING = (40.0, 30.0, 35.0)
RAIL = (12.0, 450.0, 20.0)
STACK = (130.0, 26.0, 70.0)
TRAY, WALL = (220.0, 160.0, 40.0), 4.0


def box(cx, cy, cz, sx, sy, sz):
    h = np.array([sx, sy, sz]) / 2
    c = np.array([cx, cy, cz])
    v = np.array([[i, j, k] for k in (-1, 1) for j in (-1, 1) for i in (-1, 1)]) * h + c
    quads = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    return np.array([[v[a], v[b], v[d]] for a, b, c_, d in quads for a, b, d in ((a, b, c_), (a, c_, d))])


def solid(size):
    return box(0, 0, size[2] / 2, *size)


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


def tray():
    tx, ty, tz = TRAY
    parts = [box(0, 0, WALL / 2, tx, ty, WALL)]
    for s in (1, -1):
        parts.append(box(s * (tx / 2 - WALL / 2), 0, tz / 2, WALL, ty, tz))
        parts.append(box(0, s * (ty / 2 - WALL / 2), tz / 2, tx, WALL, tz))
    return np.concatenate(parts)


def main():
    write_stl("trimmer_base_v1", solid(BASE))
    write_stl("blade_bar_v1", solid(BAR))
    write_stl("handle_block_v1", solid(HANDLE))
    write_stl("latch_plate_v1", solid(PLATE))
    write_stl("latch_knob_v1", solid(KNOB))
    write_stl("latch_guide_v1", solid(POST))
    write_stl("pivot_housing_v1", solid(HOUSING))
    write_stl("rail_v1", solid(RAIL))
    write_stl("card_stack_v1", solid(STACK))
    write_stl("tray_v1", tray())
    print("wrote 10 STL files")


if __name__ == "__main__":
    main()
