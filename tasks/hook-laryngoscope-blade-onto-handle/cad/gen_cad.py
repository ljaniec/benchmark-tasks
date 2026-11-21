"""Writes the printable closed cradle (far-side rail, blade-side short rear rail, rear end stop, front post, and all of them placed together) as ASCII STL, numpy only.
The parts locate the handle in -x (end stop), +x (front post, against the head cheeks) and across the barrel (the two rails, so it cannot turn); they are the same boxes as in the simulation. The handle and blade are off-the-shelf parts (see BUILD.md).

Run:  python gen_cad.py
Units are millimetres. The assembly file uses a frame with its origin on the hinge pin axis at tray floor level,
x toward the handle head, y away from the blade side (the pin to barrel axis offset is +16.5 mm along y), z up.
"""

import pathlib

import numpy as np

BARREL_D = 18.3
BARREL_R = BARREL_D / 2
BARREL_Y = 16.5  # barrel axis from the pin axis
RAIL_T, RAIL_H = 5.0, 9.0
RAIL_Y = 13.65  # rail centre from the barrel axis, as in the simulation (rail inner face 2 mm off the barrel)
RAIL_LEN = 108.0
NEAR = (-139.0, -25.0)  # blade-side rail, x extent in the pin frame (114 mm), clear of the blade's swing from ~30 deg
STOP_SIZE = (5.0, 2 * RAIL_Y, 9.0)  # 5 x 27.3 x 9 mm
BARREL_X = (-140.0, -12.0)  # barrel extent along x from the pin axis
FRONT = ((6.5, 15.0, 0.0), (9.5, 24.0, 12.0))  # front post: x 6.5-9.5, y 15-24, 12 mm high, in the pin frame (touches the head cheeks at y 15-18)


def box_triangles(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = [np.array([x, y, z]) for z in (lo[2], hi[2]) for y in (lo[1], hi[1]) for x in (lo[0], hi[0])]
    faces = [(0, 2, 1), (1, 2, 3), (4, 5, 6), (5, 7, 6), (0, 1, 4), (1, 5, 4), (2, 6, 3), (3, 6, 7), (0, 4, 2),
             (2, 4, 6), (1, 3, 5), (3, 7, 5)]
    return [(c[a], c[b], c[d]) for a, b, d in faces]


def cylinder_triangles(centre, radius, height, segments=48):
    cx, cy, z0 = centre
    ang = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    ring = [(cx + radius * np.cos(a), cy + radius * np.sin(a)) for a in ang]
    tris = []
    for i in range(segments):
        (x0, y0), (x1, y1) = ring[i], ring[(i + 1) % segments]
        b0, b1 = np.array([x0, y0, z0]), np.array([x1, y1, z0])
        t0, t1 = np.array([x0, y0, z0 + height]), np.array([x1, y1, z0 + height])
        tris += [(b0, b1, t1), (b0, t1, t0), (np.array([cx, cy, z0]), b1, b0),
                 (np.array([cx, cy, z0 + height]), t0, t1)]
    return tris


def write_stl(path, name, tris):
    lines = [f"solid {name}"]
    for a, b, c in tris:
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        n = n / norm if norm > 0 else n
        lines.append(f"facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n outer loop")
        for v in (a, b, c):
            lines.append(f"  vertex {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}")
        lines.append(" endloop\nendfacet")
    lines.append(f"endsolid {name}")
    pathlib.Path(path).write_text("\n".join(lines) + "\n")


def main():
    out = pathlib.Path(__file__).resolve().parent
    rail_local = box_triangles((0, 0, 0), (RAIL_LEN, RAIL_T, RAIL_H))
    stop_local = box_triangles((0, 0, 0), STOP_SIZE)
    write_stl(out / "cradle_rail.stl", "cradle_rail", rail_local)
    write_stl(out / "cradle_rail_near.stl", "cradle_rail_near", box_triangles((0, 0, 0), (NEAR[1] - NEAR[0], RAIL_T, RAIL_H)))
    write_stl(out / "cradle_stop.stl", "cradle_stop", stop_local)
    write_stl(out / "cradle_front_post.stl", "cradle_front_post", box_triangles((0, 0, 0), np.array(FRONT[1]) - np.array(FRONT[0])))

    x_mid = sum(BARREL_X) / 2
    rail_y = BARREL_Y + RAIL_Y
    rail = box_triangles((x_mid - RAIL_LEN / 2, rail_y - RAIL_T / 2, 0), (x_mid + RAIL_LEN / 2, rail_y + RAIL_T / 2, RAIL_H))
    near_y = BARREL_Y - RAIL_Y
    near = box_triangles((NEAR[0], near_y - RAIL_T / 2, 0), (NEAR[1], near_y + RAIL_T / 2, RAIL_H))
    stop_x = BARREL_X[0] - 6.0
    stop = box_triangles((stop_x - STOP_SIZE[0] / 2, BARREL_Y - RAIL_Y, 0),
                         (stop_x + STOP_SIZE[0] / 2, BARREL_Y + RAIL_Y, STOP_SIZE[2]))
    front = box_triangles(*FRONT)
    write_stl(out / "cradle_assembly.stl", "cradle_assembly", rail + near + stop + front)
    print(f"rail {RAIL_LEN} x {RAIL_T} x {RAIL_H} mm, near rail {NEAR[1] - NEAR[0]} x {RAIL_T} x {RAIL_H} mm, stop {STOP_SIZE[0]} x {STOP_SIZE[1]:.1f} x {STOP_SIZE[2]} mm, front post 3 x 9 x 12 mm")
    print(f"rail centre {RAIL_Y:.2f} mm from the barrel axis; stop centre {BARREL_X[0] - stop_x:.1f} mm behind the barrel end")


if __name__ == "__main__":
    main()
