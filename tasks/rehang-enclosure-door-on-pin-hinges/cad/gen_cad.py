#!/usr/bin/env python3
"""Generate fixtures and drawings for the enclosure-door rehang setup (ASCII STL, ASCII DXF R12).

All coordinates in mm, table frame: x forward from the arm bases, y left, z up from the table top.
Run from this folder: python gen_cad.py
"""

from __future__ import annotations

import math
from pathlib import Path

HERE = Path(__file__).resolve().parent

# Constants in metres, match task.py
W = 0.2000
H = 0.2500
D = 0.1500
X_F = 0.3066
X_A = X_F - 0.0066
Y_A = 0.0750
Y_C = Y_A - 0.0050 - W / 2
BODY_H0 = 0.015
HINGE_HC = (0.055, 0.225)
FK_LEN = 0.015
FK_IN = 0.008
DK_LEN = 0.0145
PIN_L = 0.052
DOOR_START_X = X_A - 0.040
DOOR_YE = -0.0066
B_NOM = 0.0195
BLOCK_DROP = 0.0025
BLOCK_H = B_NOM - BLOCK_DROP
B_REST = B_NOM - (2 * FK_IN - DK_LEN) / 2
PIN_START = ((0.2200, 0.1950), (0.2200, 0.2250))
PIN_HOLE_FLOOR = 0.010
PIN_BLOCK_H = 0.040
BACKBOARD_HALF_Y = 0.25
BACKBOARD_T = 0.018


def mm(v: float) -> float:
    return round(v * 1000.0, 4)


def box(x0, x1, y0, y1, z0, z1):
    """Axis-aligned box from metre extents, returned in mm."""
    return tuple(mm(v) for v in (x0, x1, y0, y1, z0, z1))


def rest_boxes():
    x0 = DOOR_START_X + DOOR_YE - W
    out = [box(x0 - 0.015, X_A - 0.012, Y_A - 0.0450, Y_A + 0.0180, 0.0, BLOCK_H)]
    for y0, y1 in ((Y_A - 0.0035, Y_A + 0.0165), (Y_A - 0.0401, Y_A - 0.0201)):
        out.append(box(x0 - 0.012, x0 + 0.004, y0, y1, BLOCK_H, BLOCK_H + 0.030))
    return out


def pin_block_extent():
    xs = PIN_START[0][0]
    ya, yb = PIN_START[0][1], PIN_START[1][1]
    return xs - 0.030, xs + 0.030, ya - 0.020, yb + 0.025


def pin_block_boxes():
    xs = PIN_START[0][0]
    ya, yb = PIN_START[0][1], PIN_START[1][1]
    h = 0.003
    x0, x1, y0, y1 = pin_block_extent()
    f, t = PIN_HOLE_FLOOR, PIN_BLOCK_H
    return [
        box(x0, x1, y0, y1, 0.0, f),
        box(x0, xs - h, y0, y1, f, t),
        box(xs + h, x1, y0, y1, f, t),
        box(xs - h, xs + h, y0, ya - h, f, t),
        box(xs - h, xs + h, ya + h, yb - h, f, t),
        box(xs - h, xs + h, yb + h, y1, f, t),
    ]


def spacer_boxes():
    y0, y1 = Y_C - W / 2, Y_C + W / 2
    return [box(X_F + 0.005, X_F + D, y0, y1, 0.0, BODY_H0)]


def _box_tris(x0, x1, y0, y1, z0, z1):
    v = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
    ]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (2, 3, 7, 6), (0, 4, 7, 3), (1, 2, 6, 5)]
    tris = []
    for a, b, c, d in faces:
        tris.append((v[a], v[b], v[c]))
        tris.append((v[a], v[c], v[d]))
    return tris


def write_tri_stl(path: Path, tris, name: str) -> None:
    lines = [f"solid {name}"]
    for a, b, c in tris:
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        n = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        lines.append(f"  facet normal {nx / n:.6e} {ny / n:.6e} {nz / n:.6e}")
        lines.append("    outer loop")
        for p in (a, b, c):
            lines.append(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {name}")
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def write_stl(path: Path, boxes, name: str) -> None:
    write_tri_stl(path, [t for bx in boxes for t in _box_tris(*bx)], name)


def ring_tris(z0, z1, r_out=10.0 / 2, bore=5.1, n=16):
    """Hollow 16-sided knuckle on the hinge axis (x=y=0), mm. Bore is 5.1 mm across flats."""
    r_in = bore / 2 / math.cos(math.pi / n)
    tris = []
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        o0 = (r_out * math.cos(a0), r_out * math.sin(a0))
        o1 = (r_out * math.cos(a1), r_out * math.sin(a1))
        i0 = (r_in * math.cos(a0), r_in * math.sin(a0))
        i1 = (r_in * math.cos(a1), r_in * math.sin(a1))
        for za, zb in ((z0, z1),):
            tris.append(((*o0, za), (*o1, za), (*o1, zb)))
            tris.append(((*o0, za), (*o1, zb), (*o0, zb)))
            tris.append(((*i0, za), (*i1, zb), (*i1, za)))
            tris.append(((*i0, za), (*i0, zb), (*i1, zb)))
        tris.append(((*o0, z1), (*o1, z1), (*i1, z1)))
        tris.append(((*o0, z1), (*i1, z1), (*i0, z1)))
        tris.append(((*o0, z0), (*i1, z0), (*o1, z0)))
        tris.append(((*o0, z0), (*i0, z0), (*i1, z0)))
    return tris


FRAME_KNUCKLES_Z = ((0.0, mm(FK_LEN)), (mm(2 * FK_IN + FK_LEN), mm(2 * FK_IN + 2 * FK_LEN)))
HOLE_U = (5.0, 14.0)
HOLE_MARGIN = 5.0
LEAF_U = (-3.6, 20.0)


def check_hinge_frame():
    (lo0, lo1), (up0, up1) = FRAME_KNUCKLES_Z
    gap = up0 - lo1
    assert gap >= 16.0 - 1e-6, f"knuckle gap {gap} mm < 16 mm"
    assert abs(lo1 - lo0 - 15.0) < 1e-6 and abs(up1 - up0 - 15.0) < 1e-6
    assert (gap - mm(DK_LEN)) / 2 <= 0.75 + 1e-6, f"axial play {(gap - mm(DK_LEN)) / 2} mm per side > 0.75 mm"
    assert BLOCK_H < B_NOM - (gap - mm(DK_LEN)) / 2000, "support block must sit below hinge height by more than the axial play"
    for u in HOLE_U:
        assert LEAF_U[0] + HOLE_MARGIN <= u <= LEAF_U[1] - HOLE_MARGIN, f"hole at u={u} closer than 5 mm to the leaf edge"
    assert LEAF_U[1] - LEAF_U[0] >= 2 * HOLE_MARGIN
    assert (lo1 - lo0) / 2 >= HOLE_MARGIN, "leaf strap too short for a hole with 5 mm margin"


def hinge_frame_leaf_tris():
    """Frame leaf: two 15 mm knuckles with a 16 mm empty gap (inner faces 8 mm either side of the hinge centre),
    each with 1.5 mm leaf strap alongside it only (no plate across the gap).
    Axis at x=y=0, z=0 at the lower knuckle bottom, leaf face at y=-4.7 .. -3.2."""
    lo, up = FRAME_KNUCKLES_Z
    x0, x1 = 3.0, mm(X_F + 0.020 - X_A)
    tris = []
    for z0, z1 in (lo, up):
        tris += ring_tris(z0, z1)
        tris += _box_tris(x0, x1, -4.7, -3.2, z0, z1)
    return tris


def hinge_door_leaf_tris():
    """Door leaf: 1.5 mm plate, 24 mm wide, plus one 12 mm knuckle. Axis at x=y=0, z=0 at the knuckle bottom."""
    tris = ring_tris(0.0, mm(DK_LEN))
    tris += hinge_door_leaf_leaf_tris()
    return tris


DOOR_LEAF_BOX = (3.5, 5.0, -27.0, -3.0)  # leaf on the outer face of the door plate (plate outer face 5.0 from the axis)


def hinge_door_leaf_leaf_tris():
    return _box_tris(*DOOR_LEAF_BOX[:4], 0.0, mm(DK_LEN))


class Dxf:
    def __init__(self):
        self.ents = []

    def line(self, x0, y0, x1, y1, layer="0"):
        self.ents.append(
            ["0", "LINE", "8", layer, "10", f"{x0:.4f}", "20", f"{y0:.4f}", "30", "0.0",
             "11", f"{x1:.4f}", "21", f"{y1:.4f}", "31", "0.0"]
        )

    def rect(self, x0, y0, x1, y1, layer="0"):
        self.line(x0, y0, x1, y0, layer)
        self.line(x1, y0, x1, y1, layer)
        self.line(x1, y1, x0, y1, layer)
        self.line(x0, y1, x0, y0, layer)

    def cross(self, x, y, r=10.0, layer="0"):
        self.line(x - r, y, x + r, y, layer)
        self.line(x, y - r, x, y + r, layer)

    def circle_lines(self, cx, cy, r, n=24, layer="0"):
        pts = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n + 1)]
        for a, b in zip(pts, pts[1:]):
            self.line(a[0], a[1], b[0], b[1], layer)

    def text(self, x, y, s, height=4.0, layer="TEXT"):
        self.ents.append(
            ["0", "TEXT", "8", layer, "10", f"{x:.4f}", "20", f"{y:.4f}", "30", "0.0",
             "40", f"{height:.3f}", "1", s]
        )

    def write(self, path: Path) -> None:
        out = ["0", "SECTION", "2", "HEADER", "9", "$ACADVER", "1", "AC1009", "0", "ENDSEC",
               "0", "SECTION", "2", "ENTITIES"]
        for e in self.ents:
            out += e
        out += ["0", "ENDSEC", "0", "EOF"]
        path.write_text("\n".join(out) + "\n", encoding="ascii")


def write_layout(path: Path) -> None:
    d = Dxf()
    y0, y1 = Y_C - W / 2, Y_C + W / 2
    x1 = X_F + D
    d.rect(mm(X_F), mm(y0), mm(x1), mm(y1), "ENCLOSURE")
    d.text(mm(X_F) + 5, mm(y0) + 5, f"ENCLOSURE FOOTPRINT {mm(D):g} x {mm(W):g} (ON 15 SPACER)")
    d.rect(mm(X_F + 0.005), mm(y0), mm(x1), mm(y1), "SPACER")
    d.rect(mm(x1), mm(Y_C - BACKBOARD_HALF_Y), mm(x1 + BACKBOARD_T), mm(Y_C + BACKBOARD_HALF_Y), "BACKBOARD")
    d.text(mm(x1) + 22, mm(Y_C) - 30, "BACKBOARD 18 x 500 (400 TALL)")
    for bx in rest_boxes():
        d.rect(bx[0], bx[2], bx[1], bx[3], "DOOR_REST")
    d.text(mm(DOOR_START_X - 0.190), mm(Y_A) + 25, f"DOOR SUPPORT BLOCK {mm(BLOCK_H):g} THICK (BELOW HINGE HEIGHT, NOT A STOP) + 2 GUIDE STRIPS 16 x 20 x 30 (ACETAL POM)")
    bx0, bx1, by0, by1 = pin_block_extent()
    d.rect(mm(bx0), mm(by0), mm(bx1), mm(by1), "PIN_BLOCK")
    d.text(mm(bx0), mm(by1) + 6, "PIN BLOCK 60 x 75 x 40, 2 HOLES 6 x 6 x 30")
    ax, ay = mm(X_A), mm(Y_A)
    d.cross(ax, ay, 10.0, "CROSSHAIR")
    d.text(ax + 12, ay + 3, f"HINGE AXIS x={ax:.1f} y={ay:.1f} (BOTH HINGES)")
    d.text(mm(X_F) + 5, mm(y1) + 8, f"FRONT PLANE x={mm(X_F):.1f}")
    for k, (px, py) in enumerate(PIN_START):
        d.cross(mm(px), mm(py), 6.0, "CROSSHAIR")
        d.text(mm(px) + 8, mm(py) + 3, f"PIN {'AB'[k]} START x={mm(px):.1f} y={mm(py):.1f}", 3.0)
    d.text(mm(DOOR_START_X) + 5, ay - 40, "UNITS MM, TABLE FRAME: X FORWARD, Y LEFT")
    d.write(path)


def write_drilling(path: Path) -> None:
    """Left side wall, y-z view. u = depth behind the front edge, v = height above the body bottom."""
    d = Dxf()
    depth, height = mm(D), mm(H)
    d.rect(0, 0, depth, height, "WALL")
    d.text(0, height + 8, f"LEFT SIDE WALL {mm(D):g} x {mm(H):g} (FRONT EDGE AT u=0), UNITS MM")
    d.text(0, height + 16, "FOR hinge_frame_leaf_v1.stl (1.5 MM LEAF, 2 SCREW HOLES DIA 4.5 PER LEAF)", 3.0)
    leaf_u0, leaf_u1 = LEAF_U
    for i, hc in enumerate(HINGE_HC):
        vc = mm(hc - BODY_H0)
        d.line(-15, vc, depth, vc, "CENTRE")
        d.text(depth + 5, vc - 2, f"HINGE {i + 1} CENTRE v={vc:.1f} ({height - vc:.1f} FROM TOP)", 3.0)
        (lo0, lo1), (up0, up1) = (
            (hc - FK_IN - FK_LEN, hc - FK_IN),
            (hc + FK_IN, hc + FK_IN + FK_LEN),
        )
        for k, (k0, k1) in enumerate(((lo0, lo1), (up0, up1))):
            v0, v1 = mm(k0 - BODY_H0), mm(k1 - BODY_H0)
            d.rect(leaf_u0, v0, leaf_u1, v1, "LEAF")
            vm = (v0 + v1) / 2
            for u in HOLE_U:
                d.circle_lines(u, vm, 2.25, 24, "HOLE")
            d.text(leaf_u1 + 4, vm - 1.5, f"2 x DIA 4.5 AT u={HOLE_U[0]:g}, {HOLE_U[1]:g}  v={vm:.1f}", 3.0)
    d.write(path)


def write_pin(path: Path) -> None:
    """Side view of one hinge pin, mm. z=0 at the ball tip, pin axis at x=0."""
    d = Dxf()
    r, ln = 2.4, mm(PIN_L)
    d.line(-r, r, -r, ln, "PIN")
    d.line(r, r, r, ln, "PIN")
    for a0, a1 in ((180, 270), (270, 360)):
        pts = [(r * math.cos(math.radians(a)), r + r * math.sin(math.radians(a))) for a in range(a0, a1 + 1, 15)]
        for p, q in zip(pts, pts[1:]):
            d.line(p[0], p[1], q[0], q[1], "PIN")
    d.rect(-4.5, ln, 4.5, ln + 6.0, "HEAD")
    d.rect(-5.5, ln, 5.5, ln + 14.0, "CAP")
    d.line(-4.5, ln, -4.5, ln + 6.0, "CAP_BORE")
    d.line(4.5, ln, 4.5, ln + 6.0, "CAP_BORE")
    d.line(0, -6, 0, ln + 20, "CENTRE")
    d.line(10, 0, 10, ln, "DIM")
    d.text(13, ln / 2, "52 SHANK LENGTH UNDER HEAD, BALL TIP INCLUDED", 3.0)
    d.text(8, 2, "DIA 4.8 STAINLESS SHANK, BALL TIP R2.4", 3.0)
    d.text(-70, ln + 2, "STEEL HEAD DIA 9 x 6", 3.0)
    d.text(-70, ln + 8, "ORANGE VINYL CAP DIA 11 x 14 TALL, BORE DIA 9", 3.0)
    d.text(-70, ln + 18, "HINGE PIN, SIDE VIEW, UNITS MM", 3.0)
    d.write(path)


# fabrication drawing constants in mm, match task.py
T_WALL_MM = 1.5
DOOR_FLANGE_D_MM = 12.0
DOOR_FLANGE_SIDE_MM = 4.5
DOOR_FLANGE_TOP_MM = 7.5
KNOB_MM = (20.0, 12.0, 30.0)
KNOB_FROM_LATCH_MM = 25.0
HINGE_AXIS_OUT_MM = 6.6
DOOR_LEAF_MM = (1.5, 24.0)
DOOR_BOTTOM_ABOVE_BODY_MM = round(mm(B_REST - BODY_H0), 2)


DK_Z = tuple(hc - B_NOM for hc in HINGE_HC)  # door knuckle centres from the door bottom, match task.py


def _leaf_ranges():
    out = []
    for hc in HINGE_HC:
        out.append((mm(hc - FK_IN - FK_LEN - BODY_H0), mm(hc - FK_IN - BODY_H0)))
        out.append((mm(hc + FK_IN - BODY_H0), mm(hc + FK_IN + FK_LEN - BODY_H0)))
    return out


def write_body(path: Path) -> None:
    """Enclosure body: front view (as seen from the robot, hinge side on the viewer's left) and left side view."""
    d = Dxf()
    w, h, dep, t = mm(W), mm(H), mm(D), T_WALL_MM
    d.text(0, h + 24, "ENCLOSURE BODY (LHC252015 OUTER SIZE), OPEN FRONT, PLAIN FRONT EDGES (NO FLANGE TROUGH, NO GASKET), UNITS MM", 4.0)
    d.text(0, h + 16, f"16 GAUGE STEEL, WALL 1.5, OVERALL {mm(H):g} HIGH x {mm(W):g} WIDE x {mm(D):g} DEEP", 3.0)
    d.text(0, h + 8, "FRONT VIEW (SEEN FROM THE ROBOT, LEFT SIDE WALL WITH THE HINGE LEAVES AT LEFT OF DRAWING)", 3.0)
    d.rect(0, 0, w, h, "BODY")
    d.rect(t, t, w - t, h - t, "BODY_INNER")
    d.text(w + 6, h / 2, f"{h:g} HIGH", 3.0)
    d.text(w / 2 - 20, -8, f"{w:g} WIDE, WALL 1.5", 3.0)
    d.text(t + 4, h - t - 10, f"CLEAR OPENING {w - 2 * t:g} x {h - 2 * t:g}", 3.0)
    for v0, v1 in _leaf_ranges():
        d.rect(0, v0, t + 2, v1, "HINGE_LEAF")
    d.text(w + 6, mm(HINGE_HC[0] - BODY_H0), "HINGE CENTRES 40 FROM BODY BOTTOM AND TOP", 3.0)
    ox = w + 90.0
    d.text(ox, h + 8, "LEFT SIDE VIEW (FRONT EDGE AT LEFT, u = DEPTH BEHIND FRONT EDGE)", 3.0)
    d.rect(ox, 0, ox + dep, h, "BODY")
    d.text(ox + 10, -8, f"{dep:g} DEEP", 3.0)
    leaf_u0, leaf_u1 = LEAF_U
    for v0, v1 in _leaf_ranges():
        d.rect(ox + leaf_u0, v0, ox + leaf_u1, v1, "HINGE_LEAF")
    for hc in HINGE_HC:
        vc = mm(hc - BODY_H0)
        d.line(ox - 15, vc, ox + dep, vc, "CENTRE")
        d.text(ox + dep + 4, vc - 2, f"HINGE CENTRE v={vc:.1f}", 3.0)
    d.text(ox, -18, "HINGE LEAVES (1.5 THICK) SPAN u=-3.6 TO 20, SCREW HOLES PER hinge_drilling_v1.dxf", 3.0)
    d.text(ox, -26, "HINGE AXIS 6.6 IN FRONT OF THE FRONT EDGE (TOWARD THE ROBOT), 5 OUTBOARD OF THE LEFT SIDE WALL", 3.0)
    d.write(path)


def door_leaf_span_u():
    """Door leaf span along the door width, u measured from the hinge-side plate edge toward the latch."""
    return -DOOR_LEAF_BOX[3] - HINGE_AXIS_OUT_MM, -DOOR_LEAF_BOX[2] - HINGE_AXIS_OUT_MM


def check_door_leaf():
    leaf = [v for t in hinge_door_leaf_leaf_tris() for v in t]
    stl_u0 = min(-v[1] for v in leaf) - HINGE_AXIS_OUT_MM
    stl_u1 = max(-v[1] for v in leaf) - HINGE_AXIS_OUT_MM
    u0, u1 = door_leaf_span_u()
    assert abs(stl_u0 - u0) < 1e-9 and abs(stl_u1 - u1) < 1e-9, ((stl_u0, stl_u1), (u0, u1))


def write_door(path: Path) -> None:
    """Door: outer-face view (hinge edge at left, latch edge at right) and hinge-side section view."""
    d = Dxf()
    w, h = mm(W), mm(H)
    t, fd = T_WALL_MM, DOOR_FLANGE_D_MM
    fs, ft = DOOR_FLANGE_SIDE_MM, DOOR_FLANGE_TOP_MM
    d.text(0, h + 24, f"DOOR, PLATE {mm(H):g} x {mm(W):g} x {t:g} STEEL, RETURN FLANGES 12 DEEP, UNITS MM", 4.0)
    d.text(0, h + 16, "OUTER FACE VIEW, HINGE EDGE AT LEFT, LATCH EDGE AT RIGHT", 3.0)
    d.rect(0, 0, w, h, "DOOR")
    d.rect(fs, 0, w - fs, h - ft, "FLANGE_LINE")
    d.text(w + 6, h / 2, f"{h:g} HIGH", 3.0)
    d.text(w / 2 - 20, -8, f"{w:g} WIDE", 3.0)
    d.text(w / 2 - 70, h - 18, "FLANGES: SIDES 4.5 IN, TOP 7.5 IN, BOTTOM FLUSH", 3.0)
    kx = w - KNOB_FROM_LATCH_MM
    d.rect(kx - KNOB_MM[0] / 2, h / 2 - KNOB_MM[1] / 2, kx + KNOB_MM[0] / 2, h / 2 + KNOB_MM[1] / 2, "KNOB")
    d.text(kx - 60, h / 2 + 14, "WING KNOB 20 x 12 x 30, 25 FROM LATCH EDGE, MID HEIGHT", 3.0)
    leaf_u0, leaf_u1 = door_leaf_span_u()
    for dz in DK_Z:
        z = mm(dz)
        d.rect(leaf_u0, z - DK_LEN * 500, leaf_u1, z + DK_LEN * 500, "DOOR_LEAF")
        d.line(-HINGE_AXIS_OUT_MM, z - 12, -HINGE_AXIS_OUT_MM, z + 12, "CENTRE")
        d.text(leaf_u1 + 8, z - 2, f"KNUCKLE CENTRE {z:.1f} FROM DOOR BOTTOM ({h - z:.1f} FROM TOP)", 3.0)
        d.text(leaf_u1 + 8, z + 4, f"DOOR LEAF u={leaf_u0:.1f} TO {leaf_u1:.1f} FROM THE HINGE-SIDE PLATE EDGE", 3.0)
    d.text(0, h + 8, "DOOR LEAF ON THE OUTER FACE OF THE DOOR PLATE, 1.5 x 24, KNUCKLE 10 OD x 14.5 LONG, 0.75 AXIAL PLAY PER SIDE; HINGE AXIS (VERTICAL CENTRE LINE) IS 6.6 OUTBOARD OF THE HINGE-SIDE PLATE EDGE", 3.0)
    ox = w + 150.0
    d.text(ox, h + 16, "SIDE SECTION (PLATE AT LEFT, FLANGES POINT RIGHT)", 3.0)
    d.rect(ox, 0, ox + t, h, "DOOR")
    d.rect(ox + t, 0, ox + t + fd, t, "FLANGE")
    d.rect(ox + t, h - ft - t, ox + t + fd, h - ft, "FLANGE")
    d.text(ox + 30, h - ft - 2, "TOP FLANGE INSET 7.5", 3.0)
    d.text(ox + 30, 4, "BOTTOM FLANGE FLUSH WITH DOOR BOTTOM", 3.0)
    d.text(ox + 30, h / 2, f"FLANGE DEPTH 12, PLATE {t:g}", 3.0)
    d.text(ox, -8, f"DOOR HANGS WITH ITS BOTTOM {DOOR_BOTTOM_ABOVE_BODY_MM:g} ABOVE THE BODY BOTTOM (4.5 WHEN KNUCKLES ARE CENTRED IN THE GAPS)", 3.0)
    d.text(ox, -16, f"SO DOOR KNUCKLES ({mm(DK_Z[0]):g} AND {mm(DK_Z[1]):g} FROM DOOR BOTTOM) MEET BODY HINGES ({mm(HINGE_HC[0] - BODY_H0):g} AND {mm(HINGE_HC[1] - BODY_H0):g} FROM BODY BOTTOM)", 3.0)
    d.write(path)


def main() -> None:
    check_hinge_frame()
    check_door_leaf()
    write_body(HERE / "enclosure_body_v1.dxf")
    write_door(HERE / "door_v1.dxf")
    write_tri_stl(HERE / "hinge_frame_leaf_v1.stl", hinge_frame_leaf_tris(), "hinge_frame_leaf_v1")
    write_tri_stl(HERE / "hinge_door_leaf_v1.stl", hinge_door_leaf_tris(), "hinge_door_leaf_v1")
    write_pin(HERE / "hinge_pin_v1.dxf")
    write_stl(HERE / "door_setting_block_v1.stl", rest_boxes(), "door_setting_block_v1")
    write_stl(HERE / "pin_block_v1.stl", pin_block_boxes(), "pin_block_v1")
    write_stl(HERE / "backboard_spacer_v1.stl", spacer_boxes(), "backboard_spacer_v1")
    write_layout(HERE / "layout_template_v1.dxf")
    write_drilling(HERE / "hinge_drilling_v1.dxf")
    names = ["enclosure_body_v1.dxf", "door_v1.dxf", "hinge_frame_leaf_v1.stl", "hinge_door_leaf_v1.stl", "hinge_pin_v1.dxf", "door_setting_block_v1.stl", "pin_block_v1.stl", "backboard_spacer_v1.stl",
             "layout_template_v1.dxf", "hinge_drilling_v1.dxf"]
    print("wrote " + ", ".join(names))


if __name__ == "__main__":
    main()
