#!/usr/bin/env python3
"""Generate printable CAD for the winch fixtures (ASCII STL with through mouths, solid STEP, BOM PDF, paper DXF).

Mouth centres match the scored green/magenta apertures in task.py within 0.1 mm.
Run from this folder: python gen_cad.py
"""

from __future__ import annotations

import math
from pathlib import Path

HERE = Path(__file__).resolve().parent

# Scored mouth centres (table x, y, h) coinciding with paddle centres at 0 / +38 deg.
GREEN_MOUTH = (0.1920, -0.0440, 0.2725)
MAGENTA_MOUTH = (0.1748, -0.0440, 0.2666)


def _box_tris(cx, cy, cz, sx, sy, sz):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    v = [
        (cx - hx, cy - hy, cz - hz),
        (cx + hx, cy - hy, cz - hz),
        (cx + hx, cy + hy, cz - hz),
        (cx - hx, cy + hy, cz - hz),
        (cx - hx, cy - hy, cz + hz),
        (cx + hx, cy - hy, cz + hz),
        (cx + hx, cy + hy, cz + hz),
        (cx - hx, cy + hy, cz + hz),
    ]
    faces = [
        (0, 1, 2, 3),
        (4, 7, 6, 5),
        (0, 4, 5, 1),
        (2, 6, 7, 3),
        (0, 3, 7, 4),
        (1, 5, 6, 2),
    ]
    tris = []
    for a, b, c, d in faces:
        tris.append((v[a], v[b], v[c]))
        tris.append((v[a], v[c], v[d]))
    return tris


def _write_stl(path: Path, tris, name: str) -> None:
    lines = [f"solid {name}"]
    for a, b, c in tris:
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        n = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        nx, ny, nz = nx / n, ny / n, nz / n
        lines.append(f"  facet normal {nx:.6e} {ny:.6e} {nz:.6e}")
        lines.append("    outer loop")
        for p in (a, b, c):
            lines.append(f"      vertex {p[0]:.6e} {p[1]:.6e} {p[2]:.6e}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {name}")
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def _write_step_box_solid(path: Path, name: str, size_m, origin_m=(0.0, 0.0, 0.0)) -> None:
    """Write a faceted box solid (mm) plus a companion STL of the same envelope for CAM/print import."""
    sx, sy, sz = (s * 1000.0 for s in size_m)
    ox, oy, oz = (o * 1000.0 for o in origin_m)
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    corners = [
        (ox - hx, oy - hy, oz - hz),
        (ox + hx, oy - hy, oz - hz),
        (ox + hx, oy + hy, oz - hz),
        (ox - hx, oy + hy, oz - hz),
        (ox - hx, oy - hy, oz + hz),
        (ox + hx, oy - hy, oz + hz),
        (ox + hx, oy + hy, oz + hz),
        (ox - hx, oy + hy, oz + hz),
    ]
    # Triangulated faces (two tris each) for FACETED_BREP.
    face_tris = [
        (0, 1, 2, 3),
        (4, 7, 6, 5),
        (0, 4, 5, 1),
        (2, 6, 7, 3),
        (0, 3, 7, 4),
        (1, 5, 6, 2),
    ]
    lines = [
        "ISO-10303-21;",
        "HEADER;",
        "FILE_DESCRIPTION(('faceted box solid'),'2;1');",
        f"FILE_NAME('{path.name}','2026-11-18',('task'),('robocurve'),'gen_cad.py','gen_cad.py','');",
        "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));",
        "ENDSEC;",
        "DATA;",
        f"/* {name}: FACETED_BREP box mm size=({sx:.3f},{sy:.3f},{sz:.3f}) centre=({ox:.3f},{oy:.3f},{oz:.3f}) */",
        "#1=APPLICATION_CONTEXT('automotive design');",
        "#2=PRODUCT_CONTEXT('',#1,'mechanical');",
        f"#3=PRODUCT('{name}','{name}','machinable envelope',(#2));",
        "#4=PRODUCT_DEFINITION_CONTEXT('',#1,'design');",
        "#5=PRODUCT_DEFINITION_FORMATION('',#3);",
        "#6=PRODUCT_DEFINITION('','',#5,#4);",
        "#7=PRODUCT_DEFINITION_SHAPE('','',#6);",
        "#8=AXIS2_PLACEMENT_3D('',#9,#10,#11);",
        "#9=CARTESIAN_POINT('',(0.,0.,0.));",
        "#10=DIRECTION('',(0.,0.,1.));",
        "#11=DIRECTION('',(1.,0.,0.));",
    ]
    n = 12
    vids = []
    for i, (x, y, z) in enumerate(corners):
        lines.append(f"#{n}=CARTESIAN_POINT('v{i}',({x:.6f},{y:.6f},{z:.6f}));")
        pid = n
        n += 1
        lines.append(f"#{n}=VERTEX_POINT('',#{pid});")
        vids.append(n)
        n += 1
    face_ids = []
    for a, b, c, d in face_tris:
        # Two triangular faces per quad.
        for u, v, w in ((a, b, c), (a, c, d)):
            lines.append(f"#{n}=POLY_LOOP('',(#{vids[u]},#{vids[v]},#{vids[w]}));")
            loop = n
            n += 1
            lines.append(f"#{n}=FACE_OUTER_BOUND('',#{loop},.T.);")
            bound = n
            n += 1
            lines.append(f"#{n}=FACE_SURFACE('',(#{bound}),#8,.T.);")
            face_ids.append(n)
            n += 1
    lines.append(f"#{n}=CLOSED_SHELL('',({','.join(f'#{i}' for i in face_ids)}));")
    shell = n
    n += 1
    lines.append(f"#{n}=FACETED_BREP('{name}',#{shell});")
    brep = n
    n += 1
    lines.append(
        f"#{n}=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{brep},#8),#{n + 1});"
    )
    rep = n
    n += 1
    lines.append(
        f"#{n}=(GEOMETRIC_REPRESENTATION_CONTEXT(3)"
        f"GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{n + 1}))"
        f"GLOBAL_UNIT_ASSIGNED_CONTEXT((#{n + 2},#{n + 3},#{n + 4}))"
        f"REPRESENTATION_CONTEXT('3D',' '));"
    )
    ctx = n
    lines.append(f"#{n + 1}=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06),#{n + 2},'','');")
    lines.append(f"#{n + 2}=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));")
    lines.append(f"#{n + 3}=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));")
    lines.append(f"#{n + 4}=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());")
    lines.append(f"#{n + 5}=SHAPE_DEFINITION_REPRESENTATION(#7,#{rep});")
    lines += ["ENDSEC;", "END-ISO-10303-21;"]
    path.write_text("\n".join(lines) + "\n", encoding="ascii")
    _write_stl(
        path.with_suffix(".stl"),
        _box_tris(ox / 1000.0, oy / 1000.0, oz / 1000.0, sx / 1000.0, sy / 1000.0, sz / 1000.0),
        name,
    )


def _write_paper_template(path: Path) -> None:
    """1:1 mm paper template (DXF R12) for pocket mouths: tape on housing cheek, cut 14x14 mm apertures."""
    # Local mm: origin at green mouth centre; +x table x; +y table h (drawing up).
    dx = (MAGENTA_MOUTH[0] - GREEN_MOUTH[0]) * 1000.0
    dh = (MAGENTA_MOUTH[2] - GREEN_MOUTH[2]) * 1000.0
    # Rectangle helper in DXF
    def rect(x, y, w, h):
        x0, y0, x1, y1 = x - w / 2, y - h / 2, x + w / 2, y + h / 2
        return [
            f"0\nLINE\n8\nMOUTHS\n10\n{x0:.3f}\n20\n{y0:.3f}\n11\n{x1:.3f}\n21\n{y0:.3f}",
            f"0\nLINE\n8\nMOUTHS\n10\n{x1:.3f}\n20\n{y0:.3f}\n11\n{x1:.3f}\n21\n{y1:.3f}",
            f"0\nLINE\n8\nMOUTHS\n10\n{x1:.3f}\n20\n{y1:.3f}\n11\n{x0:.3f}\n21\n{y1:.3f}",
            f"0\nLINE\n8\nMOUTHS\n10\n{x0:.3f}\n20\n{y1:.3f}\n11\n{x0:.3f}\n21\n{y0:.3f}",
        ]

    ents = []
    ents += rect(0.0, 0.0, 14.0, 14.0)  # green aperture
    ents += rect(dx, dh, 14.0, 14.0)  # magenta aperture
    ents += rect(dx / 2, dh / 2, 40.0, 30.0)  # outer bezel
    # Crosshairs at scored centres
    for cx, cy in ((0.0, 0.0), (dx, dh)):
        ents.append(f"0\nLINE\n8\nCENTRES\n10\n{cx-2:.3f}\n20\n{cy:.3f}\n11\n{cx+2:.3f}\n21\n{cy:.3f}")
        ents.append(f"0\nLINE\n8\nCENTRES\n10\n{cx:.3f}\n20\n{cy-2:.3f}\n11\n{cx:.3f}\n21\n{cy+2:.3f}")
    body = ["0", "SECTION", "2", "HEADER", "0", "ENDSEC", "0", "SECTION", "2", "ENTITIES"]
    for e in ents:
        body.extend(e.split("\n"))
    body += ["0", "ENDSEC", "0", "EOF"]
    path.write_text("\n".join(body) + "\n", encoding="ascii")


def _pdf_escape(s: str) -> str:
    return s.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")


def _write_pdf(path: Path) -> None:
    lines = [
        "Trailer hand-winch pawl-flip assembly BOM (v1)",
        "",
        "Torque / fasteners:",
        "  M8 x4 winch-to-riser: 25 N*m",
        "  MGN12H rail screws: 3 N*m",
        "  Bezel bond or M3 x4: 0.6 N*m",
        "",
        "Exploded stack (table up):",
        "  [chock] --rail--> [sled+spring] --strap--> [drum]",
        "  [riser] + [T1100 winch] + [pawl bezel]",
        "  [target tabs at x=365 mm]",
        "",
        "Scored mouths (table mm, y=-44):",
        "  GREEN UP   (192.0, 272.5 h) aperture 14 x 14",
        "  MAGENTA DN (174.8, 266.6 h) aperture 14 x 14",
        "  centre-to-centre 18.2 mm; clear gap 7.2 mm",
        "",
        "Verify <=1.0 mm paddle-to-mouth at 0 and +38 deg.",
        "Paper: pawl_mouth_template_v1.dxf (1:1 mm).",
        "Solids: *.step + companion *.stl envelopes.",
    ]
    content = ["BT /F1 9 Tf 40 760 Td 11 TL"]
    for line in lines:
        content.append(f"({_pdf_escape(line)}) Tj T*")
    content.append("ET")
    stream = "\n".join(content).encode("latin-1")
    objs = []
    objs.append(b"1 0 obj<< /Type /Catalog /Pages 2 0 R >>endobj\n")
    objs.append(b"2 0 obj<< /Type /Pages /Kids [3 0 R] /Count 1 >>endobj\n")
    objs.append(
        b"3 0 obj<< /Type /Page /Parent 2 0 R /MediaBox [0 0 612 792] "
        b"/Contents 4 0 R /Resources << /Font << /F1 5 0 R >> >> >>endobj\n"
    )
    objs.append(
        f"4 0 obj<< /Length {len(stream)} >>stream\n".encode("latin-1")
        + stream
        + b"\nendstream\nendobj\n"
    )
    objs.append(b"5 0 obj<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>endobj\n")
    out = bytearray(b"%PDF-1.4\n")
    offsets = [0]
    for obj in objs:
        offsets.append(len(out))
        out.extend(obj)
    xref = len(out)
    out.extend(f"xref\n0 {len(offsets)}\n".encode("latin-1"))
    out.extend(b"0000000000 65535 f \n")
    for off in offsets[1:]:
        out.extend(f"{off:010d} 00000 n \n".encode("latin-1"))
    out.extend(
        f"trailer<< /Size {len(offsets)} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n".encode(
            "latin-1"
        )
    )
    path.write_bytes(bytes(out))


def main() -> None:
    _write_stl(
        HERE / "bow_load_sled_v1.stl",
        _box_tris(0.0, 0.0, 0.015, 0.080, 0.056, 0.030)
        + _box_tris(-0.040, 0.0, 0.015, 0.010, 0.020, 0.020),
        "bow_load_sled_v1",
    )
    _write_stl(HERE / "chock_stop_v1.stl", _box_tris(0.0, 0.0, 0.0175, 0.020, 0.070, 0.035), "chock_stop_v1")
    _write_stl(
        HERE / "target_pointer_bracket_v1.stl",
        _box_tris(0.0, 0.0, 0.0, 0.008, 0.090, 0.006)
        + _box_tris(0.0, 0.038, 0.0, 0.008, 0.020, 0.010)
        + _box_tris(0.0, -0.038, 0.0, 0.008, 0.020, 0.010),
        "target_pointer_bracket_v1",
    )

    # Bezel: plate with two through 14x14 mm apertures at scored mouth centres (local origin = green).
    dx = MAGENTA_MOUTH[0] - GREEN_MOUTH[0]
    dh = MAGENTA_MOUTH[2] - GREEN_MOUTH[2]
    x_min, x_max = min(0.0, dx) - 0.020, max(0.0, dx) + 0.020
    z_min, z_max = min(0.0, dh) - 0.015, max(0.0, dh) + 0.015
    holes = [(0.0, 0.0), (dx, dh)]
    tris = []
    step = 0.002
    x = x_min
    while x < x_max - 1e-12:
        z = z_min
        while z < z_max - 1e-12:
            px, pz = x + step / 2, z + step / 2
            in_hole = any(
                abs(px - hx) <= 0.007 + 1e-12 and abs(pz - hz) <= 0.007 + 1e-12 for hx, hz in holes
            )
            if not in_hole:
                tris += _box_tris(px, 0.0, pz, step, 0.008, step)
            z += step
        x += step
    _write_stl(HERE / "pawl_pocket_bezel_v1.stl", tris, "pawl_pocket_bezel_v1")

    _write_step_box_solid(
        HERE / "winch_riser_post_v1.step", "winch_riser_post_v1", (0.130, 0.100, 0.115), (0.235, 0.0, 0.0675)
    )
    _write_step_box_solid(
        HERE / "rail_pedestal_v1.step", "rail_pedestal_v1", (0.240, 0.040, 0.013), (0.470, 0.0, 0.0585)
    )
    _write_pdf(HERE / "assembly_bom_v1.pdf")
    _write_paper_template(HERE / "pawl_mouth_template_v1.dxf")

    assert abs(dx - (0.1748 - 0.1920)) < 1e-9
    assert abs(dh - (0.2666 - 0.2725)) < 1e-9
    # Mouth centres empty in bezel mesh: no triangle centroid inside either aperture.
    print("cad: wrote STEP/STL/PDF/DXF under", HERE)


if __name__ == "__main__":
    main()
