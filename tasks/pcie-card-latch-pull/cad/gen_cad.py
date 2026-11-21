#!/usr/bin/env python3
"""Generate printable gauges, a 1:1 layout template and a BOM page for the PCIe card latch-pull bench.

Pure Python + numpy, deterministic. Run from this folder: python3 gen_cad.py
Outputs: tail_lift_gauge_v1.stl, flat_check_wedge_v1.stl, layout_template_v1.dxf, assembly_bom_v1.pdf
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent

# Numbers below mirror task.py (metres, world frame). The card and slot are built in the card frame (origin at the tail
# end) and turned 90 deg about vertical: card x (tail to bracket) = world +y, card y (shroud side) = world -x.
CARD_ORIGIN = np.array([0.180, -0.030])
FRAME_R = np.array([[0.0, -1.0], [1.0, 0.0]])
CARD_LEN = 0.1559
PCB_HALF = 0.00157 / 2
SHROUD_Y1 = 0.0150
BAND_X = (0.095, 0.135)
SLOT_HALF_W = 0.0040
SLOT_X = (0.0405, 0.1465)
FINGER_X = (0.0525, 0.1415)
TAB_X = (0.0205, 0.0305)  # under the card overhang, beside the PCB back face
TAB_Y = (-0.022, -0.003)
LAYOUT_SHIFT = np.array([0.200, -0.060])


def cf(p):
    """Card-frame point to world x, y."""
    return CARD_ORIGIN + FRAME_R @ np.asarray(p, float)


def lay(lo, hi):
    """Box laid out in the card frame (numbers = card-frame position + (0.200, -0.060)) to world lo, hi."""
    a, b = cf(np.asarray(lo, float) - LAYOUT_SHIFT), cf(np.asarray(hi, float) - LAYOUT_SHIFT)
    return np.minimum(a, b), np.maximum(a, b)


MB_LO, MB_HI = lay((0.110, -0.240), (0.354, 0.004))
COOLER_LO, COOLER_HI = lay((0.230, -0.195), (0.325, -0.100))
DIMMS = [lay((x - 0.0035, -0.233), (x + 0.0035, -0.100)) for x in (0.125, 0.136)]
BENCH_LO, BENCH_HI = lay((0.080, -0.245), (0.375, 0.005))
MAT_LO = np.array([0.060, -0.300])
MAT_HI = np.array([0.500, 0.340])
TRAY_LO = np.array([0.150, 0.145])
TRAY_HI = np.array([0.450, 0.325])
TAIL_REF_H = 0.0146 + 0.0083  # card origin h + gold finger depth
SLOT_TOP_H = 0.0146 + 0.0090
MB_TOP_H = 0.0126


def box_tris(lo, hi):
    """12 outward-wound triangles of an axis-aligned box (mm)."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    v = np.array(
        [
            (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
            (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
        ],
        dtype=float,
    )
    quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (2, 3, 7, 6), (0, 4, 7, 3), (1, 2, 6, 5)]
    tris = []
    for a, b, c, d in quads:
        tris.append((v[a], v[b], v[c]))
        tris.append((v[a], v[c], v[d]))
    return tris


def wedge_tris(length, width, height):
    """Triangular prism: right-angle profile in x-z (thick end at x=0), extruded along y."""
    p = [(0, 0, 0), (length, 0, 0), (0, 0, height), (0, width, 0), (length, width, 0), (0, width, height)]
    p = [np.array(q, dtype=float) for q in p]
    return [
        (p[0], p[1], p[2]),  # y=0 face
        (p[3], p[5], p[4]),  # y=width face
        (p[0], p[3], p[4]), (p[0], p[4], p[1]),  # bottom
        (p[0], p[2], p[5]), (p[0], p[5], p[3]),  # vertical end at x=0
        (p[1], p[4], p[5]), (p[1], p[5], p[2]),  # slope
    ]


def write_binary_stl(path: Path, tris, name: str) -> None:
    header = f"{name} (mm)".encode("ascii").ljust(80, b" ")
    arr = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    for i, (a, b, c) in enumerate(tris):
        n = np.cross(b - a, c - a)
        n = n / (np.linalg.norm(n) or 1.0)
        arr[i]["n"] = n
        arr[i]["v"] = np.array([a, b, c])
    path.write_bytes(header + np.uint32(len(tris)).tobytes() + arr.tobytes())


def check_watertight(tris) -> bool:
    edges = {}
    for t in tris:
        keys = [tuple(np.round(p, 6)) for p in t]
        for i in range(3):
            e = (keys[i], keys[(i + 1) % 3])
            edges[e] = edges.get(e, 0) + 1
    return all(edges.get((b, a), 0) == c for (a, b), c in edges.items())


def build_gauge():
    # 60 x 20 mm footprint: 15.3 mm step (rest tail height + 5 mm) over x 0-30, 21.0 mm step (slot top + 10 mm) over x 30-60.
    # stands on the motherboard top: tail bottom at rest is 10.3 mm above it, slot top 11.0 mm above it
    assert abs((TAIL_REF_H - MB_TOP_H + 0.005) * 1000 - 15.3) < 1e-6 and abs((SLOT_TOP_H - MB_TOP_H + 0.010) * 1000 - 21.0) < 1e-6
    return box_tris((0, 0, 0), (30, 20, 15.3)) + box_tris((30, 0, 0), (60, 20, 21.0))


def build_wedge():
    return wedge_tris(60.0, 40.0, 60.0 * math.tan(math.radians(10.0)))


def write_dxf(path: Path) -> None:
    ents = []

    def line(layer, x0, y0, x1, y1):
        ents.extend(["0", "LINE", "8", layer, "10", f"{x0:.3f}", "20", f"{y0:.3f}", "30", "0.0",
                     "11", f"{x1:.3f}", "21", f"{y1:.3f}", "31", "0.0"])

    def frect(layer, lo, hi):
        a, b = lay_cf(lo, hi)
        rect(layer, a, b)

    def lay_cf(lo, hi):
        a, b = cf(lo), cf(hi)
        return np.minimum(a, b), np.maximum(a, b)

    def rect(layer, lo, hi):
        x0, y0 = lo[0] * 1000.0, lo[1] * 1000.0
        x1, y1 = hi[0] * 1000.0, hi[1] * 1000.0
        line(layer, x0, y0, x1, y0)
        line(layer, x1, y0, x1, y1)
        line(layer, x1, y1, x0, y1)
        line(layer, x0, y1, x0, y0)

    rect("MAT", MAT_LO, MAT_HI)
    rect("BENCH_TRAY", BENCH_LO, BENCH_HI)
    rect("MOTHERBOARD", MB_LO, MB_HI)
    rect("COOLER", COOLER_LO, COOLER_HI)
    for lo, hi in DIMMS:
        rect("DIMM", lo, hi)
    rect("ANTISTATIC_TRAY", TRAY_LO, TRAY_HI)
    frect("SLOT", (SLOT_X[0], -SLOT_HALF_W), (SLOT_X[1], SLOT_HALF_W))
    frect("CARD", (0.0, -PCB_HALF), (CARD_LEN, SHROUD_Y1))
    frect("TAPE_BAND", (BAND_X[0], -PCB_HALF), (BAND_X[1], SHROUD_Y1))
    frect("FINGERS", (FINGER_X[0], -PCB_HALF), (FINGER_X[1], PCB_HALF))
    frect("LATCH_TAB", (TAB_X[0], TAB_Y[0]), (TAB_X[1], TAB_Y[1]))
    line("ORIGIN", -20, 0, 20, 0)
    line("ORIGIN", 0, -20, 0, 20)
    body = ["0", "SECTION", "2", "HEADER", "0", "ENDSEC", "0", "SECTION", "2", "ENTITIES", *ents,
            "0", "ENDSEC", "0", "EOF"]
    path.write_text("\n".join(body) + "\n", encoding="ascii")


def _pdf_escape(s: str) -> str:
    return s.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")


def lay_card(x0, x1, y0, y1):
    a, b = cf((x0, y0)), cf((x1, y1))
    return np.minimum(a, b), np.maximum(a, b)


def write_pdf(path: Path) -> None:
    def mm(lo, hi):
        return f"x {lo[0] * 1000:.0f} to {hi[0] * 1000:.0f}, y {lo[1] * 1000:.0f} to {hi[1] * 1000:.0f}"

    card_lo, card_hi = lay_card(0.0, CARD_LEN, -PCB_HALF, SHROUD_Y1)
    band_lo, band_hi = lay_card(BAND_X[0], BAND_X[1], -PCB_HALF, SHROUD_Y1)
    tab_lo, tab_hi = lay_card(TAB_X[0], TAB_X[1], TAB_Y[0], TAB_Y[1])
    lines = [
        "PCIe card latch-pull bench: assembly BOM (v1)",
        "",
        "Bill of materials:",
        "  1  Dell Precision 3660 Tower, side cover off, PCIe door open, laid on its side",
        "     (own system board with spring-return x16 latch, own CPU heatsink, 2 DDR5 UDIMMs)",
        "  1  NVIDIA T400 4GB low profile card in the primary x16 slot, full-height bracket",
        "  1  conductive polypropylene antistatic tray 300 x 180 x 15 mm",
        "  1  push-pull digital force gauge, 0-50 N, 0.01 N resolution",
        "  1  rubber bench mat 440 x 640 x 3 mm",
        "  1  green vinyl tape, 40 mm wide",
        "  1  orange paint pen",
        "  1  tail_lift_gauge_v1 (5 mm and 10 mm steps), 1 flat_check_wedge_v1 (10 deg)",
        "",
        "Key positions (mm, table frame, origin = world origin on mat):",
        f"  Card turned 90 deg: plane at x {CARD_ORIGIN[0] * 1000:.0f}, long axis along +y, shroud face toward -x",
        f"  Card tail end / gold finger bottom: x {CARD_ORIGIN[0] * 1000:.1f}, y {CARD_ORIGIN[1] * 1000:.1f}",
        f"  Card: {mm(card_lo, card_hi)}; bracket end at y {(CARD_ORIGIN[1] + CARD_LEN) * 1000:.1f}",
        f"  Green band: {mm(band_lo, band_hi)} (95-135 mm from tail)",
        f"  Orange latch tab: {mm(tab_lo, tab_hi)} (under the card overhang, beside the PCB back face)",
        f"  Gold fingers: {mm(*lay_card(FINGER_X[0], FINGER_X[1], -PCB_HALF, PCB_HALF))} (52.5-141.5 mm from tail)",
        f"  System board (modelled, approximate): {mm(MB_LO, MB_HI)}",
        f"  Chassis wall plate (modelled): {mm(BENCH_LO, BENCH_HI)}",
        f"  CPU heatsink (modelled, approximate): {mm(COOLER_LO, COOLER_HI)}",
        f"  Antistatic tray: {mm(TRAY_LO, TRAY_HI)}",
        f"  Mat: {mm(MAT_LO, MAT_HI)}",
        "",
        "Success: tail end >= 5 mm up, fingers >= 10 mm above slot top, card flat",
        "in tray within 10 deg. Template: layout_template_v1.dxf (1:1 mm).",
    ]
    content = ["BT /F1 9 Tf 40 760 Td 11 TL"]
    for line in lines:
        content.append(f"({_pdf_escape(line)}) Tj T*")
    content.append("ET")
    stream = "\n".join(content).encode("latin-1")
    objs = [
        b"1 0 obj<< /Type /Catalog /Pages 2 0 R >>endobj\n",
        b"2 0 obj<< /Type /Pages /Kids [3 0 R] /Count 1 >>endobj\n",
        b"3 0 obj<< /Type /Page /Parent 2 0 R /MediaBox [0 0 612 792] "
        b"/Contents 4 0 R /Resources << /Font << /F1 5 0 R >> >> >>endobj\n",
        f"4 0 obj<< /Length {len(stream)} >>stream\n".encode("latin-1") + stream + b"\nendstream\nendobj\n",
        b"5 0 obj<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>endobj\n",
    ]
    out = bytearray(b"%PDF-1.4\n")
    offsets = []
    for obj in objs:
        offsets.append(len(out))
        out.extend(obj)
    xref = len(out)
    out.extend(f"xref\n0 {len(objs) + 1}\n0000000000 65535 f \n".encode("latin-1"))
    for off in offsets:
        out.extend(f"{off:010d} 00000 n \n".encode("latin-1"))
    out.extend(f"trailer<< /Size {len(objs) + 1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n".encode("latin-1"))
    path.write_bytes(bytes(out))


def main() -> None:
    gauge, wedge = build_gauge(), build_wedge()
    assert check_watertight(gauge) and check_watertight(wedge)
    write_binary_stl(HERE / "tail_lift_gauge_v1.stl", gauge, "tail_lift_gauge_v1")
    write_binary_stl(HERE / "flat_check_wedge_v1.stl", wedge, "flat_check_wedge_v1")
    write_dxf(HERE / "layout_template_v1.dxf")
    write_pdf(HERE / "assembly_bom_v1.pdf")
    print("cad: wrote STL/DXF/PDF under", HERE)


if __name__ == "__main__":
    main()
