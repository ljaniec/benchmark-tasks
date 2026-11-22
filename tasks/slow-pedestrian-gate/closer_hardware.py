"""Colliding hex socket and driver for a first native contact test."""

import numpy as np

import mechkit as m

from gate_leaf import HINGE


CAP_Z = 0.844
SPEED_CENTRE = (0.212, -0.317, CAP_Z)
SNAP_CENTRE = (0.229, -0.343, CAP_Z)
SOCKET_DEPTH = .005
KEY_FLATS = .003
SOCKET_FLATS = .0032
SOCKET_LEAD_DEPTH = .00135
SOCKET_MOUTH_FLATS = .0048
KEY_LEAD_LENGTH = .00075
EXTENSION_DIAMETER = .00635
TIP_TO_GRIP_M = .203

LOOKS = {
    "housing": m.Look(color=m.rgb(.43, .48, .49), roughness=.55),
    "steel": m.Look(color=m.rgb(.75, .77, .79), roughness=.36, metallic=.15),
    "grip": m.Look(color=m.rgb(.16, .32, .43), roughness=.76),
}


def hex_points(flats):
    theta = np.linspace(0, 2 * np.pi, 6, endpoint=False) + np.pi / 6
    radius = flats / np.sqrt(3)
    return radius * np.c_[np.cos(theta), np.sin(theta)]


def tapered_prism(lower, upper, z0, z1):
    """Join two matching outlines so the contact surface has a real bevel."""
    mesh = m.prism(lower, z0, z1)
    vertices = mesh.v.copy()
    count = len(lower)
    vertices[count:2 * count, :2] = upper
    vertices[-1, :2] = np.mean(upper, axis=0)
    return m.Mesh(vertices, mesh.f)


def hex_socket(centre, name):
    """Six physical walls around the unfilled cavity."""
    inner = hex_points(SOCKET_FLATS)
    mouth = hex_points(SOCKET_MOUTH_FLATS)
    outer = hex_points(.0084)
    parts = []
    for i in range(6):
        j = (i + 1) % 6
        rim = [inner[i], outer[i], outer[j], inner[j]]
        lip = [mouth[i], outer[i], outer[j], mouth[j]]
        wall = m.prism(rim, -SOCKET_DEPTH, -SOCKET_LEAD_DEPTH)
        bevel = tapered_prism(rim, lip, -SOCKET_LEAD_DEPTH, 0.)
        parts.extend([
            (f"{name}_wall_{i}", wall.moved(centre), "steel"),
            (f"{name}_lead_{i}", bevel.moved(centre), "steel"),
        ])
    return parts


def narrow_hull(name, mesh):
    return m.col_hull(name, mesh).replace("contactOffset = 0.004", "contactOffset = 0.00012")


def hydraulic_closer():
    speed = hex_socket(SPEED_CENTRE, "speed")
    fixed_snap = hex_socket(SNAP_CENTRE, "snap")
    # The main body backs the sockets without filling their key wells.
    body_top = CAP_Z - SOCKET_DEPTH
    housing = m.cylinder(.0235, .265, (*HINGE[:2], body_top - .265 / 2))
    fixed = [("hydraulic_body", housing, "housing"), *fixed_snap]
    speed_link = m.hinge("speed", "Z", SPEED_CENTRE, -270.0, 0.0, speed,
                         [narrow_hull(name + "_shape", mesh) for name, mesh, _ in speed],
                         mass=.004, friction=.012, damping=.002,
                         effort=.15, armature=.000001)
    fixed_collision = [narrow_hull(name + "_shape", mesh) for name, mesh, _ in fixed]
    return m.mechanism("hydraulic_closer", (0., 0., 0.), [speed_link], LOOKS,
                       frame_parts=fixed, frame_colliders=fixed_collision, solver_iters=64)


def hex_driver():
    grip = m.box((.026, .015, .040), (0., 0., TIP_TO_GRIP_M))
    key_section = hex_points(KEY_FLATS)
    key_lead = tapered_prism(hex_points(.00255), key_section, 0., KEY_LEAD_LENGTH)
    bit = m.prism(key_section, KEY_LEAD_LENGTH, .007)
    extension_length = TIP_TO_GRIP_M - .013 - .007
    extension = m.cylinder(EXTENSION_DIAMETER / 2, extension_length,
                           (0., 0., .007 + extension_length / 2))
    visuals = [("grip", grip, "grip"), ("bit", bit, "steel"),
               ("key_lead", key_lead, "steel"), ("extension", extension, "steel")]
    return m.free("hex_driver", (.12, -.49, .76), visuals,
                  [narrow_hull(name + "_shape", mesh) for name, mesh, _ in visuals],
                  LOOKS, mass=.072, friction=(.84, .67), solver_iters=64)


if __name__ == "__main__":
    from pathlib import Path
    from pxr import Sdf

    for asset in (hydraulic_closer(), hex_driver()):
        source = asset.build()
        layer = Sdf.Layer.CreateAnonymous()
        assert layer.ImportFromString(source), f"Invalid USD for {asset.name}"
        Path(__file__).with_name(asset.name + ".usda").write_text(source)
