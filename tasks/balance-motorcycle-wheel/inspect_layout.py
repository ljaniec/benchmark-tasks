"""Inspectable placement of the native wheel, tools, and robot support posts."""

from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import mechkit as m
from yam_arena import ik, station

from press_shoe import SHOE_MASS_KG, shoe_boxes
from probe_strip_press import RADIAL_M, STRIP_KG, TANGENT_M, THICK_M
from wheel_index_rgb import calibrated_lens


HERE = Path(__file__).resolve().parent
FIXTURE_ASSETS = HERE / 'service-wheel-rim-index-v6-assets'
COLORS = {
    'support': m.Look(color=m.rgb(.22, .28, .31), roughness=.65),
    'strip': m.Look(color=m.rgb(.61, .64, .66), roughness=.42, metallic=.22),
    'shoe': m.Look(color=m.rgb(.17, .50, .58), roughness=.71),
}


def overhead_camera(camera, configuration='delivery_candidate'):
    calibration = json.loads((HERE / 'camera-projection.json').read_text())[configuration]
    orientation = np.asarray(calibration['camera_axes_columns'])
    return replace(camera, pos=tuple(calibration['camera_m']),
                   rot_xyzw=tuple(ik.matrix_to_quat_xyzw(orientation)),
                   lens=calibrated_lens(camera.lens, calibration))


def wheel_fixture():
    """Preserve the tested native rigid wheel and passive world revolute joint."""
    asset_source = FIXTURE_ASSETS / 'assembly.usda'
    configured = asset_source.read_text()
    wheel_line = '    def Xform "Wheel" (prepend references = @wheel.usda@) {}'
    if configured.count(wheel_line) != 1:
        raise ValueError('The probed wheel fixture has changed its composition')
    mount = f'''    def Xform "Wheel" (
        prepend references = @{FIXTURE_ASSETS / 'wheel.usda'}@
        prepend apiSchemas = ["PhysxContactReportAPI"]
    )
    {{
        float xformOp:rotateY = 90
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateY"]
        float physxContactReport:threshold = 0
    }}'''
    configured = configured.replace(wheel_line, mount)
    configured = configured.replace('@stand.usda@', f"@{FIXTURE_ASSETS / 'stand.usda'}@")
    return m.Part('service_stand', pos=(0., 0., 0.), role='fixed', build=lambda: configured)


def _support_geometry(plan):
    blocks = [(f'{side}_{name}', center, size) for side, parts in plan['fork_slots'].items()
              for name, center, size in parts]
    floor = station.FLOOR_Z
    for side, (x, y, base_z) in station.BASES.items():
        blocks.append((f'{side}_robot_pedestal', (x, y, (base_z + floor) / 2.),
                       (.174, .146, base_z - floor)))
    deck_underside = .415
    for i, (x, y) in enumerate(((.05, -.35), (.05, .35), (.85, -.35), (.85, .35))):
        blocks.append((f'deck_leg_{i}', (x, y, (deck_underside + floor) / 2.),
                       (.058, .058, deck_underside - floor)))
    mount_base = station.TABLE_TOP
    blocks.append(('camera_pedestal', (.87, .42, (mount_base + floor) / 2.),
                   (.085, .085, mount_base - floor)))
    visual = [(name, m.box(extent, center), 'support') for name, center, extent in blocks]
    collision = [m.col_box(name + '_shape', extent, center) for name, center, extent in blocks]
    return m.fixed('workshop_supports', (0., 0., 0.), visual, collision, COLORS)


def _free_part(name, center, quaternion, components, material, mass, offset):
    visual = [(tag, m.box(extent, local), material) for tag, local, extent in components]
    collision = [m.col_box(tag + '_shape', extent, local) for tag, local, extent in components]
    part = m.free(name, center, visual, collision, COLORS, mass=mass, start_quat=tuple(quaternion),
                  torsional_patch=None, friction=(.65, .5), solver_iters=16)
    build = part.build

    def with_contact_report():
        configured = build().replace('"PhysicsMassAPI"]', '"PhysicsMassAPI", "PhysxContactReportAPI"]', 1)
        return configured.replace('\n{\n', '\n{\n    float physxContactReport:threshold = 0\n', 1)

    return replace(part, build=with_contact_report, contact_offset=offset)


def task_objects(plan):
    parts = [wheel_fixture(), _support_geometry(plan)]
    strip_shape = (('strip', (0., 0., 0.), (RADIAL_M, THICK_M, TANGENT_M)),)
    for side, center in plan['strip_start_m'].items():
        rotation = plan['strip_start_quat'][side]
        parts.append(_free_part(f'strip_{side}', center, rotation, strip_shape, 'strip', STRIP_KG, .0005))
        if plan.get('press_shoes', False):
            parts.append(_free_part(f'shoe_{side}', center, rotation, shoe_boxes(), 'shoe', SHOE_MASS_KG, .0004))
    return parts
