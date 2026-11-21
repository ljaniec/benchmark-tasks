"""Shared full-size vacuum chassis, receiver and caster dimensions."""

import math

import numpy as np

import mechkit as m


SOCKET = np.array([.358, 0., .823])
TIRE_OFFSET = np.array([0., 0., .0235])
BODY_CENTRE = (.487, 0.)
BODY_RADIUS = .1684
WHEEL_RADIUS = .0165
FORK_WIDTH = .036
MAT_CENTRE = np.array([.331, -.196, .752])
MAT_SIZE = np.array([.165, .084, .004])
CASE = 'case'
POLYMER = 'polymer'
RUBBER = 'rubber'
METAL = 'metal'
LOOKS = {
    CASE: m.Look(color=m.rgb(.67, .69, .70), roughness=.67),
    POLYMER: m.Look(color=m.rgb(.21, .25, .27), roughness=.65),
    'fork': m.Look(color=m.rgb(.43, .46, .48), roughness=.61),
    RUBBER: m.Look(color=m.rgb(.075, .077, .081), roughness=.87),
    'brush': m.Look(color=m.rgb(.22, .54, .47), roughness=.86),
    METAL: m.Look(color=m.rgb(.72, .71, .70), roughness=.34),
    'mat': m.Look(color=m.rgb(.17, .32, .37), roughness=.85),
}


def part(name, mesh, appearance):
    collision = m.col_hull(name + '_collision', mesh).replace('contactOffset = 0.004', 'contactOffset = 0.00035')
    return (name, mesh, appearance), collision


def split_parts(items):
    return tuple(zip(*items, strict=True))


def ring(name, centre, inner, outer, low, high, appearance):
    for i in range(16):
        angles = np.array([i, i + 1]) * math.tau / 16.
        radial = np.c_[np.cos(angles), np.sin(angles)]
        points = np.vstack([outer * radial, inner * radial[::-1]]) + centre
        yield part(f'{name}_{i:02}', m.prism(points, low, high), appearance)


def platform():
    # Full-size chassis shell. The open socket and actual front-wheel access
    # are retained while cosmetic underside features are simplified.
    body = [
        part('shell', m.cylinder(BODY_RADIUS, .051, (*BODY_CENTRE, .7755)), CASE),
        part('rear_deck', m.box((.185, .240, .020), (.525, 0., .812)), POLYMER),
        part('left_motor_wheel', m.cylinder(.028, .018).aimed('+Y').moved((.475, .120, .828)), RUBBER),
        part('right_motor_wheel', m.cylinder(.028, .018).aimed('+Y').moved((.475, -.120, .828)), RUBBER),
        part('setdown_mat', m.box(MAT_SIZE, MAT_CENTRE), 'mat'),
    ]
    body.extend(ring('seat_rim', SOCKET[:2], .0070, .0190, .819, SOCKET[2], POLYMER))
    # The flexible detent needs a relief behind the seat collar. A solid
    # ring of the seat diameter through the lower well would trap the tabs.
    body.extend(ring('detent_relief', SOCKET[:2], .0100, .0190, .803, .819, POLYMER))
    visuals, collisions = split_parts(body)
    # The roller door and cliff sensor windows are fixed during front-caster
    # service. Their visible shapes make the underside recognizable.
    details = [
        (f'brush_roll_{n}', m.cylinder(.008, .134, (x, 0., .819), axis='y'), 'brush')
        for n, x in enumerate((.486, .508))
    ]
    details.extend(
        (f'cliff_window_{n}', m.box((.017, .012, .001), (.357, y, .801)), RUBBER)
        for n, y in enumerate((-.093, .093))
    )
    return m.fixed('vacuum_and_mat', (0., 0., 0.), [*visuals, *details], collisions, LOOKS)


def detent():
    links = []
    for sign, name in ((-1., 'left'), (1., 'right')):
        deck = part(name + '_finger', m.box((.0026, .008, .0022), (sign * .0053, 0., -.008)), POLYMER)
        visuals, collisions = split_parts([deck])
        low, high = (0., .0018) if sign > 0 else (-.0018, 0.)
        links.append(m.slider(name, 'X', low, high, visuals, collisions, mass=.0028,
                              spring=4500., damping=1.5, friction=.06))
    return m.mechanism('detent_receiver', SOCKET, links, LOOKS, friction=(.26, .22))


def pin_collar():
    angles = np.arange(16) * math.tau / 16.
    radial = np.c_[np.cos(angles), np.sin(angles)]
    stations = ((.0031, -.0164), (.0047, -.0124), (.0047, -.0116), (.00345, -.0100))
    vertices = np.vstack([np.c_[r * radial, np.full(16, z)] for r, z in stations])
    faces = []
    for k in range(len(stations) - 1):
        for i in range(16):
            a = k * 16 + i
            b = k * 16 + (i + 1) % 16
            faces.extend(((a, b, b + 16), (a, b + 16, a + 16)))
    end = (len(stations) - 1) * 16
    for i in range(1, 15):
        faces.extend(((0, i + 1, i), (end, end + i, end + i + 1)))
    return part('pin_collar', m.Mesh(vertices, np.array(faces)), METAL)
