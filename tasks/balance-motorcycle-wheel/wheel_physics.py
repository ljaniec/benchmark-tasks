"""Apply the applicator and adhesive forces at native physics frequency."""

from dataclasses import asdict

import numpy as np

from mount_witness import rim_fit, seated_on_rim
from press_shoe import magnetic_reaction
from probe_strip_press import DT, PRESS_SUBSTEPS, measure_patch, read_contacts
from tape_bond import TapeBond


SIDES = ('left', 'right')
WHEEL_GLOB = '/World/envs/env_0/*/Wheel'
PROPS = ('strip_left', 'strip_right', 'shoe_left', 'shoe_right')


class WheelPhysics:
    """Read native contacts and apply paired forces without setting any body pose."""

    def __init__(self, base):
        import warp as wp
        from isaaclab_physx.physics import PhysxManager
        from yam_arena.urdf import quat_xyzw_to_matrix

        self.wp = wp
        self.rotation = quat_xyzw_to_matrix
        self.device = base.device
        if abs(base.physics_dt - DT) > 1e-8:
            raise ValueError('Wheel adhesive requires 240 Hz physics')

        backend = PhysxManager.get_physics_sim_view()
        wheel = backend.create_rigid_body_view(WHEEL_GLOB)
        if wheel.count != 1:
            raise RuntimeError(f'Expected one wheel, found {wheel.prim_paths}')
        wheel_path = wheel.prim_paths[0]
        paths = [wheel_path, *(base.scene[name].root_view.prim_paths[0] for name in PROPS)]
        self.bodies = backend.create_rigid_body_view(paths)
        if set(self.bodies.prim_paths) != set(paths) or self.bodies.count != len(paths):
            raise RuntimeError(f'Wheel and carrier body views differ from the fixture: {self.bodies.prim_paths}')
        self.order = [self.bodies.prim_paths.index(path) for path in paths]
        self.force_ids = wp.array(list(range(len(paths))), dtype=wp.int32, device=self.device)
        self.contacts = [backend.create_rigid_contact_view(path, filter_patterns=[wheel_path],
                                                          max_contact_data_count=64) for path in paths[1:3]]
        if any(contact.sensor_count != 1 or contact.filter_count != 1 for contact in self.contacts):
            raise RuntimeError('Expected one wheel-pressure filter per strip')

        self.substeps = 0
        self.streak = [0, 0]
        self.longest_streak = [0, 0]
        self.peak_n = [0., 0.]
        self.peak_spans_m = [(0., 0.), (0., 0.)]
        self.bond = [None, None]
        self.first_bond = [None, None]
        self.break_count = [0, 0]

    def poses(self):
        return self.bodies.get_transforms().numpy().reshape(5, 7)[self.order]

    def apply_loads(self):
        poses = self.poses()
        rotations = [self.rotation(row[3:]) for row in poses]
        loads = np.zeros((5, 3), dtype=np.float32)
        torques = np.zeros_like(loads)
        for strip in (1, 2):
            for shoe in (3, 4):
                force = magnetic_reaction(poses[strip, :3], rotations[strip], poses[shoe, :3], rotations[shoe])
                loads[self.order[strip]] += force
                loads[self.order[shoe]] -= force
        if any(self.bond):
            velocities = self.bodies.get_velocities().numpy().reshape(5, 6)[self.order]
            for i, bond in enumerate(self.bond):
                if bond is None:
                    continue
                strip = i + 1
                wrench = bond.reaction(poses[0, :3], rotations[0], velocities[0],
                                       poses[strip, :3], rotations[strip], velocities[strip])
                if wrench is None:
                    self.bond[i] = None
                    self.streak[i] = 0
                    self.break_count[i] += 1
                    continue
                force, torque = wrench
                loads[self.order[strip]] += force
                loads[self.order[0]] -= force
                torques[self.order[strip]] += torque
                torques[self.order[0]] -= torque + np.cross(poses[strip, :3] - poses[0, :3], force)

        wp = self.wp
        forces = wp.array(loads.ravel(), dtype=wp.float32, device=self.device)
        twists = wp.array(torques.ravel(), dtype=wp.float32, device=self.device)
        self.bodies.apply_forces_and_torques_at_position(forces, twists, None, self.force_ids, True)

    def sample_contact(self):
        self.substeps += 1
        if all(self.bond):
            return
        poses = self.poses()
        wheel, wheel_rotation = poses[0, :3], self.rotation(poses[0, 3:])
        for i, contact in enumerate(self.contacts):
            if self.bond[i] is not None:
                continue
            strip = poses[i + 1, :3]
            rotation = self.rotation(poses[i + 1, 3:])
            side = 1. if (strip - wheel) @ wheel_rotation[:, 1] >= 0. else -1.
            points, normals, loads = read_contacts(contact)
            force, spans, qualifies = measure_patch(points, normals, loads, strip, rotation, wheel, wheel_rotation, side)
            if force > self.peak_n[i]:
                self.peak_n[i] = force
                self.peak_spans_m[i] = tuple(map(float, spans))
            qualifies = qualifies and seated_on_rim(wheel, wheel_rotation, strip, rotation, side)
            self.streak[i] = self.streak[i] + 1 if qualifies else 0
            self.longest_streak[i] = max(self.longest_streak[i], self.streak[i])
            if self.streak[i] >= PRESS_SUBSTEPS:
                self.bond[i] = TapeBond.at_contact(wheel, wheel_rotation, strip, rotation, side)
                if self.first_bond[i] is None:
                    self.first_bond[i] = self.substeps

    def snapshot(self):
        poses = self.poses()
        wheel_rotation = self.rotation(poses[0, 3:])
        index = wheel_rotation[:, 0]
        result = {
            'substeps': self.substeps,
            'index_deg': float(np.degrees(np.arctan2(index[2], index[0])) % 360.),
            'parts': {},
        }
        for i, side in enumerate(SIDES):
            strip, shoe = poses[i + 1], poses[i + 3]
            fit = rim_fit(poses[0, :3], wheel_rotation, strip[:3], self.rotation(strip[3:]),
                          1. if side == 'left' else -1.)
            result['parts'][side] = {
                'rim_fit': asdict(fit),
                'seated': fit.seated,
                'strip_m': strip[:3].tolist(),
                'shoe_m': shoe[:3].tolist(),
                'clearance_m': float(np.linalg.norm(strip[:3] - shoe[:3])),
                'peak_n': self.peak_n[i],
                'peak_spans_m': self.peak_spans_m[i],
                'longest_pressure_streak': self.longest_streak[i],
                'bond_active': self.bond[i] is not None,
                'first_bond_substep': self.first_bond[i],
                'breaks': self.break_count[i],
            }
        return result
