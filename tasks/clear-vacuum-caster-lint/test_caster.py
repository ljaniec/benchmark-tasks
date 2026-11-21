from dataclasses import replace
from math import pi
import unittest

from loads import free_advance_rad, report
from proof import CasterProof, Observation, SpinReference


REST = Observation(False, True, False, True, True, 0.)
REFERENCE = SpinReference(4., 12., 2., 10)


def serviced(proof):
    proof.update(replace(REST, module_removed=True, original_parts_seated=False))
    return replace(REST, debris_cleared=True, hands_clear=False, angle_rad=2.9)


class CasterTests(unittest.TestCase):
    def test_service_sequence_and_released_spin_through_angle_wrap(self):
        proof = CasterProof(REFERENCE)
        self.assertEqual(proof.update(REST), 0.)
        self.assertEqual(proof.update(replace(REST, module_removed=True, original_parts_seated=False)), .25)
        clean = replace(REST, debris_cleared=True, hands_clear=False, angle_rad=2.9)
        self.assertEqual(proof.update(clean), .5)
        for tick in range(1, 11):
            angle = (2.9 + .3 * tick + pi) % (2. * pi) - pi
            score = proof.update(replace(clean, hands_clear=True, angle_rad=angle))
            self.assertEqual(score, 1. if tick == 10 else .5)
        self.assertEqual(proof.update(replace(clean, hands_clear=True, original_parts_seated=False)), .5)

    def test_held_spin_and_short_coast_do_not_pass(self):
        proof = CasterProof(REFERENCE)
        clean = serviced(proof)
        for tick in range(30):
            self.assertEqual(proof.update(replace(clean, angle_rad=tick * .3)), .5)
        for tick in range(30):
            angle = (29 + min(tick + 1, 3)) * .3
            self.assertEqual(proof.update(replace(clean, hands_clear=True, angle_rad=angle)), .5)

    def test_overpowered_flick_is_excluded(self):
        proof = CasterProof(REFERENCE)
        clean = serviced(proof)
        proof.update(clean)
        for tick in range(15):
            self.assertEqual(proof.update(replace(clean, hands_clear=True, angle_rad=3.8 + .2 * tick)), .5)

    def test_slow_accumulation_cannot_extend_the_spin_window(self):
        proof = CasterProof(REFERENCE)
        clean = serviced(proof)
        proof.update(clean)
        for tick in range(1, 31):
            angle = clean.angle_rad + .3 + .10 * (tick - 1)
            self.assertEqual(proof.update(replace(clean, hands_clear=True, angle_rad=angle)), .5)

    def test_lost_part_or_damage_zeros_credit(self):
        proof = CasterProof(REFERENCE)
        clean = serviced(proof)
        self.assertEqual(proof.update(replace(clean, axle_and_wheel_intact=False)), 0.)
        self.assertEqual(proof.update(clean), 0.)

    def test_sensitivity_distinguishes_a_loaded_detent_and_stalled_wheel(self):
        model = report()
        for row in model['detent_forces_n'].values():
            self.assertGreater(row['release_52_deg'], row['seat_25_deg'])
        self.assertGreater(free_advance_rad(8., .00001, .5, 4e-6),
                           free_advance_rad(8., .00010, .5, 4e-6))


if __name__ == '__main__':
    unittest.main()
