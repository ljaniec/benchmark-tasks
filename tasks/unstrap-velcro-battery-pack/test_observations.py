"""Check the actual policy boundary without starting the simulator."""
import unittest
from types import SimpleNamespace

try:
    import numpy as np
    from inspect_task import BatteryStrap, RGBDemo, NAMES, STATE_KEY
    AVAILABLE = True
except ModuleNotFoundError:
    AVAILABLE = False


@unittest.skipUnless(AVAILABLE, 'Run with the documented Inspect Python overlay')
class ObservationBoundaryTests(unittest.TestCase):
    def make_embodiment(self, robot_state):
        env = BatteryStrap.__new__(BatteryStrap)
        env.robot_state = robot_state
        env.instruction = 'Remove the battery.'
        env.world = SimpleNamespace(
            steps=12,
            images=lambda: {name: np.zeros((176,320,3),dtype=np.uint8) for name in NAMES.values()},
            joints=lambda: np.arange(14,dtype=np.float32),
            # Evaluator values exist in the same world but must not be forwarded.
            score=1., object_pose=np.ones(7), contact_force=40.)
        return env

    def test_dream_receives_only_rgb_and_robot_readings(self):
        obs = self.make_embodiment(True).observation()
        self.assertEqual(set(obs.images), set(NAMES))
        self.assertEqual(set(obs.state), {STATE_KEY})
        np.testing.assert_array_equal(obs.state[STATE_KEY], np.arange(14))
        self.assertEqual(obs.state_time, .6)
        self.assertEqual(set(obs.image_times.values()), {.6})
        self.assertTrue(all(im.shape==(176,320,3) for im in obs.images.values()))

    def test_rgb_demo_boundary_rejects_joint_state(self):
        env = self.make_embodiment(False)
        def forbidden():
            raise AssertionError('RGB demonstration requested robot state')
        env.world.joints = forbidden
        self.assertFalse(env.observation().state)
        policy = RGBDemo()
        with self.assertRaisesRegex(ValueError, 'Unexpected state'):
            policy.act(self.make_embodiment(True).observation())


if __name__ == '__main__':
    unittest.main()
