"""Diagnostic controller for two independently metered pneumatic stops."""
from flow_estimate import FAULT_TURNS, REFERENCE_TURNS, extension_speed_mps
from timing_scorer import HOME_M


STROKE_M = .080
CONTROL_HZ = 20
RETURN_SPEED_MPS = .085
RETRIGGER_DELAY_STEPS = 10
TEST_EXTENSION_STEPS = 40


class TestCycle:
    """Sample the mechanically set flow when a physical test button is pressed."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.elapsed = None
        self.button_was_pressed = False
        self.speeds = (0., 0.)
        self.targets = (0., 0.)

    def update(self, button_pressed, knob_locked, dial_degrees, rod_positions):
        rising_edge = button_pressed and not self.button_was_pressed
        self.button_was_pressed = button_pressed
        home = all(position <= HOME_M for position in rod_positions)
        if self.elapsed is None:
            if rising_edge and knob_locked and home:
                fault_turns = FAULT_TURNS + dial_degrees / 360.
                self.speeds = (extension_speed_mps(REFERENCE_TURNS), extension_speed_mps(fault_turns))
                self.elapsed = 0
            return self.targets

        self.elapsed += 1
        if self.elapsed <= TEST_EXTENSION_STEPS:
            self.targets = tuple(min(STROKE_M, speed * self.elapsed / CONTROL_HZ) for speed in self.speeds)
        else:
            distance = RETURN_SPEED_MPS * (self.elapsed - TEST_EXTENSION_STEPS) / CONTROL_HZ
            self.targets = tuple(max(0., min(STROKE_M, speed * TEST_EXTENSION_STEPS / CONTROL_HZ) - distance)
                                 for speed in self.speeds)
            if home and self.targets == (0., 0.) and self.elapsed >= TEST_EXTENSION_STEPS + RETRIGGER_DELAY_STEPS:
                self.elapsed = None
        return self.targets
