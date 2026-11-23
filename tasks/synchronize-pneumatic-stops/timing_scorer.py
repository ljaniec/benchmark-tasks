"""Provisional video-equivalent scoring for paired 80 mm pneumatic strokes."""
from dataclasses import dataclass
import math


HOME_M = .003
END_M = .077
MIN_EXTENSION_STEPS = 16
MAX_EXTENSION_STEPS = 30
PARTIAL_LAG_STEPS = 4
MATCH_LAG_STEPS = 2
DWELL_STEPS = 10
CYCLE_TIMEOUT_STEPS = 80


@dataclass(frozen=True)
class Frame:
    """Readings corresponding to marks visible in the test recording."""

    reference_m: float
    adjusted_m: float
    button_pressed: bool
    knob_locked: bool
    reference_unchanged: bool
    safe: bool = True


class TimingScorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.valid = True
        self.step = 0
        self.start = None
        self.arrivals = [None, None]
        self.button_was_pressed = False
        self.remained_locked = False
        self.matches = 0
        self.tier = 0.
        self.dwell = 0

    def update(self, frame):
        self.step += 1
        positions = frame.reference_m, frame.adjusted_m
        self.valid &= (frame.safe and frame.reference_unchanged
                       and all(math.isfinite(x) and -.002 <= x <= .082 for x in positions))
        pressed_now = frame.button_pressed and not self.button_was_pressed
        self.button_was_pressed = frame.button_pressed
        home = all(x <= HOME_M for x in positions)
        if pressed_now and home and self.start is None:
            self.start = self.step
            self.arrivals = [None, None]
            self.remained_locked = frame.knob_locked
        if self.start is not None:
            self.remained_locked &= frame.knob_locked
            for index, distance in enumerate(positions):
                if self.arrivals[index] is None and distance >= END_M:
                    self.arrivals[index] = self.step - self.start
            if home and all(step is not None for step in self.arrivals):
                self._finish_cycle()
            elif self.step - self.start >= CYCLE_TIMEOUT_STEPS:
                self.start = None
                self.matches = 0
                self.tier = 0.
        settled = self.start is None and home and frame.knob_locked and not frame.button_pressed
        self.dwell = self.dwell + 1 if settled else 0
        if not frame.knob_locked:
            self.matches = 0
            self.tier = 0.
        return self.score

    def _finish_cycle(self):
        in_window = all(MIN_EXTENSION_STEPS <= tick <= MAX_EXTENSION_STEPS for tick in self.arrivals)
        lag = abs(self.arrivals[0] - self.arrivals[1])
        matched = self.remained_locked and in_window and lag <= MATCH_LAG_STEPS
        self.matches = self.matches + 1 if matched else 0
        self.tier = .5 if matched else .25 if self.remained_locked and in_window and lag <= PARTIAL_LAG_STEPS else 0.
        self.start = None

    @property
    def score(self):
        if not self.valid:
            return 0.
        if self.matches >= 2 and self.dwell >= DWELL_STEPS:
            return 1.
        return self.tier
