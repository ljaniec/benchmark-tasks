"""Evaluate one guarded two-blank confirmation run from its visible events."""
from dataclasses import dataclass
import math


CONTROL_HZ = 20
SEPARATION_STEPS = 12
DWELL_STEPS = 10
FRONT_BEAM_M = .049
GATE_EXIT_M = -.010
BLOCKED_STOCK_LIMIT_M = .014


@dataclass(frozen=True)
class Frame:
    start_pressed: bool
    motor_on: bool
    blanks_aligned_at_load: bool
    safe: bool
    lower_front_m: float
    lower_rear_m: float
    upper_front_m: float
    output_stationary: bool


class FeedScorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.step = 0
        self.dwell = 0
        self.started = False
        self.start_was_pressed = False
        self.valid = True
        self.lower_passage = None
        self.upper_passage = None
        self.held_at_gate = False
        self.result = 0.

    def update(self, frame):
        self.step += 1
        self.valid &= frame.safe and all(math.isfinite(x) for x in
                                        (frame.lower_front_m, frame.lower_rear_m, frame.upper_front_m))
        start = frame.start_pressed and not self.start_was_pressed
        self.start_was_pressed = frame.start_pressed
        if start and not self.started:
            self.started = frame.blanks_aligned_at_load
        if not self.valid or not self.started:
            self.result = 0.
            return self.result
        if frame.motor_on:
            if self.lower_passage is None and frame.lower_front_m >= FRONT_BEAM_M:
                self.lower_passage = self.step
            if self.lower_passage is not None and self.upper_passage is None and frame.lower_rear_m >= GATE_EXIT_M:
                self.held_at_gate |= frame.upper_front_m <= BLOCKED_STOCK_LIMIT_M
            if self.upper_passage is None and frame.upper_front_m >= FRONT_BEAM_M:
                self.upper_passage = self.step
        if self.upper_passage is not None and (
            self.lower_passage is None or self.upper_passage - self.lower_passage < SEPARATION_STEPS
        ):
            self.valid = False
            self.result = 0.
            return self.result
        both_fed = self.held_at_gate and self.upper_passage is not None
        self.dwell = self.dwell + 1 if both_fed and frame.output_stationary and not frame.motor_on else 0
        self.result = 0.
        if self.held_at_gate:
            self.result = .25
        if both_fed:
            self.result = .5
        if self.dwell >= DWELL_STEPS:
            self.result = 1.
        return self.result
