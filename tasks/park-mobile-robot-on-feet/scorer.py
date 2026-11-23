"""Score released support transfer from two wheels to their leveling pads."""
from dataclasses import dataclass
import math


DURATION_CONTROLS = 10
PAD_GAP_M = .001
WHEEL_CLEARANCE_M = .002
PARKING_TOLERANCE_M = .006


@dataclass(frozen=True)
class Frame:
    pad_gaps_m: tuple[float, float]
    wheels_clear_m: tuple[float, float]
    arm_clear: bool
    released_at_rest: bool
    base_offset_m: float
    tilt_deg: float
    blocked_or_moved_by_hand: bool


class SupportTransferScorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.valid = True
        self.dwell = 0

    def update(self, frame):
        measurements = (*frame.pad_gaps_m, *frame.wheels_clear_m, frame.base_offset_m, frame.tilt_deg)
        self.valid &= (len(frame.pad_gaps_m) == len(frame.wheels_clear_m) == 2
                       and all(math.isfinite(x) for x in measurements) and not frame.blocked_or_moved_by_hand
                       and 0. <= frame.base_offset_m <= PARKING_TOLERANCE_M and abs(frame.tilt_deg) <= 15.)
        if not self.valid or not frame.arm_clear or not frame.released_at_rest:
            self.dwell = 0
            return 0.
        supported = sum(abs(pad) <= PAD_GAP_M and wheel >= WHEEL_CLEARANCE_M
                        for pad, wheel in zip(frame.pad_gaps_m, frame.wheels_clear_m, strict=True))
        self.dwell = self.dwell + 1 if supported == 2 else 0
        if self.dwell >= DURATION_CONTROLS:
            return 1.
        return .25 * supported
