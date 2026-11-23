"""Score the visible stop setting, tightening witness and two proof strokes."""
from dataclasses import dataclass
import math


TARGET_M = .026
TOLERANCE_M = .00008
RESET_REGION_M = .005
BACKOFF_TOLERANCE_RAD = math.radians(3.)
DWELL = 10


@dataclass(frozen=True)
class Frame:
    stroke_m: float
    adjusting_key_seated: bool
    original_stop_sleeve_meets_yoke: bool
    calibrated_clamp_witness: bool
    clamp_angle_rad: float
    hands_clear: bool
    proof_advance: bool
    safe: bool


class StrokeScorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.valid = True
        self.adjusted = False
        self.fastening_angle = None
        self.secured = False
        self.proof_armed = False
        self.proof_count = 0
        self.dwell = 0
        self.score = 0.

    def update(self, frame):
        self.valid &= frame.safe and math.isfinite(frame.stroke_m) and math.isfinite(frame.clamp_angle_rad)
        if not self.valid:
            self.score = 0.
            return self.score

        on_mark = abs(frame.stroke_m - TARGET_M) <= TOLERANCE_M
        contact = frame.original_stop_sleeve_meets_yoke and on_mark
        self.adjusted |= frame.adjusting_key_seated and contact and not frame.proof_advance
        if (self.fastening_angle is not None
                and frame.clamp_angle_rad < self.fastening_angle - BACKOFF_TOLERANCE_RAD):
            self.fastening_angle = None
            self.secured = False
            self.proof_armed = False
            self.proof_count = 0
            self.dwell = 0
        if self.adjusted and frame.calibrated_clamp_witness:
            current_angle = self.fastening_angle if self.fastening_angle is not None else frame.clamp_angle_rad
            self.fastening_angle = max(current_angle, frame.clamp_angle_rad)
        self.secured |= self.fastening_angle is not None and frame.hands_clear

        if self.secured and not frame.proof_advance and frame.stroke_m <= RESET_REGION_M:
            self.proof_armed = True
            self.dwell = 0
        proof_holds = frame.proof_advance and frame.hands_clear and contact
        if self.proof_armed:
            self.dwell = self.dwell + 1 if proof_holds else 0
            if self.dwell >= DWELL:
                self.proof_armed = False
                self.proof_count += 1
        if self.proof_count >= 2 and proof_holds:
            self.score = 1.
        elif self.secured:
            self.score = .5
        else:
            self.score = .25 if self.adjusted else 0.
        return self.score
