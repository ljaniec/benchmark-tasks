"""Observable release checks for the monitor arm. Operates at 20 Hz."""
from dataclasses import dataclass, field
import math


HOLD_HEIGHTS_M = (-.062, .062)
HEIGHT_TOLERANCE_M = .012
DRIFT_LIMIT_M = .005
SHORT_HOLD = 10
LONG_HOLD = 40


@dataclass
class ReleaseScore:
    held_heights: set[int] = field(default_factory=set)
    height_index: int | None = None
    min_height: float = 0.
    max_height: float = 0.
    steps: int = 0
    fault: bool = False
    value: float = 0.

    def update(self, height_m, hands_clear, key_in_holder, fixture_valid):
        self.fault |= not math.isfinite(height_m) or not fixture_valid
        if self.fault:
            self.value = 0.
            return 0.

        band = next((i for i, level in enumerate(HOLD_HEIGHTS_M)
                     if abs(height_m - level) <= HEIGHT_TOLERANCE_M), None)
        if band is None or not hands_clear or not key_in_holder:
            self.height_index = None
            self.steps = 0
        elif self.height_index != band:
            self.height_index = band
            self.min_height = self.max_height = height_m
            self.steps = 1
        else:
            self.min_height = min(self.min_height, height_m)
            self.max_height = max(self.max_height, height_m)
            if self.max_height - self.min_height > DRIFT_LIMIT_M:
                self.min_height = self.max_height = height_m
                self.steps = 1
            else:
                self.steps += 1

        if self.steps >= LONG_HOLD:
            self.held_heights.add(self.height_index)
        self.value = .5 if self.held_heights else (.25 if self.steps >= SHORT_HOLD else 0.)
        if self.steps >= LONG_HOLD and self.held_heights == {0, 1}:
            self.value = 1.
        return self.value
