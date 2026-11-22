"""Visible released-door checks for the 20 Hz prototype."""

from dataclasses import dataclass
from math import isfinite


SWEEP_MIN_S = 5.0
SWEEP_MAX_S = 8.0
LATCH_MAX_S = 2.0
DWELL_STEPS = 10
FIRST_RELEASE_SAMPLE_MIN_DEG = 87.5


@dataclass
class ClosingScore:
    step: int = 0
    held_open: bool = False
    released_step: int | None = None
    crossed_latch_step: int | None = None
    dwell: int = 0
    invalid: bool = False

    def tick(self, *, angle_deg, holder_engaged, sweep_clear, latch_engaged, fixture_intact):
        """Score timing, then verify the latch stays visibly engaged."""
        self.step += 1
        self.invalid |= not fixture_intact or not isfinite(angle_deg)
        if self.invalid or holder_engaged or not sweep_clear:
            self.released_step = self.crossed_latch_step = None
            self.held_open = holder_engaged and not self.invalid and 89.5 <= angle_deg <= 90.5
            self.dwell = 0
            return 0.0

        if self.held_open:
            self.held_open = False
            # The preceding control proved the held 90-degree start.
            # The first moving sample can be below the hold tolerance.
            if FIRST_RELEASE_SAMPLE_MIN_DEG <= angle_deg <= 90.5:
                self.released_step = self.step

        if self.released_step is None:
            return 0.0
        if self.crossed_latch_step is None:
            if angle_deg > 12.0:
                return 0.0
            self.crossed_latch_step = self.step

        sweep = (self.crossed_latch_step - self.released_step) / 20
        if not 4.0 <= sweep <= SWEEP_MAX_S:
            return 0.0

        latched = latch_engaged and abs(angle_deg) <= 0.5
        elapsed = (self.step - self.crossed_latch_step) / 20
        if elapsed <= LATCH_MAX_S or self.dwell > 0:
            self.dwell = self.dwell + 1 if latched else 0

        if sweep < SWEEP_MIN_S:
            return 0.25 if self.dwell > 0 else 0.0
        return 1.0 if self.dwell >= DWELL_STEPS else 0.5
