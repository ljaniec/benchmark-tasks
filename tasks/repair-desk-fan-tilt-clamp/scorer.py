"""Judge the fan's aim and its released support under the weight of the head."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Reading:
    pitch_deg: float
    steady: bool
    proof_supported: bool
    hands_clear: bool
    valid: bool


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.valid = True
        self.ticks = 0
        self.score = 0.

    def update(self, reading):
        self.valid &= reading.valid
        on_aim = abs(reading.pitch_deg) <= 2.
        free_and_still = self.valid and on_aim and reading.steady and reading.proof_supported and reading.hands_clear
        self.ticks = self.ticks + 1 if free_and_still else 0
        raised = -12. <= reading.pitch_deg <= 6.
        self.score = (0. if not self.valid else 1. if self.ticks >= 10 else
                      .5 if on_aim else .25 if raised else 0.)
        return self.score
