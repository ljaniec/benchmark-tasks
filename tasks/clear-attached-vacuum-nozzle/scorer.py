"""Judge extraction through the raised nozzle and the released result."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Reading:
    pulled_through_mouth: bool
    card_in_tray: bool
    sole_down: bool
    released: bool
    valid: bool = True


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.score = 0.
        self.valid = True
        self.extracted = False
        self.tray_ticks = 0
        self.full_ticks = 0

    def update(self, reading):
        self.valid &= reading.valid
        self.extracted |= self.valid and reading.pulled_through_mouth
        collected = self.valid and self.extracted and reading.card_in_tray
        complete = collected and reading.sole_down and reading.released
        self.tray_ticks = self.tray_ticks+1 if collected else 0
        self.full_ticks = self.full_ticks+1 if complete else 0
        partial = .25 if self.valid and self.extracted else 0.
        self.score = 1. if self.full_ticks >= 10 else .5 if self.tray_ticks >= 10 else partial
        return self.score
