"""Judge the retained cutter and its independent crank proof."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Reading:
    seated: bool
    retained: bool
    crank_deg: float
    clear: bool
    valid: bool = True


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.valid = True
        self.seated_ticks = 0
        self.retained_ticks = 0
        self.clear_ticks = 0
        self.proof_start_deg = None
        self.crank_proved = False
        self.score = 0.

    def update(self, reading):
        self.valid &= reading.valid
        seated = self.valid and reading.seated
        retained = seated and reading.retained
        self.seated_ticks = self.seated_ticks + 1 if seated else 0
        self.retained_ticks = self.retained_ticks + 1 if retained else 0
        if not retained:
            self.proof_start_deg = None
            self.crank_proved = False
        elif self.retained_ticks == 10:
            self.proof_start_deg = reading.crank_deg
        elif self.proof_start_deg is not None:
            self.crank_proved |= reading.crank_deg - self.proof_start_deg >= 45.
        complete = retained and self.crank_proved and reading.clear
        self.clear_ticks = self.clear_ticks + 1 if complete else 0
        if self.clear_ticks >= 10:
            self.score = 1.
        elif self.retained_ticks >= 10:
            self.score = .5
        elif self.seated_ticks >= 10:
            self.score = .25
        else:
            self.score = 0.
        return self.score
