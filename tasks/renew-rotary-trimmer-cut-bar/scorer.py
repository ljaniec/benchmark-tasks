"""Grade the fresh cutting surface, its seating and the closed guide."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Reading:
    removed: bool
    fresh_and_aligned: bool
    flush: bool
    guide_closed: bool
    clear: bool
    valid: bool = True


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.score = 0.
        self.valid = True
        self.removed = False
        self.holds = (0, 0, 0)

    def update(self, reading):
        self.valid &= reading.valid
        self.removed |= self.valid and reading.removed
        aligned = self.valid and self.removed and reading.fresh_and_aligned
        complete = aligned and reading.flush and reading.guide_closed and reading.clear
        conditions = (self.removed and self.valid, aligned, complete)
        self.holds = tuple(old + 1 if state else 0 for old, state in zip(self.holds, conditions, strict=True))
        self.score = max((tier for tier, count in zip((.25,.5,1.), self.holds,strict=True) if count >= 10),
                         default=0.)
        return self.score
