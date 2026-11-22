"""Grade the visible obstruction, cleared bore and released tools."""
from dataclasses import dataclass


HOLD_TICKS = 10
TIERS = (.25, .5, 1.)


@dataclass(frozen=True)
class Outcome:
    visible_at_mouth: bool
    debris_in_tray: bool
    tube_on_rest: bool
    rod_on_rest: bool
    hands_clear: bool
    still: bool
    intact: bool
    inside_workspace: bool


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.score = 0.
        self._holds = (0, 0, 0)
        self._valid = True

    def update(self, result):
        self._valid &= result.intact and result.inside_workspace
        caught = self._valid and result.debris_in_tray
        protruding = self._valid and result.visible_at_mouth and not caught
        finished = caught and all((result.tube_on_rest, result.rod_on_rest, result.hands_clear, result.still))
        self._holds = tuple(hold + 1 if qualifies else 0
                            for hold, qualifies in zip(self._holds, (protruding, caught, finished), strict=True))
        self.score = max((tier for tier, hold in zip(TIERS, self._holds, strict=True) if hold >= HOLD_TICKS),
                         default=0.)
        return self.score
