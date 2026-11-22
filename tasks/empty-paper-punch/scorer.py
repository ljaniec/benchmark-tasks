"""Score a released punch and scraps left in the intended waste tray."""
from dataclasses import dataclass


TOTAL_DISKS = 48
RELEASE_CLEARANCE_M = .035
HOLD_TICKS = 10


@dataclass(frozen=True)
class Outcome:
    in_tray: int
    in_chamber: int
    hands_clearance_m: float
    cover_engaged: bool
    flat_on_desk_pad: bool
    parts_intact: bool
    within_workspace: bool
    still: bool


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.score = 0.
        self._dwell = 0
        self._grade = 0.
        self._valid = True

    def update(self, state: Outcome) -> float:
        self._valid &= state.parts_intact and state.within_workspace
        accounted_for = (0 <= state.in_tray <= TOTAL_DISKS and 0 <= state.in_chamber <= TOTAL_DISKS
                         and state.in_tray + state.in_chamber == TOTAL_DISKS)
        released = state.hands_clearance_m >= RELEASE_CLEARANCE_M and state.still
        grade = 0.
        if self._valid and accounted_for and released:
            if state.in_tray == TOTAL_DISKS:
                grade = 1. if state.flat_on_desk_pad and state.cover_engaged else .5
            elif state.in_tray >= TOTAL_DISKS // 2:
                grade = .25
        if grade == 0.:
            self._dwell = 0
        elif grade != self._grade:
            self._dwell = 1
        else:
            self._dwell += 1
        self._grade = grade
        self.score = grade if self._dwell >= HOLD_TICKS else 0.
        return self.score
