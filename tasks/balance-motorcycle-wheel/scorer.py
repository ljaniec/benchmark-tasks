"""Score filmed balance releases from an intact wheel and its two strips."""

from dataclasses import dataclass
from math import isfinite


MARKS_DEG = (0., 45., 90., 135.)
HOLD_STEPS = 10
RELEASE_STEPS = 90
END_DWELL_STEPS = 10


def offset_deg(a, b):
    return (a - b + 180.) % 360. - 180.


@dataclass(frozen=True)
class Reading:
    angle_deg: float | None
    left_strip: bool
    right_strip: bool
    hands_clear: bool
    invalid: bool = False


@dataclass
class Release:
    index: int
    start_deg: float
    steps: int = 0
    peak_deg: float = 0.


class WheelScore:
    """Use the 20 Hz wheel index and visible attachment and clearance witnesses."""

    def __init__(self):
        self.invalid = False
        self.attached = (False, False)
        self.attachment_angle = None
        self.moved_strip = False
        self.held = []
        self.release = None
        self.checks = {}

    def update(self, reading):
        angle = reading.angle_deg
        self.invalid |= reading.invalid or (angle is not None and not isfinite(angle))
        if self.invalid:
            return 0.

        attached = (reading.left_strip, reading.right_strip)
        if attached != self.attached:
            self.attached = attached
            self.attachment_angle = angle
            self.moved_strip = False
            self.held.clear()
            self.release = None
            self.checks.clear()

        if angle is None:
            self.held.clear()
            self.release = None
            return 0.

        if self.attachment_angle is None:
            self.attachment_angle = angle
        self.moved_strip |= any(attached) and abs(offset_deg(angle, self.attachment_angle)) >= 20.
        partial = .25 if self.moved_strip and reading.hands_clear else 0.

        if not all(attached):
            return partial
        if not reading.hands_clear:
            self.release = None
            self.held.append(angle)
            self.held = self.held[-HOLD_STEPS:]
        elif self.held:
            index = min(range(len(MARKS_DEG)), key=lambda i: abs(offset_deg(angle, MARKS_DEG[i])))
            mark = MARKS_DEG[index]
            if (len(self.held) == HOLD_STEPS and abs(offset_deg(angle, mark)) <= 1.
                    and max(abs(offset_deg(p, mark)) for p in self.held) <= 1.):
                self.release = Release(index, angle)
            self.held.clear()
        elif self.release is not None:
            test = self.release
            test.steps += 1
            if test.steps <= RELEASE_STEPS + END_DWELL_STEPS:
                test.peak_deg = max(test.peak_deg, abs(offset_deg(angle, test.start_deg)))
            if test.steps == RELEASE_STEPS:
                self.checks[test.index] = test.peak_deg

        if (self.release is None or self.release.steps < RELEASE_STEPS
                or len(self.checks) != len(MARKS_DEG) or max(self.checks.values()) > 12.):
            return partial
        if (max(self.checks.values()) <= 5. and self.release.peak_deg <= 5.
                and self.release.steps >= RELEASE_STEPS + END_DWELL_STEPS):
            return 1.
        return .5
