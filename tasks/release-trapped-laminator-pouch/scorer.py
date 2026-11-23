"""Judge a pouch withdrawn through the original slot with the release held."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Reading:
    lever_held: bool
    draw_m: float
    moving_through_entry: bool
    cleared: bool
    flat_in_rest_area: bool
    lever_home: bool
    released: bool
    valid: bool = True


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.score = 0.
        self.valid = True
        self.extraction_started = False
        self.release_path = False
        self.extracted_under_release = False
        self.full_ticks = 0

    def update(self, reading):
        self.valid &= reading.valid
        if not self.extracted_under_release:
            path = reading.lever_held and (reading.moving_through_entry or reading.cleared)
            if reading.draw_m < .004:
                self.release_path = path
                self.extraction_started = False
            elif not path:
                self.release_path = False
                self.extraction_started = False
            if self.release_path and reading.moving_through_entry and reading.draw_m >= .010:
                self.extraction_started = True
            if self.extraction_started and reading.cleared and reading.lever_held:
                self.extracted_under_release = True
        cleared = self.valid and self.extracted_under_release and reading.cleared
        complete = cleared and reading.flat_in_rest_area and reading.lever_home and reading.released
        self.full_ticks = self.full_ticks + 1 if complete else 0
        self.score = 1. if self.full_ticks >= 10 else .5 if cleared else .25 if self.valid and self.extraction_started else 0.
        return self.score
