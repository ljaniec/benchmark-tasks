"""Observable proof for removing debris and restoring an installed caster."""
from dataclasses import dataclass
from math import isfinite, pi


TICKS_PER_SECOND = 20
TAU = 2. * pi


@dataclass(frozen=True)
class SpinReference:
    """Wheel-flick limits measured on a clean, assembled reference caster."""

    minimum_initial_rad_s: float
    maximum_initial_rad_s: float
    minimum_advance_rad: float
    held_ticks: int = TICKS_PER_SECOND // 2

    def __post_init__(self):
        values = (self.minimum_initial_rad_s, self.maximum_initial_rad_s, self.minimum_advance_rad)
        if not all(isfinite(v) for v in values) or not 0. < values[0] <= values[1] < pi * TICKS_PER_SECOND:
            raise ValueError('Set a positive unambiguous initial speed range')
        if self.minimum_advance_rad <= 0. or self.held_ticks < TICKS_PER_SECOND // 2:
            raise ValueError('Use a positive angular advance and a half-second or longer check')


@dataclass(frozen=True)
class Observation:
    module_removed: bool
    axle_and_wheel_intact: bool
    debris_cleared: bool
    original_parts_seated: bool
    hands_clear: bool
    angle_rad: float
    fault: bool = False


@dataclass
class CasterProof:
    reference: SpinReference
    opened: bool = False
    void: bool = False
    previous_angle: float | None = None
    last_clear: bool = True
    spin_ticks: int = 0
    spin_advance: float = 0.
    spin_sign: float = 0.
    spin_verified: bool = False

    def update(self, obs):
        self.void |= obs.fault or not obs.axle_and_wheel_intact or not isfinite(obs.angle_rad)
        if self.void:
            return 0.
        self.opened |= obs.module_removed
        delta = 0. if self.previous_angle is None else (obs.angle_rad - self.previous_angle + pi) % TAU - pi
        self.previous_angle = obs.angle_rad
        clean = self.opened and obs.debris_cleared
        installed = clean and obs.original_parts_seated
        if not installed:
            self.spin_verified = False
        if not installed or not obs.hands_clear:
            self.spin_ticks = 0
            self.spin_advance = 0.
            self.spin_sign = 0.
        elif not self.last_clear:
            speed = abs(delta) * TICKS_PER_SECOND
            if self.reference.minimum_initial_rad_s <= speed <= self.reference.maximum_initial_rad_s:
                self.spin_sign = 1. if delta > 0. else -1.
        if installed and obs.hands_clear and self.spin_sign and self.spin_ticks < self.reference.held_ticks:
            self.spin_ticks += 1
            self.spin_advance += delta * self.spin_sign
            self.spin_verified |= (self.spin_ticks == self.reference.held_ticks and
                                   self.spin_advance >= self.reference.minimum_advance_rad)
        self.last_clear = obs.hands_clear
        return 1. if installed and obs.hands_clear and self.spin_verified else .5 if clean else .25 if self.opened else 0.
