"""First-pass closing model for an installed, full-size door.

The values below are simulation starting points. A closer and force gauge
are needed to identify the real spring, hydraulic and latch response.
"""

from dataclasses import dataclass
from math import exp, radians


DOOR_WIDTH_M = 0.8636
DOOR_MASS_KG = 26.0
DOOR_INERTIA_KG_M2 = DOOR_MASS_KG * DOOR_WIDTH_M**2 / 3.0
OPEN_ANGLE = radians(90)
LATCH_START = radians(12)
LATCH_RAMP = radians(3)
SPRING_PRELOAD_NM = 7.0
SPRING_RATE_NM_PER_RAD = 2.0
LATCH_RISE_NM = 4.0
MAX_VALVE_ADJUSTMENT_TURNS = 0.75


def spring_torque(angle):
    """Closing torque derived from an approximately linear door spring."""
    return SPRING_PRELOAD_NM + SPRING_RATE_NM_PER_RAD * angle


def hydraulic_torque(angle, speed, clockwise_turns):
    """Signed closer torque with separate main and final phases."""
    if not 0.0 <= clockwise_turns <= MAX_VALVE_ADJUSTMENT_TURNS:
        raise ValueError("Outside the prototype valve travel")
    damping = 8.8 * exp(3.1 * clockwise_turns)
    if angle <= LATCH_START:
        damping = 22.0

    return -spring_torque(angle) - damping * speed


def closing_torque(angle, speed, clockwise_turns):
    """Bench approximation including a lumped latch load."""
    latch_load = 0.0
    if angle <= LATCH_RAMP:
        latch_load = LATCH_RISE_NM * (1 - angle / LATCH_RAMP)
    return hydraulic_torque(angle, speed, clockwise_turns) + latch_load


@dataclass(frozen=True)
class ClosingTrial:
    sweep_seconds: float | None
    latch_seconds: float | None
    speed_at_strike_rad_s: float | None


def passive_release(clockwise_turns, *, inertia=DOOR_INERTIA_KG_M2, dt=0.002, timeout=16.0):
    """Integrate a hands-off release with zero initial angular velocity."""
    if not 0.0 < dt <= 0.005:
        raise ValueError("Use a step of no more than five milliseconds")
    if not inertia > 0.0:
        raise ValueError("Use positive hinge inertia")
    angle, speed, time = OPEN_ANGLE, 0.0, 0.0
    sweep = None
    while time < timeout:
        torque = closing_torque(angle, speed, clockwise_turns)
        next_speed = speed + torque / inertia * dt
        next_angle = angle + next_speed * dt
        if sweep is None and next_angle <= LATCH_START:
            sweep = time + dt * (angle - LATCH_START) / (angle - next_angle)
        if next_angle <= 0.0:
            stop = time + dt * angle / (angle - next_angle)
            return ClosingTrial(sweep, stop - sweep, next_speed)
        angle, speed, time = next_angle, next_speed, time + dt
    return ClosingTrial(sweep, None, None)


if __name__ == "__main__":
    from dataclasses import asdict
    import json

    turns = [0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75]
    print(json.dumps(
        {str(turn): asdict(passive_release(turn)) for turn in turns},
        indent=2,
    ))
