"""Reduced split-clamp load model and calibration for the size-6 shuttle."""
import math


TIGHTENING_NM = .20
TORQUE_BAND_NM = (.18, .22)
TOOL_STIFFNESS_NM_PER_RAD = 2.
TOOL_EFFECTIVE_LENGTH_M = .100
LOOSE_RUNNING_NM = .006
LOOSE_ANGLE_RAD = -.50 * math.pi
ESTIMATED_SEAT_STIFFNESS_NM_PER_RAD = .78
ESTIMATED_CLAMP_NUT_FACTOR = .20
ESTIMATED_CLAMP_SCREW_DIAMETER_M = .0025
ESTIMATED_SLEEVE_RADIUS_M = .0025
ESTIMATED_SPLIT_COLLAR_FRICTION = .16


def clamp_torque(angle_rad):
    return LOOSE_RUNNING_NM + ESTIMATED_SEAT_STIFFNESS_NM_PER_RAD * max(0., angle_rad)


def sleeve_breakaway_torque(clamp_angle_rad):
    """Estimated Coulomb limit at the threaded sleeve, before thread friction."""
    preload = max(0., (clamp_torque(clamp_angle_rad) - LOOSE_RUNNING_NM)
                  / (ESTIMATED_CLAMP_NUT_FACTOR * ESTIMATED_CLAMP_SCREW_DIAMETER_M))
    return ESTIMATED_SPLIT_COLLAR_FRICTION * preload * ESTIMATED_SLEEVE_RADIUS_M


def indicated_torque(beam_deflection_rad):
    return TOOL_STIFFNESS_NM_PER_RAD * beam_deflection_rad


def slip_reference(current, rest, stiffness, resistance):
    """Update an elastic friction rest only when real displacement exceeds the friction limit."""
    deflection = min(resistance / stiffness, max(-resistance / stiffness, current - rest))
    return current - deflection
