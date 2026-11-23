"""Quasi-static force model for the fan's hand-tightened pitch clamp."""
import math


SCREW_DIAMETER_M = .006
NUT_FACTOR = .2
COLLAR_RADIUS_M = .012
COLLAR_FRICTION = .25
SEATING_STIFFNESS_NM_RAD = .65
LOOSE_RUNNING_NM = .012
LOOSE_DEG = -45.
PROOF_MOMENT_NM = .12


def running_moment(knob_rad):
    return LOOSE_RUNNING_NM + SEATING_STIFFNESS_NM_RAD * max(0., knob_rad)


def breakaway_moment(knob_rad):
    preload = (running_moment(knob_rad) - LOOSE_RUNNING_NM) / (NUT_FACTOR * SCREW_DIAMETER_M)
    return .018 + COLLAR_RADIUS_M * COLLAR_FRICTION * preload


def loaded_head_moment(head_mass, head_com_x):
    return head_mass * abs(head_com_x) * 9.81
