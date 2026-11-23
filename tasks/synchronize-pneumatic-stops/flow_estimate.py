"""Provisional flow-to-speed estimate for the guarded pneumatic trainer."""
from math import pi

from numpy import interp


# SMC AS-FS catalog p. 6, upper middle chart, 6 mm tube.
# Sampled visually at integer turns. The plotted valve has a tapered inlet
# thread. Use these values to plan the G-thread test, then check measured strokes.
FLOW_LPM = (0., 1.2, 5.8, 11.1, 18.2, 29.3, 43.5, 64.4)
SUPPLY_ABSOLUTE_BAR = 6.
STANDARD_PRESSURE_BAR = 1.013
BORE_M = .032
# The 415 N return force at 0.6 MPa implies a 12 mm rod.
ROD_M = .012
ANNULAR_AREA_M2 = pi * (BORE_M**2 - ROD_M**2) / 4.
REFERENCE_TURNS = 4.
ADJUSTMENT_DEG = 180.
FAULT_TURNS = REFERENCE_TURNS - ADJUSTMENT_DEG / 360.


def extension_speed_mps(turns):
    """Approximate steady-speed extension for a lightly loaded meter-out rod."""
    standard_flow = float(interp(turns, range(len(FLOW_LPM)), FLOW_LPM))
    chamber_flow_m3_s = standard_flow * STANDARD_PRESSURE_BAR / (60_000. * SUPPLY_ABSOLUTE_BAR)
    return chamber_flow_m3_s / ANNULAR_AREA_M2
