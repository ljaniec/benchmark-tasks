"""Dimensions for a detached domestic vacuum extension and bench clearing tool."""

import numpy as np

from yam_arena.station import TABLE_TOP


TUBE_LENGTH_M = .422
TUBE_OD_M = .032
TUBE_BORE_M = .030
TUBE_MASS_KG = .329
TUBE_Z_M = TABLE_TOP + .103
ROD_SHANK_M = .484
ROD_RADIUS_M = .005
HANDLE_M = (.032, .035, .030)
PLUG_LENGTH_M = .023
PLUG_FROM_OUTLET_M = .050
TUBE_REST_Y_M = (.040, .289)
ROD_REST_Y_M = (-.422, -.154)
TUBE_REST_CENTER_TOL_M = .003
ROD_REST_CENTER_TOL_M = .008
REST_LATERAL_OFFSET_M = .052


def geometry(x=.280, inlet=-.038):
    outlet = inlet + TUBE_LENGTH_M
    plug_y = outlet - PLUG_FROM_OUTLET_M
    # Allow for a small axial slide of the tube on its open saddles.
    insertion_tip = outlet + .032
    first_tip = inlet - .014
    first_handle = first_tip - ROD_SHANK_M - HANDLE_M[1] / 2.
    last_handle = insertion_tip - ROD_SHANK_M - HANDLE_M[1] / 2.
    return {
        "tube": np.array((x, (inlet + outlet) / 2., TUBE_Z_M)),
        "inlet": float(inlet),
        "outlet": float(outlet),
        "plug": np.array((x, plug_y, TUBE_Z_M)),
        "rod_rest": np.array((x + REST_LATERAL_OFFSET_M, first_handle, TUBE_Z_M)),
        "rod_aligned": np.array((x, first_handle, TUBE_Z_M)),
        "rod_inserted": np.array((x, last_handle, TUBE_Z_M)),
        "hold": np.array((x, inlet + .273, TUBE_Z_M)),
    }
