"""Frozen local prototype parameters in metres, kg and seconds (not OEM CAD)."""
CONTROL_HZ = 20
PHYSICS_HZ = 480
DWELL_STEPS = 10
TABLE_Z = 0.7475
BATTERY_SIZE = (0.0177, 0.0338, 0.0527)
BATTERY_MASS = 0.05
LAYOUT_X_SHIFT = -0.04
MODULE_POS = (0.365 + LAYOUT_X_SHIFT, 0.0, TABLE_Z + 0.006)
BATTERY_POS = (0.342 + LAYOUT_X_SHIFT, 0.0, TABLE_Z + 0.012 + BATTERY_SIZE[2] / 2)
MAT_CENTER = (0.27, -0.20, TABLE_Z + 0.001)
MAT_SIZE = (0.12, 0.12, 0.002)
WORKSPACE = ((-0.19, -0.65, TABLE_Z - 0.01), (0.91, 0.65, 1.25))
STRAP_Z = TABLE_Z + 0.041
# A reconstructed reusable wrap with a folded-back free tail on its accessible side.
# It does not pass through the backplate or require an extra unthreading operation.
# Dimensions and force response are prototype estimates, never OEM measurements.
STRAP_WIDTH = 0.014
STRAP_THICKNESS = 0.0012
STRAP_DENSITY = 1200.0  # kg/m^3, an estimated bulk polymer-webbing density
STRAP_PATH = tuple((x + LAYOUT_X_SHIFT,y) for x,y in ((0.375, 0.018), (0.3323, 0.018), (0.3323, -0.0242), (0.372, -0.0242), (0.372, -0.0362), (0.338, -0.0362)))
BEND_STIFFNESS_NM_RAD = .005
BEND_DAMPING_NMS_RAD = .00015
BEND_TORQUE_LIMIT_NM = .012
BOND_FORCE = 1.2
BOND_TORQUE = 0.012
STATIC_FRICTION = 0.8
DYNAMIC_FRICTION = 0.6
FINGER_FORCE_LIMIT = 40.0


def finger_joint_position(aperture):
    """YAM URDF: q=0 closed; negative q opens each finger outward."""
    if not 0 <= aperture <= .075:
        raise ValueError('Requested aperture exceeds the 75 mm task limit')
    return -min(aperture,.0749)/2
