"""Prototype SI parameters. Manufacturer envelopes; estimated friction/compliance."""
import math
CONTROL_HZ = 20
PHYSICS_HZ = 480
DWELL_STEPS = 10
TABLE_Z = .7475
FINGER_FORCE_LIMIT = 40.
WORKSPACE = ((-.19,-.65,TABLE_Z-.01),(.91,.65,1.25))
BASE = (.36, 0., TABLE_Z+.025)
PIVOT = (.330, 0., TABLE_Z+.060)
CLAMP = (.312, 0., TABLE_Z+.060)
INITIAL_YAW = math.radians(-14)
SENSOR_CENTER_LOCAL = (.031,0,.02525)
SENSOR_SIZE = (.042,.0156,.0485)
LENS_LOCAL = (.0525,0,.02525)
REFLECTOR = (.900,0,TABLE_Z+.08525)
REFLECTOR_SIZE = (.004,.018,.060)
CARD_SIZE = (.040,.040,.060)
CARD_START = (.38,-.18,TABLE_Z+.031)
DRIVER_POS = (.23,-.15,TABLE_Z+.01005)
THREAD_PITCH = .0008
CLAMP_STIFFNESS = 1.e7
CLAMP_FRICTION_RADIUS = .018
CLAMP_BRAKE_DAMPING_NMS_RAD = 1000.
CLAMP_GAP = .0002  # One quarter-turn of the M5 x 0.8 thread.
CLAMP_MIN_TURN_DEG = 95.  # Beyond contact at 90 degrees; visibly loaded clamp.
MAX_CLAMP_FORCE = 600.
NUT_FACTOR = .20
BOLT_DIAMETER = .005
FRICTION = .25
PAD_STATIC_FRICTION = 1.2  # Estimated rubber-like pad, matching upstream YAM.
PAD_DYNAMIC_FRICTION = 1.0
PAD_TORSIONAL_RADIUS = .002  # Estimated finite rubber contact patch, native TGS friction.
CABLE_STIFFNESS_NM_RAD = .05
CABLE_DAMPING_NMS_RAD = .001
BEAM_HALF_ANGLE_RAD = math.atan(.0125/1.5)
MAX_REFLECTOR_INCIDENCE_DEG = 30.

def finger_joint_position(aperture):
    if not 0<=aperture<=.075: raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture,.0749)/2
