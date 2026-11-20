"""Full-size cabinet and tools, in SI units."""
CONTROL_HZ=20
PHYSICS_HZ=480
DWELL_STEPS=10
FINGER_FORCE_LIMIT=40.
PAD_TORSIONAL_RADIUS=.002
TABLE_Z=.500
FRONT=.450
DRAWER_Z={'lower':.535,'upper':.685}
INITIAL_OPEN={'lower':.200,'upper':0.}
TRAVEL=.400
BAR_X=.520
BAR_Y=.235
CAM_Z={'lower':.550,'upper':.7004}
CAM_RISE=.006
BAR_TRAVEL=.0065
SCREW_START=(.310,.070,.539)
SCREW_GRIP=.157
SOCKET_START=(.540,-.070,.690)
SOCKET_HEIGHT=.037
SOCKET_RADIUS=.013
MAT_CENTER=(.230,-.420,.501)
MAT_SIZE=(.200,.200,.002)
WORKSPACE=((-0.15,-.65,.490),(1.20,.65,1.35))

def finger_joint_position(aperture):
    if not 0<=aperture<=.075:raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture,.0749)/2
