"""Prime the paper in an already open, wall-mounted lever roll-towel dispenser."""
from pathlib import Path
from functools import lru_cache
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as m
from towel_contact_forces import instrument

# The contact and reset audit writes evidence without changing the policy input.
instrument(m)
from web_contact import (FeedScore, LEADER_TIP_X, LEADER_TIP_Z, NIP_X, NIP_Z, ROLL_CENTER, ROLLER_RADIUS,
                         Ratchet, WEB_THICKNESS, WIDTH, reset_path)
from yam_arena.constants import FINGER_CLOSED, GRIPPER

FINGER_OPEN = -.03745
PAPER_OPEN = -.020
PAPER_RELEASE = -.012
# The one-task runner loads this before it constructs the native arm drives.
GRIPPER["effort"] = 40.

SCENE = "station"
INSTRUCTION = "Start the new roll in the open paper-towel dispenser. Feed a straight, unbroken tail through the outlet and leave the lever released."
SUCCESS_HOLD_S = 0.
TOWEL_TASK = True

LOOKS = {
    "shell": m.Look(color=m.rgb(.18, .25, .32), roughness=.65),
    "plate": m.Look(color=m.rgb(.76, .79, .80), roughness=.75),
    "rubber": m.Look(color=m.rgb(.22, .24, .24), roughness=.9),
    "axle": m.Look(color=m.rgb(.68, .71, .74), roughness=.45, metallic=.35),
    "paper": m.Look(color=m.rgb(.96, .89, .69), roughness=.95),
    "lever": m.Look(color=m.rgb(.13, .36, .57), roughness=.55),
    "core": m.Look(color=m.rgb(.55, .40, .25), roughness=.9),
    "cradle": m.Look(color=m.rgb(.38, .56, .63), roughness=.62),
}
LEVER_PIVOT = (.365, .167, .953)
LEVER_LENGTH = .075
LOADING_PIVOT = (.408, 0., .850)
LOADING_TAB = (.319, -.231, .950)
TRUNNION_RADIUS = .005
TRUNNION_CLEARANCE_X = (.397, .419)
TRUNNION_CLEARANCE_Z = (.840, .860)
LOADING_OPEN_DEG = -58.
LOADING_GRIP_PITCH = -11.
RETURN_SPRING_RATE = .18
RETURN_SPRING_REST_DEG = 50.
CRADLE_FRONT_X = .326
CRADLE_DEPTH = .028
INNER_CRADLE_Y = -.018
CRADLE_TOP = .9806
PAPER_GRIP_Y = .064
ENTRY_X = .414
INSERT_PITCH_DEG = -53.
INSERT_TARGET_Z = .925
PAPER_PATH = reset_path()
# The segmented roller leaves space for the fingers between its two right bands.
ROLLER_BANDS = ((-.1095, -.046), (-.032, .022), (.119, .127))

features = [
    ("mount", (.018, .410, .410), (.557, 0., .9525), "plate"),
    ("back", (.008, .318, .324), (.544, 0., .982), "shell"),
    ("base_front", (.059, .318, .008), (.369, 0., .824), "shell"),
    ("base_rear", (.103, .318, .008), (.489, 0., .824), "shell"),
    ("side_a", (.198, .008, .324), (.445, .155, .982), "shell"),
    ("right_lower", (.198, .008, .020), (.445, -.155, .830), "shell"),
    ("right_upper", (.198, .008, .284), (.445, -.155, 1.002), "shell"),
    ("right_front", (.051, .008, .020), (.3715, -.155, .850), "shell"),
    ("right_rear", (.125, .008, .020), (.4815, -.155, .850), "shell"),
    ("cover_open", (.195, .310, .006), (.247, 0., .813), "shell"),
]
for rail, y in (("outer", -.067), ("inner", INNER_CRADLE_Y)):
    x = CRADLE_FRONT_X + CRADLE_DEPTH / 2
    features.append((rail + "_cradle", (CRADLE_DEPTH, .030, .004), (x, y, CRADLE_TOP - .002), "cradle"))
features.extend([
    ("outer_cradle_stem", (.008, .030, .158), (.294, -.067, CRADLE_TOP - .089), "shell"),
    ("outer_cradle_cantilever", (.054, .030, .006), (.317, -.067, CRADLE_TOP - .013), "shell"),
    ("cradle_crosspiece", (.012, .079, .006), (.340, -.0425, CRADLE_TOP - .007), "cradle"),
])
frame_parts = [(name, m.box(size, pos), look) for name, size, pos, look in features]
frame_colliders = [m.col_box(name + "_shape", size, pos) for name, size, pos, _ in features]
def narrow_contact(collider):
    return collider.replace("contactOffset = 0.004", "contactOffset = 0.00025")


# Two passive guide pairs sit between the roller bands and outside the
# finger lane. The rails funnel the leader toward the nip through contact.
for guide, y, sidewall in (("far", -.0397, -.151), ("near", .099, .151)):
    bottom, top = NIP_Z + .002, NIP_Z + .036
    height = top - bottom
    z = (top + bottom) / 2.
    thickness = .004
    for name, sign, entry in (("front", -1., .413), ("rear", 1., .442)):
        exit_face = NIP_X + sign * .00165
        shape = m.box((thickness, .008, height), ((entry + exit_face) / 2 + sign * thickness / 2., y, z))
        vertices = shape.v.copy()
        vertices[:, 0] += (vertices[:, 2] - z) * (entry - exit_face) / height
        rail = m.Mesh(vertices, shape.f)
        label = f"{guide}_{name}_guide"
        frame_parts.append((label, rail, "axle"))
        frame_colliders.append(narrow_contact(m.col_hull(label + "_shape", rail)))
        support_x = entry + sign * .003
        support = m.box((.006, abs(sidewall - y), .004), (support_x, (sidewall + y) / 2., top + .001))
        frame_parts.append((label + "_support", support, "shell"))
        frame_colliders.append(narrow_contact(m.col_hull(label + "_support_shape", support)))


def bearing_surface(name, radius, width, center, appearance):
    mesh = m.cylinder(radius, width, center, axis="y", segments=64)
    collision = m.col_cylinder(name + "_shape", radius, width, center)
    collision = narrow_contact(collision.replace('axis = "Z"', 'axis = "Y"'))
    return (name, mesh, appearance), collision


def web_strip(name, centre, length, pitch):
    bounds = (WEB_THICKNESS, WIDTH, length - .0001)
    mesh = m.box(bounds).turned(pitch, axis="y").moved(centre)
    contact = narrow_contact(m.col_box(name + "_shape", bounds, centre))
    contact = contact.replace("float3 xformOp:scale", f"float xformOp:rotateY = {pitch:.7f}\n        float3 xformOp:scale")
    contact = contact.replace('"xformOp:translate", "xformOp:scale"',
                              '"xformOp:translate", "xformOp:rotateY", "xformOp:scale"')
    return mesh, contact


def roller_parts(name, x):
    shaft_radius = .003 if name == "front" else .0045
    shaft_width = .250 if name == "front" else .283
    pieces = [bearing_surface(f"{name}_axle", shaft_radius, shaft_width, (x, 0., NIP_Z), "axle")]
    for i, (left, right) in enumerate(ROLLER_BANDS):
        pieces.append(bearing_surface(f"{name}_ring_{i}", ROLLER_RADIUS, right - left,
                                      (x, (left + right) / 2., NIP_Z), "rubber"))
    visuals, collisions = zip(*pieces, strict=True)
    witness = m.box((.002, .004, .002), (x, .106, NIP_Z + ROLLER_RADIUS - .00025))
    return [*visuals, (f"{name}_index", witness, "plate")], collisions


def loading_arm():
    x, _, z = LOADING_PIVOT
    x_tab, y_tab, z_tab = LOADING_TAB
    tab_mesh = m.box((.012, .030, .016)).turned(LOADING_GRIP_PITCH, axis="y").moved(LOADING_TAB)
    arm = m.tube([(x, y_tab, z), (x_tab, y_tab, z_tab)], .005)
    tab_shaft = m.tube([(x, -.134, z), (x, y_tab, z)], TRUNNION_RADIUS)
    parts = [("lifting_tab", tab_mesh, "lever"), ("handle_arm", arm, "shell"),
             ("trunnion_extension", tab_shaft, "axle")]
    for side, y in (("left", .134), ("right", -.134)):
        leg = m.box((.012, .009, NIP_Z - z + .008), (x, y, (NIP_Z + z) / 2.))
        parts.append((side + "_bearing_arm", leg, "shell"))
    colliders = [narrow_contact(m.col_hull(name + "_shape", mesh)) for name, mesh, _ in parts]
    return m.hinge("loading_arm", "Y", LOADING_PIVOT, LOADING_OPEN_DEG, .15,
                   parts, colliders, mass=.110, spring=RETURN_SPRING_RATE, damping=.12,
                   friction=.018, effort=.55, armature=.0015)


def nip_links():
    driven_center = (.448, 0., NIP_Z)
    idler_center = (.408, 0., NIP_Z)
    visuals, collisions = roller_parts("rear", driven_center[0])
    driven = m.hinge("driven", "Y", driven_center, -math.inf, math.inf,
                     visuals, collisions,
                     mass=.065, damping=.045, friction=0., effort=.12, armature=.00002)
    visuals, collisions = roller_parts("front", idler_center[0])
    bearing = m.slider("pressure", "X", -.003, .0001, [], [],
                       at=idler_center, parent="loading_arm", mass=.030, spring=1500.,
                       damping=3., friction=0., effort=8., armature=.001)
    idler = m.hinge("idler", "Y", idler_center, -math.inf, math.inf, visuals, collisions,
                    parent="pressure", mass=.055, friction=0., damping=.0001, armature=.00002)
    return driven, loading_arm(), bearing, idler


def web_links():
    roll_surface, roll_contact = bearing_surface("roll_paper", .0736, WIDTH, ROLL_CENTER, "paper")
    core = ("core", m.cylinder(.018, .218, ROLL_CENTER, axis="y"), "core")
    roll = m.hinge("roll", "Y", ROLL_CENTER, -math.inf, math.inf, [roll_surface, core], [roll_contact],
                   mass=.28, damping=.015, spring=.018, friction=0., effort=.023, armature=.0006)
    links = [roll]
    for i, (start, end) in enumerate(zip(PAPER_PATH[:-1], PAPER_PATH[1:], strict=True)):
        delta = end - start
        length = np.linalg.norm(delta)
        centre = (end + start) / 2.
        pitch = math.degrees(math.atan2(delta[0], delta[2]))
        name = f"web_{i:02}"
        mesh, coll = web_strip(name, centre, length, pitch)
        web = m.hinge(name, "Y", start, -125., 125., [(name, mesh, "paper")], [coll],
                      parent=links[-1].name, mass=.0002, spring=.0005, damping=.00009,
                      friction=0., gravity=True, effort=.012, armature=.000015, com=tuple(centre),
                      inertia=(.0000007, .00000030, .0000007))
        links.append(web)
    return links

lever = m.lever("lever", LEVER_PIVOT, (-1., 0., 0.), axis="Y", look="lever",
                length=LEVER_LENGTH, radius=.009, ball_r=.016, lower=-38.,
                upper=0., spring=1.5, damping=.05, friction=.016, mass=.080)
OBJECTS = [
    m.mechanism("dispenser", (0., 0., 0.), [lever], LOOKS,
                frame_parts=frame_parts, frame_colliders=frame_colliders),
    m.mechanism("feed_rollers", (0., 0., 0.), nip_links(), LOOKS, self_collide=True,
                friction=(1.25, 1.05), solver_iters=48),
    m.mechanism("towel_web", (0., 0., 0.), web_links(), LOOKS,
                friction=(.50, .38), solver_iters=48, self_collide=True),
]
_feed = FeedScore()
_ratchet = Ratchet()
_roller_speed = 0.


@lru_cache(maxsize=1)
def _web_body_indices(web):
    return [web.body_names.index(f"web_{i:02}") for i in range(len(PAPER_PATH) - 1)]


def _paper_edges(base):
    web = base.scene["towel_web"]
    indices = _web_body_indices(web)
    origin = m._np(base.scene.env_origins[0])
    poses = m._np(web.data.body_link_pose_w)[0, [*indices, indices[-1]]]
    edges = PAPER_PATH[:, None, :] + np.array([[0., WIDTH / 2, 0.], [0., -WIDTH / 2, 0.]])
    for i, pose in enumerate(poses):
        edges[i] = edges[i] @ m.quat_matrix(pose[3:7]).T + pose[:3] - origin
    return edges


def _set_drive(env, speed):
    rollers, joint = m._joint(env, "feed_rollers", "driven")
    rollers.set_joint_velocity_target(m._tensor(rollers, speed), joint_ids=[joint])


@m.on_before_reset
def restore_mechanisms(env):
    for name in ("dispenser", "feed_rollers", "towel_web"):
        asset = env.unwrapped.scene[name]
        position = asset.data.default_joint_pos.torch.clone()
        velocity = asset.data.default_joint_vel.torch.clone()
        asset.write_joint_state_to_sim_index(position=position, velocity=velocity)
        asset.actuators.target_command.set_position_index(value=position)
        asset.actuators.target_command.set_velocity_index(value=velocity)


@m.on_reset
def reset(env):
    global _feed, _ratchet, _roller_speed
    # Set the authored loading configuration after the stock settle period,
    # so contacts from a previous episode cannot shift the flexible leader.
    restore_mechanisms(env)
    base = env.unwrapped
    base.sim.forward()
    for _ in range(3):
        base.sim.render()
    for name in ("top_cam", "left_cam", "right_cam"):
        base.scene[name].update(0., force_recompute=True)
    _feed, _ratchet, _roller_speed = FeedScore(), Ratchet(), 0.
    _web_body_indices.cache_clear()
    _set_drive(env, 0.)
    # The spring free angle is past the closed mechanical stop. This
    # leaves closed preload without requiring a stiff spring to open.
    m.set_target(env, "feed_rollers", "loading_arm", math.radians(RETURN_SPRING_REST_DEG))
    path = os.environ.get("TOWEL_TRACE")
    if path:
        Path(path).write_text("")


@m.on_step
def update(env):
    global _roller_speed
    lever_angle = m.state(env, "dispenser").joint("lever")
    rollers = m.state(env, "feed_rollers")
    edges = _paper_edges(env.unwrapped)
    tips = m.fingertips(env)
    # The lever's handle and the hanging tail must both remain unheld.
    handle = np.asarray(LEVER_PIVOT) - LEVER_LENGTH * np.array(
        [math.cos(lever_angle), 0., -math.sin(lever_angle)])
    centreline = edges.mean(axis=1)
    paper_samples = np.vstack((edges[:, 0], centreline, edges[:, 1],
                               (centreline + edges[:, 0]) / 2.,
                               (centreline + edges[:, 1]) / 2.))
    web_clear = np.linalg.norm(tips[:, None] - paper_samples[None, :], axis=2).min() >= .018
    lever_clear = np.linalg.norm(tips - handle, axis=1).min() >= .030
    loading_angle = rollers.joint("loading_arm")
    _feed.observe(edges, lever_angle, rollers.joint("driven"), rollers.joint("idler"),
                  web_clear and lever_clear, _roller_speed < 0.,
                  nip_closed=abs(loading_angle) <= math.radians(3.))
    _roller_speed = _ratchet.roller_speed(lever_angle)
    # No drive command acts on the web. The powered roller can advance the
    # paper only by contact with it and with the loaded idler pressing back.
    _set_drive(env, _roller_speed)
    path = os.environ.get("TOWEL_TRACE")
    if path:
        from yam_arena.runtime import tcp_world

        base = env.unwrapped
        tcp = [np.asarray(tcp_world(base.scene[f"{side}_robot"])).tolist() for side in ("left", "right")]
        apertures = []
        for side in ("left", "right"):
            arm = base.scene[f"{side}_robot"]
            ids = arm.find_joints(["joint7", "joint8"])[0]
            apertures.append(float(np.abs(m._np(arm.data.joint_pos)[0, ids]).sum()))
        with open(path, "a") as trace:
            print(json.dumps({"edges": edges.tolist(), "tcp": tcp, "apertures_m": apertures,
                              "lever_deg": math.degrees(lever_angle), "loading_arm_deg": math.degrees(loading_angle),
                              "idler_travel_m": rollers.joint("pressure"),
                              "driven_rad": rollers.joint("driven"), "idler_rad": rollers.joint("idler"),
                              "drive_speed_rad_s": _roller_speed, "unspool_rad": m.state(env, "towel_web").joint("roll"),
                              "dwell": _feed.dwell, "driven_m": _feed.driven_m, "exposed_m": _feed.exposed,
                              "engaged": bool(_feed.engaged), "invalid": bool(_feed.invalid),
                              "score": _feed.score}), file=trace)


def score(objects):
    return _feed.score


def success(objects):
    return score(objects) == 1.


class Pitch(float):
    """Yaw with a pitch for a downward grasp that clears the mounted roll."""

    def __new__(cls, yaw, degrees):
        item = float.__new__(cls, yaw)
        item.pitch = math.radians(degrees)
        return item


class WristView(float):
    """A fixed wrist camera orientation for viewing the outlet."""

    def __new__(cls, rotation):
        item = float.__new__(cls, 0.0)
        item.rotation = rotation
        return item


def outlet_view():
    from yam_arena import station
    from yam_arena.constants import TCP_OFFSET
    from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

    camera = station.wrist_camera("right")
    eye = np.array([.206, -.386, .803])
    forward = np.array([.417, -.026, .793]) - eye
    forward /= np.linalg.norm(forward)
    up = np.array([0., 0., 1.])
    lateral = np.cross(forward, up)
    lateral /= np.linalg.norm(lateral)
    view = np.column_stack((lateral, np.cross(lateral, forward), -forward))
    view = view @ axis_angle_matrix(up, math.radians(-119.))
    wrist = view @ quat_xyzw_to_matrix(camera.rot_xyzw).T
    tcp = eye + wrist @ (np.array(TCP_OFFSET) - camera.pos)
    return tuple(tcp), WristView(wrist)


def demo():
    from yam_arena import demo as motion
    from yam_arena.urdf import axis_angle_matrix

    # The stock planner accepts yaw-only waypoints. Extend that conversion
    # for the paper handling poses while preserving every stock yaw.
    if not getattr(motion.grip_rotation, "towel_pitch", False):
        plain = motion.grip_rotation

        def grip_rotation(yaw=0.):
            if isinstance(yaw, WristView):
                return yaw.rotation
            rotation = plain(float(yaw))
            if isinstance(yaw, Pitch):
                return axis_angle_matrix(np.array([0., 1., 0.]), yaw.pitch) @ rotation
            return rotation

        grip_rotation.towel_pitch = True
        motion.grip_rotation = grip_rotation
    Waypoint = motion.Waypoint

    paper_pitch = Pitch(-math.pi / 2, -78.)
    lever_pitch = Pitch(-math.pi / 2, -22.)
    y = PAPER_GRIP_Y
    grasp = (LEADER_TIP_X + .004, y, LEADER_TIP_Z)
    entry = (ENTRY_X, y, .969)
    insert_pitch = Pitch(-math.pi / 2, INSERT_PITCH_DEG)
    held_offset = axis_angle_matrix(
        np.array([0., 1., 0.]), math.radians(INSERT_PITCH_DEG + 78.),
    ) @ np.array([.008, 0., -.0019])
    # Maintain the same terminal grip while orienting its short free end
    # downward. This offset is from the observed grip in the prior trial.
    anchor = np.array([NIP_X - .0110, y, .944])
    lower = tuple(anchor - [.008, 0., -.0019])
    turn = tuple(anchor - held_offset)
    feed = tuple(np.array([NIP_X - .0005, y, INSERT_TARGET_Z]) - held_offset)
    left = [
        Waypoint(1.8, (.269, y, .985), -.018, "align with the sheet from the front", paper_pitch),
        Waypoint(.85, (.269, y, .985), -.018, "settle the hand in the open aisle", paper_pitch),
        Waypoint(1.65, (LEADER_TIP_X - .005, y, .985), -.018, "approach the terminal edge", paper_pitch),
        Waypoint(.5, grasp, -.018, "straddle the edge overhanging the cradle", paper_pitch),
        Waypoint(.95, grasp, FINGER_CLOSED, "pinch the terminal strip of the sheet", paper_pitch),
        Waypoint(1.25, (.349, y, .993), FINGER_CLOSED, "lift the leader off the cradle", paper_pitch),
        Waypoint(1.5, (ENTRY_X, y, .990), FINGER_CLOSED, "carry the free edge towards the rollers", paper_pitch),
        Waypoint(1.2, entry, FINGER_CLOSED, "align the free edge over the roller nip", paper_pitch),
        Waypoint(1.18, lower, FINGER_CLOSED, "lower the held edge clear of the roll", paper_pitch),
        Waypoint(.90, turn, FINGER_CLOSED, "turn the held end toward the slot", insert_pitch),
        Waypoint(1.17, feed, FINGER_CLOSED, "introduce the edge with the idler open", insert_pitch),
        Waypoint(.35, feed, FINGER_CLOSED, "hold the edge in the passive guides", insert_pitch),
        Waypoint(.75, (.411, y, .949), PAPER_RELEASE, "release the leader and lift the pads clear", insert_pitch),
        Waypoint(1.15, (.301, y, .924), PAPER_RELEASE, "withdraw from the rollers", paper_pitch),
    ]
    p = np.asarray(LEVER_PIVOT)
    hover = tuple(p + [-LEVER_LENGTH, 0., .052])
    left.append(Waypoint(.50, hover, FINGER_CLOSED, "reach the opposite-side feed lever", lever_pitch))
    tab = np.asarray(LOADING_TAB)
    pivot = np.asarray(LOADING_PIVOT)
    tab_pitch = Pitch(math.pi / 2, LOADING_GRIP_PITCH)
    right = [Waypoint(1.8, tuple(tab + [-.027, 0., .045]), -.020, "approach the rectangular loading tab", tab_pitch),
             Waypoint(1.1, LOADING_TAB, -.020, "surround the loading tab", tab_pitch),
             Waypoint(.75, LOADING_TAB, FINGER_CLOSED, "grip the loading tab", tab_pitch)]

    arc = ((.45, -18.), (.45, -38.), (.45, LOADING_OPEN_DEG),
           (10.00, LOADING_OPEN_DEG), (.45, -38.), (.45, -18.), (.45, 0.))
    for duration, angle in arc:
        rotation = axis_angle_matrix(np.array([0., 1., 0.]), math.radians(angle))
        at = tuple(pivot + rotation @ (tab - pivot))
        right.append(Waypoint(duration, at, FINGER_CLOSED,
                              "position the loading arm",
                              Pitch(math.pi / 2, LOADING_GRIP_PITCH + angle)))
    park = tuple(tab + [-.036, 0., .051])
    right.extend([
        Waypoint(.4, LOADING_TAB, -.022, "release the loading tab", tab_pitch),
        Waypoint(.75, park, -.022, "clear the loading tab", tab_pitch),
    ])
    view, wrist = outlet_view()
    right.extend([
        Waypoint(19.5, park, -.022, "wait clear of the dispenser", tab_pitch),
        Waypoint(1.5, (.249, -.320, .940), -.022, "move clear of the loading tab", tab_pitch),
        Waypoint(1.6, (*view[:2], .945), -.022, "orient the wrist toward the outlet", wrist),
        Waypoint(1.8, view, -.022, "observe the tail below the outlet", wrist),
    ])
    for stroke in range(9):
        touch, press = (.20, .85) if stroke == 0 else (1.0, 1.4)
        # Leave room below the outlet by finishing with a shorter stroke.
        stroke_end = -26. if stroke == 8 else -35.
        for duration, angle, dz, label in [
            (touch, 0., .024, "touch lever"),
            (press, stroke_end, .010, "press lever"),
            (1.0, 0., .052, "release lever"),
        ]:
            a = math.radians(angle)
            tip = p + [-LEVER_LENGTH * math.cos(a), 0., LEVER_LENGTH * math.sin(a) + dz]
            left.append(Waypoint(duration, tuple(tip), FINGER_CLOSED, label, lever_pitch))
    left.append(Waypoint(2., (.21, .21, 1.025), FINGER_OPEN, "clear of the lever", lever_pitch))
    return {"left": left, "right": right}
