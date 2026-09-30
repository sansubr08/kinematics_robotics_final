"""
Combined demo: the assembled mobile manipulator drives to point A, picks
up a box there, drives the whole assembly in a straight line to point B,
and places the box down there.

This combines two previously-separate scripts:
  - move_mobile_manipulator_straight_line.py: mecanum driving (wheel
    kinematics via assignment4.Mecanum, chassis pose feedback).
  - pick_and_place.py: IK-driven grasp/place of a box on a raised
    pedestal (see that file's history for why the box sits on a
    pedestal rather than the floor -- reachability with a fixed
    straight-down tool orientation).

How the box survives the drive: while driving, the arm is held in a
fixed joint pose (not IK-controlled) so it moves rigidly with the
chassis, and the box's pose is re-pinned to the tip's pose every control
tick throughout -- both during arm motion (grasp/place) and during
driving. This "weld" approach is used everywhere in this project instead
of a single sim.setObjectParent() call, because parenting alone isn't a
physical constraint: it doesn't survive CoppeliaSim's dynamics stepping
(see move_mobile_manipulator_straight_line.py's docstring for the full
story with the UR5-to-chassis mount, which hit the exact same issue).

Run assemble_mobile_manipulator.py first (once), if you haven't already.
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import math
import time

from assignment4 import Mecanum


# ---------------------------------------------------------------------
# Driving constants
# ---------------------------------------------------------------------

WHEEL_JOINT_NAMES = {
    "front_left": ["rollingJoint_fl"],
    "front_right": ["rollingJoint_fr"],
    "back_left": ["rollingJoint_rl", "rollingJoint_bl"],
    "back_right": ["rollingJoint_rr", "rollingJoint_br"],
}
WHEEL_RADIUS = 0.05
ROLLER_ANGLE_DEG = 45.0

UR5_MOUNT_POSITION = [0.0, 0.0, 0.3]      # local to youBot -- must match assemble_mobile_manipulator.py
UR5_MOUNT_ORIENTATION = [0.0, 0.0, 0.0]

UR5_HOME_POSE = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]  # carry pose while driving

DRIVE_SPEED = 0.3
POSITION_TOLERANCE = 0.05
CONTROL_DT = 0.05
DRIVE_MAX_TIME = 60.0

# Chassis parking spots. The default scene's Floor is only 5x5m (+-2.5m) --
# both points, plus the pedestal/reach offset added below, stay well inside.
POINT_A = (-0.7, 0.3)   # chassis parks here to pick up the box
POINT_B = (0.9, 0.3)    # chassis parks here to drop the box off


# ---------------------------------------------------------------------
# Pick/place constants
# ---------------------------------------------------------------------

BOX_SIZE = 0.06
PEDESTAL_HEIGHT = 0.3    # raises the box near the UR5's own mount height instead of
                         # floor level -- see pick_and_place.py's history for why
PEDESTAL_SIZE = 0.15
APPROACH_CLEARANCE = 0.15
MOVE_DURATION = 3.0
CONTROL_FREQ = 20


print("Connecting to CoppeliaSim...")
client = RemoteAPIClient()
sim = client.require("sim")
simIK = client.require("simIK")
print("Connected!")


def find_by_alias(sim, alias):
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        if sim.getObjectAlias(h, 0) == alias:
            return h
    return None


def find_child_of_type(sim, parent, obj_type):
    i = 0
    while True:
        child = sim.getObjectChild(parent, i)
        if child == -1:
            return None
        if sim.getObjectType(child) == obj_type:
            return child
        i += 1


# ---------------------------------------------------------------------
# Find the robot: YouBot base + wheels + reference frame, UR5 base +
# joints (chain-walked -- the bundled model aliases every joint just
# "joint") + tip (the force-sensor "connection" past the last joint).
# ---------------------------------------------------------------------

youbot_base = find_by_alias(sim, "youBot")
if youbot_base is None:
    raise RuntimeError("Could not find 'youBot' in the scene. Run assemble_mobile_manipulator.py first.")

ur5_base = find_by_alias(sim, "UR5")
if ur5_base is None:
    raise RuntimeError("Could not find 'UR5' in the scene. Run assemble_mobile_manipulator.py first.")

robot_ref = find_by_alias(sim, "youBot_ref")
if robot_ref is None:
    raise RuntimeError("Could not find 'youBot_ref' under youBot.")

all_joints = sim.getObjectsInTree(sim.handle_scene, sim.object_joint_type, 0)
alias_to_handle = {sim.getObjectAlias(h, 0): h for h in all_joints}

wheel_joints = {}
missing_roles = []
for role, candidate_names in WHEEL_JOINT_NAMES.items():
    handle = next((alias_to_handle[n] for n in candidate_names if n in alias_to_handle), None)
    if handle is None:
        missing_roles.append(role)
    else:
        wheel_joints[role] = handle
if missing_roles:
    raise RuntimeError(f"Could not find wheel joints for: {missing_roles}.")

ur5_joints = []
current = ur5_base
for _ in range(6):
    j = find_child_of_type(sim, current, sim.object_joint_type)
    if j is None:
        raise RuntimeError(f"UR5 kinematic chain broken: found only {len(ur5_joints)} of 6 joints")
    ur5_joints.append(j)
    nxt = find_child_of_type(sim, j, sim.object_shape_type)
    current = nxt if nxt is not None else j

tip = find_child_of_type(sim, current, sim.object_forcesensor_type)
if tip is None:
    print("  Warning: no force-sensor tip found past the last joint; falling back to the last joint.")
    tip = ur5_joints[-1]

for j in wheel_joints.values():
    sim.setIntProperty(j, "dynCtrlMode", sim.jointdynctrl_velocity)
for j in ur5_joints:
    sim.setIntProperty(j, "dynCtrlMode", sim.jointdynctrl_position)

print(f"Found youBot base ({len(wheel_joints)} wheels), UR5 ({len(ur5_joints)} joints), tip handle {tip}.")


# ---------------------------------------------------------------------
# Chassis geometry (measured relative to youBot_ref -- its raw /youBot
# frame is NOT forward=X, left=Y) and the Mecanum kinematics object.
# ---------------------------------------------------------------------

local_pos = {role: sim.getObjectPosition(h, robot_ref) for role, h in wheel_joints.items()}
length = abs(local_pos["front_left"][1] - local_pos["back_left"][1])
width = abs(local_pos["front_left"][0] - local_pos["front_right"][0])
print(f"Derived chassis geometry: length={length:.3f} m, width={width:.3f} m")

mecanum = Mecanum(
    length=length,
    width=width,
    wheel_radius=WHEEL_RADIUS,
    roller_angle=math.radians(ROLLER_ANGLE_DEG),
    roller_radius=0.01,
)


# ---------------------------------------------------------------------
# IK setup: base -> joints -> tip, driven by a free target dummy.
# ---------------------------------------------------------------------

tip_pose_world = sim.getObjectPose(tip, -1)
ik_target = sim.createDummy(0.02)
sim.setObjectAlias(ik_target, "pick_drive_place_target")
sim.setObjectPose(ik_target, -1, tip_pose_world)

ik_env = simIK.createEnvironment()
ik_group = simIK.createGroup(ik_env)
simIK.setGroupCalculation(ik_env, ik_group, simIK.method_damped_least_squares, 0.1, 99)

try:
    constraints = simIK.constraint_pose
except AttributeError:
    constraints = (
        simIK.constraint_x | simIK.constraint_y | simIK.constraint_z
        | simIK.constraint_alpha_beta | simIK.constraint_gamma
    )
simIK.addElementFromScene(ik_env, ik_group, ur5_base, tip, ik_target, constraints)

GRASP_ORIENTATION = [1.0, 0.0, 0.0, 0.0]  # [qx,qy,qz,qw] -- tool pointing straight down


# ---------------------------------------------------------------------
# Wheel command mapping + world-to-body correction (see
# move_mobile_manipulator_straight_line.py for the empirical derivation
# of both the cross-wire/negation and the lateral-velocity sign flip).
# ---------------------------------------------------------------------

def apply_wheel_commands(sim, wheel_joints, u):
    psi_fl, psi_fr, psi_bl, psi_br = u
    sim.setJointTargetVelocity(wheel_joints["front_left"], -psi_fr)
    sim.setJointTargetVelocity(wheel_joints["front_right"], -psi_fl)
    sim.setJointTargetVelocity(wheel_joints["back_left"], -psi_br)
    sim.setJointTargetVelocity(wheel_joints["back_right"], -psi_bl)


def world_velocity_to_wheels(mecanum, x_state, v_world):
    x, y, theta = x_state
    vx_w, vy_w, w = v_world
    c, s = math.cos(theta), math.sin(theta)
    vx_body = c * vx_w + s * vy_w
    vy_body = -(-s * vx_w + c * vy_w)  # corrective negation -- see docstring reference above
    return mecanum.inverse([x, y, 0.0], [vx_body, vy_body, w])


def get_pose(sim, robot_ref):
    pos = sim.getObjectPosition(robot_ref, -1)
    m = sim.getObjectMatrix(robot_ref, -1)
    forward_x, forward_y = m[1], m[5]  # local Y axis (forward) in world
    theta = math.atan2(forward_y, forward_x)
    return pos[0], pos[1], theta


def weld_ur5():
    """Re-pin UR5's local pose to youBot every tick (see module docstring)."""
    sim.setObjectPosition(ur5_base, youbot_base, UR5_MOUNT_POSITION)
    sim.setObjectOrientation(ur5_base, youbot_base, UR5_MOUNT_ORIENTATION)


def weld_to_tip(obj, local_offset_pos=(0.0, 0.0, -BOX_SIZE / 2), local_offset_orient=(0.0, 0.0, 0.0)):
    """Re-pins obj's pose relative to the tip every call (the grasp)."""
    sim.setObjectPosition(obj, tip, list(local_offset_pos))
    sim.setObjectOrientation(obj, tip, list(local_offset_orient))


# ---------------------------------------------------------------------
# Quaternion slerp + streaming Cartesian arm move
# ---------------------------------------------------------------------

def slerp(q0, q1, t):
    q0 = np.array(q0, dtype=float)
    q1 = np.array(q1, dtype=float)
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = np.clip(dot, -1.0, 1.0)
    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return (result / np.linalg.norm(result)).tolist()
    theta_0 = np.arccos(dot)
    theta = theta_0 * t
    q_perp = q1 - q0 * dot
    q_perp = q_perp / np.linalg.norm(q_perp)
    return (q0 * np.cos(theta) + q_perp * np.sin(theta)).tolist()


def move_tip(pose_b, duration=MOVE_DURATION, control_freq=CONTROL_FREQ, carried_obj=None):
    """Streams the tip from its current pose to pose_b via IK, welding
    UR5 to the chassis and (optionally) an object to the tip every tick."""
    pose_a = sim.getObjectPose(tip, -1)
    pos_a, quat_a = pose_a[:3], pose_a[3:]
    pos_b, quat_b = pose_b[:3], pose_b[3:]

    n_steps = max(int(duration * control_freq), 1)
    dt = 1.0 / control_freq

    for i in range(1, n_steps + 1):
        weld_ur5()

        t = i / n_steps
        pos = [pos_a[k] + t * (pos_b[k] - pos_a[k]) for k in range(3)]
        quat = slerp(quat_a, quat_b, t)

        sim.setObjectPose(ik_target, -1, pos + quat)
        result = simIK.handleGroup(ik_env, ik_group, {"syncWorlds": True})
        result_code = result[0] if isinstance(result, (tuple, list)) else result
        if result_code != simIK.result_success:
            print(f"  warning: IK did not fully converge at t={t:.2f}")

        if carried_obj is not None:
            weld_to_tip(carried_obj)

        time.sleep(dt)

    actual_pos = sim.getObjectPose(tip, -1)[:3]
    error = math.sqrt(sum((actual_pos[k] - pos_b[k]) ** 2 for k in range(3)))
    if error > 0.05:
        print(f"  warning: tip ended {error:.3f} m from the intended target")


def go_to_carry_pose(carried_obj=None, duration=2.0):
    """Moves the arm to UR5_HOME_POSE via joint targets (not IK) -- the
    pose the driving script already uses and knows doesn't collide with
    the chassis. Welds the carried object to the tip throughout so it
    comes along for the transition, not just once it's done."""
    for j, a in zip(ur5_joints, UR5_HOME_POSE):
        sim.setJointTargetPosition(j, a)
    start = time.time()
    while time.time() - start < duration:
        weld_ur5()
        if carried_obj is not None:
            weld_to_tip(carried_obj)
        time.sleep(0.05)


def reclaim_ik_at_current_pose():
    """After driving with fixed joint targets, re-seat the IK target at
    the tip's actual current pose so the next move_tip() call starts
    from where the arm really is, not a stale earlier target."""
    current_pose = sim.getObjectPose(tip, -1)
    sim.setObjectPose(ik_target, -1, current_pose)
    simIK.handleGroup(ik_env, ik_group, {"syncWorlds": True})


def drive_to(goal_xy, carried_obj=None, speed=DRIVE_SPEED,
             tolerance=POSITION_TOLERANCE, dt=CONTROL_DT, max_time=DRIVE_MAX_TIME):
    start_time = time.time()
    while time.time() - start_time < max_time:
        weld_ur5()
        if carried_obj is not None:
            weld_to_tip(carried_obj)

        x, y, theta = get_pose(sim, robot_ref)
        dx = goal_xy[0] - x
        dy = goal_xy[1] - y
        dist = math.hypot(dx, dy)
        if dist < tolerance:
            break

        direction = (dx / dist, dy / dist)
        v_desired = [speed * direction[0], speed * direction[1], 0.0]
        u = world_velocity_to_wheels(mecanum, [x, y, theta], v_desired)
        apply_wheel_commands(sim, wheel_joints, u)

        time.sleep(dt)
    else:
        print("  (timed out before reaching the goal)")

    for wheel in wheel_joints.values():
        sim.setJointTargetVelocity(wheel, 0.0)


# ---------------------------------------------------------------------
# Box + pedestal creation
# ---------------------------------------------------------------------

def create_pedestal(xy, height):
    pedestal = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [PEDESTAL_SIZE, PEDESTAL_SIZE, height], 0)
    sim.setObjectAlias(pedestal, "pick_drive_place_pedestal")
    sim.setObjectPosition(pedestal, -1, [xy[0], xy[1], height / 2])
    sim.setObjectInt32Param(pedestal, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(pedestal, sim.shapeintparam_static, 1)
    sim.setShapeColor(pedestal, None, sim.colorcomponent_ambient_diffuse, [0.6, 0.6, 0.65])
    return pedestal


def create_box_and_settle(spawn_xy, spawn_z):
    box = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [BOX_SIZE] * 3, 0)
    sim.setObjectAlias(box, "pick_drive_place_box")
    sim.setObjectPosition(box, -1, [spawn_xy[0], spawn_xy[1], spawn_z])
    sim.setObjectInt32Param(box, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(box, sim.shapeintparam_static, 0)
    sim.setShapeColor(box, None, sim.colorcomponent_ambient_diffuse, [0.85, 0.15, 0.15])

    settle_start = time.time()
    while time.time() - settle_start < 2.0:
        weld_ur5()
        time.sleep(0.1)
    return box


def main():
    print("Starting simulation...")
    sim.startSimulation()

    # UR5 drifts off the chassis the instant simulation starts -- pin it
    # and let it settle before trusting any position reads.
    settle_start = time.time()
    while time.time() - settle_start < 1.0:
        weld_ur5()
        time.sleep(0.05)

    print(f"Driving to point A {POINT_A} to pick up the box...")
    drive_to(POINT_A)

    ur5_world_pos = sim.getObjectPosition(ur5_base, -1)
    pick_xy = (ur5_world_pos[0] + 0.25, ur5_world_pos[1] + 0.15)
    print(f"Placing pedestal + box near {pick_xy}...")
    create_pedestal(pick_xy, PEDESTAL_HEIGHT)
    box = create_box_and_settle(pick_xy, PEDESTAL_HEIGHT + 0.05)
    box_pos = sim.getObjectPosition(box, -1)
    grasp_z = box_pos[2] + BOX_SIZE / 2  # tip pose at grasp: box top surface
    print(f"Box settled at {[round(v, 3) for v in box_pos]}")

    pregrasp_a = [pick_xy[0], pick_xy[1], grasp_z + APPROACH_CLEARANCE] + GRASP_ORIENTATION
    grasp_a = [pick_xy[0], pick_xy[1], grasp_z] + GRASP_ORIENTATION

    print("Approaching the box...")
    move_tip(pregrasp_a)

    print("Descending to grasp...")
    move_tip(grasp_a)

    print("Grasping (welding box to tip)...")
    # While actively teleported every tick, the box is still a free
    # dynamic body in between corrections -- tested and found it visibly
    # drooped ~0.35m under gravity before each correction snapped it
    # back (very noticeable at this sim's faster-than-real-time rate).
    # Marking it static while carried stops gravity acting on it at all;
    # switching back to dynamic on release lets it settle normally.
    sim.setObjectInt32Param(box, sim.shapeintparam_static, 1)
    weld_ur5()
    weld_to_tip(box)
    time.sleep(0.2)

    print("Lifting...")
    move_tip(pregrasp_a, carried_obj=box)

    print("Tucking arm into carry pose for the drive...")
    go_to_carry_pose(carried_obj=box)

    print(f"Driving to point B {POINT_B} with the box...")
    drive_to(POINT_B, carried_obj=box)

    print("Reclaiming IK control for placing...")
    reclaim_ik_at_current_pose()

    ur5_world_pos_b = sim.getObjectPosition(ur5_base, -1)
    place_xy = (ur5_world_pos_b[0] + 0.25, ur5_world_pos_b[1] + 0.15)
    print(f"Placing a pedestal at {place_xy}...")
    create_pedestal(place_xy, PEDESTAL_HEIGHT)

    pregrasp_b = [place_xy[0], place_xy[1], grasp_z + APPROACH_CLEARANCE] + GRASP_ORIENTATION
    place_b = [place_xy[0], place_xy[1], grasp_z] + GRASP_ORIENTATION

    print("Moving above point B...")
    move_tip(pregrasp_b, carried_obj=box)

    print("Descending to place the box...")
    move_tip(place_b, carried_obj=box)

    print("Releasing...")
    sim.setObjectInt32Param(box, sim.shapeintparam_static, 0)  # let it settle under gravity again
    weld_ur5()
    time.sleep(0.2)

    print("Retreating...")
    move_tip(pregrasp_b)

    final_box_pos = sim.getObjectPosition(box, -1)
    print(f"Final box position: {[round(v, 3) for v in final_box_pos]}")

    print("Stopping simulation...")
    sim.stopSimulation(True)
    print("Done!")


if __name__ == "__main__":
    try:
        main()
    finally:
        try:
            simIK.eraseEnvironment(ik_env)
        except Exception:
            pass
        try:
            sim.removeObjects([ik_target])
        except Exception:
            pass
