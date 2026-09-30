"""
Pick-and-place: the mounted UR5 picks up a box at point A and places it
at point B. The mecanum base stays stationary for this demo -- only the
arm moves.

Builds on the other scripts in this project:
  - Joint/tip discovery via kinematic-chain walking, same as
    move_mobile_manipulator_straight_line.py (the bundled UR5's joints
    are all generically aliased "joint", not "UR5_joint1".."6").
  - IK-driven Cartesian streaming (position lerp + orientation slerp),
    same as move_ur5_straight_line.py.
  - "Welding" a moved object's pose every tick rather than trusting a
    single sim.setObjectParent() call to hold under dynamics -- this
    project already hit that exact bug with the UR5-to-chassis mount
    (see move_mobile_manipulator_straight_line.py's docstring), and a
    grasped box has the same issue: parenting it to the tip is not
    itself a physical constraint, so its pose is re-asserted every tick
    while "grasped" instead.

Run assemble_mobile_manipulator.py first (once), if you haven't already,
so the scene has the UR5 mounted on the YouBot.
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import math
import time


BOX_SIZE = 0.06          # m, cube side length
PEDESTAL_HEIGHT = 0.3    # m -- raises the box near the UR5's own mount height (~0.4m
                         # above the floor) instead of leaving it down at floor level,
                         # which was pushing parts of each trajectory to the edge of
                         # what's reachable with a fixed straight-down tool orientation.
PEDESTAL_SIZE = 0.15     # m, pedestal footprint (square)
APPROACH_CLEARANCE = 0.15  # m above the box to pass through before/after grasping
GRASP_ORIENTATION = None   # set after connecting: tool pointing straight down
MOVE_DURATION = 3.0        # s, per straight-line leg
CONTROL_FREQ = 20          # Hz


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
# Find the robot: UR5 base, its 6 joints (chain-walked, not by alias --
# see module docstring), and its tip (the force-sensor "connection"
# object just past the last joint).
# ---------------------------------------------------------------------

ur5_base = find_by_alias(sim, "UR5")
if ur5_base is None:
    raise RuntimeError(
        "Could not find 'UR5' in the scene. Run assemble_mobile_manipulator.py "
        "first (with both YouBot and UR5 dragged in), then save the scene."
    )

youbot_base = find_by_alias(sim, "youBot")
if youbot_base is None:
    raise RuntimeError("Could not find 'youBot' in the scene.")

# UR5 drifts away from the chassis the instant simulation starts (a
# CoppeliaSim dynamics quirk -- see move_mobile_manipulator_straight_line.py's
# docstring for the full story). Re-pin it every tick rather than trusting
# it to stay put; otherwise point A/B end up computed relative to a
# position the arm no longer occupies once the sim is actually running.
UR5_MOUNT_POSITION = [0.0, 0.0, 0.3]
UR5_MOUNT_ORIENTATION = [0.0, 0.0, 0.0]


def weld_ur5_to_chassis():
    sim.setObjectPosition(ur5_base, youbot_base, UR5_MOUNT_POSITION)
    sim.setObjectOrientation(ur5_base, youbot_base, UR5_MOUNT_ORIENTATION)

joints = []
current = ur5_base
for _ in range(6):
    j = find_child_of_type(sim, current, sim.object_joint_type)
    if j is None:
        raise RuntimeError(f"UR5 kinematic chain broken: found only {len(joints)} of 6 joints")
    joints.append(j)
    nxt = find_child_of_type(sim, j, sim.object_shape_type)
    current = nxt if nxt is not None else j

tip = find_child_of_type(sim, current, sim.object_forcesensor_type)
if tip is None:
    print("  Warning: no force-sensor tip found past the last joint; falling back to the last joint.")
    tip = joints[-1]

for j in joints:
    sim.setIntProperty(j, "dynCtrlMode", sim.jointdynctrl_position)

print(f"Found UR5 with {len(joints)} joints and tip handle {tip}.")


# ---------------------------------------------------------------------
# IK setup: base -> joints -> tip, driven by a free target dummy.
# ---------------------------------------------------------------------

tip_pose_world = sim.getObjectPose(tip, -1)
ik_target = sim.createDummy(0.02)
sim.setObjectAlias(ik_target, "pick_place_target")
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

# Tool points straight down (gripper approach orientation): rotate 180 deg
# about X from the tip's current resting orientation isn't reliable across
# arm poses, so instead build a fixed "pointing down" quaternion directly:
# tool Z axis anti-parallel to world Z, tool X aligned with world X.
GRASP_ORIENTATION = [1.0, 0.0, 0.0, 0.0]  # [qx,qy,qz,qw] -- 180 deg about X


# ---------------------------------------------------------------------
# Quaternion slerp + streaming Cartesian move (same technique as
# move_ur5_straight_line.py)
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
    """
    Streams the tip from its current pose to pose_b (a 7-element
    [x,y,z,qx,qy,qz,qw] pose) via IK, at a fixed update rate.

    If carried_obj is given, that object's pose is welded to the tip's
    pose every tick (see module docstring for why this is welded rather
    than parented once).
    """
    pose_a = sim.getObjectPose(tip, -1)
    pos_a, quat_a = pose_a[:3], pose_a[3:]
    pos_b, quat_b = pose_b[:3], pose_b[3:]

    n_steps = max(int(duration * control_freq), 1)
    dt = 1.0 / control_freq

    for i in range(1, n_steps + 1):
        weld_ur5_to_chassis()

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
        print(f"  warning: tip ended {error:.3f} m from the intended target (no retry -- see module notes)")


def weld_to_tip(obj, local_offset_pos=(0.0, 0.0, -BOX_SIZE / 2), local_offset_orient=(0.0, 0.0, 0.0)):
    """Re-pins obj's pose relative to the tip every call (the grasp)."""
    sim.setObjectPosition(obj, tip, list(local_offset_pos))
    sim.setObjectOrientation(obj, tip, list(local_offset_orient))


# ---------------------------------------------------------------------
# Create the box at point A, let it settle under gravity so we know its
# real resting height, then define point B alongside it.
# ---------------------------------------------------------------------

def create_box_and_settle(spawn_xy, spawn_z):
    box = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [BOX_SIZE] * 3, 0)
    sim.setObjectAlias(box, "pick_place_box")
    sim.setObjectPosition(box, -1, [spawn_xy[0], spawn_xy[1], spawn_z])
    sim.setObjectInt32Param(box, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(box, sim.shapeintparam_static, 0)
    sim.setShapeColor(box, None, sim.colorcomponent_ambient_diffuse, [0.85, 0.15, 0.15])

    settle_start = time.time()
    while time.time() - settle_start < 2.0:
        weld_ur5_to_chassis()
        time.sleep(0.1)
    return box


def create_pedestal(xy, height):
    """A static stand the box rests on, so it sits near the arm's own
    mount height instead of down at floor level (see PEDESTAL_HEIGHT)."""
    pedestal = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [PEDESTAL_SIZE, PEDESTAL_SIZE, height], 0)
    sim.setObjectAlias(pedestal, "pick_place_pedestal")
    sim.setObjectPosition(pedestal, -1, [xy[0], xy[1], height / 2])
    sim.setObjectInt32Param(pedestal, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(pedestal, sim.shapeintparam_static, 1)
    sim.setShapeColor(pedestal, None, sim.colorcomponent_ambient_diffuse, [0.6, 0.6, 0.65])
    return pedestal


def main():
    print("Starting simulation...")
    sim.startSimulation()

    # UR5 drifts off the chassis the instant simulation starts (see
    # weld_ur5_to_chassis's comment) -- pin it and let it settle BEFORE
    # reading its position, so point A/B are computed relative to where
    # the arm will actually stay for the rest of the run, not wherever it
    # happened to be in edit mode.
    settle_start = time.time()
    while time.time() - settle_start < 1.0:
        weld_ur5_to_chassis()
        time.sleep(0.05)

    ur5_world_pos = sim.getObjectPosition(ur5_base, -1)
    point_a_xy = (ur5_world_pos[0] + 0.25, ur5_world_pos[1] + 0.15)
    point_b_xy = (ur5_world_pos[0] + 0.25, ur5_world_pos[1] - 0.15)

    print(f"Placing pedestals at A {point_a_xy} and B {point_b_xy}...")
    create_pedestal(point_a_xy, PEDESTAL_HEIGHT)
    create_pedestal(point_b_xy, PEDESTAL_HEIGHT)

    print(f"Spawning box on the pedestal at point A and letting it settle...")
    box = create_box_and_settle(point_a_xy, PEDESTAL_HEIGHT + 0.05)
    box_pos = sim.getObjectPosition(box, -1)
    grasp_z = box_pos[2] + BOX_SIZE / 2  # tip pose at grasp: box top surface
    print(f"Box settled at {[round(v, 3) for v in box_pos]}")

    pregrasp_a = [point_a_xy[0], point_a_xy[1], grasp_z + APPROACH_CLEARANCE] + GRASP_ORIENTATION
    grasp_a = [point_a_xy[0], point_a_xy[1], grasp_z] + GRASP_ORIENTATION
    place_b = [point_b_xy[0], point_b_xy[1], grasp_z] + GRASP_ORIENTATION
    pregrasp_b = [point_b_xy[0], point_b_xy[1], grasp_z + APPROACH_CLEARANCE] + GRASP_ORIENTATION

    def report(label, intended_xyz):
        actual = sim.getObjectPose(tip, -1)[:3]
        print(f"  {label}: tip at {[round(v,3) for v in actual]} (target xyz {[round(v,3) for v in intended_xyz]})")

    print("Moving above point A...")
    move_tip(pregrasp_a)
    report("above A", pregrasp_a[:3])

    print("Descending to grasp the box...")
    move_tip(grasp_a)
    report("at A", grasp_a[:3])

    print("Grasping (welding box to tip)...")
    # While actively teleported every tick, the box is still a free
    # dynamic body in between corrections -- tested and found it visibly
    # drooped ~0.35m under gravity before each correction snapped it back
    # (very noticeable at this sim's faster-than-real-time rate). Marking
    # it static while carried stops gravity acting on it at all;
    # switching back to dynamic on release lets it settle normally.
    sim.setObjectInt32Param(box, sim.shapeintparam_static, 1)
    weld_ur5_to_chassis()
    weld_to_tip(box)
    time.sleep(0.2)

    print("Lifting...")
    move_tip(pregrasp_a, carried_obj=box)
    report("lifted", pregrasp_a[:3])

    print("Moving to above point B...")
    move_tip(pregrasp_b, carried_obj=box)
    report("above B", pregrasp_b[:3])

    print("Descending to place the box...")
    move_tip(place_b, carried_obj=box)
    report("at B", place_b[:3])

    print("Releasing...")
    sim.setObjectInt32Param(box, sim.shapeintparam_static, 0)  # let it settle under gravity again
    weld_ur5_to_chassis()
    time.sleep(0.2)  # last weld already left the box resting at place height

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
