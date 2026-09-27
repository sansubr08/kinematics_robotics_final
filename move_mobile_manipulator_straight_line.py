"""
Drives the assembled mobile manipulator (UR5 mounted on a YouBot mecanum
base -- see assemble_mobile_manipulator.py) in a straight line from
(0,0,0) to (5,0,0). The UR5 is commanded to a fixed home pose once at the
start and just holds there (under its own dynamic position control)
while the mecanum base drives underneath it.

Run assemble_mobile_manipulator.py first (once) and save the scene, then
run this script whenever you want to replay the straight-line drive.

Why UR5's base pose is re-asserted every tick: even though UR5 is
reparented under youBot and marked kinematic, testing found it still
drifts away from its mounted offset by tens of cm the instant the
simulation starts (independent of collisions or joint pose -- looks like
CoppeliaSim resetting it to a stale cached dynamics pose). Rather than
depend on native parent/kinematic tracking, this script explicitly
re-pins UR5's local pose relative to youBot every control tick, which
reliably keeps it welded in place regardless of that underlying quirk.
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import time

from assignment4 import Mecanum


WHEEL_JOINT_NAMES = {
    "front_left": ["rollingJoint_fl"],
    "front_right": ["rollingJoint_fr"],
    "back_left": ["rollingJoint_rl", "rollingJoint_bl"],
    "back_right": ["rollingJoint_rr", "rollingJoint_br"],
}

WHEEL_RADIUS = 0.05
ROLLER_ANGLE_DEG = 45.0

UR5_MOUNT_POSITION = [0.0, 0.0, 0.3]      # local to youBot -- must match assemble_mobile_manipulator.py's MOUNT_HEIGHT
UR5_MOUNT_ORIENTATION = [0.0, 0.0, 0.0]   # local to youBot -- must match assemble_mobile_manipulator.py's MOUNT_ORIENTATION

WHEEL_SIGN = {
    "front_left": 1,
    "front_right": 1,
    "back_left": 1,
    "back_right": 1,
}

UR5_HOME_POSE = [
    0.0,
    -math.pi / 2,
    math.pi / 2,
    -math.pi / 2,
    -math.pi / 2,
    0.0,
]

GOAL = (5.0, 0.0)
SPEED = 0.3
POSITION_TOLERANCE = 0.05
CONTROL_DT = 0.05
MAX_TIME = 60.0


print("Connecting to CoppeliaSim...")
client = RemoteAPIClient()
sim = client.require("sim")
print("Connected!")


def find_by_alias(sim, alias):
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        if sim.getObjectAlias(h, 0) == alias:
            return h
    return None


# ---------------------------------------------------------------------
# Find the YouBot base + wheels, and the (now-mounted) UR5 + its joints.
# Looked up by alias rather than a fixed path, since the UR5's path
# changed from '/UR5' to '/youBot/UR5' once assemble_mobile_manipulator.py
# reparented it.
# ---------------------------------------------------------------------

youbot_base = find_by_alias(sim, "youBot")
if youbot_base is None:
    raise RuntimeError(
        "Could not find 'youBot' in the scene. Run assemble_mobile_manipulator.py "
        "first (with both YouBot and UR5 dragged in), then save the scene."
    )

ur5_base = find_by_alias(sim, "UR5")
if ur5_base is None:
    raise RuntimeError(
        "Could not find 'UR5' in the scene. Run assemble_mobile_manipulator.py "
        "first (with both YouBot and UR5 dragged in), then save the scene."
    )

# The YouBot's own top-level object frame is NOT aligned with its wheel
# geometry (its local X/Y/Z don't correspond to forward/left/up) -- but
# "youBot_ref", a child dummy included in the bundled model, IS: its
# local Y axis is the true forward direction, local X is left/right.
# Geometry and heading feedback below are measured relative to this
# reference frame instead of the raw youBot object.
robot_ref = find_by_alias(sim, "youBot_ref")
if robot_ref is None:
    raise RuntimeError(
        "Could not find 'youBot_ref' under youBot. This script relies on "
        "it to know which way the chassis actually faces."
    )

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
    raise RuntimeError(f"Could not find wheel joints for: {missing_roles}. Check WHEEL_JOINT_NAMES.")

wheel_order = ["front_left", "front_right", "back_left", "back_right"]
wheels = [wheel_joints[role] for role in wheel_order]
signs = [WHEEL_SIGN[role] for role in wheel_order]

# The bundled UR5 model aliases every joint just "joint" (not
# "UR5_joint1".."UR5_joint6"), so they can't be told apart by name.
# Walk the kinematic chain from the UR5 base instead: each joint's
# single shape child leads to the next joint, 6 times in a row.
def _find_child_of_type(sim, parent, obj_type):
    i = 0
    while True:
        child = sim.getObjectChild(parent, i)
        if child == -1:
            return None
        if sim.getObjectType(child) == obj_type:
            return child
        i += 1


ur5_joints = []
current = ur5_base
for _ in range(6):
    j = _find_child_of_type(sim, current, sim.object_joint_type)
    if j is None:
        raise RuntimeError(f"UR5 kinematic chain broken: found only {len(ur5_joints)} of 6 joints")
    ur5_joints.append(j)
    nxt = _find_child_of_type(sim, j, sim.object_shape_type)
    current = nxt if nxt is not None else j

for j in wheel_joints.values():
    sim.setIntProperty(j, "dynCtrlMode", sim.jointdynctrl_velocity)
for j in ur5_joints:
    sim.setIntProperty(j, "dynCtrlMode", sim.jointdynctrl_position)

print(f"Found youBot base ({len(wheel_joints)} wheels) and UR5 ({len(ur5_joints)} joints).")


# ---------------------------------------------------------------------
# Vehicle geometry, derived from wheel positions (see
# move_mecanum_straight_line.py for why this is auto-derived rather
# than hardcoded).
# ---------------------------------------------------------------------

# Measured relative to youBot_ref: local Y separates front/back (length),
# local X separates left/right (width) -- verified against the actual
# bundled model (its raw /youBot frame is NOT forward=X, left=Y).
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


def get_pose(sim, robot_ref):
    """
    Returns (x, y, theta) for the vehicle, where theta is the world-frame
    heading of the chassis's actual forward direction (youBot_ref's local
    Y axis), computed from the rotation matrix rather than Euler angles
    to avoid axis-convention mismatches.
    """
    pos = sim.getObjectPosition(robot_ref, -1)
    m = sim.getObjectMatrix(robot_ref, -1)  # 3x4 row-major: rotation columns are m[j], m[4+j], m[8+j]
    forward_x, forward_y = m[1], m[5]  # local Y axis (forward) in world
    theta = math.atan2(forward_y, forward_x)
    return pos[0], pos[1], theta


def weld_ur5(sim, ur5_base, youbot_base):
    """Re-pin UR5's local pose to youBot every tick (see module docstring)."""
    sim.setObjectPosition(ur5_base, youbot_base, UR5_MOUNT_POSITION)
    sim.setObjectOrientation(ur5_base, youbot_base, UR5_MOUNT_ORIENTATION)


def drive_to(sim, mecanum, robot_ref, wheels, signs, goal_xy, speed,
             ur5_base, youbot_base,
             tolerance=0.05, dt=0.05, max_time=60.0):
    start_time = time.time()
    while time.time() - start_time < max_time:
        weld_ur5(sim, ur5_base, youbot_base)

        x, y, theta = get_pose(sim, robot_ref)
        pos = (x, y)

        dx = goal_xy[0] - pos[0]
        dy = goal_xy[1] - pos[1]
        dist = math.hypot(dx, dy)

        if dist < tolerance:
            break

        direction = (dx / dist, dy / dist)
        v_desired = [speed * direction[0], speed * direction[1], 0.0]
        x_state = [pos[0], pos[1], theta]

        u = mecanum.inverse(x_state, v_desired)
        for wheel, psi, sign in zip(wheels, u, signs):
            sim.setJointTargetVelocity(wheel, sign * psi)

        time.sleep(dt)
    else:
        print("  (timed out before reaching the goal)")

    for wheel in wheels:
        sim.setJointTargetVelocity(wheel, 0.0)


def main():
    print("Starting simulation...")
    sim.startSimulation()

    print("Moving UR5 to home pose (it will hold this pose while the base drives)...")
    for joint, angle in zip(ur5_joints, UR5_HOME_POSE):
        sim.setJointTargetPosition(joint, angle)

    # Weld continuously during the settle wait too, not just in drive_to --
    # the drift happens the instant simulation starts, before driving begins.
    settle_start = time.time()
    while time.time() - settle_start < 1.0:
        weld_ur5(sim, ur5_base, youbot_base)
        time.sleep(0.05)

    start_pos = sim.getObjectPosition(robot_ref, -1)
    print(f"Start position: {[round(v, 3) for v in start_pos]}")
    print(f"Driving straight to {GOAL}...")

    drive_to(sim, mecanum, robot_ref, wheels, signs, GOAL, SPEED,
             ur5_base, youbot_base,
             tolerance=POSITION_TOLERANCE, dt=CONTROL_DT, max_time=MAX_TIME)

    end_pos = sim.getObjectPosition(robot_ref, -1)
    print(f"End position: {[round(v, 3) for v in end_pos]}")

    print("Stopping simulation...")
    sim.stopSimulation(True)
    print("Done!")


if __name__ == "__main__":
    main()
