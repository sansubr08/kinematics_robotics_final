"""
One-time scene assembly: takes a KUKA YouBot (mecanum base + its own small
5-DOF arm) and a UR5, both already dragged into the scene, and:
  1. Removes the YouBot's own arm.
  2. Mounts the UR5 on top of the YouBot chassis in its place.

Run this ONCE with the simulation STOPPED, then save the scene (Ctrl+S).
After that, move_mobile_manipulator_straight_line.py can be run directly
against the saved scene without re-running this script.

Setup before running this:
    1. Model browser > robots > mobile > KUKA YouBot -> drag into scene.
    2. Model browser > robots > non-mobile > UR5 -> drag into scene.
    3. Leave the simulation stopped.

After running, visually check in the CoppeliaSim 3D view:
    - Is the UR5 sitting where you want on the chassis? Adjust
      MOUNT_HEIGHT below and re-run if it's floating or clipped in.
    - Is it facing the direction the base drives (+X)? Adjust
      MOUNT_ORIENTATION below and re-run if not.
This script is safe to re-run: if the YouBot's arm was already removed,
it just skips that step.
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


MOUNT_HEIGHT = 0.3          # m, UR5 base height above the YouBot chassis origin -- adjust after visual check
MOUNT_ORIENTATION = [0.0, 0.0, 0.0]  # radians, UR5 base orientation relative to chassis -- adjust if not facing +X


client = RemoteAPIClient()
sim = client.require("sim")

if sim.getSimulationState() != sim.simulation_stopped:
    raise RuntimeError("Stop the simulation before running this assembly script.")


def find_by_alias(sim, alias):
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        if sim.getObjectAlias(h, 0) == alias:
            return h
    return None


youbot_base = find_by_alias(sim, "youBot")
if youbot_base is None:
    raise RuntimeError(
        "Could not find a 'youBot' object in the scene. Drag it in from "
        "the Model browser (robots > mobile > KUKA YouBot) first."
    )

ur5_base = find_by_alias(sim, "UR5")
if ur5_base is None:
    raise RuntimeError(
        "Could not find a 'UR5' object in the scene. Drag it in from "
        "the Model browser (robots > non-mobile > UR5) first."
    )


# ---------------------------------------------------------------------
# Step 1: remove the YouBot's own arm (if still present)
# ---------------------------------------------------------------------

all_descendants = sim.getObjectsInTree(youbot_base, sim.handle_all, 0)
direct_children = [h for h in all_descendants if sim.getObjectParent(h) == youbot_base]

# The YouBot's arm joints are each parented to their own visual mesh link
# (e.g. youBotArmJoint1's parent is a shape called "Rectangle22", not
# youBotArmJoint2), so the joints aren't chained directly to each other.
# Instead, find whichever direct child of the chassis has an "arm" joint
# ANYWHERE in its subtree -- that child is the arm's mounting link, and
# removing it takes the whole arm (and gripper) with it.
arm_container = None
for child in direct_children:
    descendant_joints = sim.getObjectsInTree(child, sim.object_joint_type, 0)
    if any("arm" in sim.getObjectAlias(j, 0).lower() for j in descendant_joints):
        arm_container = child
        break

if arm_container is None:
    print("No YouBot arm found under /youBot (already removed, or differently structured).")
else:
    print(f"Removing YouBot's own arm (mount link: '{sim.getObjectAlias(arm_container, 0)}' and its subtree)...")
    sim.removeObjects([arm_container])


# ---------------------------------------------------------------------
# Step 2: mount the UR5 on top of the YouBot chassis
# ---------------------------------------------------------------------

sim.setObjectParent(ur5_base, youbot_base, True)  # True = preserve world pose during reparent
sim.setObjectPosition(ur5_base, youbot_base, [0.0, 0.0, MOUNT_HEIGHT])
sim.setObjectOrientation(ur5_base, youbot_base, MOUNT_ORIENTATION)

print(f"Mounted UR5 on youBot chassis at local height {MOUNT_HEIGHT} m.")
print("Check the 3D view, adjust MOUNT_HEIGHT/MOUNT_ORIENTATION and re-run if needed.")
print("Once it looks right, save the scene (Ctrl+S).")
