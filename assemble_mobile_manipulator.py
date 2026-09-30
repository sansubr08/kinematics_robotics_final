"""
One-time scene assembly: takes a KUKA YouBot (mecanum base + its own small
5-DOF arm) and a UR5, both already dragged into the scene, and:
  1. Removes the YouBot's own arm.
  2. Mounts the UR5 on top of the YouBot chassis in its place.
  3. Neutralizes the YouBot's own factory child script.

Run this ONCE with the simulation STOPPED, then save the scene (Ctrl+S).
After that, move_mobile_manipulator_straight_line.py can be run directly
against the saved scene without re-running this script.

Why step 3 matters: the YouBot ships with a child script that looks up
its OWN original arm joints (youBotArmJoint0..4) at simulation start and
also drives the wheels on a timer as a demo. Once step 1 deletes those
joints, that script throws a Lua error every single time simulation
starts, which was found (the hard way) to destabilize the ENTIRE
simulation -- not just that script -- causing it to freeze after a
handful of steps, made UR5 appear to violently drift off its mount, and
produced spurious IK failures in other scripts. None of that was a bug
in the Python control code; it was this crash. Since our Python scripts
drive both the wheels and the UR5 externally, the factory script isn't
needed at all and is replaced with a no-op.

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
    # sim.removeObjects only deletes the handles you pass it -- it does NOT
    # cascade to children (orphaned children get reparented up instead of
    # deleted, silently leaving the rest of the arm behind). Gather the
    # whole subtree explicitly first.
    subtree = [arm_container] + sim.getObjectsInTree(arm_container, sim.handle_all, 0)
    subtree = list(dict.fromkeys(subtree))
    print(f"Removing YouBot's own arm ({len(subtree)} objects, rooted at "
          f"'{sim.getObjectAlias(arm_container, 0)}')...")
    sim.removeObjects(subtree)


# ---------------------------------------------------------------------
# Step 2: mount the UR5 on top of the YouBot chassis
# ---------------------------------------------------------------------

# Position UR5 relative to youBot's current pose, but do NOT make it an
# actual scene-graph child of youBot: sim.setObjectPosition/Orientation's
# "relative to" argument works off any object's current world transform
# regardless of real parentage, so the weld scripts' per-tick repositioning
# (e.g. weld_ur5_to_chassis) works identically either way. Real parenting
# was tried first and caused CoppeliaSim to log "static tree built on top
# of a non-static tree" and visibly glitch, because UR5 is marked static
# (see below) while youBot is a dynamically-simulated (non-static) body --
# a static object nested under a moving dynamic parent is exactly the
# configuration that warning is about. Keeping UR5 a separate top-level
# object avoids that conflict entirely.
sim.setObjectParent(ur5_base, -1, True)  # True = preserve world pose
sim.setObjectPosition(ur5_base, youbot_base, [0.0, 0.0, MOUNT_HEIGHT])
sim.setObjectOrientation(ur5_base, youbot_base, MOUNT_ORIENTATION)

# Critical for any script using simIK on this arm: without this, simIK's
# solver treats the UR5 base as a free body it can reposition to help
# satisfy a target pose (confirmed by testing -- holding the IK target
# perfectly still still made the base fly off, unbounded, within a dozen
# ticks). Marking it static tells the solver it's a fixed anchor, exactly
# like a real arm bolted to a base. It can still be moved by our own
# scripts (e.g. weld_ur5_to_chassis in the driving scripts) -- "static"
# only means the physics/IK engines won't move it on their own.
sim.setObjectInt32Param(ur5_base, sim.shapeintparam_static, 1)

# Now that UR5 isn't parented to youBot, it and the chassis are two
# separate top-level bodies that happen to spatially overlap at the
# mount point -- normally CoppeliaSim skips collision between a parent
# and its own children, but two unrelated top-level bodies get no such
# exemption. Tested without this: the two immediately exploded apart
# violently (hundreds of meters) the instant simulation started. Since
# the box is teleport-welded to the tip rather than physically gripped,
# UR5 doesn't need real collision response for this project anyway.
ur5_shapes = sim.getObjectsInTree(ur5_base, sim.object_shape_type, 0) + [ur5_base]
for h in ur5_shapes:
    sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
print(f"Disabled collision response on {len(ur5_shapes)} UR5 shapes (prevents exploding against the chassis).")

print(f"Positioned UR5 (as a separate, static top-level object) at local height {MOUNT_HEIGHT} m above youBot.")


# ---------------------------------------------------------------------
# Step 3: neutralize the YouBot's own factory child script (see module
# docstring for why this is necessary, not just cosmetic).
# ---------------------------------------------------------------------

youbot_script = sim.getScriptAssociatedWithObject(youbot_base)
if youbot_script != -1:
    current_text = sim.getScriptStringParam(youbot_script, sim.scriptstringparam_text)
    if "youBotArmJoint" in current_text:
        neutralized = (
            'sim=require"sim"\n\n'
            "function sysCall_thread()\n"
            "    -- Original youBot demo script removed: it looked up\n"
            "    -- youBotArmJoint0..4, which no longer exist now that the\n"
            "    -- UR5 replaces the stock arm, and it also drove the wheels\n"
            "    -- on its own timer -- both are now handled externally by\n"
            "    -- the Python control scripts in this project instead.\n"
            "end\n"
        )
        sim.setScriptText(youbot_script, neutralized)
        print("Neutralized youBot's factory child script (it referenced the deleted arm joints).")
    else:
        print("youBot's child script already neutralized/doesn't reference the arm -- left as is.")

print("Check the 3D view, adjust MOUNT_HEIGHT/MOUNT_ORIENTATION and re-run if needed.")
print("Once it looks right, save the scene (Ctrl+S).")
