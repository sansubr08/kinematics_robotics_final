"""
CoppeliaSim: ABB IRB 140 and a Quadcopter both follow a moving sphere.

- IRB 140 base at (0, -0.5, 0), rotated +90 deg about z
- Sphere oscillates along the line (-0.5, 0, 0.25) -> (0.5, 0, 0.25)
- Robot tip tracked with position-only IK (simIK)
- Quadcopter hovers 2 m above the sphere, driven by the model's own
  target-following controller (we move its target dummy each step)

Setup:
    pip install coppeliasim-zmqremoteapi-client
    Start CoppeliaSim (any empty scene), then run:
        python irb140_follow_sphere.py
    Optional: pass full model paths if the defaults aren't found:
        python irb140_follow_sphere.py "/path/to/ABB IRB 140.ttm" "/path/to/Quadcopter.ttm"
"""
import math
import os
import sys

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ---------------- parameters ----------------
ROBOT_POS = [0.0, -0.5, 0.0]
ROBOT_YAW = math.pi / 2          # +90 deg about z
LINE_A = [-0.5, 0.0, 0.25]
LINE_B = [0.5, 0.0, 0.25]
SPHERE_DIAMETER = 0.06
QUAD_OFFSET = [0.0, 0.0, 2.0]    # quadcopter target = sphere + 2 m in z
PERIOD = 6.0                     # seconds for one full back-and-forth cycle
SIM_TIME = 30.0                  # seconds to run
IRB_REL_PATH = "/robots/non-mobile/ABB IRB 140.ttm"
QUAD_REL_PATH = "/robots/mobile/Quadcopter.ttm"
FLANGE_OFFSET = 0.065            # used only if the model has no tip dummy

# ---------------- connect ----------------
client = RemoteAPIClient()
sim = client.require("sim")
simIK = client.require("simIK")

sim.stopSimulation()
while sim.getSimulationState() != sim.simulation_stopped:
    pass

# Remove only leftovers from earlier runs (floor, camera, lights are kept)
MY_ALIASES = ("IRB140", "TargetSphere", "IK_target", "Quadcopter")
to_remove = []
for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 1):
    alias = sim.getObjectAlias(h, 0)
    if alias.startswith(MY_ALIASES):
        to_remove.extend(sim.getObjectsInTree(h, sim.handle_all, 0))
if to_remove:
    sim.removeObjects(list(dict.fromkeys(to_remove)))

# Physics stays ON: the quadcopter needs it to fly. The arm is moved
# kinematically by IK, so its shapes are made static further below.
sim.setBoolParam(sim.boolparam_dynamics_handling_enabled, True)


# ---------------- model path lookup ----------------
def find_model_path(rel_path, argv_index):
    if len(sys.argv) > argv_index:
        return sys.argv[argv_index]
    # Try the CoppeliaSim install directory via whichever constant this version has
    for name in ("stringparam_application_path", "stringparam_modeldir"):
        const = getattr(sim, name, None)
        if const is None:
            continue
        try:
            base = sim.getStringParam(const)
        except Exception:
            continue
        for sub in ("/models", ""):
            candidate = base + sub + rel_path
            if os.path.exists(candidate):
                return candidate
    # Fall back to common install locations
    for root in (
        "/Applications/coppeliaSim.app/Contents/Resources/models",
        os.path.expanduser("~/CoppeliaSim/models"),
        "C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/models",
        "C:/Program Files/CoppeliaRobotics/CoppeliaSimPlayer/models",
    ):
        candidate = root + rel_path
        if os.path.exists(candidate):
            return candidate
    raise RuntimeError(
        f"Couldn't locate model '{os.path.basename(rel_path)}'. Pass full paths as arguments:\n"
        '  python irb140_follow_sphere.py "/path/to/ABB IRB 140.ttm" "/path/to/Quadcopter.ttm"'
    )


# ---------------- load IRB 140 ----------------
irb_path = find_model_path(IRB_REL_PATH, 1)
print("Loading robot arm:", irb_path)

robot = sim.loadModel(irb_path)
sim.setObjectPosition(robot, ROBOT_POS, sim.handle_world)
sim.setObjectOrientation(robot, [0.0, 0.0, ROBOT_YAW], sim.handle_world)

# The IRB 140 model ships with its own embedded script (its own IK + cleanup)
# that fights ours and errors on cleanup. Disable every script in its tree.
def disable_embedded_scripts(model):
    disabled = 0
    for h in sim.getObjectsInTree(model, sim.handle_all, 0):
        for type_name in ("scripttype_simulation", "scripttype_childscript"):
            script_type = getattr(sim, type_name, None)
            if script_type is None:
                continue
            try:
                s = sim.getScript(script_type, h)
                if s != -1:
                    sim.setScriptInt32Param(s, sim.scriptintparam_enabled, 0)
                    disabled += 1
            except Exception:
                pass
    return disabled


n_disabled = disable_embedded_scripts(robot)
print(f"Disabled {n_disabled} embedded script(s) on the IRB 140")
if n_disabled == 0:
    print("WARNING: no embedded script found/disabled; if you still see "
          "simIK errors, disable the IRB140 script manually in the scene hierarchy.")

# The arm is driven kinematically: make its shapes static so gravity can't
# pull the links apart now that physics is on.
for shp in sim.getObjectsInTree(robot, sim.object_shape_type, 0):
    sim.setObjectInt32Param(shp, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(shp, sim.shapeintparam_respondable, 0)

# ---------------- find joints and tip ----------------
joints = sim.getObjectsInTree(robot, sim.object_joint_type, 0)
if len(joints) < 6:
    raise RuntimeError(f"Expected at least 6 joints in the IRB 140 model, found {len(joints)}")
for j in joints:
    sim.setJointMode(j, sim.jointmode_kinematic)

tip = None
for d in sim.getObjectsInTree(robot, sim.object_dummy_type, 0):
    if "tip" in sim.getObjectAlias(d, 0).lower():
        tip = d
        break

if tip is None:
    # No tip dummy in the model: create one at the flange of the last link
    last_joint = joints[-1]
    last_link = sim.getObjectChild(last_joint, 0)
    tip = sim.createDummy(0.01)
    sim.setObjectAlias(tip, "IRB140_tip")
    sim.setObjectPose(tip, sim.getObjectPose(last_joint, sim.handle_world), sim.handle_world)
    pos = sim.getObjectPosition(tip, tip)
    sim.setObjectPosition(tip, [pos[0], pos[1], pos[2] + FLANGE_OFFSET], tip)
    sim.setObjectParent(tip, last_link if last_link != -1 else last_joint, True)

# ---------------- sphere and IK target ----------------
sphere = sim.createPrimitiveShape(
    sim.primitiveshape_spheroid, [SPHERE_DIAMETER] * 3, 0
)
sim.setObjectAlias(sphere, "TargetSphere")
sim.setObjectPosition(sphere, LINE_A, sim.handle_world)
sim.setObjectInt32Param(sphere, sim.shapeintparam_static, 1)
sim.setObjectInt32Param(sphere, sim.shapeintparam_respondable, 0)
sim.setShapeColor(sphere, None, sim.colorcomponent_ambient_diffuse, [1.0, 0.2, 0.2])

target = sim.createDummy(0.02)
sim.setObjectAlias(target, "IK_target")
sim.setObjectPosition(target, LINE_A, sim.handle_world)
sim.setObjectParent(target, sphere, True)   # target rides on the sphere

# ---------------- load Quadcopter ----------------
quad_path = find_model_path(QUAD_REL_PATH, 2)
print("Loading quadcopter:", quad_path)

quad = sim.loadModel(quad_path)
quad_start = [LINE_A[i] + QUAD_OFFSET[i] for i in range(3)]
sim.setObjectPosition(quad, quad_start, sim.handle_world)

# The Quadcopter model's built-in controller chases a dummy called "target".
quad_target = None
for d in sim.getObjectsInTree(quad, sim.object_dummy_type, 0):
    if "target" in sim.getObjectAlias(d, 0).lower():
        quad_target = d
        break
if quad_target is None:
    for d in sim.getObjectsInTree(sim.handle_scene, sim.object_dummy_type, 1):
        if sim.getObjectAlias(d, 0).lower() in ("quadcopter_target", "target"):
            quad_target = d
            break
if quad_target is None:
    raise RuntimeError("Couldn't find the Quadcopter's target dummy in the model or scene.")

# Detach the target from the quadcopter so it doesn't move with it
sim.setObjectParent(quad_target, -1, True)
sim.setObjectAlias(quad_target, "Quadcopter_target")
sim.setObjectPosition(quad_target, quad_start, sim.handle_world)

# ---------------- IK setup (position only) ----------------
ik_env = simIK.createEnvironment()
ik_group = simIK.createGroup(ik_env)
simIK.setGroupCalculation(
    ik_env, ik_group, simIK.method_damped_least_squares, 0.1, 100
)
simIK.addElementFromScene(
    ik_env, ik_group, robot, tip, target, simIK.constraint_position
)

# ---------------- run ----------------
client.setStepping(True)
sim.startSimulation()

try:
    while True:
        t = sim.getSimulationTime()
        if t >= SIM_TIME:
            break
        # s goes 0 -> 1 -> 0 smoothly (back and forth along the line)
        s = 0.5 * (1.0 - math.cos(2.0 * math.pi * t / PERIOD))
        p = [LINE_A[i] + s * (LINE_B[i] - LINE_A[i]) for i in range(3)]
        sim.setObjectPosition(sphere, p, sim.handle_world)

        # Quadcopter controller chases this point: 2 m above the sphere
        sim.setObjectPosition(
            quad_target, [p[i] + QUAD_OFFSET[i] for i in range(3)], sim.handle_world
        )

        simIK.handleGroup(ik_env, ik_group, {"syncWorlds": True})
        client.step()
finally:
    sim.stopSimulation()
    simIK.eraseEnvironment(ik_env)
