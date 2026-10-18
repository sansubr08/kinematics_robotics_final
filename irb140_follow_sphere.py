"""
CoppeliaSim: ABB IRB 140 follows a moving sphere with its tip.

- Robot base at (0, -0.5, 0), rotated +90 deg about z
- Sphere oscillates along the line (-0.5, 0, 0.25) -> (0.5, 0, 0.25)
- Robot tip tracked with position-only IK (simIK)

Setup:
    pip install coppeliasim-zmqremoteapi-client
    Start CoppeliaSim (any empty scene), then run:
        python irb140_follow_sphere.py
    Optional: pass the full path to the IRB 140 model if the default isn't found:
        python irb140_follow_sphere.py "/path/to/ABB IRB 140.ttm"
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
PERIOD = 6.0                     # seconds for one full back-and-forth cycle
SIM_TIME = 30.0                  # seconds to run
MODEL_REL_PATH = "/robots/non-mobile/ABB IRB 140.ttm"
FLANGE_OFFSET = 0.065            # used only if the model has no tip dummy

# ---------------- connect ----------------
client = RemoteAPIClient()
sim = client.require("sim")
simIK = client.require("simIK")

sim.stopSimulation()
while sim.getSimulationState() != sim.simulation_stopped:
    pass

# Remove only leftovers from earlier runs (floor, camera, lights are kept)
MY_ALIASES = ("IRB140", "TargetSphere", "IK_target")
to_remove = []
for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 1):
    alias = sim.getObjectAlias(h, 0)
    if alias.startswith(MY_ALIASES):
        to_remove.extend(sim.getObjectsInTree(h, sim.handle_all, 0))
if to_remove:
    sim.removeObjects(list(dict.fromkeys(to_remove)))

# The arm is moved kinematically by IK, so turn physics off.
# Otherwise gravity pulls the links apart (joints aren't dynamically driven).
sim.setBoolParam(sim.boolparam_dynamics_handling_enabled, False)

# ---------------- load robot ----------------
def find_model_path():
    if len(sys.argv) > 1:
        return sys.argv[1]
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
            candidate = base + sub + MODEL_REL_PATH
            if os.path.exists(candidate):
                return candidate
    # Fall back to common install locations
    for root in (
        "/Applications/coppeliaSim.app/Contents/Resources/models",
        os.path.expanduser("~/CoppeliaSim/models"),
        "C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/models",
        "C:/Program Files/CoppeliaRobotics/CoppeliaSimPlayer/models",
    ):
        candidate = root + MODEL_REL_PATH
        if os.path.exists(candidate):
            return candidate
    raise RuntimeError(
        "Couldn't locate 'ABB IRB 140.ttm'. Pass its full path as an argument:\n"
        '  python irb140_follow_sphere.py "/path/to/ABB IRB 140.ttm"'
    )


model_path = find_model_path()
print("Loading model:", model_path)

robot = sim.loadModel(model_path)
sim.setObjectPosition(robot, ROBOT_POS, sim.handle_world)
sim.setObjectOrientation(robot, [0.0, 0.0, ROBOT_YAW], sim.handle_world)

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

        simIK.handleGroup(ik_env, ik_group, {"syncWorlds": True})
        client.step()
finally:
    sim.stopSimulation()
    simIK.eraseEnvironment(ik_env)
