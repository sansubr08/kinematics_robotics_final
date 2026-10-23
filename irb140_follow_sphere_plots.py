"""
CoppeliaSim: ABB IRB 140 and a Quadcopter both follow a moving sphere,
with data logging and plots for Problems 3 and 4.

Same scene as irb140_follow_sphere.py:
- IRB 140 base at (0, -0.5, 0), rotated +90 deg about z
- Sphere oscillates along the line (-0.5, 0, 0.25) -> (0.5, 0, 0.25)
- Robot tip tracked with position-only IK (simIK)
- Quadcopter hovers 2 m above the sphere, driven by the model's own
  target-following controller (we move its target dummy each step)

Added here: every simulation step the script logs the tip and the quadcopter
position expressed in the sphere frame, writes sphere_follow_log.csv, then
saves four figures into ./plots:
    problem3_x_error.png   x position error vs time (tip and quadcopter)
    problem3_y_error.png   y position error vs time
    problem4_vx.png        relative x velocity in the sphere frame
    problem4_vy.png        relative y velocity in the sphere frame

Setup:
    pip install coppeliasim-zmqremoteapi-client numpy matplotlib
    Start CoppeliaSim (any empty scene), then run:
        python irb140_follow_sphere_plots.py
    Optional: pass full model paths if the defaults aren't found:
        python irb140_follow_sphere_plots.py "/path/to/ABB IRB 140.ttm" "/path/to/Quadcopter.ttm"
    Re-plot from an existing log without CoppeliaSim:
        python irb140_follow_sphere_plots.py --plot-only sphere_follow_log.csv
"""
import csv
import math
import os
import sys

LOG_CSV = "sphere_follow_log.csv"
PLOT_DIR = "plots"
LOG_HEADER = [
    "t",
    "sphere_x", "sphere_y", "sphere_z",          # sphere, world frame
    "tip_x", "tip_y", "tip_z",                   # IRB 140 tip, in the sphere frame
    "quad_x", "quad_y", "quad_z",                # quadcopter, in the sphere frame
]


# ---------------- plotting (no CoppeliaSim needed) ----------------
def make_plots(csv_path, out_dir=PLOT_DIR):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    d = np.genfromtxt(csv_path, delimiter=",", names=True)
    t = d["t"]
    os.makedirs(out_dir, exist_ok=True)

    # Positions in the sphere frame ARE the position errors (object - sphere).
    err = {
        "Manipulator tip": {"x": d["tip_x"], "y": d["tip_y"]},
        "Quadcopter end effector": {"x": d["quad_x"], "y": d["quad_y"]},
    }
    # The sphere frame only translates (never rotates), so the velocity of an
    # object in that frame is the time derivative of its position in that
    # frame, which equals v_object - v_sphere in world axes.
    vel = {
        name: {ax: np.gradient(e[ax], t) for ax in ("x", "y")}
        for name, e in err.items()
    }
    colors = {"Manipulator tip": "tab:blue", "Quadcopter end effector": "tab:orange"}

    # ---- Problem 3: position error ----
    for ax_name in ("x", "y"):
        fig, (a1, a2) = plt.subplots(
            2, 1, figsize=(9, 7), sharex=True, gridspec_kw={"height_ratios": [2, 1]}
        )
        for name, e in err.items():
            a1.plot(t, e[ax_name], label=name, color=colors[name], lw=1.6)
        a1.set_ylabel(f"{ax_name} error  (object - sphere)  [m]")
        a1.set_title(f"Problem 3: {ax_name} position error relative to the sphere")
        a1.grid(True, alpha=0.3)
        a1.legend()
        a2.plot(t, err["Manipulator tip"][ax_name] * 1000.0,
                color=colors["Manipulator tip"], lw=1.4)
        a2.set_ylabel("tip only  [mm]")
        a2.set_xlabel("simulation time [s]")
        a2.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f"problem3_{ax_name}_error.png"), dpi=200)
        plt.close(fig)

    # ---- Problem 4: relative linear velocity in the sphere frame ----
    for ax_name in ("x", "y"):
        fig, a = plt.subplots(figsize=(9, 5))
        for name, v in vel.items():
            a.plot(t, v[ax_name], label=name, color=colors[name], lw=1.6)
        a.set_xlabel("simulation time [s]")
        a.set_ylabel(f"relative {ax_name} velocity in sphere frame [m/s]")
        a.set_title(f"Problem 4: relative {ax_name} linear velocity (sphere frame)")
        a.grid(True, alpha=0.3)
        a.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f"problem4_v{ax_name}.png"), dpi=200)
        plt.close(fig)

    # ---- summary numbers for the write-up ----
    print(f"\nSaved 4 figures to ./{out_dir}/   ({len(t)} samples, {t[0]:.2f}-{t[-1]:.2f} s)")
    print(f"{'':26s}{'x err RMS':>11s}{'x err max':>11s}{'y err RMS':>11s}{'y err max':>11s}   [m]")
    for name, e in err.items():
        rms = lambda a: float(np.sqrt(np.mean(a ** 2)))
        print(f"{name:26s}{rms(e['x']):11.4f}{np.max(np.abs(e['x'])):11.4f}"
              f"{rms(e['y']):11.4f}{np.max(np.abs(e['y'])):11.4f}")
    print(f"{'':26s}{'vx RMS':>11s}{'vx max':>11s}{'vy RMS':>11s}{'vy max':>11s}   [m/s]")
    for name, v in vel.items():
        rms = lambda a: float(np.sqrt(np.mean(a ** 2)))
        print(f"{name:26s}{rms(v['x']):11.4f}{np.max(np.abs(v['x'])):11.4f}"
              f"{rms(v['y']):11.4f}{np.max(np.abs(v['y'])):11.4f}")


if len(sys.argv) > 1 and sys.argv[1] == "--plot-only":
    make_plots(sys.argv[2] if len(sys.argv) > 2 else LOG_CSV)
    sys.exit(0)

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

before_load = set(sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0))
quad = sim.loadModel(quad_path)
quad_start = [LINE_A[i] + QUAD_OFFSET[i] for i in range(3)]
sim.setObjectPosition(quad, quad_start, sim.handle_world)

# The Quadcopter's controller chases a target dummy. Only accept dummies that
# appeared when the quadcopter loaded (never the IRB 140's own "target").
new_objs = [h for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
            if h not in before_load]
new_dummies = set(sim.getObjectsInTree(sim.handle_scene, sim.object_dummy_type, 0))
print("DEBUG objects added by quadcopter load:",
      [(sim.getObjectAlias(h, 1), "dummy" if h in new_dummies else "other") for h in new_objs])
# The model's controller script does sim.getObject('../target'), so the target
# is any object (not necessarily a dummy) whose alias is exactly "target".
cands = [h for h in new_objs if sim.getObjectAlias(h, 0).lower() == "target"]
if not cands:
    cands = [h for h in new_objs if "target" in sim.getObjectAlias(h, 0).lower()]
quad_target = cands[0] if cands else None

if quad_target is None:
    print("DEBUG no target dummy found. Controller script lines mentioning 'target':")
    for tname in ("scripttype_simulation", "scripttype_childscript", "scripttype_customization"):
        ttype = getattr(sim, tname, None)
        if ttype is None:
            continue
        for h in new_objs:
            try:
                sc = sim.getScript(ttype, h)
                if sc == -1:
                    continue
                text = sim.getScriptStringParam(sc, sim.scriptstringparam_text)
            except Exception:
                continue
            print(f"DEBUG --- {tname} on {sim.getObjectAlias(h, 1)}")
            for line in text.splitlines():
                if "target" in line.lower() or "getobject" in line.lower():
                    print("DEBUG   ", line.strip())
    raise RuntimeError("Couldn't find the Quadcopter's target dummy. Paste the DEBUG lines above.")

print("DEBUG chosen quad target:", sim.getObjectAlias(quad_target, 1))

# Do NOT reparent or rename the target here: the controller script looks it up
# as '../target' when the simulation starts and detaches it itself. We keep the
# handle and just move it every step.
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

log_rows = []

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

        # Log after the step so sphere, tip and quadcopter are all read at the
        # same simulation time. Tip/quadcopter positions are read relative to
        # the sphere, i.e. expressed in the sphere frame.
        sp = sim.getObjectPosition(sphere, sim.handle_world)
        tp = sim.getObjectPosition(tip, sphere)
        qp = sim.getObjectPosition(quad, sphere)
        log_rows.append([sim.getSimulationTime(), *sp, *tp, *qp])
        if len(log_rows) % 20 == 1:
            qt = sim.getObjectPosition(quad_target, sim.handle_world)
            qw = sim.getObjectPosition(quad, sim.handle_world)
            print(f"DEBUG t={log_rows[-1][0]:5.2f} sphere_x={sp[0]:+.3f} "
                  f"quad_target_x={qt[0]:+.3f} quad_x={qw[0]:+.3f} quad_z={qw[2]:.3f}")
finally:
    sim.stopSimulation()
    simIK.eraseEnvironment(ik_env)

if log_rows:
    with open(LOG_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(LOG_HEADER)
        w.writerows(log_rows)
    print(f"Logged {len(log_rows)} samples to {LOG_CSV}")
    make_plots(LOG_CSV)
