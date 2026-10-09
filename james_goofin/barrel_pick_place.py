"""
Description: Barrel pick-and-place with the OmniPlatform + UR5 in CoppeliaSim

    Stages (from the project plan)
        1. Park the robot near the barrel (NO omni motion). Only the UR5 moves, until the
           two dummies GrabPoint_UR5 and GrabPoint_Barrel coincide, then the barrel is
           "glued" to the UR5.
        2. UR5 rotates the glued barrel so it lies sideways (barrel axis horizontal).
        3. Start over. This time the OmniPlatform drives from its starting position to the
           barrel, and the UR5 grabs it (same as stage 1).
        4. OmniPlatform drives to PointB, the UR5 turns the barrel sideways (same as stage 2),
           lowers it to the floor and lets go.
        5. Repeat 1-4 several times with no problems (scene is restored between runs).

    Modes (--mode)
        ur5   stages 1-2 only
        full  stages 3-4 only
        all   stages 1-2, then 3-4  (default)      --repeat N repeats whatever mode you picked

Inputs:
    CoppeliaSim scene (james_goofin/scenes/_main_scene.ttt) with
        /OmniPlatform                      (UR5 mounted at /OmniPlatform/body/forceSensor/UR5)
        dummies  GrabPoint_UR5   (child of the UR5 arm, i.e. it is the arm's tool point)
                 GrabPoint_Barrel (on the barrel)
                 PointB           (where the barrel should end up)
        the barrel shape (found as the parent of GrabPoint_Barrel, or by an alias with "barrel")

Outputs:
    Simulated pick, rotate, drive, place. Prints a PASS/FAIL line per stage.

NOTES:
    - Run python barrel_pick_place.py --discover first. It lists the dummies/shapes it found,
      so a wrong name shows up immediately instead of mid-run.
    - Omni wheel mixing is CALIBRATED at the start of every drive (short test bursts), so you
      do not need to know which way the platform's wheel signs point.
    - The barrel is "glued" by making it static, non-respondable, and parenting it to
      GrabPoint_UR5 after snapping GrabPoint_Barrel exactly onto GrabPoint_UR5.
    - Everything tunable is in the CONSTANTS block right below the imports.
"""



#### IMPORTS
import argparse
import math
import re
import sys



#### CONSTANTS (tune here)
# --- object names (matched ignoring case, spaces and underscores) ---
NAME_GRAB_UR5    = "GrabPoint_UR5"
NAME_GRAB_BARREL = "GrabPoint_Barrel"
NAME_POINT_B     = "PointB"
NAME_BARREL      = "Barrel"          # only used if GrabPoint_Barrel is not a child of the barrel shape
PATH_PLATFORM    = "/OmniPlatform"
PATH_UR5         = "/OmniPlatform/body/forceSensor/UR5"
PATH_WHEELS      = [                 # same order as omni_test_move.py
    "/OmniPlatform/link[0]/regularRotation",   # back right
    "/OmniPlatform/link[1]/regularRotation",   # front right
    "/OmniPlatform/link[2]/regularRotation",   # front left
    "/OmniPlatform/link[3]/regularRotation",   # back left
]

# --- omni driving ---
WHEEL_V          = 80 * 2.398795 * math.pi / 180   # default wheel speed from the OmniPlatform lua script
WHEEL_MAX        = 2.0 * WHEEL_V      # rad/s, never command a wheel faster than this
CAL_SCALE        = 1.0                # calibration burst speed, as a multiple of WHEEL_V
CAL_STEPS        = 30                 # sim steps per calibration burst (one wheel at a time)
CAL_SETTLE_STEPS = 40                 # sim steps to let the platform coast to a stop between bursts
DRIVE_GAIN       = 2.0                # 1/s, P gain on position error
DRIVE_MAX_SPEED  = 0.25               # m/s
DRIVE_TOL        = 0.02               # m, stop when this close
DRIVE_MAX_STEPS  = 4000
YAW_GAIN         = 1.5                # 1/s, P gain on heading error
YAW_MAX_RATE     = 0.5                # rad/s, cap on the turning command that holds the heading
STOP_ON_CONTACT  = True               # stop driving when the platform runs into something (e.g. the white block)
STALL_WINDOW     = 40                 # sim steps; we check progress toward the goal over this window
STALL_MIN_FRAC   = 0.15               # blocked if progress is less than this fraction of what we commanded
STALL_MIN_M      = 0.01               # ...and less than this many metres

# --- arm / grasp geometry ---
ARM_STANDOFF     = 0.55    # m, horizontal distance from UR5 base to the barrel when parked for the grab
APPROACH_AXIS    = (0.0, 0.0, 1.0)   # direction (in GrabPoint_Barrel frame) the gripper comes in from
APPROACH_DIST    = 0.12    # m, pre-grasp offset along APPROACH_AXIS
LIFT_HEIGHT      = 0.15    # m, lift after the grab before rotating
FLOOR_CLEARANCE  = 0.01    # m, barrel is released this far above the floor
MOVE_STEPS       = 120     # sim steps for a normal arm move
ROTATE_STEPS     = 160     # sim steps for the sideways rotation
SETTLE_STEPS     = 100     # sim steps after release to let the barrel settle

# --- pass/fail thresholds ---
GLUE_TOL_M       = 0.01    # m, dummies must be this close for the grab to count
SIDEWAYS_TOL_DEG = 5.0     # deg, barrel axis must be this close to horizontal
PLACE_TOL_M      = 0.10    # m, barrel final xy must be this close to PointB



#### PURE MATH HELPERS (no simulator needed, unit-testable)
def vsub(a, b):   return [a[i] - b[i] for i in range(len(a))]
def vadd(a, b):   return [a[i] + b[i] for i in range(len(a))]
def vscale(a, s): return [x * s for x in a]
def vdot(a, b):   return sum(a[i] * b[i] for i in range(len(a)))
def vnorm(a):     return math.sqrt(vdot(a, a))

def vunit(a):
    n = vnorm(a)
    return [x / n for x in a] if n > 1e-12 else [0.0] * len(a)

def vcross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]

def quat_mul(a, b):
    """Hamilton product, quaternions as [x, y, z, w] (CoppeliaSim order). Result = a then-applied-after b."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]

def quat_axis_angle(axis, angle):
    ux, uy, uz = vunit(axis)
    s = math.sin(angle / 2.0)
    return [ux * s, uy * s, uz * s, math.cos(angle / 2.0)]

def quat_rotate(q, v):
    """Rotate vector v by quaternion q ([x,y,z,w])."""
    qv = [q[0], q[1], q[2]]
    t = vscale(vcross(qv, v), 2.0)
    return vadd(vadd(v, vscale(t, q[3])), vcross(qv, t))

def slerp(q0, q1, t):
    d = vdot(q0, q1)
    if d < 0.0:
        q1 = vscale(q1, -1.0)
        d = -d
    d = min(1.0, d)
    if d > 0.9995:
        r = [q0[i] + t * (q1[i] - q0[i]) for i in range(4)]
        return vunit(r)
    th0 = math.acos(d)
    th = th0 * t
    perp = vunit(vsub(q1, vscale(q0, d)))
    return [q0[i] * math.cos(th) + perp[i] * math.sin(th) for i in range(4)]

def axis_elevation_deg(axis_world):
    """Angle between an axis and the horizontal plane (0 = lying flat, 90 = standing upright)."""
    a = vunit(axis_world)
    return math.degrees(math.asin(max(-1.0, min(1.0, abs(a[2])))))

def sideways_rotation(axis_world, radial_xy):
    """
    Rotation (as a world-frame quaternion) that brings the barrel axis to horizontal.
    If the axis is already (nearly) horizontal returns the identity.
    If it is upright, it tips toward the radial direction (from the UR5 base toward the barrel),
    which is a plain wrist/elbow pitch motion that the UR5 does easily.
    """
    a = vunit(axis_world)
    if abs(a[2]) < 1e-3:
        return [0.0, 0.0, 0.0, 1.0]
    if abs(a[2]) > 0.999:
        rad = vunit([radial_xy[0], radial_xy[1], 0.0])
        a_h = rad if vnorm(rad) > 0.5 else [1.0, 0.0, 0.0]
    else:
        a_h = vunit([a[0], a[1], 0.0])
    axis = vcross(a, a_h)
    angle = math.acos(max(-1.0, min(1.0, vdot(a, a_h))))
    if vnorm(axis) < 1e-9:
        axis = vcross(a, [1.0, 0.0, 0.0] if abs(a[0]) < 0.9 else [0.0, 1.0, 0.0])
    return quat_axis_angle(axis, angle)

def solve_2x2(j1, j2, v):
    """Solve  c1*j1 + c2*j2 = v  for (c1, c2). j1, j2, v are 2-vectors. Returns None if singular."""
    det = j1[0] * j2[1] - j1[1] * j2[0]
    if abs(det) < 1e-12:
        return None
    c1 = (v[0] * j2[1] - v[1] * j2[0]) / det
    c2 = (j1[0] * v[1] - j1[1] * v[0]) / det
    return c1, c2

def solve_min_norm(A, u):
    """
    Smallest wheel-speed vector c with  A c = u.   A is 3 rows (vx, vy, yaw rate) x 4 wheel columns.
    c = A^T (A A^T)^-1 u.   Returns None if A cannot produce all three motions (singular).
    """
    m = [[sum(A[i][k] * A[j][k] for k in range(4)) for j in range(3)] for i in range(3)]
    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
           - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
           + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    scale = (m[0][0] + m[1][1] + m[2][2]) / 3.0
    if scale <= 0.0 or abs(det) < 1e-9 * scale ** 3:
        return None
    # Cramer's rule for y = m^-1 u
    def det3(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    y = []
    for col in range(3):
        mc = [row[:] for row in m]
        for r in range(3):
            mc[r][col] = u[r]
        y.append(det3(mc) / det)
    return [sum(A[r][k] * y[r] for r in range(3)) for k in range(4)]

def rot2(v, ang):
    c, s = math.cos(ang), math.sin(ang)
    return [c * v[0] - s * v[1], s * v[0] + c * v[1]]

def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

def norm_name(s):
    return re.sub(r"[\s_]+", "", s).lower()

def standoff_goal(barrel_xy, from_xy, standoff):
    """Point `standoff` away from the barrel, on the side the robot comes from."""
    d = vunit(vsub(from_xy, barrel_xy))
    if vnorm(d) < 0.5:
        d = [-1.0, 0.0]
    return vadd(barrel_xy, vscale(d, standoff))



#### SIM WRAPPER
class Robot:
    """Everything that talks to CoppeliaSim lives in here."""

    def __init__(self, platform_at_b=False):
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient   # imported here so the math above is testable offline
        self.client = RemoteAPIClient()
        self.sim    = self.client.require("sim")
        self.simIK  = self.client.require("simIK")
        self.platform_at_b = platform_at_b
        self.ik_env = self.ik_group = self.ik_target = None
        self._resolve()

    # ---------- finding things ----------
    def _all_objects(self):
        return self.sim.getObjectsInTree(self.sim.handle_scene, self.sim.handle_all, 0)

    def _find(self, name, obj_type=None, required=True):
        want = norm_name(name)
        hits = [h for h in self._all_objects()
                if norm_name(self.sim.getObjectAlias(h, 0)) == want
                and (obj_type is None or self.sim.getObjectType(h) == obj_type)]
        if hits:
            return hits[0]
        if required:
            raise RuntimeError(f"Could not find '{name}' in the scene. Run with --discover to see what is there.")
        return None

    def _resolve(self):
        sim = self.sim
        self.platform = sim.getObject(PATH_PLATFORM)
        self.ur5      = sim.getObject(PATH_UR5)
        self.wheels   = [sim.getObject(p) for p in PATH_WHEELS]
        self.joints   = sim.getObjectsInTree(self.ur5, sim.object_joint_type, 0)

        self.grab_ur5    = self._find(NAME_GRAB_UR5, sim.object_dummy_type)
        self.grab_barrel = self._find(NAME_GRAB_BARREL, sim.object_dummy_type)
        self.point_b     = self._find(NAME_POINT_B)

        parent = sim.getObjectParent(self.grab_barrel)
        if parent != -1 and sim.getObjectType(parent) == sim.object_shape_type:
            self.barrel = parent
        else:
            self.barrel = self._find(NAME_BARREL, sim.object_shape_type)

        # GrabPoint_UR5 must hang off the UR5 or the IK chain cannot be built
        node, on_arm = self.grab_ur5, False
        while node != -1:
            if node == self.ur5:
                on_arm = True
                break
            node = sim.getObjectParent(node)
        if not on_arm:
            raise RuntimeError(f"'{NAME_GRAB_UR5}' is not a descendant of the UR5. Parent it to the UR5's last link "
                               f"(it has to move with the arm for IK to work).")

        # barrel long axis (cylinder axis) = the longest bounding-box dimension, in the barrel's local frame
        bb = self._local_bbox(self.barrel)
        size = [bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]]
        self.barrel_axis_index = size.index(max(size))

        # remembered starting state (taken once, BEFORE anything is moved)
        self.platform_start_pose = sim.getObjectPose(self.platform, -1)
        self.joint_start         = [sim.getJointPosition(j) for j in self.joints]
        self.barrel_start_pose   = sim.getObjectPose(self.barrel, -1)
        self.barrel_start_parent = sim.getObjectParent(self.barrel)
        self.barrel_static0      = sim.getObjectInt32Param(self.barrel, sim.shapeintparam_static)
        self.barrel_respond0     = sim.getObjectInt32Param(self.barrel, sim.shapeintparam_respondable)
        self.floor_z             = self.world_min_z(self.barrel)

    def discover(self):
        sim = self.sim
        print("Dummies and shapes in the scene (alias  |  path  |  type):")
        for h in self._all_objects():
            t = sim.getObjectType(h)
            if t in (sim.object_dummy_type, sim.object_shape_type):
                kind = "dummy" if t == sim.object_dummy_type else "shape"
                print(f"  {sim.getObjectAlias(h, 0):28s} {sim.getObjectAlias(h, 1):60s} {kind}")
        print(f"\nResolved: GrabPoint_UR5={sim.getObjectAlias(self.grab_ur5, 1)}  "
              f"GrabPoint_Barrel={sim.getObjectAlias(self.grab_barrel, 1)}  "
              f"PointB={sim.getObjectAlias(self.point_b, 1)}  barrel={sim.getObjectAlias(self.barrel, 1)}")
        print(f"UR5 joints found: {len(self.joints)}   barrel long axis: local {'XYZ'[self.barrel_axis_index]}")

    # ---------- small geometry helpers ----------
    def _local_bbox(self, obj):
        sim = self.sim
        return [sim.getObjectFloatParam(obj, p) for p in (
            sim.objfloatparam_objbbox_min_x, sim.objfloatparam_objbbox_min_y, sim.objfloatparam_objbbox_min_z,
            sim.objfloatparam_objbbox_max_x, sim.objfloatparam_objbbox_max_y, sim.objfloatparam_objbbox_max_z)]

    def world_min_z(self, obj):
        sim = self.sim
        bb = self._local_bbox(obj)
        pose = sim.getObjectPose(obj, -1)
        zs = []
        for x in (bb[0], bb[3]):
            for y in (bb[1], bb[4]):
                for z in (bb[2], bb[5]):
                    zs.append(vadd(pose[:3], quat_rotate(pose[3:], [x, y, z]))[2])
        return min(zs)

    def barrel_axis_world(self):
        local = [0.0, 0.0, 0.0]
        local[self.barrel_axis_index] = 1.0
        return quat_rotate(self.sim.getObjectPose(self.barrel, -1)[3:], local)

    def pos(self, obj):  return self.sim.getObjectPosition(obj, -1)
    def pose(self, obj): return self.sim.getObjectPose(obj, -1)

    def platform_yaw(self):
        """Heading from wheel positions: back-wheel midpoint -> front-wheel midpoint (works whatever the body frame is)."""
        br, fr, fl, bl = [self.pos(w) for w in self.wheels]
        fwd = vsub(vscale(vadd(fr, fl), 0.5), vscale(vadd(br, bl), 0.5))
        return math.atan2(fwd[1], fwd[0])

    def step(self, n=1):
        for _ in range(n):
            self.sim.step()

    # ---------- simulation lifecycle ----------
    def begin_run(self, near_barrel):
        """Reset the scene to a clean start, optionally teleporting the platform next to the barrel."""
        sim = self.sim
        self._restore_barrel()
        sim.setObjectPose(self.platform, -1, self.platform_start_pose)
        for j, q in zip(self.joints, self.joint_start):
            sim.setJointPosition(j, q)
        if near_barrel:
            goal = standoff_goal(self.pos(self.barrel)[:2], self.pos(self.ur5)[:2], ARM_STANDOFF)
            shift = vsub(goal, self.pos(self.ur5)[:2])
            p = sim.getObjectPosition(self.platform, -1)
            sim.setObjectPosition(self.platform, -1, [p[0] + shift[0], p[1] + shift[1], p[2]])

        sim.setStepping(True)
        sim.startSimulation()
        # keep the arm exactly where it started, and freeze the barrel so the gripper can't knock it over
        for j in self.joints:
            try:
                sim.setIntProperty(j, "dynCtrlMode", sim.jointdynctrl_position)
            except Exception:
                pass
        for j, q in zip(self.joints, self.joint_start):
            sim.setJointTargetPosition(j, q)
        sim.setObjectInt32Param(self.barrel, sim.shapeintparam_static, 1)
        self.step(30)
        self._build_ik()

    def end_run(self):
        try:
            if self.ik_env is not None:
                self.simIK.eraseEnvironment(self.ik_env)
        except Exception:
            pass
        try:
            if self.ik_target is not None:
                self.sim.removeObjects([self.ik_target])
        except Exception:
            pass
        self.ik_env = self.ik_group = self.ik_target = None
        for w in self.wheels:
            try:
                self.sim.setJointTargetVelocity(w, 0.0)
            except Exception:
                pass
        self.sim.stopSimulation(True)
        self._restore_barrel()

    def _restore_barrel(self):
        sim = self.sim
        try:
            sim.setObjectParent(self.barrel, self.barrel_start_parent, True)
            sim.setObjectPose(self.barrel, -1, self.barrel_start_pose)
            sim.setObjectInt32Param(self.barrel, sim.shapeintparam_static, self.barrel_static0)
            sim.setObjectInt32Param(self.barrel, sim.shapeintparam_respondable, self.barrel_respond0)
        except Exception as exc:
            print(f"  (could not fully restore the barrel: {exc})")

    # ---------- IK ----------
    def _build_ik(self):
        sim, simIK = self.sim, self.simIK
        self.ik_target = sim.createDummy(0.02)
        sim.setObjectAlias(self.ik_target, "barrel_task_ik_target")
        sim.setObjectPose(self.ik_target, -1, self.pose(self.grab_ur5))
        self.ik_env = simIK.createEnvironment()
        self.ik_group = simIK.createGroup(self.ik_env)
        simIK.setGroupCalculation(self.ik_env, self.ik_group, simIK.method_damped_least_squares, 0.1, 99)
        try:
            constraints = simIK.constraint_pose
        except AttributeError:
            constraints = (simIK.constraint_x | simIK.constraint_y | simIK.constraint_z
                           | simIK.constraint_alpha_beta | simIK.constraint_gamma)
        simIK.addElementFromScene(self.ik_env, self.ik_group, self.ur5, self.grab_ur5, self.ik_target, constraints)

    def _ik_to(self, pose):
        self.sim.setObjectPose(self.ik_target, -1, pose)
        res = self.simIK.handleGroup(self.ik_env, self.ik_group, {"syncWorlds": True})
        code = res[0] if isinstance(res, (tuple, list)) else res
        return code == self.simIK.result_success

    def move_tool(self, target_pose, steps=MOVE_STEPS, settle=40):
        """Stream GrabPoint_UR5 from where it is to target_pose (straight line + slerp), one IK solve per sim step."""
        start = self.pose(self.grab_ur5)
        p0, q0, p1, q1 = start[:3], start[3:], target_pose[:3], target_pose[3:]
        misses = 0
        for i in range(1, steps + 1):
            t = i / steps
            pos = [p0[k] + t * (p1[k] - p0[k]) for k in range(3)]
            if not self._ik_to(pos + slerp(q0, q1, t)):
                misses += 1
            self.step()
        for _ in range(settle):                 # hold the final target so the arm catches up
            self._ik_to(list(target_pose))
            self.step()
        err = vnorm(vsub(self.pos(self.grab_ur5), p1))
        if misses > steps * 0.3 or err > 0.03:
            print(f"  warning: IK trouble (unsolved {misses}/{steps} ticks, final tool error {err * 1000:.0f} mm). "
                  f"Target may be out of reach: try a different --standoff.")
        return err

    # ---------- the glue ----------
    def glue_barrel(self):
        """Snap GrabPoint_Barrel onto GrabPoint_UR5 exactly, then lock the barrel to the arm."""
        sim = self.sim
        gu = self.pose(self.grab_ur5)
        t_barrel_in_gb = sim.getObjectPose(self.barrel, self.grab_barrel)   # barrel pose in the barrel-dummy frame
        sim.setObjectPose(self.barrel, -1, sim.multiplyPoses(gu, t_barrel_in_gb))
        sim.setObjectInt32Param(self.barrel, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(self.barrel, sim.shapeintparam_respondable, 0)
        sim.setObjectParent(self.barrel, self.grab_ur5, True)
        self.step(10)
        gap = vnorm(vsub(self.pos(self.grab_barrel), self.pos(self.grab_ur5)))
        return gap

    def release_barrel(self):
        sim = self.sim
        sim.setObjectParent(self.barrel, -1, True)
        sim.setObjectInt32Param(self.barrel, sim.shapeintparam_respondable, self.barrel_respond0 or 1)
        sim.setObjectInt32Param(self.barrel, sim.shapeintparam_static, 0)
        self.step(SETTLE_STEPS)

    # ---------- the two arm tasks ----------
    def grab(self):
        """Stage 1: move GrabPoint_UR5 onto GrabPoint_Barrel and glue the barrel on."""
        gb = self.pose(self.grab_barrel)
        pre = self.sim.multiplyPoses(gb, [*vscale(list(APPROACH_AXIS), APPROACH_DIST), 0.0, 0.0, 0.0, 1.0])
        print("  approaching the barrel...")
        self.move_tool(pre)
        print("  closing in on GrabPoint_Barrel...")
        err = self.move_tool(gb)
        gap = self.glue_barrel()
        print(f"  dummy gap before snap {err * 1000:.1f} mm, after glue {gap * 1000:.2f} mm")
        ok = err < GLUE_TOL_M * 3 and gap < GLUE_TOL_M
        return ok

    def lift(self):
        p = self.pose(self.grab_ur5)
        self.move_tool([p[0], p[1], p[2] + LIFT_HEIGHT] + p[3:])

    def rotate_sideways(self):
        """Stage 2: rotate the held barrel about the grab point until its axis is horizontal."""
        axis = self.barrel_axis_world()
        radial = vsub(self.pos(self.barrel)[:2], self.pos(self.ur5)[:2])
        before = axis_elevation_deg(axis)
        q_rot = sideways_rotation(axis, radial)
        p = self.pose(self.grab_ur5)
        q_new = quat_mul(q_rot, p[3:])
        print(f"  barrel axis is {before:.0f} deg off horizontal, rotating...")
        self.move_tool(p[:3] + q_new, steps=ROTATE_STEPS)
        after = axis_elevation_deg(self.barrel_axis_world())
        print(f"  barrel axis now {after:.1f} deg from horizontal")
        return after < SIDEWAYS_TOL_DEG

    def lower_and_release(self, target_xy):
        """Stage 4 end: put the sideways barrel over target_xy, lower to the floor, let go."""
        center = self.pos(self.barrel)
        tool = self.pose(self.grab_ur5)
        dx = target_xy[0] - center[0]
        dy = target_xy[1] - center[1]
        self.move_tool([tool[0] + dx, tool[1] + dy, tool[2]] + tool[3:])                   # slide over the spot
        dz = (self.floor_z + FLOOR_CLEARANCE) - self.world_min_z(self.barrel)
        tool = self.pose(self.grab_ur5)
        self.move_tool([tool[0], tool[1], tool[2] + dz] + tool[3:])                        # lower
        self.release_barrel()
        final = self.pos(self.barrel)
        return final, axis_elevation_deg(self.barrel_axis_world())

    def swing_to(self, target_xy, steps=MOVE_STEPS, settle=40):
        """
        Swing the held barrel around the UR5's base axis (same radius, same height) until it sits on the
        line from the arm base to target_xy. Used at PointB so the arm can reach it from any side.
        (A straight-line move between opposite sides would pass through the arm's own base.)
        """
        base = self.pos(self.ur5)[:2]
        cur = vsub(self.pos(self.barrel)[:2], base)
        want = vsub(target_xy, base)
        dtheta = wrap_angle(math.atan2(want[1], want[0]) - math.atan2(cur[1], cur[0]))
        if abs(dtheta) < math.radians(2.0):
            return
        print(f"  swinging the barrel {math.degrees(dtheta):+.0f} deg around the arm base toward the drop spot...")
        start = self.pose(self.grab_ur5)
        p0, q0 = start[:3], start[3:]
        rel0 = vsub(p0[:2], base)
        misses = 0
        last = start
        for i in range(1, steps + 1):
            a = dtheta * i / steps
            xy = vadd(base, rot2(rel0, a))
            last = [xy[0], xy[1], p0[2]] + quat_mul(quat_axis_angle([0.0, 0.0, 1.0], a), q0)
            if not self._ik_to(last):
                misses += 1
            self.step()
        for _ in range(settle):
            self._ik_to(last)
            self.step()
        err = vnorm(vsub(self.pos(self.grab_ur5), last[:3]))
        if misses > steps * 0.3 or err > 0.03:
            print(f"  warning: IK trouble during the swing (unsolved {misses}/{steps}, final tool error "
                  f"{err * 1000:.0f} mm). Try a different --standoff.")

    # ---------- omni driving ----------
    def _set_wheels(self, cmd):
        for w, c in zip(self.wheels, cmd):
            self.sim.setJointTargetVelocity(w, c)

    def _wheel_burst(self, i):
        """
        Spin ONLY wheel i, measure what the platform does. Returns the response in the platform's own
        frame: [forward-ish m/s, sideways-ish m/s, yaw rate rad/s] per rad/s of that wheel.
        (Nothing is assumed about which wheel does what, or which way the signs go.)
        """
        sim = self.sim
        dt = sim.getSimulationTimeStep()
        cmd = [0.0] * 4
        cmd[i] = WHEEL_V * CAL_SCALE
        self._set_wheels(cmd)
        half = CAL_STEPS // 2
        self.step(half)                                       # let the wheel spin up, discard this part
        if i == 0:
            try:                                              # is something else fighting our wheel command?
                got = sim.getJointTargetVelocity(self.wheels[0])
                if abs(got - cmd[0]) > 1e-3:
                    print(f"  WARNING: wheel 0 target velocity reads {got:.3f} after we set {cmd[0]:.3f}. "
                          f"Something else (the OmniPlatform's own child script?) is overwriting the wheels. "
                          f"Disable that script in CoppeliaSim.")
            except Exception:
                pass
        p0, y0 = self.pos(self.ur5)[:2], self.platform_yaw()
        self.step(CAL_STEPS - half)
        p1, y1 = self.pos(self.ur5)[:2], self.platform_yaw()
        self._set_wheels([0.0] * 4)
        self.step(CAL_SETTLE_STEPS)
        t = (CAL_STEPS - half) * dt
        v_world = vscale(vsub(p1, p0), 1.0 / t)
        v_body = rot2(v_world, -(y0 + wrap_angle(y1 - y0) / 2.0))        # world -> platform frame
        w = wrap_angle(y1 - y0) / t
        k = 1.0 / (WHEEL_V * CAL_SCALE)
        return [v_body[0] * k, v_body[1] * k, w * k]

    def calibrate_omni(self):
        print("  calibrating wheels (one wheel at a time, short bursts)...")
        self.heading0 = self.platform_yaw()            # the heading we will hold while driving
        cols = [self._wheel_burst(i) for i in range(4)]
        self.A = [[cols[k][r] for k in range(4)] for r in range(3)]       # 3 x 4: body (vx, vy, yaw) per wheel
        names = ["back right", "front right", "front left", "back left"]
        print("    response per wheel (rad/s):   body-x m/s | body-y m/s | yaw rad/s")
        for k in range(4):
            print(f"      wheel {k} ({names[k]:11s}): {cols[k][0]:+.4f} | {cols[k][1]:+.4f} | {cols[k][2]:+.4f}")
        if max(abs(x) for c in cols for x in c) < 1e-4:
            raise RuntimeError("No wheel moved the platform. Check the wheel paths and that the wheel joints are "
                               "velocity-controlled and the platform is not fixed/static.")
        if solve_min_norm(self.A, [1.0, 0.0, 0.0]) is None:
            raise RuntimeError("The four wheels cannot produce all of forward, sideways and turning motion "
                               "(calibration matrix is singular). Paste the response table above to me.")
        self.step(10)

    def drive_to(self, goal_xy, tol=DRIVE_TOL):
        """
        Drive the UR5 base to goal_xy (world xy) with the calibrated wheel model, holding the start heading.
        Stops early if the platform runs into something (no progress although we are still commanding motion).
        Sets self.blocked to say whether that happened.
        """
        dt = self.sim.getSimulationTimeStep()
        self.blocked = False
        anchor, commanded = self.pos(self.ur5)[:2], 0.0           # progress bookkeeping for contact detection
        for n in range(DRIVE_MAX_STEPS):
            p = self.pos(self.ur5)[:2]
            err = vsub(goal_xy, p)
            dist = vnorm(err)
            yaw = self.platform_yaw()
            yaw_err = wrap_angle(self.heading0 - yaw)
            if dist < tol:
                break
            speed = min(DRIVE_MAX_SPEED, DRIVE_GAIN * dist)
            v_world = vscale(err, speed / dist)
            v_body = rot2(v_world, -yaw)                                          # into the platform's frame
            w_des = max(-YAW_MAX_RATE, min(YAW_MAX_RATE, YAW_GAIN * yaw_err))
            c = solve_min_norm(self.A, [v_body[0], v_body[1], w_des])
            peak = max(abs(x) for x in c)
            if peak > WHEEL_MAX:
                c = [x * WHEEL_MAX / peak for x in c]
            self._set_wheels(c)
            self.step()
            commanded += speed * dt
            if n % 100 == 0:
                print(f"    t={n:4d}  {dist * 1000:5.0f} mm to go, heading off by {math.degrees(yaw_err):+5.1f} deg")

            if STOP_ON_CONTACT and (n + 1) % STALL_WINDOW == 0:
                moved = vsub(self.pos(self.ur5)[:2], anchor)
                toward = vdot(moved, vunit(err))                                  # progress toward the goal
                if n + 1 > STALL_WINDOW and dist > 2 * tol and toward < max(STALL_MIN_M, STALL_MIN_FRAC * commanded):
                    self.blocked = True
                    print(f"  platform is not making progress ({dist * 1000:.0f} mm from the goal): it has run into "
                          f"something, stopping here.")
                    break
                anchor, commanded = self.pos(self.ur5)[:2], 0.0
        else:
            print("  (drive timed out before reaching the goal)")
        self._set_wheels([0.0] * 4)
        self.step(CAL_SETTLE_STEPS)
        return vnorm(vsub(goal_xy, self.pos(self.ur5)[:2]))



#### STAGES
def run_ur5_only(robot):
    """Stages 1-2."""
    print("== Stage 1-2: robot parked next to the barrel, UR5 only ==")
    robot.begin_run(near_barrel=True)
    try:
        ok1 = robot.grab()
        print(f"  Stage 1 (grab): {'PASS' if ok1 else 'FAIL'}")
        robot.lift()
        ok2 = robot.rotate_sideways()
        robot.step(60)
        print(f"  Stage 2 (sideways): {'PASS' if ok2 else 'FAIL'}")
        return ok1 and ok2
    finally:
        robot.end_run()


def run_full(robot):
    """Stages 3-4."""
    print("== Stage 3-4: omni drives to the barrel, grabs, drives to PointB, places it sideways ==")
    robot.begin_run(near_barrel=False)
    try:
        robot.calibrate_omni()

        barrel_xy = robot.pos(robot.barrel)[:2]
        start_xy  = robot.pos(robot.ur5)[:2]
        goal = standoff_goal(barrel_xy, start_xy, ARM_STANDOFF)
        print(f"  driving from ({start_xy[0]:.2f}, {start_xy[1]:.2f}) to ({goal[0]:.2f}, {goal[1]:.2f}) near the barrel...")
        miss = robot.drive_to(goal)
        print(f"  parked {miss * 1000:.0f} mm from the goal")

        ok3 = robot.grab()
        print(f"  Stage 3 (drive + grab): {'PASS' if ok3 else 'FAIL'}")
        robot.lift()

        # Park on the side we arrive from, so PointB is straight ahead at the same reach the grab used. The arm
        # then swings the barrel around to face PointB, so PointB can be in ANY direction (even behind the barrel).
        rel = vsub(robot.pos(robot.barrel)[:2], robot.pos(robot.ur5)[:2])
        reach = vnorm(rel)
        b_xy = robot.pos(robot.point_b)[:2]
        cur_xy = robot.pos(robot.ur5)[:2]
        if robot.platform_at_b:
            plat_xy  = robot.pos(robot.platform)[:2]
            ur5_goal = vadd(b_xy, vsub(cur_xy, plat_xy))                         # platform centre ends on PointB
            place_xy = vadd(ur5_goal, rel)
        else:
            ur5_goal = standoff_goal(b_xy, cur_xy, reach)                        # barrel (not platform) ends on PointB
            place_xy = b_xy
        print(f"  driving to PointB ({b_xy[0]:.2f}, {b_xy[1]:.2f}) with the barrel...")
        miss = robot.drive_to(ur5_goal)
        print(f"  parked {miss * 1000:.0f} mm from the goal")
        robot.swing_to(place_xy)

        ok_side = robot.rotate_sideways()
        final, elev = robot.lower_and_release(place_xy)
        off = vnorm(vsub(final[:2], place_xy))
        ok4 = ok_side and off < PLACE_TOL_M and elev < SIDEWAYS_TOL_DEG * 3
        print(f"  barrel ended {off * 1000:.0f} mm from the target spot, axis {elev:.1f} deg from horizontal")
        print(f"  Stage 4 (drive + place sideways): {'PASS' if ok4 else 'FAIL'}")
        return ok3 and ok4
    finally:
        robot.end_run()



#### MAIN
def main():
    ap = argparse.ArgumentParser(description="Barrel pick-and-place with OmniPlatform + UR5")
    ap.add_argument("--mode", choices=["ur5", "full", "all"], default="all",
                    help="ur5 = stages 1-2, full = stages 3-4, all = both (default)")
    ap.add_argument("--repeat", type=int, default=1, help="how many times to repeat the chosen mode")
    ap.add_argument("--discover", action="store_true", help="list dummies/shapes and what was resolved, then exit")
    ap.add_argument("--standoff", type=float, default=None, help="UR5-base-to-barrel distance when parked (m)")
    ap.add_argument("--platform-at-b", action="store_true",
                    help="put the platform itself on PointB (default: the BARREL is placed on PointB)")
    args = ap.parse_args()

    global ARM_STANDOFF
    if args.standoff is not None:
        ARM_STANDOFF = args.standoff

    robot = Robot(platform_at_b=args.platform_at_b)
    if args.discover:
        robot.discover()
        return 0

    results = []
    for n in range(1, args.repeat + 1):
        print(f"\n######## Run {n} of {args.repeat} ########")
        if args.mode in ("ur5", "all"):
            results.append(("ur5", n, run_ur5_only(robot)))
        if args.mode in ("full", "all"):
            results.append(("full", n, run_full(robot)))

    print("\n######## Summary ########")
    for mode, n, ok in results:
        print(f"  run {n} {mode:5s}: {'PASS' if ok else 'FAIL'}")
    return 0 if all(ok for _, _, ok in results) else 1


if __name__ == "__main__":
    sys.exit(main())
