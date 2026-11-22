"""
Description: Barrel pick-and-place with the OmniPlatform + UR5 in CoppeliaSim

    Stages (from the project plan)
        1. Park the robot near the barrel (NO omni motion). Only the UR5 moves, until the
           two dummies GrabPoint_UR5 and GrabPoint_Barrel coincide, then the barrel is
           "glued" to the UR5.
        2. UR5 rotates the glued barrel so it lies sideways (barrel axis horizontal).
        3. Start over. This time the OmniPlatform drives from its starting position to the
           barrel, and the UR5 grabs it (same as stage 1).
        4. OmniPlatform drives to PointB (the white box; --place-on NAME picks another shape), the UR5 turns the barrel sideways (same as stage 2),
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
        the walking human (Walkin' Bill): found by an alias containing "bill", "human", "person", ...
                           (or pass --human NAME, or --no-human-sensor to turn the safety off)

Outputs:
    Simulated pick, rotate, drive, place. Prints a PASS/FAIL line per stage.
    If the human comes within HUMAN_STOP_RADIUS of the arm, the wheels stop and the arm retracts at once.

NOTES:
    - Run python barrel_pick_place.py --discover first. It lists the dummies/shapes it found,
      so a wrong name shows up immediately instead of mid-run.
    - Omni wheel mixing is CALIBRATED at the start of every drive (short test bursts), so you
      do not need to know which way the platform's wheel signs point.
    - The barrel is "glued" by making it static, non-respondable, and parenting it to
      GrabPoint_UR5 after snapping GrabPoint_Barrel exactly onto GrabPoint_UR5.
    - Everything tunable is in the CONSTANTS block right below the imports.
    - HUMAN SAFETY: a virtual proximity sensor measures the horizontal distance between the human and the
      nearest point of the arm (every UR5 joint, the tool point, and the barrel while it is held). Inside
      HUMAN_STOP_RADIUS the wheels are stopped and the UR5 joints are sent straight back to their start pose
      (no IK, so it cannot fail to solve). Once the human has stayed outside HUMAN_STOP_RADIUS +
      HUMAN_CLEAR_MARGIN for a while, the arm eases back out and the interrupted move carries on. If the
      human never leaves (HUMAN_MAX_WAIT_STEPS) the run is aborted and counted as FAIL.
      The human's position is read from the model's base and its largest shapes, so it works whichever of
      them the walking script moves. The robot side of the distance is every arm point, the held barrel AND the
      edge of the platform chassis, so the wheels also stop for a human walking up to the body.
    - The human is NOT part of the static obstacle list used by the bump checker / parking-spot search (that list
      is frozen at start-up, which is wrong for someone who walks); the proximity sensor tracks him live instead.
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
CAL_SCALE        = 0.7                # calibration burst speed, as a multiple of WHEEL_V
CAL_STEPS        = 24                 # sim steps per calibration burst (one wheel at a time, forward then back)
SYNTH_ALIAS      = "SyntheticBill"     # the stand-in human made by --sim-human
SYNTH_SIZE       = (0.5, 0.5, 1.7)     # m, a cylinder about as wide and tall as a person


def ping_pong(s, x_min, x_max):
    """Position along a straight walk that goes x_min -> x_max -> x_min -> ... after s metres of walking."""
    length = x_max - x_min
    u = s % (2.0 * length)
    return x_min + (u if u <= length else 2.0 * length - u)


# Bill's walking cycle, copied from scenes/bill_walking.lua (22 samples over one 1 m stride, linear in between)
BILL_STRIDE_M = 1.0
BILL_GAIT = {
    "Leg":      [0.237,0.228,0.175,-0.014,-0.133,-0.248,-0.323,-0.450,-0.450,-0.442,-0.407,-0.410,-0.377,-0.303,-0.178,-0.111,-0.010,0.046,0.104,0.145,0.188,0.237],
    "Knee":     [0.282,0.403,0.577,0.929,1.026,1.047,0.939,0.664,0.440,0.243,0.230,0.320,0.366,0.332,0.269,0.222,0.133,0.089,0.065,0.073,0.092,0.282],
    "Ankle":    [-0.133,0.041,0.244,0.382,0.304,0.232,0.266,0.061,-0.090,-0.145,-0.043,0.041,0.001,0.011,-0.099,-0.127,-0.121,-0.120,-0.107,-0.100,-0.090,-0.133],
    "Shoulder": [0.028,0.043,0.064,0.078,0.091,0.102,0.170,0.245,0.317,0.337,0.402,0.375,0.331,0.262,0.188,0.102,0.094,0.086,0.080,0.051,0.058,0.028],
    "Elbow":    [-1.148,-1.080,-1.047,-0.654,-0.517,-0.366,-0.242,-0.117,-0.078,-0.058,-0.031,-0.001,-0.009,0.008,-0.108,-0.131,-0.256,-0.547,-0.709,-0.813,-1.014,-1.148],
}


def gait_value(table, t):
    """Linear interpolation of a gait table at phase t in [0, 1)."""
    f = (t % 1.0) * (len(table) - 1)
    i = min(int(f), len(table) - 2)
    return table[i] + (table[i + 1] - table[i]) * (f - i)


REPLY_TIMEOUT_S  = 90                 # s, give up if CoppeliaSim does not answer a call for this long (a hang, not a slow step)
CAL_IN_PLACE     = True               # calibrate where the platform stands (small bursts); False = drive to a clear spot first
CAL_CLEAR_M      = 0.6                # m, calibration happens where the platform is at least this far from everything
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
BUMP_STOP        = True               # stop the wheels as soon as the platform body touches ANY solid object
BUMP_MARGIN      = 0.02               # m, "touching" = platform body within this distance of the object
BUMP_CHECK_EVERY = 2                  # sim steps between bump checks
FLOOR_TOP_Z      = 0.05               # m, shapes whose top is below this are treated as the floor (not an obstacle)
ROOM_SIZE_M      = 3.0                # m, a shape wider than this in BOTH x and y is the room/floor, not an obstacle
BUMP_IGNORE      = []                 # aliases of shapes the bump checker should ignore
PARK_CLEARANCE   = 0.05               # m, platform body stays at least this far from every obstacle when parked
PARK_MAX_DIST    = 1.6                # m, how far back from the target we search for a clear parking spot
MIN_PARK_DIST    = 0.35               # m, closest the arm base is allowed to park to the drop spot
CARRY_RADIUS     = 0                  # m, the held barrel is pulled in to this distance from the arm base after the grab
PLACE_REACH_MAX  = 0.80               # m, farthest horizontal reach asked of the arm when placing
RAISE_CLEAR      = 0.05               # m, barrel bottom stays this far above a surface it is swung over

# --- arm / grasp geometry ---
ARM_STANDOFF     = 0.55    # m, horizontal distance from UR5 base to the barrel when parked for the grab
APPROACH_AXIS    = (0.0, 0.0, 1.0)   # direction (in GrabPoint_Barrel frame) the gripper comes in from
APPROACH_DIST    = 0.12    # m, pre-grasp offset along APPROACH_AXIS
LIFT_HEIGHT      = 0.15    # m, lift after the grab before rotating
FLOOR_CLEARANCE  = 0.01    # m, barrel is released this far above the floor
MOVE_STEPS       = 120     # sim steps for a normal arm move
ROTATE_STEPS     = 160     # sim steps for the sideways rotation
SETTLE_STEPS     = 100     # sim steps after release to let the barrel settle
SETTLE_MAX_STEPS = 300     # sim steps an arm move may wait for the arm to actually arrive (it lags the IK target)

# --- pass/fail thresholds ---
GLUE_TOL_M       = 0.01    # m, dummies must be this close for the grab to count
ROTATE_LIFT      = 0.0     # m, raise the held barrel this much while turning it sideways (more room for the wrist); --rotate-lift
SIDEWAYS_TOL_DEG = 5.0     # deg, barrel axis must be this close to horizontal
PLACE_ON         = "pointb"              # where to set the barrel: "pointb" = on PointB (the white box in this scene), "white" = the whitest shape, or part of a shape's name
PLACE_TOL_M      = 0.10    # m, barrel final xy must be this close to PointB

# --- human safety (proximity sensor -> retract the arm) ---
HUMAN_ALIASES        = ["bill", "human", "person", "pedestrian", "walker"]   # alias substrings used to find the human (--human overrides)
HUMAN_STOP_RADIUS    = 0.8     # m, a human closer than this to ANY part of the arm (or the held barrel) trips the sensor
HUMAN_CLEAR_MARGIN   = 0.2     # m, the human must get this much farther than the stop radius to count as clear
HUMAN_CLEAR_CHECKS   = 5      # consecutive sensor readings the human must stay clear before the arm goes back out
HUMAN_CHECK_EVERY    = 2       # sim steps between sensor readings
HUMAN_SETTLE_STEPS   = 60      # extra sim steps allowed for the joints to arrive at the resume pose before IK takes over
HUMAN_RESUME_STEPS   = 25      # sim steps taken to re-extend the arm after the human has left
HUMAN_MAX_WAIT_STEPS = 2400    # sim steps to wait for the human to leave before giving up (the run is then aborted)



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

#### BUILT-IN IK (used when CoppeliaSim's simIK plugin cannot be loaded, or with --builtin-ik)
def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

def mat_vec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]

def mat_T(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]

def quat_to_mat(q):
    """3x3 rotation matrix of a quaternion [x, y, z, w]."""
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    x, y, z, w = (c / n for c in q)
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]

def axis_angle_mat(axis, angle):
    """Rotation matrix for a turn of `angle` about `axis` (Rodrigues)."""
    ux, uy, uz = vunit(axis)
    c, s = math.cos(angle), math.sin(angle)
    k = 1.0 - c
    return [[c + ux * ux * k, ux * uy * k - uz * s, ux * uz * k + uy * s],
            [uy * ux * k + uz * s, c + uy * uy * k, uy * uz * k - ux * s],
            [uz * ux * k - uy * s, uz * uy * k + ux * s, c + uz * uz * k]]

def mat_to_rotvec(R):
    """Rotation vector (axis * angle, angle in [0, pi]) of a rotation matrix."""
    cos_a = max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0))
    angle = math.acos(cos_a)
    if angle < 1e-9:
        return [0.0, 0.0, 0.0]
    if math.pi - angle < 1e-4:                    # near 180 deg sin(angle) ~ 0, so read the axis off the diagonal
        k = max(range(3), key=lambda i: R[i][i])
        a = [0.0, 0.0, 0.0]
        a[k] = math.sqrt(max(0.0, (R[k][k] + 1.0) / 2.0))
        for j in range(3):
            if j != k:
                a[j] = (R[k][j] + R[j][k]) / (4.0 * a[k]) if a[k] > 1e-9 else 0.0
        return vscale(vunit(a), angle)
    f = angle / (2.0 * math.sin(angle))
    return vscale([R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]], f)

def solve_linear(A, b):
    """Solve A x = b by Gaussian elimination with partial pivoting. Returns None if A is singular."""
    n = len(b)
    M = [list(A[i]) + [b[i]] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-12:
            return None
        M[c], M[p] = M[p], M[c]
        for r in range(c + 1, n):
            f = M[r][c] / M[c][c]
            if f:
                for k in range(c, n + 1):
                    M[r][k] -= f * M[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (M[r][n] - sum(M[r][k] * x[k] for k in range(r + 1, n))) / M[r][r]
    return x

class ScrewChain:
    """
    Forward kinematics and damped-least-squares IK for a serial chain of revolute joints, described by where
    each joint axis sits when the arm is in the snapshot configuration q0 (product-of-exponentials model).
    Everything is expressed in the chain's BASE frame, so the platform driving around does not matter.

        axes[i], points[i]   unit axis and a point on it for joint i, in the base frame, at q0
        tool_pose            tool pose [x, y, z, qx, qy, qz, qw] in the base frame, at q0
        limits[i]            (lo, hi) in rad, or None for a joint without limits
    """
    def __init__(self, axes, points, q0, tool_pose, limits=None):
        self.n = len(axes)
        self.axes = [vunit(a) for a in axes]
        self.points = [list(p) for p in points]
        self.q0 = list(q0)
        self.R0 = quat_to_mat(tool_pose[3:])
        self.p0 = list(tool_pose[:3])
        self.limits = list(limits) if limits else [None] * self.n

    def _frames(self, q):
        R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        t = [0.0, 0.0, 0.0]
        axes_w, points_w = [], []
        for i in range(self.n):
            axes_w.append(mat_vec(R, self.axes[i]))                       # joint i as it sits for this q
            points_w.append(vadd(mat_vec(R, self.points[i]), t))
            Ri = axis_angle_mat(self.axes[i], q[i] - self.q0[i])
            ti = vsub(self.points[i], mat_vec(Ri, self.points[i]))        # a turn about an axis that does not pass through 0
            t = vadd(mat_vec(R, ti), t)
            R = mat_mul(R, Ri)
        return R, t, axes_w, points_w

    def fk(self, q):
        """(R, p): tool rotation matrix and position in the base frame for joint angles q."""
        R, t, _, _ = self._frames(q)
        return mat_mul(R, self.R0), vadd(mat_vec(R, self.p0), t)

    def jacobian(self, q):
        """6 x n geometric Jacobian: rows 0-2 tool velocity, rows 3-5 angular velocity, per unit joint speed."""
        R, t, axes_w, points_w = self._frames(q)
        tool = vadd(mat_vec(R, self.p0), t)
        cols = [vcross(axes_w[i], vsub(tool, points_w[i])) + axes_w[i] for i in range(self.n)]
        return [[cols[k][r] for k in range(self.n)] for r in range(6)]

    def pose_error(self, q, target_R, target_p):
        R, p = self.fk(q)
        return vsub(target_p, p), mat_to_rotvec(mat_mul(target_R, mat_T(R)))

    def solve(self, q_start, target_R, target_p, max_iter=99, damping=0.1,
              tol_pos=5e-4, tol_ang=5e-3, max_step=0.3):
        """
        Damped least squares from q_start toward the target pose (matrix + position, base frame).
        Returns (q, converged, position error m, orientation error rad). Always returns the best joint
        angles it reached, so an out-of-reach target still moves the arm as close as it can get.
        """
        q = list(q_start)
        for _ in range(max_iter):
            e_pos, e_rot = self.pose_error(q, target_R, target_p)
            ep, ea = vnorm(e_pos), vnorm(e_rot)
            if ep < tol_pos and ea < tol_ang:
                return q, True, ep, ea
            J = self.jacobian(q)
            e = e_pos + e_rot
            A = [[sum(J[i][k] * J[j][k] for k in range(self.n)) + (damping * damping if i == j else 0.0)
                  for j in range(6)] for i in range(6)]
            y = solve_linear(A, e)                                        # dq = J^T (J J^T + damping^2 I)^-1 e
            if y is None:
                break
            dq = [sum(J[r][k] * y[r] for r in range(6)) for k in range(self.n)]
            big = max(abs(x) for x in dq)
            if big > max_step:                                            # never swing a joint wildly in one iteration
                dq = [x * max_step / big for x in dq]
            q = [q[k] + dq[k] for k in range(self.n)]
            for k, lim in enumerate(self.limits):
                if lim is not None:
                    q[k] = min(lim[1], max(lim[0], q[k]))
        e_pos, e_rot = self.pose_error(q, target_R, target_p)
        ep, ea = vnorm(e_pos), vnorm(e_rot)
        return q, (ep < tol_pos and ea < tol_ang), ep, ea


def min_pair_distance(pts_a, pts_b):
    """Smallest horizontal (xy) distance between any point of pts_a and any point of pts_b."""
    best = float("inf")
    for a in pts_a:
        for b in pts_b:
            d = math.hypot(a[0] - b[0], a[1] - b[1])
            if d < best:
                best = d
    return best

def point_box_distance(p, lo, hi):
    """Horizontal (xy) distance from point p to the box lo..hi; 0 if p is over the box."""
    dx = max(lo[0] - p[0], 0.0, p[0] - hi[0])
    dy = max(lo[1] - p[1], 0.0, p[1] - hi[1])
    return math.hypot(dx, dy)

class HumanTooClose(RuntimeError):
    """The human stayed inside the stop radius for longer than HUMAN_MAX_WAIT_STEPS."""

class ProximitySensor:
    """
    Turns a stream of distances into a near/clear state with hysteresis, so a human hovering right at the edge
    of the stop radius does not make the arm twitch in and out.
        trips (near = True)  as soon as a reading is below stop_radius
        clears (near = False) only after clear_checks readings in a row above clear_radius
    """
    def __init__(self, stop_radius, clear_radius, clear_checks):
        self.stop_radius  = stop_radius
        self.clear_radius = max(clear_radius, stop_radius)
        self.clear_checks = clear_checks
        self.reset()

    def reset(self):
        self.near = False
        self._clear_count = 0

    def update(self, dist):
        if not self.near:
            if dist < self.stop_radius:
                self.near = True
                self._clear_count = 0
        elif dist > self.clear_radius:
            self._clear_count += 1
            if self._clear_count >= self.clear_checks:
                self.near = False
        else:
            self._clear_count = 0
        return self.near



#### SIM WRAPPER
class Robot:
    """Everything that talks to CoppeliaSim lives in here."""

    def __init__(self, human_name=None, use_human_sensor=True, force_builtin_ik=False, synthetic_human=None):
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient   # imported here so the math above is testable offline
        self.client = RemoteAPIClient()
        try:                                              # never hang forever on a CoppeliaSim that has stopped answering
            import zmq
            self.client.socket.setsockopt(zmq.RCVTIMEO, REPLY_TIMEOUT_S * 1000)
        except Exception:
            pass
        self.sim    = self.client.require("sim")
        self.simIK  = None
        self.arm_ik = None                 # ScrewChain, used instead of simIK when the plugin is unavailable
        self.ik_chain = []
        if force_builtin_ik:
            print("IK: using the built-in solver (--builtin-ik).")
        else:
            try:
                self.simIK = self.client.require("simIK")
            except Exception as exc:
                msg = str(exc).strip().splitlines()[-1] if str(exc).strip() else type(exc).__name__
                print(f"IK: CoppeliaSim's simIK plugin could not be loaded ({msg}).")
                print("    Falling back to the built-in IK solver; everything else works the same.")
                self.simIK = None
        self.ik_env = self.ik_group = self.ik_target = None
        self.A = None                      # wheel response matrix, filled in once by calibrate_once()
        self.floor_box = None
        # human safety state
        self.human_name       = human_name
        self.use_human_sensor = use_human_sensor
        self.synthetic        = synthetic_human   # None, or dict(speed, pass_y, x_min, x_max) for --sim-human
        self._synth           = None
        self._synth_s         = 0.0
        self._synth_is_bill   = False      # True when --sim-human is walking the scene's own Bill model
        self._bill_joints     = {}
        self._bill_z          = 0.0
        self._bill_scripts    = []
        self._bill_check      = None       # expected position of Bill, checked once after the first step
        self.human            = None       # handle of the human model (top of its tree)
        self.human_points     = []         # handles whose positions together describe where the human is
        self.human_sensor     = ProximitySensor(HUMAN_STOP_RADIUS, HUMAN_STOP_RADIUS + HUMAN_CLEAR_MARGIN, HUMAN_CLEAR_CHECKS)
        self.guard_on         = False      # True only while a run is in progress
        self.guard_intervened = False      # set after a retract/resume so loops can reset their progress bookkeeping
        self.human_trips      = 0
        self.barrel_held      = False
        self._guarding        = False      # stops the guard from re-entering itself
        self._guard_tick      = 0
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
        self._resolve_human()                # before the bump monitor: it must know which shapes are the human
        self._build_bump_monitor()

    def _make_synthetic(self):
        """--sim-human: make a person-sized cylinder that Python walks back and forth at a speed we choose."""
        sim = self.sim
        for h in self._all_objects():                                        # clear one left over from a crashed run
            if sim.getObjectAlias(h, 0) == SYNTH_ALIAS:
                sim.removeObjects([h])
        if not self.synthetic.get("cylinder") and self._adopt_bill():
            return
        h = sim.createPrimitiveShape(sim.primitiveshape_cylinder, list(SYNTH_SIZE), 0)
        sim.setObjectAlias(h, SYNTH_ALIAS)
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)              # we move it ourselves; it must not push anything
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
        try:
            sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, [1.0, 0.45, 0.0])
        except Exception:
            pass
        self._synth, self._synth_s = h, 0.0
        self._place_synthetic()
        self.human_name = SYNTH_ALIAS
        others = [sim.getObjectAlias(o, 0) for o in self._all_objects() if sim.getObjectAlias(o, 0) == "Bill"]
        c = self.synthetic
        print(f"  --sim-human: walking a stand-in human at {c['speed']:.2f} m/s along y = {c['pass_y']:+.2f} m, "
              f"x from {c['x_min']:+.1f} to {c['x_max']:+.1f} m and back")
        if others:
            print("  note: the scene's own 'Bill' is also there and will keep walking. For a clean test use a scene without "
                  "him, or set BILL_MODE = 0 and BILL_TEST_SPOT = 1 in his script.")

    def _adopt_bill(self):
        """--sim-human: use the scene's own Bill model as the walker (we move him and swing his limbs ourselves)."""
        sim = self.sim
        bills = [h for h in self._all_objects()
                 if sim.getObjectAlias(h, 0) == "Bill" and len(sim.getObjectsInTree(h, sim.object_shape_type, 0)) > 0]
        if not bills:
            print("  --sim-human: no model called 'Bill' in this scene, so using an orange cylinder instead.")
            return False
        bill = bills[0]
        joints = {}
        for h in sim.getObjectsInTree(bill, sim.object_joint_type, 0):
            joints[norm_name(sim.getObjectAlias(h, 0))] = h
        need = [f"{side}{part}joint" for side in ("left", "right") for part in ("leg", "knee", "ankle", "shoulder", "elbow")]
        if any(n not in joints for n in need):
            print("  --sim-human: Bill's joints were not all found, so using an orange cylinder instead.")
            return False
        self._bill_joints = {n: joints[n] for n in need}
        self._bill_z = sim.getObjectPosition(bill, -1)[2]
        self._bill_scripts = []
        for h in [bill] + list(sim.getObjectsInTree(bill, sim.handle_all, 0)):
            try:
                if sim.getObjectType(h) == sim.sceneobject_script:
                    self._bill_scripts.append(h)
            except Exception:
                pass
        for kind in ("scripttype_simulation", "scripttype_childscript"):
            try:
                hs = sim.getScript(getattr(sim, kind), bill)
                if hs not in self._bill_scripts and hs != -1:
                    self._bill_scripts.append(hs)
            except Exception:
                pass
        self._synth, self._synth_is_bill, self._synth_s = bill, True, 0.0
        self.human_name = "Bill"
        c = self.synthetic
        print(f"  --sim-human: walking the scene's Bill at {c['speed']:.2f} m/s along y = {c['pass_y']:+.2f} m, "
              f"x from {c['x_min']:+.1f} to {c['x_max']:+.1f} m and back (his own walking script is switched off)")
        return True

    def _set_bill_script(self, on):
        sim = self.sim
        for sc in self._bill_scripts:
            done = False
            for fn in (lambda: sim.setBoolProperty(sc, "enabled", bool(on)),
                       lambda: sim.setScriptInt32Param(sc, sim.scriptintparam_enabled, 1 if on else 0)):
                try:
                    fn()
                    done = True
                    break
                except Exception:
                    continue
            if not done and not on:
                print("  warning: could not switch Bill's own walking script off")

    def claim_bill(self):
        """Call right after startSimulation(): stop Bill's own script and start our walk from the beginning."""
        if self._synth is None:
            return
        self._synth_s = 0.0
        if self._synth_is_bill:
            self._set_bill_script(False)
        self._place_synthetic()

    def _place_synthetic(self):
        c = self.synthetic
        sim = self.sim
        x = ping_pong(self._synth_s, c["x_min"], c["x_max"])
        if not self._synth_is_bill:
            sim.setObjectPosition(self._synth, -1, [x, c["pass_y"], SYNTH_SIZE[2] / 2.0])
            return
        ahead = ping_pong(self._synth_s + 0.05, c["x_min"], c["x_max"]) - x
        yaw = getattr(self, "_bill_yaw", 0.0) if abs(ahead) < 1e-9 else (0.0 if ahead > 0 else math.pi)
        self._bill_yaw = yaw
        pos = [x, c["pass_y"], self._bill_z]
        sim.setObjectPosition(self._synth, -1, pos)
        sim.setObjectOrientation(self._synth, -1, [0.0, 0.0, yaw])
        t1 = (self._synth_s / BILL_STRIDE_M) % 1.0
        t2 = (t1 + 0.5) % 1.0
        for side, t in (("left", t1), ("right", t2)):
            for part in ("leg", "knee", "ankle", "shoulder", "elbow"):
                sim.setJointPosition(self._bill_joints[f"{side}{part}joint"], gait_value(BILL_GAIT[part.capitalize()], t))
        self._bill_check = pos

    def _sim_step(self):
        """One simulation step; also advances the stand-in human (--sim-human) by one step of walking."""
        self.sim.step()
        if self._synth is not None:
            if self._synth_is_bill and self._bill_check is not None:
                got = self.sim.getObjectPosition(self._synth, -1)
                if math.dist(got[:2], self._bill_check[:2]) > 0.05:
                    print("  warning: Bill's own script is still moving him, so --sim-human speed will not match. "
                          "Run with --cylinder-human, or set BILL_MODE = 0 in his script.")
                self._bill_check = None
            self._synth_s += self.synthetic["speed"] * self.sim.getSimulationTimeStep()
            self._place_synthetic()

    def remove_synthetic(self):
        if self._synth is not None:
            if self._synth_is_bill:
                try:
                    self._set_bill_script(True)                              # give Bill his own walking script back
                except Exception:
                    pass
            else:
                try:
                    self.sim.removeObjects([self._synth])
                except Exception:
                    pass
            self._synth = None

    def _resolve_human(self):
        """Find the walking human (the topmost object whose alias matches) and the handles that track where he is."""
        sim = self.sim
        self.human, self.human_points = None, []
        if self.synthetic is not None and self._synth is None:
            self._make_synthetic()
        if not self.use_human_sensor:
            print("  human-proximity sensor DISABLED (--no-human-sensor): the arm will NOT retract for people.")
            return
        hints = [norm_name(self.human_name)] if self.human_name else [norm_name(h) for h in HUMAN_ALIASES]
        hits = [h for h in self._all_objects() if any(x in norm_name(sim.getObjectAlias(h, 0)) for x in hints)]
        hit_set = set(hits)
        robot_tree = set(sim.getObjectsInTree(self.platform, sim.handle_all, 0)) | {self.platform}
        tops = [h for h in hits if sim.getObjectParent(h) not in hit_set     # the model root, not its body parts
                and h not in robot_tree and h != self.barrel]
        # The scene also holds marker dummies such as BillSpot_1..3 (test positions for Bill) whose names contain "bill".
        # They are not people: a real human model has shapes in its tree, a marker dummy has none. Keep only the former,
        # and put an exact name match ("Bill") ahead of the rest, then the model with the most shapes.
        def n_shapes(h):
            return len(sim.getObjectsInTree(h, sim.object_shape_type, 0)) + (1 if sim.getObjectType(h) == sim.object_shape_type else 0)
        tops = [h for h in tops if n_shapes(h) > 0]
        tops.sort(key=lambda h: (norm_name(sim.getObjectAlias(h, 0)) not in hints, -n_shapes(h)))
        if not tops:
            raise RuntimeError(f"Could not find the human (looked for an alias containing {hints}). Run with --discover to "
                               f"see what is in the scene, then pass --human NAME, or use --no-human-sensor to run without it.")
        self.human = tops[0]
        if len(tops) > 1:
            print(f"  note: several objects look like the human ({[sim.getObjectAlias(h, 1) for h in tops]}); using the first. "
                  f"Pass --human NAME to choose.")

        def volume(h):
            bb = self._local_bbox(h)
            return (bb[3] - bb[0]) * (bb[4] - bb[1]) * (bb[5] - bb[2])
        shapes = sorted(sim.getObjectsInTree(self.human, sim.object_shape_type, 0), key=volume, reverse=True)
        pts = []
        for h in [self.human] + shapes[:3]:                                  # the root plus his 3 biggest shapes
            if h not in pts:
                pts.append(h)
        self.human_points = pts
        print(f"  human sensor: tracking '{sim.getObjectAlias(self.human, 1)}' ({len(pts)} points); the arm retracts if he "
              f"gets within {HUMAN_STOP_RADIUS:.2f} m of it")

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
        if self.human is not None:
            print(f"Human: {sim.getObjectAlias(self.human, 1)}  tracked via {[sim.getObjectAlias(h, 0) for h in self.human_points]}  "
                  f"distance to arm right now {self.human_distance():.2f} m (stop radius {HUMAN_STOP_RADIUS:.2f} m)")

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
            self._sim_step()
            if self.guard_on and not self._guarding:
                self._guard()

    # ---------- human safety ----------
    def arm_points(self):
        """Everything on the robot a person could be hurt by: every UR5 joint, the tool point, the held barrel."""
        pts = list(self.joints) + [self.grab_ur5]
        if self.barrel_held:
            pts.append(self.barrel)
        return pts

    def human_distance(self):
        """
        Horizontal distance (m) from the human to the nearest part of the robot: any arm point, the held barrel,
        or the edge of the platform chassis (so he is also caught walking up to the body). inf if there is no human.
        """
        if not self.human_points:
            return float("inf")
        human = [self.pos(h) for h in self.human_points]
        d = min_pair_distance(human, [self.pos(h) for h in self.arm_points()])
        if self.bump_body:
            lo, hi = self.body_box()
            d = min(d, min(point_box_distance(p, lo, hi) for p in human))
        return d

    def _joint_targets(self, qs):
        for j, q in zip(self.joints, qs):
            self.sim.setJointTargetPosition(j, q)

    def _guard(self):
        """Runs after every sim step while a run is active: read the sensor, react if the human is too close."""
        self._guard_tick += 1
        if self._guard_tick % HUMAN_CHECK_EVERY:
            return
        if self.human_sensor.update(self.human_distance()):
            self._respond_to_human()

    def _respond_to_human(self):
        """
        Wheels off, UR5 joints straight back to the start pose (direct joint targets, so no IK solve can fail),
        wait for the human to leave, ease the arm back to where it was, and hand control back to the interrupted move.
        """
        sim = self.sim
        self._guarding = True
        try:
            self.human_trips += 1
            resume_q = [sim.getJointPosition(j) for j in self.joints]
            print(f"  !! HUMAN NEAR ({self.human_distance():.2f} m from the arm): wheels stopped, retracting the arm")
            while True:
                self._set_wheels([0.0] * 4)
                self._joint_targets(self.joint_start)                           # retract NOW
                waited = 0
                while self.human_sensor.near:
                    self._sim_step()
                    waited += 1
                    if waited > HUMAN_MAX_WAIT_STEPS:
                        raise HumanTooClose(f"human stayed within {HUMAN_STOP_RADIUS:.2f} m of the arm for "
                                            f"{HUMAN_MAX_WAIT_STEPS} sim steps; giving up with the arm retracted.")
                    self.human_sensor.update(self.human_distance())
                    if waited % 200 == 0:
                        print(f"    ...still waiting: human is {self.human_distance():.2f} m away "
                              f"(needs > {HUMAN_STOP_RADIUS + HUMAN_CLEAR_MARGIN:.2f} m for {HUMAN_CLEAR_CHECKS} readings in a row)")
                print("  human is clear: easing the arm back out and resuming")
                interrupted = False
                for i in range(1, HUMAN_RESUME_STEPS + 1):
                    t = i / HUMAN_RESUME_STEPS
                    self._joint_targets([a + t * (b - a) for a, b in zip(self.joint_start, resume_q)])
                    self._sim_step()
                    if self.human_sensor.update(self.human_distance()):
                        print("  human came back: retracting again")
                        interrupted = True
                        break
                if not interrupted:                                             # let the joints really arrive before IK resumes
                    for _ in range(HUMAN_SETTLE_STEPS):
                        if max(abs(sim.getJointPosition(j) - q) for j, q in zip(self.joints, resume_q)) < 0.01:
                            break
                        self._joint_targets(resume_q)
                        self._sim_step()
                        if self.human_sensor.update(self.human_distance()):
                            print("  human came back: retracting again")
                            interrupted = True
                            break
                if not interrupted:
                    break
            sim.setObjectPose(self.ik_target, -1, self.pose(self.grab_ur5))   # IK carries on from where the arm really is
            self.reset_ik()
            print("  arm is back where it was; carrying on with the move")
            self.guard_intervened = True
        finally:
            self._guarding = False

    # ---------- simulation lifecycle ----------
    def begin_run(self, near_barrel):
        """Reset the scene to a clean start, optionally teleporting the platform next to the barrel."""
        sim = self.sim
        self._restore_barrel()
        sim.setObjectPose(self.platform, -1, self.platform_start_pose)
        for j, q in zip(self.joints, self.joint_start):
            sim.setJointPosition(j, q)
        if near_barrel:
            goal, _ = self.park_spot(self.pos(self.barrel)[:2], self.pos(self.ur5)[:2], ARM_STANDOFF)
            shift = vsub(goal, self.pos(self.ur5)[:2])
            p = sim.getObjectPosition(self.platform, -1)
            sim.setObjectPosition(self.platform, -1, [p[0] + shift[0], p[1] + shift[1], p[2]])

        sim.setStepping(True)
        sim.startSimulation()
        self.claim_bill()
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
        self.heading0 = self.platform_yaw()      # heading to hold while driving
        self.human_sensor.reset()                # the human watch is only on while the run is in progress
        self._guard_tick = 0
        self.guard_on = self.human is not None

    def end_run(self):
        self.guard_on = False
        try:
            if self.ik_env is not None and self.simIK is not None:
                self.simIK.eraseEnvironment(self.ik_env)
        except Exception:
            pass
        try:
            if self.ik_target is not None:
                self.sim.removeObjects([self.ik_target])
        except Exception:
            pass
        self.ik_env = self.ik_group = self.ik_target = None
        self.arm_ik = None
        for w in self.wheels:
            try:
                self.sim.setJointTargetVelocity(w, 0.0)
            except Exception:
                pass
        try:
            self.sim.stopSimulation(True)
            self._restore_barrel()
        except Exception as exc:                          # e.g. CoppeliaSim stopped answering: keep the ORIGINAL error visible
            print(f"  (could not stop the simulation cleanly: {type(exc).__name__})")

    def _restore_barrel(self):
        sim = self.sim
        self.barrel_held = False
        try:
            sim.setObjectParent(self.barrel, self.barrel_start_parent, True)
            sim.setObjectPose(self.barrel, -1, self.barrel_start_pose)
            sim.setObjectInt32Param(self.barrel, sim.shapeintparam_static, self.barrel_static0)
            sim.setObjectInt32Param(self.barrel, sim.shapeintparam_respondable, self.barrel_respond0)
        except Exception as exc:
            print(f"  (could not fully restore the barrel: {exc})")

    # ---------- IK ----------
    def _build_builtin_ik(self):
        """Read the arm's geometry from the scene once (joint axes, tool pose, limits), in the arm base frame."""
        sim = self.sim
        chain, h = [], self.grab_ur5
        while h != -1 and h != self.ur5:
            if sim.getObjectType(h) == sim.object_joint_type:
                chain.append(h)
            h = sim.getObjectParent(h)
        chain.reverse()                                            # base -> tip
        if not chain:
            raise RuntimeError("built-in IK: found no joints between the UR5 base and GrabPoint_UR5")
        axes, points, q0, limits = [], [], [], []
        for j in chain:
            jp = sim.getObjectPose(j, self.ur5)
            R = quat_to_mat(jp[3:])
            axes.append([R[0][2], R[1][2], R[2][2]])               # a CoppeliaSim joint turns about its local Z
            points.append(list(jp[:3]))
            q0.append(sim.getJointPosition(j))
            try:
                cyclic, (lo, rng) = sim.getJointInterval(j)
                limits.append(None if cyclic else (lo, lo + rng))
            except Exception:
                limits.append(None)
        self.ik_chain = chain
        self.arm_ik = ScrewChain(axes, points, q0, sim.getObjectPose(self.grab_ur5, self.ur5), limits)
        print(f"IK: built-in solver ready ({len(chain)} joints).")

    def _build_ik(self):
        sim, simIK = self.sim, self.simIK
        self.ik_target = sim.createDummy(0.02)
        sim.setObjectAlias(self.ik_target, "barrel_task_ik_target")
        sim.setObjectPose(self.ik_target, -1, self.pose(self.grab_ur5))
        if simIK is None:
            self._build_builtin_ik()
            return
        self._make_ik_group()

    def _make_ik_group(self):
        simIK = self.simIK
        self.ik_env = simIK.createEnvironment()
        self.ik_group = simIK.createGroup(self.ik_env)
        simIK.setGroupCalculation(self.ik_env, self.ik_group, simIK.method_damped_least_squares, 0.1, 99)
        try:
            constraints = simIK.constraint_pose
        except AttributeError:
            constraints = (simIK.constraint_x | simIK.constraint_y | simIK.constraint_z
                           | simIK.constraint_alpha_beta | simIK.constraint_gamma)
        simIK.addElementFromScene(self.ik_env, self.ik_group, self.ur5, self.grab_ur5, self.ik_target, constraints)

    def reset_ik(self):
        """Throw the IK world away and rebuild it from the scene as it is NOW. Done after the barrel is glued on or let
        go, and after every human stop, so the solver never works from a stale copy of the arm."""
        if self.simIK is None or self.ik_env is None:
            return
        try:
            self.simIK.eraseEnvironment(self.ik_env)
        except Exception:
            pass
        self._make_ik_group()

    def _ik_to_builtin(self, pose):
        sim = self.sim
        base = sim.getObjectPose(self.ur5, -1)
        Rb_T = mat_T(quat_to_mat(base[3:]))                        # world -> arm base frame
        target_p = mat_vec(Rb_T, vsub(list(pose[:3]), base[:3]))
        target_R = mat_mul(Rb_T, quat_to_mat(list(pose[3:])))
        q_now = [sim.getJointPosition(j) for j in self.ik_chain]
        q, ok, _, _ = self.arm_ik.solve(q_now, target_R, target_p)
        for j, qi in zip(self.ik_chain, q):
            sim.setJointTargetPosition(j, qi)
        return ok

    def _ik_to(self, pose):
        self.sim.setObjectPose(self.ik_target, -1, pose)
        if self.simIK is None:
            return self._ik_to_builtin(pose)
        res = self.simIK.handleGroup(self.ik_env, self.ik_group, {"syncWorlds": True})
        code = res[0] if isinstance(res, (tuple, list)) else res
        return code == self.simIK.result_success

    def _hold_until_reached(self, target_pose, tol=0.01, max_steps=SETTLE_MAX_STEPS):
        """Keep the IK target on target_pose until the real tool point is within tol of it (the arm lags behind)."""
        good, err = 0, float("inf")
        for _ in range(max_steps):
            self._ik_to(list(target_pose))
            self.step()
            cur = self.pose(self.grab_ur5)
            err = vnorm(vsub(cur[:3], target_pose[:3]))
            dot = abs(sum(a * b for a, b in zip(cur[3:], target_pose[3:])))
            ang = 2.0 * math.acos(min(1.0, dot))                              # orientation error, rad
            good = good + 1 if (err < tol and ang < math.radians(2.0)) else 0
            if good >= 5:
                break
        return err

    def move_tool(self, target_pose, steps=MOVE_STEPS, settle=SETTLE_MAX_STEPS):
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
        err = self._hold_until_reached(target_pose, max_steps=settle)        # wait for the arm to catch up
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
        self.barrel_held = True
        self.reset_ik()
        self.step(10)
        gap = vnorm(vsub(self.pos(self.grab_barrel), self.pos(self.grab_ur5)))
        return gap

    def release_barrel(self):
        sim = self.sim
        self.barrel_held = False
        sim.setObjectParent(self.barrel, -1, True)
        sim.setObjectInt32Param(self.barrel, sim.shapeintparam_respondable, self.barrel_respond0 or 1)
        sim.setObjectInt32Param(self.barrel, sim.shapeintparam_static, 0)
        self.reset_ik()
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

    def retract(self, radius):
        """Pull the held barrel in towards the arm base (same height) so it is carried at a reachable radius."""
        base = self.pos(self.ur5)[:2]
        c = self.pos(self.barrel)
        v = vsub(c[:2], base)
        r = vnorm(v)
        if r <= radius + 0.02:
            return
        shift = vscale(vunit(v), radius - r)
        t = self.pose(self.grab_ur5)
        print(f"  pulling the barrel in from {r:.2f} m to {radius:.2f} m from the arm base...")
        self.move_tool([t[0] + shift[0], t[1] + shift[1], t[2]] + t[3:])
        print(f"  barrel now {vnorm(vsub(self.pos(self.barrel)[:2], base)):.2f} m from the arm base")

    def rotate_sideways(self):
        """Stage 2: rotate the held barrel about the grab point until its axis is horizontal."""
        axis = self.barrel_axis_world()
        radial = vsub(self.pos(self.barrel)[:2], self.pos(self.ur5)[:2])
        before = axis_elevation_deg(axis)
        q_rot = sideways_rotation(axis, radial)
        p = self.pose(self.grab_ur5)
        q_new = quat_mul(q_rot, p[3:])
        print(f"  barrel axis is {before:.0f} deg off horizontal, rotating...")
        if ROTATE_LIFT > 0:
            print(f"  lifting the barrel {ROTATE_LIFT * 100:.0f} cm first (--rotate-lift)")
            self.move_tool([p[0], p[1], p[2] + ROTATE_LIFT] + list(p[3:]), steps=ROTATE_STEPS // 2)
            p = self.pose(self.grab_ur5)
        self.move_tool(p[:3] + q_new, steps=ROTATE_STEPS)
        after = axis_elevation_deg(self.barrel_axis_world())
        for _ in range(3):                                                    # the arm lags: finish the turn if it fell short
            if after < SIDEWAYS_TOL_DEG:
                break
            radial = vsub(self.pos(self.barrel)[:2], self.pos(self.ur5)[:2])
            q_rot = sideways_rotation(self.barrel_axis_world(), radial)
            p = self.pose(self.grab_ur5)
            self.move_tool(p[:3] + quat_mul(q_rot, p[3:]), steps=ROTATE_STEPS // 2)
            after = axis_elevation_deg(self.barrel_axis_world())
        print(f"  barrel axis now {after:.1f} deg from horizontal")
        return after < SIDEWAYS_TOL_DEG

    def lower_and_release(self, target_xy, surface_z):
        """Stage 4 end: put the sideways barrel over target_xy, lower it onto the surface at surface_z, let go."""
        center = self.pos(self.barrel)
        tool = self.pose(self.grab_ur5)
        dx = target_xy[0] - center[0]
        dy = target_xy[1] - center[1]
        self.move_tool([tool[0] + dx, tool[1] + dy, tool[2]] + tool[3:])                   # slide over the spot
        for _ in range(3):                                                                 # re-aim if the arm fell short
            c = self.pos(self.barrel)
            ex, ey = target_xy[0] - c[0], target_xy[1] - c[1]
            if math.hypot(ex, ey) < 0.01:
                break
            t = self.pose(self.grab_ur5)
            self.move_tool([t[0] + ex, t[1] + ey, t[2]] + t[3:])
        dz = (surface_z + FLOOR_CLEARANCE) - self.world_min_z(self.barrel)
        tool = self.pose(self.grab_ur5)
        self.move_tool([tool[0], tool[1], tool[2] + dz] + tool[3:])                        # lower
        for _ in range(4):                                                                 # keep going until it is really down
            gap = self.world_min_z(self.barrel) - (surface_z + FLOOR_CLEARANCE)
            if gap < 0.008:                                                                # never push down into the block
                break
            t = self.pose(self.grab_ur5)
            self.move_tool([t[0], t[1], t[2] - gap] + t[3:])
        c = self.pos(self.barrel)
        print(f"  before release: barrel {math.hypot(target_xy[0]-c[0], target_xy[1]-c[1])*1000:.0f} mm from the spot, "
              f"bottom {abs(self.world_min_z(self.barrel)-surface_z)*1000:.0f} mm from the surface")
        self.release_barrel()
        final = self.pos(self.barrel)
        return final, axis_elevation_deg(self.barrel_axis_world()), self.world_min_z(self.barrel)

    # ---------- bump checker ----------
    def _world_aabb(self, obj, bb):
        """Axis-aligned world box (lo, hi) around an object's local bounding box bb."""
        pose = self.sim.getObjectPose(obj, -1)
        lo, hi = [1e9] * 3, [-1e9] * 3
        for x in (bb[0], bb[3]):
            for y in (bb[1], bb[4]):
                for z in (bb[2], bb[5]):
                    w = vadd(pose[:3], quat_rotate(pose[3:], [x, y, z]))
                    lo = [min(lo[i], w[i]) for i in range(3)]
                    hi = [max(hi[i], w[i]) for i in range(3)]
        return lo, hi

    def _build_bump_monitor(self):
        """Work out which shapes are the platform body and which are obstacles (everything else except the floor)."""
        sim = self.sim
        shape = sim.object_shape_type
        ur5_tree = set(sim.getObjectsInTree(self.ur5, shape, 0))
        plat_tree = set(sim.getObjectsInTree(self.platform, shape, 0))
        for root, tree in ((self.ur5, ur5_tree), (self.platform, plat_tree)):
            if sim.getObjectType(root) == shape:
                tree.add(root)
        body = [h for h in plat_tree if h not in ur5_tree and h != self.barrel]

        def volume(h):
            bb = self._local_bbox(h)
            return (bb[3] - bb[0]) * (bb[4] - bb[1]) * (bb[5] - bb[2])
        body.sort(key=volume, reverse=True)
        self.bump_body = [(h, self._local_bbox(h)) for h in body[:4]]      # the chassis is the big one

        ignore = {norm_name(n) for n in BUMP_IGNORE}
        # The human walks, so a box frozen at his start position is wrong everywhere else. He is handled by the
        # proximity sensor instead (it follows him every step), not by this static list.
        human_tree = set()
        if self.human is not None:
            human_tree = set(sim.getObjectsInTree(self.human, shape, 0))
            if sim.getObjectType(self.human) == shape:
                human_tree.add(self.human)
        self.bump_obstacles = []
        self.bump_handles = []                                            # same order as bump_obstacles
        for h in self._all_objects():
            if sim.getObjectType(h) != shape or h in plat_tree or h in ur5_tree or h == self.barrel or h in human_tree:
                continue
            alias = sim.getObjectAlias(h, 0)
            if norm_name(alias) in ignore:
                continue
            node, under_floor = h, False                                  # the Floor model and its child 'box'
            while node != -1:
                if norm_name(sim.getObjectAlias(node, 0)) == "floor":
                    under_floor = True
                    break
                node = sim.getObjectParent(node)
            if under_floor:
                continue
            lo, hi = self._world_aabb(h, self._local_bbox(h))
            if hi[2] < FLOOR_TOP_Z:                                       # that's the floor
                area = (hi[0] - lo[0]) * (hi[1] - lo[1])
                if self.floor_box is None or area > self.floor_box[2]:
                    self.floor_box = (lo, hi, area)
                continue
            if hi[0] - lo[0] > ROOM_SIZE_M and hi[1] - lo[1] > ROOM_SIZE_M:
                print(f"  note: ignoring room-sized shape '{alias}' as an obstacle")
                continue
            self.bump_obstacles.append((alias, lo, hi))
            self.bump_handles.append(h)
        self.bump_baseline = {}

    def body_box(self):
        """World box (lo, hi) around the platform body (chassis), at its current pose."""
        lo, hi = [1e9] * 3, [-1e9] * 3
        for h, bb in self.bump_body:
            l, u = self._world_aabb(h, bb)
            lo = [min(lo[i], l[i]) for i in range(3)]
            hi = [max(hi[i], u[i]) for i in range(3)]
        return lo, hi

    @staticmethod
    def _gap_box(lo, hi, olo, ohi):
        """Horizontal gap between the platform box and an obstacle box (inf if the obstacle is not at body height)."""
        if ohi[2] < lo[2] + 0.03 or olo[2] > hi[2]:
            return float("inf")
        dx = max(0.0, olo[0] - hi[0], lo[0] - ohi[0])
        dy = max(0.0, olo[1] - hi[1], lo[1] - ohi[1])
        return math.hypot(dx, dy)

    def _min_gap(self, lo, hi):
        gaps = [self._gap_box(lo, hi, olo, ohi) for _, olo, ohi in self.bump_obstacles]
        return min(gaps) if gaps else float("inf")

    def check_bump(self):
        """
        Returns the name of an obstacle the platform body is touching (and still closing in on), else None.
        Something already touching at the start of a drive is only reported if we push further into it.
        """
        if not BUMP_STOP or not self.bump_body or not self.bump_obstacles:
            return None
        lo, hi = self.body_box()
        hit = None
        for name, olo, ohi in self.bump_obstacles:
            gap = self._gap_box(lo, hi, olo, ohi)
            base = max(self.bump_baseline.get(name, 0.0), min(gap, 10 * BUMP_MARGIN))
            self.bump_baseline[name] = base
            if gap < BUMP_MARGIN and gap < base - 0.003:
                hit = name
        return hit

    def nearest_obstacle(self):
        """(name, gap in m) of the obstacle closest to the platform body right now."""
        lo, hi = self.body_box()
        best = (None, float("inf"))
        for name, olo, ohi in self.bump_obstacles:
            gap = self._gap_box(lo, hi, olo, ohi)
            if gap < best[1]:
                best = (name, gap)
        return best

    def park_spot(self, target_xy, from_xy, min_d):
        """
        Where to park the UR5 base so that it is at least min_d from target_xy, on the side we come from, and
        the platform body stays PARK_CLEARANCE away from every obstacle (so it never touches the blocks).
        Returns (xy, distance_from_target).
        """
        lo, hi = self.body_box()
        base = self.pos(self.ur5)
        off_lo = [lo[0] - base[0], lo[1] - base[1]]
        off_hi = [hi[0] - base[0], hi[1] - base[1]]
        u = vunit(vsub(from_xy, target_xy))
        if vnorm(u) < 0.5:
            u = [-1.0, 0.0]
        d = min_d
        while d <= PARK_MAX_DIST:
            c = vadd(target_xy, vscale(u, d))
            clo = [c[0] + off_lo[0], c[1] + off_lo[1], lo[2]]
            chi = [c[0] + off_hi[0], c[1] + off_hi[1], hi[2]]
            if self._min_gap(clo, chi) >= PARK_CLEARANCE:
                return c, d
            d += 0.01
        print(f"  warning: no obstacle-free parking spot found within {PARK_MAX_DIST} m of "
              f"({target_xy[0]:.2f}, {target_xy[1]:.2f}); parking {min_d:.2f} m away anyway.")
        return vadd(target_xy, vscale(u, min_d)), min_d

    def find_place_box(self, name=None):
        """
        The box the barrel should end up on: the obstacle whose name contains `name`, or (default) the whitest
        solid shape in the scene. Returns (alias, lo, hi) or None.
        """
        sim = self.sim
        if name:
            key = norm_name(name)
            hits = [o for o in self.bump_obstacles if key in norm_name(o[0])]
            return max(hits, key=lambda o: (o[2][0] - o[1][0]) * (o[2][1] - o[1][1])) if hits else None
        best, best_w = None, 0.0
        for obs, h in zip(self.bump_obstacles, self.bump_handles):
            try:
                rgb = sim.getShapeColor(h, None, getattr(sim, "colorcomponent_ambient_diffuse", 0))
                rgb = rgb[0] if isinstance(rgb[0], (list, tuple)) else rgb
                w = min(rgb[:3])
            except Exception:
                continue
            if w > best_w:
                best, best_w = obs, w
        return best if best_w >= 0.8 else None

    def surface_z_at(self, xy, fallback_z):
        """Height of the top of whatever solid object is under xy (the white block), else fallback_z."""
        best, src = None, None
        for name, olo, ohi in self.bump_obstacles:
            if olo[0] - 0.02 <= xy[0] <= ohi[0] + 0.02 and olo[1] - 0.02 <= xy[1] <= ohi[1] + 0.02:
                if best is None or ohi[2] > best:
                    best, src = ohi[2], name
        return (best, src) if best is not None else (fallback_z, None)

    def raise_clear(self, surface_z):
        """Lift the held barrel so its bottom clears surface_z before it is swung over that surface."""
        need = (surface_z + RAISE_CLEAR) - self.world_min_z(self.barrel)
        if need > 0.005:
            p = self.pose(self.grab_ur5)
            print(f"  raising the barrel {need * 100:.0f} cm to clear the surface it will be swung over...")
            self.move_tool([p[0], p[1], p[2] + need] + p[3:])

    def swing_to(self, target_xy, steps=ROTATE_STEPS, settle=SETTLE_MAX_STEPS):
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
        err = self._hold_until_reached(last, max_steps=settle)
        base = self.pos(self.ur5)[:2]
        now = vsub(self.pos(self.barrel)[:2], base)
        want = vsub(target_xy, base)
        off = math.degrees(abs(wrap_angle(math.atan2(want[1], want[0]) - math.atan2(now[1], now[0]))))
        print(f"  barrel now faces the drop spot to within {off:.0f} deg")
        if misses > steps * 0.3 or err > 0.03:
            print(f"  warning: IK trouble during the swing (unsolved {misses}/{steps}, final tool error "
                  f"{err * 1000:.0f} mm). Try a different --standoff.")

    # ---------- omni driving ----------
    def _set_wheels(self, cmd):
        for w, c in zip(self.wheels, cmd):
            self.sim.setJointTargetVelocity(w, c)

    def _burst_once(self, i, sign):
        """Spin ONLY wheel i (forward if sign=+1, backward if -1) and measure the response per rad/s, in the
        platform's own frame: [forward-ish m/s, sideways-ish m/s, yaw rate rad/s]."""
        sim = self.sim
        dt = sim.getSimulationTimeStep()
        speed = sign * WHEEL_V * CAL_SCALE
        cmd = [0.0] * 4
        cmd[i] = speed
        self._set_wheels(cmd)
        half = CAL_STEPS // 2
        self._cal_step(half)                                  # let the wheel spin up, discard this part
        if i == 0 and sign > 0:
            try:                                              # is something else fighting our wheel command?
                got = sim.getJointTargetVelocity(self.wheels[0])
                if abs(got - cmd[0]) > 1e-3:
                    print(f"  WARNING: wheel 0 target velocity reads {got:.3f} after we set {cmd[0]:.3f}. "
                          f"Something else (the OmniPlatform's own child script?) is overwriting the wheels. "
                          f"Disable that script in CoppeliaSim.")
            except Exception:
                pass
        p0, y0 = self.pos(self.ur5)[:2], self.platform_yaw()
        self._cal_step(CAL_STEPS - half)
        p1, y1 = self.pos(self.ur5)[:2], self.platform_yaw()
        self._set_wheels([0.0] * 4)
        self.step(CAL_SETTLE_STEPS)
        t = (CAL_STEPS - half) * dt
        v_world = vscale(vsub(p1, p0), 1.0 / t)
        v_body = rot2(v_world, -(y0 + wrap_angle(y1 - y0) / 2.0))        # world -> platform frame
        w = wrap_angle(y1 - y0) / t
        return [v_body[0] / speed, v_body[1] / speed, w / speed]

    def _cal_step(self, n):
        """Step n times; note if the walking human came near the platform (a person brushing past ruins a measurement)."""
        for _ in range(n):
            self.step()
            if self.human_points and self.human_distance() < HUMAN_STOP_RADIUS + HUMAN_CLEAR_MARGIN:
                self._cal_disturbed = True

    def _wheel_burst(self, i):
        """
        Forward burst then backward burst: the platform ends up about where it began, and the two are averaged.
        If the human walked close during either burst the measurement is thrown away: wait until he is clear, redo it.
        """
        for attempt in range(1, 13):
            self._cal_disturbed = False
            a = self._burst_once(i, +1)
            b = self._burst_once(i, -1)
            if not self._cal_disturbed:
                break
            print(f"  wheel {i}: the human walked close during the measurement; discarding it and waiting for him to clear (try {attempt})")
            self._set_wheels([0.0] * 4)
            for _ in range(HUMAN_MAX_WAIT_STEPS):
                if self.human_distance() > HUMAN_STOP_RADIUS + 2 * HUMAN_CLEAR_MARGIN:
                    break
                self.step()
        else:
            print(f"  warning: wheel {i} could not be measured without the human nearby; using the last reading.")
        return [(a[k] + b[k]) / 2.0 for k in range(3)]

    def _find_clear_spot(self):
        """A place for the arm base where the platform body is CAL_CLEAR_M away from every obstacle, on the floor."""
        lo, hi = self.body_box()
        base = self.pos(self.ur5)
        off_lo = [lo[0] - base[0], lo[1] - base[1]]
        off_hi = [hi[0] - base[0], hi[1] - base[1]]
        for r in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5):
            for k in range(1 if r == 0.0 else 16):
                ang = 2.0 * math.pi * k / 16.0
                c = [base[0] + r * math.cos(ang), base[1] + r * math.sin(ang)]
                clo = [c[0] + off_lo[0], c[1] + off_lo[1], lo[2]]
                chi = [c[0] + off_hi[0], c[1] + off_hi[1], hi[2]]
                if self._min_gap(clo, chi) < CAL_CLEAR_M:
                    continue
                if self.floor_box is not None:
                    flo, fhi, _ = self.floor_box
                    if not (flo[0] + 0.3 < clo[0] and chi[0] < fhi[0] - 0.3 and flo[1] + 0.3 < clo[1] and chi[1] < fhi[1] - 0.3):
                        continue
                return c
        return None

    def calibrate_once(self):
        """
        Measure how each wheel moves the platform (3 x 4 matrix: forward, sideways, yaw per wheel). This is a
        property of the platform, not of where it stands, so it is done ONCE, in a clear part of the room, and the
        platform is put back at its real start afterwards. Each wheel gets a short forward burst and a short
        backward burst, so the platform barely wanders.
        """
        if self.A is not None:
            return
        sim = self.sim
        print("== Calibrating the omni wheels (once) ==")
        sim.setObjectPose(self.platform, -1, self.platform_start_pose)
        for j, q in zip(self.joints, self.joint_start):
            sim.setJointPosition(j, q)
        spot = None if CAL_IN_PLACE else self._find_clear_spot()
        if CAL_IN_PLACE:
            print("  calibrating in place with small bursts (CAL_IN_PLACE)")
        elif spot is not None:
            shift = vsub(spot, self.pos(self.ur5)[:2])
            p = sim.getObjectPosition(self.platform, -1)
            sim.setObjectPosition(self.platform, -1, [p[0] + shift[0], p[1] + shift[1], p[2]])
            print(f"  calibrating at a clear spot ({spot[0]:.2f}, {spot[1]:.2f}); the platform goes back to its start afterwards")
        else:
            print("  no clear spot found (nothing {:.1f} m away from every obstacle); calibrating in place with small bursts".format(CAL_CLEAR_M))
        sim.setStepping(True)
        sim.startSimulation()
        self.claim_bill()
        try:
            for j in self.joints:
                try:
                    sim.setIntProperty(j, "dynCtrlMode", sim.jointdynctrl_position)
                except Exception:
                    pass
            for j, q in zip(self.joints, self.joint_start):
                sim.setJointTargetPosition(j, q)
            self.step(40)
            cols = [self._wheel_burst(i) for i in range(4)]
        finally:
            self._set_wheels([0.0] * 4)
            sim.stopSimulation(True)
            sim.setObjectPose(self.platform, -1, self.platform_start_pose)
            for j, q in zip(self.joints, self.joint_start):
                sim.setJointPosition(j, q)

        A = [[cols[k][r] for k in range(4)] for r in range(3)]            # 3 x 4: body (vx, vy, yaw) per wheel
        names = ["back right", "front right", "front left", "back left"]
        print("  response per wheel (rad/s):   body-x m/s | body-y m/s | yaw rad/s")
        for k in range(4):
            print(f"    wheel {k} ({names[k]:11s}): {cols[k][0]:+.4f} | {cols[k][1]:+.4f} | {cols[k][2]:+.4f}")
        if max(abs(x) for c in cols for x in c) < 1e-4:
            raise RuntimeError("No wheel moved the platform. Check the wheel paths and that the wheel joints are "
                               "velocity-controlled and the platform is not fixed/static.")
        if solve_min_norm(A, [1.0, 0.0, 0.0]) is None:
            raise RuntimeError("The four wheels cannot produce all of forward, sideways and turning motion "
                               "(calibration matrix is singular). Paste the response table above to me.")
        self.A = A

    def scene_summary(self):
        """What the bump checker and the parking logic believe about the scene (for debugging)."""
        lo, hi = self.body_box()
        print("Scene as the script sees it:")
        print(f"  platform body box  x {lo[0]:+.2f}..{hi[0]:+.2f}  y {lo[1]:+.2f}..{hi[1]:+.2f}  z {lo[2]:+.2f}..{hi[2]:+.2f}")
        for name, olo, ohi in self.bump_obstacles:
            gap = self._gap_box(lo, hi, olo, ohi)
            print(f"  obstacle {name:20s} x {olo[0]:+.2f}..{ohi[0]:+.2f}  y {olo[1]:+.2f}..{ohi[1]:+.2f}  "
                  f"z {olo[2]:+.2f}..{ohi[2]:+.2f}   gap to platform {gap * 1000:.0f} mm")
        if not self.bump_obstacles:
            print("  (no obstacles found: the bump checker and clear-parking have nothing to avoid)")

    def drive_to(self, goal_xy, tol=DRIVE_TOL):
        """
        Drive the UR5 base to goal_xy (world xy) with the calibrated wheel model, holding the start heading.
        Stops early if the platform runs into something (no progress although we are still commanding motion).
        Sets self.blocked to say whether that happened.
        """
        dt = self.sim.getSimulationTimeStep()
        self.blocked = False
        self.bump_baseline = {}
        anchor, commanded, anchor_n = self.pos(self.ur5)[:2], 0.0, 0   # progress bookkeeping for contact detection
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
            if self.guard_intervened:                                             # we sat still for the human: that is not a stall
                self.guard_intervened = False
                anchor, commanded, anchor_n = self.pos(self.ur5)[:2], 0.0, n + 1
            commanded += speed * dt
            if BUMP_STOP and n % BUMP_CHECK_EVERY == 0:
                hit = self.check_bump()
                if hit:
                    self.blocked = True
                    self._set_wheels([0.0] * 4)
                    print(f"  bumped into '{hit}' ({self.pos(self.ur5)[0]:.2f}, {self.pos(self.ur5)[1]:.2f}): "
                          f"wheels stopped.")
                    break
            if n % 100 == 0:
                near, gap = self.nearest_obstacle()
                near_txt = f", nearest solid '{near}' {gap * 1000:.0f} mm" if near is not None and gap < 5.0 else ""
                print(f"    t={n:4d}  {dist * 1000:5.0f} mm to go, heading off by {math.degrees(yaw_err):+5.1f} deg{near_txt}")

            if STOP_ON_CONTACT and n + 1 - anchor_n >= STALL_WINDOW:              # a full window since the last reset
                moved = vsub(self.pos(self.ur5)[:2], anchor)
                toward = vdot(moved, vunit(err))                                  # progress toward the goal
                if dist > 2 * tol and toward < max(STALL_MIN_M, STALL_MIN_FRAC * commanded):
                    self.blocked = True
                    print(f"  platform is not making progress ({dist * 1000:.0f} mm from the goal): it has run into "
                          f"something, stopping here.")
                    print(f"    diagnostics: in the last {n + 1 - anchor_n} steps it moved ({moved[0] * 1000:+.0f}, {moved[1] * 1000:+.0f}) mm "
                          f"toward a goal at direction ({err[0]:+.2f}, {err[1]:+.2f}); nearest solid {self.nearest_obstacle()}; "
                          f"human {self.human_distance():.2f} m from the arm")
                    break
                anchor, commanded, anchor_n = self.pos(self.ur5)[:2], 0.0, n + 1
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
    robot.calibrate_once()
    robot.begin_run(near_barrel=False)
    try:

        barrel_xy = robot.pos(robot.barrel)[:2]
        start_xy  = robot.pos(robot.ur5)[:2]
        goal, d_grab = robot.park_spot(barrel_xy, start_xy, ARM_STANDOFF)
        if d_grab > PLACE_REACH_MAX + 0.05:
            print(f"  warning: to stay clear of obstacles the platform must park {d_grab:.2f} m from the barrel; "
                  f"the arm may not reach it.")
        print(f"  driving from ({start_xy[0]:.2f}, {start_xy[1]:.2f}) to ({goal[0]:.2f}, {goal[1]:.2f}) near the barrel...")
        miss = robot.drive_to(goal)
        print(f"  parked {miss * 1000:.0f} mm from the goal")

        ok3 = robot.grab()
        print(f"  Stage 3 (drive + grab): {'PASS' if ok3 else 'FAIL'}")
        robot.lift()
        if CARRY_RADIUS:
            robot.retract(CARRY_RADIUS)

        # Where the barrel goes: park on the side we arrive from, clear of every solid object (the platform never
        # touches the blocks), as close to PointB as that allows, and set the barrel down ON the surface at PointB.
        b_xy = robot.pos(robot.point_b)[:2]
        b_z = robot.pos(robot.point_b)[2]
        if PLACE_ON != "pointb":
            box = robot.find_place_box(None if PLACE_ON == "white" else PLACE_ON)
            if box is not None:
                name, lo, hi = box
                b_xy = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0]
                b_z = hi[2]
                print(f"  placing ON '{name}' (centre {b_xy[0]:.2f}, {b_xy[1]:.2f}, top z {hi[2]:.3f} m)")
            else:
                print("  warning: could not find the white box (or the --place-on name); using PointB instead.")
        cur_xy = robot.pos(robot.ur5)[:2]
        ur5_goal, _ = robot.park_spot(b_xy, cur_xy, MIN_PARK_DIST)
        place_xy = b_xy
        surface_z, src = robot.surface_z_at(b_xy, b_z)
        print(f"  PointB is on {'the top of ' + repr(src) if src else 'the floor'} (surface z = {surface_z:.3f} m)")

        # Turn around NOW, while standing still: lift clear of the white block's height, then swing the arm round so
        # the barrel points at PointB. The drive then happens with the barrel already facing the white block.
        robot.raise_clear(max(surface_z, robot.body_box()[1][2]))            # also clear the platform's own top
        print("  turning the barrel around to face PointB before driving...")
        robot.swing_to(b_xy)

        print(f"  driving to PointB ({b_xy[0]:.2f}, {b_xy[1]:.2f}) with the barrel...")
        miss = robot.drive_to(ur5_goal)
        print(f"  parked {miss * 1000:.0f} mm from the goal")

        base_xy = robot.pos(robot.ur5)[:2]
        to_b = vsub(b_xy, base_xy)
        if vnorm(to_b) > PLACE_REACH_MAX:                                        # keep the ask inside the arm's reach
            place_xy = vadd(base_xy, vscale(vunit(to_b), PLACE_REACH_MAX))
            print(f"  PointB is {vnorm(to_b):.2f} m from the arm base; placing at {PLACE_REACH_MAX:.2f} m instead "
                  f"(still on the block as long as it is big enough).")
        surface_z, src = robot.surface_z_at(place_xy, surface_z)
        robot.raise_clear(surface_z)
        robot.swing_to(place_xy)                                                 # fine alignment, usually ~0 deg

        ok_side = robot.rotate_sideways()
        final, elev, bottom = robot.lower_and_release(place_xy, surface_z)
        off = vnorm(vsub(final[:2], place_xy))
        on_surface = abs(bottom - surface_z) < 0.05
        ok4 = ok_side and off < PLACE_TOL_M and elev < SIDEWAYS_TOL_DEG * 3 and on_surface
        print(f"  barrel ended {off * 1000:.0f} mm from the target spot, axis {elev:.1f} deg from horizontal, "
              f"bottom {abs(bottom - surface_z) * 1000:.0f} mm from the surface")
        print(f"  Stage 4 (drive + place sideways): {'PASS' if ok4 else 'FAIL'}")
        return ok3 and ok4
    finally:
        robot.end_run()



#### MAIN
def attempt(stage_fn, robot):
    """Run one stage; a human who never leaves counts as a FAIL instead of crashing the whole script."""
    try:
        return stage_fn(robot)
    except HumanTooClose as exc:
        print(f"  ABORTED: {exc}")
        return False


def main():
    global ARM_STANDOFF, HUMAN_STOP_RADIUS, PLACE_ON, ROTATE_LIFT
    ap = argparse.ArgumentParser(description="Barrel pick-and-place with OmniPlatform + UR5")
    ap.add_argument("--mode", choices=["ur5", "full", "all"], default="all",
                    help="ur5 = stages 1-2, full = stages 3-4, all = both (default)")
    ap.add_argument("--repeat", type=int, default=1, help="how many times to repeat the chosen mode")
    ap.add_argument("--discover", action="store_true", help="list dummies/shapes and what was resolved, then exit")
    ap.add_argument("--standoff", type=float, default=None, help="UR5-base-to-barrel distance when parked (m)")
    ap.add_argument("--human", default=None, help="alias (or part of it) of the walking human; default: look for bill/human/person")
    ap.add_argument("--human-radius", type=float, default=None,
                    help=f"retract the arm when the human is closer than this to any part of it (m, default {HUMAN_STOP_RADIUS})")
    ap.add_argument("--place-on", default=None,
                    help="where the barrel goes: 'pointb' (default: on PointB, the white box), 'white' (whitest shape), or part of a shape's name")
    ap.add_argument("--builtin-ik", action="store_true", help="use the built-in IK solver even if CoppeliaSim's simIK loads")
    ap.add_argument("--no-human-sensor", action="store_true", help="turn the human-proximity safety off")
    ap.add_argument("--sim-human", action="store_true",
                    help="walk the scene's Bill (his own script switched off) back and forth at a speed you choose")
    ap.add_argument("--rotate-lift", type=float, default=None,
                    help="raise the held barrel this many metres before turning it sideways (try 0.10-0.25 if the turn fails)")
    ap.add_argument("--cylinder-human", action="store_true",
                    help="with --sim-human: walk an orange cylinder instead of the scene's Bill model")
    ap.add_argument("--human-speed", type=float, default=0.5, help="stand-in human walking speed in m/s (default 0.5; 0 = standing still)")
    ap.add_argument("--human-pass-y", type=float, default=-0.7,
                    help="the line he walks along (world y, m). The platform sits at y -0.32..+0.32, so -0.7 passes close enough to trigger the stop")
    ap.add_argument("--human-x-range", type=float, nargs=2, default=[-3.0, 3.0], metavar=("X_MIN", "X_MAX"),
                    help="ends of his walk along x (m)")
    args = ap.parse_args()

    if args.rotate_lift is not None:
        ROTATE_LIFT = args.rotate_lift
    if args.standoff is not None:
        ARM_STANDOFF = args.standoff
    if args.place_on:
        PLACE_ON = args.place_on.lower() if args.place_on.lower() in ("white", "pointb") else args.place_on
    if args.human_radius is not None:
        HUMAN_STOP_RADIUS = args.human_radius

    synthetic = None
    if args.sim_human:
        synthetic = dict(speed=args.human_speed, pass_y=args.human_pass_y,
                         x_min=min(args.human_x_range), x_max=max(args.human_x_range),
                         cylinder=args.cylinder_human)
    robot = Robot(human_name=args.human, use_human_sensor=not args.no_human_sensor,
                  force_builtin_ik=args.builtin_ik, synthetic_human=synthetic)
    import atexit
    atexit.register(robot.remove_synthetic)                                    # take the stand-in human out of the scene again
    robot.scene_summary()
    if args.discover:
        robot.discover()
        return 0

    results = []
    for n in range(1, args.repeat + 1):
        print(f"\n######## Run {n} of {args.repeat} ########")
        if args.mode in ("ur5", "all"):
            results.append(("ur5", n, attempt(run_ur5_only, robot)))
        if args.mode in ("full", "all"):
            results.append(("full", n, attempt(run_full, robot)))

    print("\n######## Summary ########")
    for mode, n, ok in results:
        print(f"  run {n} {mode:5s}: {'PASS' if ok else 'FAIL'}")
    if robot.human is not None:
        print(f"  human-proximity stops: {robot.human_trips}")
    return 0 if all(ok for _, _, ok in results) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        chain, e = [], exc
        while e is not None:
            chain.append(type(e).__name__)
            e = e.__cause__ or e.__context__
        if "Again" in chain:                              # zmq.Again = our receive timeout
            print(f"\nCoppeliaSim did not answer for {REPLY_TIMEOUT_S} s, so the script stopped instead of hanging.\n"
                  f"Look at the CoppeliaSim window: is a dialog box or error message open, is the simulation paused,\n"
                  f"or does the console at the bottom show a Lua error (for example from Bill's script)?")
            sys.exit(2)
        raise
