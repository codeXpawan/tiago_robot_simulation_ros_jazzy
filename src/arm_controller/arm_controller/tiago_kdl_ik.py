"""
tiago_kdl_ik.py
---------------
KDL-based IK solver for the TIAGo arm.

Builds the kinematic chain directly from the robot URDF (via /robot_description),
runs KDL's TRAC-IK / ChainIkSolverPos_NR_JL, and returns joint angles.

Dependencies (all ship with ROS 2 Jazzy):
    sudo apt install ros-jazzy-kdl-parser-py python3-pykdl

Usage:
    solver = TiagoIKSolver(urdf_string, config)
    result = solver.solve(target_pose_stamped, seed_state=None)
"""

import math
import time
import random
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

# PyKDL - ships with ROS 2 (ros-jazzy-python-orocos-kdl-vendor / python3-pykdl)
# Note: ros-jazzy-kdl-parser-py does not exist as a binary; treeFromUrdfModel
# is implemented inline below using urdf_parser_py + PyKDL directly.
try:
    import PyKDL as kdl
    from urdf_parser_py.urdf import URDF
    KDL_AVAILABLE = True
except ImportError:
    KDL_AVAILABLE = False


# ─────────────────────────────────────────────
# Inline URDF → KDL tree builder
# (replaces the missing ros-jazzy-kdl-parser-py)
# ─────────────────────────────────────────────

def _pose_to_kdl_frame(origin):
    """Convert urdf_parser_py Pose to KDL Frame."""
    xyz = origin.xyz if (origin is not None and origin.xyz is not None) else [0.0, 0.0, 0.0]
    rpy = origin.rpy if (origin is not None and origin.rpy is not None) else [0.0, 0.0, 0.0]
    return kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))


def _joint_to_kdl_joint(joint):
    """Convert urdf_parser_py Joint to KDL Joint."""
    origin_frame = _pose_to_kdl_frame(joint.origin)
    if joint.joint_type == 'fixed':
        return kdl.Joint(joint.name, kdl.Joint.Fixed)
    axis_xyz = joint.axis if joint.axis is not None else [1.0, 0.0, 0.0]
    axis = origin_frame.M * kdl.Vector(*axis_xyz)
    if joint.joint_type in ('revolute', 'continuous'):
        return kdl.Joint(joint.name, origin_frame.p, axis, kdl.Joint.RotAxis)
    elif joint.joint_type == 'prismatic':
        return kdl.Joint(joint.name, origin_frame.p, axis, kdl.Joint.TransAxis)
    return kdl.Joint(joint.name, kdl.Joint.Fixed)


def treeFromUrdfModel(robot):
    """
    Build a KDL tree from a urdf_parser_py URDF robot model.
    Returns (success: bool, tree: kdl.Tree).
    """
    try:
        root_name = robot.get_root()
        tree = kdl.Tree(root_name)

        def _add_children(parent_name):
            for joint_name, child_name in robot.child_map.get(parent_name, []):
                joint = robot.joint_map[joint_name]
                child_link = robot.link_map[child_name]
                f = _pose_to_kdl_frame(joint.origin)
                kdl_jnt = _joint_to_kdl_joint(joint)
                if child_link.inertial is not None:
                    inert = child_link.inertial
                    cog_xyz = inert.origin.xyz if inert.origin else [0.0, 0.0, 0.0]
                    i = inert.inertia
                    kdl_inert = kdl.RigidBodyInertia(
                        inert.mass,
                        kdl.Vector(*cog_xyz),
                        kdl.RotationalInertia(i.ixx, i.iyy, i.izz, i.ixy, i.ixz, i.iyz),
                    )
                else:
                    kdl_inert = kdl.RigidBodyInertia()
                seg = kdl.Segment(child_name, kdl_jnt, f, kdl_inert)
                if not tree.addSegment(seg, parent_name):
                    return False
                if not _add_children(child_name):
                    return False
            return True

        return (_add_children(root_name), tree)
    except Exception as e:
        return False, None


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from builtin_interfaces.msg import Duration


# ─────────────────────────────────────────────
# Data classes
# ─────────────────────────────────────────────

@dataclass
class IKResult:
    success: bool
    joint_angles: List[float] = field(default_factory=list)
    joint_names: List[str] = field(default_factory=list)
    message: str = ""
    solve_time_ms: float = 0.0
    is_reachable: bool = False
    position_error: float = float("inf")   # metres; used internally by multi-seed
    _best_seed: List[float] = field(default_factory=list, repr=False)


@dataclass
class ArmConfig:
    base_link: str = "torso_lift_link"
    tip_link: str = "arm_tool_link"
    joint_names: List[str] = field(default_factory=lambda: [
        "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint",
        "arm_5_joint", "arm_6_joint", "arm_7_joint"
    ])
    joint_lower: List[float] = field(default_factory=lambda: [
        -1.178, -1.518, -3.534, -0.393, -2.094, -1.396, -2.094
    ])
    joint_upper: List[float] = field(default_factory=lambda: [
        1.571, 0.524, 1.571, 2.618, 2.094, 1.396, 2.094
    ])
    max_iterations: int = 1000
    epsilon: float = 1e-5
    timeout_sec: float = 2.0
    # Workspace bounding box in torso frame
    workspace_x: Tuple[float, float] = (-0.1, 0.85)
    workspace_y: Tuple[float, float] = (-0.7, 0.7)
    workspace_z: Tuple[float, float] = (-0.5, 0.9)
    max_reach: float = 0.90


# ─────────────────────────────────────────────
# Reachability checker (fast, geometry-based)
# ─────────────────────────────────────────────

class ReachabilityChecker:
    """
    Fast workspace pre-filter before running full IK.
    Uses bounding box + max reach sphere.
    """

    def __init__(self, config: ArmConfig):
        self.config = config

    def check(self, x: float, y: float, z: float) -> Tuple[bool, str]:
        """
        Check if (x, y, z) in torso_lift_link frame is geometrically reachable.
        Returns (is_reachable, reason_string)
        """
        c = self.config

        # 1. Bounding box check
        if not (c.workspace_x[0] <= x <= c.workspace_x[1]):
            return False, f"x={x:.3f} outside workspace [{c.workspace_x[0]}, {c.workspace_x[1]}]"
        if not (c.workspace_y[0] <= y <= c.workspace_y[1]):
            return False, f"y={y:.3f} outside workspace [{c.workspace_y[0]}, {c.workspace_y[1]}]"
        if not (c.workspace_z[0] <= z <= c.workspace_z[1]):
            return False, f"z={z:.3f} outside workspace [{c.workspace_z[0]}, {c.workspace_z[1]}]"

        # 2. Max reach sphere
        dist = math.sqrt(x**2 + y**2 + z**2)
        if dist > c.max_reach:
            return False, f"distance={dist:.3f}m exceeds max reach {c.max_reach}m"

        # 3. Min reach (too close to shoulder = singularity zone)
        if dist < 0.05:
            return False, f"distance={dist:.3f}m too close (singularity zone)"

        return True, f"Point ({x:.3f}, {y:.3f}, {z:.3f}) is within workspace. dist={dist:.3f}m"


# ─────────────────────────────────────────────
# KDL IK Solver
# ─────────────────────────────────────────────

class TiagoIKSolver:
    """
    Builds a KDL kinematic chain from TIAGo's URDF and solves IK.

    Solver chain:
        FK:  ChainFkSolverPos_recursive
        IK:  ChainIkSolverVel_pinv (velocity) +
             ChainIkSolverPos_NR_JL (position, with joint limits)
    """

    # ── Standard end-effector orientations (RPY) tried during multi-seed ──
    _STANDARD_RPYS: List[Tuple[float, float, float]] = [
        (0,             math.pi / 2,   0),           # pointing down
        (0,             0,             0),            # identity / forward
        (0,            -math.pi / 2,   0),            # pointing up
        (math.pi,       0,             0),            # 180° roll
        (0,             math.pi / 4,   0),            # 45° pitch down
        (0,            -math.pi / 4,   0),            # −45° pitch up
        (math.pi / 2,   0,             0),            # 90° roll
        (-math.pi / 2,  0,             0),            # −90° roll
        (0,             math.pi / 2,   math.pi / 2),  # down + 90° yaw
        (0,             math.pi / 2,  -math.pi / 2),  # down − 90° yaw
        (math.pi,       math.pi / 2,   0),            # down + 180° roll
        (0,             math.pi,       0),            # pointing backward
    ]

    def __init__(self, urdf_string: str, config: ArmConfig):
        if not KDL_AVAILABLE:
            raise ImportError(
                "PyKDL not found. Install with:\n"
                "  sudo apt install ros-jazzy-kdl-parser-py python3-pykdl"
            )

        self.config = config
        self.reachability = ReachabilityChecker(config)
        self._chain = None
        self._fk_solver = None
        self._ik_vel_solver = None   # must be kept alive — _ik_solver holds a C++ pointer to it
        self._ik_solver = None
        self._q_min = None           # same: KDL holds raw pointers to these JntArrays
        self._q_max = None
        self._n_joints = len(config.joint_names)

        self._build_chain(urdf_string)

    def _build_chain(self, urdf_string: str):
        """Parse URDF and build KDL chain from base_link to tip_link."""
        robot = URDF.from_xml_string(urdf_string)
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            raise RuntimeError("Failed to parse URDF into KDL tree")

        self._chain = tree.getChain(self.config.base_link, self.config.tip_link)
        if self._chain.getNrOfJoints() == 0:
            raise RuntimeError(
                f"KDL chain from '{self.config.base_link}' to "
                f"'{self.config.tip_link}' has 0 joints. "
                "Check your URDF link names."
            )

        # Build joint limit arrays — stored as instance vars so Python's GC
        # doesn't free them while the KDL solver holds raw C++ pointers to them.
        self._q_min = kdl.JntArray(self._n_joints)
        self._q_max = kdl.JntArray(self._n_joints)
        for i, (lo, hi) in enumerate(zip(self.config.joint_lower, self.config.joint_upper)):
            self._q_min[i] = lo
            self._q_max[i] = hi

        # FK solver
        self._fk_solver = kdl.ChainFkSolverPos_recursive(self._chain)

        # IK velocity solver — also stored as instance var (same GC reason)
        self._ik_vel_solver = kdl.ChainIkSolverVel_pinv(self._chain)

        # IK position solver with joint limits
        self._ik_solver = kdl.ChainIkSolverPos_NR_JL(
            self._chain,
            self._q_min, self._q_max,
            self._fk_solver,
            self._ik_vel_solver,
            self.config.max_iterations,
            self.config.epsilon,
        )

    def pose_to_kdl_frame(self, pose: Pose) -> kdl.Frame:
        """Convert geometry_msgs/Pose to KDL Frame."""
        p = pose.position
        q = pose.orientation
        rotation = kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w)
        vector = kdl.Vector(p.x, p.y, p.z)
        return kdl.Frame(rotation, vector)

    def get_seed_state(self, current_joints: Optional[List[float]] = None) -> kdl.JntArray:
        """
        Seed for IK solver. Uses current joint state if provided,
        otherwise uses a safe neutral pose (arm slightly extended forward).
        """
        q_seed = kdl.JntArray(self._n_joints)
        if current_joints and len(current_joints) == self._n_joints:
            for i, v in enumerate(current_joints):
                q_seed[i] = v
        else:
            # TIAGo safe neutral: arm extended ~forward
            neutral = [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]
            for i, v in enumerate(neutral[:self._n_joints]):
                q_seed[i] = v
        return q_seed

    def solve(
        self,
        target_pose: Pose,
        current_joints: Optional[List[float]] = None,
    ) -> IKResult:
        """
        Solve IK for target_pose (in torso_lift_link frame).

        Args:
            target_pose: geometry_msgs/Pose in torso_lift_link frame
            current_joints: current arm joint state (used as IK seed)

        Returns:
            IKResult with success flag, joint angles, and diagnostics
        """
        t_start = time.perf_counter()

        # ── Step 1: fast reachability pre-check ──
        p = target_pose.position
        reachable, reason = self.reachability.check(p.x, p.y, p.z)

        if not reachable:
            return IKResult(
                success=False,
                is_reachable=False,
                message=f"Pre-check FAILED: {reason}",
                solve_time_ms=(time.perf_counter() - t_start) * 1000,
            )

        # ── Step 2: convert target to KDL frame ──
        kdl_target = self.pose_to_kdl_frame(target_pose)

        # ── Step 3: seed state ──
        q_seed = self.get_seed_state(current_joints)
        q_out = kdl.JntArray(self._n_joints)

        # ── Step 4: run IK ──
        ret = self._ik_solver.CartToJnt(q_seed, kdl_target, q_out)

        elapsed_ms = (time.perf_counter() - t_start) * 1000
        joint_angles = [q_out[i] for i in range(self._n_joints)]

        if ret >= 0:
            # Verify with FK
            fk_frame = kdl.Frame()
            self._fk_solver.JntToCart(q_out, fk_frame)
            pos_err = (fk_frame.p - kdl_target.p).Norm()

            return IKResult(
                success=True,
                is_reachable=True,
                joint_angles=joint_angles,
                joint_names=self.config.joint_names,
                message=f"IK solved. FK position error: {pos_err*1000:.2f}mm. {reason}",
                solve_time_ms=elapsed_ms,
                position_error=pos_err,
            )
        else:
            # KDL SolverI error codes
            err_map = {
                -1: "no convergence",
                -2: "undefined / joint limits exceeded",
                -3: "not up to date",
                -4: "size mismatch",
                -5: "max iterations exceeded",
                -6: "out of range",
                -7: "not implemented",
                -8: "SVD failed",
            }
            err_str = err_map.get(ret, f"KDL error code {ret}")

            # Compute FK position error from seed for best-result tracking
            fk_frame = kdl.Frame()
            self._fk_solver.JntToCart(q_out, fk_frame)
            pos_err = (fk_frame.p - kdl_target.p).Norm()

            return IKResult(
                success=False,
                is_reachable=True,   # geometrically reachable but IK failed
                joint_angles=joint_angles,
                joint_names=self.config.joint_names,
                message=f"IK FAILED ({err_str}). Try a different seed or orientation.",
                solve_time_ms=elapsed_ms,
                position_error=pos_err,
            )

    # ─────────────────────────────────────────────
    # Multi-seed solver helpers
    # ─────────────────────────────────────────────

    def _cone_sample_orientations(
        self,
        base_orientation: Quaternion,
        n: int = 6,
        half_angle: float = 0.3,
    ) -> List[Quaternion]:
        """
        Sample n orientations uniformly distributed around base_orientation
        within a cone of radius half_angle (radians).
        Helps when the exact orientation doesn't matter but nearby variants do.
        """
        result = []
        q = base_orientation
        base_rot = kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w)
        for i in range(n):
            azimuth = 2 * math.pi * i / n
            tilt_axis = kdl.Vector(
                math.cos(azimuth) * math.sin(half_angle),
                math.sin(azimuth) * math.sin(half_angle),
                math.cos(half_angle),
            )
            tilt_axis.Normalize()
            perturb = kdl.Rotation.Rot(tilt_axis, half_angle)
            new_rot = base_rot * perturb
            qx, qy, qz, qw = new_rot.GetQuaternion()
            result.append(Quaternion(x=qx, y=qy, z=qz, w=qw))
        return result

    def _perturb_joints(self, joints: List[float], sigma: float = 0.05) -> List[float]:
        """
        Add Gaussian noise to joint angles, clamped to joint limits.
        Small sigma (~0.05 rad) often escapes narrow local minima near the
        current configuration without straying far from a valid region.
        """
        lo = self.config.joint_lower
        hi = self.config.joint_upper
        return [
            float(np.clip(j + random.gauss(0, sigma), l, h))
            for j, l, h in zip(joints, lo, hi)
        ]

    def _stratified_seeds(self, n_per_joint: int = 3) -> List[List[float]]:
        """
        Build a small set of seeds that evenly cover joint space via a
        rotated Latin hypercube — n_per_joint seeds total (not n^7).

        Each joint range is split into n_per_joint equal buckets; the seed
        samples the midpoint of each bucket, rotating the bucket index per
        joint to avoid diagonal bias.
        """
        lo = self.config.joint_lower
        hi = self.config.joint_upper
        n = self._n_joints
        strata = []
        for l, h in zip(lo, hi):
            width = (h - l) / n_per_joint
            strata.append([l + width * (k + 0.5) for k in range(n_per_joint)])

        seeds = []
        for i in range(n_per_joint):
            seed = [strata[j][(i + j) % n_per_joint] for j in range(n)]
            seeds.append(seed)
        return seeds

    def _jacobian_nudge_retry(
        self,
        target_pose: Pose,
        best_seed: List[float],
        orientations: List[Quaternion],
        t_start: float,
        n_nudges: int = 5,
        sigma: float = 0.15,
    ) -> Optional[IKResult]:
        """
        Last-chance fallback: aggressively perturb the best-failed seed and
        retry KDL against the top orientations.  A larger sigma (0.15 rad)
        is used here because fine perturbations have already been exhausted
        in the main loop.  Only the first four orientations are retried to
        keep wall-time bounded.
        """
        for _ in range(n_nudges):
            nudged = self._perturb_joints(best_seed, sigma=sigma)
            for orient in orientations[:4]:
                trial = Pose()
                trial.position = target_pose.position
                trial.orientation = orient
                result = self.solve(trial, nudged)
                if result.success:
                    result.solve_time_ms = (time.perf_counter() - t_start) * 1000
                    return result
        return None

    # ─────────────────────────────────────────────
    # Public multi-seed entry point
    # ─────────────────────────────────────────────

    def solve_multi_seed(
        self,
        target_pose: Pose,
        current_joints: Optional[List[float]] = None,
        num_attempts: int = 30,
    ) -> IKResult:
        """
        Robust IK via multiple seeds and orientations.

        Strategy (returns immediately on first success):
          1. Orientation bank  — user-requested + 12 standard EEF poses +
                                 6 cone-sampled variants near the request.
          2. Seed bank         — current joints (+ small perturbations),
                                 known-good neutral poses, uniform random,
                                 and a stratified joint-space grid.
          3. Jacobian-nudge    — aggressively perturbs the best-failed seed
                                 and retries KDL as a last resort.

        Much more robust than a single solve() call because KDL's
        Newton-Raphson solver is very sensitive to the initial seed.
        """
        t_start = time.perf_counter()

        # ── 1. Fast reachability pre-check ──────────────────────────────────
        p = target_pose.position
        reachable, reason = self.reachability.check(p.x, p.y, p.z)
        if not reachable:
            return IKResult(
                success=False,
                is_reachable=False,
                message=f"Pre-check FAILED: {reason}",
                solve_time_ms=(time.perf_counter() - t_start) * 1000,
            )

        # ── 2. Build orientation bank ────────────────────────────────────────
        orientations: List[Quaternion] = [target_pose.orientation]

        for r, p_angle, y in self._STANDARD_RPYS:
            rot = kdl.Rotation.RPY(r, p_angle, y)
            qx, qy, qz, qw = rot.GetQuaternion()
            orientations.append(Quaternion(x=qx, y=qy, z=qz, w=qw))

        # 6 cone-sampled orientations clustered near the user's request
        orientations.extend(
            self._cone_sample_orientations(target_pose.orientation, n=6, half_angle=0.3)
        )

        # ── 3. Build seed bank ───────────────────────────────────────────────
        seeds: List[List[float]] = []

        if current_joints and len(current_joints) == self._n_joints:
            seeds.append(current_joints)
            # Small perturbations around current config often escape local minima
            for _ in range(4):
                seeds.append(self._perturb_joints(current_joints, sigma=0.05))

        # Known-good TIAGo neutral poses
        seeds.append([0.20, -1.34, -0.20,  1.94, -1.57,  1.37, 0.0])
        seeds.append([0.0,  -0.785, 0.0,   1.57,  0.0,   0.785, 0.0])
        seeds.append([0.0,   0.0,   0.0,   1.57, -1.57,  0.0,   0.0])

        # Uniform random seeds within joint limits
        lo = self.config.joint_lower
        hi = self.config.joint_upper
        for _ in range(num_attempts):
            seeds.append([random.uniform(l, h) for l, h in zip(lo, hi)])

        # Stratified grid seeds — evenly spaced through joint space
        seeds.extend(self._stratified_seeds(n_per_joint=3))

        # ── 4. Main solve loop ───────────────────────────────────────────────
        best_result: Optional[IKResult] = None

        for orient in orientations:
            trial = Pose()
            trial.position = target_pose.position
            trial.orientation = orient

            for seed in seeds:
                result = self.solve(trial, seed)

                if result.success:
                    result.solve_time_ms = (time.perf_counter() - t_start) * 1000
                    return result

                # Track the attempt closest to convergence for the nudge stage
                if best_result is None or result.position_error < best_result.position_error:
                    best_result = result
                    best_result._best_seed = seed

        # ── 5. Jacobian-nudge fallback ───────────────────────────────────────
        if best_result is not None and best_result._best_seed:
            nudge_result = self._jacobian_nudge_retry(
                target_pose, best_result._best_seed, orientations, t_start
            )
            if nudge_result is not None and nudge_result.success:
                return nudge_result

        if best_result is not None:
            best_result.solve_time_ms = (time.perf_counter() - t_start) * 1000
        return best_result

    # ─────────────────────────────────────────────
    # Forward kinematics
    # ─────────────────────────────────────────────

    def forward_kinematics(self, joint_angles: List[float]) -> Optional[Pose]:
        """Compute FK: joint angles → end-effector pose in torso_lift_link frame."""
        q = kdl.JntArray(self._n_joints)
        for i, v in enumerate(joint_angles[:self._n_joints]):
            q[i] = v

        frame = kdl.Frame()
        ret = self._fk_solver.JntToCart(q, frame)
        if ret < 0:
            return None

        pose = Pose()
        pose.position.x = frame.p.x()
        pose.position.y = frame.p.y()
        pose.position.z = frame.p.z()
        qx, qy, qz, qw = frame.M.GetQuaternion()
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose