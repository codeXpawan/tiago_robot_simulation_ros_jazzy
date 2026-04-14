"""
arm_reach_action_server.py
--------------------------
ROS 2 Action Server for TIAGo arm reachability + IK + execution.

This node:
  1. Listens for ArmReach action goals (a 3D PoseStamped)
  2. Transforms the pose into torso_lift_link frame (for IK)
  3. Runs the fast reachability pre-check
  4. Solves KDL IK to get joint angles
  5. Sends a JointTrajectory command to the arm controller
  6. Reports result back (compatible with Nav2 BehaviorTree action client)

Start the node:
    ros2 run tiago_arm_kinematics arm_reach_server

Call from terminal (for testing):
    ros2 action send_goal /arm_reach tiago_arm_kinematics/action/ArmReach \
      "{target_pose: {header: {frame_id: 'base_footprint'}, \
        pose: {position: {x: 0.5, y: 0.0, z: 0.9}, \
               orientation: {w: 1.0}}}, dry_run: false}"
"""

import asyncio
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import yaml
import os
import time
from typing import Optional, List

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration

# Local imports
import sys
sys.path.insert(0, os.path.dirname(__file__))
from tiago_kdl_ik import TiagoIKSolver, ArmConfig, IKResult
from tiago_tf_utils import TiagoTFUtils

# Action message (generated from ArmReach.action)
# If the action package is not yet built, we provide a runtime fallback below.
try:
    from arm_controller_msgs.action import ArmReach
    ACTION_AVAILABLE = True
except ImportError:
    ACTION_AVAILABLE = False


class ArmReachServer(Node):
    """
    ROS 2 Action Server: receives a 3D pose, checks reachability,
    solves IK, and commands the TIAGo arm.
    """

    def __init__(self):
        super().__init__("arm_reach_server")
        self.logger = self.get_logger()

        # ── Load config ──────────────────────────────────────────
        config_path = os.path.join(
            get_package_share_directory('arm_controller'), 'config', 'tiago_arm_config.yaml'
        )
        self.arm_cfg = self._load_config(config_path)

        # ── State ────────────────────────────────────────────────
        self.urdf_string: Optional[str] = None
        self.current_joint_state: Optional[List[float]] = None
        self.ik_solver: Optional[TiagoIKSolver] = None
        self.tf_utils: Optional[TiagoTFUtils] = None

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(
            String,
            "/robot_description",
            self._on_robot_description,
            qos_profile=rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            10,
        )

        # ── Arm trajectory publisher ──────────────────────────────
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )

        # ── Action server ────────────────────────────────────────
        cb_group = ReentrantCallbackGroup()
        if ACTION_AVAILABLE:
            self._action_server = ActionServer(
                self,
                ArmReach,
                "arm_reach",
                execute_callback=self._execute_goal,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                callback_group=cb_group,
            )
            self.logger.info("ArmReach action server started on /arm_reach")
        else:
            self.logger.warn(
                "ArmReach action type not found. Build the package first:\n"
                "  colcon build --packages-select tiago_arm_kinematics\n"
                "Action server NOT started. Use the direct service instead."
            )

        # TF utils (initialised after node is running)
        self.tf_utils = TiagoTFUtils(self)
        self.logger.info("Waiting for TF tree and robot_description...")

    # ─────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────

    def _on_robot_description(self, msg: String):
        """Parse URDF and build KDL chain once."""
        if self.ik_solver is not None:
            return  # already built

        self.urdf_string = msg.data
        try:
            self.ik_solver = TiagoIKSolver(self.urdf_string, self.arm_cfg)
            self.logger.info(
                f"KDL chain built: {self.arm_cfg.base_link} → {self.arm_cfg.tip_link} "
                f"({len(self.arm_cfg.joint_names)} joints)"
            )
        except Exception as e:
            self.logger.error(f"Failed to build KDL chain: {e}")

    def _on_joint_state(self, msg: JointState):
        """Cache current arm joint positions."""
        arm_joints = self.arm_cfg.joint_names
        state = []
        name_to_pos = dict(zip(msg.name, msg.position))
        for jname in arm_joints:
            state.append(name_to_pos.get(jname, 0.0))
        self.current_joint_state = state

    def _goal_callback(self, goal_request):
        self.logger.info("Received ArmReach goal")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.logger.info("ArmReach goal cancelled")
        return CancelResponse.ACCEPT

    # ─────────────────────────────────────────
    # Main execution
    # ─────────────────────────────────────────

    async def _execute_goal(self, goal_handle):
        """Full pipeline: transform → reachability → IK → (execute)."""
        result = ArmReach.Result()
        feedback = ArmReach.Feedback()

        target_pose: PoseStamped = goal_handle.request.target_pose
        dry_run: bool = goal_handle.request.dry_run
        timeout: float = goal_handle.request.timeout_sec or 2.0

        # ── 1. Check IK solver is ready ──────────────────────────
        feedback.status = "Checking IK solver..."
        feedback.progress = 0.1
        goal_handle.publish_feedback(feedback)

        if self.ik_solver is None:
            result.success = False
            result.message = "IK solver not ready (URDF not received yet)"
            goal_handle.abort()
            return result

        # ── 2. Transform pose to torso_lift_link frame ────────────
        feedback.status = "Transforming pose to torso frame..."
        feedback.progress = 0.2
        goal_handle.publish_feedback(feedback)

        # Retry with await asyncio.sleep() so the asyncio event loop thread is
        # released between attempts, letting TF subscription callbacks populate
        # the buffer (tf2's built-in blocking timeout deadlocks here).
        torso_pose = None
        tf_deadline = timeout + 2.0  # seconds to wait for TF
        tf_elapsed = 0.0
        while torso_pose is None and tf_elapsed < tf_deadline:
            torso_pose = self.tf_utils.any_frame_to_torso(target_pose)
            if torso_pose is None:
                await asyncio.sleep(0.05)
                tf_elapsed += 0.05

        if torso_pose is None:
            result.success = False
            result.is_reachable = False
            result.message = (
                f"TF transform failed after {tf_deadline:.1f}s: "
                f"{target_pose.header.frame_id} → torso_lift_link. "
                "Is the full TF tree up? Check: ros2 run tf2_tools view_frames"
            )
            goal_handle.abort()
            return result

        # ── 3. Solve IK ───────────────────────────────────────────
        feedback.status = "Running IK solver..."
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        ik: IKResult = self.ik_solver.solve(
            torso_pose.pose,
            current_joints=self.current_joint_state,
        )

        result.is_reachable = ik.is_reachable
        result.message = ik.message
        result.ik_solve_time_ms = ik.solve_time_ms
        result.joint_angles = ik.joint_angles
        result.joint_names = ik.joint_names

        self.logger.info(
            f"IK result: {'✓ success' if ik.success else '✗ failed'} | "
            f"{ik.solve_time_ms:.1f}ms | {ik.message}"
        )

        if not ik.success:
            result.success = False
            goal_handle.succeed()
            return result

        # ── 4. Execute (unless dry_run) ───────────────────────────
        if dry_run:
            feedback.status = "Dry run complete (not moving arm)"
            feedback.progress = 1.0
            goal_handle.publish_feedback(feedback)
            result.success = True
            result.message = f"[DRY RUN] IK solved. {ik.message}"
            goal_handle.succeed()
            return result

        feedback.status = "Sending trajectory to arm controller..."
        feedback.progress = 0.8
        goal_handle.publish_feedback(feedback)

        self._send_joint_trajectory(ik.joint_angles, ik.joint_names, duration_sec=3.0)

        feedback.status = "Trajectory sent. Waiting for completion..."
        feedback.progress = 0.9
        goal_handle.publish_feedback(feedback)

        # Wait for motion (simple time-based wait; replace with /follow_joint_trajectory feedback for production)
        await self._async_sleep(3.5)

        result.success = True
        result.message = f"Arm moved successfully. {ik.message}"
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        return result

    # ─────────────────────────────────────────
    # Arm execution
    # ─────────────────────────────────────────

    def _send_joint_trajectory(
        self,
        joint_angles: List[float],
        joint_names: List[str],
        duration_sec: float = 3.0,
    ):
        """
        Publish a JointTrajectory command to /arm_controller/joint_trajectory.
        The arm_controller (ros2_controllers JointTrajectoryController) executes it.
        """
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.velocities = [0.0] * len(joint_angles)
        point.accelerations = [0.0] * len(joint_angles)
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9),
        )

        traj.points = [point]
        self.traj_pub.publish(traj)
        self.logger.info(
            f"Published trajectory: {len(joint_angles)} joints, "
            f"duration={duration_sec}s\n"
            + "\n".join(
                f"  {n}: {math.degrees(a):.1f}°"
                for n, a in zip(joint_names, joint_angles)
            )
        )

    async def _async_sleep(self, seconds: float):
        """Non-blocking sleep for async action execution."""
        await asyncio.sleep(seconds)

    # ─────────────────────────────────────────
    # Config loading
    # ─────────────────────────────────────────

    def _load_config(self, config_path: str) -> ArmConfig:
        """Load ArmConfig from YAML, fall back to defaults."""
        try:
            with open(config_path) as f:
                raw = yaml.safe_load(f)
            arm = raw["arm"]
            limits = arm["joint_limits"]
            jnames = arm["joint_names"]
            ws = arm["workspace"]

            return ArmConfig(
                base_link=arm["base_link"],
                tip_link=arm["tip_link"],
                joint_names=jnames,
                joint_lower=[limits[j][0] for j in jnames],
                joint_upper=[limits[j][1] for j in jnames],
                max_iterations=arm["ik_solver"]["max_iterations"],
                epsilon=arm["ik_solver"]["epsilon"],
                timeout_sec=arm["ik_solver"]["timeout_sec"],
                workspace_x=(ws["x_min"], ws["x_max"]),
                workspace_y=(ws["y_min"], ws["y_max"]),
                workspace_z=(ws["z_min"], ws["z_max"]),
                max_reach=ws["max_reach"],
            )
        except Exception as e:
            self.get_logger().warn(
                f"Could not load config from {config_path}: {e}. Using defaults."
            )
            return ArmConfig()


def main():
    rclpy.init()
    node = ArmReachServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
