"""
arm_reach_action_server.py
--------------------------
ROS 2 Action Server for TIAGo arm reachability + IK + execution.

Pipeline:
  1. (Optional) Pre-position torso via /torso_adjust
  2. Transform target pose into torso_lift_link frame
  3. Multi-seed KDL IK solve
  4. Send JointTrajectory to the arm controller
"""

import math
import os
import sys
import time
import yaml
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration

sys.path.insert(0, os.path.dirname(__file__))
from tiago_kdl_ik import TiagoIKSolver, ArmConfig, IKResult
from tiago_tf_utils import TiagoTFUtils

try:
    from arm_controller_msgs.action import ArmReach
    ACTION_AVAILABLE = True
except ImportError:
    ACTION_AVAILABLE = False

try:
    from torso_controller_msgs.action import TorsoAdjust
    TORSO_ACTION_AVAILABLE = True
except ImportError:
    TORSO_ACTION_AVAILABLE = False

try:
    from collision_detector_msgs.srv import CheckCollision
    COLLISION_SRV_AVAILABLE = True
except ImportError:
    COLLISION_SRV_AVAILABLE = False


class ArmReachServer(Node):

    def __init__(self):
        super().__init__("arm_reach_server")
        self.logger = self.get_logger()

        config_path = os.path.join(
            get_package_share_directory('arm_controller'), 'config', 'tiago_arm_config.yaml')
        self.arm_cfg = self._load_config(config_path)

        self.urdf_string: Optional[str] = None
        self.current_joint_state: Optional[List[float]] = None
        self.ik_solver: Optional[TiagoIKSolver] = None

        self.create_subscription(
            String, "/robot_description", self._on_robot_description,
            qos_profile=rclpy.qos.QoSProfile(
                depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        self.traj_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10)

        # Torso client (optional)
        if TORSO_ACTION_AVAILABLE:
            self._torso_client = ActionClient(
                self, TorsoAdjust, 'torso_adjust',
                callback_group=ReentrantCallbackGroup())
            self.logger.info('TorsoAdjust client ready.')
        else:
            self._torso_client = None

        # Collision detector client (optional)
        if COLLISION_SRV_AVAILABLE:
            self._collision_client = self.create_client(
                CheckCollision, '/check_collision',
                callback_group=ReentrantCallbackGroup())
            self.logger.info('CheckCollision client ready.')
        else:
            self._collision_client = None

        # Action server
        if ACTION_AVAILABLE:
            self._action_server = ActionServer(
                self, ArmReach, "arm_reach",
                execute_callback=self._execute_goal,
                goal_callback=lambda _: GoalResponse.ACCEPT,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=ReentrantCallbackGroup())
            self.logger.info("ArmReach action server on /arm_reach")

        self.tf_utils = TiagoTFUtils(self)
        self.logger.info("Waiting for TF + robot_description...")

    # ── Subscription callbacks ───────────────────────────────────

    def _on_robot_description(self, msg: String):
        if self.ik_solver is not None:
            return
        self.urdf_string = msg.data
        try:
            self.ik_solver = TiagoIKSolver(self.urdf_string, self.arm_cfg)
            self.logger.info(
                f"KDL chain built: {self.arm_cfg.base_link} → {self.arm_cfg.tip_link} "
                f"({len(self.arm_cfg.joint_names)} joints)")
        except Exception as e:
            self.logger.error(f"KDL chain failed: {e}")

    def _on_joint_state(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        self.current_joint_state = [
            name_to_pos.get(j, 0.0) for j in self.arm_cfg.joint_names]

    # ── Synchronous execute (no asyncio) ─────────────────────────

    def _execute_goal(self, goal_handle):
        result = ArmReach.Result()
        feedback = ArmReach.Feedback()

        target_pose: PoseStamped = goal_handle.request.target_pose
        dry_run: bool = goal_handle.request.dry_run
        timeout: float = goal_handle.request.timeout_sec or 2.0

        # 1. Check solver ready
        feedback.status = "Checking IK solver..."
        feedback.progress = 0.1
        goal_handle.publish_feedback(feedback)

        if self.ik_solver is None:
            result.success = False
            result.message = "IK solver not ready (URDF not received)"
            goal_handle.abort()
            return result

        # 2. Pre-position torso
        feedback.status = "Pre-positioning torso..."
        feedback.progress = 0.15
        goal_handle.publish_feedback(feedback)

        self._adjust_torso(target_pose, dry_run)

        # 3. Transform to torso frame
        feedback.status = "Transforming to torso frame..."
        feedback.progress = 0.3
        goal_handle.publish_feedback(feedback)

        torso_pose = self._wait_for_torso_tf(target_pose, timeout + 2.0)
        if torso_pose is None:
            result.success = False
            result.is_reachable = False
            result.message = (
                f"TF failed: {target_pose.header.frame_id} → torso_lift_link")
            goal_handle.abort()
            return result

        # 4. Solve IK (multi-seed)
        feedback.status = "Running IK solver..."
        feedback.progress = 0.55
        goal_handle.publish_feedback(feedback)

        ik: IKResult = self.ik_solver.solve_multi_seed(
            torso_pose.pose, current_joints=self.current_joint_state)

        result.is_reachable = ik.is_reachable
        result.message = ik.message
        result.ik_solve_time_ms = ik.solve_time_ms
        result.joint_angles = ik.joint_angles
        result.joint_names = ik.joint_names

        self.logger.info(
            f"IK: {'OK' if ik.success else 'FAIL'} | "
            f"{ik.solve_time_ms:.1f}ms | {ik.message}")

        if not ik.success:
            result.success = False
            goal_handle.succeed()
            return result

        # 4.5 Collision check (if service available)
        if not dry_run:
            collision_ok = self._check_collision(
                ik.joint_angles, ik.joint_names, feedback, goal_handle)
            if not collision_ok:
                result.success = False
                result.message = f"Collision detected — arm motion blocked. {ik.message}"
                goal_handle.succeed()
                return result

        # 5. Execute (unless dry_run)
        if dry_run:
            result.success = True
            result.message = f"[DRY RUN] IK solved. {ik.message}"
            feedback.progress = 1.0
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            return result

        feedback.status = "Sending trajectory..."
        feedback.progress = 0.8
        goal_handle.publish_feedback(feedback)

        self._send_joint_trajectory(ik.joint_angles, ik.joint_names, duration_sec=3.0)

        feedback.status = "Waiting for arm motion..."
        feedback.progress = 0.9
        goal_handle.publish_feedback(feedback)

        time.sleep(3.5)

        result.success = True
        result.message = f"Arm moved. {ik.message}"
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        return result

    # ── Torso pre-positioning ────────────────────────────────────

    def _adjust_torso(self, target_pose: PoseStamped, dry_run: bool):
        """Call /torso_adjust synchronously. Fails silently if unavailable."""
        if self._torso_client is None:
            return
        if not self._torso_client.wait_for_server(timeout_sec=1.0):
            self.logger.info('TorsoAdjust server not available. Skipping.')
            return

        goal = TorsoAdjust.Goal()
        goal.target_pose = target_pose
        goal.dry_run = dry_run
        goal.timeout_sec = 5.0

        try:
            send_future = self._torso_client.send_goal_async(goal)
            if not self._poll_future(send_future, 10.0):
                self.logger.warn('TorsoAdjust send timeout.')
                return

            gh = send_future.result()
            if not gh.accepted:
                self.logger.warn('TorsoAdjust goal rejected.')
                return

            result_future = gh.get_result_async()
            if not self._poll_future(result_future, 15.0):
                self.logger.warn('TorsoAdjust result timeout.')
                return

            res = result_future.result().result
            if res.success:
                self.logger.info(
                    f'Torso: {res.previous_torso_height:.3f}m → {res.torso_height:.3f}m')
            else:
                self.logger.warn(f'TorsoAdjust failed: {res.message}')
        except Exception as e:
            self.logger.warn(f'TorsoAdjust error: {e}')

    # ── Collision check ────────────────────────────────────────────

    def _check_collision(self, joint_angles, joint_names, feedback, goal_handle):
        """
        Call /check_collision service. Returns True if collision-free.
        Silently passes if the service is unavailable.
        """
        if self._collision_client is None:
            return True  # no collision detector — allow motion

        if not self._collision_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('CheckCollision service not available. Skipping check.')
            return True

        feedback.status = "Checking for collisions..."
        feedback.progress = 0.65
        goal_handle.publish_feedback(feedback)

        req = CheckCollision.Request()
        req.joint_angles = list(joint_angles)
        req.joint_names = list(joint_names)
        req.safety_margin = 0.0  # use server default

        future = self._collision_client.call_async(req)
        if not self._poll_future(future, 5.0):
            self.logger.warn('CheckCollision service timeout. Allowing motion.')
            return True

        resp = future.result()
        if not resp.has_point_cloud:
            self.logger.info(f'Collision check: {resp.message}. Allowing motion.')
            return True

        self.logger.info(f'Collision check: {resp.message}')
        return resp.collision_free

    # ── Helpers ───────────────────────────────────────────────────

    def _poll_future(self, future, timeout_sec):
        """Block until rclpy future completes. Returns True if done."""
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.done()

    def _wait_for_torso_tf(self, target_pose, timeout):
        """Poll TF for torso_lift_link transform."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = self.tf_utils.any_frame_to_torso(target_pose)
            if result is not None:
                return result
            time.sleep(0.05)
        return None

    def _send_joint_trajectory(self, joint_angles, joint_names, duration_sec=3.0):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = joint_names
        pt = JointTrajectoryPoint()
        pt.positions = joint_angles
        pt.velocities = [0.0] * len(joint_angles)
        pt.accelerations = [0.0] * len(joint_angles)
        pt.time_from_start = Duration(
            sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        traj.points = [pt]
        self.traj_pub.publish(traj)
        self.logger.info(
            f"Trajectory: {len(joint_angles)} joints, {duration_sec}s\n" +
            "\n".join(f"  {n}: {math.degrees(a):.1f}°"
                      for n, a in zip(joint_names, joint_angles)))

    def _load_config(self, config_path):
        try:
            with open(config_path) as f:
                raw = yaml.safe_load(f)
            arm = raw["arm"]
            limits = arm["joint_limits"]
            jnames = arm["joint_names"]
            ws = arm["workspace"]
            return ArmConfig(
                base_link=arm["base_link"], tip_link=arm["tip_link"],
                joint_names=jnames,
                joint_lower=[limits[j][0] for j in jnames],
                joint_upper=[limits[j][1] for j in jnames],
                max_iterations=arm["ik_solver"]["max_iterations"],
                epsilon=arm["ik_solver"]["epsilon"],
                timeout_sec=arm["ik_solver"]["timeout_sec"],
                workspace_x=(ws["x_min"], ws["x_max"]),
                workspace_y=(ws["y_min"], ws["y_max"]),
                workspace_z=(ws["z_min"], ws["z_max"]),
                max_reach=ws["max_reach"])
        except Exception as e:
            self.get_logger().warn(f"Config load failed: {e}. Using defaults.")
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
