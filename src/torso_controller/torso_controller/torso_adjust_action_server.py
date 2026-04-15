"""
torso_adjust_action_server.py
------------------------------
ROS 2 Action Server for TIAGo torso height adjustment.

Given a target 3D pose (in any frame), this node:
  1. Transforms the pose into base_footprint to extract the target Z
  2. Computes the optimal torso_lift_joint value via TorsoHeightCalculator
  3. Sends a JointTrajectory command to /torso_controller/joint_trajectory
  4. Waits for the torso to settle, then reports success

Start:
    ros2 run torso_controller torso_adjust_server
"""

import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration

import sys
sys.path.insert(0, os.path.dirname(__file__))
from torso_height_calculator import TorsoHeightCalculator, TorsoConfig

import tf2_ros
import tf2_geometry_msgs  # noqa: F401
import rclpy.time

try:
    from torso_controller_msgs.action import TorsoAdjust
    ACTION_AVAILABLE = True
except ImportError:
    ACTION_AVAILABLE = False


class TorsoAdjustServer(Node):

    def __init__(self):
        super().__init__('torso_adjust_server')
        self.logger = self.get_logger()

        config_path = os.path.join(
            get_package_share_directory('torso_controller'), 'config', 'tiago_torso_config.yaml'
        )
        self.torso_cfg = self._load_config(config_path)
        self.calculator = TorsoHeightCalculator(self.torso_cfg)

        self.current_torso_height: float = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

        self.traj_pub = self.create_publisher(
            JointTrajectory, '/torso_controller/joint_trajectory', 10)

        if ACTION_AVAILABLE:
            self._action_server = ActionServer(
                self, TorsoAdjust, 'torso_adjust',
                execute_callback=self._execute_goal,
                goal_callback=lambda _: GoalResponse.ACCEPT,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=ReentrantCallbackGroup(),
            )
            self.logger.info('TorsoAdjust action server started on /torso_adjust')

    def _on_joint_state(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if self.torso_cfg.joint_name in name_to_pos:
            self.current_torso_height = name_to_pos[self.torso_cfg.joint_name]

    # ── Synchronous execute (no asyncio) ─────────────────────────

    def _execute_goal(self, goal_handle):
        result = TorsoAdjust.Result()
        feedback = TorsoAdjust.Feedback()

        target_pose: PoseStamped = goal_handle.request.target_pose
        dry_run: bool = goal_handle.request.dry_run
        timeout: float = goal_handle.request.timeout_sec or 5.0

        prev_height = self.current_torso_height
        result.previous_torso_height = prev_height

        # 1. Transform target to base_footprint
        feedback.status = 'Transforming target to base_footprint...'
        feedback.progress = 0.1
        goal_handle.publish_feedback(feedback)

        base_pose = self._wait_for_tf(target_pose, 'base_footprint', timeout)
        if base_pose is None:
            result.success = False
            result.torso_height = prev_height
            result.message = (
                f'TF to base_footprint failed after {timeout:.1f}s '
                f'(source: {target_pose.header.frame_id})')
            goal_handle.abort()
            return result

        target_z = base_pose.pose.position.z

        # 2. Compute optimal height
        feedback.status = 'Computing optimal torso height...'
        feedback.progress = 0.3
        goal_handle.publish_feedback(feedback)

        optimal_h = self.calculator.compute(target_z)
        z_in_torso = self.calculator.target_z_in_torso(target_z, optimal_h)

        self.logger.info(
            f'Target z={target_z:.3f}m → torso_lift={self.calculator.height_description(optimal_h)} '
            f'| z in torso frame={z_in_torso:.3f}m')

        result.torso_height = optimal_h

        # 3. Dry run
        if dry_run:
            result.success = True
            result.message = (
                f'[DRY RUN] Optimal: {self.calculator.height_description(optimal_h)}. '
                f'z in torso frame would be {z_in_torso:.3f}m.')
            feedback.progress = 1.0
            feedback.status = result.message
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            return result

        # 4. Skip if already at target
        if abs(optimal_h - prev_height) < 0.005:
            result.success = True
            result.message = (
                f'Torso already at {self.calculator.height_description(optimal_h)} '
                f'(Δ={abs(optimal_h - prev_height)*1000:.1f}mm).')
            feedback.progress = 1.0
            feedback.status = result.message
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            return result

        # 5. Send trajectory
        feedback.status = f'Moving torso to {optimal_h:.3f}m...'
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        self._send_torso_trajectory(optimal_h, self.torso_cfg.motion_duration)

        # 6. Wait for motion
        feedback.status = 'Waiting for torso to settle...'
        feedback.progress = 0.8
        goal_handle.publish_feedback(feedback)

        time.sleep(self.torso_cfg.settle_wait)

        result.success = True
        result.message = (
            f'Torso moved to {self.calculator.height_description(optimal_h)} '
            f'(was {prev_height:.3f}m). z in torso frame: {z_in_torso:.3f}m.')
        feedback.progress = 1.0
        feedback.status = result.message
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        return result

    # ── Helpers ───────────────────────────────────────────────────

    def _wait_for_tf(self, pose_stamped, target_frame, timeout):
        """Poll TF until transform succeeds or timeout."""
        src = pose_stamped.header.frame_id
        if src == target_frame:
            return pose_stamped
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                ps = PoseStamped()
                ps.header.frame_id = src
                ps.header.stamp = rclpy.time.Time().to_msg()
                ps.pose = pose_stamped.pose
                return self.tf_buffer.transform(ps, target_frame)
            except Exception:
                time.sleep(0.05)
        return None

    def _send_torso_trajectory(self, height, duration_sec=3.0):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = [self.torso_cfg.joint_name]
        pt = JointTrajectoryPoint()
        pt.positions = [height]
        pt.velocities = [0.0]
        pt.accelerations = [0.0]
        pt.time_from_start = Duration(
            sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        traj.points = [pt]
        self.traj_pub.publish(traj)
        self.logger.info(f'Torso trajectory: {height:.4f}m over {duration_sec:.1f}s')

    def _load_config(self, config_path):
        try:
            with open(config_path) as f:
                raw = yaml.safe_load(f)
            t = raw['torso']
            cfg = TorsoConfig(
                joint_name=t['joint_name'],
                lower_limit=float(t['lower_limit']),
                upper_limit=float(t['upper_limit']),
                torso_base_z=float(t['torso_base_z']),
                arm_workspace_target_z=float(t['arm_workspace_target_z']),
            )
            cfg.motion_duration = float(t['motion']['trajectory_duration_sec'])
            cfg.settle_wait = float(t['motion']['settle_wait_sec'])
            return cfg
        except Exception as e:
            self.get_logger().warn(f'Config load failed: {e}. Using defaults.')
            cfg = TorsoConfig()
            cfg.motion_duration = 3.0
            cfg.settle_wait = 3.5
            return cfg


def main():
    rclpy.init()
    node = TorsoAdjustServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
