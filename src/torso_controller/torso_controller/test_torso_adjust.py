"""
test_torso_adjust.py
--------------------
Standalone test node: compute and optionally execute a torso height adjustment.

Usage:
    # Move torso to optimal height for a target at (x=0.5, y=0.0, z=1.0):
    ros2 run torso_controller test_torso_adjust --ros-args \\
        -p x:=0.5 -p y:=0.0 -p z:=1.0

    # Dry run (no movement):
    ros2 run torso_controller test_torso_adjust --ros-args \\
        -p x:=0.5 -p y:=0.0 -p z:=1.0 -p dry_run:=true

    # Move to an explicit height (m) directly:
    ros2 run torso_controller test_torso_adjust --ros-args \\
        -p direct_height:=0.2

    # Use /torso_adjust action server (must be running):
    ros2 run torso_controller test_torso_adjust --ros-args \\
        -p x:=0.5 -p y:=0.0 -p z:=1.2 -p use_action:=true
"""

import math
import os
import sys
import time as wall_time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

sys.path.insert(0, os.path.dirname(__file__))
from torso_height_calculator import TorsoHeightCalculator, TorsoConfig

try:
    from torso_controller_msgs.action import TorsoAdjust
    ACTION_AVAILABLE = True
except ImportError:
    ACTION_AVAILABLE = False


class TorsoAdjustTest(Node):

    def __init__(self):
        super().__init__('torso_adjust_test')
        self.log = self.get_logger()

        # ── Parameters ──
        self.declare_parameter('x', 0.5)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 1.0)
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('direct_height', -1.0)   # < 0 means "compute from target"
        self.declare_parameter('use_action', False)
        self.declare_parameter('duration', 3.0)

        # ── State ──
        self.current_torso_height = None
        self.torso_cfg = TorsoConfig()
        self.calculator = TorsoHeightCalculator(self.torso_cfg)

        # ── Subscriber ──
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)

        # ── Publishers / clients ──
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/torso_controller/joint_trajectory', 10
        )
        self.follow_client = ActionClient(
            self, FollowJointTrajectory,
            '/torso_controller/follow_joint_trajectory',
        )
        if ACTION_AVAILABLE:
            self.adjust_client = ActionClient(self, TorsoAdjust, 'torso_adjust')

        self.log.info('Waiting for /joint_states ...')

    def _on_joints(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if 'torso_lift_joint' in name_to_pos:
            self.current_torso_height = name_to_pos['torso_lift_joint']

    def is_ready(self):
        return self.current_torso_height is not None

    def _run(self):
        direct = self.get_parameter('direct_height').value
        use_action = self.get_parameter('use_action').value
        dry_run = self.get_parameter('dry_run').value

        self.log.info(f'Current torso height: {self.current_torso_height:.4f} m')

        # ── Decide target height ──
        if direct >= 0.0:
            # Use the directly specified height
            target_h = max(self.torso_cfg.lower_limit,
                           min(self.torso_cfg.upper_limit, direct))
            self.log.info(f'Direct height requested: {target_h:.4f} m')
        else:
            x = self.get_parameter('x').value
            y = self.get_parameter('y').value
            z = self.get_parameter('z').value
            frame = self.get_parameter('frame_id').value
            self.log.info(f'Target: ({x:.3f}, {y:.3f}, {z:.3f}) in "{frame}"')

            if use_action and ACTION_AVAILABLE:
                self._run_via_action(x, y, z, frame, dry_run)
                return

            # Compute height locally
            target_h = self.calculator.compute(z)
            z_in_torso = self.calculator.target_z_in_torso(z, target_h)
            self.log.info(
                f'Optimal torso height: {self.calculator.height_description(target_h)}'
            )
            self.log.info(f'Target z in torso frame at this height: {z_in_torso:.3f} m')

        if dry_run:
            self.log.info('[DRY RUN] Torso will NOT move.')
            rclpy.shutdown()
            return

        delta = abs(target_h - self.current_torso_height)
        if delta < 0.005:
            self.log.info(
                f'Torso already at {target_h:.4f}m (Δ={delta*1000:.1f}mm). No move needed.'
            )
            rclpy.shutdown()
            return

        # ── Send trajectory via FollowJointTrajectory action ──
        self.log.info('Waiting for /torso_controller/follow_joint_trajectory ...')
        if not self.follow_client.wait_for_server(timeout_sec=5.0):
            self.log.error('FollowJointTrajectory action server not available!')
            rclpy.shutdown()
            return

        duration = self.get_parameter('duration').value
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['torso_lift_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [target_h]
        pt.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9),
        )
        goal.trajectory.points = [pt]

        self.log.info(f'Sending torso to {target_h:.4f}m over {duration:.1f}s ...')
        send_future = self.follow_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.log.error('Trajectory REJECTED by torso controller!')
            rclpy.shutdown()
            return

        self.log.info('Trajectory accepted — torso is moving ...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status

        if status == 4:  # STATUS_SUCCEEDED
            self.log.info(f'Torso reached {target_h:.4f} m!')
        else:
            self.log.warn(f'Trajectory finished with status={status}')

        rclpy.shutdown()

    def _run_via_action(self, x, y, z, frame, dry_run):
        """Call the /torso_adjust action server."""
        if not ACTION_AVAILABLE:
            self.log.error('torso_controller_msgs not available!')
            rclpy.shutdown()
            return

        self.log.info('Waiting for /torso_adjust action server ...')
        if not self.adjust_client.wait_for_server(timeout_sec=5.0):
            self.log.error('/torso_adjust action server not available!')
            rclpy.shutdown()
            return

        goal = TorsoAdjust.Goal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.w = 1.0
        goal.dry_run = dry_run

        self.log.info(f'Sending TorsoAdjust goal: target=({x:.3f},{y:.3f},{z:.3f}) ...')
        send_future = self.adjust_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self.log.info(
                f'  [{fb.feedback.progress*100:.0f}%] {fb.feedback.status}'
            ),
        )
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()

        if not gh.accepted:
            self.log.error('TorsoAdjust goal REJECTED')
            rclpy.shutdown()
            return

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result

        if res.success:
            self.log.info(
                f'TorsoAdjust SUCCESS: height={res.torso_height:.4f}m  |  {res.message}'
            )
        else:
            self.log.error(f'TorsoAdjust FAILED: {res.message}')

        rclpy.shutdown()


def main():
    rclpy.init()
    node = TorsoAdjustTest()
    while rclpy.ok() and not node.is_ready():
        rclpy.spin_once(node, timeout_sec=0.5)
    if rclpy.ok():
        node._run()


if __name__ == '__main__':
    main()
