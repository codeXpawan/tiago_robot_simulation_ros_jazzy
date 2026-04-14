"""
test_arm_reach.py
-----------------
Standalone node: check if a position / TF frame is reachable, then move the arm.

No action server required -- builds the IK solver directly from /robot_description
and commands the arm via the ros2_control FollowJointTrajectory action.

Usage:
    # Position in base_footprint (dry-run):
    ros2 run arm_controller test_arm_reach --ros-args \
        -p x:=0.5 -p y:=0.0 -p z:=1.0 -p dry_run:=true

    # Position (actually move):
    ros2 run arm_controller test_arm_reach --ros-args \
        -p x:=0.5 -p y:=0.0 -p z:=1.0

    # TF frame (e.g. an ArUco tag or object):
    ros2 run arm_controller test_arm_reach --ros-args \
        -p target_frame:=aruco_marker_frame
"""

import math
import os
import sys
import time as wall_time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

sys.path.insert(0, os.path.dirname(__file__))
from tiago_kdl_ik import TiagoIKSolver, ArmConfig
from tiago_tf_utils import TiagoTFUtils


class ArmReachTest(Node):

    def __init__(self):
        super().__init__('arm_reach_test')
        self.log = self.get_logger()

        # ── Parameters ──
        self.declare_parameter('target_frame', '')   # TF frame name (overrides x/y/z)
        self.declare_parameter('x', 0.5)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 1.0)             # in base_footprint frame
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('duration', 3.0)      # trajectory duration (seconds)

        # ── Internal state ──
        self.ik_solver = None
        self.current_joints = None
        self.arm_cfg = ArmConfig()
        self.tf_utils = TiagoTFUtils(self)

        # ── Subscribers ──
        self.create_subscription(
            String, '/robot_description', self._on_urdf,
            qos_profile=rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)

        # ── Arm action client (ros2_control standard) ──
        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )

        self.log.info('Waiting for /robot_description and /joint_states ...')

    # ─────────────── callbacks ───────────────

    def _on_urdf(self, msg: String):
        if self.ik_solver is not None:
            return
        try:
            self.ik_solver = TiagoIKSolver(msg.data, self.arm_cfg)
            self.log.info(
                f'IK solver ready  '
                f'{self.arm_cfg.base_link} -> {self.arm_cfg.tip_link}  '
                f'({len(self.arm_cfg.joint_names)} joints)')
        except Exception as e:
            self.log.error(f'IK solver build failed: {e}')

    def _on_joints(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        self.current_joints = [
            name_to_pos.get(j, 0.0) for j in self.arm_cfg.joint_names
        ]

    def is_ready(self):
        return self.ik_solver is not None and self.current_joints is not None

    # ─────────────── main logic ───────────────

    def _run(self):
        target_frame = self.get_parameter('target_frame').value

        # ── 1. Build target pose ──
        if target_frame:
            self.log.info(f'Looking up TF frame: {target_frame}')
            tf = self.tf_utils.get_transform(target_frame, 'torso_lift_link')
            if tf is None:
                self.log.error(f'TF frame "{target_frame}" not found')
                rclpy.shutdown(); return
            target = PoseStamped()
            target.header.frame_id = 'torso_lift_link'
            target.pose.position.x = tf.transform.translation.x
            target.pose.position.y = tf.transform.translation.y
            target.pose.position.z = tf.transform.translation.z
            target.pose.orientation = tf.transform.rotation
        else:
            x = self.get_parameter('x').value
            y = self.get_parameter('y').value
            z = self.get_parameter('z').value
            frame_id = self.get_parameter('frame_id').value
            target = TiagoTFUtils.euler_to_pose_stamped(
                x, y, z, 0.0, 0.0, 0.0, frame_id)
            self.log.info(
                f'Target: ({x:.3f}, {y:.3f}, {z:.3f}) in "{frame_id}"')

        # ── 2. Transform to torso_lift_link ──
        if target.header.frame_id != 'torso_lift_link':
            # Non-blocking retry: spin between attempts so TF data arrives
            torso_pose = None
            for _ in range(40):
                torso_pose = self.tf_utils.any_frame_to_torso(target)
                if torso_pose is not None:
                    break
                rclpy.spin_once(self, timeout_sec=0.05)
            if torso_pose is None:
                self.log.error(
                    f'TF: {target.header.frame_id} -> torso_lift_link failed')
                rclpy.shutdown(); return
        else:
            torso_pose = target

        p = torso_pose.pose.position
        self.log.info(
            f'In torso frame: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')

        # ── 3. Solve IK (multi-seed) ──
        self.log.info('Solving IK (multi-seed) ...')
        ik = self.ik_solver.solve_multi_seed(
            torso_pose.pose, self.current_joints, num_attempts=30)

        self.log.info(
            f'IK {"SUCCESS" if ik.success else "FAILED"} | '
            f'reachable={ik.is_reachable} | '
            f'{ik.solve_time_ms:.0f} ms')
        self.log.info(f'  {ik.message}')

        if ik.joint_angles:
            self.log.info('  Joint solution:')
            for name, angle in zip(ik.joint_names, ik.joint_angles):
                self.log.info(f'    {name:20s}: {math.degrees(angle):+7.1f} deg')

        if not ik.success:
            self.log.error('Target not reachable by the arm.')
            rclpy.shutdown(); return

        # ── 4. Dry-run check ──
        if self.get_parameter('dry_run').value:
            self.log.info('[DRY RUN] IK solved -- arm will NOT move.')
            rclpy.shutdown(); return

        # ── 5. Command arm via FollowJointTrajectory ──
        self.log.info('Waiting for /arm_controller/follow_joint_trajectory ...')
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.log.error('Action server not available!')
            rclpy.shutdown(); return

        duration = self.get_parameter('duration').value
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ik.joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = list(ik.joint_angles)
        pt.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9),
        )
        goal.trajectory.points = [pt]

        self.log.info(f'Sending trajectory ({duration:.1f}s) ...')
        send_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.log.error('Trajectory REJECTED by controller')
            rclpy.shutdown(); return

        self.log.info('Trajectory accepted -- arm is moving ...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status

        # action_msgs/GoalStatus: STATUS_SUCCEEDED = 4
        if status == 4:
            self.log.info('Arm reached target position!')
        else:
            self.log.warn(f'Trajectory finished with status={status}')

        rclpy.shutdown()


def main():
    rclpy.init()
    node = ArmReachTest()
    # Spin until IK solver and joint states are available
    while rclpy.ok() and not node.is_ready():
        rclpy.spin_once(node, timeout_sec=0.5)
    if rclpy.ok():
        node._run()


if __name__ == '__main__':
    main()
