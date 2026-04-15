"""
test_reach_target.py
--------------------
CLI test for the ReachTarget action server.

Usage:
    # Full pipeline (navigate + torso + arm):
    ros2 run tiago_controller test_reach_target --ros-args \\
        -p x:=2.0 -p y:=0.0 -p z:=1.0

    # Skip navigation (only torso + arm):
    ros2 run tiago_controller test_reach_target --ros-args \\
        -p x:=0.5 -p y:=0.0 -p z:=1.0 -p skip_navigation:=true

    # Target TF frame:
    ros2 run tiago_controller test_reach_target --ros-args \\
        -p target_frame:=aruco_marker_frame

    # Dry run:
    ros2 run tiago_controller test_reach_target --ros-args \\
        -p x:=2.0 -p y:=0.0 -p z:=1.0 -p dry_run:=true
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

try:
    from tiago_controller_msgs.action import ReachTarget
    ACTION_AVAILABLE = True
except ImportError:
    ACTION_AVAILABLE = False


class ReachTargetTest(Node):

    def __init__(self):
        super().__init__('reach_target_test')
        self.log = self.get_logger()

        self.declare_parameter('x', 0.5)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 1.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('target_frame', '')
        self.declare_parameter('skip_navigation', False)
        self.declare_parameter('skip_torso', False)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('standoff', 0.0)

        if ACTION_AVAILABLE:
            self.client = ActionClient(self, ReachTarget, 'reach_target')
        else:
            self.log.error('tiago_controller_msgs not available!')

    def run(self):
        if not ACTION_AVAILABLE:
            rclpy.shutdown()
            return

        self.log.info('Waiting for /reach_target server...')
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.log.error('/reach_target server not available!')
            rclpy.shutdown()
            return

        goal = ReachTarget.Goal()

        target_frame = self.get_parameter('target_frame').value
        if target_frame:
            goal.target_frame = target_frame
            self.log.info(f'Target: TF frame "{target_frame}"')
        else:
            x = self.get_parameter('x').value
            y = self.get_parameter('y').value
            z = self.get_parameter('z').value
            frame = self.get_parameter('frame_id').value
            goal.target_pose.header.frame_id = frame
            goal.target_pose.pose.position.x = float(x)
            goal.target_pose.pose.position.y = float(y)
            goal.target_pose.pose.position.z = float(z)
            goal.target_pose.pose.orientation.w = 1.0
            self.log.info(f'Target: ({x:.2f}, {y:.2f}, {z:.2f}) in "{frame}"')

        goal.skip_navigation = self.get_parameter('skip_navigation').value
        goal.skip_torso = self.get_parameter('skip_torso').value
        goal.dry_run = self.get_parameter('dry_run').value
        goal.standoff_distance = float(self.get_parameter('standoff').value)

        flags = []
        if goal.skip_navigation:
            flags.append('skip_nav')
        if goal.skip_torso:
            flags.append('skip_torso')
        if goal.dry_run:
            flags.append('dry_run')
        self.log.info(f'Flags: {", ".join(flags) if flags else "none"}')

        self.log.info('Sending ReachTarget goal...')
        send_future = self.client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self.log.info(
                f'  [{fb.feedback.current_phase}] '
                f'{fb.feedback.progress*100:.0f}% — {fb.feedback.status}'),
        )
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()

        if not gh.accepted:
            self.log.error('Goal REJECTED')
            rclpy.shutdown()
            return

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result

        self.log.info('─' * 50)
        self.log.info(f'SUCCESS: {res.success}')
        self.log.info(f'  nav:   {res.navigation_succeeded}')
        self.log.info(f'  torso: {res.torso_succeeded} (height={res.torso_height:.3f}m)')
        self.log.info(f'  arm:   {res.arm_succeeded}')
        self.log.info(f'  msg:   {res.message}')

        if res.joint_names:
            self.log.info('  Arm joints:')
            import math
            for n, a in zip(res.joint_names, res.joint_angles):
                self.log.info(f'    {n:20s}: {math.degrees(a):+7.1f}°')

        rclpy.shutdown()


def main():
    rclpy.init()
    node = ReachTargetTest()
    node.run()


if __name__ == '__main__':
    main()
