"""
reach_target_action_server.py
------------------------------
High-level TIAGo controller that sequences:

    1. Navigate base  →  position the robot in front of the target  (Nav2)
    2. Adjust torso   →  lift / lower for optimal arm workspace      (direct)
    3. Reach with arm →  IK solve + execute trajectory               (ArmReach)

Usage:
    ros2 run tiago_controller reach_target_server

    # From terminal:
    ros2 action send_goal /reach_target tiago_controller_msgs/action/ReachTarget \
      "{target_pose: {header: {frame_id: 'map'}, \
        pose: {position: {x: 2.0, y: 0.0, z: 1.0}, \
               orientation: {w: 1.0}}}}"
"""

import math
import os
import sys
import time
import yaml
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import tf2_ros
import tf2_geometry_msgs  # noqa: F401
import rclpy.time

sys.path.insert(0, os.path.dirname(__file__))
from navigation_helper import compute_nav_goal

try:
    from tiago_controller_msgs.action import ReachTarget
    REACH_ACTION_AVAILABLE = True
except ImportError:
    REACH_ACTION_AVAILABLE = False

try:
    from nav2_msgs.action import NavigateToPose
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False

try:
    from arm_controller_msgs.action import ArmReach
    ARM_AVAILABLE = True
except ImportError:
    ARM_AVAILABLE = False


class ReachTargetServer(Node):
    """
    Orchestrator: Navigate → Torso → Arm to reach a 3D target.
    """

    def __init__(self):
        super().__init__('reach_target_server')
        self.logger = self.get_logger()

        # ── Config ───────────────────────────────────────────────
        config_path = os.path.join(
            get_package_share_directory('tiago_controller'),
            'config', 'tiago_reach_config.yaml')
        self.cfg = self._load_config(config_path)

        # ── State ────────────────────────────────────────────────
        self.current_torso_height: float = 0.0

        # ── TF ───────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

        # ── Torso trajectory publisher (direct, no action server) ─
        self.torso_pub = self.create_publisher(
            JointTrajectory, '/torso_controller/joint_trajectory', 10)

        # ── Nav2 action client ───────────────────────────────────
        cb = ReentrantCallbackGroup()
        if NAV2_AVAILABLE:
            self._nav_client = ActionClient(
                self, NavigateToPose,
                self.cfg['navigation']['action_name'],
                callback_group=cb)
            self.logger.info('Nav2 client ready.')
        else:
            self._nav_client = None
            self.logger.warn('nav2_msgs not found — navigation disabled.')

        # ── ArmReach action client ───────────────────────────────
        if ARM_AVAILABLE:
            self._arm_client = ActionClient(
                self, ArmReach,
                self.cfg['arm']['action_name'],
                callback_group=cb)
            self.logger.info('ArmReach client ready.')
        else:
            self._arm_client = None
            self.logger.warn('arm_controller_msgs not found — arm disabled.')

        # ── ReachTarget action server ────────────────────────────
        if REACH_ACTION_AVAILABLE:
            self._action_server = ActionServer(
                self, ReachTarget, 'reach_target',
                execute_callback=self._execute_goal,
                goal_callback=lambda _: GoalResponse.ACCEPT,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=cb)
            self.logger.info('ReachTarget action server on /reach_target')

    # ── Subscription callbacks ───────────────────────────────────

    def _on_joint_state(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        jn = self.cfg['torso']['joint_name']
        if jn in name_to_pos:
            self.current_torso_height = name_to_pos[jn]

    # ── Main orchestrator (synchronous) ──────────────────────────

    def _execute_goal(self, goal_handle):
        result = ReachTarget.Result()
        fb = ReachTarget.Feedback()

        req = goal_handle.request
        dry_run = req.dry_run
        standoff = req.standoff_distance if req.standoff_distance > 0 else \
            self.cfg['default_standoff_distance']

        # ── Resolve target pose ──────────────────────────────────
        fb.status = 'Resolving target...'
        fb.progress = 0.05
        fb.current_phase = 'init'
        goal_handle.publish_feedback(fb)

        target_pose = self._resolve_target(req)
        if target_pose is None:
            result.success = False
            result.message = f'Cannot resolve target (frame={req.target_frame})'
            goal_handle.abort()
            return result

        self.logger.info(
            f'Target: ({target_pose.pose.position.x:.2f}, '
            f'{target_pose.pose.position.y:.2f}, '
            f'{target_pose.pose.position.z:.2f}) in {target_pose.header.frame_id}')

        # ═════════════════════════════════════════════════════════
        # PHASE 1: NAVIGATE BASE
        # ═════════════════════════════════════════════════════════
        result.navigation_succeeded = True
        if not req.skip_navigation:
            fb.status = 'Computing navigation goal...'
            fb.progress = 0.1
            fb.current_phase = 'navigating'
            goal_handle.publish_feedback(fb)

            nav_ok, nav_msg = self._navigate_to_target(
                target_pose, standoff, goal_handle, fb, dry_run)
            result.navigation_succeeded = nav_ok
            if not nav_ok:
                result.success = False
                result.message = f'Navigation failed: {nav_msg}'
                goal_handle.abort()
                return result

        # ═════════════════════════════════════════════════════════
        # PHASE 2: ADJUST TORSO
        # ═════════════════════════════════════════════════════════
        result.torso_succeeded = True
        if not req.skip_torso:
            fb.status = 'Adjusting torso height...'
            fb.progress = 0.55
            fb.current_phase = 'adjusting_torso'
            goal_handle.publish_feedback(fb)

            torso_ok, torso_h = self._adjust_torso(
                target_pose, goal_handle, fb, dry_run)
            result.torso_succeeded = torso_ok
            result.torso_height = torso_h

        # ═════════════════════════════════════════════════════════
        # PHASE 3: REACH WITH ARM
        # ═════════════════════════════════════════════════════════
        fb.status = 'Sending arm reach goal...'
        fb.progress = 0.7
        fb.current_phase = 'reaching_arm'
        goal_handle.publish_feedback(fb)

        arm_ok, arm_msg, arm_result = self._reach_with_arm(
            target_pose, goal_handle, fb, dry_run)
        result.arm_succeeded = arm_ok
        if arm_result is not None:
            result.joint_angles = list(arm_result.joint_angles)
            result.joint_names = list(arm_result.joint_names)

        # ── Final result ─────────────────────────────────────────
        result.success = (result.navigation_succeeded and
                          result.torso_succeeded and
                          result.arm_succeeded)
        phases = []
        if result.navigation_succeeded:
            phases.append('nav OK')
        if result.torso_succeeded:
            phases.append(f'torso {result.torso_height:.3f}m')
        if result.arm_succeeded:
            phases.append('arm OK')
        result.message = ' | '.join(phases) if result.success else arm_msg

        fb.progress = 1.0
        fb.status = result.message
        goal_handle.publish_feedback(fb)

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.succeed()  # still SUCCEEDED status; success=False in result
        return result

    # ── Phase 1: Navigation ──────────────────────────────────────

    def _navigate_to_target(self, target_pose, standoff, goal_handle, fb, dry_run):
        """Navigate base to standoff position facing target. Returns (ok, msg)."""
        if self._nav_client is None:
            return False, 'Nav2 not available'

        # Transform target to map frame for navigation planning
        map_pose = self._transform_pose(target_pose, 'map', timeout=5.0)
        if map_pose is None:
            return False, f'TF to map failed (source: {target_pose.header.frame_id})'

        # Get current robot position in map
        robot_x, robot_y = self._get_robot_position_in_map()

        nav_goal_pose = compute_nav_goal(map_pose, robot_x, robot_y, standoff)
        gp = nav_goal_pose.pose.position
        self.logger.info(
            f'Nav goal: ({gp.x:.2f}, {gp.y:.2f}) facing target, '
            f'standoff={standoff:.2f}m')

        if dry_run:
            fb.status = f'[DRY RUN] Would navigate to ({gp.x:.2f}, {gp.y:.2f})'
            fb.progress = 0.5
            goal_handle.publish_feedback(fb)
            return True, 'dry run'

        # Wait for Nav2 server
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            return False, 'Nav2 server not available (timeout 10s)'

        # Send goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = nav_goal_pose

        fb.status = 'Navigating to target area...'
        fb.progress = 0.15
        goal_handle.publish_feedback(fb)

        send_future = self._nav_client.send_goal_async(nav_goal)
        if not self._poll_future(send_future, 10.0):
            return False, 'Nav2 send_goal timeout'

        nav_gh = send_future.result()
        if not nav_gh.accepted:
            return False, 'Nav2 goal rejected'

        # Wait for navigation result
        result_future = nav_gh.get_result_async()
        nav_timeout = self.cfg['navigation']['timeout_sec']
        if not self._poll_future(result_future, nav_timeout):
            # Try to cancel
            nav_gh.cancel_goal_async()
            return False, f'Navigation timeout ({nav_timeout}s)'

        nav_status = result_future.result().status
        # action_msgs GoalStatus: SUCCEEDED=4
        if nav_status == 4:
            fb.status = 'Navigation complete.'
            fb.progress = 0.5
            goal_handle.publish_feedback(fb)
            return True, 'succeeded'
        else:
            return False, f'Nav2 finished with status={nav_status}'

    # ── Phase 2: Torso ───────────────────────────────────────────

    def _adjust_torso(self, target_pose, goal_handle, fb, dry_run):
        """Compute optimal torso height and move directly. Returns (ok, height)."""
        # Transform target to base_footprint
        base_pose = self._transform_pose(target_pose, 'base_footprint', timeout=5.0)
        if base_pose is None:
            self.logger.warn('TF to base_footprint failed for torso calc.')
            return True, self.current_torso_height  # non-critical, continue

        target_z = base_pose.pose.position.z
        tcfg = self.cfg['torso']

        # Compute optimal height (same logic as TorsoHeightCalculator)
        h = target_z - tcfg['torso_base_z'] - tcfg['arm_workspace_target_z']
        optimal_h = max(tcfg['lower_limit'], min(tcfg['upper_limit'], h))

        self.logger.info(f'Torso: target_z={target_z:.3f}m → optimal={optimal_h:.3f}m')

        if dry_run:
            fb.status = f'[DRY RUN] Would set torso to {optimal_h:.3f}m'
            goal_handle.publish_feedback(fb)
            return True, optimal_h

        # Skip if already there
        if abs(optimal_h - self.current_torso_height) < 0.005:
            self.logger.info(f'Torso already at {optimal_h:.3f}m. Skipping.')
            return True, optimal_h

        # Send trajectory
        fb.status = f'Moving torso to {optimal_h:.3f}m...'
        fb.progress = 0.58
        goal_handle.publish_feedback(fb)

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = [tcfg['joint_name']]
        pt = JointTrajectoryPoint()
        pt.positions = [optimal_h]
        pt.velocities = [0.0]
        pt.accelerations = [0.0]
        dur = tcfg['trajectory_duration_sec']
        pt.time_from_start = Duration(
            sec=int(dur), nanosec=int((dur % 1) * 1e9))
        traj.points = [pt]
        self.torso_pub.publish(traj)

        fb.status = 'Waiting for torso...'
        fb.progress = 0.62
        goal_handle.publish_feedback(fb)

        time.sleep(tcfg['settle_wait_sec'])
        return True, optimal_h

    # ── Phase 3: Arm ─────────────────────────────────────────────

    def _reach_with_arm(self, target_pose, goal_handle, fb, dry_run):
        """Call /arm_reach action server. Returns (ok, message, ArmReach.Result|None)."""
        if self._arm_client is None:
            return False, 'arm_controller_msgs not available', None

        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            return False, 'ArmReach server not available', None

        arm_goal = ArmReach.Goal()
        arm_goal.target_pose = target_pose
        arm_goal.dry_run = dry_run
        arm_goal.timeout_sec = 5.0

        fb.status = 'Sending arm reach goal...'
        fb.progress = 0.75
        goal_handle.publish_feedback(fb)

        send_future = self._arm_client.send_goal_async(arm_goal)
        if not self._poll_future(send_future, 10.0):
            return False, 'ArmReach send_goal timeout', None

        arm_gh = send_future.result()
        if not arm_gh.accepted:
            return False, 'ArmReach goal rejected', None

        fb.status = 'Arm reaching target...'
        fb.progress = 0.8
        goal_handle.publish_feedback(fb)

        result_future = arm_gh.get_result_async()
        arm_timeout = self.cfg['arm']['timeout_sec']
        if not self._poll_future(result_future, arm_timeout):
            return False, f'ArmReach timeout ({arm_timeout}s)', None

        arm_result = result_future.result().result
        if arm_result.success:
            fb.status = 'Arm reached target.'
            fb.progress = 0.95
            goal_handle.publish_feedback(fb)
            return True, arm_result.message, arm_result
        else:
            return False, arm_result.message, arm_result

    # ── Utility methods ──────────────────────────────────────────

    def _resolve_target(self, request):
        """Resolve target from either target_frame or target_pose."""
        if request.target_frame:
            # Look up TF frame as a pose
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    'map', request.target_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=5.0))
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.header.stamp = tf_stamped.header.stamp
                ps.pose.position.x = tf_stamped.transform.translation.x
                ps.pose.position.y = tf_stamped.transform.translation.y
                ps.pose.position.z = tf_stamped.transform.translation.z
                ps.pose.orientation = tf_stamped.transform.rotation
                return ps
            except Exception as e:
                self.logger.error(f'TF lookup for "{request.target_frame}": {e}')
                return None
        elif request.target_pose.header.frame_id:
            return request.target_pose
        else:
            return None

    def _transform_pose(self, pose_stamped, target_frame, timeout=5.0):
        """Transform pose to target_frame with polling retry."""
        if pose_stamped.header.frame_id == target_frame:
            return pose_stamped
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                ps = PoseStamped()
                ps.header.frame_id = pose_stamped.header.frame_id
                ps.header.stamp = rclpy.time.Time().to_msg()
                ps.pose = pose_stamped.pose
                return self.tf_buffer.transform(ps, target_frame)
            except Exception:
                time.sleep(0.05)
        return None

    def _get_robot_position_in_map(self):
        """Get current base_footprint position in map frame via TF."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            self.logger.warn('Cannot get robot position in map. Using (0,0).')
            return 0.0, 0.0

    def _poll_future(self, future, timeout_sec):
        """Block until rclpy future completes."""
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.done()

    def _load_config(self, config_path):
        try:
            with open(config_path) as f:
                return yaml.safe_load(f)['reach_target']
        except Exception as e:
            self.get_logger().warn(f'Config load failed: {e}. Using defaults.')
            return {
                'default_standoff_distance': 0.55,
                'arm_x_offset': -0.062,
                'navigation': {'action_name': 'navigate_to_pose', 'timeout_sec': 120.0},
                'torso': {
                    'joint_name': 'torso_lift_joint',
                    'lower_limit': 0.0, 'upper_limit': 0.35,
                    'torso_base_z': 0.790, 'arm_workspace_target_z': 0.1,
                    'trajectory_duration_sec': 3.0, 'settle_wait_sec': 3.5,
                },
                'arm': {'action_name': 'arm_reach', 'timeout_sec': 30.0},
            }


def main():
    rclpy.init()
    node = ReachTargetServer()
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
