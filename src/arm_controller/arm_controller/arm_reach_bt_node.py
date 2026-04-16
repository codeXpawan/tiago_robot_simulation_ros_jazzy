"""
arm_reach_bt_node.py
--------------------
Nav2 Behavior Tree plugin: calls the ArmReach action server as a BT action node.

This makes the arm reach capability available inside your Nav2 BT XML trees,
so you can sequence: Navigate → ArmReach → Grasp as a single BT.

BT XML usage:
    <ArmReachAction name="reach_bottle"
                    target_pose="{object_pose}"
                    dry_run="false"
                    server_name="arm_reach"
                    server_timeout="10"/>

Register in your BT navigator config (nav2_params.yaml):
    bt_navigator:
      plugin_lib_names:
        - nav2_compute_path_to_pose_action_bt_node
        - nav2_follow_path_action_bt_node
        - tiago_arm_kinematics_arm_reach_bt_node   # ← add this

Architecture:
    Nav2 BT Navigator → BT XML → ArmReachBTNode → /arm_reach action → TIAGo arm

Note: This requires nav2_behavior_tree and BehaviorTree.CPP.
Both are installed with ros-jazzy-nav2-bringup.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

# Nav2 BT node base class
try:
    from nav2_behavior_tree import RosActionNode
    from behaviortree_ros2 import RosNodeParams
    BT_AVAILABLE = True
except ImportError:
    BT_AVAILABLE = False

try:
    from arm_controller_msgs.action import ArmReach
    ACTION_AVAILABLE = True
except ImportError:
    ACTION_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────
# Behavior Tree Node (Nav2 plugin style)
# ─────────────────────────────────────────────────────────────────

if BT_AVAILABLE and ACTION_AVAILABLE:
    import py_trees

    class ArmReachBTNode(RosActionNode):
        """
        Nav2 Behavior Tree Action Node wrapping the ArmReach action server.

        Input ports:
            target_pose  → geometry_msgs/PoseStamped  (from BT blackboard)
            dry_run      → bool  (default false)

        Output ports:
            joint_angles → list of floats
            is_reachable → bool
            ik_message   → string
        """

        @staticmethod
        def providedPorts():
            return [
                py_trees.common.BlackboardClient.create_key("target_pose", read=True),
                py_trees.common.BlackboardClient.create_key("dry_run", read=True),
                py_trees.common.BlackboardClient.create_key("joint_angles", write=True),
                py_trees.common.BlackboardClient.create_key("is_reachable", write=True),
                py_trees.common.BlackboardClient.create_key("ik_message", write=True),
            ]

        def on_tick(self):
            """Called each BT tick to set up the goal."""
            pose = self.getInput("target_pose")
            dry_run = self.getInput("dry_run") or False

            if pose is None:
                self.node.get_logger().error(
                    "ArmReachBTNode: 'target_pose' blackboard key is empty!"
                )
                return

            self.goal_msg = ArmReach.Goal()
            self.goal_msg.target_pose = pose
            self.goal_msg.dry_run = dry_run
            self.goal_msg.timeout_sec = 3.0

        def on_success(self):
            """Write results back to BT blackboard."""
            result = self.result_msg
            self.setOutput("joint_angles", list(result.joint_angles))
            self.setOutput("is_reachable", result.is_reachable)
            self.setOutput("ik_message", result.message)

            if result.success:
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().warn(
                    f"ArmReachBTNode: IK failed — {result.message}"
                )
                return py_trees.common.Status.FAILURE


# ─────────────────────────────────────────────────────────────────
# Simple BT demo (no Nav2 required) using py_trees directly
# ─────────────────────────────────────────────────────────────────

class SimpleArmReachBehavior:
    """
    A minimal py_trees behavior that wraps the ArmReach action server.
    Use this if you want BT integration WITHOUT the full Nav2 BT navigator.

    Example BT tree:
        Sequence:
          ├── NavigateToPose (nav2)
          ├── SimpleArmReachBehavior  ← this
          └── GripperClose (your gripper node)
    """

    def __init__(self, node: Node, target_pose: PoseStamped, dry_run: bool = False):
        self.node = node
        self.target_pose = target_pose
        self.dry_run = dry_run
        self._result = None
        self._done = False

        if ACTION_AVAILABLE:
            self._client = ActionClient(node, ArmReach, "arm_reach")
        else:
            node.get_logger().error(
                "ArmReach action not available. Build the package first."
            )

    def execute(self) -> bool:
        """
        Blocking execute. Sends goal and waits for result.
        Returns True on success.
        """
        if not ACTION_AVAILABLE:
            return False

        self.node.get_logger().info("SimpleArmReachBehavior: waiting for server...")
        self._client.wait_for_server()

        goal = ArmReach.Goal()
        goal.target_pose = self.target_pose
        goal.dry_run = self.dry_run
        goal.timeout_sec = 5.0

        self.node.get_logger().info("SimpleArmReachBehavior: sending goal...")
        future = self._client.send_goal_async(goal)

        # Spin until goal is accepted
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.node.get_logger().error("Goal rejected by arm_reach server")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result

        self.node.get_logger().info(
            f"ArmReach result: success={result.success}, "
            f"reachable={result.is_reachable}, msg={result.message}"
        )
        return result.success


# ─────────────────────────────────────────────────────────────────
# Example BT tree builder (for reference)
# ─────────────────────────────────────────────────────────────────

BT_XML_EXAMPLE = """
<!--
  Example Nav2 BT XML using ArmReachAction node.
  Place in your bt_navigator config as the default_bt_xml_filename.
-->
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="navigate_and_grasp">

      <!-- 1. Navigate to object location -->
      <NavigateToPose name="go_to_table"
                      goal="{navigation_goal}"
                      server_name="navigate_to_pose"
                      server_timeout="30"/>

      <!-- 2. Check and reach with arm (dry_run first to validate) -->
      <ArmReachAction name="check_reachability"
                      target_pose="{object_pose}"
                      dry_run="true"
                      server_name="arm_reach"
                      server_timeout="10"/>

      <!-- 3. Actually move the arm -->
      <ArmReachAction name="reach_for_object"
                      target_pose="{object_pose}"
                      dry_run="false"
                      server_name="arm_reach"
                      server_timeout="15"/>

      <!-- 4. Close gripper (add your gripper action here) -->
      <!-- <GripperAction name="close_gripper" command="close"/> -->

    </Sequence>
  </BehaviorTree>
</root>
"""
