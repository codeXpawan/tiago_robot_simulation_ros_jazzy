"""
navigation_helper.py
--------------------
Computes a navigation goal that places the robot base so the arm can
reach the target.

Strategy:
    - The target is at (tx, ty) in the map frame.
    - The robot should stop ``standoff_distance`` metres before the target,
      facing it.
    - The orientation is set so the robot's x-axis (forward) points at
      the target.
"""

import math
from geometry_msgs.msg import PoseStamped, Quaternion


def compute_nav_goal(
    target_map: PoseStamped,
    robot_x: float,
    robot_y: float,
    standoff_distance: float = 0.55,
    map_width: float = 0,
    map_height: float = 0,
    map_pose_x: float = 0,
    map_pose_y: float = 0
) -> PoseStamped:
    """
    Compute a navigation goal in ``map`` frame.

    The robot will be positioned ``standoff_distance`` metres from the
    target on the line between the robot's current position and the
    target, facing the target.

    Args:
        target_map: target pose in ``map`` frame
        robot_x:    current robot x in map frame
        robot_y:    current robot y in map frame
        standoff_distance: how far to stop from target (m)

    Returns:
        Navigation goal PoseStamped in ``map`` frame.
    """
    tx = target_map.pose.position.x
    ty = target_map.pose.position.y

    # Direction from robot to target
    dx = tx - robot_x
    dy = ty - robot_y
    dist = math.hypot(dx, dy)

    if dist < 0.01:
        # Already at target — just face +x
        yaw = 0.0
    else:
        yaw = math.atan2(dy, dx)

    # Place nav goal on the line from robot to target, standoff metres back
    if dist > standoff_distance:
        if tx > 0:
            standoff_distance = standoff_distance
        else:
            standoff_distance = -standoff_distance
        gx = tx - standoff_distance * math.cos(yaw)
        if ty > 0:
            standoff_distance = standoff_distance
        else:
            standoff_distance = -standoff_distance 
        gy = ty - standoff_distance * math.sin(yaw)
        # Clamp the goal to the map boundaries
        gx, gy = get_clamped_goal(None, gx, gy, map_width, map_height, map_pose_x, map_pose_y)  
    else:
        # Already within standoff — stay put, just face the target
        gx = robot_x
        gy = robot_y

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = gx
    goal.pose.position.y = gy
    goal.pose.position.z = 0.0
    goal.pose.orientation = yaw_to_quaternion(yaw)
    return goal


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw (radians) to geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def get_clamped_goal(self, target_x, target_y, width_m, height_m, map_pose_x, map_pose_y): #to be used in reach_target_action_server.py to clamp the navigation goal to the map boundaries

    # Convert meters to pixel indices (u, v)
    u = target_x - map_pose_x
    v = target_y - map_pose_y
    
    # Clamp the pixel indices to the map boundaries
    # We use 0 to (limit - 1) because indices are zero-based
    clamped_u = max(0, min(u, width_m - 0.01))
    clamped_v = max(0, min(v, height_m - 0.01))

    # Convert back to meters (Map Frame coordinates)
    # Adding 0.5 * res places the point in the center of the pixel
    final_x = map_pose_x + (clamped_u)
    final_y = map_pose_y + (clamped_v)

    return final_x, final_y