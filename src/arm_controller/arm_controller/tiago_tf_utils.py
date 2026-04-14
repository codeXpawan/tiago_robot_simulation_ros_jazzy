"""
tiago_tf_utils.py
-----------------
TF2 utilities for transforming poses between TIAGo coordinate frames.

Frame chain on TIAGo:
    base_footprint → base_link → torso_fixed_link → torso_lift_link → arm_*

The IK solver works in torso_lift_link frame, so any target pose
(e.g. from a camera in xtion_rgb_optical_frame, or world frame)
must be transformed here first.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs   # registers PoseStamped transform support
from typing import Optional
import math


class TiagoTFUtils:
    """
    Wraps tf2_ros lookupTransform + transformPose for easy frame conversions.
    Attach to any ROS 2 node.
    """

    def __init__(self, node: Node):
        self.node = node
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self.logger = node.get_logger()

    def transform_pose(
        self,
        pose_stamped: PoseStamped,
        target_frame: str,
        timeout_sec: float = 2.0,
    ) -> Optional[PoseStamped]:
        """
        Transform a PoseStamped from its current frame into target_frame.

        Args:
            pose_stamped: input pose with frame_id set
            target_frame: destination frame (e.g. 'torso_lift_link')
            timeout_sec: how long to wait for TF data

        Returns:
            Transformed PoseStamped, or None on failure
        """
        # Build a Time(0) stamped pose — tf2 treats stamp=0 as "latest available".
        pose_latest = PoseStamped()
        pose_latest.header.frame_id = pose_stamped.header.frame_id
        pose_latest.header.stamp = rclpy.time.Time().to_msg()
        pose_latest.pose = pose_stamped.pose

        try:
            # No timeout: fail immediately if data is not yet in the buffer.
            # Callers that need to retry should do so with await asyncio.sleep()
            # between attempts so the asyncio event loop can process TF callbacks.
            return self.tf_buffer.transform(pose_latest, target_frame)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.logger.debug(
                f"TF not yet available ({type(e).__name__}): "
                f"{pose_stamped.header.frame_id} → {target_frame}: {e}"
            )
            return None

    def get_transform(
        self,
        source_frame: str,
        target_frame: str,
        timeout_sec: float = 2.0,
    ) -> Optional[TransformStamped]:
        """Get raw TransformStamped between two frames."""
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
        except Exception as e:
            self.logger.error(f"TF lookup failed {source_frame}→{target_frame}: {e}")
            return None

    def torso_to_base(self, pose_stamped: PoseStamped) -> Optional[PoseStamped]:
        """Convert pose from torso_lift_link → base_footprint (for display/nav)."""
        return self.transform_pose(pose_stamped, "base_footprint")

    def base_to_torso(self, pose_stamped: PoseStamped) -> Optional[PoseStamped]:
        """Convert pose from base_footprint → torso_lift_link (for IK)."""
        return self.transform_pose(pose_stamped, "torso_lift_link")

    def any_frame_to_torso(self, pose_stamped: PoseStamped) -> Optional[PoseStamped]:
        """
        Transform any input pose into torso_lift_link frame for IK solving.
        This is the main entry point when you receive a 3D object pose
        from perception (e.g. in 'map', 'base_footprint', or camera frame).
        """
        src = pose_stamped.header.frame_id
        if src == "torso_lift_link":
            return pose_stamped  # already in the right frame

        self.logger.info(f"Transforming pose from '{src}' → 'torso_lift_link'")
        result = self.transform_pose(pose_stamped, "torso_lift_link")

        if result is None:
            self.logger.error(
                f"Could not transform from '{src}' to 'torso_lift_link'. "
                "Is the TF tree complete? Check: ros2 run tf2_tools view_frames"
            )
        return result

    def check_tf_tree(self) -> bool:
        """
        Quick sanity check that critical TIAGo frames are available.
        Logs warnings for any missing links.
        """
        critical_frames = [
            ("base_footprint", "torso_lift_link"),
            ("torso_lift_link", "arm_tool_link"),
        ]
        all_ok = True
        for src, tgt in critical_frames:
            tf = self.get_transform(src, tgt, timeout_sec=1.0)
            if tf is None:
                self.logger.warn(f"TF not available: {src} → {tgt}")
                all_ok = False
            else:
                self.logger.info(f"TF OK: {src} → {tgt}")
        return all_ok

    @staticmethod
    def euler_to_pose_stamped(
        x: float, y: float, z: float,
        roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
        frame_id: str = "torso_lift_link",
    ) -> PoseStamped:
        """
        Convenience: build a PoseStamped from x,y,z + roll,pitch,yaw (radians).
        Useful for testing without perception pipeline.
        """
        import math
        from geometry_msgs.msg import Pose, Point, Quaternion
        from std_msgs.msg import Header

        # Roll-Pitch-Yaw → quaternion
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps
