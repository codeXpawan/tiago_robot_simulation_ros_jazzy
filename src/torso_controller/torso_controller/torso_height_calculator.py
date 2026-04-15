"""
torso_height_calculator.py
--------------------------
Computes the optimal torso_lift_joint position to maximise arm reachability
for a given 3D target.

TIAGo torso kinematics (pmb2 base):
    base_footprint
      └─ base_link
           └─ torso_fixed_link   z-offset = 0.193 m  (fixed)
                └─ torso_lift_link  z-offset = 0.597 + torso_lift_joint  (prismatic)

At torso_lift_joint = h:
    height of torso_lift_link above base_footprint = 0.790 + h  (= torso_base_z + h)

The arm IK solver works in torso_lift_link frame.
Arm workspace Z span (in torso_lift_link frame): approximately [-0.5, +0.9] m.

Strategy:
    Given target Z (in base_footprint), choose h so the target lands at
    arm_workspace_target_z in the torso frame:

        target_z_in_torso = target_z_base - (torso_base_z + h)
        arm_workspace_target_z = target_z_base - torso_base_z - h
        h = target_z_base - torso_base_z - arm_workspace_target_z

    Result is clamped to [lower_limit, upper_limit].
"""

from dataclasses import dataclass


@dataclass
class TorsoConfig:
    joint_name: str = "torso_lift_joint"
    lower_limit: float = 0.0
    upper_limit: float = 0.35
    torso_base_z: float = 0.790          # height of torso_lift_link at joint=0, in base_footprint
    arm_workspace_target_z: float = 0.1  # desired target z in torso_lift_link frame


class TorsoHeightCalculator:
    """
    Computes the optimal torso_lift_joint value for a given target pose.
    """

    def __init__(self, config: TorsoConfig):
        self.config = config

    def compute(self, target_z_base: float) -> float:
        """
        Compute the optimal torso_lift_joint value (metres) so the target
        lands at ``arm_workspace_target_z`` in the torso frame.

        Args:
            target_z_base: target Z coordinate in base_footprint frame (metres)

        Returns:
            Clamped torso_lift_joint value in [lower_limit, upper_limit]
        """
        c = self.config
        h = target_z_base - c.torso_base_z - c.arm_workspace_target_z
        return max(c.lower_limit, min(c.upper_limit, h))

    def target_z_in_torso(self, target_z_base: float, torso_height: float) -> float:
        """Return what the target z would be in torso_lift_link frame at a given torso height."""
        return target_z_base - (self.config.torso_base_z + torso_height)

    def height_description(self, h: float) -> str:
        """Human-readable description of a torso height."""
        c = self.config
        pct = (h - c.lower_limit) / (c.upper_limit - c.lower_limit) * 100.0
        return f"{h:.3f} m ({pct:.0f}% of range)"
