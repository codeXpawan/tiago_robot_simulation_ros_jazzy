from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock')

    sim_time = LaunchConfiguration('use_sim_time')

    # Torso adjust server
    torso = Node(
        package='torso_controller',
        executable='torso_adjust_server',
        name='torso_adjust_server',
        parameters=[{'use_sim_time': sim_time}],
        output='screen')

    # Arm reach server
    arm = Node(
        package='arm_controller',
        executable='arm_reach_server',
        name='arm_reach_server',
        parameters=[{'use_sim_time': sim_time}],
        output='screen')

    # High-level reach-target orchestrator
    reach = Node(
        package='tiago_controller',
        executable='reach_target_server',
        name='reach_target_server',
        parameters=[{'use_sim_time': sim_time}],
        output='screen')

    return LaunchDescription([
        use_sim_time_arg,
        torso,
        arm,
        reach,
    ])
