from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )

    arm_reach_server = Node(
        package='arm_controller',
        executable='arm_reach_server',
        name='arm_reach_server',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        arm_reach_server,
    ])
