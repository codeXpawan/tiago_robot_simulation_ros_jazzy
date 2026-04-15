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

    torso_adjust_server = Node(
        package='torso_controller',
        executable='torso_adjust_server',
        name='torso_adjust_server',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        torso_adjust_server,
    ])
