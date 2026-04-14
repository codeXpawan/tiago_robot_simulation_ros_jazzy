import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch_ros.descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
import os
import subprocess

def generate_launch_description():
    use_rviz = LaunchConfiguration('rviz', default=True)
    use_nav = LaunchConfiguration('nav', default=True)
    use_slam_cartographer = LaunchConfiguration('slam_cartographer', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    package_dir  = get_package_share_directory('tiago_robot_simulation')

    xacro_file     = os.path.join(package_dir, 'robots', 'tiago.urdf.xacro')
    resolve_script = os.path.join(package_dir, 'scripts', 'resolve_urdf.py')
    tmp_urdf       = '/tmp/tiago_resolved.urdf'

    # ── Step 1: Run xacro and get the URDF string ─────────────────────────────
    print(f'[launch] Running xacro on {xacro_file} ...')
    xacro_result = subprocess.run(
        ['xacro', xacro_file, 'gazebo_version:=gazebo', 'use_sim_time:=true'],
        capture_output=True, text=True
    )
    if xacro_result.returncode != 0:
        raise RuntimeError(f'xacro failed:\n{xacro_result.stderr}')
    print('[launch] xacro done.')

    # ── Step 2: Run resolve_urdf.py on the xacro output ──────────────────────
    print(f'[launch] Resolving package:// URIs ...')
    resolve_result = subprocess.run(
        ['python3', resolve_script],
        input=xacro_result.stdout,
        capture_output=True, text=True
    )
    if resolve_result.returncode != 0:
        raise RuntimeError(f'resolve_urdf.py failed:\n{resolve_result.stderr}')
    print('[launch] Resolution done.')

    # ── Step 3: Write resolved URDF to a temp file ────────────────────────────
    with open(tmp_urdf, 'w') as f:
        f.write(resolve_result.stdout)
    print(f'[launch] Resolved URDF written to {tmp_urdf}')

    resolved_urdf_str = resolve_result.stdout

    # ── Robot description for robot_state_publisher ───────────────────────────
    from launch_ros.descriptions import ParameterValue
    robot_description = ParameterValue(
        resolved_urdf_str,
        value_type=str
    )

    # ── Gazebo ────────────────────────────────────────────────────────────────
    world_path = os.path.join(package_dir,'worlds','test.sdf')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -v4 {world_path}'
        }.items()
    )

    # ── Spawner — uses the resolved temp URDF file directly ───────────────────
    urdf_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tiago',
            '-allow_renaming', 'true',
            '-file', tmp_urdf,          # <-- file path, no Command() needed
            '-x', '2', '-y', '1', '-z', '0.0',  # Adjusted Z for correct ground contact
        ],
        output='screen'
    )

    # ── Controllers ───────────────────────────────────────────────────────────
    robot_controllers = os.path.join(package_dir, 'config', 'controllers.yaml')
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file', robot_controllers,  # ← pass controllers.yaml
        ],
    )
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diffdrive_controller',
            '--param-file', robot_controllers,
            '--controller-ros-args', 
            '-r /diffdrive_controller/odom:=/odom',
        ],
    )
 
    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller,
                '--param-file', robot_controllers,  # ← pass controllers.yaml
            ],
        )
        for controller in [
            # 'diffdrive_controller',
            'arm_controller',
            'torso_controller',
            'head_controller',
        ]
    ]

    # ── Bridge ────────────────────────────────────────────────────────────────
    bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        # Laser
        '/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        # IMU
        '/base_imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        # Sonars
        '/sonar_base_01@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/sonar_base_02@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/sonar_base_03@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        # Camera
        '/head_front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/head_front_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/head_front_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        '/head_front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        #tf
        # '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        #joint states
        '/world/rectangular_walls_world/model/tiago/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
        # odometry
        # '/ground_truth_odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
    ],
    remappings=[
        ('/world/rectangular_walls_world/model/tiago/joint_state', '/joint_states'),
        ('/base_imu', '/imu'),
        ('/scan_raw', '/scan'),
        ('/head_front_camera/points', '/points2'),
    ],
    output='screen'
)

    # ── Robot state publisher ─────────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
            'use_sim_time': use_sim_time,
            'tf_prefix': 'tiago',
        }],
        output='screen'
    )

    rviz_config = os.path.join(get_package_share_directory('webots_ros2_tiago'), 'resource', 'default_bringup.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz)
    )

        # Navigation
    navigation_nodes = []
    nav2_params_file = 'nav2_params.yaml'
    nav2_params = os.path.join(get_package_share_directory('webots_ros2_tiago'), 'resource', nav2_params_file)
    nav2_map = os.path.join(get_package_share_directory('webots_ros2_tiago'), 'resource', 'map.yaml')
    # if 'nav2_bringup' in get_packages_with_prefixes():
    #     navigation_nodes.append(IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(
    #             get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
    #         launch_arguments=[
    #             ('map', nav2_map),
    #             ('params_file', nav2_params),
    #             ('use_sim_time', use_sim_time),
    #         ],
    #         condition=launch.conditions.IfCondition(use_nav)))

    # SLAM
    cartographer_config_dir = os.path.join(package_dir, 'config')
    cartographer_config_basename = 'cartographer.lua'
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', cartographer_config_basename],
        # remappings=[('/scan', '/scan_raw')],
        condition=launch.conditions.IfCondition(use_slam_cartographer))
    navigation_nodes.append(cartographer)

    grid_executable = 'cartographer_occupancy_grid_node'
    cartographer_grid = Node(
        package='cartographer_ros',
        executable=grid_executable,
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05'],
        condition=launch.conditions.IfCondition(use_slam_cartographer))
    navigation_nodes.append(cartographer_grid)


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
            description='Use simulated clock'),
        DeclareLaunchArgument('description_format', default_value='urdf',
            description='Robot description format: urdf or sdf'),
        DeclareLaunchArgument('walk', default_value='true',
            description='Enable walker node for healthy leg'),
        DeclareLaunchArgument('control', default_value='true',
            description='Enable controller node for prosthetic leg'),

        robot_state_publisher_node,
        bridge,
        gz_launch,
        urdf_spawner,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=urdf_spawner,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=controller_spawners,
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_spawners[-1],  # wait for the last controller to spawn
                on_exit=[diffdrive_controller_spawner] + navigation_nodes,  
            )
        ),
        
    ])