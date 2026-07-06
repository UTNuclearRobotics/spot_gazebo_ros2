import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    world_file = LaunchConfiguration('world_file', default='server_room.sdf')
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='server_room.sdf',
        description='Name of the world file to load'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false', 
        description='Open RViz.'
    )

    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value='spot.rviz',
        description='RViz configuration file to use'
    )

    # Setup to launch the simulator and Gazebo world
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_spot_gazebo = get_package_share_directory('spot_gazebo')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': [
                    PathJoinSubstitution([
                        pkg_spot_gazebo, 
                        'worlds',
                        world_file
                    ]),
                ],
            }.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    pkg_spot_bringup = get_package_share_directory('spot_bringup_sim')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': os.path.join(pkg_spot_bringup, 'config', 'spot_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }]
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    pkg_spot_description = get_package_share_directory('spot_description_sim')
    urdf_file = os.path.join(pkg_spot_description, 'models', 'spot', 'model.urdf')
    with open(urdf_file, 'r') as infp: robot_desc = infp.read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
            {"publish_frequency": 200.0},
        ],
        remappings=[
            ('/joint_states', '/spot/joint_states')
        ]
    )

    # contact_adapter = Node(
    #     package='spot_bringup_sim',
    #     executable='contact_adapter_node',
    #     name='contact_adapter',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}],
    # )

    # Controller
    config_path = get_package_share_directory("champ_config")
    links_config = PathJoinSubstitution([config_path, 'config', 'links', 'links.yaml'])
    links_param = ParameterFile(param_file=links_config, allow_substs=True)

    joints_config = PathJoinSubstitution([config_path, 'config', 'joints', 'joints.yaml'])
    joints_param = ParameterFile(param_file=joints_config, allow_substs=True) 

    gait_config = PathJoinSubstitution([config_path, 'config', 'gait', 'gait.yaml'])
    gait_param = ParameterFile(param_file=gait_config, allow_substs=True) 

    urdf_file = os.path.join(pkg_spot_description, 'models', 'spot', 'model.urdf')

    # EKF
    ekf_config_path = get_package_share_directory("champ_base")

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"gazebo": True},
            {"publish_joint_states": False},
            {"publish_foot_contacts": True},
            {"publish_joint_control": True},
            {"joint_controller_topic": "/spot/joint_trajectory"},
            {"urdf": urdf_file},
            links_param,
            joints_param,
            gait_param
        ],
        remappings=[
            ("/cmd_vel/smooth", "/spot_driver/cmd_vel"),
        ],
    )

    # Visualize in RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            pkg_spot_bringup,
            'config',
            LaunchConfiguration('rviz_config_file')
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}]
    )

    # Filters the robot body out of the bridged lidar points (already in velodyne frame)
    pointcloud_filter_container = ComposableNodeContainer(
        name='spot_pointcloud_filter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                name='spot_pointcloud_filter_component',
                package='spot_navigation',
                plugin='spot_navigation::PointcloudFilterComponent',
                remappings=[
                    ('cloud_in', 'unfiltered_velodyne_points'),
                    ('cloud_out', 'velodyne_points'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[{'use_sim_time': True}],
            ),
        ],
        output='screen',
    )

    with open(urdf_file, 'r') as infp:
        urdf_content = infp.read()

    state_estimator = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"gazebo": True},
            {"urdf": urdf_content},   # <-- file CONTENT, not the path
            links_param,
            joints_param,
            gait_param,
        ],
        remappings=[
            ('/joint_states', '/spot/joint_states'),
            ("/cmd_vel/smooth", "/spot_driver/cmd_vel"),
        ],
    )

    base_to_footprint_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='base_to_footprint_ekf',
        output='screen',
        parameters=[
            os.path.join(ekf_config_path, 'config/ekf', 'base_to_footprint.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered/local'),
        ],
    )

    footprint_to_odom_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='footprint_to_odom_ekf',
        output='screen',
        parameters=[
            os.path.join(ekf_config_path, 'config/ekf', 'footprint_to_odom.yaml'),
            {'use_sim_time': True},
        ],
    )

    sim_manipulation_driver = Node(
        package='spot_bringup_sim',
        executable='sim_manipulation_driver',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    stow_arm = Node(
        package='spot_bringup_sim',
        executable='stow_arm_client',
        output='screen',
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            os.path.join(pkg_spot_bringup, 'config', 'twist_mux.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[
            ('cmd_vel_out', '/spot_driver/cmd_vel'),
        ],
    )

    return LaunchDescription([
        world_file_arg,
        rviz_arg,
        rviz_config_file_arg,
        gz_sim,
        bridge,
        robot_state_publisher,
        quadruped_controller_node,
        pointcloud_filter_container,
        state_estimator,
        base_to_footprint_ekf,
        footprint_to_odom_ekf,
        sim_manipulation_driver,
        stow_arm,
        twist_mux,
        rviz,
    ])