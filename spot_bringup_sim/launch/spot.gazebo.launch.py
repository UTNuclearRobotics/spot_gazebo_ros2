import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.descriptions import ComposableNode
from launch.event_handlers import OnProcessStart

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
        default_value='base_planner.rviz',
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
                    PathJoinSubstitution([pkg_spot_gazebo, 'worlds', world_file]),
                    # ' -s -r --headless-rendering',
                ],
            }.items(),
    )

    # Dedicated /clock bridge: isolated from the sensor bridge below so clock
    # messages never queue behind pointcloud/image serialization. Keeping them
    # in one process caused sim-time stutter that scaled with node count.
    # No use_sim_time here — this process is the *source* of ROS time.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    # Dedicated control/state bridge: joint commands out, joint states and
    # odometry back — the robot's control loop. Same isolation rationale as
    # the clock bridge: sharing a process with the pointcloud/image bridge
    # made these small high-rate messages arrive in bursts behind sensor
    # serialization, stuttering the legs and desyncing odom from joint states.
    state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='state_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': os.path.join(
                get_package_share_directory('spot_bringup_sim'),
                'config', 'spot_state_bridge.yaml'),
        }],
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
            'qos_overrides./unfiltered_velodyne_points.publisher.reliability': 'best_effort',
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
            # Matches the gz JointStatePublisher update_rate (model.sdf.xacro);
            # RSP just re-broadcasts joint states as TF.
            {"publish_frequency": 100.0},
        ],
        remappings=[
            ('/joint_states', '/spot_driver/joint_states'),
            ('robot_description', '/spot_driver/robot_description')
        ]
    )

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
            {"loop_rate": 100.0},
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
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/robot_description', '/spot_driver/robot_description'),
            ('/robot_description_semantic', '/spot_moveit/robot_description_semantic'),
        ],
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
            {"urdf": urdf_content},
            links_param,
            joints_param,
            gait_param,
        ],
        remappings=[
            ('/joint_states', '/spot_driver/joint_states'),
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

    sim_spot_driver = Node(
        package='spot_bringup_sim',
        executable='sim_spot_driver',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    sim_manipulation_driver = Node(
        package='spot_bringup_sim',
        executable='sim_manipulation_driver',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'action_namespace': 'spot_moveit',
        }],
    )

    # Depth-image -> pointcloud for the hand ToF camera, same pipeline as
    # hardware (spot_driver / spot_manipulation_driver launch).
    hand_camera_pointclouds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('spot_bringup'),
                         'launch', 'camera_pointclouds.launch.py')),
        launch_arguments={
            'hand_tof': 'True',
            'texture': 'False',
        }.items(),
    )

    # Attach the pointcloud mux to the depth_image_proc container
    pointcloud_mux = LoadComposableNodes(
        target_container='depth_image_proc_container',
        composable_node_descriptions=[
            ComposableNode(
                package='alpha_survey_3d',
                plugin='alpha_survey_3d::pointcloud_util::PointcloudMux',
                name='pointcloud_mux',
                parameters=[
                    {'mux_pointcloud_topics': ['/spot_pointclouds/hand_tof_points']},
                    {'use_sim_time': True},
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
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
            ('cmd_vel_out', '/spot_driver/cmd_vel_raw'),
        ],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            os.path.join(ekf_config_path, 'config/velocity_smoother',
                         'velocity_smoother.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[
            ('cmd_vel', '/spot_driver/cmd_vel_raw'),
            ('cmd_vel_smoothed', '/spot_driver/cmd_vel'),
        ],
    )

    velocity_smoother_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_velocity_smoother',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['velocity_smoother'],
        }],
    )

    # Filters the robot body out of the bridged lidar points (already in velodyne frame)
    pointcloud_filter_container = ComposableNodeContainer(
        name='spot_pointcloud_filter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
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

    pointcloud_filter_after_bridge = RegisterEventHandler(
        OnProcessStart(
            target_action=bridge,
            on_start=[pointcloud_filter_container],
        )
    )

    return LaunchDescription([
        world_file_arg,
        rviz_arg,
        rviz_config_file_arg,
        gz_sim,
        clock_bridge,
        state_bridge,
        bridge,
        robot_state_publisher,
        quadruped_controller_node,
        state_estimator,
        base_to_footprint_ekf,
        footprint_to_odom_ekf,
        sim_spot_driver,
        sim_manipulation_driver,
        hand_camera_pointclouds,
        pointcloud_mux,
        stow_arm,
        twist_mux,
        velocity_smoother,
        velocity_smoother_lifecycle,
        pointcloud_filter_after_bridge,
        rviz,
    ])