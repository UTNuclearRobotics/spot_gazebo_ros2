import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace, SetRemap


def launch_amcl_if_map_present(context, *args, **kwargs):
    """
    Inspect 'map_dir' at launch time. If it points to a directory that
    contains at least one .yaml file and one .png file, launch AMCL
    against that map using the spot_bringup_sim/config/spot.yaml params.
    Otherwise, launch nothing.
    """
    map_dir = LaunchConfiguration('map_dir').perform(context)

    if not map_dir or not os.path.isdir(map_dir):
        return []

    yaml_files = [f for f in os.listdir(map_dir) if f.endswith('.yaml')]
    png_files = [f for f in os.listdir(map_dir) if f.endswith('.png')]

    if not yaml_files or not png_files:
        return []

    map_yaml_path = os.path.join(map_dir, yaml_files[0])

    amcl_config = PathJoinSubstitution(
        [FindPackageShare('spot_bringup_sim'), 'config', 'spot.yaml']
    )

    amcl_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('spot_bringup_sim'), 'launch', 'amcl.launch.py'])
        ),
        launch_arguments={
            'config': amcl_config,
            'map': map_yaml_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cloud_in': LaunchConfiguration('cloud_in'),
        }.items()
    )

    return [amcl_include]


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config',
        description='Full path to the navigation configuration file',
        default_value=PathJoinSubstitution([FindPackageShare("spot_bringup_sim"), "config", "spot.yaml"])
    )

    pointcloud_arg = DeclareLaunchArgument(
        'cloud_in',
        description='The topic on which to look for pointcloud data. Default "/velodyne_points"',
        default_value='/velodyne_points'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='If true, launch slam_toolbox for live mapping'
    )

    map_dir_arg = DeclareLaunchArgument(
        'map_dir',
        default_value='',
        description='Directory containing a pre-built map (.yaml + .png pair) to localize against. '
                     'If empty, or no matching pair is found in it, AMCL is not launched.'
    )

    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('spot_bringup_sim'), 'launch', 'slam.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('slam')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cloud_in': LaunchConfiguration('cloud_in'),
            'params_file': PathJoinSubstitution(
                [FindPackageShare('spot_bringup_sim'), 'config', 'lidar_slam.yaml']
            ),
        }.items()
    )

    amcl_from_map_dir = OpaqueFunction(function=launch_amcl_if_map_present)

    nav_include = GroupAction(
        actions=[
            PushRosNamespace("spot_nav"),
            SetRemap(src='cmd_vel'   , dst='/nav_stack/cmd_vel'),
            SetRemap(src='/tf'       , dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            SetRemap(src='/velodyne_points', dst=LaunchConfiguration('cloud_in')),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
                ),
                launch_arguments={
                    "params_file": LaunchConfiguration('config')
                }.items()
            )
        ]
    )

    return LaunchDescription([
        config_arg,
        pointcloud_arg,
        use_sim_time_arg,
        slam_arg,
        map_dir_arg,
        slam_include,
        amcl_from_map_dir,
        nav_include,
    ])