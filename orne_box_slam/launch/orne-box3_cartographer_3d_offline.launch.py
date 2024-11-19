import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    cartographer_prefix = get_package_share_directory('orne_box_slam')
    cartographer_config_dir_path = os.path.join(cartographer_prefix, 'config', 'cartographer')
    configuration_basename = LaunchConfiguration('configuration_basename', default='orne-box3_3d.lua')
    resolution = LaunchConfiguration('resolution', default='0.1')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    rviz_config_dir = os.path.join(cartographer_prefix, 'config', 'rviz', 'cartographer.rviz')
    launch_include_file_dir = os.path.join(get_package_share_directory('orne_box_bringup'), 'launch/include')
    rosbag_path = LaunchConfiguration('rosbag_path', default='/home/ryusei22/rosbag/241026_tsukuba_all2')

    return LaunchDescription([
        DeclareLaunchArgument('cartographer_config_dir', default_value=cartographer_config_dir_path, description='Full path to config file to load'),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename, description='Name of lua file for cartographer'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('resolution', default_value=resolution, description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec, description='OccupancyGrid publishing period'),
        DeclareLaunchArgument('rosbag_path', default_value=rosbag_path, description='Path to the rosbag file for playback'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir_path, '-configuration_basename', configuration_basename],
            remappings=[('/points2', '/surestar_points'), ('/imu', '/imu/data')],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/offline_node.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec,
                'cartographer_config_dir': cartographer_config_dir_path,
                'configuration_basename': configuration_basename
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_include_file_dir, '/description.launch.py']),
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', rosbag_path, '--clock'],
            name='rosbag_play',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        )
    ])
