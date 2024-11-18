# $ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/username/tsukuba.pbstream'}"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    # Launch arguments
    configuration_directory_arg = DeclareLaunchArgument(
        'configuration_directory', 
        default_value=os.path.join(
            FindPackageShare('orne_box_slam').find('orne_box_slam'), 'config/cartographer'))
    config_file_arg = DeclareLaunchArgument('config_file', default_value='assets_writer_orne-box_3d.lua')
    urdf_filename_arg = DeclareLaunchArgument(
        'urdf_filename', 
        default_value=os.path.join(
            FindPackageShare('orne_box_description').find('orne_box_description'), 'urdf/orne_box_lidar_with_mirror.urdf'))
    bag_filenames_arg = DeclareLaunchArgument('bag_filenames', default_value='/home/username/rosbag_dirname')
    pose_graph_filename_arg = DeclareLaunchArgument('pose_graph_filename', default_value='/home/username/rosbag.pbstream')

    ## ***** Nodes *****
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_assets_writer',
        parameters = [{'use_sim_time': False}],
        arguments = [
            '-configuration_directory', LaunchConfiguration('configuration_directory'),
            '-configuration_basename', LaunchConfiguration('config_file'),
            '-urdf_filename', LaunchConfiguration('urdf_filename'),
            '-bag_filenames', LaunchConfiguration('bag_filenames'),
            '-pose_graph_filename', LaunchConfiguration('pose_graph_filename')
            ],
        remappings = [
            ('/surestar_points', 'points2'),
            ('/imu/data', 'imu')
            ],
        output = 'screen'
        )

    return LaunchDescription([
        configuration_directory_arg,
        config_file_arg,
        urdf_filename_arg,
        bag_filenames_arg,
        pose_graph_filename_arg,
        # Nodes
        cartographer_node
    ])
