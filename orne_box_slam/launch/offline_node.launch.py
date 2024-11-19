from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import Shutdown

def generate_launch_description():

    # Launch arguments
    bag_filenames_arg = DeclareLaunchArgument(
        'bag_filenames',
        default_value='/path/to/your/bag/file',
        description='Path to the bag file for offline processing'
    )
    configuration_directory_arg = DeclareLaunchArgument(
        'configuration_directory',
        default_value='/path/to/your/configuration/directory',
        description='Path to the directory containing Cartographer configuration files'
    )
    configuration_basenames_arg = DeclareLaunchArgument(
        'configuration_basenames',
        default_value='your_configuration_file.lua',
        description='Name of the Cartographer configuration .lua file'
    )
    urdf_filenames_arg = DeclareLaunchArgument(
        'urdf_filenames',
        default_value='/path/to/your/urdf/file.urdf',
        description='Path to the URDF file for the robot'
    )

    # Cartographer Occupancy Grid Node
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': True},
            {'resolution': 0.1}
        ]
    )

    # Cartographer Offline Node
    cartographer_offline_node = Node(
        package='cartographer_ros',
        executable='cartographer_offline_node',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', LaunchConfiguration('configuration_directory'),
            '-configuration_basename', LaunchConfiguration('configuration_basenames'),
            '-urdf_filename', LaunchConfiguration('urdf_filenames'),
            '-bag_filename', LaunchConfiguration('bag_filenames')
        ]
    )

    return LaunchDescription([
        # Launch arguments
        bag_filenames_arg,
        configuration_directory_arg,
        configuration_basenames_arg,
        urdf_filenames_arg,
        
        # Nodes
        cartographer_occupancy_grid_node,
        cartographer_offline_node
    ])
