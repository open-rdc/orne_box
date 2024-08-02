import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    icart_mini_driver_dir = get_package_share_directory('orne_box_navigation_executor')

    launch_file_dir = os.path.join(get_package_share_directory('orne_box_navigation_executor'), 'launch')

    run_file_dir = get_package_share_directory('waypoint_manager2')

    config_dir = os.path.join(icart_mini_driver_dir, 'config')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            config_dir,
            'maps',
            # 'tsukuba_check_area_res01.yaml'))
            # 'tsukuba_all_clean_res01.yaml'))
            # 'tsudanuma_gaisyu_res01.yaml'))
            # 'tsukuba_park3.yaml'))
            #'tsukuba_1118.yaml'))
            'tsukuba_1119_okunote.yaml'))
            #'tsukuba_1119.yaml'))

    param_file_name = 'nav2_params.yaml'

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            config_dir,
            'params',
            param_file_name))

    return LaunchDescription([

        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        # add waypoint_manager2
        Node(
            package='waypoint_manager2',
            executable='waypoint_manager2_node'
            #name='waypoint_manager2'
        ),        

        # add costmap
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_dir, '/costmap_filter_info.launch.py'])
        ),
        launch.actions.LogInfo(
            msg="Launch costmap filter node."
        ),

        # add navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_dir, '/navigation2.launch.py'])
        ),
        launch.actions.LogInfo(
            msg="Launch navigation2 node."
        )


    ])
