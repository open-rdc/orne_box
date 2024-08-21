import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, GroupAction, IncludeLaunchDescription

from nav2_common.launch import RewrittenYaml


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
            'cit_3f_map.yaml'))
            
    ### add navigation2 ###
    
    bt_file_name ='navigate_w_replanning_and_wait.xml'
    # bt_file_name ='navigate_w_replanning_and_recovery.xml'

    bt_dir = LaunchConfiguration(
        'default_bt_xml_filename',
        default=os.path.join(
            config_dir,
            'behavior_trees',
            bt_file_name))
    rviz_config_dir = os.path.join(
        config_dir,
        'rviz',
        'nav2_default_view2.rviz')
    
    ### end navigation2 ###

    ### add costmap_filter_info ###

    mask_yaml_file = LaunchConfiguration(
        'mask',
        default=os.path.join(
            config_dir,
            'maps',
            'cit_3f_map_keepout.yaml'))
    # Create our own temporary YAML files that include substitutions
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    # Parameters
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    ### end costmap_filter_info ###
    
    return LaunchDescription([

        # add waypoint_manager2
        #Node(
        #    package='waypoint_manager2',
        #    executable='waypoint_manager2_node'
        #    #name='waypoint_manager2'
        #),

        # add costmap
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #        [launch_file_dir, '/costmap_filter_info.launch.py'])
        #),
        #launch.actions.LogInfo(
        #    msg="Launch costmap filter node."
        #),


        ### costmap_filter_info ###

        # Declare the launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(config_dir, 'params', 'keepout_params.yaml'),
            description='Full path to the ROS 2 parameters file to use'),

        DeclareLaunchArgument(
            'mask',
            default_value = mask_yaml_file,
            description='Full path to filter mask yaml file to load'),

        # Nodes launching commands
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[RewrittenYaml(
                        source_file=params_file,
                        root_key=namespace,
                        param_rewrites={'use_sim_time': use_sim_time,
                                        'yaml_filename': mask_yaml_file},
                        convert_types=True)]),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[RewrittenYaml(
                        source_file=params_file,
                        root_key=namespace,
                        param_rewrites={'use_sim_time': use_sim_time,
                                        'yaml_filename': mask_yaml_file},
                        convert_types=True)]),

        ### end costmap_filter_info ###

        ### add navigation2 ###
        
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),
            
        SetLaunchConfiguration(
            name='params_file',
            value=os.path.join(config_dir, 'params', 'nav2_params.yaml')
        ),
        
        SetLaunchConfiguration(
            name='use_sim_time',
            value='false'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_composition':'True',
                'params_file': params_file,
                'emcl2_params_file': params_file,
                'default_bt_xml_filename':bt_dir}.items(),
                
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        
        ### end navigation2 ###


    ])

