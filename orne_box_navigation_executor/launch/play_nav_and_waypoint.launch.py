import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    icart_mini_driver_dir = get_package_share_directory('orne_box_navigation_executor')

    launch_file_dir = os.path.join(get_package_share_directory('orne_box_navigation_executor'), 'launch')

    run_file_dir = get_package_share_directory('waypoint_manager2')

    config_dir = os.path.join(icart_mini_driver_dir, 'config')
            
    ### add waypoint_manager2 ###
    waypoint_manager2_cmd = Node(
        package='waypoint_manager2',
        executable='waypoint_manager2_node')
        #name='waypoint_manager2')
            
    ### start costmap_filter_info ###

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
    
    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(config_dir, 'params', 'keepout_params.yaml'),
            description='Full path to the ROS 2 parameters file to use')

    declare_mask_yaml_file_cmd = DeclareLaunchArgument(
            'mask',
            default_value = mask_yaml_file,
            description='Full path to filter mask yaml file to load')

    # Make re-written yaml
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': mask_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
        
    # Nodes launching commands
    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    start_map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[configured_params])

    start_costmap_filter_info_server_cmd = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[configured_params])

    ### end costmap_filter_info ###
    
    nav2_use_sim_time = LaunchConfiguration('nav2_use_sim_time', default='true')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            config_dir,
            'maps',
            'cit_3f_map.yaml'))

    param_file_name = 'nav2_params.yaml'

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            config_dir,
            'params',
            param_file_name))

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
    
    ### start navigation2 ###

    declare_map_file_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='Full path to map file to load')

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=param_dir,
        description='Full path to param file to load')

    declare_nav2_use_sim_time_cmd = DeclareLaunchArgument(
        'nav2_use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')


    navigation2_cmd_group = GroupAction([
        #DeclareLaunchArgument(
        #    'map',
        #    default_value=map_dir,
        #    description='Full path to map file to load'),
        
        #DeclareLaunchArgument(
        #    'nav2_params_file',
        #    default_value=param_dir,
        #    description='Full path to param file to load'),

        #DeclareLaunchArgument(
        #    'nav2_use_sim_time',
        #    default_value='false',
        #    description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'nav2_use_sim_time': nav2_use_sim_time,
                'use_composition':'True',
                'nav2_params_file': param_dir,
                'emcl2_params_file': param_dir,
                'default_bt_xml_filename':bt_dir}.items(),
                
        ),

        #    Node(
        #        package='rviz2',
        #        executable='rviz2',
        #        name='rviz2',
        #        arguments=['-d', rviz_config_dir],
        #        parameters=[{'nav2_use_sim_time': nav2_use_sim_time}],
        #        output='screen'),
    ])

    start_rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'nav2_use_sim_time': nav2_use_sim_time}],
        output='screen')

    ### end navigation2 ###
    
    
    
    ld = LaunchDescription()
    
    # add waypoint_manager2
    #ld.add_action(waypoint_manager2_cmd)
    
    ### start costmap_filter_info add_action ###
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_mask_yaml_file_cmd)

    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_costmap_filter_info_server_cmd)
    ### end costmap_filter_info add_action ###
    
    ### start navigation2 add_action ###
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_nav2_use_sim_time_cmd)
    ld.add_action(navigation2_cmd_group)
    ld.add_action(start_rviz2_cmd)
    ### end navigation2 add_action ###


    return ld
