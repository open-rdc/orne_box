import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, IncludeLaunchDescription
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # === デフォルト設定 ===
    package_name = 'orne_box_navigation_executor'
    map_name = 'tsukuba2024'
    waypoint_path = 'tsukuba2024_all'
    bt_file_name = 'navigate_w_replanning_and_wait.xml'
    rviz_config_file = 'nav2_default_view2.rviz'

    # === パス設定 ===
    nav_dir = get_package_share_directory(package_name)
    config_dir = os.path.join(nav_dir, 'config')
    launch_file_dir = os.path.join(nav_dir, 'launch')

    # === ファイルのパス設定 ===
    map_file = os.path.join(config_dir, 'maps', f'{map_name}.yaml')
    mask_file = os.path.join(config_dir, 'maps', f'{map_name}_keepout.yaml')
    waypoint_file = os.path.join(config_dir, 'waypoints', f'{waypoint_path}.yaml')
    bt_file = os.path.join(config_dir, 'behavior_trees', bt_file_name)
    rviz_config_dir = os.path.join(config_dir, 'rviz', rviz_config_file)
    params_file_path = os.path.join(config_dir, 'params', 'nav2_params.yaml')

    # === LaunchConfigurations ===
    map_dir = LaunchConfiguration('map', default=map_file)
    mask_yaml_file = LaunchConfiguration('mask', default=mask_file)
    bt_dir = LaunchConfiguration('default_bt_xml_filename', default=bt_file)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=params_file_path)

    # === ライフサイクルノード設定 ===
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    # === LaunchDescription ===
    return LaunchDescription([
        # --- ウェイポイントマネージャー ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/waypoint_manager2.launch.py']),
            launch_arguments={
                'WAYPOINT_PATH': waypoint_file,
                'OVERWRITE': 'True',
                'WP_FEEDBACK_VISIBLE': 'True',
            }.items()
        ),

        # --- ジョイスティックコマンド ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/joy_command.launch.py'])
        ),

        # --- Launch 引数の宣言 ---
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('params_file', default_value=params_file_path, description='Full path to the ROS 2 parameters file to use'),
        DeclareLaunchArgument('mask', default_value=mask_yaml_file, description='Full path to filter mask yaml file to load'),

        # --- ライフサイクルマネージャー ---
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': LaunchConfiguration('autostart'),
                'node_names': lifecycle_nodes,
            }]
        ),

        # --- フィルターマスク用マップサーバー ---
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            parameters=[RewrittenYaml(
                source_file=params_file,
                root_key=LaunchConfiguration('namespace'),
                param_rewrites={
                    'use_sim_time': use_sim_time,
                    'yaml_filename': mask_yaml_file,
                },
                convert_types=True,
            )]
        ),

        # --- コストマップフィルタ情報サーバー ---
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            parameters=[RewrittenYaml(
                source_file=params_file,
                root_key=LaunchConfiguration('namespace'),
                param_rewrites={
                    'use_sim_time': use_sim_time,
                    'yaml_filename': mask_yaml_file,
                },
                convert_types=True,
            )]
        ),

        # --- Bringup のインクルード ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_composition': 'True',
                'params_file': params_file,
                'emcl2_params_file': params_file,
                'default_bt_xml_filename': bt_dir,
            }.items()
        ),

        # --- RViz2 の起動 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
