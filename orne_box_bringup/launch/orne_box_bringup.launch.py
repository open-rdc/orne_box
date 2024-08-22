import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    icart_mini_driver_dir = get_package_share_directory('orne_box_bringup')
    adi_driver2_dir = get_package_share_directory('adi_driver2')
    launch_include_file_dir = os.path.join(get_package_share_directory('orne_box_bringup'), 'launch/include') 

    ypspur_param = os.path.join(icart_mini_driver_dir,'config/ypspur','box_v3.param')
    imu_param = os.path.join(adi_driver2_dir, 'config', 'adis16465.param.yaml')

    ypspur_coordinator_path = os.path.join(icart_mini_driver_dir,'scripts','ypspur_coordinator_bridge')

    packages_name = "orne_box_description"
    xacro_file_name = "orne_box_3d_lidar_rfans.urdf.xacro"

    imu_declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='imu',
        description='Set namespace for node.'
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(packages_name), "urdf", xacro_file_name]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    namespace = launch.substitutions.LaunchConfiguration('namespace', default='rfans')
    output = launch.substitutions.LaunchConfiguration('output', default='screen')
    respawn = launch.substitutions.LaunchConfiguration('respawn', default='true')
    revise_angle_128 = launch.substitutions.LaunchConfiguration('revise_angle_128', default='0.027721,0.002361,-0.043092,-0.023711,0.003442,-0.046271,-0.018255,0.036872,-0.048702,-0.025516,0.002106,-0.040973,-0.019308,0.001663,-0.045102,-0.015489,0.001148,-0.047884,-0.018850,0.016742,-0.044528,-0.033280,0.001597,-0.039345,1,1.8,1,1.8,0.15,0.15,')
    revise_angle_32 = launch.substitutions.LaunchConfiguration('revise_angle_32', default='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,45,0,0,0,0.1,0,')
    read_fast = launch.substitutions.LaunchConfiguration('read_fast', default='false')
    read_once = launch.substitutions.LaunchConfiguration('read_once', default='false')
    repeat_delay = launch.substitutions.LaunchConfiguration('repeat_delay', default='0.0')

    icart_mini_driver_node = GroupAction(
        actions=[
            launch.actions.LogInfo(
            msg="Launch ypspur coordinator."
            ),
            launch.actions.ExecuteProcess(
                cmd=[ypspur_coordinator_path,ypspur_param],
                shell=True,
            ),
            launch.actions.LogInfo(
                msg="Launch icart_mini_mini_driver node."
            ),
            Node(
                package='orne_box_bringup',
                executable='icart_mini_driver',
                parameters=[{'odom_frame_id':'odom',
                            'base_frame_id':'base_footprint',
                            'Hz':40,
                            'left_wheel_joint':'left_wheel_joint',
                            'right_wheel_joint':'right_wheel_joint',
                            'liner_vel_lim':1.5,
                            'liner_accel_lim':1.5,
                            'angular_vel_lim':3.14,
                            'angular_accel_lim':3.14,
                            'calculate_odom_from_ypspur':True,
                            'publish_odom_tf':True
                }]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [launch_include_file_dir, '/teleop.launch.py'])
            ),
            launch.actions.LogInfo(
                msg="Launch joy node."
            ),
        ]
    )

    description_node = GroupAction(
        actions=[
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description]),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                output="screen"),
            Node(
                name="imu_filter_msdgwick_node",
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                parameters=
                    [{
                        "use_mag": False
                    }],
                output="screen")
        ]
    )

    imu_node = GroupAction(
        actions=[
            PushRosNamespace([LaunchConfiguration('namespace')]),
            Node(
                name='adis16465_node',
                package='adi_driver2',
                executable='adis16465',
                parameters=[imu_param],
                output='screen'),
            ]
    )

    rfans_driver_node = GroupAction(
        actions=[
            DeclareLaunchArgument('namespace', default_value=namespace),
            DeclareLaunchArgument('output', default_value=output),
            DeclareLaunchArgument('respawn', default_value=respawn),
            DeclareLaunchArgument('revise_angle_128', default_value=revise_angle_128),
            DeclareLaunchArgument('revise_angle_32', default_value=revise_angle_32),
            DeclareLaunchArgument('read_fast', default_value=read_fast),
            DeclareLaunchArgument('read_once', default_value=read_once),
            DeclareLaunchArgument('repeat_delay', default_value=repeat_delay),
            Node(
                name='rfans_driver_node',
                package='rfans_driver_ros2',
                executable='driver_node',
                parameters=[
                    {'model':'R-Fans-16'},
                    #publish paket name /rfans_driver/ + advertise_name
                    #{'advertise_name': 'surestar_packets'},
                    {'control_name': 'surestar_control'},
                    {'device_ip': '192.168.0.3'},
                    {'device_port': 2014},
                    {'rps': 10},
                    {'pcap': ''},
                    {'data_level': 3},
                    {'use_double_echo': False},
                    {'read_fast': read_fast},
                    {'read_once': read_once},
                    {'repeat_delay': repeat_delay},
                ]
            ),
            Node(
                name='calculation_node',
                package='rfans_driver_ros2',
                executable='calculation_node',

                #remappings=[
                #    ('/rfans_driver/rfans_packets','rfans_packets'),
                    # ('/rfans_driver/rfans_points','rfans_points')
                #],
                parameters=[
                    #publish topic name
                    #{'advertise_name': 'rfans_points'},
                    #subscribe topic name
                    #{'subscribe_name': 'surestar_packets'},
                    {'frame_id': 'surestar'},
                    {'use_gps': False},
                    {'revise_angle_128': revise_angle_128},
                    {'revise_angle_32': revise_angle_32},
                    {'min_range':0.0},
                    {'max_range': 180.0},
                    {'min_angle':0.0},
                    {'max_angle': 360.0},
                    {'angle_duration':360.0},
                    {'model':'R-Fans-16'}
                ]
            ),
            Node(
                name='cloud_process',
                package='rfans_driver_ros2',
                executable='cloud_process',
                remappings=[
                    #publish pointcloud2
                    ('rfans_points', 'surestar_points')
                    #subsclibe pointcloud2
                    # ('/rfans_driver/rfans_points', '/rfans_points')
                ]
            ),
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan',
                remappings=[
                    ('cloud_in', 'surestar_points'),
                    ('scan', 'surestar_scan'),
                ],
                parameters=[{
                        'target_frame': '',
                        'transform_tolerance': 0.01,
                        'min_height': 0.0,
                        'max_height': 30.0,
                        'angle_min': -3.1415,  # -M_PI/2
                        'angle_max': 3.1415,  # M_PI/2
                        'angle_increment': 0.0087,  # M_PI/360.0
                        'scan_time': 0.3333,
                        'range_min': 0.5,
                        'range_max': 200.0,
                        'use_inf': True,
                        'inf_epsilon': 1.0
                }]
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(imu_declare_namespace)

    ld.add_action(icart_mini_driver_node)
    ld.add_action(description_node)
    ld.add_action(imu_node)
    ld.add_action(rfans_driver_node)

    return ld