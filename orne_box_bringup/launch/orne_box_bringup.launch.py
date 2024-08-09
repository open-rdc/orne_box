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
    imu_params_file = os.path.join(adi_driver2_dir, 'config', 'adis16465.param.yaml')

    # launch_file_dir = os.path.join(get_package_share_directory('orne_box_bringup'), 'launch') 
    ypspur_coordinator_path = os.path.join(icart_mini_driver_dir,'scripts','ypspur_coordinator_bridge')

    packages_name = "orne_box_description"
    xacro_file_name = "orne_box_3d_lidar_rfans.urdf.xacro"

    push_ns = PushRosNamespace([LaunchConfiguration('namespace')])

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

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_pub_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    imu_adis16465_node = Node(
        name="adis16465_node",
        package="adi_driver2",
        executable="adis16465",
        parameters=[imu_params_file],
        output="screen"
    )

    imu_filter_node = Node(
        name="imu_filter_msdgwick_node"
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=
            [{
                "use_mag": False
            }],
        output="screen"
    )

    icart_mini_driver_node = [
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
            parameters=
                [{
                    'odom_frame_id':'odom',
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

    imu_raw = GroupAction(
        actions=[
        push_ns,
        imu_adis16465_node,
        ]
    )

    nodes = [
        icart_mini_driver_node,
        robot_state_pub_node,
        joint_state_pub_node,
        imu_declare_namespace,
        imu_raw,
        imu_filter_node
    ]

    return LaunchDescription(nodes)