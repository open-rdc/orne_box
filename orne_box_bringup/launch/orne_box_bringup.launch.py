import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    icart_mini_driver_dir = get_package_share_directory('orne_box_bringup')

    #for ypspur param file ex.RADIUS,TREAD
    ypspur_param = os.path.join(icart_mini_driver_dir,'config/ypspur','box_v3.param')
    
    launch_file_dir = os.path.join(get_package_share_directory('orne_box_bringup'), 'launch') 
    launch_include_file_dir = os.path.join(get_package_share_directory('orne_box_bringup'), 'launch/include') 
    
    ypspur_coordinator_path = os.path.join(icart_mini_driver_dir,'scripts','ypspur_coordinator_bridge')

    ### start description ###
    packages_name = "orne_box_description"
    xacro_file_name = "orne_box_3d_lidar_rfans.urdf.xacro"
    # Get URDF via xacro
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
    
    # joint_state_pub_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     output="screen",
    # )
    joint_state_pub_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )
        
    description_nodes = [
        robot_state_pub_node,
        # joint_state_pub_gui_node
        joint_state_pub_node
    ]

    ### end description ###

    return LaunchDescription([
        launch.actions.LogInfo(
            msg="Launch ypspur coordinator."
        ),
        # Node(
        #     package='icart_mini_driver',
        #     namespace='ypspur_coordinator',
        #     executable='ypspur_coordinator_bridge',
        #     parameters=[icart_mini_param]
        # ),
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
                        'base_frame_id':'base_link',
                        'Hz':40,
                        'left_wheel_joint':'left_wheel_joint',
                        'right_wheel_joint':'right_wheel_joint',
                        'liner_vel_lim':1.5,
                        'liner_accel_lim':1.5,
                        'angular_vel_lim':3.14,
                        'angular_accel_lim':3.14,
                        'calculate_odom_from_ypspur':True,
                        'publish_odom_tf':False
            }]
        ),
        #robot_state_publisher and joint_state_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_include_file_dir, '/description.launch.py'])
        ),
        launch.actions.LogInfo(
            msg="Launch robot_description  node."
        ),

        ### start description
        # #description_nodes,
        # robot_state_pub_node,
        # joint_state_pub_node,

        #mixed wheel_odom and other (IMU etc..)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_include_file_dir, '/robot_localization_ekf.launch.py'])
        ),
        launch.actions.LogInfo(
            msg="Launch robot_localization_ekf node."
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_include_file_dir, '/teleop.launch.py'])
        ),
        launch.actions.LogInfo(
            msg="Launch joy node."
        ),

        # sensors
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [launch_include_file_dir, '/urg_node2.launch.py'])
        # ),
        # launch.actions.LogInfo(
        #     msg="Launch URG  node."
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_include_file_dir, '/adis16465.launch.py'])
        ),
        launch.actions.LogInfo(
            msg="Launch IMU node."
        ),
        # IncludeLaunchDescription(    
        #     PythonLaunchDescriptionSource(
        #         [launch_include_file_dir, '/imu_filter.launch.py'])
        # ),
        # launch.actions.LogInfo(
        #     msg="Launch IMU Filter node."
        # ),

        # add rfans16
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [launch_include_file_dir, '/rfans16.launch.py'])
        # ),
        # launch.actions.LogInfo(
        #     msg="Launch rfans16 node."
        # ),

        # add rfans16_filters
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_include_file_dir, '/rfans16_filters.launch.py'])
        ),
        launch.actions.LogInfo(
            msg="Launch rfans16_filters node."
        )

    ])