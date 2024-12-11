import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    waypoint_node = Node(
        package='waypoint_manager2',
        executable='traffic_waypoint_manager2_node',
        name='traffic_waypoint_manager2',
        output='screen'
    )

    reconfigure_node = Node(
        package='waypoint_reconfigure',
        executable='waypoint_reconfigure',
        output='screen'
    )

    launch_discription = LaunchDescription()

    launch_discription.add_entity(waypoint_node)
    launch_discription.add_entity(reconfigure_node)

    return launch_discription
