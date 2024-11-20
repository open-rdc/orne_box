from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_manager2',
            executable='traffic_waypoint_manager2_node',
            name='traffic_waypoint_manager2',
            output='screen'
        )
    ])
