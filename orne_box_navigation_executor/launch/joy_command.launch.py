from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_command',
            executable='joy_command_node',
            name='joy_command_node',
            output='screen',
        )
    ])