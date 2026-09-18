from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='franka_server',
            executable='franka_server_node',
            name='franka_server',
            output='screen',
        ),
    ])
