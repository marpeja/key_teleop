from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='key_teleop',
            executable='key_teleop_3',
            name='sim'
        )
    ])