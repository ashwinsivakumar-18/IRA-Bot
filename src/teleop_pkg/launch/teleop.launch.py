from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_pkg',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e'
        )
    ])
