from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_package',
            executable='scan_filter',
            output='screen'
        )
    ])