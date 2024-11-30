import launch
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='warehouse_bot',
            executable='publisher',
            name='launch_publisher',
            output='screen', # ensure that output is printed to the console
            emulate_tty=True, # ensure that output is printed to the console
        ),
         Node(
            package='warehouse_bot',
            executable='subscriber',
            name='launch_subscriber',
            output='screen', # ensure that output is printed to the console
            emulate_tty=True, # ensure that output is printed to the console
        )
  ])