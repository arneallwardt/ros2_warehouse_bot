from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([ 

        # image_provider
        Node(
            package='warehouse_bot_sensors',
            executable='image_provider',
            name='image_provider',
            output='screen',
        ),

        # scan_filter node
        Node(
            package='warehouse_bot_sensors',
            executable='scan_filter',
            name='scan_filter',
            output='screen', # ensure that output is printed to the console
            emulate_tty=True, # ensure that output is printed to the console
        ),
    ])