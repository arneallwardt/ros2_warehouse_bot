from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([ 

        # image_provider
        Node(
            package='warehouse_bot_world',
            executable='product_info_provider',
            name='product_info_provider',
            output='screen',
        )
    ])