from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([ 

        # product_aligner
        Node(
            package='warehouse_bot_pilots',
            executable='product_aligner',
            name='product_aligner',
            output='screen',
        ),

        # product_manipulator
        Node(
            package='warehouse_bot_pilots',
            executable='product_manipulator',
            name='product_manipulator',
            output='screen',
        ),
    ])