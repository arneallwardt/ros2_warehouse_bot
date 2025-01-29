from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([ 

        # move_back_server
        Node(
            package='warehouse_bot_pilots',
            executable='move_back_server',
            name='move_back_server',
            output='screen',
        ),

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