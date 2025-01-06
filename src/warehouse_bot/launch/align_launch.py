from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    nodes = [
        # product_info_provider
        # Node(
        #     package='warehouse_bot_world',
        #     executable='product_info_provider',
        #     name='product_info_provider',
        #     output='screen',
        # ),

        # product_aligner
        Node(
            package='warehouse_bot_pilots',
            executable='product_aligner',
            name='product_aligner',
            output='screen',
        ),

        # warehouse_bot_main
        Node(
            package='warehouse_bot',
            executable='warehouse_bot_main',
            name='warehouse_bot_main',
            output='screen',
        ),
    ]

    # event_handler = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=nodes[0],
    #         on_start=[nodes[1], nodes[2]]
    #     )
    # )

    return LaunchDescription(nodes)