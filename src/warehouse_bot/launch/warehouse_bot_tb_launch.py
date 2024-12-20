import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    turtlebot_bringup_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_bringup'),
        'launch',
        'robot.launch.py'
    )


    return LaunchDescription([ 

        # slam_toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot_bringup_launch_file)
        ),

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