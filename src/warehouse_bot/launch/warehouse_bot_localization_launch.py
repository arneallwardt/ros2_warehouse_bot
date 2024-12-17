import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # twist_mux
    twist_mux_config = '/home/kilab/ros2_warehouse_bot/src/warehouse_bot/config/twist_mux.yaml'
    
    # slam_toolbox
    slam_params_file = '/home/kilab/ros2_warehouse_bot/src/warehouse_bot/config/mapper_params_localization.yaml'
    slam_toolbox_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'localization_launch.py'
    )

    # rviz2 
    rviz2_config_file = '/home/kilab/ros2_warehouse_bot/src/warehouse_bot/config/warehouse_bot_default_view.rviz'


    return LaunchDescription([ 

        # twist_mux
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_config],
            remappings=[('cmd_vel_out', 'diff_cont/cmd_vel_unstamped')]
        ),
        
        # scan_filter node
        Node(
            package='warehouse_bot',
            executable='scan_filter',
            name='scan_filter',
            output='screen', # ensure that output is printed to the console
            emulate_tty=True, # ensure that output is printed to the console
        ),

        # slam_toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_file),
            launch_arguments={
                'slam_params_file': slam_params_file
            }.items()
        ),

        # rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz2_config_file]
        ),
    ])