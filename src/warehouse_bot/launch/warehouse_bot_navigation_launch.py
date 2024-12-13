import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    nav2_params_file = '/home/kilab/ros2_warehouse_bot/src/warehouse_bot/config/nav2_params.yaml'

    # nav2
    nav2_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )


    return LaunchDescription([ 

        # nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'params_file': nav2_params_file
            }.items()
        ),   
    ])