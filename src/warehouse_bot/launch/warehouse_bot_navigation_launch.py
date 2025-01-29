import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # nav2
    nav2_launch_file = os.path.join(
        get_package_share_directory('warehouse_bot'),
        'launch',
        'navigation_launch.py'
    )


    return LaunchDescription([ 

        # nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file)
        ),   
    ])