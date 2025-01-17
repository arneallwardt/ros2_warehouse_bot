import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # navigation
    navigation_launch_file = os.path.join(
        get_package_share_directory('warehouse_bot'),
        'launch',
        'warehouse_bot_navigation_launch.py'
    )

    # localization
    localization_launch_file = os.path.join(
        get_package_share_directory('warehouse_bot'),
        'launch',
        'warehouse_bot_localization_launch.py'
    )

    return LaunchDescription([ 

        # navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_file)
        ),

        # localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch_file)
        ),   
    ])