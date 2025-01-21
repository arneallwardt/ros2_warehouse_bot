import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    workspace_root = os.path.join(os.path.dirname(__file__), '../../../../..')
    config_dir = os.path.join(workspace_root, 'src', 'warehouse_bot', 'config')

    # slam_toolbox
    slam_params_file = os.path.join(
        config_dir,
        'mapper_params_online_sync.yaml' #'mapper_params_online_async.yaml' 
    )
    slam_toolbox_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'localization_launch.py'
    )

    # rviz2 
    rviz2_config_file = os.path.join(
        config_dir,
        'warehouse_bot_default_view.rviz'
    )

    return LaunchDescription([ 

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