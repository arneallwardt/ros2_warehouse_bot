import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    turtlebot_bringup_launch_file = os.path.join(
        get_package_share_directory('warehouse_bot'),
        'launch',
        'robot.launch.py'
    )

    open_manipulator_controller_launch_file = os.path.join(
        get_package_share_directory('warehouse_bot'),
        'launch',
        'open_manipulator_x_controller_modified.launch.py'
    )

    warehouse_bot_sensors_launch_file = os.path.join(
        get_package_share_directory('warehouse_bot_sensors'),
        'launch',
        'warehouse_bot_sensors_launch.py'
    )

    return LaunchDescription([ 

        DeclareLaunchArgument(
            'usb_port_open_manipulator', default_value='/dev/ttyUSB1', description='USB port for open manipulator'
        ),

        DeclareLaunchArgument(
            'usb_port_lds', default_value='/dev/ttyUSB0', description='USB port for Lidar'
        ),

        # turtlebot_bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot_bringup_launch_file),
            launch_arguments={'usb_port_lds': LaunchConfiguration('usb_port_lds')}.items()
        ),

        # open_manipulator_x_controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(open_manipulator_controller_launch_file),
            launch_arguments={'usb_port': LaunchConfiguration('usb_port_open_manipulator')}.items()
        ),

        # warehouse_bot_sensors
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(warehouse_bot_sensors_launch_file)
        ),

    ])