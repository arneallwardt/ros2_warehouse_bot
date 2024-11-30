from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='python_parameters',
            executable='minimal_param_node',
            name='custom_minimal_param_node',
            output='screen', # ensure that output is printed to the console
            emulate_tty=True, # ensure that output is printed to the console
            parameters=[
                {'my_parameter': 'param_value'}
            ]
        )
    ])
