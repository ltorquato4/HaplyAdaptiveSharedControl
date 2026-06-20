from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='haply_interface',
            executable='haply_driver_node',
            name='haply_driver_node',
            output='screen',
            parameters=[{'frequency': 200.0}],
        ),
        Node(
            package='haply_study_gui',
            executable='study_traffic_light_gui',
            name='study_traffic_light_gui',
            output='screen',
            parameters=[{
                'controller_type': 'adaptive',
                'condition_order': 'red,yellow,green',
                'phase_duration_s': 60.0,
                'run_duration_s': 900.0,
                'auto_start': False,
            }],
        ),
    ])
