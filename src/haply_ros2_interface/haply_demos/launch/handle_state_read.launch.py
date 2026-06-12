from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='haply_interface',
            executable='handle_driver_node',
            name='handle_driver_node',
            output='screen',
            parameters=[{'frequency': 200.0}],
        ),
        Node(
            package='haply_interface',
            executable='state_subscriber_handle',
            name='state_subscriber_handle',
            output='screen',
        ),
    ])
