from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Haply driver node
        Node(
            package='haply_interface',
            executable='haply_driver_node',
            name='haply_driver_node',
            output='screen',
            parameters=[
                {"frequency": 200.0},
                {"max_force": 10.0}
            ]
        ),

        # Target position sinus node
        Node(
            package='haply_interface',
            executable='target_position_sinus',
            name='target_position_sinus'
        ),

        # Plotter node
        Node(
            package='haply_interface',
            executable='plotter_node',
            name='plotter_node',
            output='screen',
            parameters=[
                {"plot_window": 30.0}
            ]
        ),
    ])
