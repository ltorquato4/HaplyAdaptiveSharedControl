from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Start the headless control node
        Node(
            package='control_node',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[{'log_level': 'DEBUG'}] 
        ),
        
        # 2. Start the Pygame Visualizer Node
        Node(
            package='control_node',
            executable='test_control_node_output',
            name='test_control_node_output',
            output='screen'
        )
    ])