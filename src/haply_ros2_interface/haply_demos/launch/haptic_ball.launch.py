from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    haply_driver = Node(
        package="haply_interface",
        executable="haply_driver_node",
        name="haply_driver_node",
        parameters=[{'frequency': 200.0}]
    )

    visualization_node = Node(
        package="haply_interface",
        executable="rviz_visualization_node",
        name="rviz_visualization_node"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2"
    )

    haptic_ball = Node(
        package="haply_interface",
        executable="haptic_ball",
        name="haptic_ball"
    )

    ld.add_action(haply_driver)
    ld.add_action(visualization_node)
    ld.add_action(rviz)
    ld.add_action(haptic_ball)

    return ld