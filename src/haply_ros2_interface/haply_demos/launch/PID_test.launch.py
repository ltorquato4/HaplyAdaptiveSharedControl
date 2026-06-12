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

    PID_test = Node(
        package="haply_interface",
        executable="PID_test",
        name="PID_test"
    )

    plotter = Node(
            package='haply_interface',
            executable='plotter_node',
            name='plotter_node',
            output='screen',
            parameters=[
                {"plot_window": 30.0}
            ]
        )

    ld.add_action(haply_driver)
    ld.add_action(PID_test)
    ld.add_action(plotter)

    return ld
