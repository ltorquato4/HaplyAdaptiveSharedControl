"""Add the controller visualizer to the production Haply study launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Use the production hardware stack with debug logging and visualization."""
    controller = LaunchConfiguration("controller")
    controller_log_level = LaunchConfiguration("controller_log_level")
    participant_id = LaunchConfiguration("participant_id")

    production_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("haply_study_gui"),
                    "launch",
                    "study_gui.launch.py",
                ]
            )
        ),
        launch_arguments={
            "controller": controller,
            "controller_log_level": controller_log_level,
            "participant_id": participant_id,
        }.items(),
    )
    visualizer = Node(
        package="control_node",
        executable="test_control_node_output",
        name="test_control_node_output",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller",
                default_value="state_feedback",
                description="Controller family: state_feedback or mpc.",
            ),
            DeclareLaunchArgument(
                "controller_log_level",
                default_value="DEBUG",
                description="Controller and Estimator log level.",
            ),
            DeclareLaunchArgument(
                "participant_id",
                default_value="DEBUG_HAPLY",
                description="Log/session label; defaults to hardware debugging.",
            ),
            production_launch,
            visualizer,
        ]
    )
