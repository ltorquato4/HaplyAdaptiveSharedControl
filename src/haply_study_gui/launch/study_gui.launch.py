"""Launch the study GUI with the real Haply driver, mapper, and scenario."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from haply_study_gui.study_launch import create_study_stack


def generate_launch_description():
    """Build the launch description for hardware-backed GUI runs."""
    controller = LaunchConfiguration("controller")
    controller_log_level = LaunchConfiguration("controller_log_level")
    controller_enabled_expression = PythonExpression(
        ["'", controller, "' in ['mpc', 'state_feedback']"]
    )
    controller_enabled = ParameterValue(
        controller_enabled_expression,
        value_type=bool,
    )
    nodes, study_gui = create_study_stack(
        "haply",
        controller,
        include_driver=True,
        controller_log_level=controller_log_level,
        require_system_ready=controller_enabled,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller",
                default_value="none",
                description="Controller family: none, mpc, or state_feedback.",
            ),
            DeclareLaunchArgument(
                "controller_log_level",
                default_value="INFO",
                description="Log level for the Controller and Estimator.",
            ),
            SetEnvironmentVariable("SDL_AUDIODRIVER", "dummy"),
            SetEnvironmentVariable("PYGAME_HIDE_SUPPORT_PROMPT", "1"),
            *nodes,
            Node(
                package="data_logger",
                executable="data_logger_node",
                name="data_logger_node",
                output="screen",
                condition=IfCondition(controller_enabled_expression),
                parameters=[{"save_directory": "./logs", "log_level": controller_log_level}],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=study_gui,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="study_gui window closed"))
                    ],
                )
            ),
        ]
    )
