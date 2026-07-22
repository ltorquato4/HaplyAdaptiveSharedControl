"""Launch the study GUI with mouse input, mapper, and scenario generator."""

from haply_study_gui.study_launch import create_study_stack
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Build the launch description for mouse-only GUI testing."""
    controller = LaunchConfiguration("controller")
    controller_log_level = LaunchConfiguration("controller_log_level")
    docking_enabled = LaunchConfiguration("docking_enabled")
    nodes, study_gui = create_study_stack(
        "mouse",
        controller,
        controller_log_level=controller_log_level,
        docking_enabled=docking_enabled,
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
            DeclareLaunchArgument(
                "docking_enabled",
                default_value="false",
                description="Enable optional terminal state-feedback docking.",
            ),
            SetEnvironmentVariable("SDL_AUDIODRIVER", "dummy"),
            SetEnvironmentVariable("PYGAME_HIDE_SUPPORT_PROMPT", "1"),
            *nodes,
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
