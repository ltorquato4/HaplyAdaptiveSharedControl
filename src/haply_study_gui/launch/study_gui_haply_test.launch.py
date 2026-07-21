"""Launch the study GUI with Haply input, mapper, and scenario data."""

from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from haply_study_gui.study_launch import create_study_stack


def generate_launch_description():
    """Build the launch description for hardware GUI testing."""
    nodes, study_gui = create_study_stack("haply", include_driver=True)

    return LaunchDescription(
        [
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
