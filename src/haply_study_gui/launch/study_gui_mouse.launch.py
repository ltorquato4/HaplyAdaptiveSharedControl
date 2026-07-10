"""Launch the study GUI with mouse input, mapper, and scenario generator."""

from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for mouse-only GUI testing."""
    scenario_generator = Node(
        package="study_orchestration",
        executable="scenario_generator",
        name="scenario_generator",
        output="screen",
        parameters=[
            {
                "endpoint_reached_radius": 0.01,
            }
        ],
    )
    experiment_mapper = Node(
        package="study_orchestration",
        executable="experiment_mapper",
        name="experiment_mapper",
        output="screen",
        parameters=[
            {
                "mapping_mode": "identity",
            }
        ],
    )
    study_gui = Node(
        package="haply_study_gui",
        executable="study_gui",
        name="study_gui",
        output="screen",
        additional_env={
            "SDL_AUDIODRIVER": "dummy",
            "PYGAME_HIDE_SUPPORT_PROMPT": "1",
            "AUDIODEV": "null",
        },
        parameters=[
            {
                "source": "mouse",
                "render_fps": 30.0,
                "state_publish_hz": 100.0,
                "mouse_simulation_hz": 100.0,
                "auto_start": True,
                "endpoint_reached_radius": 0.01,
            }
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("SDL_AUDIODRIVER", "dummy"),
            SetEnvironmentVariable("PYGAME_HIDE_SUPPORT_PROMPT", "1"),
            study_gui,
            experiment_mapper,
            scenario_generator,
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
