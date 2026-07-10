"""Launch the study GUI with Haply input, mapper, and scenario data."""

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
    """Build the launch description for hardware GUI testing."""
    haply_driver = Node(
        package="haply_interface",
        executable="haply_driver_node",
        name="haply_driver_node",
        output="screen",
        parameters=[
            {
                "frequency": 100.0,
            }
        ],
    )
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
                "mapping_mode": "anchored_delta",
                "use_z_as_y": True,
                "scale_x": 2.0,        # physical 10 cm -> task 20 cm
                "scale_y": 2.0,
                "clamp_raw": True,
                "raw_x_min": -0.20,    # Haply x: +/-20 cm left/right
                "raw_x_max": 0.20,
                "raw_second_min": -0.20,  # Haply z: allow movement below anchor
                "raw_second_max": 0.20,
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
                "source": "haply",
                "render_fps": 100.0,
                "state_publish_hz": 100.0,
                "auto_start": True,
                "endpoint_reached_radius": 0.01,
            }
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("SDL_AUDIODRIVER", "dummy"),
            SetEnvironmentVariable("PYGAME_HIDE_SUPPORT_PROMPT", "1"),
            haply_driver,
            experiment_mapper,
            scenario_generator,
            study_gui,
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
