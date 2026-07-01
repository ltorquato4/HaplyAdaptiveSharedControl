"""Launch the study GUI with Haply input and dummy scenario data."""

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
        package='haply_interface',
        executable='haply_driver_node',
        name='haply_driver_node',
        output='screen',
        parameters=[{
            'frequency': 100.0,
        }],
    )
    dummy_scenario = Node(
        package='haply_study_gui',
        executable='dummy_scenario_generator',
        name='dummy_scenario_generator',
        output='screen',
        parameters=[{
            'seed': 7,
        }],
    )
    study_gui = Node(
        package='haply_study_gui',
        executable='study_gui',
        name='study_gui',
        output='screen',
        additional_env={
            'SDL_AUDIODRIVER': 'dummy',
            'PYGAME_HIDE_SUPPORT_PROMPT': '1',
            'AUDIODEV': 'null',
        },
        parameters=[{
            'source': 'haply',
            'render_fps': 100.0,
            'state_publish_hz': 100.0,
            'auto_start': True,
            'endpoint_reached_radius': 0.01,
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable('SDL_AUDIODRIVER', 'dummy'),
        SetEnvironmentVariable('PYGAME_HIDE_SUPPORT_PROMPT', '1'),
        haply_driver,
        study_gui,
        dummy_scenario,
        RegisterEventHandler(
            OnProcessExit(
                target_action=study_gui,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason='study_gui window closed'
                        )
                    )
                ],
            )
        ),
    ])
