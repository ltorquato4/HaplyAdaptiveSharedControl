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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the launch description for hardware-backed GUI runs."""
    use_controller = LaunchConfiguration("use_controller")
    use_estimator = LaunchConfiguration("use_estimator")
    task_file = LaunchConfiguration("task_file")
    controller_modes = LaunchConfiguration("controller_modes")
    default_task_file = PathJoinSubstitution(
        [FindPackageShare("study_orchestration"), "config", "default_tasks.yaml"]
    )

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
                "task_file": task_file,
                "controller_modes": controller_modes,
                "endpoint_reached_radius": 0.01,
                "inter_trial_delay_s": 1.0,
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
                "scale_x": 2.0,  # physical 10 cm -> task 20 cm
                "scale_y": 2.0,
                "clamp_raw": True,
                "raw_x_min": -0.20,  # Haply x: +/-20 cm left/right
                "raw_x_max": 0.20,
                "raw_second_min": -0.20,  # Haply z: allow movement below anchor
                "raw_second_max": 0.20,
            }
        ],
    )
    controller = Node(
        package="control_node",
        executable="control_node",
        name="control_node",
        output="screen",
        condition=IfCondition(use_controller),
        parameters=[
            {
                "log_level": "INFO",
            }
        ],
    )
    estimator = Node(
        package="estimator_node",
        executable="estimator_node",
        name="estimator_node",
        output="screen",
        condition=IfCondition(use_estimator),
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
                "width": 1280,
                "height": 720,
                "side_panel_width": 300,
                "workspace_padding": 52,
                "render_fps": 100.0,
                "state_publish_hz": 100.0,
                "auto_start": False,
                "endpoint_reached_radius": 0.01,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_controller",
                default_value="false",
                description="Start control_node with the study GUI.",
            ),
            DeclareLaunchArgument(
                "use_estimator",
                default_value="false",
                description="Start estimator_node with the study GUI.",
            ),
            DeclareLaunchArgument(
                "task_file",
                default_value=default_task_file,
                description="YAML file defining scenario path geometry.",
            ),
            DeclareLaunchArgument(
                "controller_modes",
                default_value="fixed",
                description=(
                    "Comma-separated controller modes: adaptive, fixed, "
                    "or adaptive,fixed."
                ),
            ),
            SetEnvironmentVariable("SDL_AUDIODRIVER", "dummy"),
            SetEnvironmentVariable("PYGAME_HIDE_SUPPORT_PROMPT", "1"),
            haply_driver,
            experiment_mapper,
            scenario_generator,
            controller,
            estimator,
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
