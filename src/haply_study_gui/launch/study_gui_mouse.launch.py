"""Launch the study GUI with mouse input, mapper, scenario generator, estimator, control node, and data logger."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the launch description for mouse-only GUI testing."""
    task_file = LaunchConfiguration("task_file")
    controller_modes = LaunchConfiguration("controller_modes")
    log_level = LaunchConfiguration("log_level")
    save_directory = LaunchConfiguration("save_directory")

    default_task_file = PathJoinSubstitution(
        [FindPackageShare("study_orchestration"), "config", "default_tasks.yaml"]
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
                "mapping_mode": "identity",
            }
        ],
    )

    control_node = Node(
        package="control_node",
        executable="control_node",
        name="control_node",
        output="screen",
        parameters=[
            {
                "log_level": log_level,
            }
        ],
    )

    estimator_node = Node(
        package="estimator_node",
        executable="estimator_node",
        name="estimator_node",
        output="screen",
        parameters=[
            {
                "log_level": log_level,
            }
        ],
    )

    data_logger_node = Node(
        package="data_logger",
        executable="data_logger_node",
        name="data_logger_node",
        output="screen",
        parameters=[
            {
                "save_directory": save_directory,
                "log_level": log_level,
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
                "width": 1280,
                "height": 720,
                "side_panel_width": 300,
                "workspace_padding": 52,
                "render_fps": 30.0,
                "state_publish_hz": 100.0,
                "mouse_simulation_hz": 100.0,
                "auto_start": False,
                "endpoint_reached_radius": 0.01,
            }
        ],
    )

    return LaunchDescription(
        [
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
            DeclareLaunchArgument(
                "log_level",
                default_value="INFO",
                description="Logging level for the nodes (DEBUG, INFO, WARN, ERROR).",
            ),
            DeclareLaunchArgument(
                "save_directory",
                default_value="./logs",
                description="Directory path where trial CSV files will be saved.",
            ),
            SetEnvironmentVariable("SDL_AUDIODRIVER", "dummy"),
            SetEnvironmentVariable("PYGAME_HIDE_SUPPORT_PROMPT", "1"),
            study_gui,
            experiment_mapper,
            scenario_generator,
            control_node,
            estimator_node,
            data_logger_node,
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