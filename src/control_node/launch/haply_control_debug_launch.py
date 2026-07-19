"""Launch the controller debug environment with Haply input, orchestrator nodes, estimator, logger, and visualizer."""

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
    """Build the launch description for hardware GUI testing with visual debugging."""
    task_file = LaunchConfiguration("task_file")
    controller_modes = LaunchConfiguration("controller_modes")
    log_level = LaunchConfiguration("log_level")
    save_directory = LaunchConfiguration("save_directory")

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
                "scale_x": 2.0,
                "scale_y": 2.0,
                "clamp_raw": True,
                "raw_x_min": -0.20,
                "raw_x_max": 0.20,
                "raw_second_min": -0.20,
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

    # Pygame Visualizer Node for controller debugging
    visualizer_node = Node(
        package="control_node",
        executable="test_control_node_output",
        name="test_control_node_output",
        output="screen",
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
                default_value="adaptive",
                description="Comma-separated controller modes: adaptive, fixed, or adaptive,fixed.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="DEBUG",  # Defaulting to DEBUG for debugging purposes
                description="Logging level for the nodes (DEBUG, INFO, WARN, ERROR).",
            ),
            DeclareLaunchArgument(
                "save_directory",
                default_value="./logs",
                description="Directory path where trial CSV files will be saved.",
            ),
            SetEnvironmentVariable("SDL_AUDIODRIVER", "dummy"),
            SetEnvironmentVariable("PYGAME_HIDE_SUPPORT_PROMPT", "1"),
            haply_driver,
            experiment_mapper,
            scenario_generator,
            study_gui,
            estimator_node,
            control_node,
            data_logger_node,
            visualizer_node,
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