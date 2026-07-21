"""Shared launch-node factory for mouse and Haply study entry points."""

from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def create_study_stack(
    source,
    controller=None,
    include_driver=False,
    controller_log_level="INFO",
    require_system_ready=False,
):
    """Return common study nodes and the GUI node used for shutdown handling."""
    config_dir = get_package_share_directory("study_orchestration") + "/config"
    parameters = [
        config_dir + "/study_base.yaml",
        config_dir + f"/study_{source}.yaml",
    ]
    scenario_parameters = [*parameters, {
        "task_file": config_dir + "/default_tasks.yaml",
        "require_controller_ready": require_system_ready,
        "require_estimator_ready": require_system_ready,
        "require_logger_ready": require_system_ready,
    }]
    nodes = []
    if include_driver:
        nodes.append(
            Node(
                package="haply_interface",
                executable="haply_driver_node",
                name="haply_driver_node",
                output="screen",
                parameters=[{"frequency": 100.0}],
            )
        )
    nodes.extend(
        [
            Node(
                package="study_orchestration",
                executable="experiment_mapper",
                name="experiment_mapper",
                output="screen",
                parameters=parameters,
            ),
            Node(
                package="study_orchestration",
                executable="scenario_generator",
                name="scenario_generator",
                output="screen",
                parameters=scenario_parameters,
            ),
        ]
    )
    if controller is not None:
        controller_enabled = PythonExpression(
            ["'", controller, "' in ['mpc', 'state_feedback']"]
        )
        controller_parameters = [*parameters, {"log_level": controller_log_level}]
        controller_parameters.append(
            {
                "use_mpc_controller": ParameterValue(
                    PythonExpression(["'", controller, "' == 'mpc'"]),
                    value_type=bool,
                )
            }
        )
        nodes.append(
            Node(
                package="control_node",
                executable="control_node",
                name="control_node",
                output="screen",
                condition=IfCondition(controller_enabled),
                parameters=controller_parameters,
            )
        )
        # Start the estimator with either controller family. A session can move
        # from fixed to adaptive tasks without a process restart.
        nodes.append(
            Node(
                package="estimator_node",
                executable="estimator_node",
                name="estimator_node",
                output="screen",
                condition=IfCondition(controller_enabled),
                parameters=[{"log_level": controller_log_level}],
            )
        )
    gui = Node(
        package="haply_study_gui",
        executable="study_gui",
        name="study_gui",
        output="screen",
        additional_env={
            "SDL_AUDIODRIVER": "dummy",
            "PYGAME_HIDE_SUPPORT_PROMPT": "1",
            "AUDIODEV": "null",
        },
        parameters=[*parameters, {
            "require_system_ready": require_system_ready,
            "controller_family": controller or "none",
        }],
    )
    nodes.append(gui)
    return nodes, gui
