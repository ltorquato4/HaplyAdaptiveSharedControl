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
    docking_enabled=False,
):
    """Return common study nodes and the GUI node used for shutdown."""
    config_dir = get_package_share_directory("study_orchestration") + "/config"
    controller_config_dir = (
        get_package_share_directory("control_node") + "/config"
    )
    mpc_config = controller_config_dir + "/mpc.yaml"
    state_feedback_config = controller_config_dir + "/state_feedback.yaml"
    controller_family = controller or "none"
    mpc_enabled = PythonExpression(["'", controller_family, "' == 'mpc'"])
    state_feedback_enabled = PythonExpression(
        ["'", controller_family, "' == 'state_feedback'"]
    )
    no_controller = PythonExpression(
        ["'", controller_family, "' == 'none'"]
    )
    parameters = [
        config_dir + "/study_base.yaml",
        config_dir + f"/study_{source}.yaml",
    ]
    scenario_overrides = {
        "task_file": config_dir + "/default_tasks.yaml",
        "input_source": source,
        "controller_family": controller_family,
        "estimator_state_policy": "persist_session",
        "require_controller_ready": require_system_ready,
        "require_estimator_ready": require_system_ready,
        "require_logger_ready": require_system_ready,
    }
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
    nodes.append(
        Node(
            package="study_orchestration",
            executable="experiment_mapper",
            name="experiment_mapper",
            output="screen",
            parameters=parameters,
        )
    )
    for condition, controller_parameters in (
        (IfCondition(mpc_enabled), [mpc_config]),
        (IfCondition(state_feedback_enabled), [state_feedback_config]),
        (IfCondition(no_controller), [{"max_control_amplitude": 0.0}]),
    ):
        nodes.append(
            Node(
                package="study_orchestration",
                executable="scenario_generator",
                name="scenario_generator",
                output="screen",
                condition=condition,
                parameters=[
                    *parameters,
                    *controller_parameters,
                    scenario_overrides,
                ],
            )
        )
    if controller is not None:
        controller_enabled = PythonExpression(
            ["'", controller, "' in ['mpc', 'state_feedback']"]
        )
        nodes.append(
            Node(
                package="control_node",
                executable="mpc_control_node",
                name="control_node",
                output="screen",
                condition=IfCondition(mpc_enabled),
                parameters=[
                    mpc_config,
                    {
                        "log_level": controller_log_level,
                    },
                ],
            )
        )
        nodes.append(
            Node(
                package="control_node",
                executable="state_feedback_control_node",
                name="control_node",
                output="screen",
                condition=IfCondition(state_feedback_enabled),
                parameters=[
                    state_feedback_config,
                    {
                        "log_level": controller_log_level,
                        "docking_enabled": ParameterValue(
                            docking_enabled, value_type=bool
                        ),
                    },
                ],
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
        # Controller-enabled mouse runs produce the same session/task metadata
        # and analysis schema as hardware runs, so they must be logged too.
        # Readiness is controlled independently by require_system_ready; the
        # lightweight mouse path therefore never waits for this node.
        nodes.append(
            Node(
                package="data_logger",
                executable="data_logger_node",
                name="data_logger_node",
                output="screen",
                condition=IfCondition(controller_enabled),
                parameters=[
                    {
                        "save_directory": "./logs",
                        "log_level": controller_log_level,
                    }
                ],
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
        parameters=[
            *parameters,
            {
                "require_system_ready": require_system_ready,
                "controller_family": controller_family,
            },
        ],
    )
    nodes.append(gui)
    return nodes, gui
