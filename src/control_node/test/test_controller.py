import logging
import os
import sys
from math import sqrt
from pathlib import Path

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from control_node.mpc_controller.adaptive_mpc_controller import AdaptiveMpcController
from control_node.mpc_controller.mpc_controller import MpcController
from control_node.state_feedback_controller.adaptive_state_feedback_controller import (
    AdaptiveStateFeedbackController,
)
from control_node.state_feedback_controller.state_feedback_controller import (
    StateFeedbackController,
)

START_POINT = [-6, 0]
END_POINT = [10, 10]
DT = 0.1


def create_logger(log_filename):
    logger = logging.getLogger(log_filename)
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    log_file = Path(__file__).resolve().parent / log_filename

    handler = logging.FileHandler(log_file, mode="w")
    handler.setFormatter(logging.Formatter("%(message)s"))

    logger.addHandler(handler)
    logger.propagate = False

    return logger


state_feedback_compute_logger = create_logger(
    "test_state_feedback_controller_compute_control.log"
)
state_feedback_adapt_logger = create_logger("test_state_feedback_controller_adapt.log")
mpc_compute_logger = create_logger("test_mpc_compute_control.log")
mpc_adapt_logger = create_logger("test_mpc_controller_adapt.log")

state_feedback_controller = StateFeedbackController(START_POINT, END_POINT, DT)
adaptive_state_feedback_controller = AdaptiveStateFeedbackController(
    START_POINT, END_POINT, DT
)
mpc_controller = MpcController(START_POINT, END_POINT, DT, max_control=(100, 100))
adaptive_mpc_controller = AdaptiveMpcController(
    START_POINT, END_POINT, DT, max_control=(100, 100)
)


def test_compute_control(logger, controller):
    current_point = START_POINT.copy()

    logger.info("Starting Compute Control Test")
    step = 0

    while (
        sqrt(
            (current_point[0] - END_POINT[0]) ** 2
            + (current_point[1] - END_POINT[1]) ** 2
        )
        > 0.2
    ):
        controller.compute_control(current_point)

        distance = sqrt(
            (current_point[0] - END_POINT[0]) ** 2
            + (current_point[1] - END_POINT[1]) ** 2
        )

        logger.info(
            f"step {step} current point{current_point} control {controller.u_a} "
            f"distance {distance}"
        )

        current_point[0] += controller.u_a[0] * 0.005
        current_point[1] += controller.u_a[1] * 0.005

        step += 1

    logger.info(f"Target reached after {step} steps. Final position = {current_point}")

    assert len(controller.u_a) == 2


def test_adaptive_gain(logger, controller):

    current_point = START_POINT.copy()

    logger.info("Starting Adaptivity Test")

    step = 0

    while (
        sqrt(
            (current_point[0] - END_POINT[0]) ** 2
            + (current_point[1] - END_POINT[1]) ** 2
        )
        > 0.2
    ):
        K_h = [[1.0, 0.0], [0.0, 1.0]]

        controller.current_point = current_point
        controller.adapt(K_h)

        logger.info(
            f"step={step} "
            f"position={current_point} "
            f"progress={controller.progress_along_path:.3f} "
            f"human_gain={K_h} "
            f"adapted_gain={controller.publish_control_parameter()}"
        )

        controller.compute_control(current_point)

        current_point[0] += controller.u_a[0] * 0.005
        current_point[1] += controller.u_a[1] * 0.005

        step += 1

    logger.info(f"Target reached after {step} steps. Final position = {current_point}")

    assert True


test_compute_control(state_feedback_compute_logger, state_feedback_controller)
test_adaptive_gain(state_feedback_adapt_logger, adaptive_state_feedback_controller)
test_compute_control(mpc_compute_logger, mpc_controller)
test_adaptive_gain(mpc_adapt_logger, adaptive_mpc_controller)
