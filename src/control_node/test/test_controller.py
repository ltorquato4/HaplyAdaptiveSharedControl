import logging
import os
import sys
from math import sqrt
from pathlib import Path

# Fix path to resolve parent imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from control_node.mpc_controller.adaptive_mpc_controller import AdaptiveMpcController
from control_node.mpc_controller.mpc_controller import MpcController
from control_node.state_feedback_controller.adaptive_state_feedback_controller import (
    AdaptiveStateFeedbackController,
)
from control_node.state_feedback_controller.state_feedback_controller import (
    StateFeedbackController,
)

# --- Configuration Constants ---
START_POINT = [-6, 0]
START_POINT_ALT = [23, 21]
END_POINT = [10, 10]
DT = 0.1
DISTANCE_THRESHOLD = 0.2
STEP_SIZE = 0.005


def get_distance(p1, p2):
    """Calculates Euclidean distance between two 2D points."""
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def create_logger(log_filename):
    """Creates a localized clean logger that writes directly to a file."""
    logger = logging.getLogger(log_filename)
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    log_file = Path(__file__).resolve().parent / log_filename
    handler = logging.FileHandler(log_file, mode="w")
    handler.setFormatter(logging.Formatter("%(message)s"))

    logger.addHandler(handler)
    logger.propagate = False
    return logger


# --- Core Test Suites ---

def run_compute_control_test(logger, controller, start_point, end_point):
    """Simulates a path tracking loop and logs output controls."""
    current_point = start_point.copy()
    logger.info("Starting Compute Control Test")
    
    step = 0
    distance = get_distance(current_point, end_point)

    while distance > DISTANCE_THRESHOLD:
        controller.compute_control(current_point)
        distance = get_distance(current_point, end_point)

        logger.info(
            f"step {step} | current point {current_point} | "
            f"control {controller.u_a} | distance {distance:.4f}"
        )

        # Update simulated environment state
        current_point[0] += controller.u_a[0] * STEP_SIZE
        current_point[1] += controller.u_a[1] * STEP_SIZE
        step += 1

    logger.info(f"Target reached after {step} steps. Final position = {current_point}")
    assert len(controller.u_a) == 2, "Control vector dimension mismatch!"


def run_adaptive_gain_test(logger, controller, start_point, end_point):
    """Simulates adaptive gain shifts using a fake human interaction matrix."""
    current_point = start_point.copy()
    logger.info("Starting Adaptivity Test")

    step = 0
    while get_distance(current_point, end_point) > DISTANCE_THRESHOLD:
        # Identity matrix for simulated human gain matrix
        k_h = [[1.0, 0.0], [0.0, 1.0]]

        controller.current_point = current_point
        controller.adapt(k_h)

        logger.info(
            f"step={step} | position={current_point} | "
            f"progress={controller.progress_along_path:.3f} | "
            f"adapted_gain={controller.publish_control_parameter()}"
        )

        controller.compute_control(current_point)

        current_point[0] += controller.u_a[0] * STEP_SIZE
        current_point[1] += controller.u_a[1] * STEP_SIZE
        step += 1

    logger.info(f"Target reached after {step} steps. Final position = {current_point}")


# --- Execution Entrypoint ---

def main():
    # 1. Initialize Loggers
    loggers = {
        "sf_compute": create_logger("test_state_feedback_controller_compute_control.log"),
        "sf_adapt": create_logger("test_state_feedback_controller_adapt.log"),
        "mpc_compute_0": create_logger("test_mpc_compute_control.log"),
        "mpc_compute_1": create_logger("test_mpc_compute_control1.log"),
        "mpc_adapt": create_logger("test_mpc_controller_adapt.log"),
    }

    # 2. Run Test Case: MPC on Standard Starting Point
    mpc_0 = MpcController(START_POINT, END_POINT, DT, max_control=(100, 100))
    run_compute_control_test(loggers["mpc_compute_0"], mpc_0, START_POINT, END_POINT)
    mpc_0.destroy()

    # 3. Run Test Case: MPC on Alternative Starting Point
    mpc_0 = MpcController(START_POINT_ALT, END_POINT, DT)
    run_compute_control_test(loggers["mpc_compute_1"], mpc_0, START_POINT_ALT, END_POINT)
    mpc_0.destroy()

    # Note: To run state feedback or adaptive test suites in the future, 
    # simply initialize them here and pass them into the runners:
    # 
    # sf_adaptive = AdaptiveStateFeedbackController(START_POINT, END_POINT, DT)
    # run_adaptive_gain_test(loggers["sf_adapt"], sf_adaptive, START_POINT, END_POINT)


if __name__ == "__main__":
    main()