import logging
import os
import sys
from pathlib import Path

# Add the parent directory to the path so the package can be imported
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from estimator_node.estimator.rls_estimator import RLSEstimator


class MockPoint:
    """Mock geometry_msgs/Point for testing without ROS 2 context."""

    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z


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


estimator_logger = create_logger("test_rls_estimator_compute.log")


def test_initialization(logger):
    logger.info("Starting Initialization Test")
    estimator = RLSEstimator()

    # Simulate a start point of (-6.0, 0.0)
    start_point = MockPoint(-6.0, 0.0)
    estimator.initialize_from_start_point(start_point)

    matrix = estimator.get_matrix()
    logger.info(f"Initialized matrix: \n{matrix}")

    # Since norm of (-6, 0) is 6.0, the initial guesses should be 6.0
    assert matrix[0, 0] == 6.0
    assert matrix[0, 1] == 6.0
    assert matrix[1, 2] == 6.0
    assert matrix[1, 3] == 6.0


def test_behavioral_state_careful(logger):
    """
    Simulates a 'careful' behavioral state.
    Defined by: Low velocities, very low accelerations, tight tracking errors
    """
    logger.info("--- Starting Careful Behavioral State Test ---")
    estimator = RLSEstimator()
    estimator.initialize_from_start_point(MockPoint(1.0, 1.0))

    # Small error, low velocity, near-zero acceleration
    ex, vx, ey, vy = 0.05, 0.02, 0.05, 0.02
    ax, ay = 0.001, 0.001

    for step in range(10):
        estimator.update(ex, vx, ey, vy, ax, ay)
        matrix = estimator.get_matrix()

        logger.info(
            f"Careful Step {step}:\n"
            f"  Pos Error: ({ex:.4f}, {ey:.4f}) | "
            f"Vel: ({vx:.4f}, {vy:.4f}) | "
            f"Acc: ({ax:.4f}, {ay:.4f})\n"
            f"  Kh matrix=\n{matrix}"
        )

        # Smooth, tiny corrections
        ex *= 0.95
        ey *= 0.95

    assert matrix.shape == (2, 4)


def test_behavioral_state_normal(logger):
    """
    Simulates a 'normal' behavioral state
    Defined by: Moderate velocities, standard accelerations, average tracking errors
    """
    logger.info("--- Starting Normal Behavioral State Test ---")
    estimator = RLSEstimator()
    estimator.initialize_from_start_point(MockPoint(1.0, 1.0))

    # Moderate error, medium velocity, standard acceleration
    ex, vx, ey, vy = 0.5, 0.2, 0.5, 0.2
    ax, ay = 0.05, -0.05

    for step in range(10):
        estimator.update(ex, vx, ey, vy, ax, ay)
        matrix = estimator.get_matrix()

        logger.info(
            f"Normal Step {step}:\n"
            f"  Pos Error: ({ex:.4f}, {ey:.4f}) | "
            f"Vel: ({vx:.4f}, {vy:.4f}) | "
            f"Acc: ({ax:.4f}, {ay:.4f})\n"
            f"  Kh matrix=\n{matrix}"
        )

        # Standard corrections
        ex -= 0.05
        ey -= 0.05
        vx += 0.01
        vy -= 0.01

    assert matrix.shape == (2, 4)


def test_behavioral_state_aggressive(logger):
    """
    Simulates an 'aggressive' behavioral state
    Defined by: High tracking errors, fast velocities, sharp, erratic accelerations
    """
    logger.info("--- Starting Aggressive Behavioral State Test ---")
    estimator = RLSEstimator()
    estimator.initialize_from_start_point(MockPoint(1.0, 1.0))

    # Large error, high velocity, sharp acceleration spikes
    ex, vx, ey, vy = 2.0, 1.5, 2.0, -1.5
    ax, ay = 0.8, -0.8

    for step in range(10):
        estimator.update(ex, vx, ey, vy, ax, ay)
        matrix = estimator.get_matrix()

        logger.info(
            f"Aggressive Step {step}:\n"
            f"  Pos Error: ({ex:.4f}, {ey:.4f}) | "
            f"Vel: ({vx:.4f}, {vy:.4f}) | "
            f"Acc: ({ax:.4f}, {ay:.4f})\n"
            f"  Kh matrix=\n{matrix}"
        )

        # Erratic corrections (overshooting behavior)
        ex = -ex * 0.8
        ey = -ey * 0.8
        ax = -ax * 1.1
        ay = -ay * 1.1

    assert matrix.shape == (2, 4)


if __name__ == "__main__":
    test_initialization(estimator_logger)
    test_behavioral_state_careful(estimator_logger)
    test_behavioral_state_normal(estimator_logger)
    test_behavioral_state_aggressive(estimator_logger)
