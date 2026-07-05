import logging
import os
import sys
from pathlib import Path

# Add the parent directory to the path so the package can be imported
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from rls_estimator_node.estimator.rls_estimator import RLSEstimator


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


def test_update_loop(logger):
    logger.info("Starting Update Loop Test")
    estimator = RLSEstimator()

    # Initialize to avoid zero-state starting issues
    estimator.initialize_from_start_point(MockPoint(1.0, 1.0))

    # Simulate errors, velocities, and accelerations for 10 steps
    ex, vx, ey, vy = 1.0, 0.1, 1.0, 0.1
    ax, ay = 0.01, -0.01

    for step in range(10):
        estimator.update(ex, vx, ey, vy, ax, ay)
        matrix = estimator.get_matrix()
        logger.info(f"step {step} Kh matrix=\n{matrix}")

        # Simulate slight changes in kinematics over time
        ex -= 0.1
        ey -= 0.1
        vx += 0.01
        vy += 0.01

    assert matrix.shape == (2, 4)


# Execute tests if run as a standalone script
test_initialization(estimator_logger)
test_update_loop(estimator_logger)
