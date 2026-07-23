import logging

from estimator_node.estimator.rls_estimator import RLSEstimator


class MockPoint:
    """Mock geometry_msgs/Point for testing without ROS 2 context."""

    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z


def create_logger(logger_name):
    """Create a quiet in-memory test logger without writing into the source tree."""
    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    logger.addHandler(logging.NullHandler())
    logger.propagate = False
    return logger


estimator_logger = create_logger("test_rls_estimator_compute.log")


def test_initialization():
    logger = estimator_logger
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


def test_behavioral_state_careful():
    """Simulate the existing low-motion careful input profile."""
    logger = estimator_logger
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


def test_behavioral_state_normal():
    """Simulate the existing moderate-motion normal input profile."""
    logger = estimator_logger
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


def test_behavioral_state_aggressive():
    """Simulate the existing high-motion aggressive input profile."""
    logger = estimator_logger
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
    test_initialization()
    test_behavioral_state_careful()
    test_behavioral_state_normal()
    test_behavioral_state_aggressive()
