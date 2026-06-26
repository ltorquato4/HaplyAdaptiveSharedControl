import os
import sys
import logging
from pathlib import Path

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
from math import sqrt

import matplotlib.pyplot as plt

from control_node.state_feedback_controller.state_feedback_controller import StateFeedbackController
from control_node.state_feedback_controller.adaptive_state_feedback_controller import AdaptiveStateFeedbackController

START_POINT = [-6, 0]
END_POINT = [10, 10]
cntr = StateFeedbackController(START_POINT, END_POINT, dt=0.1)

LOG_FILE = Path(__file__).resolve().with_suffix('.log')
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.handlers.clear()
file_handler = logging.FileHandler(LOG_FILE, mode='w')
file_handler.setFormatter(logging.Formatter('%(message)s'))
logger.addHandler(file_handler)
logger.propagate = False

def test_compute_control():
    current_point = START_POINT.copy()

    logger.info("Starting Compute Control Test")

    t = []
    xs, ys = [], []
    u_x, u_y = [], []

    step = 0

    while sqrt((current_point[0] - END_POINT[0])**2 +
               (current_point[1] - END_POINT[1])**2) > 0.2:

        cntr.compute_control(current_point)

        t.append(step)
        xs.append(current_point[0])
        ys.append(current_point[1])
        u_x.append(cntr.u_a[0])
        u_y.append(cntr.u_a[1])

        current_point[0] += cntr.u_a[0] * 0.005
        current_point[1] += cntr.u_a[1] * 0.005

        step += 1

    # ---- 4 subplots in one window ----
    fig, axes = plt.subplots(4, 1, figsize=(8, 14))

    # X position
    axes[0].plot(t, xs, label="x position")
    axes[0].hlines(END_POINT[0], t[0], t[-1], linestyles='dashed', label="x target")
    axes[0].set_xlabel("time step")
    axes[0].set_ylabel("x")
    axes[0].legend()
    axes[0].set_title("X Position vs Time")

    # Y position
    axes[1].plot(t, ys, label="y position")
    axes[1].hlines(END_POINT[1], t[0], t[-1], linestyles='dashed', label="y target")
    axes[1].set_xlabel("time step")
    axes[1].set_ylabel("y")
    axes[1].legend()
    axes[1].set_title("Y Position vs Time")

    # Control u_x
    axes[2].plot(t, u_x, label="u_x")
    axes[2].set_xlabel("time step")
    axes[2].set_ylabel("u_x")
    axes[2].legend()
    axes[2].set_title("u_x vs Time")

    # Control u_y
    axes[3].plot(t, u_y, label="u_y")
    axes[3].set_xlabel("time step")
    axes[3].set_ylabel("u_y")
    axes[3].legend()
    axes[3].set_title("u_y vs Time")

    plt.tight_layout()
    plt.show()

    assert len(cntr.u_a) == 2

def test_adaptive_gain():
    controller = AdaptiveStateFeedbackController(START_POINT, END_POINT, dt=0.1)

    current_point = START_POINT.copy()

    logger.info("Starting Adaptivity Test")

    while sqrt((current_point[0] - END_POINT[0])**2 +
               (current_point[1] - END_POINT[1])**2) > 0.2:

        # Simulated human gain matrix (example: constant)
        K_h = [[1.0, 0.0],
               [0.0, 1.0]]

        controller.current_point = current_point
        controller.adapt_gain(K_h)

        logger.info(
            f'position of current point {controller.progress_along_path:.3f} '
            f'current human estimation {K_h} '
            f'adapted gain {controller.K_p.tolist()}'
        )

        # Apply control step
        controller.compute_control(current_point)
        current_point[0] += controller.u_a[0] * 0.005
        current_point[1] += controller.u_a[1] * 0.005

    assert True