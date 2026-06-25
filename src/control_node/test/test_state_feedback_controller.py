import matplotlib.pyplot as plt
from math import sqrt
from control_node.controller.controller import Controller

START_POINT = [-6, 0]
END_POINT = [10, 10]
cntr = Controller(START_POINT, END_POINT, dt=0.1)

def test_compute_control():
    current_point = START_POINT

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