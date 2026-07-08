import numpy as np


class RLSEstimator:
    def __init__(self):

        # x-axis estimator
        self.theta_x = np.array([[1.0], [1.0]])

        self.P_x = np.eye(2) * 1000.0

        # y-axis estimator
        self.theta_y = np.array([[1.0], [1.0]])

        self.P_y = np.eye(2) * 1000.0

        self.lam = 0.995

    def initialize_from_start_point(self, start_point):

        dist = np.linalg.norm([start_point.x, start_point.y])

        guess = max(dist, 0.1)

        self.theta_x = np.array([[guess], [guess]])

        self.theta_y = np.array([[guess], [guess]])

    def _update_axis(self, theta, P, phi, y):

        phi = phi.reshape((2, 1))

        gain = (P @ phi) / (self.lam + phi.T @ P @ phi)

        prediction = phi.T @ theta

        theta = theta + gain * (y - prediction)

        P = (P - gain @ phi.T @ P) / self.lam

        return theta, P

    def update(self, ex, vx, ey, vy, ax, ay):

        phi_x = np.array([ex, vx])

        phi_y = np.array([ey, vy])

        self.theta_x, self.P_x = self._update_axis(self.theta_x, self.P_x, phi_x, ax)

        self.theta_y, self.P_y = self._update_axis(self.theta_y, self.P_y, phi_y, ay)

    def get_matrix(self):

        kp_x = float(self.theta_x[0, 0])
        kd_x = float(self.theta_x[1, 0])

        kp_y = float(self.theta_y[0, 0])
        kd_y = float(self.theta_y[1, 0])

        return np.array([[kp_x, kd_x, 0.0, 0.0], [0.0, 0.0, kp_y, kd_y]])
