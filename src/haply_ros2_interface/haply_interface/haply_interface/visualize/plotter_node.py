#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from haply_msgs.msg import HaplyControl, HaplyState
from geometry_msgs.msg import Point

import threading
import time
from collections import deque
import matplotlib


class TargetVsActualAndErrorPlotterNode(Node):
    """Node for plotting the actual and target position of the device"""

    def __init__(self):
        super().__init__('plotter_node')

        # Parameters
        self.declare_parameter("plot_window", 30.0)   # seconds in rolling window
        self.plot_window = float(self.get_parameter("plot_window").value)

        # Latest target
        self.latest_target = Point(x=0.0, y=0.0, z=0.0)
        self.has_target = False

        # Buffers
        self.t0 = time.time()
        self.lock = threading.Lock()
        self.t_buf = deque()

        # actual position buffers
        self.xa_buf, self.ya_buf, self.za_buf = deque(), deque(), deque()
        # target position buffers
        self.xt_buf, self.yt_buf, self.zt_buf = deque(), deque(), deque()
        # % error buffers
        self.xe_buf, self.ye_buf, self.ze_buf = deque(), deque(), deque()

        # Subscriptions
        self.create_subscription(HaplyControl, 'haply_target', self.on_target, 10)
        self.create_subscription(HaplyState,   'haply_state',  self.on_state,  10)

        self.get_logger().info(
            f"Plotter started. plot_window={self.plot_window:.1f}s"
        )

    def on_target(self, msg: HaplyControl):
        """Update stored target when position control is used."""
        if bool(msg.use_position):
            self.latest_target.x = float(msg.target_position.x)
            self.latest_target.y = float(msg.target_position.y)
            self.latest_target.z = float(msg.target_position.z)
            self.has_target = True

    def on_state(self, msg: HaplyState):
        """Append actual, target, and % error samples at this timestamp."""
        t = time.time() - self.t0
        ax, ay, az = float(msg.position.x), float(msg.position.y), float(msg.position.z)

        if self.has_target:
            tx, ty, tz = self.latest_target.x, self.latest_target.y, self.latest_target.z
        else:
            tx, ty, tz = ax, ay, az

        ex = ((tx - ax) / tx * 100.0) if abs(tx) > 1e-9 else 0.0
        ey = ((ty - ay) / ty * 100.0) if abs(ty) > 1e-9 else 0.0
        ez = ((tz - az) / tz * 100.0) if abs(tz) > 1e-9 else 0.0

        with self.lock:
            self.t_buf.append(t)
            self.xa_buf.append(ax); self.ya_buf.append(ay); self.za_buf.append(az)
            self.xt_buf.append(tx); self.yt_buf.append(ty); self.zt_buf.append(tz)
            self.xe_buf.append(ex); self.ye_buf.append(ey); self.ze_buf.append(ez)

            # trim to rolling window
            t_min = t - self.plot_window
            while self.t_buf and self.t_buf[0] < t_min:
                self.t_buf.popleft()
                self.xa_buf.popleft(); self.ya_buf.popleft(); self.za_buf.popleft()
                self.xt_buf.popleft(); self.yt_buf.popleft(); self.zt_buf.popleft()
                self.xe_buf.popleft(); self.ye_buf.popleft(); self.ze_buf.popleft()

    def run_plot(self):
        """Live plot in 3x2 grid: actual vs target | % error."""
        try:
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt

            plt.ion()
            fig, axes = plt.subplots(nrows=3, ncols=2, sharex=True, figsize=(12, 9))

            # Row 1: X
            ax_x_val, ax_x_err = axes[0]
            (line_xa,) = ax_x_val.plot([], [], label="x(t)")
            (line_xt,) = ax_x_val.plot([], [], linestyle="--", label="x_target(t)")
            ax_x_val.set_ylabel("x [m]")
            ax_x_val.grid(True); ax_x_val.legend(loc="upper right")

            (line_xe,) = ax_x_err.plot([], [])  # no label
            ax_x_err.set_ylabel("x error [%]")
            ax_x_err.grid(True)

            # Row 2: Y
            ax_y_val, ax_y_err = axes[1]
            (line_ya,) = ax_y_val.plot([], [], label="y(t)")
            (line_yt,) = ax_y_val.plot([], [], linestyle="--", label="y_target(t)")
            ax_y_val.set_ylabel("y [m]")
            ax_y_val.grid(True); ax_y_val.legend(loc="upper right")

            (line_ye,) = ax_y_err.plot([], [])
            ax_y_err.set_ylabel("y error [%]")
            ax_y_err.grid(True)

            # Row 3: Z
            ax_z_val, ax_z_err = axes[2]
            (line_za,) = ax_z_val.plot([], [], label="z(t)")
            (line_zt,) = ax_z_val.plot([], [], linestyle="--", label="z_target(t)")
            ax_z_val.set_ylabel("z [m]")
            ax_z_val.grid(True); ax_z_val.legend(loc="upper right")

            (line_ze,) = ax_z_err.plot([], [])
            ax_z_err.set_ylabel("z error [%]")
            ax_z_err.grid(True)

            ax_z_val.set_xlabel("Time [s]")
            ax_z_err.set_xlabel("Time [s]")
            fig.suptitle("Haply: target vs actual (left) and % error (right) for X, Y, Z")

            def set_ylim(ax, *series_lists):
                vals = []
                for s in series_lists:
                    vals.extend(s)
                if not vals:
                    return
                vmin, vmax = min(vals), max(vals)
                pad = 0.05 * max(1.0, (vmax - vmin))
                ax.set_ylim(vmin - pad, vmax + pad)

            while rclpy.ok():
                with self.lock:
                    t  = list(self.t_buf)
                    xa, ya, za = list(self.xa_buf), list(self.ya_buf), list(self.za_buf)
                    xt, yt, zt = list(self.xt_buf), list(self.yt_buf), list(self.zt_buf)
                    xe, ye, ze = list(self.xe_buf), list(self.ye_buf), list(self.ze_buf)

                if t:
                    t_now, t_min = t[-1], t[-1] - self.plot_window
                    for row in axes:
                        for ax in row:
                            ax.set_xlim(t_min, t_now)

                    # update lines
                    line_xa.set_data(t, xa); line_xt.set_data(t, xt)
                    line_ya.set_data(t, ya); line_yt.set_data(t, yt)
                    line_za.set_data(t, za); line_zt.set_data(t, zt)
                    line_xe.set_data(t, xe); line_ye.set_data(t, ye); line_ze.set_data(t, ze)

                    # set limits
                    set_ylim(ax_x_val, xa, xt)
                    set_ylim(ax_y_val, ya, yt)
                    set_ylim(ax_z_val, za, zt)
                    set_ylim(ax_x_err, xe)
                    set_ylim(ax_y_err, ye)
                    set_ylim(ax_z_err, ze)

                plt.pause(0.05)

            plt.ioff()
            plt.show(block=False)
        except Exception as e:
            self.get_logger().error(f"Plotting error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TargetVsActualAndErrorPlotterNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_plot()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
