#!/usr/bin/env python3

import random

import rclpy
from geometry_msgs.msg import Point, Vector3

# Change import if your package name differs
from haply_msgs.msg import HaplyState
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String


class TestDataLoggerNode(Node):
    def __init__(self):

        super().__init__("test_data_logger_node")

        self.declare_parameter("log_level", "debug")
        self.log_level = (
            self.get_parameter("log_level").get_parameter_value().string_value
        )
        self.get_logger().set_level(self._resolve_log_level(self.log_level))

        #
        # Publishers
        #

        self.pub_running = self.create_publisher(Bool, "/study_is_running", 10)

        self.pub_phase = self.create_publisher(String, "/study_phase", 10)

        self.pub_mode = self.create_publisher(String, "/study_controller_mode", 10)

        self.pub_start = self.create_publisher(Point, "/study_start_point", 10)

        self.pub_end = self.create_publisher(Point, "/study_end_point", 10)

        self.pub_cursor = self.create_publisher(
            Point, "/experiment_cursor_position", 10
        )

        self.pub_haply = self.create_publisher(HaplyState, "/haply_state", 10)

        self.pub_Kh = self.create_publisher(Float64, "/estimation/K_h", 10)

        self.pub_uh = self.create_publisher(Vector3, "/estimation/u_h", 10)

        self.pub_Ka = self.create_publisher(Float64, "/control/K_a", 10)

        self.pub_Ua = self.create_publisher(Vector3, "/control/U_a", 10)

        self.pub_endpoint = self.create_publisher(Bool, "/study_endpoint_reached", 10)

        #
        # State
        #

        self.study_running = False

        #
        # Timers
        #

        # Publish data at 100 Hz
        self.create_timer(0.01, self.publish_random_data)

        # Toggle logger state every 10 sec
        self.create_timer(10.0, self.toggle_running)

        self.get_logger().info("Random test publisher started.")

    def _resolve_log_level(self, log_level_name):

        log_levels = {
            "debug": LoggingSeverity.DEBUG,
            "info": LoggingSeverity.INFO,
            "warn": LoggingSeverity.WARN,
            "warning": LoggingSeverity.WARN,
            "error": LoggingSeverity.ERROR,
            "fatal": LoggingSeverity.FATAL,
        }

        normalized = str(log_level_name).strip().lower()

        return log_levels.get(normalized, LoggingSeverity.DEBUG)

    def _message_to_debug_value(self, msg):

        if hasattr(msg, "data"):
            return msg.data

        if hasattr(msg, "position") and hasattr(msg, "velocity"):
            return {
                "position": self._message_to_debug_value(msg.position),
                "velocity": self._message_to_debug_value(msg.velocity),
            }

        if all(hasattr(msg, attr) for attr in ("x", "y", "z")):
            return {
                "x": msg.x,
                "y": msg.y,
                "z": msg.z,
            }

        return repr(msg)

    def _log_sent_message(self, topic_name, msg):

        self.get_logger().debug(
            f"sent {topic_name}: {self._message_to_debug_value(msg)}"
        )

    # ---------------------------------------------------------
    # Utilities
    # ---------------------------------------------------------

    def rand(self, low=-1.0, high=1.0):
        return random.uniform(low, high)

    def random_point(self):

        msg = Point()

        msg.x = self.rand(-0.5, 0.5)
        msg.y = self.rand(-0.5, 0.5)
        msg.z = self.rand(-0.5, 0.5)

        return msg

    def random_vector(self):

        msg = Vector3()

        msg.x = self.rand(-20, 20)
        msg.y = self.rand(-20, 20)
        msg.z = self.rand(-20, 20)

        return msg

    # ---------------------------------------------------------
    # Toggle recording
    # ---------------------------------------------------------

    def toggle_running(self):

        self.study_running = not self.study_running

        msg = Bool()
        msg.data = self.study_running

        self.pub_running.publish(msg)
        self._log_sent_message("/study_is_running", msg)

        self.get_logger().info(f"study_is_running = {self.study_running}")

    # ---------------------------------------------------------
    # Publish random data
    # ---------------------------------------------------------

    def publish_random_data(self):

        #
        # Start point
        #

        start = self.random_point()
        self.pub_start.publish(start)
        self._log_sent_message("/study_start_point", start)

        #
        # End point
        #

        end = self.random_point()
        self.pub_end.publish(end)
        self._log_sent_message("/study_end_point", end)

        #
        # Cursor
        #

        cursor = self.random_point()
        self.pub_cursor.publish(cursor)
        self._log_sent_message("/experiment_cursor_position", cursor)

        #
        # u_h
        #

        uh = self.random_vector()
        self.pub_uh.publish(uh)
        self._log_sent_message("/estimation/u_h", uh)

        #
        # U_a
        #

        ua = self.random_vector()
        self.pub_Ua.publish(ua)
        self._log_sent_message("/control/U_a", ua)

        #
        # K_h
        #

        kh = Float64()
        kh.data = random.uniform(0.0, 1000.0)

        self.pub_Kh.publish(kh)
        self._log_sent_message("/estimation/K_h", kh)

        #
        # K_a
        #

        ka = Float64()
        ka.data = random.uniform(0.0, 1000.0)

        self.pub_Ka.publish(ka)
        self._log_sent_message("/control/K_a", ka)

        #
        # Phase
        #

        phase = String()

        phase.data = random.choice(["aggressive", "normal", "careful"])

        self.pub_phase.publish(phase)
        self._log_sent_message("/study_phase", phase)

        #
        # Controller mode
        #

        mode = String()

        mode.data = random.choice(["adaptive", "fixed"])

        self.pub_mode.publish(mode)
        self._log_sent_message("/study_controller_mode", mode)

        #
        # Endpoint reached
        #

        endpoint = Bool()
        endpoint.data = random.choice([True, False])

        self.pub_endpoint.publish(endpoint)
        self._log_sent_message("/study_endpoint_reached", endpoint)

        #
        # Random HaplyState
        #
        # Assumes HaplyState contains:
        #   position
        #   velocity
        #

        haply = HaplyState()

        try:
            haply.position.x = self.rand()
            haply.position.y = self.rand()
            haply.position.z = self.rand()

            haply.velocity.x = self.rand()
            haply.velocity.y = self.rand()
            haply.velocity.z = self.rand()

            self.pub_haply.publish(haply)
            self._log_sent_message("/haply_state", haply)

        except Exception:
            #
            # If your HaplyState definition differs,
            # comment this block and populate fields
            # according to your message definition.
            #
            pass


def main(args=None):

    rclpy.init(args=args)

    node = TestDataLoggerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
