#!/usr/bin/env python3

"""Map raw Inverse3 or mouse-simulated state into experiment task coordinates."""

import rclpy
from geometry_msgs.msg import Point
from haply_msgs.msg import Inverse3State
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from study_orchestration.mapper_logic import (
    AnchoredDeltaMapper,
    MappingConfig,
    TaskPoint,
    map_identity,
)


class ExperimentMapper(Node):
    """Publish the task-frame cursor used by GUI, Scenario, Controller, and Logger."""

    VALID_MAPPING_MODES = {"identity", "anchored_delta"}

    def __init__(self):
        """Create mapper subscriptions and publishers."""
        super().__init__("experiment_mapper")

        self.declare_parameter("mapping_mode", "anchored_delta")
        self.declare_parameter("scale_x", 1.0)
        self.declare_parameter("scale_y", 1.0)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)

        self.mapping_mode = (
            str(self.get_parameter("mapping_mode").value).strip().lower()
        )
        if self.mapping_mode not in self.VALID_MAPPING_MODES:
            raise ValueError(
                f"mapping_mode must be one of {sorted(self.VALID_MAPPING_MODES)}"
            )

        config = MappingConfig(
            scale_x=float(self.get_parameter("scale_x").value),
            scale_y=float(self.get_parameter("scale_y").value),
            invert_x=bool(self.get_parameter("invert_x").value),
            invert_y=bool(self.get_parameter("invert_y").value),
        )
        self.anchored_mapper = AnchoredDeltaMapper(config)
        self.latest_raw_position: TaskPoint | None = None
        self.study_start_point: TaskPoint | None = None
        self.is_running = False
        self.anchor_pending = True

        self.cursor_pub = self.create_publisher(Point, "experiment_cursor_position", 10)
        self.create_subscription(
            Inverse3State, "inverse3_state", self._inverse3_state, 10
        )
        self.create_subscription(
            Point, "study_start_point", self._study_start_point, 10
        )
        self.create_subscription(Bool, "study_is_running", self._study_is_running, 10)

    def _inverse3_state(self, msg: Inverse3State) -> None:
        raw_position = self._from_point_msg(msg.position)
        self.latest_raw_position = raw_position
        self._capture_anchor_if_ready()

        if self.mapping_mode == "identity":
            self._publish_cursor(map_identity(raw_position))
            return

        mapped_position = self.anchored_mapper.map_position(raw_position)
        if mapped_position is not None:
            self._publish_cursor(mapped_position)

    def _study_start_point(self, msg: Point) -> None:
        next_start_point = self._from_point_msg(msg)
        if self.study_start_point is None or self._point_changed(
            self.study_start_point, next_start_point
        ):
            self.study_start_point = next_start_point
            self.anchor_pending = True
        self._capture_anchor_if_ready()

    def _study_is_running(self, msg: Bool) -> None:
        was_running = self.is_running
        self.is_running = bool(msg.data)
        if self.is_running and not was_running:
            self.anchor_pending = True
            self._capture_anchor_if_ready()
        elif not self.is_running:
            self.anchor_pending = True

    def _capture_anchor_if_ready(self) -> None:
        if (
            self.mapping_mode != "anchored_delta"
            or not self.anchor_pending
            or not self.is_running
            or self.latest_raw_position is None
            or self.study_start_point is None
        ):
            return

        self.anchored_mapper.capture_anchor(
            raw_position=self.latest_raw_position,
            task_start=self.study_start_point,
        )
        self.anchor_pending = False
        self.get_logger().info(
            "Captured mapper anchor: "
            f"raw=({self.latest_raw_position.x:.4f}, "
            f"{self.latest_raw_position.y:.4f}), "
            f"task_start=({self.study_start_point.x:.4f}, "
            f"{self.study_start_point.y:.4f})"
        )

    def _publish_cursor(self, position: TaskPoint) -> None:
        msg = Point()
        msg.x = position.x
        msg.y = position.y
        msg.z = position.z
        self.cursor_pub.publish(msg)

    def _from_point_msg(self, msg: Point) -> TaskPoint:
        return TaskPoint(x=float(msg.x), y=float(msg.y), z=float(msg.z))

    def _point_changed(self, first: TaskPoint, second: TaskPoint) -> bool:
        return (
            abs(first.x - second.x) > 1e-6
            or abs(first.y - second.y) > 1e-6
            or abs(first.z - second.z) > 1e-6
        )


def main(args=None):
    """Start the Experiment Mapper."""
    rclpy.init(args=args)
    node = ExperimentMapper()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
