"""Pure mapping helpers used by the Experiment Mapper node."""

from dataclasses import dataclass


@dataclass(frozen=True)
class TaskPoint:
    """Simple point representation for raw and mapped positions."""

    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True)
class MappingConfig:
    """Scale and axis direction configuration."""

    scale_x: float = 1.0
    scale_y: float = 1.0
    invert_x: bool = False
    invert_y: bool = False


def map_identity(raw_position: TaskPoint) -> TaskPoint:
    """Map raw coordinates directly into task coordinates."""
    return TaskPoint(raw_position.x, raw_position.y, raw_position.z)


class AnchoredDeltaMapper:
    """Map raw displacement onto a task-frame start point."""

    def __init__(self, config: MappingConfig):
        self.config = config
        self.raw_anchor: TaskPoint | None = None
        self.task_anchor: TaskPoint | None = None

    @property
    def is_ready(self) -> bool:
        """Return true after both anchors have been captured."""
        return self.raw_anchor is not None and self.task_anchor is not None

    def capture_anchor(self, raw_position: TaskPoint, task_start: TaskPoint) -> None:
        """Anchor the current raw pose to the current task start point."""
        self.raw_anchor = raw_position
        self.task_anchor = task_start

    def map_position(self, raw_position: TaskPoint) -> TaskPoint | None:
        """Return the mapped task position, or None before anchors exist."""
        if self.raw_anchor is None or self.task_anchor is None:
            return None

        sign_x = -1.0 if self.config.invert_x else 1.0
        sign_y = -1.0 if self.config.invert_y else 1.0
        dx = (raw_position.x - self.raw_anchor.x) * self.config.scale_x * sign_x
        dy = (raw_position.y - self.raw_anchor.y) * self.config.scale_y * sign_y
        return TaskPoint(
            x=self.task_anchor.x + dx,
            y=self.task_anchor.y + dy,
            z=0.0,
        )
