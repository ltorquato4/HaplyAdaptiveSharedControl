"""Pure mapping helpers used by the Experiment Mapper node."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TaskPoint:
    """Simple point representation for raw and mapped positions."""

    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True)
class MappingConfig:
    """Scale, axis direction, and raw-workspace clamping configuration.

    Raw workspace bounds (raw_x_min/max, raw_second_min/max) are expressed as
    **deltas from the anchor position** in metres.  They are only applied when
    ``clamp_raw=True``.

    Typical Haply 2-DoF setup (use_z_as_y=True):
      - raw_x delta:       left ↔ right,  e.g. -0.10 … +0.10
      - raw_z delta:       down ↔ up,     e.g.  0.00 … +0.15
        (constrain to ≥ 0 so the user only moves upward from the rest position)
      - raw_y (Haply depth) is ignored for mapping; y is naturally constrained
        to 0 → negative by the device geometry.
    """

    scale_x: float = 1.0
    scale_y: float = 1.0
    invert_x: bool = False
    invert_y: bool = False
    use_z_as_y: bool = True   # Use Haply z (up/down) as task-y instead of Haply y (depth)

    # --- Raw workspace clamping (deltas from anchor, in metres) ---
    clamp_raw: bool = False   # Set True to enforce the bounds below
    raw_x_min: float = -0.10  # Haply x delta lower bound (left)
    raw_x_max: float = 0.10   # Haply x delta upper bound (right)
    raw_second_min: float = 0.0   # z (or y) delta lower bound
    raw_second_max: float = 0.15  # z (or y) delta upper bound


def validate_mapping_config(
    config: MappingConfig,
    task_anchor: TaskPoint,
    workspace_bounds: tuple[float, float, float, float] | None = None,
) -> None:
    """Reject invalid mapping parameters before hardware input is accepted."""
    values = (
        config.scale_x,
        config.scale_y,
        config.raw_x_min,
        config.raw_x_max,
        config.raw_second_min,
        config.raw_second_max,
        task_anchor.x,
        task_anchor.y,
        task_anchor.z,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("mapping parameters and task anchor must be finite")
    if config.scale_x <= 0.0 or config.scale_y <= 0.0:
        raise ValueError("mapping scales must be positive; use invert_* for direction")
    if config.raw_x_min > config.raw_x_max:
        raise ValueError("raw_x_min must not exceed raw_x_max")
    if config.raw_second_min > config.raw_second_max:
        raise ValueError("raw_second_min must not exceed raw_second_max")
    if workspace_bounds is not None and config.clamp_raw:
        x_min, x_max, y_min, y_max = workspace_bounds
        if not all(math.isfinite(value) for value in workspace_bounds):
            raise ValueError("workspace bounds must be finite")
        if x_min >= x_max or y_min >= y_max:
            raise ValueError("workspace minimums must be below maximums")
        sign_x = -1.0 if config.invert_x else 1.0
        sign_y = -1.0 if config.invert_y else 1.0
        mapped_x = sorted(
            (config.raw_x_min * config.scale_x * sign_x,
             config.raw_x_max * config.scale_x * sign_x)
        )
        mapped_y = sorted(
            (config.raw_second_min * config.scale_y * sign_y,
             config.raw_second_max * config.scale_y * sign_y)
        )
        if (
            task_anchor.x + mapped_x[0] > x_min
            or task_anchor.x + mapped_x[1] < x_max
            or task_anchor.y + mapped_y[0] > y_min
            or task_anchor.y + mapped_y[1] < y_max
        ):
            raise ValueError("clamped mapper range cannot reach the task workspace")


def map_identity(raw_position: TaskPoint) -> TaskPoint:
    """Map raw coordinates directly into task coordinates."""
    return TaskPoint(raw_position.x, raw_position.y, raw_position.z)


class AnchoredDeltaMapper:
    """Map raw displacement onto a task-frame start point."""

    def __init__(self, config: MappingConfig):
        self.config = config
        self.raw_anchor: TaskPoint | None = None
        self.task_anchor: TaskPoint | None = None
        self.last_clamped_x = False
        self.last_clamped_second = False

    @property
    def is_ready(self) -> bool:
        """Return true after both anchors have been captured."""
        return self.raw_anchor is not None and self.task_anchor is not None

    def capture_anchor(self, raw_position: TaskPoint, task_start: TaskPoint) -> None:
        """Anchor the current raw pose to the current task start point."""
        self.raw_anchor = raw_position
        self.task_anchor = task_start

    def map_position(self, raw_position: TaskPoint) -> TaskPoint | None:
        """Return the mapped task position, or None before anchors exist.

        Raw deltas are optionally clamped to the configured workspace bounds
        *before* scaling, so the physical movement range is hard-bounded
        regardless of scale factor.
        """
        if self.raw_anchor is None or self.task_anchor is None:
            return None

        # --- Compute raw deltas from anchor ---
        delta_x = raw_position.x - self.raw_anchor.x

        # Use Haply z (vertical) as the second task dimension when use_z_as_y is
        # True, because Haply y is the depth axis (not part of the 2-DoF plane).
        raw_second = raw_position.z if self.config.use_z_as_y else raw_position.y
        anchor_second = self.raw_anchor.z if self.config.use_z_as_y else self.raw_anchor.y
        delta_second = raw_second - anchor_second

        # --- Optional clamping to physical workspace bounds ---
        if self.config.clamp_raw:
            unclamped_x, unclamped_second = delta_x, delta_second
            delta_x = max(self.config.raw_x_min, min(self.config.raw_x_max, delta_x))
            delta_second = max(
                self.config.raw_second_min, min(self.config.raw_second_max, delta_second)
            )
            self.last_clamped_x = delta_x != unclamped_x
            self.last_clamped_second = delta_second != unclamped_second
        else:
            self.last_clamped_x = False
            self.last_clamped_second = False

        # --- Apply scale and inversion ---
        sign_x = -1.0 if self.config.invert_x else 1.0
        sign_y = -1.0 if self.config.invert_y else 1.0
        dx = delta_x * self.config.scale_x * sign_x
        dy = delta_second * self.config.scale_y * sign_y

        return TaskPoint(
            x=self.task_anchor.x + dx,
            y=self.task_anchor.y + dy,
            z=0.0,
        )
