"""Timestamped Cartesian force control for a straight-line virtual fixture."""

from __future__ import annotations

import json
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StateFeedbackForceConfig:
    """Physical force gains and optional terminal docking configuration."""

    along_stiffness_n_per_m: float = 10.0
    along_damping_ns_per_m: float = 2.0
    fixture_stiffness_n_per_m: float = 20.0
    fixture_damping_ns_per_m: float = 2.0
    max_force_n: float = 2.0
    velocity_filter_alpha: float = 0.25
    docking_enabled: bool = False
    docking_start_percent: float = 85.0
    docking_stiffness_scale: float = 2.0
    docking_max_cross_track_m: float = 0.02
    adaptation_normalization: float = 50.0
    adaptation_strength: float = 0.7

    def validate(self) -> None:
        positive = {
            "along_stiffness_n_per_m": self.along_stiffness_n_per_m,
            "along_damping_ns_per_m": self.along_damping_ns_per_m,
            "fixture_stiffness_n_per_m": self.fixture_stiffness_n_per_m,
            "fixture_damping_ns_per_m": self.fixture_damping_ns_per_m,
            "max_force_n": self.max_force_n,
            "docking_stiffness_scale": self.docking_stiffness_scale,
            "docking_max_cross_track_m": self.docking_max_cross_track_m,
            "adaptation_normalization": self.adaptation_normalization,
        }
        invalid = [name for name, value in positive.items() if value <= 0.0]
        if invalid:
            raise ValueError(f"parameters must be positive: {', '.join(invalid)}")
        if not 0.0 < self.velocity_filter_alpha <= 1.0:
            raise ValueError("velocity_filter_alpha must be in (0, 1]")
        if not 0.0 <= self.docking_start_percent < 100.0:
            raise ValueError("docking_start_percent must be in [0, 100)")
        if not 0.0 <= self.adaptation_strength < 1.0:
            raise ValueError("adaptation_strength must be in [0, 1)")


class VirtualFixtureStateFeedbackController:
    """Generate a bounded force with independent along/cross-path feedback."""

    def __init__(
        self,
        start_point: Sequence[float],
        end_point: Sequence[float],
        config: StateFeedbackForceConfig | None = None,
        adaptive: bool = False,
    ) -> None:
        self.config = config or StateFeedbackForceConfig()
        self.config.validate()
        self.start = np.asarray(start_point, dtype=float).reshape(2)
        self.end = np.asarray(end_point, dtype=float).reshape(2)
        path = self.end - self.start
        self.path_length = float(np.linalg.norm(path))
        if not np.isfinite(self.path_length) or self.path_length <= 1e-9:
            raise ValueError("state-feedback path must have nonzero finite length")
        self.path_direction = path / self.path_length
        self.adaptive = bool(adaptive)
        self.adaptation_scale = 1.0
        self.previous_position: np.ndarray | None = None
        self.previous_timestamp_s: float | None = None
        self.filtered_velocity = np.zeros(2, dtype=float)
        self.last_progress = 0.0
        self.last_cross_track_m = 0.0
        self.last_force = np.zeros(2, dtype=float)

    def reset_kinematics(self) -> None:
        """Prevent derivative history from crossing a trial boundary."""
        self.previous_position = None
        self.previous_timestamp_s = None
        self.filtered_velocity = np.zeros(2, dtype=float)

    def path_coordinates(self, position: Sequence[float]) -> tuple[float, np.ndarray]:
        """Return directed normalized progress and the cross-track vector."""
        point = np.asarray(position, dtype=float).reshape(2)
        relative = point - self.start
        along_distance = float(relative @ self.path_direction)
        progress = along_distance / self.path_length
        projection = self.start + along_distance * self.path_direction
        return progress, point - projection

    def _velocity(self, position: np.ndarray, timestamp_s: float) -> np.ndarray:
        if not np.isfinite(timestamp_s):
            raise ValueError("cursor timestamp must be finite")
        if self.previous_timestamp_s is None:
            velocity = np.zeros(2, dtype=float)
        else:
            dt = timestamp_s - self.previous_timestamp_s
            if dt <= 0.0:
                raise ValueError("cursor timestamps must be strictly increasing")
            raw_velocity = (position - self.previous_position) / dt
            alpha = self.config.velocity_filter_alpha
            velocity = alpha * raw_velocity + (1.0 - alpha) * self.filtered_velocity
        self.previous_position = position.copy()
        self.previous_timestamp_s = float(timestamp_s)
        self.filtered_velocity = velocity
        return velocity

    def update_effective_interaction_model(
        self, coefficients: Sequence[Sequence[float]] | Sequence[float]
    ) -> None:
        """Adapt from effective closed-loop coefficients, not measured human force."""
        if not self.adaptive:
            return
        values = np.asarray(coefficients, dtype=float).reshape(-1)
        if values.size != 4 or not np.isfinite(values).all():
            raise ValueError("expected four finite effective interaction coefficients")
        position_terms = (abs(values[0]) + abs(values[2])) / 2.0
        velocity_terms = (abs(values[1]) + abs(values[3])) / 2.0
        dominance = np.clip(
            (position_terms - velocity_terms) / self.config.adaptation_normalization,
            -1.0,
            1.0,
        )
        self.adaptation_scale = float(
            np.clip(
                1.0 - self.config.adaptation_strength * dominance,
                1.0 - self.config.adaptation_strength,
                1.0 + self.config.adaptation_strength,
            )
        )

    def _docking_scale(self, progress: float, cross_track_m: float) -> float:
        if not self.config.docking_enabled:
            return 1.0
        if cross_track_m > self.config.docking_max_cross_track_m:
            return 1.0
        threshold = self.config.docking_start_percent / 100.0
        fraction = np.clip((progress - threshold) / (1.0 - threshold), 0.0, 1.0)
        smooth_fraction = fraction * fraction * (3.0 - 2.0 * fraction)
        return float(
            1.0 + (self.config.docking_stiffness_scale - 1.0) * smooth_fraction
        )

    def compute_force(
        self, position: Sequence[float], timestamp_s: float
    ) -> np.ndarray:
        """Compute task-frame force in newtons for one unique cursor sample."""
        point = np.asarray(position, dtype=float).reshape(2)
        if not np.isfinite(point).all():
            raise ValueError("cursor position must be finite")
        velocity = self._velocity(point, timestamp_s)
        progress, cross_track = self.path_coordinates(point)
        cross_track_m = float(np.linalg.norm(cross_track))
        direction = self.path_direction
        velocity_along = float(velocity @ direction) * direction
        velocity_cross = velocity - velocity_along
        remaining_along = float((self.end - point) @ direction)

        along_stiffness = self.config.along_stiffness_n_per_m * self.adaptation_scale
        along_damping = self.config.along_damping_ns_per_m * self.adaptation_scale
        docking_scale = self._docking_scale(progress, cross_track_m)
        along_force = (
            along_stiffness * docking_scale * remaining_along * direction
            - along_damping * velocity_along
        )
        fixture_force = (
            -self.config.fixture_stiffness_n_per_m * cross_track
            - self.config.fixture_damping_ns_per_m * velocity_cross
        )
        force = along_force + fixture_force
        magnitude = float(np.linalg.norm(force))
        if magnitude > self.config.max_force_n:
            force *= self.config.max_force_n / magnitude
        if not np.isfinite(force).all():
            raise ValueError("state-feedback force is non-finite")

        self.last_progress = float(np.clip(progress, 0.0, 1.0))
        self.last_cross_track_m = cross_track_m
        self.last_force = force
        return force.copy()

    def parameter_json(self) -> str:
        """Return logger-ready controller configuration and live adaptation state."""
        return json.dumps(
            {
                "Controller Type": "Virtual Fixture State Feedback",
                "output_units": "N",
                "estimator_interpretation": "effective_closed_loop",
                "adaptive": self.adaptive,
                "adaptation_scale": self.adaptation_scale,
                "along_stiffness_n_per_m": self.config.along_stiffness_n_per_m,
                "along_damping_ns_per_m": self.config.along_damping_ns_per_m,
                "fixture_stiffness_n_per_m": self.config.fixture_stiffness_n_per_m,
                "fixture_damping_ns_per_m": self.config.fixture_damping_ns_per_m,
                "max_force_n": self.config.max_force_n,
                "docking_enabled": self.config.docking_enabled,
                "docking_start_percent": self.config.docking_start_percent,
                "docking_stiffness_scale": self.config.docking_stiffness_scale,
                "docking_max_cross_track_m": (self.config.docking_max_cross_track_m),
            },
            sort_keys=True,
        )
