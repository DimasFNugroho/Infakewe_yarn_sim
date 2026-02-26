"""Configuration dataclasses for yarn simulation scenarios.

This module defines the primary inputs used by scene builders and simulation
runners. The dataclasses are intentionally lightweight and serializable so they
can be created in user scripts and tests without importing PyChrono types.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal


ContactModel = Literal["NSC", "SMC"]


@dataclass(slots=True)
class SolverTuning:
    """Low-level solver and collision tuning parameters.

    These values are applied when initializing a Chrono system. They are kept in
    a dedicated dataclass so defaults can evolve independently from the main
    simulation settings.
    """

    collision_envelope: float = 0.003
    collision_margin: float = 0.002
    single_thread: bool = True


@dataclass(slots=True)
class SimulationConfig:
    """Top-level settings for a simulation run.

    Attributes:
        contact_model: Contact formulation to use (`"NSC"` or `"SMC"`).
        dt: Fixed integration time step in seconds.
        t_end: End time for the run in seconds.
        gravity: World gravity vector `(gx, gy, gz)` in m/s^2.
        sample_every_n_steps: Record state every N solver steps.
        solver: Optional solver/collision tuning parameters.
    """

    contact_model: ContactModel = "NSC"
    dt: float = 1e-3
    t_end: float = 1.0
    gravity: tuple[float, float, float] = (0.0, -9.81, 0.0)
    sample_every_n_steps: int = 10
    solver: SolverTuning = field(default_factory=SolverTuning)


@dataclass(slots=True)
class YarnConfig:
    """Geometric and physical parameters for a segmented yarn approximation.

    The yarn is represented as a chain of rigid segments connected by joints.
    `length` and `segment_count` define the discretization used by builders.
    """

    length: float = 0.75
    segment_count: int = 20
    radius: float = 0.01
    density: float = 500.0
    start_position: tuple[float, float, float] = (0.0, 0.9, 0.0)
    start_direction: tuple[float, float, float] = (1.0, 0.0, 0.0)

    @property
    def segment_length(self) -> float:
        """Return the nominal length of each segment in meters."""
        return self.length / self.segment_count


@dataclass(slots=True)
class FloorConfig:
    """Parameters for a simple axis-aligned box floor."""

    half_size: tuple[float, float, float] = (1.0, 0.05, 1.0)
    position: tuple[float, float, float] = (0.0, 0.05, 0.0)
    friction: float = 0.3
    restitution: float = 0.05
