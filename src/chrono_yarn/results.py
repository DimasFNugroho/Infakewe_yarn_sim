"""Result containers for recorded simulation data.

These dataclasses are plain Python structures intended for logging, testing, and
post-processing. They intentionally avoid direct dependence on PyChrono objects.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class SegmentKinematicsSample:
    """Snapshot of yarn segment positions at a single time sample."""

    segment_positions: list[tuple[float, float, float]]


@dataclass(slots=True)
class SimulationSample:
    """Recorded simulation state at one time instant."""

    time: float
    yarn: SegmentKinematicsSample


@dataclass(slots=True)
class SimulationResult:
    """Accumulated samples produced by a simulation runner."""

    samples: list[SimulationSample] = field(default_factory=list)

    def add_sample(self, sample: SimulationSample) -> None:
        """Append one sample to the result sequence."""
        self.samples.append(sample)
