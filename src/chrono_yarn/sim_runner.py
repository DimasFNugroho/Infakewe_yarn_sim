"""Simulation stepping and sampling interfaces.

`SimulationRunner` is the intended entrypoint for advancing a built scene and
recording results in a consistent format. The concrete stepping logic is not yet
implemented in this skeleton.
"""

from __future__ import annotations

from dataclasses import dataclass

from .config import SimulationConfig
from .results import SimulationResult


@dataclass(slots=True)
class SimulationRunner:
    """Execute a configured simulation and collect sampled outputs."""

    config: SimulationConfig

    def run(self, scene) -> SimulationResult:
        """Advance a built scene and record samples.

        Expected input:
        - `scene.system`: Chrono system
        - `scene.yarn_chain`: handle consumed by recorder

        Returns:
            `SimulationResult` containing recorded samples.
        """
        _ = scene
        raise NotImplementedError("SimulationRunner.run is a skeleton stub")
