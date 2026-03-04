"""Simulation stepping and sampling interfaces.

`SimulationRunner` is the intended entrypoint for advancing a built scene and
recording results in a consistent format. The concrete stepping logic is not yet
implemented in this skeleton.
"""

from __future__ import annotations

from dataclasses import dataclass

from .config import SimulationConfig
from .results import SegmentKinematicsSample, SimulationResult, SimulationSample
from .yarn_chain import extract_segment_positions


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
        result = SimulationResult()

        dt = float(self.config.dt)
        if dt <= 0.0:
            raise ValueError("config.dt must be > 0")
        t_end = float(self.config.t_end)
        if t_end < 0.0:
            raise ValueError("config.t_end must be >= 0")
        sample_n = max(1, int(self.config.sample_every_n_steps))

        system = scene.system
        chain = scene.yarn_chain
        steps = int(t_end / dt)

        # Always capture initial state.
        result.add_sample(
            SimulationSample(
                time=float(system.GetChTime()),
                yarn=SegmentKinematicsSample(segment_positions=extract_segment_positions(chain)),
            )
        )

        for i in range(steps):
            system.DoStepDynamics(dt)
            if (i + 1) % sample_n == 0:
                result.add_sample(
                    SimulationSample(
                        time=float(system.GetChTime()),
                        yarn=SegmentKinematicsSample(segment_positions=extract_segment_positions(chain)),
                    )
                )

        if not result.samples or result.samples[-1].time < float(system.GetChTime()):
            result.add_sample(
                SimulationSample(
                    time=float(system.GetChTime()),
                    yarn=SegmentKinematicsSample(segment_positions=extract_segment_positions(chain)),
                )
            )

        return result
