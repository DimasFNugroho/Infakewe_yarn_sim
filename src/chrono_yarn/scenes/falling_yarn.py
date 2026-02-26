"""Scene composition for a free-falling yarn onto a floor.

This module owns assembly of the first milestone scenario:
- create and configure a Chrono system,
- add a floor body,
- add a segmented yarn chain positioned in the air,
- return handles consumed by simulation runners.
"""

from __future__ import annotations

from dataclasses import dataclass

import pychrono as chrono

from ..compat import prefer_bullet, set_gravity, set_single_thread, tune_collision_defaults
from ..config import FloorConfig, SimulationConfig, YarnConfig


@dataclass(slots=True)
class SceneHandles:
    """References to the core objects needed to simulate and record a scene."""

    system: chrono.ChSystem
    floor_body: object
    yarn_chain: object


@dataclass(slots=True)
class FallingYarnScene:
    """Configuration object for the falling-yarn scene builder."""

    sim: SimulationConfig
    yarn: YarnConfig
    floor: FloorConfig

    def build(self) -> SceneHandles:
        """Build and return the Chrono scene objects for this scenario."""
        return build_falling_yarn_scene(self.sim, self.yarn, self.floor)


def build_falling_yarn_scene(
    sim_cfg: SimulationConfig,
    yarn_cfg: YarnConfig,
    floor_cfg: FloorConfig,
) -> SceneHandles:
    """Build the milestone-1 scene: a free yarn chain dropped onto a floor.

    The function currently initializes and configures the Chrono system only.
    Floor and yarn construction are intentionally left as a later implementation
    step.
    """
    system = chrono.ChSystemNSC() if sim_cfg.contact_model == "NSC" else chrono.ChSystemSMC()
    prefer_bullet(system)
    tune_collision_defaults(
        envelope=sim_cfg.solver.collision_envelope,
        margin=sim_cfg.solver.collision_margin,
    )
    if sim_cfg.solver.single_thread:
        set_single_thread(system)
    set_gravity(system, chrono.ChVectorD(*sim_cfg.gravity))

    # Wiring of floor/yarn construction will be added in the implementation step.
    _ = (yarn_cfg, floor_cfg)
    raise NotImplementedError("build_falling_yarn_scene is a skeleton stub")
