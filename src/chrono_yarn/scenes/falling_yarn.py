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
from ..geometry import add_floor_box
from ..materials import make_contact_material
from ..yarn_chain import build_yarn_chain


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

    This scene uses a fixed floor and a free yarn chain (not anchored), so the
    chain can fall under gravity and interact with the floor.
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

    material = make_contact_material(
        sim_cfg.contact_model,
        friction=floor_cfg.friction,
        restitution=floor_cfg.restitution,
    )
    floor_body = add_floor_box(
        system=system,
        half_size=floor_cfg.half_size,
        position=floor_cfg.position,
        material=material,
    )
    yarn_chain = build_yarn_chain(
        system=system,
        yarn_cfg=yarn_cfg,
        material=material,
        anchor_body=None,
        fixed_segments=False,
    )

    return SceneHandles(system=system, floor_body=floor_body, yarn_chain=yarn_chain)
