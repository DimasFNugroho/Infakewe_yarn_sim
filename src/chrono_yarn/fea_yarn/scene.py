"""Scene assembly helpers for hanging FEA cable/yarn examples."""

from __future__ import annotations

from dataclasses import dataclass

import pychrono as chrono
import pychrono.fea as fea

from ..compat import set_gravity
from .cable import CableBuildHandles, build_cable_ancf_yarn
from .config import FEAHangingYarnConfig, FEASolverConfig


@dataclass(slots=True)
class FEAHangingSceneHandles:
    """References to objects needed to simulate and inspect the FEA hanging yarn scene."""

    system: chrono.ChSystem
    mesh: fea.ChMesh
    cable: CableBuildHandles


def build_hanging_yarn_scene(
    sim_cfg: FEASolverConfig,
    yarn_cfg: FEAHangingYarnConfig,
) -> FEAHangingSceneHandles:
    """Build a minimal FEA hanging-yarn scene (no rigid contact yet)."""
    system = chrono.ChSystemNSC()
    set_gravity(system, chrono.ChVectorD(*sim_cfg.gravity))
    _configure_solver_and_timestepper(system, sim_cfg)

    mesh = fea.ChMesh()
    if hasattr(mesh, "SetAutomaticGravity"):
        mesh.SetAutomaticGravity(True)
    system.Add(mesh)

    cable = build_cable_ancf_yarn(mesh, yarn_cfg)
    return FEAHangingSceneHandles(system=system, mesh=mesh, cable=cable)


def _configure_solver_and_timestepper(system, sim_cfg: FEASolverConfig) -> None:
    """Apply an explicit solver/timestepper configuration known to work for ANCF cables."""
    solver_kind = (sim_cfg.solver_kind or "MINRES").upper()
    if solver_kind == "SPARSEQR" and hasattr(chrono, "ChSolverSparseQR"):
        solver = chrono.ChSolverSparseQR()
    elif solver_kind == "SPARSELU" and hasattr(chrono, "ChSolverSparseLU"):
        solver = chrono.ChSolverSparseLU()
    else:
        # MINRES is a good default for this cable example.
        solver = chrono.ChSolverMINRES()
        if hasattr(solver, "SetMaxIterations"):
            solver.SetMaxIterations(int(sim_cfg.solver_max_iterations))
        if hasattr(solver, "SetTolerance"):
            solver.SetTolerance(float(sim_cfg.solver_tolerance))
        if hasattr(solver, "EnableDiagonalPreconditioner"):
            try:
                solver.EnableDiagonalPreconditioner(True)
            except Exception:
                pass
        if hasattr(solver, "SetVerbose"):
            try:
                solver.SetVerbose(False)
            except Exception:
                pass

    if hasattr(system, "SetSolver"):
        system.SetSolver(solver)

    # FEA cable also works with default stepping in this build; HHT is optional.
    if sim_cfg.use_hht and hasattr(chrono, "ChTimestepperHHT"):
        hht = chrono.ChTimestepperHHT(system)
        if hasattr(hht, "SetAlpha"):
            hht.SetAlpha(float(sim_cfg.hht_alpha))
        if hasattr(hht, "SetMaxiters"):
            hht.SetMaxiters(int(sim_cfg.hht_max_iters))
        if hasattr(hht, "SetModifiedNewton"):
            try:
                hht.SetModifiedNewton(True)
            except Exception:
                pass
        if hasattr(system, "SetTimestepper"):
            system.SetTimestepper(hht)
