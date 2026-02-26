"""Configuration dataclasses for FEA-based yarn/cable scenarios."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class FEASolverConfig:
    """Solver/time-stepping settings for FEA scene examples."""

    dt: float = 2e-4
    gravity: tuple[float, float, float] = (0.0, -9.81, 0.0)
    solver_max_iterations: int = 200
    solver_tolerance: float = 1e-10
    solver_kind: str = "MINRES"
    use_hht: bool = False
    hht_alpha: float = -0.1
    hht_max_iters: int = 20


@dataclass(slots=True)
class FEAHangingYarnConfig:
    """Geometry/material settings for a hanging ANCF cable approximation."""

    length: float = 1.0
    element_count: int = 80
    radius: float = 0.0015
    density: float = 1200.0
    young_modulus: float = 5e8
    rayleigh_damping: float = 2e-4
    start: tuple[float, float, float] = (-0.55, 0.75, 0.0)
    end: tuple[float, float, float] = (0.45, 0.75, 0.0)
    fix_start_node: bool = True
    fix_end_node: bool = False

    @property
    def element_length(self) -> float:
        """Nominal ANCF cable element length in meters."""
        return self.length / self.element_count


@dataclass(slots=True)
class FEAVisualizationConfig:
    """Visualization settings for FEA cable examples."""

    beam_resolution: int = 6
    beam_section_resolution: int = 6
    wireframe: bool = False
    draw_node_glyphs: bool = True
    node_glyph_scale: float = 0.005
    node_glyph_thickness: float = 0.003
