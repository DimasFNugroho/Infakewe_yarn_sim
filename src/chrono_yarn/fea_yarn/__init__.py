"""Cable-based FEA yarn helpers built on `pychrono.fea`.

This package provides a first FEA implementation path for yarn-like behavior
using ANCF cable elements. The initial scenario target is a hanging yarn/cable
strand under gravity with one fixed endpoint.
"""

from .config import FEAHangingYarnConfig, FEASolverConfig, FEAVisualizationConfig
from .cable import CableBuildHandles, build_cable_ancf_yarn, make_cable_section
from .scene import FEAHangingSceneHandles, build_hanging_yarn_scene
from .metrics import cable_tip_position, cable_max_sag
from .visualization import attach_fea_cable_visuals

__all__ = [
    "FEAHangingYarnConfig",
    "FEASolverConfig",
    "FEAVisualizationConfig",
    "CableBuildHandles",
    "build_cable_ancf_yarn",
    "make_cable_section",
    "FEAHangingSceneHandles",
    "build_hanging_yarn_scene",
    "cable_tip_position",
    "cable_max_sag",
    "attach_fea_cable_visuals",
]

