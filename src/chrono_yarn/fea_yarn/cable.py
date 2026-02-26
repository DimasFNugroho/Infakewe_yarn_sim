"""ANCF cable-section construction helpers for FEA yarn models."""

from __future__ import annotations

from dataclasses import dataclass, field

import pychrono as chrono
import pychrono.fea as fea

from .config import FEAHangingYarnConfig


@dataclass(slots=True)
class CableBuildHandles:
    """References to FEA objects created for one cable/yarn strand."""

    mesh: fea.ChMesh
    section: fea.ChBeamSectionCable
    nodes: list = field(default_factory=list)
    elements: list = field(default_factory=list)

    @property
    def first_node(self):
        return self.nodes[0] if self.nodes else None

    @property
    def last_node(self):
        return self.nodes[-1] if self.nodes else None


def make_cable_section(cfg: FEAHangingYarnConfig) -> fea.ChBeamSectionCable:
    """Create and configure the ANCF cable section for the yarn strand."""
    section = fea.ChBeamSectionCable()
    section.SetDiameter(float(2.0 * cfg.radius))
    section.SetYoungModulus(float(cfg.young_modulus))
    section.SetDensity(float(cfg.density))
    if hasattr(section, "SetBeamRaleyghDamping"):
        section.SetBeamRaleyghDamping(float(cfg.rayleigh_damping))
    return section


def build_cable_ancf_yarn(mesh: fea.ChMesh, cfg: FEAHangingYarnConfig) -> CableBuildHandles:
    """Build an ANCF cable strand in `mesh` and return created node/element handles."""
    if cfg.element_count <= 0:
        raise ValueError("cfg.element_count must be > 0")
    if cfg.length <= 0.0:
        raise ValueError("cfg.length must be > 0")

    section = make_cable_section(cfg)
    builder = fea.ChBuilderCableANCF()
    n_nodes_before = int(mesh.GetNnodes()) if hasattr(mesh, "GetNnodes") else 0
    n_elems_before = int(mesh.GetNelements()) if hasattr(mesh, "GetNelements") else 0
    builder.BuildBeam(
        mesh,
        section,
        int(cfg.element_count),
        chrono.ChVectorD(*cfg.start),
        chrono.ChVectorD(*cfg.end),
    )

    # Important: avoid storing wrappers from `GetLastBeamNodes/Elements()` directly.
    # In some PyChrono builds those SWIG vector-backed wrappers become invalid once
    # the temporary vector object goes out of scope, which can segfault on access.
    n_nodes_after = int(mesh.GetNnodes()) if hasattr(mesh, "GetNnodes") else n_nodes_before
    n_elems_after = int(mesh.GetNelements()) if hasattr(mesh, "GetNelements") else n_elems_before
    nodes = []
    for i in range(n_nodes_before, n_nodes_after):
        node = mesh.GetNode(i)
        if hasattr(fea, "CastToChNodeFEAbase"):
            try:
                node = fea.CastToChNodeFEAbase(node)
            except Exception:
                pass
        if hasattr(fea, "CastToChNodeFEAxyzD"):
            try:
                node = fea.CastToChNodeFEAxyzD(node)
            except Exception:
                pass
        nodes.append(node)
    elements = [mesh.GetElement(i) for i in range(n_elems_before, n_elems_after)]

    if cfg.fix_start_node and nodes:
        if hasattr(nodes[0], "SetFixed"):
            nodes[0].SetFixed(True)
        if hasattr(nodes[0], "SetFixedD"):
            try:
                nodes[0].SetFixedD(True)
            except Exception:
                pass
    if cfg.fix_end_node and nodes:
        if hasattr(nodes[-1], "SetFixed"):
            nodes[-1].SetFixed(True)
        if hasattr(nodes[-1], "SetFixedD"):
            try:
                nodes[-1].SetFixedD(True)
            except Exception:
                pass

    return CableBuildHandles(mesh=mesh, section=section, nodes=nodes, elements=elements)
