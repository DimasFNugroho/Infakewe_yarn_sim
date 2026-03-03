"""ANCF cable-section construction helpers for FEA yarn models."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

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
    """Build an ANCF cable strand in `mesh` and return created node/element handles.

    Note:
    `cfg.length` is the target centerline rest length. When it differs from the
    start-end chord, nodes are initialized on a smooth sagged polyline whose arc
    length matches `cfg.length` (within numerical tolerance).
    """
    if cfg.element_count <= 0:
        raise ValueError("cfg.element_count must be > 0")
    if cfg.length <= 0.0:
        raise ValueError("cfg.length must be > 0")

    section = make_cable_section(cfg)
    points = _build_centerline_points(cfg)
    nodes, elements = _build_cable_from_points(mesh, section, points)

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


def _build_centerline_points(cfg: FEAHangingYarnConfig) -> list[tuple[float, float, float]]:
    """Return centerline points with arc length approximately equal to `cfg.length`."""
    p0 = cfg.start
    p1 = cfg.end
    d = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    chord = math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
    if chord < 1e-12:
        raise ValueError("start and end cannot be the same point")
    if cfg.length + 1e-12 < chord:
        raise ValueError(
            "cfg.length is shorter than the start-end chord; "
            "increase cfg.length or move endpoints closer."
        )

    node_count = int(cfg.element_count) + 1
    if node_count < 2:
        node_count = 2

    # Prefer sag in global -Y direction, but remove tangent component.
    t = (d[0] / chord, d[1] / chord, d[2] / chord)
    down = (0.0, -1.0, 0.0)
    n = _sub3(down, _mul3(t, _dot3(down, t)))
    if _norm3(n) < 1e-10:
        n = _sub3((0.0, 0.0, 1.0), _mul3(t, _dot3((0.0, 0.0, 1.0), t)))
    n = _normalize3(n)

    def make_points(amplitude: float) -> list[tuple[float, float, float]]:
        pts = []
        for i in range(node_count):
            s = i / float(node_count - 1)
            base = (
                p0[0] + d[0] * s,
                p0[1] + d[1] * s,
                p0[2] + d[2] * s,
            )
            offs = _mul3(n, amplitude * math.sin(math.pi * s))
            pts.append(_add3(base, offs))
        return pts

    target_len = float(cfg.length)
    if abs(target_len - chord) <= 1e-10:
        return make_points(0.0)

    # Binary-search amplitude that matches target arc length.
    lo = 0.0
    hi = max(0.25 * chord, 1e-3)
    for _ in range(30):
        if _polyline_length(make_points(hi)) >= target_len:
            break
        hi *= 1.6
    for _ in range(45):
        mid = 0.5 * (lo + hi)
        if _polyline_length(make_points(mid)) < target_len:
            lo = mid
        else:
            hi = mid
    return make_points(0.5 * (lo + hi))


def _build_cable_from_points(mesh, section, points):
    """Build ChNodeFEAxyzD + ChElementCableANCF chain from centerline points."""
    tangents = _polyline_tangents(points)
    nodes = []
    for p, t in zip(points, tangents):
        node = fea.ChNodeFEAxyzD(chrono.ChVectorD(*p), chrono.ChVectorD(*t))
        mesh.AddNode(node)
        nodes.append(node)

    elements = []
    for i in range(len(nodes) - 1):
        elem = fea.ChElementCableANCF()
        elem.SetNodes(nodes[i], nodes[i + 1])
        elem.SetSection(section)
        mesh.AddElement(elem)
        elements.append(elem)
    return nodes, elements


def _polyline_tangents(points):
    tangents = []
    for i in range(len(points)):
        p_prev = points[i - 1] if i > 0 else points[i]
        p_next = points[i + 1] if i < len(points) - 1 else points[i]
        d = _sub3(p_next, p_prev)
        if _norm3(d) < 1e-12:
            d = (1.0, 0.0, 0.0)
        tangents.append(_normalize3(d))
    return tangents


def _polyline_length(points) -> float:
    total = 0.0
    for i in range(len(points) - 1):
        total += _norm3(_sub3(points[i + 1], points[i]))
    return total


def _dot3(a, b) -> float:
    return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2])


def _add3(a, b):
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def _sub3(a, b):
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def _mul3(a, s: float):
    return a[0] * s, a[1] * s, a[2] * s


def _norm3(a) -> float:
    return math.sqrt(_dot3(a, a))


def _normalize3(a):
    n = _norm3(a)
    if n < 1e-16:
        return 0.0, 0.0, 0.0
    return a[0] / n, a[1] / n, a[2] / n
