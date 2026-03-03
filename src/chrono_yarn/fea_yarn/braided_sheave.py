"""Braided multi-strand ANCF yarn over a rigid sheave (CBOS-style milestone).

This module provides a practical middle step toward the paper workflow:
- braided multi-strand yarn geometry (clockwise + counterclockwise strands),
- ANCF cable elements for each strand,
- rigid sheave with node-cloud contact,
- prescribed pulling of strand endpoints toward the sheave,
- relative-displacement proxy around interlacing locations.

The implementation intentionally favors compatibility with typical PyChrono
builds and keeps computational cost moderate.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import pychrono as chrono
import pychrono.fea as fea

from ..compat import prefer_bullet, set_gravity, set_single_thread, tune_collision_defaults
from .cable import CableBuildHandles, make_cable_section
from .config import FEAHangingYarnConfig, FEASolverConfig
from .scene import _configure_solver_and_timestepper


@dataclass(slots=True)
class FEABraidedYarnConfig:
    """Geometry/material settings for a braided multi-strand ANCF yarn."""

    length: float = 0.60
    element_count: int = 120
    strand_radius: float = 0.0008
    braid_radius: float = 0.0045
    braid_pitch: float = 0.08
    clockwise_strands: int = 4
    counterclockwise_strands: int = 4
    density: float = 900.0
    young_modulus: float = 3.0e8
    rayleigh_damping: float = 2e-4
    fix_start_nodes: bool = True
    fix_end_nodes: bool = True

    @property
    def strand_count(self) -> int:
        return int(self.clockwise_strands + self.counterclockwise_strands)


@dataclass(slots=True)
class FEASheaveConfig:
    """Rigid sheave and contact settings."""

    radius: float = 0.02
    width: float = 0.05
    density: float = 7800.0
    friction: float = 0.20
    restitution: float = 0.02
    wrap_angle_deg: float = 120.0
    contact_node_radius: float = 0.0011
    collision_envelope: float = 2e-4
    collision_margin: float = 1e-4
    center: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(slots=True)
class FEAPullProgramConfig:
    """Endpoint pulling program for CBOS-like loading."""

    pull_distance: float = 0.045
    pull_duration: float = 1.2
    hold_duration: float = 0.6

    def kinematics(self, t: float) -> tuple[float, float]:
        """Return `(displacement, speed)` at time `t` using hold + smooth ramp.

        Profile:
        - `0 <= t < hold_duration`: no pulling (wait for scene to settle/render)
        - next `pull_duration`: smooth cosine ramp to `pull_distance`
        - afterwards: hold final displacement
        """
        if t < self.hold_duration:
            return 0.0, 0.0
        if self.pull_duration <= 0.0:
            return self.pull_distance, 0.0
        t_pull = t - self.hold_duration
        tau = max(0.0, min(1.0, t_pull / self.pull_duration))
        disp = 0.5 * self.pull_distance * (1.0 - math.cos(math.pi * tau))
        spd = 0.5 * self.pull_distance * math.pi * math.sin(math.pi * tau) / self.pull_duration
        return disp, spd


@dataclass(slots=True)
class InterlacingProbe:
    """Pair of opposite-handed strand nodes used for RD estimation."""

    node_a: object
    node_b: object
    pos_a0: tuple[float, float, float]
    pos_b0: tuple[float, float, float]


@dataclass(slots=True)
class EndpointAnchor:
    """Kinematically driven endpoint node."""

    node: object
    pos0: tuple[float, float, float]
    direction: tuple[float, float, float]


@dataclass(slots=True)
class FEABraidedSheaveSceneHandles:
    """Handles for simulation, actuation, and metrics."""

    system: chrono.ChSystem
    mesh: fea.ChMesh
    sheave: chrono.ChBody
    strands: list[CableBuildHandles] = field(default_factory=list)
    probes: list[InterlacingProbe] = field(default_factory=list)
    anchors_left: list[EndpointAnchor] = field(default_factory=list)
    anchors_right: list[EndpointAnchor] = field(default_factory=list)
    pull_cfg: FEAPullProgramConfig = field(default_factory=FEAPullProgramConfig)


def build_braided_sheave_scene(
    sim_cfg: FEASolverConfig,
    braid_cfg: FEABraidedYarnConfig,
    sheave_cfg: FEASheaveConfig,
    pull_cfg: FEAPullProgramConfig,
) -> FEABraidedSheaveSceneHandles:
    """Build a braided ANCF yarn scene with sheave contact and endpoint pull anchors."""
    if braid_cfg.element_count <= 2:
        raise ValueError("braid_cfg.element_count must be > 2")
    if braid_cfg.strand_count <= 1:
        raise ValueError("Need at least two strands for braiding.")
    if braid_cfg.braid_pitch <= 0.0:
        raise ValueError("braid_cfg.braid_pitch must be > 0")

    system = chrono.ChSystemNSC()
    prefer_bullet(system)
    tune_collision_defaults(
        envelope=float(sheave_cfg.collision_envelope),
        margin=float(sheave_cfg.collision_margin),
    )
    set_single_thread(system)
    set_gravity(system, chrono.ChVectorD(*sim_cfg.gravity))
    _configure_solver_and_timestepper(system, sim_cfg)

    mesh = fea.ChMesh()
    if hasattr(mesh, "SetAutomaticGravity"):
        mesh.SetAutomaticGravity(True)
    system.Add(mesh)

    sheave_mat = chrono.ChMaterialSurfaceNSC()
    if hasattr(sheave_mat, "SetFriction"):
        sheave_mat.SetFriction(float(sheave_cfg.friction))
    if hasattr(sheave_mat, "SetRestitution"):
        sheave_mat.SetRestitution(float(sheave_cfg.restitution))

    sheave = chrono.ChBodyEasyCylinder(
        float(sheave_cfg.radius),
        float(sheave_cfg.width),
        float(sheave_cfg.density),
        True,
        True,
        sheave_mat,
    )
    sheave.SetBodyFixed(True)
    sheave.SetPos(chrono.ChVectorD(*sheave_cfg.center))
    system.Add(sheave)

    strand_proto_cfg = FEAHangingYarnConfig(
        length=braid_cfg.length,
        element_count=braid_cfg.element_count,
        radius=braid_cfg.strand_radius,
        density=braid_cfg.density,
        young_modulus=braid_cfg.young_modulus,
        rayleigh_damping=braid_cfg.rayleigh_damping,
    )
    section = make_cable_section(strand_proto_cfg)

    strands: list[CableBuildHandles] = []
    handedness: list[int] = []
    # Keep the strand centerlines outside the sheave surface at t=0.
    # Without this clearance, inward helical offsets can start interpenetrating
    # the sheave and quickly trigger numerical blow-ups.
    centerline_radius = (
        float(sheave_cfg.radius)
        + float(braid_cfg.braid_radius)
        + float(braid_cfg.strand_radius)
        + float(sheave_cfg.contact_node_radius)
    )

    center_points, center_s = _sample_centerline_points(
        total_length=braid_cfg.length,
        sheave_radius=centerline_radius,
        wrap_angle_deg=sheave_cfg.wrap_angle_deg,
        node_count=braid_cfg.element_count + 1,
    )
    for i in range(braid_cfg.clockwise_strands):
        phase0 = 2.0 * math.pi * i / max(1, braid_cfg.clockwise_strands)
        pts = _strand_points_from_centerline(
            center_points,
            center_s,
            braid_radius=braid_cfg.braid_radius,
            braid_pitch=braid_cfg.braid_pitch,
            phase0=phase0,
            handedness=+1,
        )
        strands.append(
            _build_cable_from_polyline(
                mesh,
                section,
                pts,
                fix_start=braid_cfg.fix_start_nodes,
                fix_end=braid_cfg.fix_end_nodes,
            )
        )
        handedness.append(+1)
    for i in range(braid_cfg.counterclockwise_strands):
        phase0 = (2.0 * math.pi * i / max(1, braid_cfg.counterclockwise_strands)) + (
            math.pi / max(1, braid_cfg.counterclockwise_strands)
        )
        pts = _strand_points_from_centerline(
            center_points,
            center_s,
            braid_radius=braid_cfg.braid_radius,
            braid_pitch=braid_cfg.braid_pitch,
            phase0=phase0,
            handedness=-1,
        )
        strands.append(
            _build_cable_from_polyline(
                mesh,
                section,
                pts,
                fix_start=braid_cfg.fix_start_nodes,
                fix_end=braid_cfg.fix_end_nodes,
            )
        )
        handedness.append(-1)

    contact_surface = fea.ChContactSurfaceNodeCloud(sheave_mat, mesh)
    contact_surface.AddAllNodes(float(sheave_cfg.contact_node_radius))
    mesh.AddContactSurface(contact_surface)

    probes = _build_interlacing_probes(strands, handedness, samples_per_pair=2)
    left_anchors, right_anchors = _build_endpoint_anchors(strands, sheave_cfg.center)
    return FEABraidedSheaveSceneHandles(
        system=system,
        mesh=mesh,
        sheave=sheave,
        strands=strands,
        probes=probes,
        anchors_left=left_anchors,
        anchors_right=right_anchors,
        pull_cfg=pull_cfg,
    )


def step_braided_sheave_scene(scene: FEABraidedSheaveSceneHandles, dt: float) -> None:
    """Advance one step while applying endpoint pull kinematics."""
    t = float(scene.system.GetChTime())
    disp, speed = scene.pull_cfg.kinematics(t)
    _apply_anchor_kinematics(scene.anchors_left, disp, speed)
    _apply_anchor_kinematics(scene.anchors_right, disp, speed)
    scene.system.DoStepDynamics(dt)


def estimate_relative_displacement(scene: FEABraidedSheaveSceneHandles) -> float:
    """Estimate average RD across sampled interlacing probes."""
    if not scene.probes:
        return 0.0
    rd_sum = 0.0
    for probe in scene.probes:
        pa = _node_pos_tuple(probe.node_a)
        pb = _node_pos_tuple(probe.node_b)
        ua = _sub3(pa, probe.pos_a0)
        ub = _sub3(pb, probe.pos_b0)
        u_interlace = _mul3(_add3(ua, ub), 0.5)
        va = _sub3(ua, u_interlace)
        vb = _sub3(ub, u_interlace)
        rd_sum += 0.5 * (_norm3(va) + _norm3(vb))
    return rd_sum / len(scene.probes)


def max_node_nan(scene: FEABraidedSheaveSceneHandles) -> bool:
    """Return True if any strand node has NaN coordinates."""
    for strand in scene.strands:
        for node in strand.nodes:
            p = node.GetPos()
            if (p.x != p.x) or (p.y != p.y) or (p.z != p.z):
                return True
    return False


def _sample_centerline_points(
    *,
    total_length: float,
    sheave_radius: float,
    wrap_angle_deg: float,
    node_count: int,
) -> tuple[list[tuple[float, float, float]], list[float]]:
    wrap = math.radians(float(wrap_angle_deg))
    arc_len = sheave_radius * wrap
    straight_len = max(0.0, 0.5 * (total_length - arc_len))
    phi0 = (0.5 * math.pi) + (0.5 * wrap)
    phi1 = (0.5 * math.pi) - (0.5 * wrap)
    t0 = _normalize3((math.sin(phi0), 0.0, -math.cos(phi0)))
    t1 = _normalize3((math.sin(phi1), 0.0, -math.cos(phi1)))
    p_arc0 = (sheave_radius * math.cos(phi0), 0.0, sheave_radius * math.sin(phi0))
    p_arc1 = (sheave_radius * math.cos(phi1), 0.0, sheave_radius * math.sin(phi1))
    p_left = _sub3(p_arc0, _mul3(t0, straight_len))
    p_right = _add3(p_arc1, _mul3(t1, straight_len))

    total = (2.0 * straight_len) + arc_len
    points: list[tuple[float, float, float]] = []
    svals: list[float] = []
    for i in range(max(2, node_count)):
        s = total * i / float(max(1, node_count - 1))
        if s <= straight_len:
            p = _add3(p_left, _mul3(t0, s))
        elif s <= (straight_len + arc_len):
            sa = s - straight_len
            phi = phi0 - (sa / sheave_radius)
            p = (sheave_radius * math.cos(phi), 0.0, sheave_radius * math.sin(phi))
        else:
            sr = s - straight_len - arc_len
            p = _add3(p_arc1, _mul3(t1, sr))
        points.append(p)
        svals.append(s)
    points[0] = p_left
    points[-1] = p_right
    return points, svals


def _strand_points_from_centerline(
    center_points: list[tuple[float, float, float]],
    center_s: list[float],
    *,
    braid_radius: float,
    braid_pitch: float,
    phase0: float,
    handedness: int,
) -> list[tuple[float, float, float]]:
    count = len(center_points)
    if count < 2:
        return center_points[:]
    points: list[tuple[float, float, float]] = []
    up = (0.0, 1.0, 0.0)
    xaxis = (1.0, 0.0, 0.0)
    for i in range(count):
        p_prev = center_points[i - 1] if i > 0 else center_points[i]
        p_next = center_points[i + 1] if i < (count - 1) else center_points[i]
        tangent = _normalize3(_sub3(p_next, p_prev))
        n1 = _cross3(tangent, up)
        if _norm3(n1) < 1e-12:
            n1 = _cross3(tangent, xaxis)
        n1 = _normalize3(n1)
        n2 = _normalize3(_cross3(tangent, n1))
        phase = phase0 + (float(handedness) * 2.0 * math.pi * center_s[i] / braid_pitch)
        offs = _add3(_mul3(n1, braid_radius * math.cos(phase)), _mul3(n2, braid_radius * math.sin(phase)))
        points.append(_add3(center_points[i], offs))
    return points


def _build_cable_from_polyline(
    mesh: fea.ChMesh,
    section: fea.ChBeamSectionCable,
    points: list[tuple[float, float, float]],
    *,
    fix_start: bool,
    fix_end: bool,
) -> CableBuildHandles:
    if len(points) < 2:
        raise ValueError("Need at least 2 points to build a cable strand.")
    tangents = _polyline_tangents(points)
    nodes = []
    for p, d in zip(points, tangents):
        node = fea.ChNodeFEAxyzD(chrono.ChVectorD(*p), chrono.ChVectorD(*d))
        mesh.AddNode(node)
        nodes.append(node)
    elements = []
    for i in range(len(nodes) - 1):
        elem = fea.ChElementCableANCF()
        elem.SetNodes(nodes[i], nodes[i + 1])
        elem.SetSection(section)
        mesh.AddElement(elem)
        elements.append(elem)
    if fix_start and nodes:
        if hasattr(nodes[0], "SetFixed"):
            nodes[0].SetFixed(True)
        if hasattr(nodes[0], "SetFixedD"):
            nodes[0].SetFixedD(True)
    if fix_end and nodes:
        if hasattr(nodes[-1], "SetFixed"):
            nodes[-1].SetFixed(True)
        if hasattr(nodes[-1], "SetFixedD"):
            nodes[-1].SetFixedD(True)
    return CableBuildHandles(mesh=mesh, section=section, nodes=nodes, elements=elements)


def _build_interlacing_probes(
    strands: list[CableBuildHandles],
    handedness: list[int],
    *,
    samples_per_pair: int = 2,
) -> list[InterlacingProbe]:
    cw = [s for s, h in zip(strands, handedness) if h > 0]
    ccw = [s for s, h in zip(strands, handedness) if h < 0]
    if not cw or not ccw:
        return []
    probes: list[InterlacingProbe] = []
    pair_count = min(len(cw), len(ccw))
    for i in range(pair_count):
        sa = cw[i]
        sb = ccw[i]
        n = min(len(sa.nodes), len(sb.nodes))
        if n < 6:
            continue
        mid = n // 2
        idxs = [mid]
        if samples_per_pair > 1:
            idxs = [max(1, mid - 2), mid]
        for idx in idxs:
            na = sa.nodes[idx]
            nb = sb.nodes[idx]
            probes.append(
                InterlacingProbe(
                    node_a=na,
                    node_b=nb,
                    pos_a0=_node_pos_tuple(na),
                    pos_b0=_node_pos_tuple(nb),
                )
            )
    return probes


def _build_endpoint_anchors(
    strands: list[CableBuildHandles],
    sheave_center: tuple[float, float, float],
) -> tuple[list[EndpointAnchor], list[EndpointAnchor]]:
    left: list[EndpointAnchor] = []
    right: list[EndpointAnchor] = []
    c = sheave_center
    for strand in strands:
        if not strand.nodes:
            continue
        n0 = strand.nodes[0]
        n1 = strand.nodes[-1]
        p0 = _node_pos_tuple(n0)
        p1 = _node_pos_tuple(n1)
        d0 = _normalize3(_sub3(c, p0))
        d1 = _normalize3(_sub3(c, p1))
        left.append(EndpointAnchor(node=n0, pos0=p0, direction=d0))
        right.append(EndpointAnchor(node=n1, pos0=p1, direction=d1))
    return left, right


def _apply_anchor_kinematics(anchors: list[EndpointAnchor], disp: float, speed: float) -> None:
    for anchor in anchors:
        p = _add3(anchor.pos0, _mul3(anchor.direction, disp))
        v = _mul3(anchor.direction, speed)
        anchor.node.SetPos(chrono.ChVectorD(*p))
        if hasattr(anchor.node, "SetPos_dt"):
            anchor.node.SetPos_dt(chrono.ChVectorD(*v))
        if hasattr(anchor.node, "SetPos_dtdt"):
            anchor.node.SetPos_dtdt(chrono.ChVectorD(0.0, 0.0, 0.0))
        if hasattr(anchor.node, "SetNoSpeedNoAcceleration"):
            try:
                anchor.node.SetNoSpeedNoAcceleration(False)
            except Exception:
                pass


def _polyline_tangents(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    tangents = []
    for i in range(len(points)):
        p_prev = points[i - 1] if i > 0 else points[i]
        p_next = points[i + 1] if i < (len(points) - 1) else points[i]
        d = _sub3(p_next, p_prev)
        if _norm3(d) < 1e-12:
            d = (1.0, 0.0, 0.0)
        tangents.append(_normalize3(d))
    return tangents


def _node_pos_tuple(node) -> tuple[float, float, float]:
    p = node.GetPos()
    return float(p.x), float(p.y), float(p.z)


def _add3(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def _sub3(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def _mul3(a: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return a[0] * s, a[1] * s, a[2] * s


def _dot3(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2])


def _cross3(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        (a[1] * b[2]) - (a[2] * b[1]),
        (a[2] * b[0]) - (a[0] * b[2]),
        (a[0] * b[1]) - (a[1] * b[0]),
    )


def _norm3(a: tuple[float, float, float]) -> float:
    return math.sqrt(_dot3(a, a))


def _normalize3(a: tuple[float, float, float]) -> tuple[float, float, float]:
    n = _norm3(a)
    if n < 1e-16:
        return 0.0, 0.0, 0.0
    return a[0] / n, a[1] / n, a[2] / n
