"""Yarn chain assembly utilities.

This module defines the public interface for building a discretized yarn as a
set of rigid segments connected by joints. It also provides helpers to extract
simulation state into plain Python data structures for recording.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import pychrono as chrono

from .config import YarnConfig
from .geometry import add_yarn_segment_capsule


@dataclass(slots=True)
class YarnChainHandles:
    """References to Chrono objects created for a yarn chain."""

    segments: list = field(default_factory=list)
    joints: list = field(default_factory=list)
    aux_links: list = field(default_factory=list)
    segment_length: float = 0.0


def build_yarn_chain(
    system,
    yarn_cfg: YarnConfig,
    material,
    anchor_body=None,
    *,
    fixed_segments: bool = False,
) -> YarnChainHandles:
    """Build a free or anchored yarn chain from short rigid segments.

    Args:
        system: Chrono system receiving bodies and links.
        yarn_cfg: Yarn discretization and initial placement configuration.
        material: Contact material for segment collision, or ``None`` for
            visual-only segments.
        anchor_body: Optional body to spherical-joint to the first segment at
            `yarn_cfg.start_position`.
        fixed_segments: If true, each segment body is fixed and no inter-segment
            joints are added. Useful for simple visualization layouts.
    """
    if yarn_cfg.segment_count <= 0:
        raise ValueError("yarn_cfg.segment_count must be > 0")
    if yarn_cfg.length <= 0.0:
        raise ValueError("yarn_cfg.length must be > 0")

    direction = _normalize(yarn_cfg.start_direction)
    q = _rotation_from_local_y_to(direction)
    rot = chrono.ChMatrix33D(q)

    x0, y0, z0 = yarn_cfg.start_position
    start = chrono.ChVectorD(x0, y0, z0)
    seg_len = yarn_cfg.segment_length
    half_len = 0.5 * seg_len

    handles = YarnChainHandles(segment_length=seg_len)

    for i in range(yarn_cfg.segment_count):
        c = 0.20 + 0.60 * i / max(1, yarn_cfg.segment_count - 1)
        seg = add_yarn_segment_capsule(
            system,
            half_len=half_len,
            radius=yarn_cfg.radius,
            density=yarn_cfg.density,
            material=material,
            color=(c, 0.55, 0.86),
        )

        left_end = start + chrono.ChVectorD(direction[0] * i * seg_len, direction[1] * i * seg_len, direction[2] * i * seg_len)
        com = left_end + (rot * chrono.ChVectorD(0, +half_len, 0))
        seg.SetPos(com)
        seg.SetRot(q)
        seg.SetBodyFixed(fixed_segments)
        handles.segments.append(seg)

    if fixed_segments:
        return handles

    for i in range(1, yarn_cfg.segment_count):
        p = start + chrono.ChVectorD(direction[0] * i * seg_len, direction[1] * i * seg_len, direction[2] * i * seg_len)
        joint = chrono.ChLinkLockSpherical()
        joint.Initialize(handles.segments[i], handles.segments[i - 1], chrono.ChCoordsysD(p, chrono.QUNIT))
        system.AddLink(joint)
        handles.joints.append(joint)

    if anchor_body is not None:
        anchor_joint = chrono.ChLinkLockSpherical()
        anchor_joint.Initialize(handles.segments[0], anchor_body, chrono.ChCoordsysD(start, chrono.QUNIT))
        system.AddLink(anchor_joint)
        handles.joints.append(anchor_joint)

    return handles


def _normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = map(float, v)
    n = math.sqrt(x * x + y * y + z * z)
    if n <= 0.0:
        raise ValueError("Direction vector must be non-zero")
    return (x / n, y / n, z / n)


def _rotation_from_local_y_to(target: tuple[float, float, float]):
    """Return a quaternion rotating local +Y onto `target`."""
    ux, uy, uz = 0.0, 1.0, 0.0
    vx, vy, vz = target
    dot = max(-1.0, min(1.0, ux * vx + uy * vy + uz * vz))

    if dot > 1.0 - 1e-12:
        return chrono.QUNIT

    if dot < -1.0 + 1e-12:
        # 180-degree flip from +Y to -Y around X.
        return chrono.Q_from_AngAxis(math.pi, chrono.ChVectorD(1, 0, 0))

    ax = uy * vz - uz * vy
    ay = uz * vx - ux * vz
    az = ux * vy - uy * vx
    an = math.sqrt(ax * ax + ay * ay + az * az)
    if an <= 0.0:
        return chrono.QUNIT

    axis = chrono.ChVectorD(ax / an, ay / an, az / an)
    angle = math.acos(dot)
    return chrono.Q_from_AngAxis(angle, axis)


def extract_segment_positions(chain: YarnChainHandles) -> list[tuple[float, float, float]]:
    """Convert segment body positions into plain Python tuples.

    This helper is intended for sampling code in the simulation runner and for
    tests that need stable, serialization-friendly outputs.
    """
    positions: list[tuple[float, float, float]] = []
    for body in chain.segments:
        p = body.GetPos()
        positions.append((float(p.x), float(p.y), float(p.z)))
    return positions


def max_neighbor_joint_gap(chain: YarnChainHandles) -> float:
    """Return the maximum endpoint mismatch between neighboring segments.

    For each neighboring pair `(i-1, i)`, this computes the distance between:
    - the right tip of segment `i-1`
    - the left tip of segment `i`

    In an ideal spherical-joint chain this should be zero. In practice, a small
    nonzero value appears because of numerical constraint error.

    Returns:
        Maximum gap in meters.
    """
    if len(chain.segments) < 2 or chain.segment_length <= 0.0:
        return 0.0

    half_len = 0.5 * chain.segment_length
    max_gap = 0.0
    for i in range(1, len(chain.segments)):
        prev = chain.segments[i - 1]
        curr = chain.segments[i]

        p_prev = _segment_tip_world(prev, +half_len)
        p_curr = _segment_tip_world(curr, -half_len)
        d = p_prev - p_curr
        gap = math.sqrt(d.x * d.x + d.y * d.y + d.z * d.z)
        if gap > max_gap:
            max_gap = gap
    return max_gap


def _segment_tip_world(body, local_y: float):
    """Return world position of a segment tip located at local `(0, local_y, 0)`."""
    p = body.GetPos()
    rot = chrono.ChMatrix33D(body.GetRot())
    return p + rot * chrono.ChVectorD(0.0, local_y, 0.0)


def add_bending_proxy_tsdas(
    system,
    chain: YarnChainHandles,
    *,
    span: int = 2,
    spring_k: float = 1.0,
    damping_c: float = 0.01,
) -> list:
    """Add translational spring-dampers as a simple bending-resistance proxy.

    This is not a physically rigorous yarn constitutive model. It is a practical
    improvement over a pure spherical-joint chain for early visualization and
    tuning. Each spring connects segment COMs separated by `span` segments
    (typically `i` to `i+2`), which resists sharp folding and reduces the
    "bead-chain" look.

    Args:
        system: Chrono system receiving the links.
        chain: Built yarn chain handles.
        span: Segment separation for spring links. `2` is a good default.
        spring_k: Spring coefficient (N/m).
        damping_c: Damping coefficient (N*s/m).

    Returns:
        The list of created `ChLinkTSDA` links.
    """
    if span < 2:
        raise ValueError("span must be >= 2 for a bending proxy")
    if len(chain.segments) <= span:
        return []

    links = []
    rest_length = chain.segment_length * span
    for i in range(len(chain.segments) - span):
        a = chain.segments[i]
        b = chain.segments[i + span]
        tsda = chrono.ChLinkTSDA()
        tsda.Initialize(a, b, False, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
        tsda.SetRestLength(rest_length)
        tsda.SetSpringCoefficient(float(spring_k))
        tsda.SetDampingCoefficient(float(damping_c))
        system.AddLink(tsda)
        links.append(tsda)

    chain.aux_links.extend(links)
    return links


def add_bending_rsdas(
    system,
    chain: YarnChainHandles,
    *,
    spring_k: float = 0.0,
    damping_c: float = 0.0,
    rest_angle: float = 0.0,
) -> list:
    """Add rotational spring-dampers at inter-segment joints.

    This helper attaches one ``ChLinkRSDA`` between each neighboring segment
    pair at the same location as the spherical joint connecting them. It acts as
    a compliant bending/twist resistance term (axis-dependent in the chosen RSDA
    frame), which is usually more stable and more physically meaningful than the
    translational spring proxy for this chain model.

    Notes:
        - One RSDA provides one rotational spring-damper axis.
        - With ``chrono.QUNIT`` as the RSDA frame, the axis is fixed in world
          coordinates according to Chrono's RSDA convention.
        - This is a practical approximation, not a full yarn constitutive model.

    Args:
        system: Chrono system receiving the RSDA links.
        chain: Built yarn chain handles with neighboring segment joints.
        spring_k: Rotational spring coefficient.
        damping_c: Rotational damping coefficient.
        rest_angle: Rest relative angle for the RSDA.

    Returns:
        The list of created ``ChLinkRSDA`` links.
    """
    if len(chain.segments) < 2:
        return []

    links = []
    seg_len = chain.segment_length
    for i in range(1, len(chain.segments)):
        a = chain.segments[i]
        b = chain.segments[i - 1]

        # Use the current interface point as the RSDA joint location. The chain
        # builder initializes adjacent segment interfaces at this point.
        pa = a.GetPos()
        pb = b.GetPos()
        p = chrono.ChVectorD(
            0.5 * (pa.x + pb.x),
            0.5 * (pa.y + pb.y),
            0.5 * (pa.z + pb.z),
        )

        rsda = chrono.ChLinkRSDA()
        rsda.Initialize(a, b, chrono.ChCoordsysD(p, chrono.QUNIT))
        rsda.SetRestAngle(float(rest_angle))
        rsda.SetSpringCoefficient(float(spring_k))
        rsda.SetDampingCoefficient(float(damping_c))
        rsda.SetNumInitRevolutions(0)
        system.AddLink(rsda)
        links.append(rsda)

    chain.aux_links.extend(links)
    return links
