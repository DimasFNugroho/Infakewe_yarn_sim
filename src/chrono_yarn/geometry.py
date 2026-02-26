"""Geometry and rigid-body construction helpers for yarn simulation scenes.

The functions in this module should be responsible for:
- creating Chrono bodies,
- attaching visual shapes,
- configuring collision shapes/models,
- adding bodies to a system.

They are kept separate from scene assembly to make geometry reusable and easier
to test in isolation.
"""

from __future__ import annotations

import math

import pychrono as chrono


def colorize(vshape, rgb: tuple[float, float, float]) -> None:
    """Apply a diffuse color to a Chrono visual shape."""
    mat = chrono.ChVisualMaterial()
    mat.SetDiffuseColor(chrono.ChColor(*rgb))
    mats = vshape.GetMaterials() if hasattr(vshape, "GetMaterials") else vshape.material_list
    mats.push_back(mat)


def add_floor_box(
    system,
    half_size: tuple[float, float, float],
    position: tuple[float, float, float],
    material,
    color: tuple[float, float, float] = (0.6, 0.6, 0.6),
):
    """Create and add a fixed floor body with visual and optional collision geometry.

    Args:
        system: The Chrono system to add the floor body to.
        half_size: Half-dimensions `(hx, hy, hz)` of the floor box.
        position: World position of the floor body center.
        material: Contact material compatible with the active contact model. If
            ``None``, collision is disabled and only a visual body is created.
        color: RGB diffuse color for visualization.

    Returns:
        The created Chrono body representing the floor.

    """
    hx, hy, hz = half_size
    px, py, pz = position

    body = chrono.ChBody()
    body.SetBodyFixed(True)
    body.SetPos(chrono.ChVectorD(px, py, pz))

    vis = chrono.ChBoxShape()
    vis.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    colorize(vis, color)
    body.AddVisualShape(vis)

    if material is None:
        body.SetCollide(False)
    else:
        cm = body.GetCollisionModel()
        cm.ClearModel()
        cm.AddBox(material, hx, hy, hz, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
        cm.BuildModel()
        body.SetCollide(True)

    system.Add(body)
    return body


def add_yarn_segment_capsule(
    system,
    half_len: float,
    radius: float,
    density: float,
    material,
    color: tuple[float, float, float] = (0.2, 0.6, 0.9),
):
    """Create and add one rigid yarn segment body.

    Args:
        system: The Chrono system receiving the body.
        half_len: Half of the cylindrical centerline length of the segment.
        radius: Segment radius.
        density: Material density used to estimate mass and inertia.
        material: Contact material compatible with the active contact model. If
            ``None``, collision is disabled and only visual geometry is added.
        color: RGB diffuse color for visualization.

    Returns:
        The created Chrono rigid body for the segment.

    """
    total_len = 2.0 * half_len + 2.0 * radius
    vol_cyl = math.pi * radius * radius * (2.0 * half_len)
    vol_sph = (4.0 / 3.0) * math.pi * radius**3
    mass = max(1e-6, density * (vol_cyl + vol_sph))

    i_xz = (1.0 / 12.0) * mass * (total_len * total_len) + 0.25 * mass * (radius * radius)
    i_y = 0.5 * mass * (radius * radius)

    body = chrono.ChBody()
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVectorD(i_xz, i_y, i_xz))

    # Capsule-like visual representation: cylinder + end spheres, local axis = +Y.
    cyl = chrono.ChCylinderShape()
    cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(0, -half_len, 0)
    cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0, +half_len, 0)
    cyl.GetCylinderGeometry().rad = radius
    colorize(cyl, color)
    body.AddVisualShape(cyl)

    cap_a = chrono.ChSphereShape()
    cap_a.GetSphereGeometry().rad = radius
    cap_b = chrono.ChSphereShape()
    cap_b.GetSphereGeometry().rad = radius
    colorize(cap_a, color)
    colorize(cap_b, color)
    body.AddVisualShape(cap_a, chrono.ChFrameD(chrono.ChVectorD(0, +half_len, 0)))
    body.AddVisualShape(cap_b, chrono.ChFrameD(chrono.ChVectorD(0, -half_len, 0)))

    if material is None:
        body.SetCollide(False)
    else:
        # Conservative fallback collision approximation using a box aligned with
        # the segment local Y axis. This is sufficient for early prototypes and
        # keeps compatibility across PyChrono builds.
        cm = body.GetCollisionModel()
        cm.ClearModel()
        cm.AddBox(
            material,
            radius,
            half_len + radius,
            radius,
            chrono.ChVectorD(0, 0, 0),
            chrono.ChMatrix33D(1),
        )
        cm.BuildModel()
        body.SetCollide(True)

    system.Add(body)
    return body
