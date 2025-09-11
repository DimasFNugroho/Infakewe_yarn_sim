"""
links_01_constraints_viz.py (Chrono 8.x safe, Irrlicht)
=======================================================

Level 2.1 — Links & joints: constraints (visual)

What this shows
---------------
Two pendulums hung from a fixed ground:
  - Left: ChLinkLockRevolute (hinge about Z) -> stays in a plane.
  - Right: ChLinkLockSpherical (ball) -> can swing out of plane (we add a tiny x-spin).

Why this version is robust
--------------------------
- Uses manual bodies and visual shapes (no ChBodyEasy* pitfalls).
- Forces Bullet collision (though collisions are off here).
- Colors via ChVisualMaterial (no deprecated ChColorAsset).
- Works with PyChrono 8.x Python bindings.

Run
---
    conda activate chrono
    python -u scripts/tutorials/level_2_1_links_and_joints/links_01_constraints_viz.py
"""

import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr


# ---------- visuals helpers ----------

def set_color(vshape, rgb=(0.7, 0.7, 0.7)):
    mat = chrono.ChVisualMaterial()
    mat.SetDiffuseColor(chrono.ChColor(*rgb))
    if hasattr(vshape, "GetMaterials"):
        vshape.GetMaterials().push_back(mat)
    elif hasattr(vshape, "material_list"):
        vshape.material_list.push_back(mat)


# ---------- system & bodies ----------

def make_system():
    sys = chrono.ChSystemNSC()
    try:
        sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    return sys


def make_ground(sys):
    g = chrono.ChBody()
    g.SetBodyFixed(True)
    g.SetCollide(False)
    sys.Add(g)

    # draw a thin horizontal beam for reference
    beam = chrono.ChBoxShape()
    beam.GetBoxGeometry().Size = chrono.ChVectorD(1.2, 0.01, 0.05)
    set_color(beam, (0.55, 0.55, 0.55))
    g.AddVisualShape(beam, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))

    return g


def make_rod(sys, length=1.0, thickness=0.05, density=500.0, color=(0.2, 0.6, 0.9)):
    # Aligned along local +Y
    sx, sy, sz = thickness, length, thickness
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5

    volume = sx * sy * sz
    mass = max(1e-6, density * volume)
    Ix = (1.0 / 12.0) * mass * (sy * sy + sz * sz)
    Iy = (1.0 / 12.0) * mass * (sx * sx + sz * sz)
    Iz = (1.0 / 12.0) * mass * (sx * sx + sy * sy)

    b = chrono.ChBody()
    b.SetMass(mass)
    b.SetInertiaXX(chrono.ChVectorD(Ix, Iy, Iz))
    b.SetCollide(False)  # no contacts in this demo

    vbox = chrono.ChBoxShape()
    vbox.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    set_color(vbox, color)
    b.AddVisualShape(vbox)

    sys.Add(b)
    return b


def place_pendulum(rod, pivot, length, theta_deg, axis=chrono.ChVectorD(0, 0, 1)):
    # angle from vertical-down about 'axis'; rod local +Y is lengthwise
    theta = math.radians(theta_deg)
    q = chrono.Q_from_AngAxis(theta, axis)
    R = chrono.ChMatrix33D(q)
    offset_local = chrono.ChVectorD(0, length * 0.5, 0)  # from COM to top end
    com_world = pivot - R * offset_local                  # COM so top end = pivot
    rod.SetPos(com_world)
    rod.SetRot(q)


def main():
    sys = make_system()
    ground = make_ground(sys)

    # Pivots along the beam
    L = 1.0
    pivot_left  = chrono.ChVectorD(-0.6, 0.0, 0.0)
    pivot_right = chrono.ChVectorD(+0.6, 0.0, 0.0)

    # Left: revolute hinge (planar)
    rod_rev = make_rod(sys, length=L, color=(0.2, 0.6, 0.9))
    place_pendulum(rod_rev, pivot_left, L, theta_deg=25)
    rev = chrono.ChLinkLockRevolute()
    rev.Initialize(rod_rev, ground, chrono.ChCoordsysD(pivot_left, chrono.ChQuaternionD(1, 0, 0, 0)))
    sys.AddLink(rev)

    # Right: spherical ball joint (3D)
    rod_sph = make_rod(sys, length=L, color=(0.2, 0.9, 0.2))
    place_pendulum(rod_sph, pivot_right, L, theta_deg=25)
    # tiny angular velocity about +X to kick it out of plane
    rod_sph.SetWvel_loc(chrono.ChVectorD(0.6, 0.0, 0.0))
    sph = chrono.ChLinkLockSpherical()
    sph.Initialize(rod_sph, ground, chrono.ChCoordsysD(pivot_right, chrono.ChQuaternionD(1, 0, 0, 0)))
    sys.AddLink(sph)

    # Visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("Links 01 — Revolute vs Spherical (Level 2.1)")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVectorD(2.2, 1.2, 2.2), chrono.ChVectorD(0, -0.2, 0))

    try:
        vis.BindAll()
    except Exception:
        pass

    timestep = 1e-3
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(timestep)


if __name__ == "__main__":
    main()
