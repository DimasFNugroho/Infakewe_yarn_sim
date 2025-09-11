"""
links_02_motors_viz.py (Chrono 8.x safe, Irrlicht)
==================================================

Level 2.1 — Links & joints: motors (visual)

What this shows
---------------
Three short bars, each hinged at its left end:
  - Left: passive revolute (no motor) → swings under gravity.
  - Middle: speed motor (ChLinkMotorRotationSpeed) → angle follows commanded speed.
  - Right: torque motor (ChLinkMotorRotationTorque) → constant torque; angle accelerates dynamically.

Robust choices
--------------
- Manual bodies & visuals, collisions off (we focus on joints).
- Bullet collision system forced, but unused here.
- Colors via ChVisualMaterial; Chrono 8.x–safe API usage.

Run
---
    conda activate chrono
    python -u scripts/tutorials/level_2_1_links_and_joints/links_02_motors_viz.py
"""

import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr


def set_color(vshape, rgb):
    mat = chrono.ChVisualMaterial()
    mat.SetDiffuseColor(chrono.ChColor(*rgb))
    if hasattr(vshape, "GetMaterials"):
        vshape.GetMaterials().push_back(mat)
    elif hasattr(vshape, "material_list"):
        vshape.material_list.push_back(mat)


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

    # draw a beam for reference
    beam = chrono.ChBoxShape()
    beam.GetBoxGeometry().Size = chrono.ChVectorD(1.6, 0.01, 0.05)
    set_color(beam, (0.55, 0.55, 0.55))
    g.AddVisualShape(beam, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    return g


def make_bar(sys, length=0.4, thickness=0.04, density=500.0, color=(0.7, 0.7, 0.7)):
    # aligned along local +X
    sx, sy, sz = length, thickness, thickness
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5

    volume = sx * sy * sz
    mass = max(1e-6, density * volume)
    Ix = (1.0 / 12.0) * mass * (sy * sy + sz * sz)
    Iy = (1.0 / 12.0) * mass * (sx * sx + sz * sz)
    Iz = (1.0 / 12.0) * mass * (sx * sx + sy * sy)

    b = chrono.ChBody()
    b.SetMass(mass)
    b.SetInertiaXX(chrono.ChVectorD(Ix, Iy, Iz))
    b.SetCollide(False)

    vb = chrono.ChBoxShape()
    vb.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    set_color(vb, color)
    b.AddVisualShape(vb)

    sys.Add(b)
    return b


def place_bar_with_left_end_at(bar, pivot, length, angle_rad):
    q = chrono.Q_from_AngAxis(angle_rad, chrono.ChVectorD(0, 0, 1))
    R = chrono.ChMatrix33D(q)
    com_local = chrono.ChVectorD(length * 0.5, 0, 0)  # COM half length from left end
    bar.SetPos(pivot + R * com_local)
    bar.SetRot(q)


def main():
    sys = make_system()
    ground = make_ground(sys)

    L = 0.4
    # left/mid/right pivots
    pivL = chrono.ChVectorD(-0.7, 0.0, 0.0)
    pivM = chrono.ChVectorD( 0.0, 0.0, 0.0)
    pivR = chrono.ChVectorD(+0.7, 0.0, 0.0)

    # A) Passive revolute (blue), start at 45 deg
    barA = make_bar(sys, length=L, color=(0.2, 0.6, 0.9))
    place_bar_with_left_end_at(barA, pivL, L, angle_rad=math.radians(45))
    hingeA = chrono.ChLinkLockRevolute()
    hingeA.Initialize(barA, ground, chrono.ChCoordsysD(pivL, chrono.ChQuaternionD(1, 0, 0, 0)))
    sys.AddLink(hingeA)

    # B) Speed motor (yellow), start at 0 deg, omega = 2 rad/s
    barB = make_bar(sys, length=L, color=(0.9, 0.8, 0.2))
    place_bar_with_left_end_at(barB, pivM, L, angle_rad=0.0)
    motorB = chrono.ChLinkMotorRotationSpeed()
    motorB.Initialize(barB, ground, chrono.ChFrameD(pivM, chrono.ChQuaternionD(1, 0, 0, 0)))
    motorB.SetSpeedFunction(chrono.ChFunction_Const(2.0))
    sys.AddLink(motorB)

    # C) Torque motor (green), start at 0 deg, tau = 0.05 N·m
    barC = make_bar(sys, length=L, color=(0.2, 0.9, 0.2))
    place_bar_with_left_end_at(barC, pivR, L, angle_rad=0.0)
    motorC = chrono.ChLinkMotorRotationTorque()
    motorC.Initialize(barC, ground, chrono.ChFrameD(pivR, chrono.ChQuaternionD(1, 0, 0, 0)))
    motorC.SetTorqueFunction(chrono.ChFunction_Const(0.05))
    sys.AddLink(motorC)

    # Visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1100, 720)
    vis.SetWindowTitle("Links 02 — Motors vs Constraints (Level 2.1)")
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
