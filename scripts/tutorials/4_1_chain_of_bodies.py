"""
4_1_chain_of_bodies.py  (Chrono 8.x safe, Irrlicht)
===================================================

Level 4.1.1 — Rope approximation with rigid bodies:
Horizontal chain of short segments (cylinder + end spheres) linked by
spherical joints. The **left end is locked** to ground; the rest is free,
so the rope starts horizontal and **sags under gravity**.

Run:
    conda activate chrono
    python -u scripts/tutorials/4_1_chain_of_bodies.py
"""

import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr

def colorize(vshape, rgb):
    mat = chrono.ChVisualMaterial(); mat.SetDiffuseColor(chrono.ChColor(*rgb))
    (vshape.GetMaterials() if hasattr(vshape, "GetMaterials") else vshape.material_list).push_back(mat)

def make_system():
    sys = chrono.ChSystemNSC()
    try: sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except: pass
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    return sys

def make_capsule_segment(sys, half_len=0.05, rad=0.015, density=500.0, color=(0.2, 0.6, 0.9)):
    L = 2.0 * half_len + 2.0 * rad
    vol_cyl = math.pi * rad * rad * (2.0 * half_len)
    vol_sph = (4.0/3.0) * math.pi * rad**3
    volume = vol_cyl + vol_sph
    mass = max(1e-6, density * volume)
    Ixx = Izz = (1.0/12.0) * mass * (L * L) + 0.25 * mass * (rad * rad)
    Iyy = 0.5 * mass * (rad * rad)

    b = chrono.ChBody()
    b.SetMass(mass); b.SetInertiaXX(chrono.ChVectorD(Ixx, Iyy, Izz)); b.SetCollide(False)

    cyl = chrono.ChCylinderShape()
    cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(0, -half_len, 0)
    cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0, +half_len, 0)
    cyl.GetCylinderGeometry().rad = rad
    colorize(cyl, color); b.AddVisualShape(cyl)

    s_top = chrono.ChSphereShape(); s_top.GetSphereGeometry().rad = rad
    s_bot = chrono.ChSphereShape(); s_bot.GetSphereGeometry().rad = rad
    colorize(s_top, color); colorize(s_bot, color)
    b.AddVisualShape(s_top, chrono.ChFrameD(chrono.ChVectorD(0, +half_len, 0)))
    b.AddVisualShape(s_bot, chrono.ChFrameD(chrono.ChVectorD(0, -half_len, 0)))

    sys.Add(b); return b

def build_horizontal_rope(sys, anchor_left, n=18, seg_len=0.06, rad=0.012):
    """
    Place N segments **horizontally** along +X starting at 'anchor_left'.
    Each segment's local +Y is rotated to world +X (Rz(-90°)).
    Left end is spherical-joined to ground.
    """
    half_len = seg_len * 0.5
    q = chrono.Q_from_AngAxis(-math.pi/2, chrono.ChVectorD(0,0,1))  # +Y -> +X
    R = chrono.ChMatrix33D(q)

    segs = []
    for i in range(n):
        c = 0.20 + 0.6 * i / max(1, n-1)
        seg = make_capsule_segment(sys, half_len=half_len, rad=rad, color=(c, 0.55, 0.85))
        left_end = anchor_left + chrono.ChVectorD(i * seg_len, 0, 0)        # world
        com = left_end + R * chrono.ChVectorD(0, +half_len, 0)              # make left end coincide
        seg.SetPos(com); seg.SetRot(q)
        segs.append(seg)

    # Fixed ground & joints
    ground = chrono.ChBody(); ground.SetBodyFixed(True); ground.SetCollide(False); sys.Add(ground)

    joints = []
    # left anchor (segment 0 left end at anchor_left)
    j0 = chrono.ChLinkLockSpherical()
    j0.Initialize(segs[0], ground, chrono.ChCoordsysD(anchor_left, chrono.QUNIT))
    sys.AddLink(j0); joints.append(j0)

    # inter-segment joints at interfaces: anchor_left + k*seg_len
    for k in range(1, n):
        p = anchor_left + chrono.ChVectorD(k * seg_len, 0, 0)
        j = chrono.ChLinkLockSpherical()
        j.Initialize(segs[k], segs[k-1], chrono.ChCoordsysD(p, chrono.QUNIT))
        sys.AddLink(j); joints.append(j)

    return segs, joints

def main():
    sys = make_system()

    # Build horizontal rope at y=0.8, x from -0.7 → …
    anchor = chrono.ChVectorD(-0.7, 0.8, 0.0)
    segs, _ = build_horizontal_rope(sys, anchor_left=anchor, n=18, seg_len=0.06, rad=0.012)

    # Simple reference bar at y=0
    ref = chrono.ChBody(); ref.SetBodyFixed(True); ref.SetCollide(False)
    bar = chrono.ChBoxShape(); bar.GetBoxGeometry().Size = chrono.ChVectorD(0.8, 0.01, 0.05)
    colorize(bar, (0.55,0.55,0.55)); ref.AddVisualShape(bar, chrono.ChFrameD(chrono.ChVectorD(0,0,0))); sys.Add(ref)

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys); vis.SetWindowSize(1200,720)
    vis.SetWindowTitle("Level 4.1.1 — Horizontal rope with left end locked")
    vis.Initialize(); vis.AddSkyBox(); vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVectorD(2.2,1.3,2.2), chrono.ChVectorD(0.1,0.5,0.0))
    try: vis.BindAll()
    except: pass

    dt = 8e-4; next_print = 0.0
    while vis.Run():
        vis.BeginScene(); vis.Render(); vis.EndScene()
        sys.DoStepDynamics(dt)
        t = sys.GetChTime()
        if t >= next_print:
            tail = segs[-1].GetPos()
            print(f"t={t:5.2f}s  tail=({tail.x:+.3f}, {tail.y:+.3f}, {tail.z:+.3f})")
            next_print += 0.1

if __name__ == "__main__":
    main()
