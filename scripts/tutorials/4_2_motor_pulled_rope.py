"""
4_2_motor_pulled_rope.py  (Chrono 8.x safe, Irrlicht)
— Rope starts at the HOOK, laid to the LEFT (no initial gap).
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
    sys.Set_G_acc(chrono.ChVectorD(0,-9.81,0))
    return sys

def make_capsule_segment(sys, half_len, rad, density=500.0, color=(0.2,0.6,0.9)):
    L = 2.0*half_len + 2.0*rad
    vol_cyl = math.pi*rad*rad*(2.0*half_len); vol_sph = (4.0/3.0)*math.pi*rad**3
    mass = max(1e-6, density*(vol_cyl+vol_sph))
    Ixx = Izz = (1.0/12.0)*mass*(L*L) + 0.25*mass*(rad*rad); Iyy = 0.5*mass*(rad*rad)

    b = chrono.ChBody(); b.SetMass(mass); b.SetInertiaXX(chrono.ChVectorD(Ixx,Iyy,Izz)); b.SetCollide(False)
    cyl = chrono.ChCylinderShape(); cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(0,-half_len,0)
    cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0,+half_len,0); cyl.GetCylinderGeometry().rad = rad
    colorize(cyl,color); b.AddVisualShape(cyl)
    s1 = chrono.ChSphereShape(); s1.GetSphereGeometry().rad = rad
    s2 = chrono.ChSphereShape(); s2.GetSphereGeometry().rad = rad
    colorize(s1,color); colorize(s2,color)
    b.AddVisualShape(s1, chrono.ChFrameD(chrono.ChVectorD(0,+half_len,0)))
    b.AddVisualShape(s2, chrono.ChFrameD(chrono.ChVectorD(0,-half_len,0)))
    sys.Add(b); return b

def build_rope_from_anchor_left(sys, anchor_left, n=16, seg_len=0.055, rad=0.011):
    """
    Build rope horizontally to the LEFT of 'anchor_left' (decreasing X).
    The segment local +Y is rotated to world -X (Rz(+90°)).
    The anchor_left is the RIGHT end of segment 0 (at the drum hook).
    """
    half_len = seg_len * 0.5

    # Rotate local +Y -> world -X
    q = chrono.Q_from_AngAxis(+math.pi/2, chrono.ChVectorD(0, 0, 1))
    R = chrono.ChMatrix33D(q)

    segs = []
    for i in range(n):
        # right end of segment i sits i*seg_len to the LEFT of the hook
        right_end = anchor_left - chrono.ChVectorD(i * seg_len, 0, 0)

        # *** Correct COM offset ***
        # For this orientation, the RIGHT end corresponds to local -Y.
        # World vector (COM -> RIGHT end) = R * (0, -half_len, 0) = (+half_len, 0, 0).
        # So COM must be right_end - (+half_len, 0, 0).
        com = right_end - (R * chrono.ChVectorD(0, -half_len, 0))

        c = 0.25 + 0.5 * i / max(1, n - 1)
        seg = make_capsule_segment(sys, half_len, rad, color=(c, 0.55, 0.85))
        seg.SetPos(com)
        seg.SetRot(q)
        segs.append(seg)

    # Joints at interfaces exactly every seg_len to the LEFT of the hook
    for i in range(n - 1):
        p = anchor_left - chrono.ChVectorD((i + 1) * seg_len, 0, 0)
        j = chrono.ChLinkLockSpherical()
        j.Initialize(segs[i + 1], segs[i], chrono.ChCoordsysD(p, chrono.QUNIT))
        sys.AddLink(j)

    return segs

def main():
    sys = make_system()

    # reference bar at y=0 (just visual)
    ref = chrono.ChBody(); ref.SetBodyFixed(True); ref.SetCollide(False)
    bar = chrono.ChBoxShape(); bar.GetBoxGeometry().Size = chrono.ChVectorD(0.8,0.01,0.05)
    colorize(bar,(0.55,0.55,0.55)); ref.AddVisualShape(bar, chrono.ChFrameD(chrono.ChVectorD(0,0,0))); sys.Add(ref)

    # ground for motor
    ground = chrono.ChBody(); ground.SetBodyFixed(True); ground.SetCollide(False); sys.Add(ground)

    # motorized drum at (0.6, 0.5, 0)
    drum_center = chrono.ChVectorD(+0.6, 0.5, 0.0); drum_R, drum_L = 0.10, 0.20
    drum = chrono.ChBody(); drum.SetMass(1.0); drum.SetInertiaXX(chrono.ChVectorD(0.02,0.02,0.02))
    drum.SetPos(drum_center); drum.SetCollide(False)
    drum_vis = chrono.ChCylinderShape()
    drum_vis.GetCylinderGeometry().p1 = chrono.ChVectorD(0,-drum_L*0.5,0)
    drum_vis.GetCylinderGeometry().p2 = chrono.ChVectorD(0,+drum_L*0.5,0)
    drum_vis.GetCylinderGeometry().rad = drum_R; colorize(drum_vis,(0.8,0.8,0.8))
    drum.AddVisualShape(drum_vis); sys.Add(drum)

    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(drum, ground, chrono.ChFrameD(drum_center, chrono.QUNIT))
    motor.SetSpeedFunction(chrono.ChFunction_Const(2.0))  # rad/s
    sys.AddLink(motor)

    # hook welded to the rim (at +X on the drum)
    hook = chrono.ChBody(); hook.SetMass(0.1); hook.SetInertiaXX(chrono.ChVectorD(1e-4,1e-4,1e-4))
    hook.SetCollide(False); hook.SetPos(drum_center - chrono.ChVectorD(drum_R, 0, 0))
    s = chrono.ChSphereShape(); s.GetSphereGeometry().rad = 0.015; colorize(s,(0.9,0.4,0.2))
    hook.AddVisualShape(s); sys.Add(hook)
    weld = chrono.ChLinkLockLock(); weld.Initialize(hook, drum, chrono.ChCoordsysD(hook.GetPos(), chrono.QUNIT)); sys.AddLink(weld)

    # *** Build rope starting AT the hook, going to the LEFT (no gap) ***
    segs = build_rope_from_anchor_left(sys, anchor_left=hook.GetPos(), n=16, seg_len=0.055, rad=0.011)

    # top joint: rope[0] RIGHT end coincides with the hook point (already placed that way)
    jtop = chrono.ChLinkLockSpherical()
    jtop.Initialize(segs[0], hook, chrono.ChCoordsysD(hook.GetPos(), chrono.QUNIT))
    sys.AddLink(jtop)

    # visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys); vis.SetWindowSize(1280,760)
    vis.SetWindowTitle("Level 4.1.2 — Rope pulled by a motor drum (hook-anchored)")
    vis.Initialize(); vis.AddSkyBox(); vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVectorD(2.4,1.2,2.4), chrono.ChVectorD(0.4,0.3,0.0))
    try: vis.BindAll()
    except: pass

    dt = 8e-4; next_print = 0.0
    while vis.Run():
        vis.BeginScene(); vis.Render(); vis.EndScene()
        sys.DoStepDynamics(dt)
        t = sys.GetChTime()
        if t >= next_print:
            tip = segs[-1].GetPos()
            print(f"t={t:5.2f}s  tip=({tip.x:+.3f}, {tip.y:+.3f}, {tip.z:+.3f})  drum_w=2.0 rad/s")
            next_print += 0.1

if __name__ == "__main__":
    main()
