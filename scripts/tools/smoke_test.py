"""
smoke_test.py — minimal Proof-of-Life for PyChrono + Irrlicht

- NSC system, Bullet collision if available
- Floor (top at y=0) + falling box
- Tries Irrlicht; if it fails (e.g., headless), runs ~0.5 s headless
"""

import pychrono as chrono

def set_gravity(sys, gvec):
    for name in ("Set_G_acc", "SetGravity", "Set_G_acceleration", "Set_Gacc"):
        if hasattr(sys, name):
            getattr(sys, name)(gvec)
            return
    if hasattr(sys, "Set_g_acc"):
        sys.Set_g_acc(gvec)
    else:
        raise AttributeError("No gravity setter in this PyChrono build")

def prefer_bullet(sys):
    try:
        sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass

def tune_collision_defaults(envelope=0.003, margin=0.002):
    try:
        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(envelope)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(margin)
    except Exception:
        pass

def make_system():
    sys = chrono.ChSystemNSC()
    prefer_bullet(sys)
    tune_collision_defaults(envelope=0.003, margin=0.002)
    try:
        sys.SetMaxPenetrationRecoverySpeed(0.5)
        sys.SetMinBounceSpeed(0.05)
    except Exception:
        pass
    set_gravity(sys, chrono.ChVectorD(0, -9.81, 0))
    return sys

def build_scene(sys):
    floor = chrono.ChBodyEasyBox(2, 0.1, 2, 1000, True, True)
    floor.SetBodyFixed(True)
    floor.SetPos(chrono.ChVectorD(0, -0.05, 0))
    sys.Add(floor)

    box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 500, True, True)
    box.SetPos(chrono.ChVectorD(0, 0.5, 0))
    sys.Add(box)
    return box

def try_visual(sys):
    try:
        import pychrono.irrlicht as irr
        vis = irr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetWindowSize(1024, 640)
        vis.SetWindowTitle("Chrono smoke test")
        vis.Initialize()
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddCamera(chrono.ChVectorD(1.2, 0.7, 1.2), chrono.ChVectorD(0, 0, 0))
        try: vis.BindAll()
        except Exception: pass
        return vis
    except Exception as e:
        print("[info] Visualization not available:", e)
        return None

def main():
    sys = make_system()
    box = build_scene(sys)
    vis = try_visual(sys)

    dt, t_end = 1e-3, 0.5
    ran_visual = False

    if vis:
        try:
            while vis.Run() and sys.GetChTime() < t_end:
                vis.BeginScene(); vis.Render(); vis.EndScene()
                sys.DoStepDynamics(dt)
                ran_visual = True
        except Exception as e:
            print("[warn] visual loop failed:", e)

    if not ran_visual:
        print("[info] running headless for ~0.5 s")
        while sys.GetChTime() < t_end:
            sys.DoStepDynamics(dt)

    p = box.GetPos()
    print(f"OK: t={sys.GetChTime():.3f}s, box y={p.y:.3f} m")

if __name__ == "__main__":
    main()
