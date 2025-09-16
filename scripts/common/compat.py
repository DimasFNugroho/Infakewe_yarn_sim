# scripts/common/compat.py
import pychrono as chrono

def set_gravity(sys, gvec):
    """Cross-version gravity setter for ChSystemNSC/SMC across PyChrono builds."""
    for name in ("Set_G_acc", "SetGravity", "Set_G_acceleration", "Set_Gacc"):
        if hasattr(sys, name):
            getattr(sys, name)(gvec); return
    if hasattr(sys, "Set_g_acc"):
        sys.Set_g_acc(gvec)
    else:
        raise AttributeError("No gravity setter in this PyChrono build")

def prefer_bullet(sys):
    """Prefer Bullet collision if available; ignore if not present."""
    try:
        sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass

def make_vis(sys, title, width=1280, height=760, enable=True):
    """Create Irrlicht visual system or return None if unavailable/disabled."""
    if not enable:
        return None
    try:
        import pychrono.irrlicht as irr
        vis = irr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetWindowSize(width, height)
        vis.SetWindowTitle(title)
        vis.Initialize()
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddCamera(chrono.ChVectorD(1.2, 0.7, 1.2), chrono.ChVectorD(0, 0, 0))
        try: vis.BindAll()
        except Exception: pass
        return vis
    except Exception:
        return None
