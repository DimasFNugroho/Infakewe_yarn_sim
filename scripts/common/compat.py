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

def tune_collision_defaults(envelope=0.003, margin=0.002):
    """Set global suggested envelope/margin (guards for older builds)."""
    try:
        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(envelope)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(margin)
    except Exception:
        pass

def set_single_thread(sys):
    """Force single-thread stepping to avoid rare OpenMP races in tests."""
    for name in ("SetNumThreads", "SetNumThreadsParallel"):
        if hasattr(sys, name):
            try: getattr(sys, name)(1)
            except Exception: pass
