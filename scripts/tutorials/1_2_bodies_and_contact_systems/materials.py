"""
materials.py (robust, verbose, Bullet, ASCII)
=======================================================

Goal:
    Demonstrate how material properties (friction, restitution, stiffness)
    affect simple scenarios in Chrono.

Stability tweaks for PyChrono 8.0.0 on Windows:
  - Force the Bullet collision system when available.
  - Avoid ChBodyEasy* helpers; build bodies manually with explicit collision shapes
    (AddBox/AddSphere) and attach explicit materials (NSC or SMC).

Experiments:
  A) Friction (sliding test, NSC):
     - A small box gets an initial horizontal velocity on a floor.
     - Compare low vs high friction: the sliding distance differs.

  B) Restitution (bounciness test, NSC):
     - A sphere drops onto a floor.
     - Compare low vs higher restitution: the max rebound height differs.

  C) Stiffness via Young's modulus (SMC only):
     - Same drop with two Young's moduli.
     - We report the most negative "penetration" (sphere bottom - floor top).
       Negative means compliant overlap (softer material => more negative).

Run:
    python -u scripts/tutorials/bodies_03_materials.py
"""

import math
import pychrono as chrono


# ---------- common helpers ----------

def make_system(kind: str = "NSC") -> "chrono.ChSystem":
    """
    Create an NSC or SMC system, force Bullet collision (if available), and set gravity.
    """
    sys = chrono.ChSystemNSC() if kind.upper() == "NSC" else chrono.ChSystemSMC()
    try:
        # Safer on some PyChrono 8.0.0 Windows setups
        sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    return sys


def set_min_bounce_zero_if_supported(mat):
    """
    Some builds expose a minimum bounce speed/velocity threshold on the material.
    Set it to 0 if available so small bounces are not suppressed.
    """
    for name in ("SetMinBounceSpeed", "SetMinBounceVelocity"):
        if hasattr(mat, name):
            getattr(mat, name)(0.0)


def make_material(is_smc: bool, *, friction=None, restitution=None, young=None, poisson=None):
    """
    Create a contact material (NSC or SMC) and set the provided properties if supported.
    """
    if is_smc:
        mat = chrono.ChMaterialSurfaceSMC()
        if young is not None and hasattr(mat, "SetYoungModulus"):
            mat.SetYoungModulus(float(young))
        if poisson is not None and hasattr(mat, "SetPoissonRatio"):
            mat.SetPoissonRatio(float(poisson))
    else:
        mat = chrono.ChMaterialSurfaceNSC()

    if friction is not None and hasattr(mat, "SetFriction"):
        mat.SetFriction(float(friction))
    if restitution is not None and hasattr(mat, "SetRestitution"):
        mat.SetRestitution(float(restitution))

    set_min_bounce_zero_if_supported(mat)
    return mat


def make_floor(sys, mat, size=(2.0, 0.05, 2.0), pos=(0.0, -0.025, 0.0)):
    """
    Build a fixed floor body with a box collision shape and simple visual.
    Floor top will be at y = pos.y + size_y/2.
    """
    sx, sy, sz = size
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5

    body = chrono.ChBody()
    body.SetBodyFixed(True)
    body.SetPos(chrono.ChVectorD(*pos))
    body.SetMass(1.0)
    body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))

    # visual
    vbox = chrono.ChBoxShape()
    vbox.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    body.AddVisualShape(vbox)

    # collision
    body.SetCollide(True)
    cm = body.GetCollisionModel()
    cm.ClearModel()
    cm.AddBox(mat, hx, hy, hz, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
    cm.BuildModel()

    sys.Add(body)
    return body


def make_box(sys, mat, size=(0.1, 0.1, 0.1), pos=(-0.5, 0.10 + 0.05, 0.0), density=500.0):
    """
    Dynamic box with manual inertia and box collision shape.
    Size = (sx, sy, sz), center at 'pos'. Half-height is sy/2.
    """
    sx, sy, sz = size
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5

    volume = sx * sy * sz
    mass = max(1e-6, density * volume)
    Ix = (1.0 / 12.0) * mass * (sy * sy + sz * sz)
    Iy = (1.0 / 12.0) * mass * (sx * sx + sz * sz)
    Iz = (1.0 / 12.0) * mass * (sx * sx + sy * sy)

    body = chrono.ChBody()
    body.SetPos(chrono.ChVectorD(*pos))
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVectorD(Ix, Iy, Iz))

    vbox = chrono.ChBoxShape()
    vbox.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    body.AddVisualShape(vbox)

    body.SetCollide(True)
    cm = body.GetCollisionModel()
    cm.ClearModel()
    cm.AddBox(mat, hx, hy, hz, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
    cm.BuildModel()

    sys.Add(body)
    return body


def make_sphere(sys, mat, radius=0.06, pos=(0.0, 0.80, 0.0), density=500.0):
    """
    Dynamic sphere with manual inertia and sphere collision shape.
    """
    body = chrono.ChBody()
    body.SetPos(chrono.ChVectorD(*pos))

    volume = (4.0 / 3.0) * math.pi * radius**3
    mass = max(1e-6, density * volume)
    I = 0.4 * mass * radius * radius  # 2/5 m r^2
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVectorD(I, I, I))

    vs = chrono.ChSphereShape()
    vs.GetSphereGeometry().rad = radius
    body.AddVisualShape(vs)

    body.SetCollide(True)
    cm = body.GetCollisionModel()
    cm.ClearModel()
    cm.AddSphere(mat, radius, chrono.ChVectorD(0, 0, 0))
    cm.BuildModel()

    sys.Add(body)
    return body


# ---------- A) friction sliding (NSC) ----------

def friction_sliding(mu_floor: float, mu_box: float, v0: float = 1.0, t_end: float = 1.0, dt: float = 1e-3, print_dt: float = 0.20) -> float:
    """
    Slide a small box with initial horizontal velocity across a floor (NSC) and
    return the traveled distance along +X. Prints a short time-series so you
    can see progress and confirm no premature exit.

    Args:
        mu_floor: floor friction coefficient.
        mu_box: box friction coefficient.
        v0: initial X velocity (m/s).
        t_end: simulation duration (s).
        dt: integration step (s).
        print_dt: print interval (s).

    Returns:
        Distance traveled in X (m).
    """
    sys = make_system("NSC")

    # Materials (low restitution to suppress bouncing)
    mat_floor = make_material(False, friction=mu_floor, restitution=0.0)
    mat_box   = make_material(False, friction=mu_box,   restitution=0.0)

    # Scene
    floor = make_floor(sys, mat_floor, size=(2.0, 0.05, 2.0), pos=(0.0, -0.025, 0.0))
    box   = make_box(  sys, mat_box,   size=(0.1, 0.1, 0.1),  pos=(-0.5, 0.10 + 0.05, 0.0))

    # Initial linear velocity along +X
    box.SetPos_dt(chrono.ChVectorD(v0, 0, 0))

    steps = int(round(t_end / dt))
    print_every = max(1, int(round(print_dt / dt)))
    print(f"[Friction] mu_floor={mu_floor}, mu_box={mu_box}, v0={v0} m/s")
    print("t(s)\tbox_x\tbox_y")
    print(f"{0.00:4.2f}\t{box.GetPos().x:+.5f}\t{box.GetPos().y:+.5f}", flush=True)

    for i in range(1, steps + 1):
        sys.DoStepDynamics(dt)
        if (i % print_every == 0) or (i == steps):
            t = sys.GetChTime()
            p = box.GetPos()
            print(f"{t:4.2f}\t{p.x:+.5f}\t{p.y:+.5f}", flush=True)

    x0 = -0.5
    xf = box.GetPos().x
    dist = xf - x0
    print(f"[Friction] distance traveled = {dist:.3f} m")
    return dist


# ---------- B) restitution bounce (NSC) ----------

def restitution_bounce(e_sphere: float, e_floor: float, t_end: float = 1.2, dt: float = 1e-3, print_dt: float = 0.10) -> float:
    """
    Drop a sphere on a floor in NSC and return the maximum rebound height.
    Prints a short time-series of the sphere center height.

    Args:
        e_sphere: restitution of the sphere.
        e_floor: restitution of the floor.
        t_end: duration (s).
        dt: integration step (s).
        print_dt: print interval (s).

    Returns:
        Maximum Y of the sphere center (m).
    """
    sys = make_system("NSC")

    # Materials (separate to demonstrate asymmetry if desired)
    mat_floor = make_material(False, friction=0.3, restitution=e_floor)
    mat_sph   = make_material(False, friction=0.2, restitution=e_sphere)

    # Scene
    floor = make_floor(sys, mat_floor, size=(2.0, 0.05, 2.0), pos=(0.0, -0.025, 0.0))
    r = 0.06
    sph = make_sphere(sys, mat_sph, radius=r, pos=(0.0, 0.80, 0.0))

    steps = int(round(t_end / dt))
    print_every = max(1, int(round(print_dt / dt)))
    max_y = sph.GetPos().y

    print(f"[Restitution] e_sphere={e_sphere}, e_floor={e_floor}")
    print("t(s)\tsphere_y")
    print(f"{0.00:4.2f}\t{sph.GetPos().y:+.5f}", flush=True)

    for i in range(1, steps + 1):
        sys.DoStepDynamics(dt)
        if (i % print_every == 0) or (i == steps):
            t = sys.GetChTime()
            y = sph.GetPos().y
            print(f"{t:4.2f}\t{y:+.5f}", flush=True)
            if y > max_y:
                max_y = y

    print(f"[Restitution] max height = {max_y:.3f} m")
    return max_y


# ---------- C) SMC stiffness (penetration proxy) ----------

def smc_penetration(young: float, poisson: float = 0.3, t_end: float = 0.8, dt: float = 1e-3, print_dt: float = 0.10) -> float:
    """
    In SMC, drop a sphere and report the most negative penetration estimate
    (sphere bottom - floor top). Negative => compliant overlap.

    Args:
        young: Young's modulus for both bodies (Pa).
        poisson: Poisson ratio.
        t_end: duration (s).
        dt: integration step (s).
        print_dt: print interval (s).

    Returns:
        Minimum penetration (m). Negative means overlap.
    """
    sys = make_system("SMC")

    mat = make_material(True, friction=0.3, restitution=0.1, young=young, poisson=poisson)

    # Scene: floor centered at y=0 with thickness 0.02 -> top at y=+0.01
    floor_thick = 0.02
    floor = make_floor(sys, mat, size=(1.0, floor_thick, 1.0), pos=(0.0, 0.0, 0.0))
    r = 0.05
    sph = make_sphere(sys, mat, radius=r, pos=(0.0, 0.50, 0.0))

    steps = int(round(t_end / dt))
    print_every = max(1, int(round(print_dt / dt)))
    floor_top_y = floor_thick * 0.5
    min_pen = None

    print(f"[SMC stiffness] young={young:.2e} Pa, poisson={poisson}")
    print("t(s)\tsphere_y\tpenetration")
    y0 = sph.GetPos().y
    pen0 = (y0 - r) - floor_top_y
    print(f"{0.00:4.2f}\t{y0:+.5f}\t{pen0:+.6f}", flush=True)

    for i in range(1, steps + 1):
        sys.DoStepDynamics(dt)
        if (i % print_every == 0) or (i == steps):
            t = sys.GetChTime()
            y = sph.GetPos().y
            pen = (y - r) - floor_top_y
            print(f"{t:4.2f}\t{y:+.5f}\t{pen:+.6f}", flush=True)
            if (min_pen is None) or (pen < min_pen):
                min_pen = pen

    print(f"[SMC stiffness] min penetration = {min_pen:.6f} m (negative = overlap)")
    return min_pen if min_pen is not None else 0.0


def main():
    """
    Run the three experiments and print concise results at the end.
    """
    # A) friction sliding: low vs high mu
    d_low  = friction_sliding(mu_floor=0.05, mu_box=0.05, v0=1.0, t_end=1.0, dt=1e-3, print_dt=0.20)
    d_high = friction_sliding(mu_floor=0.80, mu_box=0.80, v0=1.0, t_end=1.0, dt=1e-3, print_dt=0.20)
    print(f"[Friction] distance low mu ~ {d_low:.3f} m, high mu ~ {d_high:.3f} m")

    # B) restitution bounce: low vs high e
    h_low  = restitution_bounce(e_sphere=0.10, e_floor=0.10, t_end=1.2, dt=1e-3, print_dt=0.10)
    h_high = restitution_bounce(e_sphere=0.80, e_floor=0.80, t_end=1.2, dt=1e-3, print_dt=0.10)
    print(f"[Restitution] max height low e ~ {h_low:.3f} m, high e ~ {h_high:.3f} m")

    # C) SMC stiffness: soft vs stiff
    pen_soft  = smc_penetration(young=5e6, poisson=0.3, t_end=0.8, dt=1e-3, print_dt=0.10)
    pen_stiff = smc_penetration(young=5e8, poisson=0.3, t_end=0.8, dt=1e-3, print_dt=0.10)
    print(f"[SMC stiffness] min penetration soft ~ {pen_soft:.6f} m, stiff ~ {pen_stiff:.6f} m (negative = overlap)")

    print("OK [done]")


if __name__ == "__main__":
    main()
