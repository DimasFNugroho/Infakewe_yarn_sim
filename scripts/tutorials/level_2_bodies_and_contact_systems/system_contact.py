"""
bodies_01_systems_contact.py (robust, verbose)
==============================================

Compare Chrono's two contact models by dropping a sphere onto a floor in:
  - ChSystemNSC  : non-smooth, constraint-based contact (no compliance)
  - ChSystemSMC  : smooth, penalty-based contact (compliant overlap)

Stability tweaks for PyChrono 8.0.0 on Windows:
  - Force Bullet collision system.
  - Avoid ChBodyEasy* helpers; build bodies manually and pass a material
    into collision shapes (AddBox/AddSphere).

This prints:
  - Construction details and material parameters.
  - A line at t = 0.00 s, then about every 0.10 s, and the final line.
  - An "Exited loop ..." marker, so you know the loop finished.

Run:
    python -u scripts/tutorials/level_2_bodies_and_contact_systems/system_contact.py
"""

import math
import pychrono as chrono


def make_material(is_smc, friction=0.5, restitution=0.4, young=2.0e7, poisson=0.3):
    """
    Create and configure a contact material compatible with the chosen system.

    Args:
        is_smc (bool): True for SMC material, False for NSC.
        friction (float): Coulomb friction coefficient.
        restitution (float): Normal restitution (0..1).
        young (float): Young's modulus in Pa (SMC only).
        poisson (float): Poisson ratio (SMC only).

    Returns:
        A Chrono material object (NSC or SMC).
    """
    if is_smc:
        mat = chrono.ChMaterialSurfaceSMC()
        if hasattr(mat, "SetYoungModulus"):
            mat.SetYoungModulus(float(young))
        if hasattr(mat, "SetPoissonRatio"):
            mat.SetPoissonRatio(float(poisson))
    else:
        mat = chrono.ChMaterialSurfaceNSC()

    if hasattr(mat, "SetFriction"):
        mat.SetFriction(float(friction))
    if hasattr(mat, "SetRestitution"):
        mat.SetRestitution(float(restitution))
    return mat


def make_floor(sys, mat, size=(1.0, 0.02, 1.0), pos=(0.0, 0.0, 0.0)):
    """
    Build a fixed floor body with manual collision (box) and a simple visual.

    Args:
        sys: Chrono system.
        mat: Contact material (NSC/SMC) to use for collision.
        size: (sx, sy, sz) full sizes in meters.
        pos:  (x, y, z) center position.

    Returns:
        The created ChBody.
    """
    sx, sy, sz = size
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5

    body = chrono.ChBody()
    body.SetBodyFixed(True)
    body.SetPos(chrono.ChVectorD(*pos))
    body.SetMass(1.0)
    body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))

    # Visual box (purely for debugging)
    vbox = chrono.ChBoxShape()
    vbox.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    body.AddVisualShape(vbox)

    # Collision box with explicit material
    body.SetCollide(True)
    cm = body.GetCollisionModel()
    cm.ClearModel()
    cm.AddBox(mat, hx, hy, hz, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
    cm.BuildModel()

    sys.Add(body)
    return body


def make_sphere(sys, mat, radius=0.05, pos=(0.0, 0.5, 0.0), density=1000.0):
    """
    Build a dynamic sphere with manual collision and a simple visual.

    Args:
        sys: Chrono system.
        mat: Contact material (NSC/SMC).
        radius (float): Sphere radius (m).
        pos (tuple):   Initial center position (x, y, z).
        density (float): Used to estimate mass/inertia (rough).

    Returns:
        The created ChBody.
    """
    body = chrono.ChBody()
    body.SetPos(chrono.ChVectorD(*pos))

    # Rough mass/inertia (uniform solid sphere): I = 2/5 m r^2
    volume = (4.0 / 3.0) * math.pi * radius**3
    mass = max(1e-6, density * volume)
    inertia = 0.4 * mass * radius * radius
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVectorD(inertia, inertia, inertia))

    # Visual
    vsph = chrono.ChSphereShape()
    vsph.GetSphereGeometry().rad = radius
    body.AddVisualShape(vsph)

    # Collision
    body.SetCollide(True)
    cm = body.GetCollisionModel()
    cm.ClearModel()
    cm.AddSphere(mat, radius, chrono.ChVectorD(0, 0, 0))
    cm.BuildModel()

    sys.Add(body)
    return body


def drop_once(system_kind="NSC", dt=1e-3, t_end=1.0, friction=0.5, restitution=0.4, young=2.0e7, poisson=0.3, verbose=True):
    """
    Run one drop test (floor + sphere) for the chosen contact model.

    Returns:
        dict: { system, final_y, min_penetration (SMC only), steps }
    """
    is_smc = system_kind.upper() == "SMC"
    sys = chrono.ChSystemSMC() if is_smc else chrono.ChSystemNSC()

    # Force Bullet collision system (safer on some 8.0.0 Windows setups)
    try:
        sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass

    if verbose:
        print(f"\n=== Building {'ChSystemSMC' if is_smc else 'ChSystemNSC'} ===")
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    if verbose:
        print("Gravity: (0, -9.81, 0) m/s^2")

    # Materials
    mat = make_material(is_smc, friction=friction, restitution=restitution, young=young, poisson=poisson)
    if verbose:
        print(f"Material type: {type(mat).__name__}  friction={friction}  restitution={restitution}"
              + (f"  young={young:.2e}  poisson={poisson}" if is_smc else ""))

    # Scene
    floor_thick = 0.02
    floor = make_floor(sys, mat, size=(1.0, floor_thick, 1.0), pos=(0.0, 0.0, 0.0))
    if verbose:
        print("Added floor: size=(1.0, 0.02, 1.0) @ y=0 (top ~= +0.01)")
    r = 0.05
    sph = make_sphere(sys, mat, radius=r, pos=(0.0, 0.5, 0.0))
    if verbose:
        print("Added sphere: radius=0.05 @ y=0.5")

    # Stepping
    steps = int(round(t_end / dt))
    floor_top_y = floor_thick * 0.5
    min_pen = None

    print_dt = 0.10
    print_every = max(1, int(round(print_dt / dt)))
    if verbose:
        print(f"Stepping with dt={dt:g}s, t_end={t_end:g}s, steps={steps}, print_every={print_every} (~{print_dt}s)")
        print("t(s)\tsphere_y\tpenetration(SMC)")
        y0 = sph.GetPos().y
        if is_smc:
            pen0 = (y0 - r) - floor_top_y
            print(f"{0.00:4.2f}\t{y0:+.5f}\t{pen0:+.6f}", flush=True)
        else:
            print(f"{0.00:4.2f}\t{y0:+.5f}\t-", flush=True)

    print(f"Entering loop: steps={steps}, dt={dt}, t_end={t_end}", flush=True)

    for i in range(1, steps + 1):
        sys.DoStepDynamics(dt)
        if (i % print_every == 0) or (i == steps):
            t = sys.GetChTime()
            y = sph.GetPos().y
            if is_smc:
                pen = (y - r) - floor_top_y
                print(f"{t:4.2f}\t{y:+.5f}\t{pen:+.6f}", flush=True)
                if (min_pen is None) or (pen < min_pen):
                    min_pen = pen
            else:
                print(f"{t:4.2f}\t{y:+.5f}\t-", flush=True)

    print(f"Exited loop at t={sys.GetChTime():.3f}s (i={i})", flush=True)
    final_y = sph.GetPos().y
    if verbose:
        print(f"Finished {'ChSystemSMC' if is_smc else 'ChSystemNSC'}: final sphere y = {final_y:.6f}")
        if is_smc:
            print(f"Min penetration (negative=overlap): {min_pen:.6f} m")

    return {
        "system": "SMC" if is_smc else "NSC",
        "final_y": final_y,
        "min_penetration": min_pen if is_smc else None,
        "steps": steps,
    }


def main():
    """Run NSC then SMC, with verbose logging and a compact summary."""
    params = dict(dt=1e-3, t_end=1.0, friction=0.5, restitution=0.4, young=2.0e7, poisson=0.3, verbose=True)

    nsc = drop_once(system_kind="NSC", **params)
    smc = drop_once(system_kind="SMC", **params)

    print("\n=== Summary ===")
    short = lambda d: {k: (round(v, 6) if isinstance(v, float) else v) for k, v in d.items()}
    print("NSC:", short(nsc))
    print("SMC:", short(smc))
    if smc["min_penetration"] is not None:
        print("Note: SMC permits small compliant overlaps (negative penetration).")


if __name__ == "__main__":
    main()
