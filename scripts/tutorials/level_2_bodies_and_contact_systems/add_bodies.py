"""
bodies_02_add_bodies.py (robust, verbose, Bullet, ASCII)
========================================================

Goal:
    Add a floor + box + sphere + cylinder to a Chrono system and step the sim
    while printing time-series positions.

Stability tweaks for PyChrono 8.0.0 on Windows:
  - Force Bullet collision system (if available).
  - Avoid ChBodyEasy* helpers; build collision shapes manually and attach an
    explicit material (NSC or SMC).

What it prints:
  - System/collision system info.
  - Created bodies and start positions.
  - t=0 line, then about every 0.10 s, and the final line.
  - "Exited loop ..." marker to confirm full run.

Run:
    python -u scripts/tutorials/level_2_bodies_and_contact_systems/add_bodies.py
"""

import math
import pychrono as chrono


# ---------- materials ----------

def make_material(is_smc: bool, friction=0.5, restitution=0.4, young=2.0e7, poisson=0.3):
    """Create a contact material for NSC or SMC and set basic properties."""
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


# ---------- body builders (manual collision + simple visuals) ----------

def make_floor(sys, mat, size=(2.0, 0.05, 2.0), pos=(0.0, -0.025, 0.0)):
    """Fixed floor with a box collision shape and a simple visual box."""
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


def make_box(sys, mat, size=(0.1, 0.1, 0.1), pos=(-0.3, 0.30, 0.0), density=500.0):
    """Dynamic box with manual box collision and inertia."""
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


def make_sphere(sys, mat, radius=0.06, pos=(0.0, 0.40, 0.0), density=500.0):
    """Dynamic sphere with manual sphere collision and inertia."""
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


def make_cylinder_y(sys, mat, radius=0.05, height=0.12, pos=(0.3, 0.35, 0.0), density=500.0):
    """
    Dynamic cylinder with axis along Y, manual collision.

    Note:
      ChCollisionModel.AddCylinder takes (rx, rz, hy, [pos], [rot]).
      For a circular cylinder about Y: rx=radius, rz=radius, hy=half height.
    """
    body = chrono.ChBody()
    body.SetPos(chrono.ChVectorD(*pos))

    volume = math.pi * radius * radius * height
    mass = max(1e-6, density * volume)
    # inertia of solid cylinder about central axes:
    Ix = (1.0 / 12.0) * mass * (3 * radius * radius + height * height)
    Iy = 0.5 * mass * radius * radius
    Iz = Ix
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVectorD(Ix, Iy, Iz))

    vc = chrono.ChCylinderShape()
    vc.GetCylinderGeometry().p1 = chrono.ChVectorD(0, -height * 0.5, 0)
    vc.GetCylinderGeometry().p2 = chrono.ChVectorD(0, +height * 0.5, 0)
    vc.GetCylinderGeometry().rad = radius
    body.AddVisualShape(vc)

    body.SetCollide(True)
    cm = body.GetCollisionModel()
    cm.ClearModel()
    # rx, rz, hy (half height)
    cm.AddCylinder(mat, radius, radius, height * 0.5, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
    cm.BuildModel()

    sys.Add(body)
    return body


# ---------- sim helpers ----------

def make_system(kind: str = "NSC") -> "chrono.ChSystem":
    """Create NSC/SMC system, force Bullet collision (if available), set gravity."""
    sys = chrono.ChSystemNSC() if kind.upper() == "NSC" else chrono.ChSystemSMC()
    try:
        sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    return sys


def run(sys: "chrono.ChSystem", bodies: dict, dt=1e-3, t_end=0.5, print_dt=0.10):
    """Step the system and print y-positions at t=0, ~every print_dt, and at the end."""
    steps = int(round(t_end / dt))
    print_every = max(1, int(round(print_dt / dt)))

    try:
        cs_name = type(sys.GetCollisionSystem()).__name__
    except Exception:
        cs_name = "UnknownCollisionSystem"
    sys_name = "ChSystemNSC" if isinstance(sys, chrono.ChSystemNSC) else "ChSystemSMC"

    print(f"=== Running add_bodies in {sys_name} with {cs_name} ===")
    print("Gravity: (0, -9.81, 0) m/s^2")
    for k in ("box", "sphere", "cylinder"):
        p = bodies[k].GetPos()
        print(f"  {k:8s} start pos = ({p.x:.5f}, {p.y:.5f}, {p.z:.5f})")

    print(f"Stepping with dt={dt:g}s, t_end={t_end:g}s, steps={steps}, print_every={print_every} (~{print_dt}s)")
    print("t(s)\tbox_y\tsphere_y\tcyl_y")

    # t = 0 line
    pb = bodies["box"].GetPos().y
    ps = bodies["sphere"].GetPos().y
    pc = bodies["cylinder"].GetPos().y
    print(f"{0.00:4.2f}\t{pb:+.5f}\t{ps:+.5f}\t{pc:+.5f}", flush=True)

    print(f"Entering loop: steps={steps}, dt={dt}, t_end={t_end}", flush=True)
    for i in range(1, steps + 1):
        sys.DoStepDynamics(dt)
        if (i % print_every == 0) or (i == steps):
            t = sys.GetChTime()
            pb = bodies["box"].GetPos().y
            ps = bodies["sphere"].GetPos().y
            pc = bodies["cylinder"].GetPos().y
            print(f"{t:4.2f}\t{pb:+.5f}\t{ps:+.5f}\t{pc:+.5f}", flush=True)
    print(f"Exited loop at t={sys.GetChTime():.3f}s (i={i})")

    # Final recap
    for k in ("box", "sphere", "cylinder"):
        y = bodies[k].GetPos().y
        print(f"{k:8s} final y = {y:.6f}")


def main():
    """Build scene with manual collision shapes and run a short simulation."""
    use_smc = False  # flip to True to try SMC
    sys = make_system("SMC" if use_smc else "NSC")
    mat = make_material(is_smc=use_smc, friction=0.5, restitution=0.4, young=2.0e7, poisson=0.3)

    floor = make_floor(sys, mat, size=(2.0, 0.05, 2.0), pos=(0.0, -0.025, 0.0))
    box = make_box(sys, mat, size=(0.1, 0.1, 0.1), pos=(-0.3, 0.30, 0.0))
    sphere = make_sphere(sys, mat, radius=0.06, pos=(0.0, 0.40, 0.0))
    cylinder = make_cylinder_y(sys, mat, radius=0.05, height=0.12, pos=(0.3, 0.35, 0.0))

    bodies = dict(floor=floor, box=box, sphere=sphere, cylinder=cylinder)
    run(sys, bodies, dt=1e-3, t_end=1, print_dt=0.05)
    print("OK [done]")


if __name__ == "__main__":
    main()
