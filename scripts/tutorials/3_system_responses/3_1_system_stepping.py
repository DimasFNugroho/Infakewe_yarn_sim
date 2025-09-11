"""
system_stepping_viz_and_graph.py (Chrono 8.x safe)
==================================================

Level 3.1 — System stepping

What this demonstrates
----------------------
- A clean `DoStepDynamics(dt)` loop in Chrono with user-selectable timestep and end time.
- Interactive visualization via Irrlicht (OpenGL) so you can *see* the motion.
- Regular console prints of key states (time, positions/angles).
- Logging to CSV and post-run plotting of trajectories (matplotlib).

Scenario
--------
- A fixed floor at y = 0 for contact.
- A falling sphere (you'll see bounce and settle).
- A simple pendulum (rod + revolute hinge) swinging under gravity.

Outputs
-------
- CSV:  outputs/level_3_1_log.csv
- PNGs: outputs/level_3_1_sphere_y.png, outputs/level_3_1_pendulum_angle.png

Usage
-----
    conda activate chrono
    # (optional) conda install matplotlib
    python -u scripts/tutorials/level_3_1_system_stepping/system_stepping_viz_and_graph.py --dt 0.001 --tend 3.0 --print-dt 0.1

Controls
--------
- Left-drag: rotate • Right-drag: pan • Wheel: zoom • Close window/ESC: quit

Notes
-----
- If matplotlib isn't installed, the script will still run and write the CSV; it will print
  the command to install matplotlib to generate plots.
"""

import os
import csv
import math
import argparse

import pychrono as chrono
import pychrono.irrlicht as chronoirr

# ---------- visuals & materials helpers (Chrono 8.x safe) ----------

def _colorize(vshape, rgb):
    """Attach a ChVisualMaterial with RGB color to a visual shape (bindings-safe)."""
    mat = chrono.ChVisualMaterial()
    mat.SetDiffuseColor(chrono.ChColor(*rgb))
    if hasattr(vshape, "GetMaterials"):
        vshape.GetMaterials().push_back(mat)
    elif hasattr(vshape, "material_list"):  # older exposure
        vshape.material_list.push_back(mat)


def _zero_min_bounce_if_supported(ch_material):
    """Some bindings expose a min bounce threshold—zero it so tiny bounces show."""
    for name in ("SetMinBounceSpeed", "SetMinBounceVelocity"):
        if hasattr(ch_material, name):
            getattr(ch_material, name)(0.0)


# ---------- system & bodies ----------

def make_system():
    sys = chrono.ChSystemNSC()
    try:
        sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    return sys


def make_floor(sys, size=(2.0, 0.05, 2.0), pos=(0.0, -0.025, 0.0), color=(0.55, 0.55, 0.55)):
    """Fixed floor: box visual + box collision. Floor top ends at y = 0."""
    sx, sy, sz = size
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5

    mat = chrono.ChMaterialSurfaceNSC()
    mat.SetFriction(0.5)
    mat.SetRestitution(0.2)
    _zero_min_bounce_if_supported(mat)

    body = chrono.ChBody()
    body.SetBodyFixed(True)
    body.SetPos(chrono.ChVectorD(*pos))
    body.SetMass(1.0)
    body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))

    vbox = chrono.ChBoxShape()
    vbox.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    _colorize(vbox, color)
    body.AddVisualShape(vbox)

    body.SetCollide(True)
    cm = body.GetCollisionModel()
    cm.ClearModel()
    cm.AddBox(mat, hx, hy, hz, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
    cm.BuildModel()

    sys.Add(body)
    return body


def make_sphere(sys, radius=0.10, pos=(0.0, 0.8, 0.0), color=(0.2, 0.9, 0.2)):
    """Dynamic sphere: visual + collision with proper mass/inertia."""
    density = 500.0
    volume = (4.0 / 3.0) * math.pi * radius**3
    mass = max(1e-6, density * volume)
    I = 0.4 * mass * radius * radius

    mat = chrono.ChMaterialSurfaceNSC()
    mat.SetFriction(0.3)
    mat.SetRestitution(0.6)  # somewhat bouncy so the y(t) plot is interesting
    _zero_min_bounce_if_supported(mat)

    b = chrono.ChBody()
    b.SetPos(chrono.ChVectorD(*pos))
    b.SetMass(mass)
    b.SetInertiaXX(chrono.ChVectorD(I, I, I))

    vs = chrono.ChSphereShape()
    vs.GetSphereGeometry().rad = radius
    _colorize(vs, color)
    b.AddVisualShape(vs)

    b.SetCollide(True)
    cm = b.GetCollisionModel()
    cm.ClearModel()
    cm.AddSphere(mat, radius, chrono.ChVectorD(0, 0, 0))
    cm.BuildModel()

    sys.Add(b)
    return b


def make_rod(sys, length=0.8, thickness=0.05, density=500.0, color=(0.2, 0.6, 0.9)):
    """Slender bar aligned with local +Y (length direction). Collisions off."""
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
    b.SetCollide(False)

    v = chrono.ChBoxShape()
    v.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    _colorize(v, color)
    b.AddVisualShape(v)

    sys.Add(b)
    return b


def place_pendulum(rod, pivot, length, theta_deg, axis=chrono.ChVectorD(0, 0, 1)):
    """Place rod so top end is at 'pivot' and it's rotated by theta about 'axis' from vertical-down."""
    theta = math.radians(theta_deg)
    q = chrono.Q_from_AngAxis(theta, axis)
    R = chrono.ChMatrix33D(q)
    top_local_from_com = chrono.ChVectorD(0, length * 0.5, 0)
    com_world = pivot - R * top_local_from_com
    rod.SetPos(com_world)
    rod.SetRot(q)


def planar_angle_about_z(rod, pivot):
    """Angle in XY-plane from pivot to COM: atan2(x, -y). 0 = straight down."""
    v = rod.GetPos() - pivot
    return math.atan2(v.x, -v.y)


# ---------- CSV & plotting ----------

def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def write_csv(path, rows):
    header = ["t", "sphere_y", "pendulum_angle_rad"]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def try_plot(outputs_dir, dt, show=False):
    try:
        import matplotlib.pyplot as plt
        import csv as _csv

        t, y, ang = [], [], []
        with open(os.path.join(outputs_dir, "level_3_1_log.csv"), "r") as f:
            r = _csv.DictReader(f)
            for row in r:
                t.append(float(row["t"]))
                y.append(float(row["sphere_y"]))
                ang.append(float(row["pendulum_angle_rad"]))

        # Sphere y(t)
        plt.figure()
        plt.plot(t, y)
        plt.xlabel("time [s]")
        plt.ylabel("sphere center y [m]")
        plt.title(f"Level 3.1 — sphere y(t) (dt={dt:g}s)")
        plt.grid(True, alpha=0.3)
        p1 = os.path.join(outputs_dir, "level_3_1_sphere_y.png")
        plt.savefig(p1, dpi=160, bbox_inches="tight")
        if show:
            # keep the windows open until you close them
            plt.show()
        else:
            # headless mode: close figures
            plt.close('all')

        # Pendulum angle(t)
        plt.figure()
        plt.plot(t, ang)
        plt.xlabel("time [s]")
        plt.ylabel("pendulum angle about Z [rad]")
        plt.title(f"Level 3.1 — pendulum angle(t) (dt={dt:g}s)")
        plt.grid(True, alpha=0.3)
        p2 = os.path.join(outputs_dir, "level_3_1_pendulum_angle.png")
        plt.savefig(p2, dpi=160, bbox_inches="tight")
        if show:
            # keep the windows open until you close them
            plt.show()
        else:
            # headless mode: close figures
            plt.close('all')

        print(f"[plot] saved:\n  {p1}\n  {p2}")

    except Exception as e:
        print(f"[plot] skipped (matplotlib not available). To enable plots:\n"
              f"  conda install matplotlib\nReason: {e}")


# ---------- main ----------

def main():
    ap = argparse.ArgumentParser(description="Level 3.1 — DoStepDynamics, dt choice, and trajectory logging")
    ap.add_argument("--dt", type=float, default=1e-3, help="integration timestep [s] (e.g., 0.001)")
    ap.add_argument("--tend", type=float, default=3.0, help="end time [s] (e.g., 3.0)")
    ap.add_argument("--print-dt", type=float, default=0.1, help="print interval [s] (e.g., 0.1)")
    args = ap.parse_args()

    dt = float(args.dt)
    t_end = float(args.tend)
    print_dt = float(args.print_dt)

    sys = make_system()

    # Scene: floor + falling sphere + pendulum (hinge at y=0)
    floor  = make_floor(sys, size=(2.0, 0.05, 2.0), pos=(0.0, -0.025, 0.0))
    sphere = make_sphere(sys, radius=0.10, pos=(+0.5, 0.80, 0.0), color=(0.2, 0.9, 0.2))

    L = 0.8
    pivot = chrono.ChVectorD(-0.6, 0.0, 0.0)
    rod = make_rod(sys, length=L, color=(0.2, 0.6, 0.9))
    place_pendulum(rod, pivot, L, theta_deg=35)

    hinge = chrono.ChLinkLockRevolute()
    hinge.Initialize(rod, floor, chrono.ChCoordsysD(pivot, chrono.QUNIT))
    sys.AddLink(hinge)

    # Visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1200, 720)
    vis.SetWindowTitle(f"Level 3.1 — System stepping (dt={dt:g}s, t_end={t_end:g}s)")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVectorD(3.0, 1.4, 3.2), chrono.ChVectorD(0.0, 0.1, 0.0))
    try:
        vis.BindAll()
    except Exception:
        pass

    # Logging & prints
    rows = []
    next_print_t = 0.0
    print("t(s)\tsphere_y\tpendulum_angle(rad)")
    print(f"{0.00:4.2f}\t{sphere.GetPos().y:+.5f}\t{planar_angle_about_z(rod, pivot):+.5f}")

    # Main stepping loop — stops when either the window closes or time reaches t_end
    while vis.Run() and sys.GetChTime() < t_end:
        # render + step one fixed dt
        vis.BeginScene(); vis.Render(); vis.EndScene()
        sys.DoStepDynamics(dt)

        # collect a sample each step (fine for plotting; CSV isn't large)
        t = sys.GetChTime()
        rows.append([t, sphere.GetPos().y, planar_angle_about_z(rod, pivot)])

        # periodic console print
        if t >= next_print_t:
            print(f"{t:4.2f}\t{sphere.GetPos().y:+.5f}\t{planar_angle_about_z(rod, pivot):+.5f}")
            next_print_t += print_dt

    # Save CSV & plots
    outdir = os.path.join("outputs")
    ensure_dir(outdir)
    csv_path = os.path.join(outdir, "level_3_1_log.csv")
    write_csv(csv_path, rows)
    print(f"[csv] wrote {csv_path}  ({len(rows)} rows)")

    try_plot(outdir, dt, show=True)
    print("Done.")


if __name__ == "__main__":
    main()
