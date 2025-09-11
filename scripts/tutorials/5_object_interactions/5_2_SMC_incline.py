"""
5_1_incline_smc_manual.py — SMC incline with explicit materials & manual collisions
==================================================================================

Why this version
----------------
- Mirrors your NSC manual script, but uses ChMaterialSurfaceSMC.
- Manual collision build on ChBody (AddBox + BuildModel + SetCollide).
- Two tilt modes:
    --tilt geom  : rotated thick ramp + rotated box (tilt by geometry)
    --tilt g     : horizontal floor + tilted gravity (robust)
- CSV logging includes an estimated "penetration" (negative ≈ overlap) for intuition.

Run
---
  conda activate chrono
  # Geometry-tilt (thick ramp):
  python -u scripts/tutorials/5_1_incline_smc_manual.py --tilt geom --theta 15 --mu 0.20 --tend 6.0 --dt 0.0005
  # Gravity-tilt (robust fallback):
  python -u scripts/tutorials/5_1_incline_smc_manual.py --tilt g --theta 15 --mu 0.20 --tend 6.0 --dt 0.0005

Notes
-----
- SMC is compliant; use smaller dt (e.g., 5e-4) for stability.
- You can tune Young's modulus / Poisson via CLI if you want different stiffness.
"""

import os, csv, math, argparse
import pychrono as chrono
import pychrono.irrlicht as irr


# ---------- tiny helpers ----------

def colorize(vshape, rgb):
    mat = chrono.ChVisualMaterial()
    mat.SetDiffuseColor(chrono.ChColor(*rgb))
    if hasattr(vshape, "GetMaterials"):
        vshape.GetMaterials().push_back(mat)
    else:
        vshape.material_list.push_back(mat)

def vdot(a: "chrono.ChVectorD", b: "chrono.ChVectorD") -> float:
    return a.x * b.x + a.y * b.y + a.z * b.z


# ---------- explicit SMC material ----------

def make_smc_material(mu: float, young: float, poisson: float, e: float = 0.1) -> "chrono.ChMaterialSurfaceSMC":
    mat = chrono.ChMaterialSurfaceSMC()
    # Friction / restitution
    mat.SetFriction(float(mu))
    mat.SetRestitution(float(e))
    # Compliance (Hertzian derived stiffness uses these)
    mat.SetYoungModulus(float(young))
    mat.SetPoissonRatio(float(poisson))
    # Optional: slight adhesion (0), roll/twist friction (uncomment if desired)
    # mat.SetAdhesion(0.0)
    # mat.SetRollingFriction(0.0)
    # mat.SetSpinningFriction(0.0)
    return mat


# ---------- generic primitive body with manual collision ----------

def make_box_body_manual(
    sys: "chrono.ChSystemSMC",
    half: "chrono.ChVectorD",
    pos: "chrono.ChVectorD",
    rot: "chrono.ChQuaternionD",
    density: float,
    fixed: bool,
    contact_mat: "chrono.ChMaterialSurfaceSMC",
    color=(0.7, 0.7, 0.7),
) -> "chrono.ChBody":
    """
    Create a box body (visual + collision) explicitly:
      - ChBody()
      - visual: ChBoxShape
      - collision: AddBox(contact_mat, hx, hy, hz, loc, R); BuildModel(); SetCollide(True)
    """
    hx, hy, hz = half.x, half.y, half.z
    vol = 8.0 * hx * hy * hz
    mass = max(1e-8, density * vol)

    Ixx = (1.0/12.0) * mass * ((2*hy)**2 + (2*hz)**2)
    Iyy = (1.0/12.0) * mass * ((2*hx)**2 + (2*hz)**2)
    Izz = (1.0/12.0) * mass * ((2*hx)**2 + (2*hy)**2)

    b = chrono.ChBody()
    b.SetMass(mass)
    b.SetInertiaXX(chrono.ChVectorD(Ixx, Iyy, Izz))
    b.SetPos(pos)
    b.SetRot(rot)
    b.SetBodyFixed(fixed)

    vs = chrono.ChBoxShape()
    vs.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    colorize(vs, color)
    b.AddVisualShape(vs)

    cm = b.GetCollisionModel()
    cm.ClearModel()
    # Material is bound per-shape here:
    cm.AddBox(contact_mat, hx, hy, hz, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
    cm.BuildModel()
    b.SetCollide(True)

    sys.Add(b)
    return b


# ---------- system factory ----------
def make_system_smc(gravity_vec: "chrono.ChVectorD") -> "chrono.ChSystemSMC":
    sys = chrono.ChSystemSMC()
    # Bullet collision system (if compiled in)
    try:
        sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass

    # Suggested Bullet margins (safe if available)
    try:
        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.002)
    except Exception:
        pass

    sys.Set_G_acc(gravity_vec)

    # Compatibility-friendly stabilization knobs
    # (these exist in PyChrono 8.x; no SetContactRecoverySpeed in your build)
    try:
        sys.SetMaxPenetrationRecoverySpeed(0.5)  # helps recover interpenetration
    except Exception:
        pass
    try:
        sys.SetMinBounceSpeed(0.05)              # ignore tiny restitution jitters
    except Exception:
        pass

    return sys

# ---------- scenes ----------

def scene_rotated_ramp(sys, theta_deg: float, mu: float, young: float, poisson: float):
    """
    Rotated thick ramp + rotated box upslope.
    Returns: (box, slope_dir, start_point, plane_point, plane_normal, half_box_y)
    """
    theta = math.radians(theta_deg)
    q = chrono.Q_from_AngAxis(theta, chrono.ChVectorD(0, 0, 1))
    R = chrono.ChMatrix33D(q)

    mat = make_smc_material(mu, young, poisson, e=0.1)

    # Thick ramp to avoid thin-rotated issues
    half_ramp = chrono.ChVectorD(1.25, 0.25, 0.6)
    top_center_offset = R * chrono.ChVectorD(0, +half_ramp.y, 0)
    ramp_pos = -(top_center_offset)

    ramp = make_box_body_manual(
        sys, half_ramp, ramp_pos, q, density=800.0, fixed=True, contact_mat=mat, color=(0.6, 0.6, 0.6)
    )

    slope_dir = R * chrono.ChVectorD(1, 0, 0); slope_dir = slope_dir / slope_dir.Length()
    n_world = R * chrono.ChVectorD(0, 1, 0)
    plane_point = ramp.GetPos() + top_center_offset  # a point on ramp plane

    up_dist = 0.60
    start = plane_point + slope_dir * (-up_dist)

    half_box = chrono.ChVectorD(0.08, 0.08, 0.08)
    box_pos = start + n_world * (half_box.y + 1e-3)
    box = make_box_body_manual(
        sys, half_box, box_pos, q, density=600.0, fixed=False, contact_mat=mat, color=(0.2, 0.6, 0.9)
    )

    return box, slope_dir, start, plane_point, n_world, half_box.y


def scene_tilted_gravity(sys, theta_deg: float, mu: float, young: float, poisson: float):
    """
    Horizontal floor + tilted gravity + box upslope.
    Returns: (box, slope_dir, start_point, plane_point, plane_normal, half_box_y)
    """
    mat = make_smc_material(mu, young, poisson, e=0.1)

    # Horizontal floor: top face at y=0
    half_floor = chrono.ChVectorD(1.25, 0.025, 0.6)
    floor = make_box_body_manual(
        sys, half_floor, chrono.ChVectorD(0, -half_floor.y, 0), chrono.QUNIT,
        density=800.0, fixed=True, contact_mat=mat, color=(0.6, 0.6, 0.6)
    )
    plane_point = chrono.ChVectorD(0, 0, 0)  # point on plane (top face)
    plane_normal = chrono.ChVectorD(0, 1, 0)

    half_box = chrono.ChVectorD(0.08, 0.08, 0.08)
    start = chrono.ChVectorD(-0.70, 0.0, 0.0)
    box_pos = chrono.ChVectorD(start.x, half_box.y + 1e-3, 0.0)
    box = make_box_body_manual(
        sys, half_box, box_pos, chrono.QUNIT, density=600.0, fixed=False, contact_mat=mat, color=(0.2, 0.6, 0.9)
    )

    slope_dir = chrono.ChVectorD(1, 0, 0)
    return box, slope_dir, start, plane_point, plane_normal, half_box.y


# ---------- main ----------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tilt", choices=["geom", "g"], default="geom",
                    help="'geom' = rotated thick ramp; 'g' = horizontal floor with tilted gravity")
    ap.add_argument("--theta", type=float, default=15.0, help="incline angle in degrees")
    ap.add_argument("--mu",    type=float, default=0.20, help="Coulomb friction")
    ap.add_argument("--E",     type=float, default=2.0e7, help="Young's modulus [Pa]")
    ap.add_argument("--nu",    type=float, default=0.3, help="Poisson ratio [-]")
    ap.add_argument("--dt",    type=float, default=5e-4, help="time step [s] (SMC = small)")
    ap.add_argument("--tend",  type=float, default=6.0, help="end time [s]")
    ap.add_argument("--log",   type=int, default=1, help="write CSV log to outputs/")
    args = ap.parse_args()

    # Gravity
    if args.tilt == "g":
        g = 9.81; th = math.radians(args.theta)
        gvec = chrono.ChVectorD(-g * math.sin(th), -g * math.cos(th), 0.0)  # tilt about Z
    else:
        gvec = chrono.ChVectorD(0, -9.81, 0)

    sys = make_system_smc(gvec)

    # Scene
    if args.tilt == "g":
        box, sdir, start, plane_pt, plane_n, half_box_y = scene_tilted_gravity(sys, args.theta, args.mu, args.E, args.nu)
        title = f"SMC (tilted gravity) - theta={args.theta} deg, mu={args.mu}"
    else:
        box, sdir, start, plane_pt, plane_n, half_box_y = scene_rotated_ramp(sys, args.theta, args.mu, args.E, args.nu)
        title = f"SMC (rotated thick ramp) - theta={args.theta} deg, mu={args.mu}"

    # Viewer (same flow as your 4_2)
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 760)
    vis.SetWindowTitle(title + "  (close to exit)")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVectorD(2.4, 1.2, 2.4), chrono.ChVectorD(0, 0, 0))
    try:
        vis.BindAll()
    except Exception:
        pass

    # CSV logging
    writer = None
    if args.log:
        os.makedirs("outputs", exist_ok=True)
        tag = "geom" if args.tilt == "geom" else "gtilt"
        csv_path = os.path.join("outputs", f"smc_incline_manual_{tag}.csv")
        f = open(csv_path, "w", newline=""); writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "z", "s_along_slope", "penetration_est"])
        print(f"[log] {csv_path}")

    print(f"[SMC] mode={args.tilt}  theta={args.theta} deg, mu={args.mu}, E={args.E:.2e} Pa, nu={args.nu}, dt={args.dt}, t_end={args.tend}")

    dt = float(args.dt)
    next_print = 0.0
    while vis.Run() and sys.GetChTime() < args.tend:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        sys.DoStepDynamics(dt)

        t = sys.GetChTime()
        p = box.GetPos()
        s = vdot(p - start, sdir)

        # Penetration estimate:
        # signed distance from box bottom to plane (negative ≈ overlap)
        # plane: (X - plane_pt)·plane_n = 0
        # bottom point along normal ≈ p - plane_n * half_box_y (box aligned to plane in both modes)
        bottom_to_plane = vdot((p - plane_n * half_box_y) - plane_pt, plane_n)

        if writer:
            writer.writerow([f"{t:.6f}", f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}", f"{s:.6f}", f"{bottom_to_plane:.6f}"])

        if t >= next_print:
            print(f"t={t:5.2f}s  pos=({p.x:+.3f},{p.y:+.3f},{p.z:+.3f})  s={s:+.3f}  pen≈{bottom_to_plane:+.5f} m")
            next_print += 0.2

    if writer:
        f.close()
        print(f"[done] reached t={sys.GetChTime():.3f}s; CSV saved.")

    # Keep window open to inspect final state
    vis.SetWindowTitle(title + " — finished (close window to exit)")
    while vis.Run():
        vis.BeginScene(); vis.Render(); vis.EndScene()


if __name__ == "__main__":
    main()
