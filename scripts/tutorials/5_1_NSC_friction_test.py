"""
5_1_incline_nsc_manual.py — NSC incline with explicit materials & manual collisions
==================================================================================

Why this version:
- Mirrors the structure you shared: explicit ChMaterialSurfaceNSC, manual collision
  building on ChBody (AddBox + BuildModel + SetCollide), simple viewer loop.
- Two modes:
    --tilt geom  -> rotated ramp (thick, to avoid thin-rotated issues)
    --tilt g     -> horizontal floor + tilted gravity (very robust)
- Logs CSV to outputs/.

Run:
  conda activate chrono
  # Rotated ramp (thick); try first:
  python -u scripts/tutorials/5_1_incline_nsc_manual.py --tilt geom --theta 15 --mu 0.20 --tend 6.0 --dt 0.001
  # If your build still crashes on rotated geometry, use the robust fallback:
  python -u scripts/tutorials/5_1_incline_nsc_manual.py --tilt g --theta 15 --mu 0.20 --tend 6.0 --dt 0.001
"""

import os, csv, math, argparse
import pychrono as chrono
import pychrono.irrlicht as irr


# ---------- helpers: visual color ----------
def colorize(vshape, rgb):
    mat = chrono.ChVisualMaterial()
    mat.SetDiffuseColor(chrono.ChColor(*rgb))
    # PyChrono 8.x bindings differ; handle both styles:
    if hasattr(vshape, "GetMaterials"):
        vshape.GetMaterials().push_back(mat)
    else:
        vshape.material_list.push_back(mat)


# ---------- explicit NSC material ----------
def make_nsc_material(mu: float, e: float = 0.05) -> "chrono.ChMaterialSurfaceNSC":
    mat = chrono.ChMaterialSurfaceNSC()
    mat.SetFriction(float(mu))
    mat.SetRestitution(float(e))
    return mat


# ---------- generic primitive body with manual collision ----------
def make_box_body_manual(
    sys: "chrono.ChSystemNSC",
    half: "chrono.ChVectorD",
    pos: "chrono.ChVectorD",
    rot: "chrono.ChQuaternionD",
    density: float,
    fixed: bool,
    contact_mat: "chrono.ChMaterialSurfaceNSC",
    color=(0.7, 0.7, 0.7),
) -> "chrono.ChBody":
    """
    Create a box body (visual + collision) explicitly, like in your example:
      - ChBody()
      - visual: ChBoxShape
      - collision: GetCollisionModel().AddBox(contact_mat, hx, hy, hz, loc, R); BuildModel(); SetCollide(True)
    """
    hx, hy, hz = half.x, half.y, half.z
    vol = 8.0 * hx * hy * hz
    mass = max(1e-8, density * vol)

    # principal inertias for a box aligned with its body frame
    Ixx = (1.0/12.0) * mass * ((2*hy)**2 + (2*hz)**2)
    Iyy = (1.0/12.0) * mass * ((2*hx)**2 + (2*hz)**2)
    Izz = (1.0/12.0) * mass * ((2*hx)**2 + (2*hy)**2)

    b = chrono.ChBody()
    b.SetMass(mass)
    b.SetInertiaXX(chrono.ChVectorD(Ixx, Iyy, Izz))
    b.SetPos(pos)
    b.SetRot(rot)
    b.SetBodyFixed(fixed)

    # visual
    vs = chrono.ChBoxShape()
    vs.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    colorize(vs, color)
    b.AddVisualShape(vs)  # visual frame = body frame

    # collision (box in body-local frame; orientation identity)
    cm = b.GetCollisionModel()
    cm.ClearModel()
    cm.AddBox(contact_mat, hx, hy, hz, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
    cm.BuildModel()
    b.SetCollide(True)

    sys.Add(b)
    return b


# ---------- systems ----------
def make_system_nsc(gravity_vec: "chrono.ChVectorD") -> "chrono.ChSystemNSC":
    sys = chrono.ChSystemNSC()
    # Bullet collision system
    try:
        sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass

    # Robustness suggestions
    try:
        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.002)
    except Exception:
        pass

    sys.Set_G_acc(gravity_vec)
    sys.SetMaxPenetrationRecoverySpeed(0.5)
    sys.SetMinBounceSpeed(0.1)
    return sys


# ---------- scenes ----------
def scene_rotated_ramp(sys, theta_deg: float, mu: float):
    """Rotated thick ramp + box placed upslope."""
    theta = math.radians(theta_deg)
    q = chrono.Q_from_AngAxis(theta, chrono.ChVectorD(0, 0, 1))
    R = chrono.ChMatrix33D(q)

    mat = make_nsc_material(mu, e=0.05)

    # Thick ramp (to avoid thin-rotated Bullet issues)
    half_ramp = chrono.ChVectorD(1.25, 0.25, 0.6)     # (sx, sy, sz)/2
    # Place so that the *top face center* is at world origin:
    top_center_offset = R * chrono.ChVectorD(0, +half_ramp.y, 0)
    ramp_pos = -(top_center_offset)

    ramp = make_box_body_manual(
        sys, half_ramp, ramp_pos, q, density=800.0, fixed=True, contact_mat=mat, color=(0.6, 0.6, 0.6)
    )

    # Slope direction (world): rotated +X
    slope_dir = R * chrono.ChVectorD(1, 0, 0)
    slope_dir = slope_dir / slope_dir.Length()
    n_world = R * chrono.ChVectorD(0, 1, 0)
    top_center = ramp.GetPos() + top_center_offset

    # Upslope start and box
    up_dist = 0.60
    start = top_center + slope_dir * (-up_dist)

    half_box = chrono.ChVectorD(0.08, 0.08, 0.08)
    box_pos = start + n_world * (half_box.y + 1e-3)  # tiny gap above plane
    box = make_box_body_manual(
        sys, half_box, box_pos, q, density=600.0, fixed=False, contact_mat=mat, color=(0.2, 0.6, 0.9)
    )

    return box, slope_dir, start


def scene_tilted_gravity(sys, theta_deg: float, mu: float):
    """Horizontal floor + tilted gravity (very robust) + box upslope."""
    mat = make_nsc_material(mu, e=0.05)

    # Horizontal floor at y=0 (top face)
    half_floor = chrono.ChVectorD(1.25, 0.025, 0.6)   # thin is OK when axis-aligned
    floor = make_box_body_manual(
        sys, half_floor, chrono.ChVectorD(0, -half_floor.y, 0), chrono.QUNIT,
        density=800.0, fixed=True, contact_mat=mat, color=(0.6, 0.6, 0.6)
    )

    # Upslope start and box (upslope = -X when gravity has +X component)
    half_box = chrono.ChVectorD(0.08, 0.08, 0.08)
    start = chrono.ChVectorD(-0.70, 0.0, 0.0)
    box_pos = chrono.ChVectorD(start.x, half_box.y + 1e-3, 0.0)
    box = make_box_body_manual(
        sys, half_box, box_pos, chrono.QUNIT, density=600.0, fixed=False, contact_mat=mat, color=(0.2, 0.6, 0.9)
    )

    slope_dir = chrono.ChVectorD(1, 0, 0)  # along +X
    return box, slope_dir, start

def vdot(a: "chrono.ChVectorD", b: "chrono.ChVectorD") -> float:
    """
    Compute the dot product between two Chrono vectors.

    Args:
        a: First vector (ChVectorD).
        b: Second vector (ChVectorD).

    Returns:
        Dot product (float).
    """
    return a.x * b.x + a.y * b.y + a.z * b.z

# ---------- main ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tilt", choices=["geom", "g"], default="geom",
                    help="'geom' = rotated ramp geometry (thick); 'g' = horizontal floor with tilted gravity")
    ap.add_argument("--theta", type=float, default=15.0, help="incline angle in degrees")
    ap.add_argument("--mu",    type=float, default=0.20, help="Coulomb friction")
    ap.add_argument("--dt",    type=float, default=1e-3, help="time step [s]")
    ap.add_argument("--tend",  type=float, default=6.0, help="end time [s]")
    ap.add_argument("--log",   type=int, default=1, help="write CSV log to outputs/")
    args = ap.parse_args()

    # gravity
    if args.tilt == "g":
        g = 9.81; th = math.radians(args.theta)
        gvec = chrono.ChVectorD(-g * math.sin(th), -g * math.cos(th), 0.0)  # tilt g about Z
    else:
        gvec = chrono.ChVectorD(0, -9.81, 0)

    sys = make_system_nsc(gvec)

    if args.tilt == "g":
        box, sdir, start = scene_tilted_gravity(sys, args.theta, args.mu)
        title = f"NSC (tilted gravity) — θ={args.theta}°, μ={args.mu}"
    else:
        box, sdir, start = scene_rotated_ramp(sys, args.theta, args.mu)
        title = f"NSC (rotated thick ramp) — θ={args.theta}°, μ={args.mu}"

    # Viewer — same pattern that works in your 4_2
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
        csv_path = os.path.join("outputs", f"nsc_incline_manual_{tag}.csv")
        f = open(csv_path, "w", newline=""); writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "z", "s_along_slope"])
        print(f"[log] {csv_path}")

    print(f"[NSC] mode={args.tilt}  theta={args.theta}°, mu={args.mu}, dt={args.dt}, t_end={args.tend}")

    # Sim loop (identical to 4_2’s style)
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

        if writer:
            writer.writerow([f"{t:.6f}", f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}", f"{s:.6f}"])

        if t >= next_print:
            print(f"t={t:5.2f}s  pos=({p.x:+.3f},{p.y:+.3f},{p.z:+.3f})  s={s:+.3f}")
            next_print += 0.2

    if writer:
        f.close()
        print(f"[done] reached t={sys.GetChTime():.3f}s; CSV saved.")

    # Keep window up for inspection
    vis.SetWindowTitle(title + " — finished (close window to exit)")
    while vis.Run():
        vis.BeginScene(); vis.Render(); vis.EndScene()


if __name__ == "__main__":
    main()
