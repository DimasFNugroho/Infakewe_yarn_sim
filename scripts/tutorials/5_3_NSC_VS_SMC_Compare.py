"""
5_3_nsc_vs_smc_compare.py — Side-by-side NSC vs SMC comparison (same scene)
===========================================================================

What it does
------------
1) Runs TWO simulations headless with identical geometry & params:
   - System A: ChSystemNSC  (constraint-based, non-smooth)
   - System B: ChSystemSMC  (penalty-based, compliant)
   Each run logs time, position, and distance-along-slope s(t).

2) Writes two CSVs and one PNG with both s(t) curves overlaid.

3) Optional: opens ONE viewer that REPLAYS the recorded trajectories together
   (no physics during playback), so you can watch NSC (blue) vs SMC (orange)
   move simultaneously on the same ramp.

Why not one system with both?
-----------------------------
Chrono selects contact handling per system (NSC *or* SMC). You cannot mix them
for different bodies inside the same system. This script shows an equivalent
comparison: two runs → one synchronized playback.

Run
---
    conda activate chrono
    # install matplotlib if needed: conda install -y matplotlib
    python -u scripts/tutorials/5_3_nsc_vs_smc_compare.py --tilt g --theta 15 --mu 0.20 --tend 6 --dt_nsc 0.001 --dt_smc 0.0005 --view 1

Flags
-----
  --tilt g|geom    : 'g' = horizontal floor + tilted gravity (robust default)
                     'geom' = rotated thick ramp geometry (try if your build is stable)
  --theta          : incline angle (deg)
  --mu             : Coulomb friction (use < tan(theta) to see sliding)
  --E, --nu        : SMC contact stiffness via Young's modulus & Poisson's ratio
  --dt_nsc         : NSC time step (e.g., 1e-3)
  --dt_smc         : SMC time step (smaller, e.g., 5e-4)
  --tend           : end time (s)
  --view 0|1       : after sim & plots, open a single playback window
"""

import os, csv, math, argparse
import pychrono as chrono
import pychrono.irrlicht as irr

# --- plotting (headless backend) ---
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ===== helpers =====

def vdot(a: "chrono.ChVectorD", b: "chrono.ChVectorD") -> float:
    return a.x*b.x + a.y*b.y + a.z*b.z

def colorize(vshape, rgb):
    mat = chrono.ChVisualMaterial()
    mat.SetDiffuseColor(chrono.ChColor(*rgb))
    if hasattr(vshape, "GetMaterials"):
        vshape.GetMaterials().push_back(mat)
    else:
        vshape.material_list.push_back(mat)

def make_nsc_mat(mu: float, e: float = 0.05) -> "chrono.ChMaterialSurfaceNSC":
    m = chrono.ChMaterialSurfaceNSC()
    m.SetFriction(float(mu)); m.SetRestitution(float(e))
    return m

def make_smc_mat(mu: float, E: float, nu: float, e: float = 0.1) -> "chrono.ChMaterialSurfaceSMC":
    m = chrono.ChMaterialSurfaceSMC()
    m.SetFriction(float(mu)); m.SetRestitution(float(e))
    m.SetYoungModulus(float(E)); m.SetPoissonRatio(float(nu))
    return m

def make_box_body_manual(sys, half, pos, rot, density, fixed, contact_mat, color_rgb):
    hx, hy, hz = half.x, half.y, half.z
    vol = 8*hx*hy*hz
    mass = max(1e-8, density*vol)
    Ixx = (1/12)*mass*((2*hy)**2 + (2*hz)**2)
    Iyy = (1/12)*mass*((2*hx)**2 + (2*hz)**2)
    Izz = (1/12)*mass*((2*hx)**2 + (2*hy)**2)

    b = chrono.ChBody()
    b.SetMass(mass)
    b.SetInertiaXX(chrono.ChVectorD(Ixx,Iyy,Izz))
    b.SetPos(pos); b.SetRot(rot); b.SetBodyFixed(fixed)

    vs = chrono.ChBoxShape()
    vs.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    colorize(vs, color_rgb)
    b.AddVisualShape(vs)

    cm = b.GetCollisionModel()
    cm.ClearModel()
    cm.AddBox(contact_mat, hx, hy, hz, chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
    cm.BuildModel()
    b.SetCollide(True)

    sys.Add(b)
    return b

def make_system_nsc(gvec):
    sys = chrono.ChSystemNSC()
    try: sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except: pass
    sys.Set_G_acc(gvec)
    try:
        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.002)
    except: pass
    try: sys.SetMaxPenetrationRecoverySpeed(0.5)
    except: pass
    try: sys.SetMinBounceSpeed(0.1)
    except: pass
    return sys

def make_system_smc(gvec):
    sys = chrono.ChSystemSMC()
    try: sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except: pass
    sys.Set_G_acc(gvec)
    try:
        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.002)
    except: pass
    try: sys.SetMaxPenetrationRecoverySpeed(0.5)
    except: pass
    try: sys.SetMinBounceSpeed(0.05)
    except: pass
    return sys

# ===== scene builders (shared geometry, two tilt modes) =====

def build_scene_geom(sys, mu, is_smc, theta_deg):
    """Rotated thick ramp, rotated box upslope."""
    th = math.radians(theta_deg)
    q = chrono.Q_from_AngAxis(th, chrono.ChVectorD(0,0,1))
    R = chrono.ChMatrix33D(q)

    mat = make_smc_mat(mu, 2e7, 0.3, 0.1) if is_smc else make_nsc_mat(mu, 0.05)

    # Thick ramp (avoid thin-rotated issues)
    half_ramp = chrono.ChVectorD(1.25, 0.25, 0.6)
    top_center_off = R * chrono.ChVectorD(0, +half_ramp.y, 0)
    ramp_pos = -(top_center_off)
    ramp = make_box_body_manual(sys, half_ramp, ramp_pos, q, density=800.0, fixed=True,
                                contact_mat=mat, color_rgb=(0.6,0.6,0.6))

    slope_dir = R * chrono.ChVectorD(1,0,0); slope_dir = slope_dir / slope_dir.Length()
    n_world   = R * chrono.ChVectorD(0,1,0)
    top_center= ramp.GetPos() + top_center_off

    up = 0.60
    start = top_center + slope_dir * (-up)

    half_box = chrono.ChVectorD(0.08,0.08,0.08)
    box_pos  = start + n_world * (half_box.y + 1e-3)
    box = make_box_body_manual(sys, half_box, box_pos, q, density=600.0, fixed=False,
                               contact_mat=mat, color_rgb=(0.20,0.60,0.90) if not is_smc else (0.95,0.55,0.20))
    # plane descriptor for penetration estimate
    plane_pt, plane_n, half_box_y = top_center, n_world, half_box.y
    return box, slope_dir, start, plane_pt, plane_n, half_box_y, q

def build_scene_tiltg(sys, mu, is_smc, theta_deg):
    """Horizontal floor + tilted gravity, axis-aligned geometry."""
    mat = make_smc_mat(mu, 2e7, 0.3, 0.1) if is_smc else make_nsc_mat(mu, 0.05)

    half_floor = chrono.ChVectorD(1.25, 0.025, 0.6)
    floor = make_box_body_manual(sys, half_floor, chrono.ChVectorD(0,-half_floor.y,0), chrono.QUNIT,
                                 density=800.0, fixed=True, contact_mat=mat, color_rgb=(0.6,0.6,0.6))
    slope_dir = chrono.ChVectorD(1,0,0)
    half_box  = chrono.ChVectorD(0.08,0.08,0.08)
    start     = chrono.ChVectorD(-0.70, 0.0, 0.0)
    box_pos   = chrono.ChVectorD(start.x, half_box.y + 1e-3, 0.0)
    box = make_box_body_manual(sys, half_box, box_pos, chrono.QUNIT, density=600.0, fixed=False,
                               contact_mat=mat, color_rgb=(0.20,0.60,0.90) if not is_smc else (0.95,0.55,0.20))
    plane_pt, plane_n, half_box_y, q = chrono.ChVectorD(0,0,0), chrono.ChVectorD(0,1,0), half_box.y, chrono.QUNIT
    return box, slope_dir, start, plane_pt, plane_n, half_box_y, q

# ===== one run utility =====

def run_once(model: str, tilt: str, theta: float, mu: float, dt: float, tend: float):
    """
    model: 'NSC' | 'SMC'
    tilt : 'g' | 'geom'
    Returns dict with arrays and metadata.
    """
    # gravity
    if tilt == "g":
        g = 9.81; th = math.radians(theta)
        gvec = chrono.ChVectorD(-g*math.sin(th), -g*math.cos(th), 0.0)
    else:
        gvec = chrono.ChVectorD(0, -9.81, 0)

    sys = make_system_smc(gvec) if model == "SMC" else make_system_nsc(gvec)

    if tilt == "g":
        box, sdir, start, plane_pt, plane_n, half_box_y, qbox = build_scene_tiltg(sys, mu, model=="SMC", theta)
    else:
        box, sdir, start, plane_pt, plane_n, half_box_y, qbox = build_scene_geom(sys, mu, model=="SMC", theta)

    # integrate
    ts, xs, ys, zs, ss, pens = [], [], [], [], [], []
    steps = max(1, int(round(tend / dt)))
    nxt = 0.0
    for _ in range(steps):
        sys.DoStepDynamics(dt)
        t = sys.GetChTime()
        p = box.GetPos()
        s = vdot(p - start, sdir)
        # signed distance from bottom of box to plane
        pen = vdot((p - plane_n*half_box_y) - plane_pt, plane_n)  # <0 ≈ overlap (SMC)
        ts.append(t); xs.append(p.x); ys.append(p.y); zs.append(p.z); ss.append(s); pens.append(pen)
        if t >= nxt:
            print(f"[{model}] t={t:4.2f}s  s={s:+.3f}  y={p.y:+.3f}  pen≈{pen:+.5f}")
            nxt += 0.2

    return {
        "model": model,
        "tilt": tilt,
        "theta": theta,
        "mu": mu,
        "dt": dt,
        "tend": tend,
        "qbox": qbox,                # for playback orientation
        "sdir": sdir, "start": start,
        "plane_pt": plane_pt, "plane_n": plane_n, "half_box_y": half_box_y,
        "t": ts, "x": xs, "y": ys, "z": zs, "s": ss, "pen": pens
    }

# ===== plotting & csv =====

def save_csv(path, data):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t","x","y","z","s_along_slope","penetration_est"])
        for row in zip(data["t"], data["x"], data["y"], data["z"], data["s"], data["pen"]):
            w.writerow([f"{row[0]:.6f}", f"{row[1]:.6f}", f"{row[2]:.6f}", f"{row[3]:.6f}", f"{row[4]:.6f}", f"{row[5]:.6f}"])

def save_plot(path, nsc, smc, theta, mu):
    fig, (ax1, ax2) = plt.subplots(2,1, figsize=(7,6), dpi=120)
    ax1.plot(nsc["t"], nsc["s"], label="NSC")
    ax1.plot(smc["t"], smc["s"], label="SMC")
    ax1.set_title(f"Distance along slope s(t) — θ={theta}°, μ={mu}")
    ax1.set_xlabel("time [s]"); ax1.set_ylabel("s [m]"); ax1.grid(True); ax1.legend()
    ax2.plot(nsc["t"], nsc["y"], label="NSC")
    ax2.plot(smc["t"], smc["y"], label="SMC")
    ax2.set_title("Y position vs time")
    ax2.set_xlabel("time [s]"); ax2.set_ylabel("y [m]"); ax2.grid(True); ax2.legend()
    fig.tight_layout()
    fig.savefig(path)

# ===== playback viewer =====

def playback_two_tracks(nsc, smc, tilt, theta):
    """
    Draw ramp (visual only, no collisions), then move two colored boxes
    using recorded samples (no physics during playback).
    """
    # create a lightweight system (no gravity, no collisions needed)
    sys = chrono.ChSystemNSC()
    sys.Set_G_acc(chrono.ChVectorD(0,0,0))

    # build visual ramp (collide False)
    if tilt == "g":
        # horizontal floor
        half_floor = chrono.ChVectorD(1.25, 0.025, 0.6)
        ramp = chrono.ChBody()
        ramp.SetBodyFixed(True); ramp.SetCollide(False)
        vs = chrono.ChBoxShape(); vs.GetBoxGeometry().Size = half_floor
        colorize(vs,(0.65,0.65,0.65))
        ramp.AddVisualShape(vs, chrono.ChFrameD(chrono.ChVectorD(0,-half_floor.y,0)))
        sys.Add(ramp)
        qbox = chrono.QUNIT
    else:
        th = math.radians(theta)
        q = chrono.Q_from_AngAxis(th, chrono.ChVectorD(0,0,1))
        R = chrono.ChMatrix33D(q)
        half_ramp = chrono.ChVectorD(1.25, 0.25, 0.6)
        top_center_off = R * chrono.ChVectorD(0, +half_ramp.y, 0)
        ramp = chrono.ChBody()
        ramp.SetBodyFixed(True); ramp.SetCollide(False)
        vs = chrono.ChBoxShape(); vs.GetBoxGeometry().Size = half_ramp
        colorize(vs,(0.65,0.65,0.65))
        ramp.AddVisualShape(vs)
        ramp.SetRot(q)
        ramp.SetPos(-(top_center_off))
        sys.Add(ramp)
        qbox = q

    # two kinematic boxes (we'll set their pose each frame)
    half_box = chrono.ChVectorD(0.08,0.08,0.08)

    def make_ghost(color):
        b = chrono.ChBody()
        b.SetCollide(False); b.SetBodyFixed(False)
        vsb = chrono.ChBoxShape(); vsb.GetBoxGeometry().Size = half_box
        colorize(vsb, color)
        b.AddVisualShape(vsb)
        sys.Add(b)
        return b

    ghost_nsc = make_ghost((0.20,0.60,0.90))
    ghost_smc = make_ghost((0.95,0.55,0.20))

    # viewer
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 760)
    vis.SetWindowTitle("Playback — NSC (blue) vs SMC (orange) — close to exit")
    vis.Initialize(); vis.AddSkyBox(); vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVectorD(2.4,1.2,2.4), chrono.ChVectorD(0,0,0))
    try: vis.BindAll()
    except: pass

    # playback loop (60 FPS)
    t_nsc, t_smc = nsc["t"], smc["t"]
    p_nsc = list(zip(nsc["x"], nsc["y"], nsc["z"]))
    p_smc = list(zip(smc["x"], smc["y"], smc["z"]))
    t_end = max(t_nsc[-1], t_smc[-1])

    fps = 60.0; dt = 1.0/fps
    t_play = 0.0
    i = j = 0

    def pose_at(ts, ps, t, k):
        # advance index k while next time <= t
        while k+1 < len(ts) and ts[k+1] <= t:
            k += 1
        return k, chrono.ChVectorD(*ps[k])

    while vis.Run():
        # update indices/poses
        i, pos_i = pose_at(t_nsc, p_nsc, t_play, i)
        j, pos_j = pose_at(t_smc, p_smc, t_play, j)
        ghost_nsc.SetPos(pos_i); ghost_nsc.SetRot(qbox)
        ghost_smc.SetPos(pos_j); ghost_smc.SetRot(qbox)

        vis.BeginScene(); vis.Render(); vis.EndScene()
        # no physics stepping needed during playback
        if t_play < t_end:
            t_play += dt

# ===== main =====

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tilt", choices=["g","geom"], default="g")
    ap.add_argument("--theta", type=float, default=15.0)
    ap.add_argument("--mu",    type=float, default=0.20)
    ap.add_argument("--E",     type=float, default=2.0e7)   # SMC stiffness
    ap.add_argument("--nu",    type=float, default=0.3)
    ap.add_argument("--dt_nsc",type=float, default=1e-3)
    ap.add_argument("--dt_smc",type=float, default=5e-4)
    ap.add_argument("--tend",  type=float, default=6.0)
    ap.add_argument("--view",  type=int,   default=1)
    args = ap.parse_args()

    # 1) NSC run
    print("=== Running NSC ===")
    nsc = run_once("NSC", args.tilt, args.theta, args.mu, args.dt_nsc, args.tend)

    # 2) SMC run (use requested E, nu)
    # Override materials in scene builders by passing is_smc True; E/nu are set in make_smc_mat.
    # We reuse the same scene geometry & params.
    # For SMC stiffness, we temporarily swap the default in builders by monkey-patching if needed.
    # Here we just rely on make_smc_mat default E/nu (which we set via args).
    # To feed args.E/nu: a tiny tweak—set module globals used in builders if you wish.
    # Simpler: temporarily wrap build functions? Instead, we rely on E/nu in make_smc_mat (already uses 2e7/0.3).
    # We'll pass them through by redefining make_smc_mat on-the-fly:

    global make_smc_mat
    def make_smc_mat(mu: float, E: float=args.E, nu: float=args.nu, e: float = 0.1):
        m = chrono.ChMaterialSurfaceSMC()
        m.SetFriction(float(mu)); m.SetRestitution(float(e))
        m.SetYoungModulus(float(E)); m.SetPoissonRatio(float(nu))
        return m

    print("\n=== Running SMC ===")
    smc = run_once("SMC", args.tilt, args.theta, args.mu, args.dt_smc, args.tend)

    # 3) Save CSVs + Plot
    os.makedirs("outputs", exist_ok=True)
    tag = args.tilt
    csv_n = os.path.join("outputs", f"nsc_vs_smc_{tag}_NSC.csv")
    csv_s = os.path.join("outputs", f"nsc_vs_smc_{tag}_SMC.csv")
    png   = os.path.join("outputs", f"nsc_vs_smc_{tag}.png")
    save_csv(csv_n, nsc); save_csv(csv_s, smc); save_plot(png, nsc, smc, args.theta, args.mu)
    print(f"[log] {csv_n}")
    print(f"[log] {csv_s}")
    print(f"[plot] {png}")

    # 4) Optional single-window playback (no physics)
    if args.view:
        playback_two_tracks(nsc, smc, args.tilt, args.theta)

if __name__ == "__main__":
    main()
