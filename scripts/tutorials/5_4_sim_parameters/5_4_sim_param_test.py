"""
sim_param_test.py (robust/manual-collision version)
===================================================

- Loads JSON5 config (comments allowed).
- NSC or SMC system.
- Plane: flat / rotated geometry ("geom") / gravity tilt ("gravity").
- Manual collision building (ChBody + AddBox + BuildModel) for robustness.
- Live Irrlicht window + CSV + PNG plot.

Run:
    conda activate chrono
    pip install json5 matplotlib
    python sim_param_test.py
"""

import os, csv, math
import json5 as json
import matplotlib.pyplot as plt
import pychrono as chrono
import pychrono.irrlicht as irr


# ----------------- config -----------------
def load_config(path="config.json"):
    with open(path, "r") as f:
        return json.load(f)


# ----------------- utilities -----------------
def colorize_box(body, half, rgb=(0.7, 0.7, 0.7)):
    """Add a box visual aligned to body frame."""
    bx = chrono.ChBoxShape()
    bx.GetBoxGeometry().Size = chrono.ChVectorD(*half)
    matv = chrono.ChVisualMaterial(); matv.SetDiffuseColor(chrono.ChColor(*rgb))
    # handle 8.x bindings (GetMaterials or material_list)
    if hasattr(bx, "GetMaterials"):
        bx.GetMaterials().push_back(matv)
    else:
        bx.material_list.push_back(matv)
    body.AddVisualShape(bx)  # at identity (body-aligned)


def make_material(sys_type: str, mcfg: dict):
    """Create NSC or SMC contact material and fill fields available in 8.0.0."""
    if sys_type.upper() == "NSC":
        m = chrono.ChMaterialSurfaceNSC()
        if hasattr(m, "SetFriction"):    m.SetFriction(float(mcfg["friction"]))
        if hasattr(m, "SetRestitution"): m.SetRestitution(float(mcfg["restitution"]))
        if "rolling_friction" in mcfg and hasattr(m, "SetRollingFriction"):
            m.SetRollingFriction(float(mcfg["rolling_friction"]))
        return m

    # SMC
    m = chrono.ChMaterialSurfaceSMC()
    if hasattr(m, "SetFriction"):     m.SetFriction(float(mcfg["friction"]))
    if hasattr(m, "SetRestitution"):  m.SetRestitution(float(mcfg["restitution"]))
    if hasattr(m, "SetYoungModulus"): m.SetYoungModulus(float(mcfg["young_modulus"]))
    if hasattr(m, "SetPoissonRatio"): m.SetPoissonRatio(float(mcfg["poisson_ratio"]))
    # 8.x SMC damping: use Gn/Gt
    if "normal_damping" in mcfg and hasattr(m, "SetGn"):
        m.SetGn(float(mcfg["normal_damping"]))
    if "tangential_damping" in mcfg and hasattr(m, "SetGt"):
        m.SetGt(float(mcfg["tangential_damping"]))
    if "rolling_friction" in mcfg and hasattr(m, "SetRollingFriction"):
        m.SetRollingFriction(float(mcfg["rolling_friction"]))
    return m


def make_system(sys_type: str, gravity_vec):
    sys = chrono.ChSystemNSC() if sys_type.upper() == "NSC" else chrono.ChSystemSMC()

    # Bullet, margins, recovery speeds — safer contacts
    try: sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except: pass
    try:
        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.002)
    except: pass
    try: sys.SetMaxPenetrationRecoverySpeed(0.5)
    except: pass
    try: sys.SetMinBounceSpeed(0.05)
    except: pass

    sys.Set_G_acc(chrono.ChVectorD(*gravity_vec))
    return sys


# ----------------- manual bodies -----------------
def add_box_body_manual(sys, half, density, pos, rot_q, fixed, contact_mat, color=(0.7,0.7,0.7)):
    """Create ChBody with manual collision box (robust)."""
    hx, hy, hz = half
    vol = 8.0 * hx * hy * hz
    mass = 0.0 if fixed else max(1e-8, density * vol)

    # inertia for solid box aligned with body axes
    Ixx = (1.0/12.0) * (mass) * ((2*hy)**2 + (2*hz)**2)
    Iyy = (1.0/12.0) * (mass) * ((2*hx)**2 + (2*hz)**2)
    Izz = (1.0/12.0) * (mass) * ((2*hx)**2 + (2*hy)**2)

    b = chrono.ChBody()
    b.SetBodyFixed(fixed)
    b.SetMass(mass)
    b.SetInertiaXX(chrono.ChVectorD(Ixx, Iyy, Izz))
    b.SetPos(chrono.ChVectorD(*pos))
    b.SetRot(rot_q)

    # visual
    colorize_box(b, half, color)

    # collision model (box aligned to body frame)
    cm = b.GetCollisionModel()
    cm.ClearModel()
    # AddBox(mat, hx, hy, hz, rel_pos, rel_R)
    cm.AddBox(contact_mat, hx, hy, hz, chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
    cm.BuildModel()
    b.SetCollide(True)

    sys.Add(b)
    return b


# ----------------- scene builders -----------------
def make_plane_flat(sys, half, density, fixed, mat):
    # top face at y=0 -> center at -hy
    hx, hy, hz = half
    pos = (0.0, -hy, 0.0)
    return add_box_body_manual(sys, half, density, pos, chrono.QUNIT, fixed, mat, color=(0.6,0.6,0.6))

def make_plane_inclined_geom(sys, half, density, fixed, mat, theta_deg):
    # rotate about Z, and place so the *local top face center* ends around world origin
    hx, hy, hz = half
    q = chrono.Q_from_AngAxis(math.radians(theta_deg), chrono.ChVectorD(0,0,1))
    R = chrono.ChMatrix33D(q)
    top_center_world = R * chrono.ChVectorD(0, +hy, 0)
    pos = (-top_center_world.x, -top_center_world.y, -top_center_world.z)
    return add_box_body_manual(sys, half, density, pos, q, fixed, mat, color=(0.6,0.6,0.6))

def make_box_faller(sys, half, density, pos, vel, mat):
    b = add_box_body_manual(sys, half, density, pos, chrono.QUNIT, False, mat, color=(0.2,0.6,0.9))
    b.SetPos_dt(chrono.ChVectorD(*vel))
    return b


# ----------------- main -----------------
def main():
    cfg = load_config("config.json")

    sys_type = cfg["system"]["type"]
    dt       = float(cfg["system"]["time_step"])
    t_end    = float(cfg["system"]["t_end"])
    base_g   = cfg["environment"]["gravity"]

    # SMC needs smaller dt for stiff contacts
    if sys_type.upper() == "SMC" and dt > 5e-4:
        print(f"[warn] SMC dt={dt:g} is large; clamping to 5e-4 for stability.")
        dt = 5e-4

    # Incline handling
    inc = cfg.get("incline", {"mode": "flat", "theta_deg": 0.0})
    mode = (inc.get("mode") or "flat").lower()
    theta_deg = float(inc.get("theta_deg", 0.0))

    if mode in ("gravity", "g"):
        # tilt gravity about +Z by theta
        gmag = (base_g[0]**2 + base_g[1]**2 + base_g[2]**2) ** 0.5
        th = math.radians(theta_deg)
        gvec = [-gmag * math.sin(th), -gmag * math.cos(th), 0.0]
    else:
        gvec = base_g

    sys = make_system(sys_type, gvec)

    mat = make_material(sys_type, cfg["material"])

    # plane
    half_plane = cfg["plane"]["size"]
    density_plane = cfg["plane"]["density"]
    fixed_plane   = bool(cfg["plane"]["fixed"])
    if mode in ("geom", "geometry"):
        # Use a *thick* plane for stability when rotated
        if half_plane[1] < 0.05:
            print("[info] increasing plane half-thickness for rotated geometry.")
            half_plane = [half_plane[0], 0.1, half_plane[2]]
        plane = make_plane_inclined_geom(sys, half_plane, density_plane, fixed_plane, mat, theta_deg)
    else:
        plane = make_plane_flat(sys, half_plane, density_plane, fixed_plane, mat)

    # box
    half_box = cfg["box"]["size"]
    density_box = cfg["box"]["density"]
    pos_box = cfg["box"]["initial_pos"]
    vel_box = cfg["box"]["initial_vel"]
    box = make_box_faller(sys, half_box, density_box, pos_box, vel_box, mat)

    # visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle(f"({sys_type}) incline={mode}, θ={theta_deg}° — close to exit")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVectorD(1.8, 0.9, 1.8), chrono.ChVectorD(0, 0, 0))
    try: vis.BindAll()
    except: pass

    # outputs
    csv_path = cfg["output"]["csv_file"]
    png_path = cfg["output"]["plot_file"]
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    os.makedirs(os.path.dirname(png_path), exist_ok=True)

    f = open(csv_path, "w", newline=""); writer = csv.writer(f)
    writer.writerow(["t", "x", "y", "z"])

    times, xs, ys, zs = [], [], [], []
    next_print = 0.0

    # main loop
    while vis.Run() and sys.GetChTime() < t_end:
        vis.BeginScene(); vis.Render(); vis.EndScene()

        # if something deep in Bullet/SMC misbehaves, smaller substeps can help:
        try:
            sys.DoStepDynamics(dt)
        except Exception as e:
            print(f"[error] DoStepDynamics raised: {e}")
            break

        t = sys.GetChTime()
        p = box.GetPos()
        writer.writerow([f"{t:.6f}", f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}"])
        times.append(t); xs.append(p.x); ys.append(p.y); zs.append(p.z)

        if t >= next_print:
            print(f"t={t:5.2f}s  pos=({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})")
            next_print += 0.2

    f.close(); print(f"[CSV] saved to {csv_path}")

    # plot
    plt.figure()
    plt.plot(times, xs, label="x"); plt.plot(times, ys, label="y"); plt.plot(times, zs, label="z")
    plt.xlabel("time [s]"); plt.ylabel("position [m]")
    plt.title(f"Trajectory ({sys_type}) — {mode}, θ={theta_deg}°"); plt.grid(True); plt.legend()
    plt.savefig(png_path, dpi=150); plt.close()
    print(f"[PNG] saved to {png_path}")

    # keep window open
    vis.SetWindowTitle(f"Finished ({sys_type}) — close window to exit")
    while vis.Run():
        vis.BeginScene(); vis.Render(); vis.EndScene()


if __name__ == "__main__":
    main()
