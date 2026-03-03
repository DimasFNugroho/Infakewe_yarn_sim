"""Paper-based yarn simulation: clamped one end, free at the other.

This script follows the paper parameterization (diameter from tex + material
table) but uses a beam-continuum FEM discretization for stable runtime in
PyChrono. It is a standalone model and does not reuse previous cable examples.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import pychrono as chrono
import pychrono.fea as fea

try:
    import pychrono.irrlicht as chronoirr
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "pychrono.irrlicht is required for visualization. Install a PyChrono build with Irrlicht."
    ) from exc


DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parent / "config" / "paper_yarn_clamped_solid.json"
)
DEFAULT_CFG = {
    "simulation": {
        "dt_s": 5.0e-4,
        "integration_substeps": 1,
        "gravity_m_s2": [0.0, -9.81, 0.0],
        "solver_max_iterations": 120,
        "solver_tolerance": 1e-8,
    },
    "runtime": {
        "precheck_duration_s": 0.08,
        "physics_steps_per_render": 8,
        "print_interval_s": 0.5,
        "startup_delay_s": 0.35,
    },
    "paper_model": {
        "yarn_count_tex": 32.8,
        "diameter_coefficient_k": 0.04,
        "material_profile": "18s_4",
        "poisson_ratio": 0.2,
        "rayleigh_damping_s": 1.0e-3,
    },
    "material_table": {
        "18s": {"young_modulus_pa": 1.438e9, "density_kg_m3": 800.0},
        "18s_2": {"young_modulus_pa": 1.29e9, "density_kg_m3": 800.0},
        "18s_4": {"young_modulus_pa": 1.118e9, "density_kg_m3": 800.0},
    },
    "yarn_geometry": {
        "length_m": 0.60,
        "elements_length": 60,
        "origin_xyz_m": [0.0, 0.9, 0.0],
        "axis_direction_xyz": [1.0, 0.0, 0.0],
        "up_direction_xyz": [0.0, 1.0, 0.0],
    },
    "initial_state": {
        "tip_down_velocity_m_s": 0.0,
        "initial_sag_amplitude_m": 0.0012,
    },
    "visualization": {
        "window_size_px": [1280, 720],
        "window_title": "Paper Yarn (Beam FEM) - Clamped One End",
        "camera_pos_xyz_m": [0.04, 0.86, 0.32],
        "camera_target_xyz_m": [0.08, 0.86, 0.0],
        "use_skybox": True,
        "use_typical_lights": True,
        "background_brightness_pct": 100.0,
        "beam_resolution": 12,
        "beam_section_resolution": 8,
        "overlay_min": -0.02,
        "overlay_max": 0.04,
    },
}


def _deep_merge(base: dict, override: dict) -> dict:
    out = dict(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path: Path | None) -> dict:
    cfg = dict(DEFAULT_CFG)
    cpath = Path(path) if path else DEFAULT_CONFIG_PATH
    if cpath.exists():
        loaded = json.loads(cpath.read_text(encoding="utf-8"))
        if not isinstance(loaded, dict):
            raise ValueError(f"Config must be a JSON object: {cpath}")
        cfg = _deep_merge(cfg, loaded)
    else:
        print(f"Config not found, using defaults: {cpath}")
    return cfg


def diameter_from_tex_mm(tex: float, k: float) -> float:
    """Equation from paper: D(mm) = k * sqrt(T_tex)."""
    if tex <= 0.0 or k <= 0.0:
        raise ValueError("tex and k must be > 0")
    return float(k) * math.sqrt(float(tex))


def unit(v: tuple[float, float, float]) -> tuple[float, float, float]:
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n < 1e-12:
        raise ValueError("direction vector must be non-zero")
    return (v[0] / n, v[1] / n, v[2] / n)


def step_with_substeps(system, dt: float, n_substeps: int) -> None:
    n = max(1, int(n_substeps))
    dts = float(dt) / float(n)
    for _ in range(n):
        system.DoStepDynamics(dts)


def add_beam_visuals(mesh, vis_cfg: dict) -> None:
    base = chrono.ChVisualShapeFEA(mesh)
    base.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    if hasattr(base, "SetBeamResolution"):
        base.SetBeamResolution(int(vis_cfg["beam_resolution"]))
    if hasattr(base, "SetBeamResolutionSection"):
        base.SetBeamResolutionSection(int(vis_cfg["beam_section_resolution"]))
    base.SetWireframe(False)
    mesh.AddVisualShapeFEA(base)

    overlay = chrono.ChVisualShapeFEA(mesh)
    overlay.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ANCF_BEAM_AX)
    if hasattr(overlay, "SetBeamResolution"):
        overlay.SetBeamResolution(int(vis_cfg["beam_resolution"]))
    if hasattr(overlay, "SetBeamResolutionSection"):
        overlay.SetBeamResolutionSection(int(vis_cfg["beam_section_resolution"]))
    if hasattr(overlay, "SetColorscaleMinMax"):
        overlay.SetColorscaleMinMax(float(vis_cfg["overlay_min"]), float(vis_cfg["overlay_max"]))
    overlay.SetWireframe(False)
    mesh.AddVisualShapeFEA(overlay)


def build_paper_beam_yarn(system, cfg: dict):
    sim_c = cfg["simulation"]
    pm = cfg["paper_model"]
    geo = cfg["yarn_geometry"]
    ini = cfg["initial_state"]
    mat_table = cfg["material_table"]

    profile = str(pm["material_profile"])
    if profile not in mat_table:
        raise KeyError(f"material_profile '{profile}' missing in material_table")
    E = float(mat_table[profile]["young_modulus_pa"])
    rho = float(mat_table[profile]["density_kg_m3"])
    nu = float(pm["poisson_ratio"])
    G = E / (2.0 * (1.0 + nu))

    tex = float(pm["yarn_count_tex"])
    k = float(pm["diameter_coefficient_k"])
    dia_mm = diameter_from_tex_mm(tex, k)
    dia_m = dia_mm * 1.0e-3
    radius_m = 0.5 * dia_m

    length = float(geo["length_m"])
    n_elems = int(geo["elements_length"])
    axis = unit(tuple(float(v) for v in geo["axis_direction_xyz"]))
    up = unit(tuple(float(v) for v in geo["up_direction_xyz"]))
    ox, oy, oz = (float(v) for v in geo["origin_xyz_m"])
    start = chrono.ChVectorD(ox, oy, oz)
    end = chrono.ChVectorD(
        ox + axis[0] * length,
        oy + axis[1] * length,
        oz + axis[2] * length,
    )

    mesh = fea.ChMesh()
    if hasattr(mesh, "SetAutomaticGravity"):
        mesh.SetAutomaticGravity(True)
    system.Add(mesh)

    section = fea.ChBeamSectionEulerEasyCircular(dia_m, E, G, rho)
    if hasattr(section, "SetBeamRaleyghDamping"):
        section.SetBeamRaleyghDamping(float(pm.get("rayleigh_damping_s", 1.0e-3)))

    builder = fea.ChBuilderBeamEuler()
    builder.BuildBeam(mesh, section, n_elems, start, end, chrono.ChVectorD(*up))
    nodes = list(builder.GetLastBeamNodes())

    # Clamp at one end: both translation and rotation fixed.
    nodes[0].SetFixed(True)

    # Tiny initial sag + velocity to break symmetry and show falling.
    sag = float(ini["initial_sag_amplitude_m"])
    tip_v = float(ini["tip_down_velocity_m_s"])
    n = len(nodes)
    for i, node in enumerate(nodes):
        if i == 0:
            continue
        w = i / max(1, n - 1)
        p = node.GetPos()
        node.SetPos(chrono.ChVectorD(p.x, p.y - sag * math.sin(0.5 * math.pi * w), p.z))
        if hasattr(node, "SetPos_dt"):
            node.SetPos_dt(chrono.ChVectorD(0.0, -tip_v * w, 0.0))

    solver = chrono.ChSolverMINRES()
    if hasattr(solver, "SetMaxIterations"):
        solver.SetMaxIterations(int(sim_c["solver_max_iterations"]))
    if hasattr(solver, "SetTolerance"):
        solver.SetTolerance(float(sim_c["solver_tolerance"]))
    if hasattr(system, "SetSolver"):
        system.SetSolver(solver)

    area = math.pi * radius_m * radius_m
    return mesh, nodes, {
        "tex": tex,
        "k": k,
        "diameter_mm": dia_mm,
        "radius_m": radius_m,
        "area_m2": area,
        "E": E,
        "nu": nu,
        "rho": rho,
        "length_m": length,
        "n_elems": n_elems,
    }


def has_nan_nodes(nodes: list) -> bool:
    for n in nodes:
        p = n.GetPos()
        if (p.x != p.x) or (p.y != p.y) or (p.z != p.z):
            return True
    return False


def tip_stats(nodes: list) -> tuple[float, float]:
    tip = nodes[-1]
    return float(tip.GetPos().y), float(tip.GetPos_dt().y)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Paper-based clamped yarn simulation")
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="JSON config path")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config(args.config)
    sim_c = cfg["simulation"]
    rt_c = cfg["runtime"]
    vis_c = cfg["visualization"]

    system = chrono.ChSystemNSC()
    gx, gy, gz = [float(v) for v in sim_c["gravity_m_s2"]]
    system.Set_G_acc(chrono.ChVectorD(gx, gy, gz))

    mesh, nodes, meta = build_paper_beam_yarn(system, cfg)
    add_beam_visuals(mesh, vis_c)

    mu_from_tex = meta["tex"] * 1.0e-6
    mu_from_geom = meta["rho"] * meta["area_m2"]
    err = 100.0 * (mu_from_geom - mu_from_tex) / max(1e-16, mu_from_tex)
    print("=== paper_yarn_clamped_solid.py ===")
    print(f"config={Path(args.config)}")
    print("model=paper-parameterized beam FEM yarn (non-cable)")
    print(f"dt={float(sim_c['dt_s'])} s  integration_substeps={int(sim_c['integration_substeps'])}")
    print(f"gravity=({gx}, {gy}, {gz}) m/s^2")
    print(
        f"tex={meta['tex']}  k={meta['k']}  diameter={meta['diameter_mm']:.4f} mm "
        f"(D=k*sqrt(T_tex))"
    )
    print(
        f"E={meta['E']:.3e} Pa  nu={meta['nu']:.3f}  rho={meta['rho']:.1f} kg/m^3 "
        "(from paper table)"
    )
    print(
        f"mu_tex={mu_from_tex:.6e} kg/m  mu_geom={mu_from_geom:.6e} kg/m  error={err:+.3f}%"
    )
    print(f"mesh: beam_elements={meta['n_elems']} nodes={len(nodes)}")

    dt = float(sim_c["dt_s"])
    n_sub = int(sim_c["integration_substeps"])
    precheck_steps = max(1, int(float(rt_c["precheck_duration_s"]) / max(1e-12, dt)))
    for i in range(precheck_steps):
        step_with_substeps(system, dt, n_sub)
        if has_nan_nodes(nodes):
            print(f"PRECHECK FAILED: NaN at step {i}")
            return
    print(f"PRECHECK OK: t={system.GetChTime():.4f}s")

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    w, h = [int(v) for v in vis_c["window_size_px"]]
    vis.SetWindowSize(w, h)
    vis.SetWindowTitle(str(vis_c["window_title"]))
    vis.Initialize()
    if bool(vis_c.get("use_skybox", False)):
        vis.AddSkyBox()
    if bool(vis_c.get("use_typical_lights", False)):
        vis.AddTypicalLights()
    else:
        b = max(0.0, min(100.0, float(vis_c.get("background_brightness_pct", 100.0)))) / 100.0
        amb = chrono.ChColor(0.18 * b, 0.18 * b, 0.18 * b)
        spc = chrono.ChColor(0.10 * b, 0.10 * b, 0.10 * b)
        dif = chrono.ChColor(0.98 * b, 0.98 * b, 0.98 * b)
        vis.AddLightDirectional(40.0, 30.0, amb, spc, dif)
        vis.AddLightDirectional(20.0, 200.0, amb, spc, chrono.ChColor(0.55 * b, 0.55 * b, 0.55 * b))
    vis.AddCamera(
        chrono.ChVectorD(*[float(v) for v in vis_c["camera_pos_xyz_m"]]),
        chrono.ChVectorD(*[float(v) for v in vis_c["camera_target_xyz_m"]]),
    )
    b = max(0.0, min(100.0, float(vis_c.get("background_brightness_pct", 100.0)))) / 100.0
    bg = chrono.ChColor(0.02 + 0.75 * b, 0.02 + 0.75 * b, 0.03 + 0.72 * b)

    startup_delay_s = max(0.0, float(rt_c.get("startup_delay_s", 0.0)))
    startup_frames = int(startup_delay_s / max(1e-12, dt))
    frame = 0
    next_print = system.GetChTime()

    while vis.Run():
        vis.BeginScene(True, True, bg)
        vis.Render()
        vis.EndScene()

        if frame < startup_frames:
            frame += 1
            continue

        for _ in range(int(rt_c["physics_steps_per_render"])):
            step_with_substeps(system, dt, n_sub)
            if has_nan_nodes(nodes):
                print(f"NaN detected at t={system.GetChTime():.6f}s, stopping.")
                return

        t = system.GetChTime()
        if t >= next_print:
            y_tip, vy_tip = tip_stats(nodes)
            print(f"t={t:5.2f}s  free_end_y={y_tip:+.4f} m  free_end_vy={vy_tip:+.4f} m/s")
            next_print += float(rt_c["print_interval_s"])


if __name__ == "__main__":
    main()
