"""Single yarn example: clamped at one end, free at the other.

This example intentionally keeps one scenario only and drives everything from
one JSON config file.
"""

from __future__ import annotations

import argparse
import csv
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
        "solver_max_iterations": 150,
        "solver_tolerance": 1e-8,
    },
    "runtime": {
        "precheck_duration_s": 0.08,
        "physics_steps_per_render": 32,
        "print_interval_s": 0.5,
        "startup_delay_s": 0.0,
    },
    "yarn": {
        "length_m": 0.6,
        "element_count": 60,
        "start_support_mode": "clamped",
        "origin_xyz_m": [0.0, 0.9, 0.0],
        "axis_direction_xyz": [1.0, 0.0, 0.0],
        "up_direction_xyz": [0.0, 1.0, 0.0],
        "diameter_m": 2.29e-4,
        "young_modulus_pa": 1.118e9,
        "poisson_ratio": 0.2,
        "density_kg_m3": 800.0,
        "rayleigh_damping_s": 1e-7,
        "initial_sag_amplitude_m": 6e-4,
        "initial_tip_down_velocity_m_s": 0.0,
    },
    "visualization": {
        "window_size_px": [1280, 720],
        "window_title": "Yarn Free Fall (Clamped One End)",
        "camera_pos_xyz_m": [0.04, 0.86, 0.32],
        "camera_target_xyz_m": [0.08, 0.86, 0.0],
        "use_skybox": True,
        "use_typical_lights": True,
        "background_brightness_pct": 100.0,
        "show_overlay": True,
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

    if not bool(vis_cfg.get("show_overlay", True)):
        return

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


def build_clamped_yarn(system, cfg: dict):
    sim_c = cfg["simulation"]
    yarn_c = cfg["yarn"]

    length = float(yarn_c["length_m"])
    element_count = int(yarn_c["element_count"])
    dia_m = float(yarn_c["diameter_m"])
    radius_m = 0.5 * dia_m
    E = float(yarn_c["young_modulus_pa"])
    nu = float(yarn_c["poisson_ratio"])
    rho = float(yarn_c["density_kg_m3"])
    G = E / (2.0 * (1.0 + nu))

    axis = unit(tuple(float(v) for v in yarn_c["axis_direction_xyz"]))
    up = unit(tuple(float(v) for v in yarn_c["up_direction_xyz"]))
    ox, oy, oz = (float(v) for v in yarn_c["origin_xyz_m"])
    start = chrono.ChVectorD(ox, oy, oz)
    end = chrono.ChVectorD(ox + axis[0] * length, oy + axis[1] * length, oz + axis[2] * length)

    mesh = fea.ChMesh()
    if hasattr(mesh, "SetAutomaticGravity"):
        mesh.SetAutomaticGravity(True)
    system.Add(mesh)

    section = fea.ChBeamSectionEulerEasyCircular(dia_m, E, G, rho)
    if hasattr(section, "SetBeamRaleyghDamping"):
        section.SetBeamRaleyghDamping(float(yarn_c["rayleigh_damping_s"]))

    builder = fea.ChBuilderBeamEuler()
    builder.BuildBeam(mesh, section, element_count, start, end, chrono.ChVectorD(*up))
    nodes = list(builder.GetLastBeamNodes())

    # Clamp one end by default: fixed position and fixed rotation.
    nodes[0].SetFixed(True)
    support_mode = str(yarn_c.get("start_support_mode", "clamped")).strip().lower()
    if support_mode not in {"clamped", "pinned"}:
        raise ValueError("yarn.start_support_mode must be 'clamped' or 'pinned'")
    if support_mode == "pinned" and hasattr(nodes[0], "SetFixedD"):
        try:
            nodes[0].SetFixedD(False)
        except Exception:
            pass

    sag = float(yarn_c["initial_sag_amplitude_m"])
    tip_v = float(yarn_c["initial_tip_down_velocity_m_s"])
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
        "length_m": length,
        "element_count": element_count,
        "diameter_m": dia_m,
        "rho": rho,
        "area_m2": area,
        "E": E,
        "nu": nu,
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


def segment_centers_and_velocities(
    nodes: list,
) -> list[tuple[float, float, float, float, float, float]]:
    out: list[tuple[float, float, float, float, float, float]] = []
    for i in range(len(nodes) - 1):
        a = nodes[i]
        b = nodes[i + 1]
        pa = a.GetPos()
        pb = b.GetPos()
        va = a.GetPos_dt()
        vb = b.GetPos_dt()
        out.append(
            (
                0.5 * float(pa.x + pb.x),
                0.5 * float(pa.y + pb.y),
                0.5 * float(pa.z + pb.z),
                0.5 * float(va.x + vb.x),
                0.5 * float(va.y + vb.y),
                0.5 * float(va.z + vb.z),
            )
        )
    return out


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Yarn free-fall example (clamped one end)")
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="JSON config path")
    p.add_argument(
        "--headless-duration-s",
        type=float,
        default=0.0,
        help="If >0, run headless for this simulated duration and exit (no Irrlicht window).",
    )
    p.add_argument(
        "--trajectory-csv",
        type=Path,
        default=None,
        help="Optional CSV path for tip trajectory in headless mode.",
    )
    p.add_argument(
        "--segments-csv",
        type=Path,
        default=None,
        help="Optional CSV path for per-segment trajectory in headless mode.",
    )
    p.add_argument(
        "--sample-interval-s",
        type=float,
        default=0.01,
        help="Trajectory sampling interval in seconds for CSV outputs.",
    )
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

    mesh, nodes, meta = build_clamped_yarn(system, cfg)
    add_beam_visuals(mesh, vis_c)

    mu = meta["rho"] * meta["area_m2"]
    print("=== paper_yarn_clamped_solid.py ===")
    print(f"config={Path(args.config)}")
    print("model=single example, clamped one end + free-fall")
    print(f"dt={float(sim_c['dt_s'])} s  integration_substeps={int(sim_c['integration_substeps'])}")
    print(f"gravity=({gx}, {gy}, {gz}) m/s^2")
    print(
        f"length={meta['length_m']:.3f} m  elements={meta['element_count']}  "
        f"diameter={meta['diameter_m']*1e3:.4f} mm"
    )
    print(
        f"E={meta['E']:.3e} Pa  nu={meta['nu']:.3f}  rho={meta['rho']:.1f} kg/m^3  "
        f"linear_mass={mu:.6e} kg/m"
    )

    dt = float(sim_c["dt_s"])
    n_sub = int(sim_c["integration_substeps"])
    precheck_steps = max(1, int(float(rt_c["precheck_duration_s"]) / max(1e-12, dt)))
    for i in range(precheck_steps):
        step_with_substeps(system, dt, n_sub)
        if has_nan_nodes(nodes):
            print(f"PRECHECK FAILED: NaN at step {i}")
            return
    print(f"PRECHECK OK: t={system.GetChTime():.4f}s")

    if float(args.headless_duration_s) > 0.0:
        t_end = float(system.GetChTime()) + float(args.headless_duration_s)
        print(f"HEADLESS RUN: duration={float(args.headless_duration_s):.3f}s")
        y_start = float(nodes[-1].GetPos().y)
        y_min = y_start
        t_at_min = float(system.GetChTime())
        next_print = float(system.GetChTime())
        sample_dt = max(1e-9, float(args.sample_interval_s))
        next_sample = float(system.GetChTime())

        csv_file = None
        csv_writer = None
        if args.trajectory_csv is not None:
            cpath = Path(args.trajectory_csv)
            cpath.parent.mkdir(parents=True, exist_ok=True)
            csv_file = cpath.open("w", newline="", encoding="utf-8")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["t_s", "tip_y_m", "tip_vy_m_s"])

        seg_file = None
        seg_writer = None
        if args.segments_csv is not None:
            spath = Path(args.segments_csv)
            spath.parent.mkdir(parents=True, exist_ok=True)
            seg_file = spath.open("w", newline="", encoding="utf-8")
            seg_writer = csv.writer(seg_file)
            seg_writer.writerow(
                ["t_s", "segment_idx", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s"]
            )

        try:
            while float(system.GetChTime()) < t_end:
                step_with_substeps(system, dt, n_sub)
                if has_nan_nodes(nodes):
                    print(f"NaN detected at t={system.GetChTime():.6f}s, stopping.")
                    return
                t = float(system.GetChTime())
                y_tip, vy_tip = tip_stats(nodes)
                if y_tip < y_min:
                    y_min = y_tip
                    t_at_min = t
                if t + 1e-12 >= next_sample:
                    if csv_writer is not None:
                        csv_writer.writerow([f"{t:.9f}", f"{y_tip:.9f}", f"{vy_tip:.9f}"])
                    if seg_writer is not None:
                        segs = segment_centers_and_velocities(nodes)
                        for i, (x, y, z, vx, vy, vz) in enumerate(segs):
                            seg_writer.writerow(
                                [
                                    f"{t:.9f}",
                                    i,
                                    f"{x:.9f}",
                                    f"{y:.9f}",
                                    f"{z:.9f}",
                                    f"{vx:.9f}",
                                    f"{vy:.9f}",
                                    f"{vz:.9f}",
                                ]
                            )
                    next_sample += sample_dt
                if t >= next_print:
                    print(f"t={t:5.2f}s  free_end_y={y_tip:+.4f} m  free_end_vy={vy_tip:+.4f} m/s")
                    next_print += float(rt_c["print_interval_s"])
        finally:
            if csv_file is not None:
                csv_file.close()
            if seg_file is not None:
                seg_file.close()

        y_end = float(nodes[-1].GetPos().y)
        rebound = max(0.0, y_end - y_min)
        print(
            f"HEADLESS SUMMARY: tip_start={y_start:+.4f} m  tip_end={y_end:+.4f} m  "
            f"end_drop={y_start - y_end:+.4f} m  max_drop={y_start - y_min:+.4f} m  "
            f"rebound_from_min={rebound:+.4f} m  t_at_min={t_at_min:.3f}s"
        )
        return

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
