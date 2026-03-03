"""Realtime-leaning variant of hanging yarn heatmap visualization.

Keeps the original high-detail example untouched while offering a faster profile
for interactive inspection on modest hardware.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
import math

import pychrono as chrono

try:
    import pychrono.irrlicht as chronoirr
except Exception as exc:  # pragma: no cover - runtime environment dependent
    raise SystemExit(
        "pychrono.irrlicht is required for visualization. "
        "Install a PyChrono build with Irrlicht support."
    ) from exc


REPO_ROOT = Path(__file__).resolve().parents[3]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from chrono_yarn.fea_yarn import (  # noqa: E402
    FEAHangingYarnConfig,
    FEASolverConfig,
    FEAVisualizationConfig,
    attach_fea_cable_visuals,
    attach_fea_strain_overlay,
    build_hanging_yarn_scene,
    cable_max_sag,
)


PRECHECK_DURATION_S = 0.08
PHYSICS_STEPS_PER_RENDER = 1
PRINT_INTERVAL_S = 0.5
NODE_DRAG_GAMMA_S_INV = 0.35

# Nomex 2670 dtex / Z90 preset.
NOMEX_LINEAR_DENSITY_DTEX = 2670.0
NOMEX_LINEAR_DENSITY_KG_M = NOMEX_LINEAR_DENSITY_DTEX * 1e-7
NOMEX_DENSITY_KG_M3 = 1380.0
NOMEX_PACKING_FACTOR = 0.62
NOMEX_BULK_DENSITY_KG_M3 = NOMEX_DENSITY_KG_M3 * NOMEX_PACKING_FACTOR
# Slightly softer effective modulus for realtime robustness.
NOMEX_EFFECTIVE_YOUNG_RT_PA = 8.0e6
NOMEX_EFFECTIVE_RAYLEIGH_RT_S = 2.4e-3

DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parent / "config" / "hanging_yarn_all.json"
)
DEFAULT_CONFIG_SECTION = "heatmap_realtime"
DEFAULT_CFG = {
    "simulation": {
        "dt_s": 1.5e-3,
        "integration_substeps": 1,
        "gravity_m_s2": [0.0, -9.81, 0.0],
        "solver_max_iterations": 60,
        "solver_tolerance": 1e-6,
    },
    "runtime": {
        "precheck_duration_s": PRECHECK_DURATION_S,
        "physics_steps_per_render": PHYSICS_STEPS_PER_RENDER,
        "print_interval_s": PRINT_INTERVAL_S,
        "startup_delay_s": 0.4,
    },
    "material": {
        "linear_density_dtex": NOMEX_LINEAR_DENSITY_DTEX,
        "solid_density_kg_m3": NOMEX_DENSITY_KG_M3,
        "packing_factor": NOMEX_PACKING_FACTOR,
        "effective_young_pa": NOMEX_EFFECTIVE_YOUNG_RT_PA,
        "effective_rayleigh_s": NOMEX_EFFECTIVE_RAYLEIGH_RT_S,
    },
    "yarn": {
        "length_m": 1.0,
        "element_count": 14,
        "start_xyz_m": [-0.50, 0.90, 0.0],
        "end_xyz_m": [0.50, 0.90, 0.0],
        "fix_start_node": True,
        "fix_end_node": False,
        "release_start_slope": True,
    },
    "damping": {
        "node_drag_gamma_s_inv": NODE_DRAG_GAMMA_S_INV,
    },
    "initial_state": {
        "sag_amplitude_m": 0.006,
        "initial_down_speed_m_s": 0.0,
    },
    "visualization": {
        "window_size_px": [1280, 720],
        "window_title": "Hanging Yarn Stretch Heatmap (Realtime)",
        "camera_pos_xyz_m": [-0.05, 0.95, 1.00],
        "camera_target_xyz_m": [0.0, 0.45, 0.0],
        "use_skybox": False,
        "use_typical_lights": False,
        "background_brightness_pct": 35.0,
        "beam_resolution": 4,
        "beam_section_resolution": 4,
        "wireframe": False,
        "draw_node_glyphs": False,
        "node_glyph_scale_m": 0.005,
        "node_glyph_thickness_m": 0.003,
        "overlay_min": -0.03,
        "overlay_max": 0.06,
    },
}


def has_nan_cable_nodes(cable) -> bool:
    for node in cable.nodes:
        p = node.GetPos()
        if any(v != v for v in (p.x, p.y, p.z)):
            return True
    return False


def apply_node_drag(cable, gamma_s_inv: float) -> None:
    """Apply mass-proportional linear drag: F = -gamma * m * v."""
    g = float(max(0.0, gamma_s_inv))
    if g <= 0.0:
        return
    for i, node in enumerate(cable.nodes):
        if i == 0:
            continue
        if hasattr(node, "IsFixed") and node.IsFixed():
            continue
        v = node.GetPos_dt() if hasattr(node, "GetPos_dt") else chrono.ChVectorD(0.0, 0.0, 0.0)
        mass = float(node.GetMass()) if hasattr(node, "GetMass") else 0.0
        if mass <= 0.0:
            continue
        f_drag = chrono.ChVectorD(-g * mass * v.x, -g * mass * v.y, -g * mass * v.z)
        if hasattr(node, "SetForce"):
            node.SetForce(f_drag)


def release_start_slope(cable) -> None:
    """Make the fixed endpoint pinned-like (position fixed, slope free)."""
    node0 = cable.first_node
    if node0 is None:
        return
    if hasattr(node0, "SetFixedD"):
        try:
            node0.SetFixedD(False)
        except Exception:
            pass
    if hasattr(node0, "IsFixedD") and node0.IsFixedD():
        raise RuntimeError("Failed to release slope DOF at start node (still fixedD=True).")


def cable_length(cable) -> float:
    length = 0.0
    for i in range(len(cable.nodes) - 1):
        p0 = cable.nodes[i].GetPos()
        p1 = cable.nodes[i + 1].GetPos()
        dx = p1.x - p0.x
        dy = p1.y - p0.y
        dz = p1.z - p0.z
        length += (dx * dx + dy * dy + dz * dz) ** 0.5
    return float(length)


def equivalent_radius_from_linear_density(mu_kg_m: float, bulk_density_kg_m3: float) -> float:
    """Estimate yarn radius from SI linear density and effective bulk density."""
    area = float(mu_kg_m) / float(bulk_density_kg_m3)
    return math.sqrt(area / math.pi)


def linear_density_from_radius_and_density(radius_m: float, density_kg_m3: float) -> float:
    """Return model-implied linear density in kg/m."""
    area_m2 = math.pi * float(radius_m) * float(radius_m)
    return float(density_kg_m3) * area_m2


def initialize_fall_state(
    cable,
    *,
    sag_amplitude_m: float = 0.006,
    initial_down_speed_m_s: float = 0.0,
) -> None:
    """Inject a physically plausible initial disturbance for visible falling."""
    n = len(cable.nodes)
    if n <= 1:
        return
    for i, node in enumerate(cable.nodes):
        if i == 0:
            continue
        w = i / float(n - 1)
        p = node.GetPos()
        dy = sag_amplitude_m * math.sin(0.5 * math.pi * w)
        node.SetPos(chrono.ChVectorD(p.x, p.y - dy, p.z))
        if hasattr(node, "SetPos_dt"):
            node.SetPos_dt(chrono.ChVectorD(0.0, initial_down_speed_m_s, 0.0))


def step_system_with_substeps(system, dt: float, substeps: int) -> None:
    n = max(1, int(substeps))
    dt_sub = float(dt) / float(n)
    for _ in range(n):
        system.DoStepDynamics(dt_sub)


def _deep_merge(base: dict, override: dict) -> dict:
    out = dict(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_runtime_config(config_path: Path | None, section: str) -> dict:
    cfg = dict(DEFAULT_CFG)
    path = Path(config_path) if config_path else DEFAULT_CONFIG_PATH
    if path.exists():
        loaded = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(loaded, dict):
            raise ValueError(f"Config must be a JSON object: {path}")
        if section in loaded and isinstance(loaded[section], dict):
            loaded = loaded[section]
        cfg = _deep_merge(cfg, loaded)
    else:
        print(f"Config file not found, using built-in defaults: {path}")
    return cfg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Realtime hanging yarn heatmap simulation")
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="Path to JSON config file.",
    )
    parser.add_argument(
        "--section",
        type=str,
        default=DEFAULT_CONFIG_SECTION,
        help="Top-level section name inside config file.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_runtime_config(args.config, args.section)
    sim_c = cfg["simulation"]
    rt_c = cfg["runtime"]
    mat_c = cfg["material"]
    yarn_c = cfg["yarn"]
    damp_c = cfg["damping"]
    init_c = cfg["initial_state"]
    vis_c = cfg["visualization"]

    nomex_linear_density_dtex = float(mat_c["linear_density_dtex"])
    nomex_linear_density_kg_m = nomex_linear_density_dtex * 1e-7
    nomex_density_kg_m3 = float(mat_c["solid_density_kg_m3"])
    nomex_packing_factor = float(mat_c["packing_factor"])
    nomex_bulk_density_kg_m3 = nomex_density_kg_m3 * nomex_packing_factor

    sim_cfg = FEASolverConfig(
        dt=float(sim_c["dt_s"]),
        gravity=tuple(sim_c["gravity_m_s2"]),
        solver_max_iterations=int(sim_c["solver_max_iterations"]),
        solver_tolerance=float(sim_c["solver_tolerance"]),
    )
    nominal_radius = equivalent_radius_from_linear_density(
        nomex_linear_density_kg_m,
        nomex_bulk_density_kg_m3,
    )
    yarn_cfg = FEAHangingYarnConfig(
        length=float(yarn_c["length_m"]),
        element_count=int(yarn_c["element_count"]),
        radius=nominal_radius,
        density=nomex_bulk_density_kg_m3,
        young_modulus=float(mat_c["effective_young_pa"]),
        rayleigh_damping=float(mat_c["effective_rayleigh_s"]),
        start=tuple(yarn_c["start_xyz_m"]),
        end=tuple(yarn_c["end_xyz_m"]),
        fix_start_node=bool(yarn_c["fix_start_node"]),
        fix_end_node=bool(yarn_c["fix_end_node"]),
    )
    vis_cfg = FEAVisualizationConfig(
        beam_resolution=int(vis_c["beam_resolution"]),
        beam_section_resolution=int(vis_c["beam_section_resolution"]),
        wireframe=bool(vis_c["wireframe"]),
        draw_node_glyphs=bool(vis_c["draw_node_glyphs"]),
        node_glyph_scale=float(vis_c["node_glyph_scale_m"]),
        node_glyph_thickness=float(vis_c["node_glyph_thickness_m"]),
    )

    print("=== hanging_yarn_heatmap_realtime.py parameters ===")
    print(f"config={Path(args.config)} section={args.section}")
    print(f"dt={sim_cfg.dt} s gravity={sim_cfg.gravity}")
    print(f"integration_substeps={int(sim_c.get('integration_substeps', 1))}")
    model_mu = linear_density_from_radius_and_density(yarn_cfg.radius, yarn_cfg.density)
    mu_err = 100.0 * (model_mu - nomex_linear_density_kg_m) / nomex_linear_density_kg_m
    print(
        f"material=Nomex {nomex_linear_density_dtex:g} dtex / Z90  "
        f"rho_solid={nomex_density_kg_m3} kg/m^3  "
        f"packing_factor={nomex_packing_factor}  "
        f"rho_bulk={nomex_bulk_density_kg_m3:.1f} kg/m^3"
    )
    print(
        f"elements={yarn_cfg.element_count} E={yarn_cfg.young_modulus:.2e} "
        f"damping={yarn_cfg.rayleigh_damping}"
    )
    print(
        f"linear_density_target={nomex_linear_density_kg_m:.6e} kg/m  "
        f"linear_density_model={model_mu:.6e} kg/m  error={mu_err:+.2f}%"
    )
    support_mode = "pinned" if bool(yarn_c["release_start_slope"]) else "clamped"
    print(f"support_mode={support_mode}_at_start")
    print(
        f"node_drag_gamma={float(damp_c['node_drag_gamma_s_inv']):.3f} 1/s "
        "(mass-proportional)"
    )
    print(
        "unit_system=SI (m, kg, s, Pa); dtex is converted to kg/m internally"
    )

    scene = build_hanging_yarn_scene(sim_cfg, yarn_cfg)
    if bool(yarn_c["release_start_slope"]):
        release_start_slope(scene.cable)
    initialize_fall_state(
        scene.cable,
        sag_amplitude_m=float(init_c["sag_amplitude_m"]),
        initial_down_speed_m_s=float(init_c["initial_down_speed_m_s"]),
    )
    rest_len = cable_length(scene.cable)
    attach_fea_cable_visuals(scene.mesh, vis_cfg)
    attach_fea_strain_overlay(
        scene.mesh,
        data_type=chrono.ChVisualShapeFEA.DataType_ANCF_BEAM_AX,
        min_max=(float(vis_c["overlay_min"]), float(vis_c["overlay_max"])),
    )

    for i in range(max(1, int(float(rt_c["precheck_duration_s"]) / sim_cfg.dt))):
        apply_node_drag(scene.cable, float(damp_c["node_drag_gamma_s_inv"]))
        step_system_with_substeps(
            scene.system,
            sim_cfg.dt,
            int(sim_c.get("integration_substeps", 1)),
        )
        if has_nan_cable_nodes(scene.cable):
            print(f"PRECHECK FAILED: NaN at step {i}")
            return
    print(f"PRECHECK OK: t={scene.system.GetChTime():.3f}s")

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(scene.system)
    win_w, win_h = vis_c["window_size_px"]
    vis.SetWindowSize(int(win_w), int(win_h))
    vis.SetWindowTitle(str(vis_c["window_title"]))
    vis.Initialize()
    if bool(vis_c.get("use_skybox", False)):
        vis.AddSkyBox()
    if bool(vis_c.get("use_typical_lights", False)):
        vis.AddTypicalLights()
    else:
        b = float(vis_c.get("background_brightness_pct", 35.0))
        b = max(0.0, min(100.0, b)) / 100.0
        amb = chrono.ChColor(0.12 * b, 0.12 * b, 0.12 * b)
        spc = chrono.ChColor(0.06 * b, 0.06 * b, 0.06 * b)
        dif = chrono.ChColor(0.90 * b, 0.90 * b, 0.90 * b)
        vis.AddLightDirectional(55.0, 45.0, amb, spc, dif)
        vis.AddLightDirectional(25.0, 220.0, amb, spc, chrono.ChColor(0.50 * b, 0.50 * b, 0.50 * b))
    cam_pos = chrono.ChVectorD(*vis_c["camera_pos_xyz_m"])
    cam_target = chrono.ChVectorD(*vis_c["camera_target_xyz_m"])
    vis.AddCamera(cam_pos, cam_target)
    b = max(0.0, min(100.0, float(vis_c.get("background_brightness_pct", 35.0)))) / 100.0
    bg = chrono.ChColor(0.02 + 0.70 * b, 0.02 + 0.70 * b, 0.03 + 0.66 * b)
    try:
        vis.BindAll()
    except Exception:
        pass

    realtime_timer = chrono.ChRealtimeStepTimer()
    next_print = scene.system.GetChTime()
    ref_y = yarn_cfg.start[1]
    startup_delay_s = max(0.0, float(rt_c.get("startup_delay_s", 0.0)))
    startup_delay_frames = int(startup_delay_s / max(1e-12, sim_cfg.dt))
    frame_idx = 0

    while vis.Run():
        vis.BeginScene(True, True, bg)
        vis.Render()
        vis.EndScene()
        if frame_idx < startup_delay_frames:
            frame_idx += 1
            continue

        for _ in range(int(rt_c["physics_steps_per_render"])):
            apply_node_drag(scene.cable, float(damp_c["node_drag_gamma_s_inv"]))
            step_system_with_substeps(
                scene.system,
                sim_cfg.dt,
                int(sim_c.get("integration_substeps", 1)),
            )
            if has_nan_cable_nodes(scene.cable):
                print(f"NaN detected at t={scene.system.GetChTime():.6f}s; stopping.")
                return
        realtime_timer.Spin(sim_cfg.dt * float(rt_c["physics_steps_per_render"]))

        t = scene.system.GetChTime()
        if t >= next_print:
            cur_len = cable_length(scene.cable)
            eng_strain = (cur_len - rest_len) / max(1e-12, rest_len)
            sag = cable_max_sag(scene.cable, reference_y=ref_y)
            print(f"t={t:5.2f}s  max_sag={sag:.3f}m  engineering_strain={eng_strain:+.4f}")
            next_print += float(rt_c["print_interval_s"])


if __name__ == "__main__":
    main()
