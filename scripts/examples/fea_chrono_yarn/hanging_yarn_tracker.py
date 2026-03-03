"""Headless tracker for one-end-fixed yarn: node/element trajectories to CSV.

Purpose:
- diagnose why a yarn may appear to "float" or oscillate unrealistically,
- export per-node positions/velocities over time,
- export per-element stretch and whole-yarn summary metrics.

Outputs (created under `outputs/`):
- hanging_yarn_nodes.csv
- hanging_yarn_elements.csv
- hanging_yarn_summary.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import pychrono as chrono

from chrono_yarn.fea_yarn import FEAHangingYarnConfig, FEASolverConfig, build_hanging_yarn_scene


# SI units throughout.
TOTAL_TIME_S = 2.0
DT_S = 5e-4
SAMPLE_EVERY_STEPS = 20  # 20 * dt = 0.010 s
NODE_DRAG_GAMMA_S_INV = 0.25

NOMEX_LINEAR_DENSITY_DTEX = 2670.0
NOMEX_LINEAR_DENSITY_KG_M = NOMEX_LINEAR_DENSITY_DTEX * 1e-7
NOMEX_DENSITY_KG_M3 = 1380.0
NOMEX_PACKING_FACTOR = 0.62
NOMEX_BULK_DENSITY_KG_M3 = NOMEX_DENSITY_KG_M3 * NOMEX_PACKING_FACTOR
NOMEX_EFFECTIVE_YOUNG_PA = 1.0e7
NOMEX_EFFECTIVE_RAYLEIGH_S = 2.0e-3

DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parent / "config" / "hanging_yarn_all.json"
)
DEFAULT_CONFIG_SECTION = "tracker"
DEFAULT_CFG = {
    "simulation": {
        "total_time_s": TOTAL_TIME_S,
        "dt_s": DT_S,
        "integration_substeps": 1,
        "sample_every_steps": SAMPLE_EVERY_STEPS,
        "gravity_m_s2": [0.0, -9.81, 0.0],
        "solver_max_iterations": 120,
        "solver_tolerance": 5e-8,
    },
    "material": {
        "linear_density_dtex": NOMEX_LINEAR_DENSITY_DTEX,
        "solid_density_kg_m3": NOMEX_DENSITY_KG_M3,
        "packing_factor": NOMEX_PACKING_FACTOR,
        "effective_young_pa": NOMEX_EFFECTIVE_YOUNG_PA,
        "effective_rayleigh_s": NOMEX_EFFECTIVE_RAYLEIGH_S,
    },
    "yarn": {
        "length_m": 1.0,
        "element_count": 18,
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
    },
    "output": {
        "directory": "outputs",
        "nodes_csv": "hanging_yarn_nodes.csv",
        "elements_csv": "hanging_yarn_elements.csv",
        "summary_csv": "hanging_yarn_summary.csv",
    },
}


def equivalent_radius_from_linear_density(mu_kg_m: float, bulk_density_kg_m3: float) -> float:
    area = float(mu_kg_m) / float(bulk_density_kg_m3)
    return math.sqrt(area / math.pi)


def linear_density_from_radius_and_density(radius_m: float, density_kg_m3: float) -> float:
    area_m2 = math.pi * float(radius_m) * float(radius_m)
    return float(density_kg_m3) * area_m2


def initialize_fall_state(cable, *, sag_amplitude_m: float = 0.04) -> None:
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
            node.SetPos_dt(chrono.ChVectorD(0.0, 0.0, 0.0))


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


def apply_node_drag(cable, gamma_s_inv: float) -> None:
    g = float(max(0.0, gamma_s_inv))
    if g <= 0.0:
        return
    for i, node in enumerate(cable.nodes):
        if i == 0:
            continue
        if hasattr(node, "IsFixed") and node.IsFixed():
            continue
        v = node.GetPos_dt() if hasattr(node, "GetPos_dt") else None
        if v is None:
            continue
        mass = float(node.GetMass()) if hasattr(node, "GetMass") else 0.0
        if mass <= 0.0:
            continue
        if hasattr(node, "SetForce"):
            node.SetForce(chrono.ChVectorD(-g * mass * v.x, -g * mass * v.y, -g * mass * v.z))


def step_system_with_substeps(system, dt: float, substeps: int) -> None:
    n = max(1, int(substeps))
    dt_sub = float(dt) / float(n)
    for _ in range(n):
        system.DoStepDynamics(dt_sub)


def node_pos(node) -> tuple[float, float, float]:
    p = node.GetPos()
    return float(p.x), float(p.y), float(p.z)


def node_vel(node) -> tuple[float, float, float]:
    v = node.GetPos_dt() if hasattr(node, "GetPos_dt") else None
    if v is None:
        return 0.0, 0.0, 0.0
    return float(v.x), float(v.y), float(v.z)


def segment_length(a, b) -> float:
    ax, ay, az = node_pos(a)
    bx, by, bz = node_pos(b)
    dx = bx - ax
    dy = by - ay
    dz = bz - az
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def cable_length(cable) -> float:
    total = 0.0
    for i in range(len(cable.nodes) - 1):
        total += segment_length(cable.nodes[i], cable.nodes[i + 1])
    return total


def center_of_mass_y(cable) -> float:
    y_sum = 0.0
    n = max(1, len(cable.nodes))
    for node in cable.nodes:
        y_sum += node.GetPos().y
    return float(y_sum / n)


def max_node_nan(cable) -> bool:
    for node in cable.nodes:
        x, y, z = node_pos(node)
        if (x != x) or (y != y) or (z != z):
            return True
    return False


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
    parser = argparse.ArgumentParser(description="Headless hanging yarn tracker")
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
    mat_c = cfg["material"]
    yarn_c = cfg["yarn"]
    damp_c = cfg["damping"]
    init_c = cfg["initial_state"]
    out_c = cfg["output"]

    out_dir = Path(out_c["directory"])
    out_dir.mkdir(parents=True, exist_ok=True)
    nodes_csv = out_dir / str(out_c["nodes_csv"])
    elems_csv = out_dir / str(out_c["elements_csv"])
    summary_csv = out_dir / str(out_c["summary_csv"])

    nomex_linear_density_dtex = float(mat_c["linear_density_dtex"])
    nomex_linear_density_kg_m = nomex_linear_density_dtex * 1e-7
    nomex_density_kg_m3 = float(mat_c["solid_density_kg_m3"])
    nomex_packing_factor = float(mat_c["packing_factor"])
    nomex_bulk_density_kg_m3 = nomex_density_kg_m3 * nomex_packing_factor

    radius_m = equivalent_radius_from_linear_density(
        nomex_linear_density_kg_m,
        nomex_bulk_density_kg_m3,
    )
    sim_cfg = FEASolverConfig(
        dt=float(sim_c["dt_s"]),
        gravity=tuple(sim_c["gravity_m_s2"]),
        solver_max_iterations=int(sim_c["solver_max_iterations"]),
        solver_tolerance=float(sim_c["solver_tolerance"]),
    )
    yarn_cfg = FEAHangingYarnConfig(
        length=float(yarn_c["length_m"]),
        element_count=int(yarn_c["element_count"]),
        radius=radius_m,
        density=nomex_bulk_density_kg_m3,
        young_modulus=float(mat_c["effective_young_pa"]),
        rayleigh_damping=float(mat_c["effective_rayleigh_s"]),
        start=tuple(yarn_c["start_xyz_m"]),
        end=tuple(yarn_c["end_xyz_m"]),
        fix_start_node=bool(yarn_c["fix_start_node"]),
        fix_end_node=bool(yarn_c["fix_end_node"]),
    )
    scene = build_hanging_yarn_scene(sim_cfg, yarn_cfg)
    if bool(yarn_c["release_start_slope"]):
        release_start_slope(scene.cable)
    initialize_fall_state(scene.cable, sag_amplitude_m=float(init_c["sag_amplitude_m"]))

    target_mu = nomex_linear_density_kg_m
    model_mu = linear_density_from_radius_and_density(yarn_cfg.radius, yarn_cfg.density)
    err_mu_pct = 100.0 * (model_mu - target_mu) / max(1e-16, target_mu)

    p0 = scene.cable.first_node.GetPos()
    p1 = scene.cable.last_node.GetPos()
    chord_m = math.sqrt((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2 + (p1.z - p0.z) ** 2)
    print("=== hanging_yarn_tracker.py ===")
    print(f"config={Path(args.config)} section={args.section}")
    print("unit_system=SI (m, kg, s, Pa)")
    print(f"integration_substeps={int(sim_c.get('integration_substeps', 1))}")
    print(f"target_mu={target_mu:.6e} kg/m  model_mu={model_mu:.6e} kg/m  err={err_mu_pct:+.3f}%")
    print(f"initial_chord_length={chord_m:.6f} m  initial_cable_length={cable_length(scene.cable):.6f} m")
    if abs(chord_m - cable_length(scene.cable)) < 1e-4:
        print("note: initial geometry is near-taut; free-end drop may be small without disturbance/drag.")

    with (
        nodes_csv.open("w", newline="", encoding="utf-8") as nf,
        elems_csv.open("w", newline="", encoding="utf-8") as ef,
        summary_csv.open("w", newline="", encoding="utf-8") as sf,
    ):
        nwr = csv.writer(nf)
        ewr = csv.writer(ef)
        swr = csv.writer(sf)
        nwr.writerow(["time_s", "node_id", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s"])
        ewr.writerow(["time_s", "elem_id", "length_m", "eng_strain"])
        swr.writerow(
            [
                "time_s",
                "free_end_x_m",
                "free_end_y_m",
                "free_end_z_m",
                "com_y_m",
                "cable_length_m",
                "free_end_speed_m_s",
            ]
        )

        initial_elem_lengths = [
            segment_length(scene.cable.nodes[i], scene.cable.nodes[i + 1])
            for i in range(len(scene.cable.nodes) - 1)
        ]

        steps = int(float(sim_c["total_time_s"]) / float(sim_c["dt_s"]))
        y_free_initial = float(scene.cable.last_node.GetPos().y)
        y_initial = [float(n.GetPos().y) for n in scene.cable.nodes]
        y_free_min = y_free_initial
        for step in range(steps + 1):
            t = float(scene.system.GetChTime())
            if step % int(sim_c["sample_every_steps"]) == 0:
                for i, node in enumerate(scene.cable.nodes):
                    x, y, z = node_pos(node)
                    vx, vy, vz = node_vel(node)
                    nwr.writerow([f"{t:.6f}", i, x, y, z, vx, vy, vz])
                for i in range(len(scene.cable.nodes) - 1):
                    el = segment_length(scene.cable.nodes[i], scene.cable.nodes[i + 1])
                    l0 = initial_elem_lengths[i]
                    strain = (el - l0) / max(1e-16, l0)
                    ewr.writerow([f"{t:.6f}", i, el, strain])
                fx, fy, fz = node_pos(scene.cable.last_node)
                fvx, fvy, fvz = node_vel(scene.cable.last_node)
                fs = math.sqrt(fvx * fvx + fvy * fvy + fvz * fvz)
                swr.writerow(
                    [
                        f"{t:.6f}",
                        fx,
                        fy,
                        fz,
                        center_of_mass_y(scene.cable),
                        cable_length(scene.cable),
                        fs,
                    ]
                )

            apply_node_drag(scene.cable, float(damp_c["node_drag_gamma_s_inv"]))
            step_system_with_substeps(
                scene.system,
                float(sim_c["dt_s"]),
                int(sim_c.get("integration_substeps", 1)),
            )
            y_free_min = min(y_free_min, float(scene.cable.last_node.GetPos().y))
            if max_node_nan(scene.cable):
                print(f"NaN detected at t={scene.system.GetChTime():.6f}s, stopping early.")
                break

    y_free_final = float(scene.cable.last_node.GetPos().y)
    print(
        f"free_end_drop={y_free_initial - y_free_final:+.4f} m "
        f"(y0={y_free_initial:.4f} -> yT={y_free_final:.4f})"
    )
    print(
        f"free_end_max_drop={y_free_initial - y_free_min:+.4f} m "
        f"(y_min={y_free_min:.4f})"
    )
    y_final = [float(n.GetPos().y) for n in scene.cable.nodes]
    drops = [y0 - y1 for y0, y1 in zip(y_initial, y_final)]
    dropped_nodes = sum(1 for d in drops if d > 0.05)
    frac = dropped_nodes / max(1, len(drops))
    print(f"nodes_drop_over_5cm={dropped_nodes}/{len(drops)} ({100.0*frac:.1f}%)")

    print(f"saved: {nodes_csv}")
    print(f"saved: {elems_csv}")
    print(f"saved: {summary_csv}")


if __name__ == "__main__":
    main()
