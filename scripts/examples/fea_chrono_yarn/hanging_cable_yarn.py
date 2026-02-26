"""Hanging yarn example using a cable-based FEA model (`pychrono.fea`).

This example builds a single ANCF cable strand, fixes the start node, and lets
gravity create a hanging shape. It is the first FEA milestone for a yarn-like
model (continuous deformation, no rigid-segment joints).

The example uses reusable helpers from `src/chrono_yarn/fea_yarn`.
"""

from __future__ import annotations

import sys
from pathlib import Path

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
    build_hanging_yarn_scene,
    cable_max_sag,
    cable_tip_position,
)


PRECHECK_HEADLESS_SECONDS = 1.0
PHYSICS_STEPS_PER_RENDER = 8
SCENARIO_MODE = "hanging_knitting_cotton"  # or: "free_fall_knitting_cotton"


def has_nan_cable_nodes(cable) -> bool:
    """Return True if any cable node position contains NaN."""
    for node in cable.nodes:
        p = node.GetPos()
        if any(v != v for v in (p.x, p.y, p.z)):
            return True
    return False


def run_headless_precheck(system, cable, dt: float, seconds: float) -> bool:
    """Advance the scene before opening the window to catch instability early."""
    if seconds <= 0.0:
        return True
    for i in range(max(1, int(seconds / dt))):
        system.DoStepDynamics(dt)
        if has_nan_cable_nodes(cable):
            print(f"PRECHECK FAILED: NaN detected at t={system.GetChTime():.6f}s (step {i})")
            return False
    print(f"PRECHECK OK: simulated {system.GetChTime():.3f}s without NaN before opening window")
    return True


def print_startup_parameters(
    sim_cfg: FEASolverConfig,
    yarn_cfg: FEAHangingYarnConfig,
    *,
    scenario_mode: str,
    precheck_seconds: float,
) -> None:
    """Print the active FEA cable parameters for debugging and tuning."""
    print("=== hanging_cable_yarn.py parameters ===")
    print(f"scenario_mode={scenario_mode}")
    print(f"dt={sim_cfg.dt} s  gravity={sim_cfg.gravity}")
    print(f"solver_max_iterations={sim_cfg.solver_max_iterations}  solver_tolerance={sim_cfg.solver_tolerance}")
    print(
        f"length={yarn_cfg.length} m  elements={yarn_cfg.element_count}  "
        f"elem_len={yarn_cfg.element_length:.6f} m"
    )
    print(
        f"radius={yarn_cfg.radius} m  density={yarn_cfg.density} kg/m^3  "
        f"E={yarn_cfg.young_modulus:.3e} Pa  rayleigh={yarn_cfg.rayleigh_damping}"
    )
    print(f"start={yarn_cfg.start}  end={yarn_cfg.end}")
    print(f"fix_start={yarn_cfg.fix_start_node}  fix_end={yarn_cfg.fix_end_node}")
    print(f"precheck_headless_seconds={precheck_seconds}")


def make_yarn_preset(mode: str) -> tuple[FEAHangingYarnConfig, float]:
    """Return `(yarn_cfg, precheck_seconds)` for a named demo scenario."""
    key = (mode or "").strip().lower()

    if key == "free_fall_knitting_cotton":
        return (
            FEAHangingYarnConfig(
                length=1.0,
                element_count=80,
                radius=0.0011,             # 2.2 mm diameter knitting cotton yarn
                density=750.0,             # effective bulk density
                young_modulus=8e7,         # effective yarn stiffness
                rayleigh_damping=5e-5,
                start=(-0.15, 0.95, 0.0),
                end=(0.15, 0.95, 0.0),     # short span => visible slack-like drop
                fix_start_node=False,
                fix_end_node=False,
            ),
            0.0,  # show the actual falling motion from t=0
        )

    if key != "hanging_knitting_cotton":
        raise ValueError(
            "Unknown SCENARIO_MODE. Use 'hanging_knitting_cotton' or "
            "'free_fall_knitting_cotton'."
        )

    return (
        FEAHangingYarnConfig(
            length=1.0,
            element_count=80,
            radius=0.0011,
            density=750.0,
            young_modulus=8e7,
            rayleigh_damping=2e-4,
            start=(-0.55, 0.75, 0.0),
            end=(0.45, 0.75, 0.0),
            fix_start_node=True,
            fix_end_node=False,
        ),
        PRECHECK_HEADLESS_SECONDS,
    )


def main() -> None:
    """Build, validate, and visualize a hanging FEA cable yarn."""
    sim_cfg = FEASolverConfig(
        dt=5e-4,                   # 1.0 ms for faster progress while keeping stability
        solver_max_iterations=200,  # faster MINRES solve; good enough for this demo
        solver_tolerance=1e-8,     # relax tolerance for better throughput
    )
    yarn_cfg, precheck_seconds = make_yarn_preset(SCENARIO_MODE)
    vis_cfg = FEAVisualizationConfig(
        beam_resolution=8,
        beam_section_resolution=6,
        wireframe=False,
        draw_node_glyphs=True,
        node_glyph_scale=0.004,
        node_glyph_thickness=0.002,
    )

    print_startup_parameters(
        sim_cfg,
        yarn_cfg,
        scenario_mode=SCENARIO_MODE,
        precheck_seconds=precheck_seconds,
    )
    scene = build_hanging_yarn_scene(sim_cfg, yarn_cfg)
    attach_fea_cable_visuals(scene.mesh, vis_cfg)

    if not run_headless_precheck(scene.system, scene.cable, sim_cfg.dt, precheck_seconds):
        print("Reduce `dt`, reduce `young_modulus`, or increase damping before visualization.")
        return

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(scene.system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("FEA Hanging Cable Yarn (ANCF)")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVectorD(-0.10, 0.85, 0.65), chrono.ChVectorD(-0.05, 0.55, 0.0))
    try:
        vis.BindAll()
    except Exception:
        pass

    dt = sim_cfg.dt
    next_print = scene.system.GetChTime()
    ref_y = yarn_cfg.start[1]

    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(PHYSICS_STEPS_PER_RENDER):
            scene.system.DoStepDynamics(dt)

        t = scene.system.GetChTime()
        if t >= next_print:
            tip = cable_tip_position(scene.cable)
            sag = cable_max_sag(scene.cable, reference_y=ref_y)
            print(
                f"t={t:5.2f}s tip=({tip[0]:+.3f},{tip[1]:+.3f},{tip[2]:+.3f}) "
                f"max_sag={sag:.3f}m"
            )
            next_print += 0.5


if __name__ == "__main__":
    main()
