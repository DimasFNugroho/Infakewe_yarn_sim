"""Braided multi-strand ANCF yarn bending over a rigid sheave.

Milestone goals:
- generate opposite-handed braided strands around a common centerline,
- include sheave contact (node cloud vs rigid cylinder),
- apply prescribed endpoint pulling toward the sheave,
- print an interlacing relative-displacement proxy over time.
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
    FEABraidedYarnConfig,
    FEAPullProgramConfig,
    FEASheaveConfig,
    FEASolverConfig,
    FEAVisualizationConfig,
    attach_fea_cable_visuals,
    attach_fea_strain_overlay,
    build_braided_sheave_scene,
    estimate_relative_displacement,
    max_node_nan,
    step_braided_sheave_scene,
)


PHYSICS_STEPS_PER_RENDER = 6
PRINT_EVERY_SECONDS = 0.25
PRECHECK_HEADLESS_SECONDS = 0.2


def run_headless_precheck(scene, dt: float, seconds: float) -> bool:
    if seconds <= 0.0:
        return True
    steps = max(1, int(seconds / dt))
    for i in range(steps):
        step_braided_sheave_scene(scene, dt)
        if max_node_nan(scene):
            print(f"PRECHECK FAILED: NaN detected at t={scene.system.GetChTime():.6f}s (step {i})")
            return False
    print(f"PRECHECK OK: simulated {scene.system.GetChTime():.3f}s without NaN before opening window")
    return True


def print_startup_parameters(sim_cfg, braid_cfg, sheave_cfg, pull_cfg) -> None:
    print("=== braided_sheave_yarn.py parameters ===")
    print(f"dt={sim_cfg.dt} s  gravity={sim_cfg.gravity}")
    print(
        f"strands={braid_cfg.strand_count} "
        f"({braid_cfg.clockwise_strands} cw + {braid_cfg.counterclockwise_strands} ccw)  "
        f"elements/strand={braid_cfg.element_count}"
    )
    print(
        f"length={braid_cfg.length} m  strand_radius={braid_cfg.strand_radius} m  "
        f"braid_radius={braid_cfg.braid_radius} m  pitch={braid_cfg.braid_pitch} m"
    )
    print(
        f"sheave_radius={sheave_cfg.radius} m  wrap_angle={sheave_cfg.wrap_angle_deg} deg  "
        f"mu={sheave_cfg.friction}  contact_node_radius={sheave_cfg.contact_node_radius} m"
    )
    print(
        f"pull_distance={pull_cfg.pull_distance} m  pull_duration={pull_cfg.pull_duration} s  "
        f"hold_duration={pull_cfg.hold_duration} s"
    )


def main() -> None:
    sim_cfg = FEASolverConfig(
        dt=2e-4,
        gravity=(0.0, -9.81, 0.0),
        solver_max_iterations=250,
        solver_tolerance=1e-9,
    )
    braid_cfg = FEABraidedYarnConfig(
        length=0.60,
        element_count=120,
        strand_radius=0.0008,
        braid_radius=0.0045,
        braid_pitch=0.08,
        clockwise_strands=4,
        counterclockwise_strands=4,
        density=900.0,
        young_modulus=3.0e8,
        rayleigh_damping=2e-4,
    )
    sheave_cfg = FEASheaveConfig(
        radius=0.02,
        width=0.05,
        friction=0.2,
        wrap_angle_deg=120.0,
        contact_node_radius=0.0011,
        collision_envelope=2e-4,
        collision_margin=1e-4,
    )
    pull_cfg = FEAPullProgramConfig(
        pull_distance=0.045,
        pull_duration=1.2,
        hold_duration=0.6,
    )
    vis_cfg = FEAVisualizationConfig(
        beam_resolution=8,
        beam_section_resolution=6,
        wireframe=False,
        draw_node_glyphs=False,
    )
    print_startup_parameters(sim_cfg, braid_cfg, sheave_cfg, pull_cfg)

    scene = build_braided_sheave_scene(sim_cfg, braid_cfg, sheave_cfg, pull_cfg)
    attach_fea_cable_visuals(scene.mesh, vis_cfg)
    attach_fea_strain_overlay(scene.mesh)

    if not run_headless_precheck(scene, sim_cfg.dt, PRECHECK_HEADLESS_SECONDS):
        print("Reduce dt / modulus / contact radius to improve stability.")
        return

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(scene.system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Braided ANCF Yarn over Sheave")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVectorD(0.12, 0.10, 0.18), chrono.ChVectorD(0.0, -0.01, 0.0))
    try:
        vis.BindAll()
    except Exception:
        pass

    next_print = scene.system.GetChTime()
    dt = sim_cfg.dt
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(PHYSICS_STEPS_PER_RENDER):
            step_braided_sheave_scene(scene, dt)
            if max_node_nan(scene):
                print(f"NaN detected at t={scene.system.GetChTime():.6f}s; stopping.")
                return

        t = scene.system.GetChTime()
        if t >= next_print:
            rd = estimate_relative_displacement(scene)
            print(f"t={t:5.2f}s  RD_proxy={rd*1000.0:.3f} mm")
            next_print += PRINT_EVERY_SECONDS


if __name__ == "__main__":
    main()
