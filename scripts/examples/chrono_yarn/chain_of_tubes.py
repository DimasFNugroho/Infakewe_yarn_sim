"""Minimal floor + floating yarn visualization example.

Purpose
-------
This script is an intentionally simple application-level example that shows how
to assemble and visualize a "floating yarn" scene using reusable code from
``src/chrono_yarn``:

- create a Chrono system,
- add a floor body,
- create a segmented yarn approximation (rigid bodies),
- visualize the scene with Irrlicht.

The yarn is initialized horizontally in the air. By default gravity is disabled
so the chain remains suspended while you inspect geometry placement and camera
setup. To let it fall, set ``ENABLE_GRAVITY = True``.

Important:
When the yarn bodies are fixed, gravity has no visible effect. This example
derives the default fixation mode from the gravity toggle so that enabling
gravity also makes the yarn dynamic unless you override it.

Which `src/chrono_yarn` modules are used here
---------------------------------------------
This example uses the reusable package layer directly:

- ``chrono_yarn.config`` for ``SimulationConfig``, ``FloorConfig``, ``YarnConfig``
- ``chrono_yarn.compat`` for cross-version setup helpers (Bullet preference and
  gravity setter)
- ``chrono_yarn.geometry`` for floor construction
- ``chrono_yarn.materials`` for contact material creation
- ``chrono_yarn.yarn_chain`` for segmented-yarn construction, placement, and
  optional bending spring-damper links (RSDA / TSDA proxy)

Running
-------
Run from the repository root:

    python scripts/examples/chrono_yarn/yarn.py

Requirements
------------
- A PyChrono installation with ``pychrono.irrlicht`` available.
- Either ``pip install -e .`` or direct repo execution (this script adds a local
  ``src/`` path fallback so it can run without installation during development).
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


# Allow running directly from the repo without `pip install -e .`.
# This makes the example usable during local development while still importing
# the reusable package code from `src/chrono_yarn`.
REPO_ROOT = Path(__file__).resolve().parents[3]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    # Keep the example runnable from the repo root without installing the package.
    sys.path.insert(0, str(SRC_DIR))

from chrono_yarn.compat import prefer_bullet, set_gravity, set_single_thread, tune_collision_defaults
from chrono_yarn.config import FloorConfig, SimulationConfig, YarnConfig
from chrono_yarn.geometry import add_floor_box
from chrono_yarn.materials import make_contact_material
from chrono_yarn.yarn_chain import (
    add_bending_proxy_tsdas,
    add_bending_rsdas,
    build_yarn_chain,
    max_neighbor_joint_gap,
)


ENABLE_GRAVITY = True
ENABLE_COLLISION = True
DISABLE_YARN_SELF_COLLISION = True
ANCHOR_FIRST_SEGMENT = True
PRECHECK_HEADLESS_SECONDS = 1.0  # Run a short stability check before visualization.
ENABLE_BENDING_RSDA = False    # Experimental here; can be unstable with this chain setup.
ENABLE_BENDING_PROXY = False   # Keep off by default for a stable baseline chain.
BENDING_RSDA_K = 1e-6          # Reference value for ~120 segments (1.0 m yarn).
BENDING_RSDA_C = 1e-7          # Reference damping for ~120 segments.
BENDING_RSDA_REST_ANGLE = 0.0  # Straight chain reference.
AUTO_SCALE_RSDA_WITH_SEGMENTS = True
RSDA_REF_SEGMENT_COUNT = 120   # Reference discretization for the RSDA values above.
BENDING_PROXY_SPAN = 2    # Connect i -> i+2 to resist sharp folds.
# Important stability note:
# With millimeter-scale segments + rigid contacts, stronger bending-proxy
# springs can make this chain blow up numerically (NaNs) and "disappear".
# Start very weak when collision is enabled; increase gradually.
BENDING_PROXY_K = 1e-4    # N/m. Start here only when enabling the TSDA proxy.
BENDING_PROXY_C = 1e-7    # N*s/m. Tiny damping can stabilize the proxy links.
COLLISION_ENVELOPE = 2e-4  # 0.2 mm
COLLISION_MARGIN = 1e-4    # 0.1 mm
# If set to ``None``, the script chooses ``False`` when gravity is enabled and
# ``True`` when gravity is disabled.
FIX_SEGMENTS: bool | None = None


def make_system(sim_cfg: SimulationConfig) -> chrono.ChSystem:
    """Create and configure a Chrono system for the example."""
    sys_obj = chrono.ChSystemNSC() if sim_cfg.contact_model == "NSC" else chrono.ChSystemSMC()
    prefer_bullet(sys_obj)
    # Collision defaults must match the millimeter-scale yarn geometry.
    # Large margins (mm-scale or above) can exceed the yarn radius and cause
    # incorrect contacts / instability.
    tune_collision_defaults(envelope=COLLISION_ENVELOPE, margin=COLLISION_MARGIN)
    set_single_thread(sys_obj)
    # Tighten solver settings to reduce visible joint drift in long chains.
    for setter, value in (
        ("SetSolverMaxIterations", 300),
        ("SetMaxiter", 300),
        ("SetSolverTolerance", 1e-10),
        ("SetSolverForceTolerance", 1e-10),
    ):
        if hasattr(sys_obj, setter):
            try:
                getattr(sys_obj, setter)(value)
            except Exception:
                pass
    # Toggle gravity at the application layer while reusing config defaults.
    g = sim_cfg.gravity if ENABLE_GRAVITY else (0.0, 0.0, 0.0)
    set_gravity(sys_obj, chrono.ChVectorD(*g))
    return sys_obj


def configure_collision_filters(floor_body, segments: list[chrono.ChBody]) -> None:
    """Configure collision families for a cleaner chain simulation.

    This example disables yarn-yarn self-collision (including adjacent segments)
    while preserving floor-yarn collision. With many segments, this is simpler
    and more robust than trying to manage pairwise adjacent exclusions only.
    """
    if floor_body is None or not segments:
        return

    floor_cm = floor_body.GetCollisionModel()
    floor_cm.SetFamily(1)
    floor_cm.SetFamilyMaskDoCollisionWithFamily(2)

    for seg in segments:
        cm = seg.GetCollisionModel()
        cm.SetFamily(2)
        cm.SetFamilyMaskDoCollisionWithFamily(1)      # collide with floor
        cm.SetFamilyMaskNoCollisionWithFamily(2)      # no yarn self-collision


def effective_rsda_params(yarn_cfg: YarnConfig) -> tuple[float, float]:
    """Return RSDA coefficients adjusted for discretization sensitivity.

    When segment count increases at fixed total yarn length, each segment gets
    smaller and the chain becomes much more sensitive to the same per-joint
    rotational spring/damper coefficients. A cubic scale is a practical,
    conservative rule here because segment inertia drops rapidly with segment
    length.
    """
    if not AUTO_SCALE_RSDA_WITH_SEGMENTS:
        return BENDING_RSDA_K, BENDING_RSDA_C
    if yarn_cfg.segment_count <= 0 or RSDA_REF_SEGMENT_COUNT <= 0:
        return BENDING_RSDA_K, BENDING_RSDA_C
    scale = (RSDA_REF_SEGMENT_COUNT / yarn_cfg.segment_count) ** 3
    # Never scale above the configured base values; only weaken for finer meshes.
    scale = min(1.0, scale)
    return BENDING_RSDA_K * scale, BENDING_RSDA_C * scale


def print_startup_parameters(
    sim_cfg: SimulationConfig,
    yarn_cfg: YarnConfig,
    fixed_segments: bool,
    rsda_k_eff: float,
    rsda_c_eff: float,
) -> None:
    """Print the active physics and constraint settings for debugging."""
    print("=== yarn.py parameters ===")
    print(f"dt={sim_cfg.dt} s  contact_model={sim_cfg.contact_model}")
    print(
        f"gravity={'on' if ENABLE_GRAVITY else 'off'}  "
        f"collision={'on' if ENABLE_COLLISION else 'off'}  "
        f"self_collision={'off' if (ENABLE_COLLISION and DISABLE_YARN_SELF_COLLISION) else 'on'}"
    )
    print(
        f"fixed_segments={'yes' if fixed_segments else 'no'}  "
        f"anchor_first={'yes' if (ANCHOR_FIRST_SEGMENT and not fixed_segments) else 'no'}"
    )
    print(
        f"segments={yarn_cfg.segment_count}  length={yarn_cfg.length} m  "
        f"seg_len={yarn_cfg.segment_length:.6f} m  radius={yarn_cfg.radius} m  "
        f"density={yarn_cfg.density} kg/m^3"
    )
    print(
        f"collision_envelope={COLLISION_ENVELOPE} m  "
        f"collision_margin={COLLISION_MARGIN} m"
    )
    print(
        f"rsda={'on' if ENABLE_BENDING_RSDA else 'off'} "
        f"(k={rsda_k_eff}, c={rsda_c_eff}, rest={BENDING_RSDA_REST_ANGLE}, "
        f"auto_scale={'on' if AUTO_SCALE_RSDA_WITH_SEGMENTS else 'off'})"
    )
    print(
        f"tsda_proxy={'on' if ENABLE_BENDING_PROXY else 'off'} "
        f"(span={BENDING_PROXY_SPAN}, k={BENDING_PROXY_K}, c={BENDING_PROXY_C})"
    )
    print(f"precheck_headless_seconds={PRECHECK_HEADLESS_SECONDS}")


def has_nan_positions(segments: list[chrono.ChBody]) -> bool:
    """Return True if any sampled segment position contains NaN."""
    if not segments:
        return False
    for idx in range(len(segments)):
        p = segments[idx].GetPos()
        if any(v != v for v in (p.x, p.y, p.z)):
            return True
    return False


def run_headless_precheck(system, segments: list[chrono.ChBody], dt: float, seconds: float) -> bool:
    """Advance the live scene briefly to catch obvious instability before GUI opens.

    Note:
        This advances the same system that will later be visualized. The scene
        will start from the post-check state if the precheck passes.
    """
    if seconds <= 0.0:
        return True
    steps = max(1, int(seconds / dt))
    for i in range(steps):
        system.DoStepDynamics(dt)
        if has_nan_positions(segments):
            print(f"PRECHECK FAILED: NaN detected at t={system.GetChTime():.6f}s (step {i})")
            return False
    print(f"PRECHECK OK: simulated {system.GetChTime():.3f}s without NaN before opening window")
    return True


def main() -> None:
    """Create the scene and start the Irrlicht visualization loop."""
    sim_cfg = SimulationConfig(contact_model="NSC", dt=5e-4)  # dt = 0.0005 s (0.5 ms)
    floor_cfg = FloorConfig(
        half_size=(1.4, 0.05, 0.5),  # floor full size = 2.8 m x 0.1 m x 1.0 m
        position=(0.0, 0.05, 0.0),   # floor top surface is at y = 0.10 m
    )
    yarn_cfg = YarnConfig(
        length=1.0,                    # total yarn length = 1.0 m
        segment_count=480,             # segment length ~= 8.33 mm (1.0 / 120)
        radius=0.0015,                 # yarn radius = 1.5 mm (diameter = 3.0 mm)
        density=600.0,                # kg/m^3 (synthetic fiber-like range)
        start_position=(-0.55, 0.75, 0.0),  # chain left end starts at y = 0.75 m
        # With the current floor, initial vertical clearance to floor top is
        # about 0.65 m (0.75 - 0.10), before considering segment radius.
    )

    # By default, "floating" mode fixes the yarn; enabling gravity switches to
    # a dynamic chain unless explicitly overridden.
    fixed_segments = (not ENABLE_GRAVITY) if FIX_SEGMENTS is None else FIX_SEGMENTS
    rsda_k_eff, rsda_c_eff = effective_rsda_params(yarn_cfg)
    print_startup_parameters(sim_cfg, yarn_cfg, fixed_segments, rsda_k_eff, rsda_c_eff)

    system = make_system(sim_cfg)
    anchor_body = None
    if ANCHOR_FIRST_SEGMENT and not fixed_segments:
        # Invisible fixed body used to pin the yarn's first endpoint in space.
        anchor_body = chrono.ChBody()
        anchor_body.SetBodyFixed(True)
        anchor_body.SetCollide(False)
        system.Add(anchor_body)

    contact_mat = (
        make_contact_material(sim_cfg.contact_model, friction=0.35, restitution=0.02)
        if ENABLE_COLLISION
        else None
    )  # Set to None for visual-only geometry.
    floor_body = add_floor_box(
        system,
        half_size=floor_cfg.half_size,
        position=floor_cfg.position,
        material=contact_mat,  # Floor collision enabled when `ENABLE_COLLISION = True`.
        color=(0.65, 0.65, 0.68),
    )
    # Build a segmented chain using the reusable library function from src/.
    chain = build_yarn_chain(
        system,
        yarn_cfg,
        material=contact_mat,  # Segment collision enabled when `ENABLE_COLLISION = True`.
        anchor_body=anchor_body,
        fixed_segments=fixed_segments,
    )
    segments = chain.segments

    if ENABLE_COLLISION and DISABLE_YARN_SELF_COLLISION:
        # Reduces jitter and "locking" from dense segment-segment contacts.
        configure_collision_filters(floor_body, segments)

    if ENABLE_BENDING_RSDA and not fixed_segments:
        # Rotational spring-dampers at each segment interface. This path is
        # currently experimental in this rigid-chain setup and may be unstable
        # depending on PyChrono build/solver details.
        add_bending_rsdas(
            system,
            chain,
            spring_k=rsda_k_eff,
            damping_c=rsda_c_eff,
            rest_angle=BENDING_RSDA_REST_ANGLE,
        )

    if ENABLE_BENDING_PROXY and not fixed_segments:
        # Adds weak COM-to-COM spring-dampers between every second segment.
        # This is a simple bend-resistance proxy, but currently the most robust
        # spring-damper option for this prototype chain model.
        add_bending_proxy_tsdas(
            system,
            chain,
            span=BENDING_PROXY_SPAN,
            spring_k=BENDING_PROXY_K,
            damping_c=BENDING_PROXY_C,
        )

    if not run_headless_precheck(system, segments, sim_cfg.dt, PRECHECK_HEADLESS_SECONDS):
        print(
            "Try the stable baseline (`ENABLE_BENDING_RSDA = False`, "
            "`ENABLE_BENDING_PROXY = False`), then re-enable one bending model at a time."
        )
        return

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Floating Yarn (floor + segmented yarn)")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddTypicalLights()
    # Macro-style framing: look directly at the anchored yarn region near x=-0.55.
    vis.AddCamera(chrono.ChVectorD(-0.25, 0.72, 0.35), chrono.ChVectorD(-0.52, 0.62, 0.0))
    try:
        vis.BindAll()
    except Exception:
        pass

    dt = sim_cfg.dt  # simulation step in seconds
    # Start the telemetry schedule from the current post-precheck simulation time.
    next_print = system.GetChTime()
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        # Step the physics even in visual-only mode so time advances and any
        # dynamic bodies respond if gravity/fixation settings allow motion.
        system.DoStepDynamics(dt)

        t = system.GetChTime()
        if t >= next_print and segments:
            p0 = segments[0].GetPos()
            p1 = segments[-1].GetPos()
            if any(v != v for v in (p0.x, p0.y, p0.z, p1.x, p1.y, p1.z)):
                print(
                    "NaN detected in segment positions. "
                    "Likely culprit: active bending constraints (RSDA/TSDA) with current dt/contact settings. "
                    "Return to the stable baseline (both bending modes off), then re-enable one at a time."
                )
                break
            # Lightweight runtime telemetry to confirm whether the chain is
            # fixed/dynamic and whether positions are changing as expected.
            print(
                f"t={t:5.2f}s "
                f"gravity={'on' if ENABLE_GRAVITY else 'off'} "
                f"collision={'on' if ENABLE_COLLISION else 'off'} "
                f"self_coll={'off' if (ENABLE_COLLISION and DISABLE_YARN_SELF_COLLISION) else 'on'} "
                f"fixed={'yes' if fixed_segments else 'no'} "
                f"anchor={'yes' if (anchor_body is not None) else 'no'} "
                f"rsda={'on' if ENABLE_BENDING_RSDA else 'off'} "
                f"tsda_proxy={'on' if ENABLE_BENDING_PROXY else 'off'} "
                f"max_joint_gap={1e3 * max_neighbor_joint_gap(chain):.3f}mm "
                f"head=({p0.x:+.3f},{p0.y:+.3f},{p0.z:+.3f}) "
                f"tail=({p1.x:+.3f},{p1.y:+.3f},{p1.z:+.3f})"
            )
            next_print += 0.5


if __name__ == "__main__":
    main()
