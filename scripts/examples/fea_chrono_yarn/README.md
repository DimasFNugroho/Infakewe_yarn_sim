# Single Yarn Example (Clamped + Free Fall)

This folder intentionally contains one simulation example only:

- `scripts/examples/fea_chrono_yarn/paper_yarn_clamped_solid.py`

The scenario is:

1. yarn clamped at one end
2. free at the other end under gravity

Configuration is loaded from:

- `scripts/examples/fea_chrono_yarn/config/paper_yarn_clamped_solid.json`

The config uses SI units (`m`, `s`, `kg`, `Pa`, `N`).

## Run

```bash
python scripts/examples/fea_chrono_yarn/paper_yarn_clamped_solid.py \
  --config scripts/examples/fea_chrono_yarn/config/paper_yarn_clamped_solid.json
```

## Config sections

- `simulation`
  - `dt_s`, `integration_substeps`, `gravity_m_s2`
  - `solver_max_iterations`, `solver_tolerance`
- `runtime`
  - `precheck_duration_s`, `physics_steps_per_render`
  - `print_interval_s`, `startup_delay_s`
- `yarn`
  - geometry (`length_m`, `element_count`, origin/directions)
  - material (`young_modulus_pa`, `poisson_ratio`, `density_kg_m3`)
  - section/damping (`diameter_m`, `rayleigh_damping_s`)
  - initial state (`initial_sag_amplitude_m`, `initial_tip_down_velocity_m_s`)
- `visualization`
  - window, camera, lights, beam resolution, overlay range

## Tuning quick guide

- More stable: lower `simulation.dt_s` and/or increase `simulation.integration_substeps`
- Faster wall-clock runtime: increase `simulation.dt_s`, reduce `yarn.element_count`
- Faster visible motion: increase `runtime.physics_steps_per_render`
- Less oscillation: increase `yarn.rayleigh_damping_s`
- Softer yarn: decrease `yarn.young_modulus_pa`
