# Hanging Yarn Heatmap Config

This folder contains runtime configuration for:

- `scripts/examples/fea_chrono_yarn/hanging_yarn_heatmap.py`
- `scripts/examples/fea_chrono_yarn/hanging_yarn_heatmap_realtime.py`
- `scripts/examples/fea_chrono_yarn/hanging_yarn_tracker.py`

The script uses SI units:

- length: `m`
- time: `s`
- mass: `kg`
- force: `N`
- stiffness: `Pa`
- gravity: `m/s^2`

## File

- `hanging_yarn_all.json`

## Run with config

```bash
python scripts/examples/fea_chrono_yarn/hanging_yarn_heatmap.py \
  --config scripts/examples/fea_chrono_yarn/config/hanging_yarn_all.json \
  --section heatmap

python scripts/examples/fea_chrono_yarn/hanging_yarn_heatmap_realtime.py \
  --config scripts/examples/fea_chrono_yarn/config/hanging_yarn_all.json \
  --section heatmap_realtime

python scripts/examples/fea_chrono_yarn/hanging_yarn_tracker.py \
  --config scripts/examples/fea_chrono_yarn/config/hanging_yarn_all.json \
  --section tracker
```

## Key sections

- `simulation`
  - `dt_s`
  - `integration_substeps`  
    internal solver substeps per `dt_s` (helps stability without changing visual pace)
  - `gravity_m_s2`
  - `solver_max_iterations`
  - `solver_tolerance`

- `runtime`
  - `precheck_duration_s`
  - `physics_steps_per_render`
  - `print_interval_s`
  - `startup_delay_s`  
    render-only delay before stepping physics (helps window/visuals settle)

- `material`
  - `linear_density_dtex`
  - `solid_density_kg_m3`
  - `packing_factor`
  - `effective_young_pa`
  - `effective_rayleigh_s`

- `yarn`
  - `length_m`
  - `element_count`
  - `start_xyz_m`
  - `end_xyz_m`
  - `fix_start_node`
  - `fix_end_node`
  - `release_start_slope`  
    `true` = pinned-like support (position fixed, slope free)  
    `false` = clamped support (position + slope fixed)

- `damping`
  - `node_drag_gamma_s_inv`  
    mass-proportional drag used in script (`F = -gamma * m * v`)

- `initial_state`
  - `sag_amplitude_m`
  - `initial_down_speed_m_s`

- `visualization`
  - `window_size_px`
  - `window_title`
  - `camera_pos_xyz_m`
  - `camera_target_xyz_m`
  - `use_skybox` (`false` gives dark background)
  - `use_typical_lights`
  - `background_brightness_pct` (`0` = darkest, `100` = brightest)
  - `beam_resolution`
  - `beam_section_resolution`
  - `wireframe`
  - `draw_node_glyphs`
  - `node_glyph_scale_m`
  - `node_glyph_thickness_m`
  - `overlay_min`
  - `overlay_max`

- `output` (tracker only)
  - `directory`
  - `nodes_csv`
  - `elements_csv`
  - `summary_csv`

## Practical tuning hints

- Softer yarn: lower `material.effective_young_pa`.
- More damping/less oscillation:
  - increase `material.effective_rayleigh_s`
  - increase `damping.node_drag_gamma_s_inv`
- Faster runtime:
  - increase `simulation.dt_s`
  - keep `simulation.dt_s` moderate and increase `simulation.integration_substeps` for stability
  - reduce `yarn.element_count`
  - reduce visualization resolution fields
- More rope-like hanging:
  - set `yarn.release_start_slope = true`
- More cantilever-like hanging:
  - set `yarn.release_start_slope = false`

## Model note (paper-aligned usage)

Current scripts use a 1D ANCF yarn/rod approximation with effective properties
(Nomex dtex + bulk density + effective modulus/damping). This is suitable for
hanging/falling/tension behavior and quick tuning.

The referenced CBOS paper uses yarn-level 3D solid FEM contact in Abaqus for
internal yarn-yarn displacement and cross-section effects. That full 3D contact
model is beyond the current script scope; use the current model as an effective
engineering approximation.
