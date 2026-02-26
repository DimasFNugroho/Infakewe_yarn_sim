# `chrono_yarn` Package Overview

## Purpose

`chrono_yarn` is the reusable library layer for yarn simulation experiments built
on PyChrono.

This package is intended to contain:

- reusable configuration types
- scene-building logic
- geometry/body construction helpers
- simulation stepping and sampling
- result containers
- compatibility helpers for PyChrono API differences

Runnable demos, tutorials, and experiment scripts should remain in `scripts/`.

## Current Status

The package includes a documented architecture skeleton for the first milestone
(a segmented yarn falling onto a floor).

Some modules are already implemented (configuration, result types, compatibility
helpers), while scene assembly and stepping functions are intentionally stubbed
with `NotImplementedError` until the physics implementation is added.

## Module Responsibilities

### `__init__.py`

Public package exports (currently compatibility utilities).

### `compat.py`

PyChrono version/build compatibility helpers.

Primary role:

- centralize API differences (method names, optional features)
- keep scene/test code free of repeated `hasattr` / `try-except` checks

Examples:

- gravity setter compatibility
- Bullet collision backend selection (if available)
- collision default tuning (if supported)
- thread-count configuration (if supported)

### `config.py`

Simulation input dataclasses.

Defines:

- `SolverTuning`
- `SimulationConfig`
- `YarnConfig`
- `FloorConfig`

This module should stay free of PyChrono object construction so configs can be
created in tests/scripts without pulling in engine-specific state.

### `materials.py`

Contact material factory helpers for NSC/SMC systems.

Role:

- create consistently configured contact materials
- isolate NSC/SMC branching from scene builders

### `geometry.py`

Rigid-body and shape construction helpers.

Intended role:

- create Chrono bodies
- attach visual shapes
- configure collision models
- add bodies to the Chrono system

Current state:

- `colorize(...)` is implemented
- floor/segment creation functions are stubs

### `yarn_chain.py`

Assembly logic for the discretized yarn model (rigid segments + joints).

Intended role:

- create and place segments from `YarnConfig`
- connect neighboring segments with joints
- optionally anchor an end
- return handles to created bodies/links

Also includes:

- `extract_segment_positions(...)` for sampling/recording

Current state:

- handle dataclass and extraction helper are present
- chain builder is stubbed

### `scenes/`

Scenario-level composition modules.

These modules should combine lower-level pieces from `config`, `materials`,
`geometry`, and `yarn_chain` into complete simulation setups.

#### `scenes/falling_yarn.py`

First milestone scenario: free yarn dropped onto a floor.

Current state:

- Chrono system creation/configuration is implemented
- floor + yarn assembly is stubbed

### `sim_runner.py`

Simulation stepping and sampling interface.

Intended role:

- run fixed-step dynamics loop
- sample state at configured intervals
- return `SimulationResult`

Current state:

- `SimulationRunner` API defined
- `run(...)` is stubbed

### `results.py`

Plain-Python result dataclasses for recorded simulation data.

Role:

- provide stable output structures for tests and post-processing
- avoid exposing raw PyChrono objects in recorded results

## Intended Usage Flow (from `scripts/`)

1. Create configs (`SimulationConfig`, `YarnConfig`, `FloorConfig`)
2. Construct a scene object (for example `FallingYarnScene`)
3. Build scene handles (`scene.build()`) once implemented
4. Create a `SimulationRunner`
5. Execute `runner.run(scene_handles)` once implemented
6. Inspect/process `SimulationResult`

## Design Guidelines

- Keep `src/chrono_yarn` reusable and example-agnostic.
- Keep `scripts/` thin: orchestration, plotting, ad hoc experiments.
- Put PyChrono API compatibility logic in `compat.py`, not in every scene/script.
- Keep recording outputs in plain Python data structures (`results.py`).
- Prefer explicit module boundaries over large monolithic scene scripts.

## What To Implement Next (Milestone 1)

1. `geometry.py`
   - floor box creation with collision + visuals
   - yarn segment body creation (capsule-style approximation)
2. `yarn_chain.py`
   - segment placement and spherical joints
3. `scenes/falling_yarn.py`
   - compose floor + yarn into `SceneHandles`
4. `sim_runner.py`
   - stepping loop + sampling into `SimulationResult`

