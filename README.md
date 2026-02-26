# Yarn–Guide Simulation (PyChrono)

This project is a prototype for simulating yarn passing through a guiding element,
with tension measurement before/after the guide, a pulling motor, and optional
ultrasonic vibration effects. It is built using [PyChrono](https://projectchrono.org/),
the Python bindings of Project Chrono.

## Project structure

```
Infakewe_yarn_sim/
├─ env/                     → conda environment specs (base + lock/explicit)
│  ├─ environment.base.yml
│  ├─ environment.linux-64.lock.yml
│  ├─ environment.win-64.lock.yml
│  ├─ explicit-linux-64.txt
│  └─ explicit-win-64.txt
├─ src/chrono_yarn/         → source package (future simulation code)
│  └─ __init__.py
├─ scripts/                 → runnable examples + tools
│  ├─ verify_install.py
│  ├─ tutorials/            → example scenes
│  └─ tools/                → env + test helpers
├─ tests/                   → pytest smoke tests
├─ install_conda.sh         → helper to install Miniconda (Linux)
├─ .github/workflows/ci.yml → CI
├─ .gitignore               → ignored files
├─ README.md                → this file
└─ LICENSE                  → license information (MIT by default)
```

## Environment setup

1. Create the conda environment (recommended base spec):

   ```bash
   conda env create -f env/environment.base.yml
   conda activate chrono
   ```

2. Verify the install:

   ```bash
   python scripts/verify_install.py
   ```

   Expected output (example):

   ```
   Chrono version: unknown  # or a version string, depending on build
   FEA mesh OK: ChMesh
   System OK: ChSystemSMC
   ```

## Troubleshooting

If the base environment spec fails or you need a more reproducible setup:

```bash
conda env create -f env/environment.linux-64.lock.yml
# or: conda create -n chrono --file env/explicit-linux-64.txt
```

Terminology used in this repo:

- **Base spec** (`environment.base.yml`): human‑edited, flexible dependencies.
- **Lock spec** (`environment.*.lock.yml`): exact versions/builds, but created via
  the solver so compatibility constraints are enforced.
- **Explicit spec** (`explicit-*.txt`): exact package files, installed directly
  without solver checks.

Examples of when to use each:

- **Base**: you’re developing and want to add or update dependencies.
- **Lock**: CI or a team wants consistent environments with compatibility checks.
- **Explicit**: you need maximum reproducibility and can accept brittleness if
  exact package URLs change.

If you prefer helper scripts (these accept overrides if you want to point at
`environment.base.yml`):

```bash
ENV_NAME=chrono YML=env/environment.base.yml scripts/tools/setup_env.sh
# Windows PowerShell:
# .\scripts\tools\setup_env.ps1 -EnvName chrono -Yml env\environment.base.yml
```

Note: `install_conda.sh` installs Miniconda into `~/miniconda3`.

If PyChrono is missing after the environment is created:

```bash
conda install -c projectchrono pychrono
```

If that fails, download the `.tar.bz2` file for your platform from the
[Project Chrono Anaconda repo](https://anaconda.org/projectchrono/pychrono)
and install locally:

```bash
conda install path\to\pychrono-<version>-py310_*.tar.bz2
```

## Running the examples

```bash
python scripts/verify_install.py
python scripts/tools/smoke_test.py
python scripts/tutorials/4_chains_of_bodies/4_2_motor_pulled_rope.py
```

## Running tests

```bash
python -m pytest
# or:
scripts/tools/run_local_tests.sh
```

If you created the env from explicit specs, `pip` should already be included.
To force an editable reinstall before running tests:

```bash
REINSTALL=1 scripts/tools/run_local_tests.sh
```

Note: running `python -m pytest` directly assumes you already ran
`python -m pip install -e .` in the active environment.

## Packaging note

`pip install -e .` is supported for the helper package only. PyChrono itself is
installed via conda (it is not available on PyPI). Create the conda env first,
then use editable install if you want local imports for `chrono_yarn` and helpers.

## Tests directory

- `pytest.ini` configures pytest defaults for the repo.
- `tests/test_install.py` verifies core PyChrono imports and basic system types.
- `tests/test_sim_basic.py` runs simple NSC/SMC drop tests to validate basic dynamics.

## License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.
