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
│  ├─ hello_world.py
│  ├─ verify_install.py
│  ├─ tutorials/            → example scenes
│  └─ tools/                → env + test helpers
├─ tests/                   → pytest smoke tests
├─ install_conda.sh         → helper to fetch Miniconda (Linux)
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
   Chrono version function not found
   FEA mesh OK: ChMesh
   System OK: ChSystemSMC
   ```

## Troubleshooting

If the base environment spec fails or you need a more reproducible setup:

```bash
conda env create -f env/environment.linux-64.lock.yml
# or: conda create -n chrono --file env/explicit-linux-64.txt
```

If you prefer helper scripts (these accept overrides if you want to point at
`environment.base.yml`):

```bash
ENV_NAME=chrono YML=env/environment.base.yml scripts/tools/setup_env.sh
# Windows PowerShell:
# .\scripts\tools\setup_env.ps1 -EnvName chrono -Yml env\environment.base.yml
```

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
python scripts/hello_world.py
python scripts/tools/smoke_test.py
python scripts/tutorials/4_chains_of_bodies/4_2_motor_pulled_rope.py
```

## Running tests

```bash
python -m pytest
# or:
scripts/tools/run_local_tests.sh
```

## Tests directory

- `tests/pytest.ini` configures pytest defaults for the repo.
- `tests/test_install.py` verifies core PyChrono imports and basic system types.
- `tests/test_sim_basic.py` runs simple NSC/SMC drop tests to validate basic dynamics.

## License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.
