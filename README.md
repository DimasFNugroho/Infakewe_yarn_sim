# Yarn–Guide Simulation (PyChrono)

This project is a prototype for simulating yarn passing through a guiding element,
with tension measurement before/after the guide, a pulling motor, and optional
ultrasonic vibration effects. It is built using [PyChrono](https://projectchrono.org/),
the Python bindings of Project Chrono.

## Project structure

```
yarn-guide-chrono/
├─ env/                 → conda environment files
│  └─ environment.yml
├─ src/chrono_yarn/     → source package (future simulation code)
│  └─ __init__.py
├─ scripts/             → utility and test scripts
│  └─ verify_install.py
├─ .gitignore           → files and folders ignored by git
├─ README.md            → this file
└─ LICENSE              → license information (MIT by default)
```

## Environment setup

1. Create the conda environment:

   ```bash
   conda env create -f env/environment.yml
   conda activate chrono
   ```

2. Install PyChrono:

   * Recommended:

     ```bash
     conda install -c projectchrono pychrono
     ```
   * If that fails, download the `.tar.bz2` file for your platform from the
     [Project Chrono Anaconda repo](https://anaconda.org/projectchrono/pychrono)
     and install locally:

     ```bash
     conda install path\to\pychrono-<version>-py310_*.tar.bz2
     ```

3. Verify the install:

   ```bash
   python scripts/verify_install.py
   ```

   Expected output:

   ```
   Chrono version: 9.x.x
   FEA mesh OK: ChMesh
   ```
## License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.
