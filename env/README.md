# Environment specs

This folder contains multiple environment specs with different levels of
reproducibility. Pick the format that matches your workflow.

- `environment.base.yml`
  - Primary, human‑edited spec.
  - Use for day‑to‑day work and for regenerating lock/explicit files.

- `environment.linux-64.lock.yml`, `environment.win-64.lock.yml`
  - Pinned lockfiles for Linux/Windows.
  - Exact versions/builds resolved at a point in time.

- `explicit-linux-64.txt`, `explicit-win-64.txt`
  - Fully explicit package URLs (no solver).
  - Most reproducible but more brittle if URLs change or go stale.

## Recommended usage

- Start with `environment.base.yml` when you want to add or update deps.
- Use the lockfiles for stable CI or team reproducibility.
- Use explicit specs when you need strict, solver‑free installs.

## Create the env

```bash
conda env create -f env/environment.base.yml
# or:
conda env create -f env/environment.linux-64.lock.yml
# or:
conda create -n chrono --file env/explicit-linux-64.txt
```
