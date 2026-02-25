#!/usr/bin/env bash
set -euo pipefail

ENV_NAME="${ENV_NAME:-chrono}"
BASE_YML="${BASE_YML:-env/environment.base.yml}"

# Always run from repo root so pytest discovers tests/
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "$REPO_ROOT"

# Create env if missing
if ! conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  conda config --set channel_priority strict
  conda env create -f "$BASE_YML"
fi

# Headless by default; install xvfb if missing (optional)
if ! command -v xvfb-run >/dev/null 2>&1; then
  echo "[info] xvfb not found; if tests need rendering, install it: sudo apt-get install -y xvfb"
fi

echo "[tests] Running pytest…"
conda run -n "$ENV_NAME" python -m pytest
echo "[tests] OK"
