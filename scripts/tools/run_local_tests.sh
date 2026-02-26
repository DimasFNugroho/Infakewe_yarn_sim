#!/usr/bin/env bash
set -euo pipefail

ENV_NAME="${ENV_NAME:-chrono}"
BASE_YML="${BASE_YML:-env/environment.base.yml}"
REINSTALL="${REINSTALL:-0}"

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
if ! conda run -n "$ENV_NAME" python -m pip --version >/dev/null 2>&1; then
  echo "[error] pip is not installed in env '$ENV_NAME'."
  echo "        Create the env from env/environment.base.yml or install pip:"
  echo "        conda install -n \"$ENV_NAME\" pip"
  exit 1
fi
if [[ "$REINSTALL" == "1" ]]; then
  conda run -n "$ENV_NAME" python -m pip install -e .
elif ! conda run -n "$ENV_NAME" python - <<'PY' >/dev/null 2>&1; then
import chrono_yarn
print(chrono_yarn.__file__)
PY
  conda run -n "$ENV_NAME" python -m pip install -e .
fi
conda run -n "$ENV_NAME" python -m pytest
echo "[tests] OK"
