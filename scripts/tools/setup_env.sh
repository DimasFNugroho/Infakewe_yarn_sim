#!/usr/bin/env bash
set -euo pipefail

ENV_NAME="${ENV_NAME:-chrono}"
YML="${YML:-env/environment.base.yml}"
LOCK_YML="${LOCK_YML:-env/environment.linux-64.lock.yml}"
EXPLICIT="${EXPLICIT:-env/explicit-linux-64.txt}"
USE_MAMBA="${USE_MAMBA:-auto}"       # auto|yes|no
RUN_SMOKE="${RUN_SMOKE:-yes}"        # yes|no

have_cmd() { command -v "$1" >/dev/null 2>&1; }

# Pick solver
SOLVER="conda"
if [[ "$USE_MAMBA" == "yes" ]] || { [[ "$USE_MAMBA" == "auto" ]] && have_cmd mamba; }; then
  SOLVER="mamba"
fi

echo ">> Solver: $SOLVER"
echo ">> Target env: $ENV_NAME"

# If env already exists, skip creation
if conda env list | grep -E "^\s*${ENV_NAME}\s" >/dev/null; then
  echo "Env '${ENV_NAME}' already exists. Skipping creation."
else
  if [[ -f "$YML" ]]; then
    echo ">> Creating env from $YML"
    if $SOLVER env create -n "$ENV_NAME" -f "$YML"; then
      :
    else
      echo ">> Failed to create from $YML"
      if conda env list | grep -E "^\s*${ENV_NAME}\s" >/dev/null; then
        echo "ERROR: Partial env '${ENV_NAME}' may exist; remove it before retrying."
        echo "       conda env remove -n \"$ENV_NAME\""
        exit 1
      fi
    fi
  fi

  if ! conda env list | grep -E "^\s*${ENV_NAME}\s" >/dev/null; then
    if [[ -f "$LOCK_YML" ]]; then
      echo ">> Creating env from $LOCK_YML"
      if $SOLVER env create -n "$ENV_NAME" -f "$LOCK_YML"; then
        :
      else
        echo ">> Failed to create from $LOCK_YML"
        if conda env list | grep -E "^\s*${ENV_NAME}\s" >/dev/null; then
          echo "ERROR: Partial env '${ENV_NAME}' may exist; remove it before retrying."
          echo "       conda env remove -n \"$ENV_NAME\""
          exit 1
        fi
      fi
    fi
  fi

  if ! conda env list | grep -E "^\s*${ENV_NAME}\s" >/dev/null; then
    if [[ -f "$EXPLICIT" ]]; then
      echo ">> Creating env from $EXPLICIT"
      conda create -n "$ENV_NAME" --file "$EXPLICIT" -y
    else
      echo "ERROR: No env files found (base, lock, or explicit)."
      exit 1
    fi
  fi
fi

# Sanity check PyChrono
echo ">> PyChrono sanity check"
conda run -n "$ENV_NAME" python - <<'PY'
import pychrono as chrono
print("pychrono import OK")
print("ChSystemNSC:", hasattr(chrono,"ChSystemNSC"))
try:
    import pychrono.irrlicht as irr
    print("Irrlicht OK:", hasattr(irr, "ChVisualSystemIrrlicht"))
except Exception as e:
    print("Irrlicht not available:", e)
PY

# Optional smoke test
if [[ "$RUN_SMOKE" == "yes" ]]; then
  echo ">> Running smoke test"
  # Headless server? Use xvfb-run if DISPLAY is empty and xvfb-run exists.
  if [[ -z "${DISPLAY:-}" ]] && have_cmd xvfb-run; then
    conda run -n "$ENV_NAME" xvfb-run -a python scripts/tools/smoke_test.py
  else
    conda run -n "$ENV_NAME" python scripts/tools/smoke_test.py
  fi
fi

echo "Env ready. Next:"
echo "  conda activate $ENV_NAME"
echo "  python scripts/tutorials/5_object_interactions/5_4_sim_parameters/5_4_sim_param_test.py"
