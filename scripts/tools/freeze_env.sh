#!/usr/bin/env bash
set -euo pipefail
ENV_NAME="${1:-chrono}"

mkdir -p env

# Minimal (portable)
conda env export -n "$ENV_NAME" --from-history | sed '/^prefix:/d' > env/environment.linux-64.yml
echo "✓ env/environment.linux-64.yml"

# Frozen (no build strings)
conda env export -n "$ENV_NAME" --no-builds | sed '/^prefix:/d' > env/environment.linux-64.lock.yml
echo "✓ env/environment.linux-64.lock.yml"

# Explicit (exact builds)
conda list -n "$ENV_NAME" --explicit > env/explicit-linux-64.txt
echo "✓ env/explicit-linux-64.txt"
