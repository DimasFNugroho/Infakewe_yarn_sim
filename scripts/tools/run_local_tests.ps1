param(
  [string]$EnvName = "chrono",
  [string]$BaseYml = "env\environment.base.yml"
)

$ErrorActionPreference = "Stop"

# Create env if missing
$exists = (& conda env list) -match "^\s*$EnvName\s"
if (-not $exists) {
  conda config --set channel_priority strict
  conda env create -f $BaseYml
}

# Run pytest inside env (no activation needed)
Write-Host "`n[tests] Running pytest…" -ForegroundColor Cyan
conda run -n $EnvName python -m pytest
Write-Host "[tests] OK" -ForegroundColor Green
