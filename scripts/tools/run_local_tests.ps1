param(
  [string]$EnvName = "chrono",
  [string]$BaseYml = "env\environment.base.yml",
  [switch]$Reinstall
)

$ErrorActionPreference = "Stop"

# Always run from repo root so pytest discovers tests/ and editable install targets this repo
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path (Join-Path $scriptDir "..\..")).Path
Set-Location $repoRoot

# Create env if missing
$exists = (& conda env list) -match "^\s*$EnvName\s"
if (-not $exists) {
  conda config --set channel_priority strict
  conda env create -n $EnvName -f $BaseYml
}

# Run pytest inside env (no activation needed)
Write-Host "`n[tests] Running pytest…" -ForegroundColor Cyan
$needInstall = [bool]$Reinstall
if (-not $needInstall) {
  conda run -n $EnvName python -c "import chrono_yarn" *> $null
  if ($LASTEXITCODE -ne 0) { $needInstall = $true }
}
if ($needInstall) {
  conda run -n $EnvName python -m pip install -e .
}
conda run -n $EnvName python -m pytest
Write-Host "[tests] OK" -ForegroundColor Green
