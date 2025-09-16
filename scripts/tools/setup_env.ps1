param(
  [string]$EnvName = "chrono",
  [string]$Yml     = "env\environment.win-64.yml",
  [string]$LockYml = "env\environment.win-64.lock.yml",
  [string]$Explicit= "env\explicit-win-64.txt",
  [switch]$UseMamba
)

$ErrorActionPreference = "Stop"

function Exists-Env($name) {
  (conda env list) -match "^\s*$name\s"
}

function Try-Create($cmd, $desc) {
  try {
    Write-Host ">> $desc" -ForegroundColor Cyan
    & $cmd
    return $true
  } catch {
    Write-Warning "Failed: $desc"
    return $false
  }
}

if (Exists-Env $EnvName) {
  Write-Host "Conda env '$EnvName' already exists. Skipping creation." -ForegroundColor Yellow
} else {
  $createCmd = { conda env create -f $Yml }
  if ($UseMamba) { $createCmd = { mamba env create -f $Yml } }

  if (-not (Test-Path $Yml)) { throw "Missing $Yml" }
  if (-not (Try-Create $createCmd "Create from $Yml")) {
    if (Test-Path $LockYml) {
      $lockCmd = { conda env create -f $LockYml }
      if ($UseMamba) { $lockCmd = { mamba env create -f $LockYml } }
      if (-not (Try-Create $lockCmd "Create from $LockYml")) {
        if (Test-Path $Explicit) {
          $expCmd = { conda create -n $EnvName --file $Explicit -y }
          if (-not (Try-Create $expCmd "Create from $Explicit")) {
            throw "All env creation methods failed."
          }
        } else { throw "No lock or explicit spec available." }
      }
    } else {
      throw "No lock file found at $LockYml"
    }
  }
}

Write-Host "Running PyChrono sanity check..." -ForegroundColor Cyan
conda run -n $EnvName python - <<'PY'
import pychrono as chrono
print("pychrono import OK")
print("Has ChSystemNSC:", hasattr(chrono,"ChSystemNSC"))
print("Has ChVisualSystemIrrlicht:", hasattr(__import__("pychrono.irrlicht"), "ChVisualSystemIrrlicht"))
PY

Write-Host "Env ready. To use it:" -ForegroundColor Green
Write-Host "  conda activate $EnvName"
Write-Host "  python scripts\tools\smoke_test.py"
