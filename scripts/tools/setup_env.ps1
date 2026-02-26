param(
  [string]$EnvName = "chrono",
  [string]$Yml     = "env\environment.base.yml",
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
  $createCmd = { conda env create -n $EnvName -f $Yml }
  if ($UseMamba) { $createCmd = { mamba env create -n $EnvName -f $Yml } }

  if (Test-Path $Yml) {
    if (-not (Try-Create $createCmd "Create from $Yml")) {
      Write-Warning "Failed to create from $Yml"
    }
  } else {
    Write-Warning "Missing $Yml"
  }

  if (Test-Path $LockYml) {
    $lockCmd = { conda env create -n $EnvName -f $LockYml }
    if ($UseMamba) { $lockCmd = { mamba env create -n $EnvName -f $LockYml } }
    if (-not (Try-Create $lockCmd "Create from $LockYml")) {
      Write-Warning "Failed to create from $LockYml"
    }
  }

  if (-not (Exists-Env $EnvName) -and (Test-Path $Explicit)) {
    $expCmd = { conda create -n $EnvName --file $Explicit -y }
    if (-not (Try-Create $expCmd "Create from $Explicit")) {
      throw "All env creation methods failed."
    }
  } elseif (-not (Exists-Env $EnvName)) {
    throw "No env specs found (base, lock, or explicit)."
  }
}

Write-Host "Running PyChrono sanity check..." -ForegroundColor Cyan
$code = @'
import pychrono as chrono
print("pychrono import OK")
print("Has ChSystemNSC:", hasattr(chrono,"ChSystemNSC"))
try:
    import pychrono.irrlicht as irr
    print("Has ChVisualSystemIrrlicht:", hasattr(irr, "ChVisualSystemIrrlicht"))
except Exception as e:
    print("Irrlicht not available:", e)
'@
conda run -n $EnvName python -c $code

Write-Host "Env ready. To use it:" -ForegroundColor Green
Write-Host "  conda activate $EnvName"
Write-Host "  python scripts\tools\smoke_test.py"
