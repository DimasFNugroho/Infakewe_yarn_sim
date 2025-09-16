param([string]$EnvName = "chrono")
$ErrorActionPreference = "Stop"

Write-Host "Exporting from env '$EnvName'..." -ForegroundColor Cyan

# Minimal, portable (edit channels as needed)
$min = conda env export -n $EnvName --from-history | Select-String -NotMatch '^prefix:'
$min | Set-Content env\environment.win-64.yml
Write-Host "✓ env\environment.win-64.yml" -ForegroundColor Green

# Frozen, no builds (more reproducible)
$frozen = conda env export -n $EnvName --no-builds | Select-String -NotMatch '^prefix:'
$frozen | Set-Content env\environment.win-64.lock.yml
Write-Host "✓ env\environment.win-64.lock.yml" -ForegroundColor Green

# Explicit (exact builds)
conda list -n $EnvName --explicit > env\explicit-win-64.txt
Write-Host "✓ env\explicit-win-64.txt" -ForegroundColor Green

Write-Host "Done. Commit these to the repo." -ForegroundColor Cyan
