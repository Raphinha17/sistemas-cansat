param(
  [Parameter(Mandatory=$true)][string]$Usb,
  [string]$Mensaje = "backup $(Get-Date -Format 'yyyy-MM-dd HH:mm')"
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
Set-Location $repo
if (-not (Test-Path $Usb)) { throw "Ruta USB no existe: $Usb" }
git add -A
git diff --cached --quiet; if ($LASTEXITCODE -ne 0) { git commit -m $Mensaje }
git remote get-url origin *> $null
if ($LASTEXITCODE -eq 0) { git push } else { Write-Warning "Sin remoto 'origin'; se omite push." }
$dest = Join-Path $Usb (Split-Path -Leaf $repo)
robocopy $repo $dest /MIR /XD venv .venv __pycache__ .pio /XF *.log | Out-Null
if ($LASTEXITCODE -ge 8) { throw "robocopy fallo ($LASTEXITCODE)" }
Write-Host "Backup copiado a $dest"
