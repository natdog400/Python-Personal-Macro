param()

$ErrorActionPreference = 'Stop'

function Get-ProjectRoot {
  if ($PSScriptRoot) { return $PSScriptRoot }
  else { return Split-Path -Parent $MyInvocation.MyCommand.Path }
}

function Get-PythonCommand {
  # Try python.exe in PATH
  try {
    $ver = & python --version 2>$null
    if ($LASTEXITCODE -eq 0) { return 'python' }
  } catch {}
  # Try py launcher
  try {
    $ver = & py -3 --version 2>$null
    if ($LASTEXITCODE -eq 0) { return 'py -3' }
  } catch {}
  # Try common install path
  $common = "$env:LOCALAPPDATA\Programs\Python\Python311\python.exe"
  if (Test-Path $common) { return $common }
  Write-Host 'Python not found. Please install Python 3.10+ and ensure it is in PATH.' -ForegroundColor Red
  throw 'Python not found'
}

function Load-ServerConfig($cfgPath) {
  $cfg = @{ bind = '127.0.0.1'; port = 8765; token = 'CHANGE_ME_TOKEN'; mjpeg_fps = 6; backup_retention = 20 }
  if (Test-Path $cfgPath) {
    try {
      $json = Get-Content -Raw -Path $cfgPath | ConvertFrom-Json
      if ($json) { $cfg = $cfg + $json.PSObject.Properties | ForEach-Object { @{ ($_.Name) = $_.Value } } }
    } catch {}
  }
  return $cfg
}

function Prompt-ServerConfig([hashtable]$cfg) {
  Write-Host 'Configure web server settings (press Enter to keep defaults in [brackets])' -ForegroundColor Cyan
  $bind = Read-Host "Bind address [$($cfg.bind)]"; if (-not $bind) { $bind = $cfg.bind }
  $portIn = Read-Host "Port [$($cfg.port)]"; if (-not $portIn) { $portIn = $cfg.port }
  $port = [int]$portIn
  $token = Read-Host "Access token [$($cfg.token)]"; if (-not $token) { $token = $cfg.token }
  $fpsIn = Read-Host "MJPEG FPS [$($cfg.mjpeg_fps)]"; if (-not $fpsIn) { $fpsIn = $cfg.mjpeg_fps }
  $fps = [int]$fpsIn
  $retIn = Read-Host "Backup retention (count) [$($cfg.backup_retention)]"; if (-not $retIn) { $retIn = $cfg.backup_retention }
  $ret = [int]$retIn
  return @{ bind = $bind; port = $port; token = $token; mjpeg_fps = $fps; backup_retention = $ret }
}

function Save-ServerConfig($cfgPath, [hashtable]$cfg) {
  try {
    $json = $cfg | ConvertTo-Json -Depth 4
    Set-Content -Path $cfgPath -Value $json -Encoding UTF8
    Write-Host "Saved server_config.json" -ForegroundColor Green
  } catch {
    Write-Host "Failed to save server_config.json: $($_.Exception.Message)" -ForegroundColor Red
  }
}

function Start-InNewWindow($workDir, $commandLine) {
  # Spawn a new PowerShell window that runs the command and stays open
  $args = @('-NoExit', '-Command', $commandLine)
  Start-Process -FilePath 'powershell' -ArgumentList $args -WorkingDirectory $workDir -WindowStyle Normal
}

# Main
$root = Get-ProjectRoot
Set-Location $root
Write-Host 'Image Detection Bot Launcher' -ForegroundColor Yellow
Write-Host '1) Launch GUI'
Write-Host '2) Launch Web Server'
Write-Host '3) Launch Both'
Write-Host 'X) Exit'
$choice = Read-Host 'Choose option'

if ($choice -match '^[Xx]$') { exit 0 }

$py = Get-PythonCommand

$launchGui = $false
$launchWeb = $false
switch ($choice) {
  '1' { $launchGui = $true }
  '2' { $launchWeb = $true }
  '3' { $launchGui = $true; $launchWeb = $true }
  default { Write-Host 'Invalid choice.' -ForegroundColor Red; exit 1 }
}

if ($launchWeb) {
  $cfgPath = Join-Path $root 'server_config.json'
  $cfg = Load-ServerConfig $cfgPath
  $useLast = Read-Host 'Use last server settings? [Y/n]'
  if ($useLast -and $useLast.ToLower() -eq 'n') {
    $cfg = Prompt-ServerConfig $cfg
    Save-ServerConfig $cfgPath $cfg
  } else {
    Write-Host 'Using existing server_config.json settings.' -ForegroundColor Green
  }
  Start-InNewWindow $root "$py web_server.py"
}

if ($launchGui) {
  Start-InNewWindow $root "$py bot_gui.py"
}

Write-Host 'Launcher tasks started. Close windows to stop processes.' -ForegroundColor Yellow