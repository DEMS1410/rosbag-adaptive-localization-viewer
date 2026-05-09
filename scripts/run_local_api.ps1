$ErrorActionPreference = "Stop"

$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$Python = if ($env:PYTHON) { $env:PYTHON } else { "C:\Users\damia\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" }

$env:PYTHONPATH = "$RepoRoot\.vendor;$RepoRoot\src"

Push-Location $RepoRoot
try {
  & $Python -m rosbag_adaptive_localization_viewer.cli serve --host 127.0.0.1 --port 8765
}
finally {
  Pop-Location
}
