param(
    [Parameter(Mandatory = $true)]
    [string]$ConfigPath,

    [Parameter(Mandatory = $true)]
    [string]$OutputPath
)

$ErrorActionPreference = "Stop"

$workspace = Split-Path -Parent $PSScriptRoot
$python = "C:\Users\damia\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
$env:PYTHONPATH = "$workspace\.vendor;$workspace\src"

& $python -m rosbag_adaptive_localization_viewer.cli build-experiment $ConfigPath $OutputPath
