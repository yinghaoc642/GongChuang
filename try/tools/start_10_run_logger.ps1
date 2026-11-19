param(
    [string]$Port = "auto"
)

$workspaceRoot = Split-Path -Parent $PSScriptRoot
$platformioPython = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"
$logger = Join-Path $PSScriptRoot "continuous_experiment_logger.py"

if (-not (Test-Path -LiteralPath $platformioPython)) {
    throw "PlatformIO Python not found: $platformioPython"
}

& $platformioPython $logger --port $Port --baud 115200 --runs 10
exit $LASTEXITCODE
