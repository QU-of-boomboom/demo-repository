param(
    [string]$Python = "python",
    [string]$Output = ".cache/cars/demo"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$previousProfile = $env:MISSION_PROFILE
$runExitCode = 1
try {
    $env:MISSION_PROFILE = Join-Path $root "mission_profile_1drone_2cars.json"
    $outputPath = if ([IO.Path]::IsPathRooted($Output)) { $Output } else { Join-Path $root $Output }
    & $Python (Join-Path $root "swarm_cooperation/swarm_cooperation/demo.py") `
        --mode none --rounds 1 --quiet --output $outputPath
    $runExitCode = $LASTEXITCODE
} finally {
    $env:MISSION_PROFILE = $previousProfile
}
exit $runExitCode
