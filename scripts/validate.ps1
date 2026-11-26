param(
    [switch]$SkipClippy
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
Push-Location $repoRoot

function Invoke-CargoCheck {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$CargoArgs
    )

    & cargo @CargoArgs
    if ($LASTEXITCODE -ne 0) {
        throw "cargo $($CargoArgs -join ' ') failed with exit code $LASTEXITCODE"
    }
}

try {
    Write-Host "==> Checking Rust formatting"
    Invoke-CargoCheck -CargoArgs @("fmt", "--all", "--", "--check")

    Write-Host "==> Type-checking all targets (locked dependencies)"
    Invoke-CargoCheck -CargoArgs @("check", "--locked", "--all-targets")

    Write-Host "==> Running all tests (locked dependencies)"
    Invoke-CargoCheck -CargoArgs @("test", "--locked", "--all-targets")

    if (-not $SkipClippy) {
        Write-Host "==> Running Clippy with warnings denied"
        Invoke-CargoCheck -CargoArgs @("clippy", "--locked", "--all-targets", "--", "-D", "warnings")
    }

    Write-Host "Local validation passed."
}
finally {
    Pop-Location
}
