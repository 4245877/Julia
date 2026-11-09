[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$version = '3.3.7'
$expectedHash = '1108AD01F6A1E63D42725188404C9AF50B81902C368EF026D20D5148C115A08B'
$directory = Join-Path $projectRoot 'build/dependencies'
$archive = Join-Path $directory "mujoco-$version-windows-x86_64.zip"
$destination = Join-Path $directory "mujoco-$version"
New-Item -ItemType Directory -Force $directory | Out-Null
if (-not (Test-Path -LiteralPath $archive)) {
    & curl.exe --fail --location --retry 2 "https://github.com/google-deepmind/mujoco/releases/download/$version/mujoco-$version-windows-x86_64.zip" --output $archive
    if ($LASTEXITCODE -ne 0) { throw 'MuJoCo download failed' }
}
if ((Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash -ne $expectedHash) {
    throw 'MuJoCo SDK SHA-256 mismatch; archive was not extracted'
}
Expand-Archive -LiteralPath $archive -DestinationPath $destination -Force
Write-Host "MuJoCo $version installed: $destination"
