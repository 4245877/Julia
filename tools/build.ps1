[CmdletBinding()]
param([switch]$SkipTests, [switch]$UseInstalledDependencies,
    [switch]$Clean, [switch]$KeepDiagnostics,
    [ValidateSet('Release', 'Debug')][string]$Configuration = 'Release')

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
if (-not (Test-Path -LiteralPath $vswhere)) {
    throw 'Visual Studio C++ tools are required (vswhere.exe was not found).'
}
$visualStudioPath = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $visualStudioPath) { throw 'Install the Visual Studio Desktop development with C++ workload.' }
Import-Module (Join-Path $visualStudioPath 'Common7/Tools/Microsoft.VisualStudio.DevShell.dll')
Enter-VsDevShell -VsInstallPath $visualStudioPath -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64'
if (-not $env:VCPKG_ROOT) { $env:VCPKG_ROOT = Join-Path $visualStudioPath 'VC/vcpkg' }

Push-Location $projectRoot
try {
    # Windows PowerShell wraps native stderr as ErrorRecord when redirected;
    # warnings must not abort a successful CMake invocation. Check exit codes.
    $ErrorActionPreference = 'Continue'
    $installManifest = if ($UseInstalledDependencies) { 'OFF' } else { 'ON' }
    $configurationName = $Configuration.ToLowerInvariant()
    $configureOptions = @()
    $buildOptions = @()
    if ($Clean) {
        $configureOptions += '--fresh'
        $buildOptions += '--clean-first'
    }
    & cmake --preset "vs-$configurationName" "-DVCPKG_MANIFEST_INSTALL=$installManifest" @configureOptions
    if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }
    & cmake --build --preset "build-$configurationName" --parallel 4 @buildOptions
    if ($LASTEXITCODE -ne 0) { throw 'JULIA build failed.' }
    if (-not $SkipTests) {
        & (Join-Path $PSScriptRoot 'clean.ps1') -Configuration $Configuration -VerificationOnly
        & ctest --preset "test-$configurationName"
        if ($LASTEXITCODE -ne 0) { throw 'JULIA verification failed.' }
        if (-not $KeepDiagnostics) {
            & (Join-Path $PSScriptRoot 'clean.ps1') -Configuration $Configuration -VerificationOnly
        }
    }
    Write-Host 'Ready: julia_desktop\julia_desktop.exe'
}
finally { Pop-Location }
