[CmdletBinding()]
param([switch]$SkipTests, [switch]$UseInstalledDependencies)

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
    & cmake --preset vs-release "-DVCPKG_MANIFEST_INSTALL=$installManifest"
    if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }
    & cmake --build --preset build-release --parallel 4
    if ($LASTEXITCODE -ne 0) { throw 'JULIA build failed.' }
    if (-not $SkipTests) {
        & ctest --preset test-release
        if ($LASTEXITCODE -ne 0) { throw 'JULIA verification failed.' }
    }
    Write-Host 'Ready: julia_desktop\julia_desktop.exe'
}
finally { Pop-Location }
