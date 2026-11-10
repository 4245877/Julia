param([string]$Label='baseline',[switch]$Tests,[switch]$CpuOnly,[switch]$DesktopOnly)
$ErrorActionPreference='Stop'
$project=Split-Path -Parent $PSScriptRoot
if($Label -notmatch '^[a-zA-Z0-9][a-zA-Z0-9_-]*$') { throw 'Use a simple diagnostic label (letters, digits, dash, underscore).' }
if($CpuOnly -and $DesktopOnly) { throw 'Choose only one of CpuOnly/DesktopOnly.' }
$destination=Join-Path $project "build/diagnostics/stage7f/$Label"
if(Test-Path -LiteralPath $destination) { throw "Diagnostic destination already exists: $destination. Choose a new -Label." }
$vswhere=Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
$vs=& $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
Import-Module (Join-Path $vs 'Common7/Tools/Microsoft.VisualStudio.DevShell.dll')
Enter-VsDevShell -VsInstallPath $vs -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64'
if(-not $env:VCPKG_ROOT) { $env:VCPKG_ROOT=Join-Path $vs 'VC/vcpkg' }
Set-Location $project
$dest="build/diagnostics/stage7f/$Label"
New-Item -ItemType Directory -Force $dest | Out-Null
$ErrorActionPreference='Continue'
$variants=if($CpuOnly){@('cpu-release')}elseif($DesktopOnly){@('vs-release')}else{@('vs-release','cpu-release','cpu-no-physics')}
foreach($variant in $variants) {
    $preset=if($variant -eq 'cpu-no-physics'){'cpu-release'}else{$variant}
    $options=@('--preset',$preset,'-DJULIA_WARNINGS_AS_ERRORS=ON','-DVCPKG_MANIFEST_INSTALL=OFF')
    if($variant -eq 'cpu-no-physics') { $options+=@('-B','build/cpu-no-physics','-DJULIA_BUILD_PHYSICS=OFF') }
    & cmake @options *> "$dest/$variant-configure.log"
    if($LASTEXITCODE -ne 0){throw "Configure $variant failed"}
    & cmake --build "build/$variant" --parallel 4 *> "$dest/$variant-build.log"
    if($LASTEXITCODE -ne 0){throw "Build $variant failed"}
    if($Tests) {
        & python tools/stage7d_ctest.py --build "build/$variant" --output "$dest/$variant" *> "$dest/$variant-test.log"
        if($LASTEXITCODE -ne 0){throw "Tests $variant failed"}
    }
}
