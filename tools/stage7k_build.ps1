param([Parameter(Mandatory=$true)][string]$Label,[switch]$Tests,[string]$Regex,[switch]$Audit,[switch]$CpuOnly,[switch]$DesktopOnly,[switch]$OffOnly)
$ErrorActionPreference='Stop'
$project=Split-Path -Parent $PSScriptRoot
if($Label -notmatch '^[a-zA-Z0-9][a-zA-Z0-9_-]*$') { throw 'Use a simple new label.' }
$destination=Join-Path $project "build/diagnostics/stage7k/$Label"
if(Test-Path -LiteralPath $destination) { throw 'Destination exists.' }
$vswhere=Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
$vs=& $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
Import-Module (Join-Path $vs 'Common7/Tools/Microsoft.VisualStudio.DevShell.dll')
Enter-VsDevShell -VsInstallPath $vs -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64'
if(-not $env:VCPKG_ROOT) { $env:VCPKG_ROOT=Join-Path $vs 'VC/vcpkg' }
Set-Location $project
New-Item -ItemType Directory $destination | Out-Null
$ErrorActionPreference='Continue'
$variants=if($CpuOnly){@('cpu-release')}elseif($DesktopOnly){@('vs-release')}elseif($OffOnly){@('cpu-no-physics')}else{@('vs-release','cpu-release','cpu-no-physics')}
foreach($variant in $variants) {
    if($Audit) {
        & cmake --build "build/$variant" -- -n *> "$destination/$variant-dry-run.txt"
    } elseif(-not $Tests) {
        $preset=if($variant -eq 'cpu-no-physics'){'cpu-release'}else{$variant}
        $options=@('--preset',$preset,'-DJULIA_WARNINGS_AS_ERRORS=ON','-DVCPKG_MANIFEST_INSTALL=OFF')
        if($variant -eq 'cpu-no-physics') { $options+=@('-B','build/cpu-no-physics','-DJULIA_BUILD_PHYSICS=OFF') }
        & cmake @options *> "$destination/$variant-configure.log"
        if($LASTEXITCODE -ne 0){throw "Configure $variant failed"}
        & cmake --build "build/$variant" --parallel 4 *> "$destination/$variant-build.log"
    }
    if($LASTEXITCODE -ne 0){throw "Build/audit $variant failed"}
    if($Tests) {
        $arguments=@('tools/stage7d_ctest.py','--build',"build/$variant",'--output',"$destination/$variant")
        if($Regex) { $arguments+=@('--regex',$Regex) }
        & python @arguments *> "$destination/$variant-test.log"
        if($LASTEXITCODE -ne 0){throw "Tests $variant failed"}
    }
}

