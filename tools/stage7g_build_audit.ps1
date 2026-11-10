param([Parameter(Mandatory=$true)][string]$Label)
$ErrorActionPreference='Stop'
$project=Split-Path -Parent $PSScriptRoot
if($Label -notmatch '^[a-zA-Z0-9][a-zA-Z0-9_-]*$') { throw 'Use a new simple label.' }
$out=Join-Path $project "build/diagnostics/stage7g/$Label"
if(Test-Path -LiteralPath $out) { throw 'Destination exists.' }
New-Item -ItemType Directory $out | Out-Null
$vswhere=Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
$vs=& $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
Import-Module (Join-Path $vs 'Common7/Tools/Microsoft.VisualStudio.DevShell.dll')
Enter-VsDevShell -VsInstallPath $vs -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64'
Set-Location $project
$ErrorActionPreference='Continue'
foreach($variant in @('vs-release','cpu-release','cpu-no-physics')) {
    & cmake --build "build/$variant" -- -n *> "$out/$variant-dry-run.txt"
    if($LASTEXITCODE -ne 0) { throw 'Read-only build audit failed.' }
    Get-Content "build/$variant/CMakeCache.txt" | Select-String 'CMAKE_BUILD_TYPE:|JULIA_BUILD_PHYSICS:|JULIA_BUILD_DESKTOP:|JULIA_WARNINGS_AS_ERRORS:' | Set-Content "$out/$variant-settings.txt"
}
$headless=Join-Path $project 'build/cpu-release/apps/julia_body_physics'
$files=@(Get-Item -LiteralPath "$headless/julia_body_physics.exe") + @(Get-ChildItem -LiteralPath $headless -Filter '*.dll')
foreach($file in $files) { & dumpbin /nologo /dependents $file.FullName >> "$out/headless-dependencies.txt" }
if(Select-String -Path "$out/headless-dependencies.txt" -Pattern 'opengl|glfw|glad' -Quiet) { throw 'Unexpected GPU dependency.' }
