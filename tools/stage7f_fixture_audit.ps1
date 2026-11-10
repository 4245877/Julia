param([Parameter(Mandatory=$true)][string]$Label)
$ErrorActionPreference='Stop'
if($Label -notmatch '^[a-zA-Z0-9][a-zA-Z0-9_-]*$') { throw 'Use a simple new label.' }
$project=Split-Path -Parent $PSScriptRoot
$out=Join-Path $project "build/diagnostics/stage7f/$Label"
if(Test-Path -LiteralPath $out) { throw 'Choose a new label.' }
New-Item -ItemType Directory $out | Out-Null
$vswhere=Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
$vs=& $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
Import-Module (Join-Path $vs 'Common7/Tools/Microsoft.VisualStudio.DevShell.dll')
Enter-VsDevShell -VsInstallPath $vs -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64'
Set-Location $project
$sdk=Join-Path $project 'build/dependencies/mujoco-3.3.7'
$ErrorActionPreference='Continue'
& cl /nologo /std:c++20 /EHsc /O2 /W4 /WX "/I$sdk/include" tools/stage7f_static_fixture.cpp "/Fo$out/audit.obj" "/Fe$out/audit.exe" /link "$sdk/lib/mujoco.lib" *> "$out/build.log"
if($LASTEXITCODE -ne 0) { throw 'Independent fixture compile failed.' }
Copy-Item -LiteralPath "$sdk/bin/mujoco.dll" -Destination $out
& "$out/audit.exe" "$out/data" *> "$out/process.log"
if($LASTEXITCODE -ne 0) { throw 'Independent fixture failed.' }
Get-Content "$out/data/static-fixture.txt"
