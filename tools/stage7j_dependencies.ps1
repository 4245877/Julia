param([Parameter(Mandatory=$true)][string]$Output)
$ErrorActionPreference='Stop'
if(Test-Path -LiteralPath $Output){throw 'Destination exists'}
New-Item -ItemType Directory -Path $Output | Out-Null
$vswhere=Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
$vs=& $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
Import-Module (Join-Path $vs 'Common7/Tools/Microsoft.VisualStudio.DevShell.dll')
Enter-VsDevShell -VsInstallPath $vs -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64'
@{processor=$env:PROCESSOR_IDENTIFIER;logical_processors=$env:NUMBER_OF_PROCESSORS;os=[Environment]::OSVersion.ToString();method='Environment and registry fallback: CIM access denied in initial diagnostic attempt'} | ConvertTo-Json | Set-Content -Encoding UTF8 "$Output/host.json"
Get-ItemProperty -LiteralPath 'HKLM:\HARDWARE\DESCRIPTION\System\CentralProcessor\0' -Name ProcessorNameString | Select-Object ProcessorNameString | ConvertTo-Json | Set-Content -Encoding UTF8 "$Output/processor.json"
$ErrorActionPreference='Continue'
& cl /Bv 2>&1 | Out-File -Encoding UTF8 "$Output/compiler.txt"
$ErrorActionPreference='Stop'
& dumpbin /dependents build/cpu-release/apps/julia_body_physics/julia_body_physics.exe | Out-File -Encoding UTF8 "$Output/headless-dependencies.txt"
& dumpbin /dependents build/dependencies/mujoco-3.3.7/bin/mujoco.dll | Out-File -Encoding UTF8 "$Output/mujoco-dependencies.txt"
