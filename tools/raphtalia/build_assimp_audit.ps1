param(
    [string]$InputModel = 'assets/models/Raphtalia.fbx',
    [string]$OutputJson = 'docs/raphtalia/assimp_original.json',
    [switch]$SkipBuild
)
$ErrorActionPreference = 'Stop'
$workspace = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$toolBuild = Join-Path $workspace 'build/raphtalia_tools'
$vcpkg = Join-Path $workspace 'build/vs-release/vcpkg_installed/x64-windows'
$compilerSetup = 'C:\Program Files\Microsoft Visual Studio\18\Professional\VC\Auxiliary\Build\vcvars64.bat'
if (-not (Test-Path -LiteralPath $compilerSetup)) { throw "MSVC setup missing: $compilerSetup" }
New-Item -ItemType Directory -Path $toolBuild -Force | Out-Null
$executable = Join-Path $toolBuild 'assimp_audit.exe'
if (-not $SkipBuild) {
    # cmd is only used to initialize MSVC and compile; no filesystem cleanup or moves.
    $source = Join-Path $PSScriptRoot 'assimp_audit.cpp'
    $batch = Join-Path $toolBuild 'compile_assimp_audit.cmd'
    $compile = @"
@echo off
call "$compilerSetup" >nul
if errorlevel 1 exit /b %errorlevel%
cl /nologo /std:c++20 /EHsc /O2 /utf-8 /MD /D NOMINMAX /I "$vcpkg\include" "$source" /Fo"$toolBuild\assimp_audit.obj" /Fe"$executable" /link /LIBPATH:"$vcpkg\lib" assimp-vc143-mt.lib
exit /b %errorlevel%
"@
    Set-Content -LiteralPath $batch -Value $compile -Encoding ASCII
    & $env:ComSpec /d /c $batch
    if ($LASTEXITCODE -ne 0) { throw "Assimp audit compilation failed: $LASTEXITCODE" }
}
$inputAbsolute = if ([System.IO.Path]::IsPathRooted($InputModel)) { $InputModel } else { Join-Path $workspace $InputModel }
$outputAbsolute = if ([System.IO.Path]::IsPathRooted($OutputJson)) { $OutputJson } else { Join-Path $workspace $OutputJson }
$previousPath = $env:PATH
try {
    $env:PATH = "$vcpkg\bin;$previousPath"
    & $executable $inputAbsolute $outputAbsolute
    if ($LASTEXITCODE -ne 0) { throw "Assimp audit failed: $LASTEXITCODE" }
} finally { $env:PATH = $previousPath }
