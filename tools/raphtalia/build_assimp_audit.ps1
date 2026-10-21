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
    $assimpLibraries = @(Get-ChildItem -LiteralPath (Join-Path $vcpkg 'lib') -Filter 'assimp-*-mt.lib' -File)
    if ($assimpLibraries.Count -ne 1) { throw "Expected one Release Assimp library in $vcpkg/lib" }
    $assimpLibrary = $assimpLibraries[0].FullName
    # cmd is only used to initialize MSVC and compile; no filesystem cleanup or moves.
    $source = Join-Path $PSScriptRoot 'assimp_audit.cpp'
    $batch = Join-Path $toolBuild 'compile_assimp_audit.cmd'
    $compile = @"
@echo off
call "$compilerSetup" >nul
if errorlevel 1 exit /b %errorlevel%
cl /nologo /std:c++20 /EHsc /O2 /utf-8 /MD /D NOMINMAX /I "$vcpkg\include" "$source" /Fo"$toolBuild\assimp_audit.obj" /Fe"$executable" /link /LIBPATH:"$vcpkg\lib" "$assimpLibrary"
exit /b %errorlevel%
"@
    Set-Content -LiteralPath $batch -Value $compile -Encoding ASCII
    & $env:ComSpec /d /c $batch
    if ($LASTEXITCODE -ne 0) { throw "Assimp audit compilation failed: $LASTEXITCODE" }
}
$inputAbsolute = if ([System.IO.Path]::IsPathRooted($InputModel)) { $InputModel } else { Join-Path $workspace $InputModel }
$outputAbsolute = if ([System.IO.Path]::IsPathRooted($OutputJson)) { $OutputJson } else { Join-Path $workspace $OutputJson }
$previousPath = $env:PATH
$inputBefore = (Get-FileHash -LiteralPath $inputAbsolute -Algorithm SHA256).Hash.ToLowerInvariant()
try {
    $env:PATH = "$vcpkg\bin;$previousPath"
    & $executable $inputAbsolute $outputAbsolute
    if ($LASTEXITCODE -ne 0) { throw "Assimp audit failed: $LASTEXITCODE" }
    $inputAfter = (Get-FileHash -LiteralPath $inputAbsolute -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($inputBefore -ne $inputAfter) { throw 'Input model changed during Assimp audit' }
    $auditResult = Get-Content -LiteralPath $outputAbsolute -Raw | ConvertFrom-Json
    $auditResult | Add-Member -NotePropertyName 'input_sha256' -NotePropertyValue $inputBefore -Force
    $auditResult | Add-Member -NotePropertyName 'input_stable_during_audit' -NotePropertyValue $true -Force
    [System.IO.File]::WriteAllText($outputAbsolute, ($auditResult | ConvertTo-Json -Depth 100), (New-Object System.Text.UTF8Encoding $false))
} finally { $env:PATH = $previousPath }
