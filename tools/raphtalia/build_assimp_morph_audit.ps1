param(
    [string]$InputModel = 'assets/models/Raphtalia_prepared.fbx',
    [string]$OutputJson = 'build/raphtalia_tools/blink/morph_audit.json',
    [switch]$SkipBuild
)
$ErrorActionPreference = 'Stop'
$workspace = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$toolBuild = Join-Path $workspace 'build/raphtalia_tools'
$vcpkg = Join-Path $workspace 'build/vs-release/vcpkg_installed/x64-windows'
$compilerSetup = 'C:\Program Files\Microsoft Visual Studio\18\Professional\VC\Auxiliary\Build\vcvars64.bat'
$executable = Join-Path $toolBuild 'assimp_morph_audit.exe'
New-Item -ItemType Directory -Path $toolBuild -Force | Out-Null
if (-not $SkipBuild) {
    $assimpLibraries = @(Get-ChildItem -LiteralPath (Join-Path $vcpkg 'lib') -Filter 'assimp-*-mt.lib' -File)
    if ($assimpLibraries.Count -ne 1) { throw 'Expected exactly one Release Assimp library' }
    $source = Join-Path $PSScriptRoot 'assimp_morph_audit.cpp'
    $batch = Join-Path $toolBuild 'compile_assimp_morph_audit.cmd'
    $compile = @"
@echo off
call "$compilerSetup" >nul
if errorlevel 1 exit /b %errorlevel%
cl /nologo /std:c++20 /EHsc /O2 /utf-8 /MD /D NOMINMAX /I "$vcpkg\include" "$source" /Fo"$toolBuild\assimp_morph_audit.obj" /Fe"$executable" /link "$($assimpLibraries[0].FullName)"
exit /b %errorlevel%
"@
    Set-Content -LiteralPath $batch -Value $compile -Encoding ASCII
    & $env:ComSpec /d /c $batch
    if ($LASTEXITCODE -ne 0) { throw "Morph audit compilation failed: $LASTEXITCODE" }
}
$inputAbsolute = if ([System.IO.Path]::IsPathRooted($InputModel)) { [System.IO.Path]::GetFullPath($InputModel) } else { [System.IO.Path]::GetFullPath((Join-Path $workspace $InputModel)) }
$outputAbsolute = if ([System.IO.Path]::IsPathRooted($OutputJson)) { [System.IO.Path]::GetFullPath($OutputJson) } else { [System.IO.Path]::GetFullPath((Join-Path $workspace $OutputJson)) }
if ($inputAbsolute -eq $outputAbsolute) { throw 'Output must differ from input' }
$previousPath = $env:PATH
$inputBefore = (Get-FileHash -LiteralPath $inputAbsolute -Algorithm SHA256).Hash.ToLowerInvariant()
try {
    $env:PATH = "$vcpkg\bin;$previousPath"
    & $executable $inputAbsolute $outputAbsolute
    if ($LASTEXITCODE -ne 0) { throw "Morph audit failed: $LASTEXITCODE" }
    $inputAfter = (Get-FileHash -LiteralPath $inputAbsolute -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($inputBefore -ne $inputAfter) { throw 'Input changed during morph audit' }
    $result = Get-Content -LiteralPath $outputAbsolute -Raw -Encoding UTF8 | ConvertFrom-Json
    $result | Add-Member -NotePropertyName input_sha256 -NotePropertyValue $inputBefore
    $result | Add-Member -NotePropertyName input_stable_during_audit -NotePropertyValue $true
    [System.IO.File]::WriteAllText($outputAbsolute, ($result | ConvertTo-Json -Depth 100), (New-Object System.Text.UTF8Encoding $false))
    if (-not $result.source_validation.loaded -or -not $result.project_import.loaded) { throw 'One or both import modes failed; see output' }
} finally { $env:PATH = $previousPath }
