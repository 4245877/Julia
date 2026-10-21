param(
    [string]$InputModel = 'assets/models/Raphtalia_prepared.fbx',
    [string]$OutputDirectory = 'build/raphtalia_tools/gpu_prepared',
    [string]$BlinkSamplesCsv = '',
    [switch]$SkipBuild,
    [switch]$BuildOnly
)
$ErrorActionPreference = 'Stop'
$workspace = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$toolBuild = Join-Path $workspace 'build/raphtalia_tools'
$releaseBuild = Join-Path $workspace 'build/vs-release'
$vcpkg = Join-Path $releaseBuild 'vcpkg_installed/x64-windows'
$compilerSetup = 'C:\Program Files\Microsoft Visual Studio\18\Professional\VC\Auxiliary\Build\vcvars64.bat'
New-Item -ItemType Directory -Path $toolBuild -Force | Out-Null
$executable = Join-Path $toolBuild 'gpu_pose_audit.exe'
if (-not $SkipBuild) {
    $assimpLibraries = @(Get-ChildItem -LiteralPath (Join-Path $vcpkg 'lib') -Filter 'assimp-*-mt.lib' -File)
    if ($assimpLibraries.Count -ne 1) { throw "Expected one Release Assimp library in $vcpkg/lib" }
    $assimpLibrary = $assimpLibraries[0].FullName
    $source = Join-Path $PSScriptRoot 'gpu_pose_audit.cpp'
    $batch = Join-Path $toolBuild 'compile_gpu_pose_audit.cmd'
    $compile = @"
@echo off
call "$compilerSetup" >nul
if errorlevel 1 exit /b %errorlevel%
cl /nologo /std:c++20 /EHsc /O2 /utf-8 /MD /D NOMINMAX /D GLFW_DLL /D FMT_SHARED /D SPDLOG_COMPILED_LIB /D SPDLOG_FMT_EXTERNAL /D SPDLOG_SHARED_LIB /I "$workspace\plugins\render_opengl\include" /I "$workspace\libs\render\include" /I "$workspace\libs\core\include" /I "$vcpkg\include" "$source" /Fo"$toolBuild\gpu_pose_audit.obj" /Fe"$executable" /link "$releaseBuild\plugins\render_opengl\julia_render_opengl.lib" "$releaseBuild\libs\render\julia_render.lib" "$releaseBuild\libs\core\julia_core.lib" /LIBPATH:"$vcpkg\lib" "$assimpLibrary" glad.lib glfw3dll.lib spdlog.lib fmt.lib opengl32.lib
exit /b %errorlevel%
"@
    Set-Content -LiteralPath $batch -Value $compile -Encoding ASCII
    & $env:ComSpec /d /c $batch
    if ($LASTEXITCODE -ne 0) { throw "GPU audit compilation failed: $LASTEXITCODE" }
}
if ($BuildOnly) { return }
$inputAbsolute = if ([System.IO.Path]::IsPathRooted($InputModel)) { $InputModel } else { Join-Path $workspace $InputModel }
$outputAbsolute = if ([System.IO.Path]::IsPathRooted($OutputDirectory)) { $OutputDirectory } else { Join-Path $workspace $OutputDirectory }
$assets = Join-Path $workspace 'assets'
$previousPath = $env:PATH
$inputBefore = (Get-FileHash -LiteralPath $inputAbsolute -Algorithm SHA256).Hash.ToLowerInvariant()
try {
    $env:PATH = "$vcpkg\bin;$previousPath"
    if ($BlinkSamplesCsv) {
        $samplesAbsolute = if ([System.IO.Path]::IsPathRooted($BlinkSamplesCsv)) { $BlinkSamplesCsv } else { Join-Path $workspace $BlinkSamplesCsv }
        & $executable $inputAbsolute $assets $outputAbsolute $samplesAbsolute
    } else { & $executable $inputAbsolute $assets $outputAbsolute }
    if ($LASTEXITCODE -ne 0) { throw "GPU audit failed: $LASTEXITCODE" }
    $inputAfter = (Get-FileHash -LiteralPath $inputAbsolute -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($inputBefore -ne $inputAfter) { throw 'Input model changed during GPU audit' }
    $provenance = "input_sha256=$inputBefore`ninput_stable_during_audit=YES`n"
    [System.IO.File]::AppendAllText((Join-Path $outputAbsolute 'gpu_audit.txt'), $provenance, (New-Object System.Text.UTF8Encoding $false))
} finally { $env:PATH = $previousPath }
