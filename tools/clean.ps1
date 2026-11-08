[CmdletBinding(SupportsShouldProcess)]
param(
    [ValidateSet('All', 'Release', 'Debug')][string]$Configuration = 'All',
    [switch]$VerificationOnly
)

$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$buildRoot = [IO.Path]::GetFullPath((Join-Path $projectRoot 'build'))
$buildPrefix = $buildRoot.TrimEnd('\') + '\'
$configurations = if ($Configuration -eq 'All') { @('release', 'debug') } else { @($Configuration.ToLowerInvariant()) }
$targets = @($configurations | ForEach-Object { Join-Path $buildRoot "vs-$_/verification" })
if (-not $VerificationOnly) {
    $targets += @('diagnostics', 'packages', 'anime-audit') | ForEach-Object { Join-Path $buildRoot $_ }
}

foreach ($target in $targets) {
    $absolute = [IO.Path]::GetFullPath($target)
    if (-not $absolute.StartsWith($buildPrefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Cleanup target is outside build: $absolute"
    }
    if (-not (Test-Path -LiteralPath $absolute)) { continue }
    # Refuse redirected parents and descendants before any recursive deletion.
    $ancestor = Get-Item -LiteralPath $absolute -Force
    while ($null -ne $ancestor -and $ancestor.FullName.Length -ge $projectRoot.Length) {
        if (($ancestor.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
            throw "Cleanup refuses reparse points: $($ancestor.FullName)"
        }
        $ancestor = $ancestor.Parent
    }
    $redirected = Get-ChildItem -LiteralPath $absolute -Recurse -Force |
        Where-Object { ($_.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0 } |
        Select-Object -First 1
    if ($redirected) { throw "Cleanup refuses reparse points: $($redirected.FullName)" }
    if ($PSCmdlet.ShouldProcess($absolute, 'Remove generated diagnostics')) {
        Remove-Item -LiteralPath $absolute -Recurse -Force
    }
}
