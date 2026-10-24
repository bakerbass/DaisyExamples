<#
.SYNOPSIS  Build the host (PC) render tool: test/bin/fdn_render.exe
.NOTES     Needs a native g++ (MSYS2 mingw64). The arm-none-eabi g++ on PATH
           is NOT usable here, so this script looks for a native one.
#>
$ErrorActionPreference = 'Stop'

$candidates = @('C:\msys64\mingw64\bin\g++.exe', 'C:\msys64\ucrt64\bin\g++.exe') +
    @((Get-Command g++ -All -ErrorAction SilentlyContinue).Source)
$gpp = $candidates | Where-Object { $_ -and (Test-Path $_) } | Where-Object {
    -not ((& $_ -dumpmachine) -match '^arm') } | Select-Object -First 1
if (-not $gpp) { throw 'No native g++ found (install MSYS2 mingw64: pacman -S mingw-w64-x86_64-gcc)' }

$bin = Join-Path $PSScriptRoot 'bin'
New-Item -ItemType Directory -Force $bin | Out-Null

# The mingw runtime DLLs must be findable when the exe runs.
$env:PATH = (Split-Path $gpp) + ';' + $env:PATH

# -ffp-contract=off: no fused multiply-add, so results are reproducible
# bit-for-bit across builds (and closer to the Cortex-M7 build).
& $gpp -std=c++14 -O2 -ffp-contract=off -Wall -Wextra `
    (Join-Path $PSScriptRoot 'render.cpp') -o (Join-Path $bin 'fdn_render.exe') `
    -static-libgcc -static-libstdc++
if ($LASTEXITCODE -ne 0) { throw 'build failed' }
Write-Host "built $bin\fdn_render.exe  (using $gpp)"
