<#
.SYNOPSIS  Create a new Pedal DevKit project from pedal/_Template.
.EXAMPLE   .\pedal\new-pedal.ps1 MyFuzz
#>
param([Parameter(Mandatory)][string]$Name)

$dest = Join-Path $PSScriptRoot $Name
if (Test-Path $dest) { throw "$dest already exists" }
Copy-Item (Join-Path $PSScriptRoot '_Template') $dest -Recurse

foreach ($f in 'Makefile', 'src\main.cpp') {
    $p = Join-Path $dest $f
    (Get-Content $p -Raw).Replace('@NAME@', $Name) | Set-Content $p -NoNewline
}
Write-Host "Created $dest"
Write-Host "Build:  make -C rbfx/$Name      Flash: make -C rbfx/$Name program-dfu"
