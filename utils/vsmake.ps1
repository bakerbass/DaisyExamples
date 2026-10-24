# Runs make in the nearest ancestor directory of the given path that has a Makefile.
# Used by .vscode/tasks.json so tasks work from src/main.cpp, not just the project root.
param([Parameter(Mandatory)][string]$Path, [Parameter(ValueFromRemainingArguments)]$MakeArgs)

$dir = if (Test-Path $Path -PathType Leaf) { Split-Path $Path } else { $Path }
while ($dir -and -not (Test-Path (Join-Path $dir 'Makefile'))) { $dir = Split-Path $dir }
if (-not $dir) { Write-Error "No Makefile found above $Path"; exit 1 }

Write-Host "make -C $dir $MakeArgs"
make -C $dir @MakeArgs
exit $LASTEXITCODE
