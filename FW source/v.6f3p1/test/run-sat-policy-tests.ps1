<#
  run-sat-policy-tests.ps1 - build sat_policy.c for the host, run the
  exhaustive test against it, then prove the test can fail.

  The test only counts if it can tell a broken policy from a good one. So
  after the real run, this script builds deliberately broken copies of
  sat_policy.c (mutants), one per rule, and runs the same test against each.
  Every mutant must make the test FAIL. A mutant the test lets through is a
  rule the test does not actually check.

  Nothing here touches hardware or the firmware build.

  Usage:
    pwsh -NoProfile -File run-sat-policy-tests.ps1 [-Cc <path to gcc or clang>]
#>
[CmdletBinding()]
param(
    [string]$Cc = 'gcc'
)
$ErrorActionPreference = 'Stop'
$here   = $PSScriptRoot
$fwDir  = Split-Path $here -Parent
$policy = Join-Path $fwDir 'sat_policy.c'
$header = Join-Path $fwDir 'sat_policy.h'
$test   = Join-Path $here 'sat_policy_test.c'

$ccPath = (Get-Command $Cc -ErrorAction Stop).Source
$ccDir  = Split-Path $ccPath -Parent
$env:PATH = $ccDir + [IO.Path]::PathSeparator + $env:PATH     # the compiler's own DLLs
$ccVer  = (& $ccPath --version | Select-Object -First 1)

$work = Join-Path ([IO.Path]::GetTempPath()) ("sat-policy-test-" + [guid]::NewGuid().ToString('N').Substring(0, 8))
New-Item -ItemType Directory -Path $work | Out-Null
$exe = if ($IsWindows -or $env:OS -eq 'Windows_NT') { '.exe' } else { '' }
$cflags = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Wpedantic', '-Wconversion', '-Wshadow', '-Werror')

Write-Host "=== SAT policy: exhaustive host test + mutants ===" -ForegroundColor Cyan
Write-Host ("compiler: {0}" -f $ccPath)
Write-Host ("          {0}" -f $ccVer)
Write-Host ("subject : {0}" -f $policy)
Write-Host ("sha256  : {0}" -f (Get-FileHash $policy -Algorithm SHA256).Hash.ToLower())
Write-Host ("header  : {0}" -f (Get-FileHash $header -Algorithm SHA256).Hash.ToLower())
Write-Host ("test    : {0}" -f (Get-FileHash $test -Algorithm SHA256).Hash.ToLower())

# Mutants are built without -Werror: a mutation often leaves a variable
# unused or a comparison always false, which is the point of it.
function Invoke-Build([string]$srcDir, [string]$out, [switch]$Mutant) {
    $flags = if ($Mutant) { $cflags | Where-Object { $_ -ne '-Werror' } } else { $cflags }
    $ccArgs = $flags + @('-I', $srcDir, (Join-Path $srcDir 'sat_policy.c'), $test, '-o', $out)
    $log = & $ccPath @ccArgs 2>&1
    return @{ Ok = ($LASTEXITCODE -eq 0); Log = ($log | Out-String) }
}

# ---- 1. the real policy ------------------------------------------------------
$realDir = Join-Path $work 'real'
New-Item -ItemType Directory -Path $realDir | Out-Null
Copy-Item $policy, $header $realDir
$b = Invoke-Build $realDir (Join-Path $realDir "t$exe")
if (-not $b.Ok) {
    Write-Host "COULD NOT RUN: the test did not build against the real policy." -ForegroundColor Red
    Write-Host $b.Log
    exit 2
}
Write-Host "`n--- real policy ---" -ForegroundColor Yellow
& (Join-Path $realDir "t$exe")
$realExit = $LASTEXITCODE
if ($realExit -ne 0) {
    Write-Host "`nRESULT: the real policy FAILS its test." -ForegroundColor Red
    exit 1
}

# ---- 2. mutants ----------------------------------------------------------------
# Each is one textual change to sat_policy.c. `Find` must occur in the file
# (every occurrence is replaced), or the mutant is reported as not applied,
# which counts as a failure of this script, never as a kill.
$mutants = @(
    @{ Name = 'allow write opcode 0x30 (WRITE SECTORS)'
       Find = '    case ATA_READ_SECTORS_NR:'
       Repl = "    case ATA_READ_SECTORS_NR:`n    case 0x30:" }
    @{ Name = 'allow DMA opcode 0x25 (READ DMA EXT)'
       Find = '    case ATA_READ_SECTORS_EXT:'
       Repl = "    case ATA_READ_SECTORS_EXT:`n    case 0x25:" }
    @{ Name = 'drop the 8-sector count cap'
       Find = 'count > SAT_MAX_SECTORS'
       Repl = 'count > 255' }
    @{ Name = 'widen the count cap to 9'
       Find = 'count > SAT_MAX_SECTORS'
       Repl = 'count > SAT_MAX_SECTORS + 1' }
    @{ Name = 'accept T_DIR=0 (to device)'
       Find = 'if (b2 != SAT_BYTE2_PIO_IN)'
       Repl = 'if ((b2 | 0x08) != SAT_BYTE2_PIO_IN)' }
    @{ Name = 'accept PROTOCOL 5 (PIO data-out)'
       Find = '(((b1 >> 1) & 0x0F) != SAT_PROTO_PIO_IN)'
       Repl = '(((b1 >> 1) & 0x0F) != SAT_PROTO_PIO_IN && ((b1 >> 1) & 0x0F) != 5)' }
    @{ Name = 'accept a data-out CBW'
       Find = 'if (!in->dir_in) return'
       Repl = 'if (0) return' }
    @{ Name = 'ignore the CBW transfer length'
       Find = 'if (in->xfer_len != (uint32_t)count * SAT_SECTOR_SIZE) return'
       Repl = 'if (0) return' }
    @{ Name = 'accept CK_COND=1'
       Find = 'if (b2 != SAT_BYTE2_PIO_IN)'
       Repl = 'if ((b2 & ~0x20) != SAT_BYTE2_PIO_IN)' }
    @{ Name = 'accept MULTIPLE_COUNT != 0'
       Find = 'if ((b1 >> 5) != 0) return'
       Repl = 'if (0) return' }
    @{ Name = 'ignore FEATURES'
       Find = 'if (feat != 0 || hfeat != 0) return'
       Repl = 'if (0) return' }
    @{ Name = 'ignore CONTROL'
       Find = 'if (ctrl != 0) return'
       Repl = 'if (0) return' }
    @{ Name = 'ignore the DEV bit'
       Find = 'if (dev & 0x10) return'
       Repl = 'if (0) return' }
    @{ Name = 'allow READ SECTORS EXT in the 12-byte form'
       Find = 'if (!is16 || !ext) return'
       Repl = 'if (is16 && !ext) return' }
    @{ Name = 'allow READ SECTORS with EXTEND=1'
       Find = "    case ATA_READ_SECTORS_NR:`n        if (ext) return"
       Repl = "    case ATA_READ_SECTORS_NR:`n        if (0) return" }
    @{ Name = 'drop the 28-bit LBA end check'
       Find = '(1ull << 28)'
       Repl = '(1ull << 32)' }
    @{ Name = 'drop the 48-bit LBA end check'
       Find = '(1ull << 48)'
       Repl = '(1ull << 56)' }
    @{ Name = 'ignore non-zero HOB bytes on a 28-bit read'
       Find = 'if (hcount | lba3 | lba4 | lba5) return'
       Repl = 'if (0) return' }
    @{ Name = 'accept CHS addressing (LBA bit clear)'
       Find = 'if (!(dev & 0x40)) return'
       Repl = 'if (0) return' }
    @{ Name = 'accept IDENTIFY with a non-zero LBA'
       Find = 'if (lba0 | lba1 | lba2 | lba3 | lba4 | lba5) return'
       Repl = 'if (0) return' }
    @{ Name = 'ignore the reserved byte 10 of the 12-byte form'
       Find = 'if (c[10] != 0) return'
       Repl = 'if (0) return' }
    @{ Name = 'accept any CB length >= 12 for A1'
       Find = 'in->cdb_len == 12'
       Repl = 'in->cdb_len >= 12' }
    @{ Name = 'refuse nothing when unmounted (skip the state check)'
       Find = 'if (!in->mounted) return'
       Repl = 'if (0) return' }
    @{ Name = 'allow reads in CHS mode'
       Find = 'if (cmd != ATA_IDENTIFY && !in->lba_mode) return'
       Repl = 'if (0) return' }
    @{ Name = 'CBW parse: skip the CB byte comparison'
       Find = 'if (memcmp(img + 15, cdb, 16) != 0) return false;'
       Repl = '' }
    @{ Name = 'CBW parse: treat any flags byte with bit 7 as data-in'
       Find = 'out->dir_in = (img[12] == 0x80);'
       Repl = 'out->dir_in = (img[12] & 0x80) != 0;' }
    @{ Name = 'CBW parse: skip the signature'
       Find = 'if (img[0] != 0x55 || img[1] != 0x53 || img[2] != 0x42 || img[3] != 0x43) return false;'
       Repl = '' }
    @{ Name = 'leave the device byte in the task file on refusal'
       Find = "    uint8_t tdev = 0;`n    switch (cmd) {"
       Repl = "    uint8_t tdev = 0;`n    tf->device = dev;`n    switch (cmd) {" }
)

$orig = [IO.File]::ReadAllText($policy).Replace("`r`n", "`n")
$killed = 0; $survived = 0; $broken = 0
$rows = @()
$i = 0
foreach ($m in $mutants) {
    $i++
    $dir = Join-Path $work ("m{0:d2}" -f $i)
    New-Item -ItemType Directory -Path $dir | Out-Null
    Copy-Item $header $dir
    $hits = ([regex]::Matches($orig, [regex]::Escape($m.Find))).Count
    if ($hits -eq 0) {
        $broken++
        $rows += [pscustomobject]@{ N = $i; Mutant = $m.Name; Result = 'NOT APPLIED'; Detail = 'pattern not found' }
        continue
    }
    $mut = $orig.Replace($m.Find, $m.Repl)
    [IO.File]::WriteAllText((Join-Path $dir 'sat_policy.c'), $mut)
    $b = Invoke-Build $dir (Join-Path $dir "t$exe") -Mutant
    if (-not $b.Ok) {
        $broken++
        $rows += [pscustomobject]@{ N = $i; Mutant = $m.Name; Result = 'COULD NOT BUILD'; Detail = ($b.Log -split "`n" | Select-Object -First 1) }
        continue
    }
    $out = & (Join-Path $dir "t$exe") 2>&1 | Out-String
    $code = $LASTEXITCODE
    $summary = ([regex]::Match($out, '(\d+) failures')).Groups[1].Value
    if ($code -ne 0) {
        $killed++
        $rows += [pscustomobject]@{ N = $i; Mutant = $m.Name; Result = 'KILLED'; Detail = ("{0} failed assertions, {1} site(s)" -f $summary, $hits) }
    } else {
        $survived++
        $rows += [pscustomobject]@{ N = $i; Mutant = $m.Name; Result = 'SURVIVED'; Detail = 'test passed a broken policy' }
    }
}

Write-Host "`n--- mutants ---" -ForegroundColor Yellow
$rows | Format-Table -AutoSize | Out-String -Width 200 | Write-Host

Remove-Item -Recurse -Force $work

Write-Host ("mutants: {0} total, {1} killed, {2} survived, {3} not run" -f $mutants.Count, $killed, $survived, $broken)
if ($survived -gt 0 -or $broken -gt 0) {
    Write-Host "RESULT: FAIL (a mutant survived or could not be run)" -ForegroundColor Red
    exit 1
}
Write-Host "RESULT: PASS (real policy passes; every mutant is caught)" -ForegroundColor Green
exit 0
