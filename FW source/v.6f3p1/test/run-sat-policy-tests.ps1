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
    @{ Name = 'accept PROTOCOL 5 (PIO data-out) as data-in'
       Find = '    if (proto == SAT_PROTO_PIO_IN) {'
       Repl = "    if (proto == 5) proto = SAT_PROTO_PIO_IN;`n    if (proto == SAT_PROTO_PIO_IN) {" }
    @{ Name = 'accept a data-out CBW'
       Find = 'if (!in->dir_in) return'
       Repl = 'if (0) return' }
    @{ Name = 'ignore the CBW transfer length'
       Find = 'if (in->xfer_len != (uint32_t)count * SAT_SECTOR_SIZE) return'
       Repl = 'if (0) return' }
    @{ Name = 'accept CK_COND=1 on PIO data-in'
       Find = 'if (b2 != SAT_BYTE2_PIO_IN)'
       Repl = 'if ((b2 & ~0x20) != SAT_BYTE2_PIO_IN)' }
    @{ Name = 'accept MULTIPLE_COUNT != 0'
       Find = 'if ((b1 >> 5) != 0) return'
       Repl = 'if (0) return' }
    @{ Name = 'ignore FEATURES on IDENTIFY, reads and verify'
       Find = 'if (feat != 0) return'
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
       Find = "    case ATA_READ_SECTORS_NR:`n        want_proto = SAT_PROTO_PIO_IN;`n        user_data = true;`n        if (ext) return"
       Repl = "    case ATA_READ_SECTORS_NR:`n        want_proto = SAT_PROTO_PIO_IN;`n        user_data = true;`n        if (0) return" }
    @{ Name = 'drop the 28-bit LBA end check'
       Find = '(1ull << 28)'
       Repl = '(1ull << 32)' }
    @{ Name = 'drop the 48-bit LBA end check'
       Find = '(1ull << 48)'
       Repl = '(1ull << 56)' }
    @{ Name = 'ignore non-zero HOB LBA bytes on 28-bit rows'
       Find = 'if (lba3 | lba4 | lba5) return'
       Repl = 'if (0) return' }
    @{ Name = 'accept CHS addressing (LBA bit clear) on reads and verify'
       Find = 'if (!(dev & 0x40)) return'
       Repl = 'if (0) return' }
    @{ Name = 'accept IDENTIFY and READ NATIVE MAX with a non-zero LBA'
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
    @{ Name = 'allow reads and verify in CHS mode'
       Find = 'if (user_data && !in->lba_mode) return'
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
       Find = "    bool user_data = false;     // carries a user-data address: LBA mode only`n    switch (cmd) {"
       Repl = "    bool user_data = false;     // carries a user-data address: LBA mode only`n    tf->device = dev;`n    switch (cmd) {" }

    # ---- stage 2: non-data protocol, READ VERIFY, SMART, READ NATIVE MAX ----
    @{ Name = 'ignore HOB FEATURES and HOB COUNT'
       Find = 'if (hfeat != 0 || hcount != 0) return'
       Repl = 'if (0) return' }
    @{ Name = 'row protocol not enforced (any row as PIO-in or non-data)'
       Find = 'if (proto != want_proto) return'
       Repl = 'if (0) return' }
    @{ Name = 'non-data: accept T_LENGTH != 0'
       Find = '(uint8_t)~(SAT_BYTE2_CK_COND | SAT_BYTE2_NO_DATA_PHASE)'
       Repl = '(uint8_t)~(SAT_BYTE2_CK_COND | SAT_BYTE2_NO_DATA_PHASE | 0x03)' }
    @{ Name = 'non-data: accept OFF_LINE != 0'
       Find = '(uint8_t)~(SAT_BYTE2_CK_COND | SAT_BYTE2_NO_DATA_PHASE)'
       Repl = '(uint8_t)~(SAT_BYTE2_CK_COND | SAT_BYTE2_NO_DATA_PHASE | 0xC0)' }
    @{ Name = 'non-data: accept T_TYPE=1'
       Find = '(uint8_t)~(SAT_BYTE2_CK_COND | SAT_BYTE2_NO_DATA_PHASE)'
       Repl = '(uint8_t)~(SAT_BYTE2_CK_COND | SAT_BYTE2_NO_DATA_PHASE | 0x10)' }
    @{ Name = 'non-data: refuse T_DIR / BYTE_BLOCK set (smartctl form)'
       Find = 'SAT_BYTE2_NO_DATA_PHASE  0x0C'
       Repl = 'SAT_BYTE2_NO_DATA_PHASE  0x00' }
    @{ Name = 'non-data: accept a CBW with a data phase'
       Find = 'if (in->xfer_len != 0) return'
       Repl = 'if (0) return' }
    @{ Name = 'register rows accept CK_COND=0'
       Find = 'if (need_ck && !ck) return'
       Repl = 'if (0) return' }
    @{ Name = 'READ VERIFY: count 0 taken as 0 sectors in the end check'
       Find = 'uint32_t n = count ? count : 256u;'
       Repl = 'uint32_t n = count;' }
    @{ Name = 'READ VERIFY: accept EXTEND=1'
       Find = "        want_proto = SAT_PROTO_NON_DATA;`n        user_data = true;`n        if (ext) return"
       Repl = "        want_proto = SAT_PROTO_NON_DATA;`n        user_data = true;`n        if (0) return" }
    @{ Name = 'READ VERIFY: accept CHS addressing (LBA bit clear)'
       Find = 'if (!(dev & 0x40)) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);   // verify'
       Repl = 'if (0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);   // verify' }
    @{ Name = 'READ VERIFY: allowed in CHS mount mode'
       Find = "        want_proto = SAT_PROTO_NON_DATA;`n        user_data = true;"
       Repl = "        want_proto = SAT_PROTO_NON_DATA;`n        user_data = false;" }
    @{ Name = 'IDENTIFY, SMART and READ NATIVE MAX refused in CHS mode'
       Find = 'if (user_data && !in->lba_mode) return'
       Repl = 'if (!in->lba_mode) return' }
    @{ Name = 'allow DEVICE CONFIGURATION 0xB1 (as SMART)'
       Find = "    case ATA_SMART:`n"
       Repl = "    case ATA_SMART:`n    case 0xB1:`n" }
    @{ Name = 'allow SMART ENABLE OPERATIONS D8'
       Find = "        case SMART_RETURN_STATUS:`n"
       Repl = "        case SMART_RETURN_STATUS:`n        case 0xD8:`n" }
    @{ Name = 'allow SMART EXECUTE OFF-LINE D4'
       Find = "        case SMART_RETURN_STATUS:`n"
       Repl = "        case SMART_RETURN_STATUS:`n        case 0xD4:`n" }
    @{ Name = 'allow SMART WRITE LOG D6 (as READ LOG)'
       Find = "        case SMART_READ_LOG:`n"
       Repl = "        case SMART_READ_LOG:`n        case 0xD6:`n" }
    @{ Name = 'SMART: ignore the 4F/C2 signature'
       Find = 'if (lba1 != SMART_LBA_MID || lba2 != SMART_LBA_HIGH) return'
       Repl = 'if (0) return' }
    @{ Name = 'SMART: allow EXTEND=1'
       Find = "        if (ext) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);`n        if (lba3 | lba4 | lba5) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);`n        if (lba1 != SMART_LBA_MID"
       Repl = "        if (lba3 | lba4 | lba5) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);`n        if (lba1 != SMART_LBA_MID" }
    @{ Name = 'SMART: ignore the device low nibble'
       Find = 'if (dev & 0x0F) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);   // smart'
       Repl = 'if (0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);   // smart' }
    @{ Name = 'SMART READ DATA/THRESHOLDS: allow count != 1'
       Find = "            want_proto = SAT_PROTO_PIO_IN;`n            if (count != 1) return"
       Repl = "            want_proto = SAT_PROTO_PIO_IN;`n            if (0) return" }
    @{ Name = 'SMART READ DATA/THRESHOLDS: allow a non-zero LBA low'
       Find = "            if (count != 1) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);`n            if (lba0 != 0) return"
       Repl = "            if (count != 1) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);`n            if (0) return" }
    @{ Name = 'SMART READ LOG: no sector cap'
       Find = 'if (count == 0 || count > SAT_MAX_SECTORS) return'
       Repl = 'if (count == 0) return' }
    @{ Name = 'SMART READ LOG: allow 0 sectors'
       Find = 'if (count == 0 || count > SAT_MAX_SECTORS) return'
       Repl = 'if (count > SAT_MAX_SECTORS) return' }
    @{ Name = 'SMART RETURN STATUS: allow a non-zero count'
       Find = "            need_ck = true;`n            if (count != 0) return"
       Repl = "            need_ck = true;`n            if (0) return" }
    @{ Name = 'SMART RETURN STATUS: accept CK_COND=0'
       Find = "            want_proto = SAT_PROTO_NON_DATA;`n            need_ck = true;"
       Repl = "            want_proto = SAT_PROTO_NON_DATA;" }
    @{ Name = 'allow SET MAX ADDRESS 0xF9 (as READ NATIVE MAX)'
       Find = "    case ATA_READ_NATIVE_MAX:`n"
       Repl = "    case ATA_READ_NATIVE_MAX:`n    case 0xF9:`n" }
    @{ Name = 'allow SET MAX ADDRESS EXT 0x37 (as READ NATIVE MAX EXT)'
       Find = "    case ATA_READ_NATIVE_MAX_EXT:`n"
       Repl = "    case ATA_READ_NATIVE_MAX_EXT:`n    case 0x37:`n" }
    @{ Name = 'READ NATIVE MAX: accept CK_COND=0'
       Find = "        want_proto = SAT_PROTO_NON_DATA;`n        need_ck = true;`n        if (ext) return"
       Repl = "        want_proto = SAT_PROTO_NON_DATA;`n        if (ext) return" }
    @{ Name = 'READ NATIVE MAX: accept EXTEND=1'
       Find = "        need_ck = true;`n        if (ext) return refuse"
       Repl = "        need_ck = true;`n        if (0) return refuse" }
    @{ Name = 'READ NATIVE MAX: accept LBA bit clear'
       Find = 'if ((dev & 0x4F) != 0x40) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);  // native max: '
       Repl = 'if ((dev & 0x0F) != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);  // native max: ' }
    @{ Name = 'READ NATIVE MAX (both): accept FEATURES or count'
       Find = 'if (feat != 0 || count != 0) return'
       Repl = 'if (0) return' }
    @{ Name = 'READ NATIVE MAX (both): accept a non-zero LBA or HOB LBA'
       Find = "        if (feat != 0 || count != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);`n        if (lba0 | lba1 | lba2 | lba3 | lba4 | lba5) return"
       Repl = "        if (feat != 0 || count != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);`n        if (0) return" }
    @{ Name = 'READ NATIVE MAX EXT: accept the 12-byte form or EXTEND=0'
       Find = 'if (!is16 || !ext) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);  // native max ext'
       Repl = 'if (0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);  // native max ext' }
    @{ Name = 'READ NATIVE MAX EXT: accept LBA bit clear'
       Find = 'if ((dev & 0x4F) != 0x40) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);  // native max ext'
       Repl = 'if ((dev & 0x0F) != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);  // native max ext' }
    @{ Name = 'task file: non-data command given a data phase'
       Find = '    tf->sectors = sectors;'
       Repl = '    tf->sectors = count;' }
    @{ Name = 'task file: CK_COND not passed on'
       Find = '    tf->ck_cond = ck;'
       Repl = '    tf->ck_cond = false;' }
    @{ Name = 'task file: SMART subcommand not passed on'
       Find = '    tf->feature = feat;'
       Repl = '    tf->feature = 0;' }
    @{ Name = 'task file: every command marked PIO data-in'
       Find = '    tf->protocol = want_proto;'
       Repl = '    tf->protocol = SAT_PROTO_PIO_IN;' }
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
