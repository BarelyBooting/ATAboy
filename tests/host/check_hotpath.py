"""The READ(10) and log path of the API build is instruction for instruction the
base build's (review B M-1, hunt list "Latency or timing change on the hot read
path").

    python tests/host/check_hotpath.py <base ATAboy.elf> <api ATAboy.elf> [<api ATAboy.elf> ...]

Disassembles both with arm-none-eabi-objdump, masks every 8-digit hex word in
an instruction (branch targets, and the addresses in `@ (...)` comments, which
move when code is added elsewhere) and compares function by function the ones
on that path. Literal-pool words (`.word 0x...`) are COMPARED, not masked, except
those whose value is an ADDRESS in the image being read: inside its own flash
image (0x10000000 to the end of its sections) or in SRAM (0x20000000 to
0x20082000, the RP2350's 520 KB), which do move. A constant that is neither (a timeout, a mask, a
divisor such as the 0x10624dd3 of ide_wait_until_ready, a peripheral address)
must be equal in the two builds (review delta-B MED-3: before 2026-09-29 every
`.word` was masked and one bit of that constant went unseen). The one remaining
blind spot: a constant that happens to fall inside the flash image's or SRAM's
address range is treated as an address. The
set is derived from the base ELF's own symbol table by name, not typed in:
every `ide_pio_*`, `ide_read_*`, `ide_seek_*`, `ide_wait_*`, `ide_write_reg`,
`ide_write_control`, `obs_*` (except obs_marker: its only caller is log_mount, on a mount,
never a READ, and in the API images it also keeps the marker's sequence for
the API (obs_last_marker_seq)), `read10`, `proc_read10_cmd*`, `read_buffer`, `read_long*`, `read_partial*`,
`usb_read_ahead_task` and `tud_msc_read10_cb`.

An empty set is a refusal, not a pass. Prints "N checks, M failed"; exit 98
(COULD NOT RUN) when objdump or an ELF is missing.
"""
import re, shutil, subprocess, sys

PREFIXES = ('ide_pio_', 'ide_read_', 'ide_seek_', 'ide_wait_', 'obs_', 'read10', 'proc_read10_cmd', 'read_buffer',
            'read_long', 'read_partial', 'usb_read_ahead_task', 'tud_msc_read10_cb')
EXACT = ('ide_write_reg', 'ide_write_control')
EXCEPT = ('obs_marker',)


SRAM = (0x20000000, 0x20082000)   # RP2350's 520 KB (the board's part)


def flash_end(elf, objdump):
    """The end of the highest allocated section in the flash window, from `objdump -h`."""
    out = subprocess.run([objdump, '-h', elf], capture_output=True, text=True).stdout
    end = 0x10000000
    for line in out.splitlines():
        f = line.split()
        if len(f) >= 4 and re.fullmatch(r'[0-9a-f]{8}', f[2]) and re.fullmatch(r'[0-9a-f]{8}', f[3]):
            size, vma = int(f[2], 16), int(f[3], 16)
            if 0x10000000 <= vma < 0x11000000:
                end = max(end, vma + size)
    return end


def mask_word(m, hi):
    v = int(m.group(1), 16)
    return '.word ADDR' if (0x10000000 <= v < hi or SRAM[0] <= v < SRAM[1]) else m.group(0)


def funcs(elf, objdump):
    out = subprocess.run([objdump, '-d', '--no-show-raw-insn', elf], capture_output=True, text=True).stdout
    if not out.strip():
        raise OSError('objdump gave no disassembly of ' + elf)
    hi = flash_end(elf, objdump)
    d, cur = {}, None
    for line in out.splitlines():
        m = re.match(r'^[0-9a-f]+ <(.+)>:$', line)
        if m:
            cur = m.group(1); d[cur] = []; continue
        if cur and line.strip():
            t = line.split('\t', 1)
            if len(t) < 2: continue
            s = re.sub(r'\.word\s+0x([0-9a-f]+)', lambda m: mask_word(m, hi), t[1])
            s = re.sub(r'\b[0-9a-f]{8}\b', 'ADDR', s)
            d[cur].append(s)
    return d


def main():
    if len(sys.argv) < 3:
        print('usage: check_hotpath.py BASE.elf API.elf ...'); return 2
    import os
    objdump = os.environ.get('OBJDUMP') or shutil.which('arm-none-eabi-objdump')
    if not objdump:
        print('COULD NOT RUN: no arm-none-eabi-objdump (set OBJDUMP or put it on PATH)'); return 98
    try:
        base = funcs(sys.argv[1], objdump)
        others = [(p, funcs(p, objdump)) for p in sys.argv[2:]]
    except OSError as e:
        print('COULD NOT RUN:', e); return 98
    names = sorted(n for n in base if n not in EXCEPT and (n.startswith(PREFIXES) or n in EXACT))
    checks = failures = 0
    checks += 1
    if len(names) < 20:
        failures += 1; print(f'FAIL: only {len(names)} functions on the path in the base ELF (an empty set is a refusal)')
    for path, api in others:
        for n in names:
            checks += 1
            if n not in api:
                failures += 1; print(f'FAIL: {n} is missing from {path}')
            elif api[n] != base[n]:
                failures += 1; print(f'FAIL: {n} differs in {path} ({len(base[n])} vs {len(api[n])} instructions)')
        print(f'{path}: {len(names)} functions on the READ and log path compared with the base')
    print(f'{checks} checks, {failures} failed')
    return min(failures, 255)


if __name__ == '__main__':
    sys.exit(main())
