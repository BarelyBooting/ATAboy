"""The READ(10) and log path of the API build is instruction for instruction the
base build's (review B M-1, hunt list "Latency or timing change on the hot read
path").

    python tests/host/check_hotpath.py <base ATAboy.elf> <api ATAboy.elf> [<api ATAboy.elf> ...]

Disassembles both with arm-none-eabi-objdump, masks every 8-digit hex word
(branch targets, literal-pool addresses of statics, which move when code is
added elsewhere) and compares function by function the ones on that path. The
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


def funcs(elf, objdump):
    out = subprocess.run([objdump, '-d', '--no-show-raw-insn', elf], capture_output=True, text=True).stdout
    d, cur = {}, None
    for line in out.splitlines():
        m = re.match(r'^[0-9a-f]+ <(.+)>:$', line)
        if m:
            cur = m.group(1); d[cur] = []; continue
        if cur and line.strip():
            t = line.split('\t', 1)
            if len(t) < 2: continue
            s = re.sub(r'\b[0-9a-f]{8}\b', 'ADDR', t[1])
            s = re.sub(r'\.word\s+0x[0-9a-f]+', '.word X', s)
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
