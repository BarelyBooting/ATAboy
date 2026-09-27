"""The ATAboy Control API's link check (palimpsest api/SPEC-v1.md 11.1, I-5,
I-6, I-20, I-23), on the built ELF, like check_scratch.sh: nothing the API
runs can reach what it must never do.

    python tests/host/check_ctl.py <ATAboy.elf> [objdump] [--forbid NAME ...]

The call graph is read from `objdump -d`: every bl/b/blx to a symbol, a
linker veneer (__name_veneer, for code in RAM) taken as the name. The
op functions are called through a pointer (ctl.c, core0_fn), which objdump
cannot follow, so ctl_task and run_core0 are given edges to every op_*
function by name. From the API's entry points (core 0: ctl_task, ctl_init,
the two callbacks; core 1: ctl_core1_idle, ctl_tui_busy, ctl_mount_noted)
no path may reach:
  config_save, flash_range_erase, flash_range_program   the settings (I-5)
  ide_write_sectors                                     30h/34h (I-6)
  reset_usb_boot, rom_reset_usb_boot                    firmware update (I-20)
  ide_write_reg, ide_write_control, ide_reset_drive,    any command, Device Control
  ide_probe_devices, ide_manual_chs, ide_identify       or RESET- (stage 1 has no
                                                        op that sends one)
  queue_add_blocking, queue_try_remove                  the TUI's queues (I-23)
--forbid adds a name to that list: `--forbid ide_read_reg` must fail (the
taskfile.snapshot job reads registers), which shows the check can.
Prints "N checks, M failed"; COULD NOT RUN (exit 98) without the ELF or the
Arm objdump, never a pass.
"""
import re, subprocess, sys

ENTRIES = ['ctl_task', 'ctl_init', 'ctl_on_usb_mount', 'tud_cdc_line_state_cb',
           'ctl_core1_idle', 'ctl_tui_busy', 'ctl_mount_noted']
FORBIDDEN = ['config_save', 'flash_range_erase', 'flash_range_program', 'ide_write_sectors',
             'reset_usb_boot', 'rom_reset_usb_boot', 'ide_write_reg', 'ide_write_control',
             'ide_reset_drive', 'ide_probe_devices', 'ide_manual_chs', 'ide_manual_chs_native',
             'ide_identify', 'queue_add_blocking', 'queue_try_remove']


def main():
    args = sys.argv[1:]
    while '--forbid' in args:
        i = args.index('--forbid')
        FORBIDDEN.append(args[i + 1])
        del args[i:i + 2]
    if not args:
        print('usage: check_ctl.py <ATAboy.elf> [objdump] [--forbid NAME ...]'); return 2
    elf = args[0]
    objdump = args[1] if len(args) > 1 else 'arm-none-eabi-objdump'
    try:
        dis = subprocess.run([objdump, '-d', elf], capture_output=True, text=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError) as e:
        print(f'COULD NOT RUN: {objdump} -d {elf}: {e}'); return 98
    calls, cur = {}, None
    for line in dis.splitlines():
        m = re.match(r'^[0-9a-f]+ <([^>]+)>:$', line)
        if m:
            cur = m.group(1)
            calls[cur] = set()
            continue
        m = re.search(r'\tb(?:l|lx|\.w|\.n)?\s+[0-9a-f]+ <([^>+]+)', line)
        if cur and m:
            # A call from flash to code in RAM (SCRATCH_X: ide_read_reg and the
            # other bus routines) goes through a linker veneer, __name_veneer.
            g = re.sub(r'^__(.+)_veneer$', r'\1', m.group(1))
            if g != cur:
                calls[cur].add(g)
    ops = {f for f in calls if f.startswith('op_')}
    for f in list(calls):
        if f == 'ctl_task' or f.startswith('run_core0'):
            calls[f] |= ops
    checks = failures = 0
    missing = [e for e in ENTRIES if e not in calls]
    checks += 1
    if missing or not ops:
        failures += 1
        print(f'FAIL: entry points not in the ELF: {missing}, op functions {len(ops)}')
    reach = {}
    for e in ENTRIES:
        seen, stack, parent = {e}, [e], {e: None}
        while stack:
            f = stack.pop()
            for g in calls.get(f, ()):
                if g not in seen:
                    seen.add(g); parent[g] = f; stack.append(g)
        reach[e] = (seen, parent)
    for e in ENTRIES:
        seen, parent = reach[e]
        for bad in FORBIDDEN:
            checks += 1
            if bad in seen:
                failures += 1
                path, f = [], bad
                while f:
                    path.append(f); f = parent[f]
                print(f'FAIL: {e} reaches {bad}: ' + ' > '.join(reversed(path)))
    total = len(set().union(*(s for s, _ in reach.values())))
    print(f'API call graph: {len(ENTRIES)} entry points, {len(ops)} op functions, {total} functions reachable')
    print(f'{checks} checks, {failures} failed')
    return min(failures, 255)


if __name__ == '__main__':
    sys.exit(main())
