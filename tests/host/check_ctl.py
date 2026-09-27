"""The ATAboy Control API's link check (palimpsest api/SPEC-v1.md 11.1, I-5,
I-6, I-20, I-23), on the built ELF, like check_scratch.sh: nothing the API
runs can reach what it must never do.

    python tests/host/check_ctl.py <ATAboy.elf> [objdump] [--forbid NAME ...]

The call graph is read from `objdump -d`: every bl/b/blx to a symbol, a
linker veneer (__name_veneer, for code in RAM) taken as the name, and a
compiler's clone of a function (name.constprop.0, name.part.0, name.isra.0,
name.cold) taken as the function itself, so a static function GCC split or
specialised is still found by its name (stage 2: run_unmount.constprop.0,
menu_unmount.part.0). The
op functions are called through a pointer (ctl.c, core0_fn), which objdump
cannot follow, so ctl_task and run_core0 are given edges to every op_*
function by name. From the API's entry points (core 0: ctl_task, ctl_init,
the two callbacks; core 1: ctl_core1_idle, ctl_tui_busy, ctl_mount_noted,
ctl_setup_noted), and from each of stage 2's ratchet ops on its own (core
0: op_wp_on, op_pipelined_off, op_identify_forbid; the jobs run_unmount and
run_detach, and the menus.c parts they call), no path may reach:
  config_save, flash_range_erase, flash_range_program   the settings (I-5)
  ide_write_sectors                                     30h/34h (I-6)
  reset_usb_boot, rom_reset_usb_boot                    firmware update (I-20)
  ide_write_reg, ide_write_control, ide_reset_drive,    any command, Device Control
  ide_probe_devices, ide_manual_chs, ide_identify       or RESET- (no op of stages
                                                        1 and 2 sends one: unmount
                                                        and detach only wait for
                                                        USB, as the TUI's unmount)
  queue_add_blocking, queue_try_remove                  the TUI's queues (I-23)
Stage 3 (--jumpers): the jumpers' functions (board_jumpers.c) are entry
points too, each on its own: board_jumpers_init, jp1_locked, bj_task,
bj_pads_ok (bj_tier_m_open, stage 4's, has no caller yet and is not linked).
Besides the list above, none of them may reach
any bus access at all, reads included (ide_read_reg, ide_read_alt_status,
ide_pio_read, ide_pio_write): they read and set up GPIO30/31's pads only.
Without --jumpers the ELF must not have them (a build without
ATABOY_BOARD_JUMPERS carries no jumper code).
Stage 4, Tier M: the jobs run_setup_ctrlg and run_mount, the menus.c parts
they call (menus_api_ctrlg, menus_api_mount) and the TUI's presence row
(ctl_presence_text) are entry points too. setup.ctrlg reaches the bus, and
only through Ctrl+G's own code: ide_manual_chs and ide_manual_chs_native
are its gateways. From ctl_core1_idle, run_setup_ctrlg and menus_api_ctrlg
the walk stops at a gateway (it may be reached, not passed through), and
the list above holds for everything else; from ctl_core1_idle a gateway is
reached only through run_setup_ctrlg; and walked through the gateways too,
no entry point may reach the settings, a sector write, firmware update, an
IDENTIFY (ide_identify) or the TUI's queues, except that the walk stops at
the Q7 tripwire's stop (ide_trip_stop): on core 1 it shows the trip record
for ever (trip_screen_forever, noreturn) and never goes back to the bus.
The mount and the presence row reach no bus access at all, reads included.
Stage 4 entry points are required with --jumpers only: without the jumpers
no Tier M job gets past its pickup check, the compiler drops the rest, and
then no entry point may reach a gateway at all.
--forbid adds a name to that list: `--forbid ide_read_reg` must fail (the
taskfile.snapshot job reads registers), and so must `--forbid menu_unmount`
(the unmount and detach jobs run the TUI's own unmount), which shows the
check can.
Prints "N checks, M failed"; COULD NOT RUN (exit 98) without the ELF or the
Arm objdump, never a pass.
"""
import re, subprocess, sys

ENTRIES = ['ctl_task', 'ctl_init', 'ctl_on_usb_mount', 'tud_cdc_line_state_cb',
           'ctl_core1_idle', 'ctl_tui_busy', 'ctl_mount_noted', 'ctl_setup_noted',
           # stage 2, the ratchet, each on its own
           'op_wp_on', 'op_pipelined_off', 'op_identify_forbid', 'run_unmount', 'run_detach',
           'menus_api_unmount', 'menus_api_detach_clear',
           # stage 4, Tier M
           'run_setup_ctrlg', 'run_mount', 'menus_api_ctrlg', 'menus_api_mount', 'ctl_presence_text']
STAGE4_ENTRIES = ['run_setup_ctrlg', 'run_mount', 'menus_api_ctrlg', 'menus_api_mount', 'ctl_presence_text']
GATEWAYS = ['ide_manual_chs', 'ide_manual_chs_native']
GATEWAY_ENTRIES = ['ctl_core1_idle', 'run_setup_ctrlg', 'menus_api_ctrlg']
NO_BUS_ENTRIES = ['run_mount', 'menus_api_mount', 'ctl_presence_text']
# Never, not even through Ctrl+G's own code.
FULL_FORBIDDEN = ['config_save', 'flash_range_erase', 'flash_range_program', 'ide_write_sectors',
                  'reset_usb_boot', 'rom_reset_usb_boot', 'ide_identify', 'queue_add_blocking', 'queue_try_remove']
JUMPER_ENTRIES = ['board_jumpers_init', 'jp1_locked', 'bj_task', 'bj_pads_ok']
JUMPER_FORBIDDEN = ['ide_read_reg', 'ide_read_alt_status', 'ide_pio_read', 'ide_pio_write']
FORBIDDEN = ['config_save', 'flash_range_erase', 'flash_range_program', 'ide_write_sectors',
             'reset_usb_boot', 'rom_reset_usb_boot', 'ide_write_reg', 'ide_write_control',
             'ide_reset_drive', 'ide_probe_devices', 'ide_manual_chs', 'ide_manual_chs_native',
             'ide_identify', 'queue_add_blocking', 'queue_try_remove']


def base_name(n):
    """The function a GCC clone or split part belongs to."""
    return re.sub(r'(\.(constprop|part|isra|cold|lto_priv)(\.\d+)?)+$', '', n)


def main():
    args = sys.argv[1:]
    jumpers = '--jumpers' in args
    if jumpers:
        args.remove('--jumpers')
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
            cur = base_name(m.group(1))
            calls.setdefault(cur, set())
            continue
        m = re.search(r'\tb(?:l|lx|\.w|\.n)?\s+[0-9a-f]+ <([^>+]+)', line)
        if cur and m:
            # A call from flash to code in RAM (SCRATCH_X: ide_read_reg and the
            # other bus routines) goes through a linker veneer, __name_veneer.
            g = base_name(re.sub(r'^__(.+)_veneer$', r'\1', m.group(1)))
            if g != cur:
                calls[cur].add(g)
    ops = {f for f in calls if f.startswith('op_')}
    for f in list(calls):
        if f == 'ctl_task' or f.startswith('run_core0'):
            calls[f] |= ops
    checks = failures = 0
    entries = ENTRIES + (JUMPER_ENTRIES if jumpers else [])
    missing = [e for e in entries if e not in calls and (jumpers or e not in STAGE4_ENTRIES)]
    entries = [e for e in entries if e in calls or e not in STAGE4_ENTRIES]
    checks += 1
    if missing or not ops:
        failures += 1
        print(f'FAIL: entry points not in the ELF: {missing}, op functions {len(ops)}')
    if not jumpers:
        checks += 1
        present = [e for e in JUMPER_ENTRIES if e in calls]
        if present:
            failures += 1
            print(f'FAIL: jumper code in a build without --jumpers: {present}')
    def walk(e, stop=(), skip=()):
        seen, stack, parent = {e}, [e], {e: None}
        while stack:
            f = stack.pop()
            if f in stop:
                continue                    # reached, not passed through
            for g in calls.get(f, ()):
                if g not in seen and g not in skip:
                    seen.add(g); parent[g] = f; stack.append(g)
        return seen, parent

    def report(e, bad, parent, how=''):
        path, f = [], bad
        while f:
            path.append(f); f = parent[f]
        print(f'FAIL: {e} reaches {bad}{how}: ' + ' > '.join(reversed(path)))

    reach = {}
    for e in entries:
        gate = jumpers and e in GATEWAY_ENTRIES
        seen, parent = walk(e, stop=GATEWAYS if gate else ())
        reach[e] = (seen, parent)
        forbidden = [b for b in FORBIDDEN if not (gate and b in GATEWAYS)]
        forbidden += JUMPER_FORBIDDEN if e in JUMPER_ENTRIES or e in NO_BUS_ENTRIES else []
        for bad in forbidden:
            checks += 1
            if bad in seen:
                failures += 1
                report(e, bad, parent)
        full, fparent = walk(e, stop=('ide_trip_stop',))
        for bad in FULL_FORBIDDEN:
            checks += 1
            if bad in full:
                failures += 1
                report(e, bad, fparent, ' (through the gateways)')
    # From core 1's idle point, Ctrl+G's code only through the setup.ctrlg job.
    if jumpers and 'ctl_core1_idle' in calls:
        seen, parent = walk('ctl_core1_idle', stop=GATEWAYS, skip=('run_setup_ctrlg',))
        for g in GATEWAYS:
            checks += 1
            if g in seen:
                failures += 1
                report('ctl_core1_idle', g, parent, ' not through run_setup_ctrlg')
    total = len(set().union(*(s for s, _ in reach.values())))
    nj = len(JUMPER_ENTRIES) if jumpers else 0
    print(f'API call graph: {len(entries)} entry points ({nj} of them the jumpers), '
          f'{len(ops)} op functions, {total} functions reachable')
    print(f'{checks} checks, {failures} failed')
    return min(failures, 255)


if __name__ == '__main__':
    sys.exit(main())
