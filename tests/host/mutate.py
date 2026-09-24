"""Mutation check for the host tests.

Each mutant is a small source edit that breaks one property the tests are
meant to protect (never hand the host a failed or invented sector, keep the
good prefix, BSY before ERR, and so on). A mutant is KILLED when the tests
fail against it. A mutant that survives means the tests do not cover that
property. A mutant whose text is not found, or that does not compile, is
reported as such and makes the run fail, so it is never counted as killed.

    python tests/host/mutate.py            (CXX picks the compiler, default g++)
"""
import os, shutil, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', '..', 'FW source', 'v.6f3p1')

MUTANTS = [
    # --- usb.c: what the host is given ---
    ('zero-fill the failed part and report success (old non-strict behaviour)', 'usb.c',
     'if (done > 0) return (int32_t)done;',
     'memset(ptr + got * 512, 0, remaining - got * 512); return (int32_t)bufsize;'),
    ('count the failed sector as delivered', 'usb.c',
     '(got < aligned ? got : 0) * 512', '(got + 1 <= aligned ? got + 1 : 0) * 512'),
    ('discard the good prefix (issue #13 as it was)', 'usb.c',
     'if (done > 0) return (int32_t)done;', ''),
    ('strict media bounds off (zero-fill past the end)', 'usb.c',
     '#if ATABOY_STRICT_MEDIA_BOUNDS', '#if 0'),
    # --- ide.c: what the read routine reports ---
    ('take ERR+DRQ data as good (DRQ checked before ERR)', 'ide.c',
     '            if (st & 0x01) goto read_err;\n            if (st & 0x08) goto drq_read;',
     '            if (st & 0x08) goto drq_read;\n            if (st & 0x01) goto read_err;'),
    ('report one sector too many on ERR', 'ide.c',
     '    if (!idle_after_error(2000)) soft_reset_restore();\n    if (done) *done = s;',
     '    if (!idle_after_error(2000)) soft_reset_restore();\n    if (done) *done = s + 1;'),
    ('report one sector too many on timeout', 'ide.c',
     '    soft_reset_restore();\n    if (done) *done = s;\n    return -1;\n}',
     '    soft_reset_restore();\n    if (done) *done = s + 1;\n    return -1;\n}'),
    ('ERR checked before BSY (the old poll order)', 'ide.c',
     '            if (st & 0x80) { busy_wait_us_32(10); continue; }\n            if (st & 0x01) goto read_err;',
     '            if (st & 0x01) goto read_err;\n            if (st & 0x80) { busy_wait_us_32(10); continue; }'),
    ('no 400 ns wait after the READ command', 'ide.c',
     '    ide_write_reg(7, cmd);                                 // READ SECTORS EXT / READ SECTORS\n    wait_after_command();',
     '    ide_write_reg(7, cmd);                                 // READ SECTORS EXT / READ SECTORS'),
    ('registers captured after the reset instead of before', 'ide.c',
     '    record_failure(IDE_FAIL_ERR, cmd, st, lba + s, s, count);\n    if (st & 0x08) {\n        // Data offered for the failed sector. Not good data: discard it.\n        ide_drain_sector();\n        last_fail.drained = true;\n    }\n    if (!idle_after_error(2000)) soft_reset_restore();',
     '    soft_reset_restore();\n    record_failure(IDE_FAIL_ERR, cmd, st, lba + s, s, count);'),
    ('always soft reset after ERR (old recovery)', 'ide.c',
     '    if (!idle_after_error(2000)) soft_reset_restore();',
     '    soft_reset_restore();'),
    ('never soft reset, even when the drive is not idle', 'ide.c',
     '    if (!idle_after_error(2000)) soft_reset_restore();',
     '    (void)idle_after_error(2000);'),
    ('no drain of the flawed data', 'ide.c',
     '        ide_drain_sector();\n        last_fail.drained = true;',
     '        last_fail.drained = true;'),
    ('CHS geometry not restored after SRST', 'ide.c',
     '    if (config.use_lba_mode) return true;\n    return ide_set_geometry(config.heads, config.spt);',
     '    return true;'),
    ('read data into the caller buffer when draining', 'ide.c',
     '        ide_drain_sector();\n        last_fail.drained = true;',
     '        set_address(0); xcvr_read(); sio_hw->gpio_clr = (1 << IDE_CS0);\n'
     '        ide_pio_read(256, wbuf + s * 256);\n'
     '        sio_hw->gpio_set = (1 << IDE_CS0); bus_idle();\n'
     '        last_fail.drained = true; s++;'),
    # --- ide.c: stale DRQ before a read ---
    ('no stale DRQ guard: command issued over stranded data', 'ide.c',
     '    if (st & 0x08) {\n        record_failure(IDE_FAIL_STALE_DRQ',
     '    if (0) {\n        record_failure(IDE_FAIL_STALE_DRQ'),
    ('stale DRQ recorded but not reset', 'ide.c',
     '        record_failure(IDE_FAIL_STALE_DRQ, 0, st, lba, 0, count);\n        soft_reset_restore();',
     '        record_failure(IDE_FAIL_STALE_DRQ, 0, st, lba, 0, count);'),
    ('stale DRQ recorded as a drive ERR', 'ide.c',
     'record_failure(IDE_FAIL_STALE_DRQ,', 'record_failure(IDE_FAIL_ERR,'),
    # The pre-merge SAT branch did `goto read_err` here. C++ rejects that goto
    # (it jumps over declarations), so the mutant spells out what read_err does.
    ('stale DRQ sent down the ERR path (the pre-merge SAT form)', 'ide.c',
     '    if (st & 0x08) {\n        record_failure(IDE_FAIL_STALE_DRQ, 0, st, lba, 0, count);\n'
     '        soft_reset_restore();\n        return -1;\n    }',
     '    if (st & 0x08) {\n        record_failure(IDE_FAIL_ERR, 0, st, lba, 0, count);\n'
     '        ide_drain_sector(); last_fail.drained = true;\n'
     '        if (!idle_after_error(2000)) soft_reset_restore();\n        return -1;\n    }'),
    # --- ide.c: soft reset and CHS geometry restore (review finding F1, F3) ---
    ('the old reset: 2 s wait, then restore regardless', 'ide.c',
     'IDE_SRST_TIMEOUT_MS 31000', 'IDE_SRST_TIMEOUT_MS 2000'),
    ('no 2 ms wait after SRST before polling', 'ide.c',
     '    busy_wait_us_32(2000);                  // ATA: 2 ms before status is valid\n', ''),
    # Found on hardware 2026-09-23 (slave-only ST380011A): the reset waited on
    # an absent master, whose status floats to 0xFF, and gave up unselected.
    ('slave waits on the absent master after SRST (the bug as found)', 'ide.c',
     '    if (dev_base == 0xA0) {\n        uint32_t start', '    if (1) {\n        uint32_t start'),
    ('device not selected again after SRST', 'ide.c',
     '    ide_write_reg(6, dev_base);\n    busy_wait_us_32(1);                     // 400 ns', '    busy_wait_us_32(1);                     // 400 ns'),
    ('geometry not marked lost by SRST', 'ide.c',
     '    chs_geometry_lost = true;               // SRST drops', '    // SRST drops'),
    ('CHS transfers never refuse on a lost geometry', 'ide.c',
     '    if (config.use_lba_mode || !chs_geometry_lost) return true;', '    return true;'),
    ('geometry restore ignores ABRT', 'ide.c',
     'bool ok = ide_wait_until_ready(1000) && !(ide_read_reg(7) & 0x01);', 'bool ok = ide_wait_until_ready(1000);'),
    ('reset recorded as fine when it failed', 'ide.c',
     '    last_fail.reset_failed = !srst_and_restore();', '    (void)srst_and_restore();'),
    ('no geometry check before a write', 'ide.c',
     '    uint8_t fail_kind = IDE_FAIL_TIMEOUT;\n    if (count == 0) return -1;\n'
     '    if (!ide_wait_until_ready(5000)) {\n'
     '        record_failure(IDE_FAIL_NOT_READY, 0, ide_read_reg(7), lba, 0, count);\n'
     '        return -1;\n    }\n    if (!chs_geometry_ok()) {',
     '    uint8_t fail_kind = IDE_FAIL_TIMEOUT;\n    if (count == 0) return -1;\n'
     '    if (!ide_wait_until_ready(5000)) {\n'
     '        record_failure(IDE_FAIL_NOT_READY, 0, ide_read_reg(7), lba, 0, count);\n'
     '        return -1;\n    }\n    if (0) {'),
    # --- usb.c: host-controlled offsets (review finding F2) ---
    ('READ(10) accepts a non-zero offset', 'usb.c',
     '    if (offset != 0 || (bufsize % 512) != 0) {\n        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00);\n        return -1;\n    }\n\n    uint64_t max = total_sectors();\n    if (max == 0) return -1;\n\n    uint32_t remaining = bufsize;\n    uint8_t *ptr = (uint8_t *)buffer;',
     '    if ((bufsize % 512) != 0) {\n        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00);\n        return -1;\n    }\n\n    uint64_t max = total_sectors();\n    if (max == 0) return -1;\n\n    uint32_t remaining = bufsize;\n    uint8_t *ptr = (uint8_t *)buffer;'),
    ('WRITE(10) accepts a length that is not whole sectors', 'usb.c',
     '    if (offset != 0 || (bufsize % 512) != 0) {\n        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00);\n        return -1;\n    }\n\n    uint64_t max = total_sectors();\n    if (max == 0) return -1;\n\n    uint32_t remaining = bufsize;\n    uint8_t *ptr = buffer;',
     '    if (offset != 0) {\n        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00);\n        return -1;\n    }\n\n    uint64_t max = total_sectors();\n    if (max == 0) return -1;\n\n    uint32_t remaining = bufsize;\n    uint8_t *ptr = buffer;'),
    # --- SAT stage 2: non-data path, register capture, sense delivery ---
    ('SAT non-data: read the data the drive offers instead of aborting', 'ide.c',
     '            if (st & 0x08) {                                    // DRQ on a non-data command\n'
     '                sat_abort();',
     '            if (st & 0x08) {                                    // DRQ on a non-data command\n'
     '                ide_drain_sector();'),
    ('SAT non-data: no abort on timeout', 'ide.c',
     '            sat_abort();\n            return IDE_SAT_TIMEOUT;\n        }\n        busy_wait_us_32(10);\n    }\n}\n#endif',
     '            return IDE_SAT_TIMEOUT;\n        }\n        busy_wait_us_32(10);\n    }\n}\n#endif'),
    ('SAT non-data: ERR taken as success', 'ide.c',
     'return (st & 0x21) ? IDE_SAT_ATA_ERROR : IDE_SAT_OK;',
     'return IDE_SAT_OK;'),
    ('SAT non-data: registers read while BSY', 'ide.c',
     '        if (!(st & 0x80)) {                                     // other bits only valid with BSY=0\n'
     '            sat_read_outputs(tf, st, regs);',
     '        if (1) {\n'
     '            sat_read_outputs(tf, st, regs);'),
    ('SAT: HOB bytes never read for a 48-bit command', 'ide.c',
     'if (tf->ext && !aborted) {', 'if (0) {'),
    ('SAT: HOB selected even when the drive aborted the command', 'ide.c',
     'if (tf->ext && !aborted) {', 'if (tf->ext) {'),
    ('SAT: HOB left set in Device Control', 'ide.c',
     '        ide_write_control(0x00);\n        r->hob = true;', '        r->hob = true;'),
    ('SAT: no stale DRQ check before issuing', 'ide.c',
     '    if (ide_read_reg(7) & 0x08) return false;   // stale DRQ\n', ''),
    ('SAT PIO: only the error register kept on an error', 'ide.c',
     'sat_read_outputs(tf, st, regs);             // before the drain changes anything',
     'regs->error = ide_read_reg(1);'),
    ('SAT: CK_COND success reported as GOOD, registers lost', 'sat.c',
     '        if (tf.ck_cond) {', '        if (0) {'),
    ('SAT descriptor: LBA mid carries LBA high', 'sat.c',
     '    d[17] = r->lba_mid;', '    d[17] = r->lba_high;'),
    ('SAT descriptor: EXTEND never set', 'sat.c',
     '    d[10] = r->hob ? 0x01 : 0x00;   // EXTEND', '    d[10] = 0x00;'),
    ('SAT descriptor: sent even when the sense is no longer ours', 'sat.c',
     '    if (!ours || bufsize < SAT_SENSE_LEN) return fixed;', '    if (bufsize < SAT_SENSE_LEN) return fixed;'),
    ('SAT descriptor: attached to a timeout (registers already reset)', 'sat.c',
     '        sense_plain(lun, SCSI_SENSE_ABORTED_COMMAND, 0x00, 0x00);\n        return -1;\n    }\n}',
     '        sense_with_regs(lun, SCSI_SENSE_ABORTED_COMMAND, 0x00, 0x00, &regs);\n        return -1;\n    }\n}'),
    ('READ(10) leaves a pending SAT descriptor in place', 'usb.c',
     '    (void)lun;\n#if ATABOY_SAT\n    sat_sense_forget();     // this command may set sense of its own\n#endif\n    if (!is_mounted) return -1;',
     '    (void)lun;\n    if (!is_mounted) return -1;'),
]


def run_tests(srcdir, outdir):
    env = dict(os.environ, OUT=outdir)
    p = subprocess.run(['sh', os.path.join(HERE, 'run.sh'), srcdir], env=env,
                       capture_output=True, text=True)
    return p


def main():
    base = run_tests(SRC, tempfile.mkdtemp(prefix='ataboy-host-'))
    last = base.stdout.strip().splitlines()[-1] if base.stdout.strip() else base.stderr[-500:]
    print(f'unmutated: rc={base.returncode}  {last}')
    if base.returncode != 0:
        print('the unmutated sources must pass first'); return 2
    bad_setup, survived, killed = [], [], []
    for name, fname, old, new in MUTANTS:
        tmp = tempfile.mkdtemp(prefix='ataboy-mut-')
        src = os.path.join(tmp, 'src')
        shutil.copytree(SRC, src)
        path = os.path.join(src, fname)
        text = open(path, 'rb').read().decode('utf-8').replace('\r\n', '\n')
        if text.count(old) != 1:
            bad_setup.append((name, f'text found {text.count(old)} times'))
            print(f'NOT APPLIED  {name}'); continue
        open(path, 'wb').write(text.replace(old, new).encode('utf-8'))
        p = run_tests(src, os.path.join(tmp, 'out'))
        out = p.stdout.strip().splitlines()
        summary = out[-1] if out else ''
        built = any(os.path.exists(os.path.join(tmp, 'out', 'test_read' + x)) for x in ('', '.exe'))
        if 'checks,' not in summary and built and p.returncode != 0:
            # Built, then crashed before its summary (a segfault, say). The
            # tests caught it, just not gracefully: counts as killed.
            killed.append(name)
            print(f'KILLED       {name}\n             crashed: {(p.stderr.strip().splitlines() or ["?"])[-1][:150]}')
            shutil.rmtree(tmp, ignore_errors=True); continue
        if 'checks,' not in summary:
            bad_setup.append((name, 'did not build or run: ' + (p.stderr.strip().splitlines() or ['?'])[-1]))
            print(f'NO RESULT    {name}'); continue
        if p.returncode != 0:
            killed.append(name)
            first = next((l for l in out if l.startswith('FAIL')), '')
            print(f'KILLED       {name}\n             {summary}; first: {first[:150]}')
        else:
            survived.append(name)
            print(f'SURVIVED     {name}  ({summary})')
        shutil.rmtree(tmp, ignore_errors=True)
    print(f'\n{len(killed)} killed, {len(survived)} survived, {len(bad_setup)} not run, of {len(MUTANTS)}')
    for n, why in bad_setup:
        print(f'  not run: {n}: {why}')
    return 0 if not survived and not bad_setup else 1


if __name__ == '__main__':
    sys.exit(main())
