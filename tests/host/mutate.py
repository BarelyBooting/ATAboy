"""Mutation check for the host tests.

Each mutant is a small source edit that breaks one property the tests are
meant to protect (never hand the host a failed or invented sector, keep the
good prefix, BSY before ERR, and so on). A mutant is KILLED when the tests
fail against it. A mutant that survives means the tests do not cover that
property. A mutant whose text is not found, or that does not compile, is
reported as such and makes the run fail, so it is never counted as killed.

    python tests/host/mutate.py            (CXX picks the compiler, default g++)
    python tests/host/mutate.py WORD ...   only the mutants whose name contains a WORD
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
    ('CHS geometry not restored after a reset', 'ide.c',
     '    rec.stage = REC_GEO;\n    if (recovery_ms(IDE_GEOMETRY_TIMEOUT_MS) < IDE_GEOMETRY_TIMEOUT_MS) return 0;\n    return recovery_end(ide_set_geometry(config.heads, config.spt));',
     '    return recovery_end(true);'),
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
     'while (((st = ide_read_reg(7)) & 0x80) && st != 0xFF) {', 'while ((st = ide_read_reg(7)) & 0x80) {'),
    ('slave selected while the master is still busy after SRST (review L2)', 'ide.c',
     'while (((st = ide_read_reg(7)) & 0x80) && st != 0xFF) {', 'while (0 && st) {'),
    ('master: no wait on device 0 after SRST (review: srst master wait skipped)', 'ide.c',
     '            while (ide_read_reg(7) & 0x80) {\n                if (ms_passed(start, limit)) return budget_ends_first ? 0 : -1;',
     '            while (0) {\n                if (ms_passed(start, limit)) return budget_ends_first ? 0 : -1;'),
    ('master handled like a slave after SRST (gives up waiting, selects anyway)', 'ide.c',
     '        if (dev_base == 0xA0) {\n            while (ide_read_reg(7) & 0x80) {', '        if (0) {\n            while (ide_read_reg(7) & 0x80) {'),
    ('device not selected again after a reset', 'ide.c',
     '        ide_write_reg(6, dev_base);\n        busy_wait_us_32(1);                 // 400 ns before status is valid\n        rec.selected = true;',
     '        busy_wait_us_32(1);                 // 400 ns before status is valid\n        rec.selected = true;'),
    ('geometry not marked lost by SRST', 'ide.c',
     '    chs_geometry_lost = true;               // SRST drops', '    // SRST drops'),
    ('CHS transfers never refuse on a lost geometry', 'ide.c',
     '    if (config.use_lba_mode || !chs_geometry_lost) return true;', '    return true;'),
    ('geometry restore ignores ABRT', 'ide.c',
     'bool ok = ide_wait_until_ready(recovery_ms(IDE_GEOMETRY_TIMEOUT_MS)) && !(ide_read_reg(7) & 0x01);',
     'bool ok = ide_wait_until_ready(recovery_ms(IDE_GEOMETRY_TIMEOUT_MS));'),
    ('reset recorded as fine when it failed', 'ide.c',
     '    last_fail.reset_failed = !ok;', '    last_fail.reset_failed = false;'),
    ('no geometry check before a write', 'ide.c',
     '    if (!chs_geometry_ok()) {\n        record_failure(IDE_FAIL_NO_GEOMETRY, 0x91, ide_read_reg(7), lba, 0, count);\n        return -1;\n    }\n'
     '    if (!time_to_send(lba, count)) return -1;              // as for a read',
     '    if (0) {\n        record_failure(IDE_FAIL_NO_GEOMETRY, 0x91, ide_read_reg(7), lba, 0, count);\n        return -1;\n    }\n'
     '    if (!time_to_send(lba, count)) return -1;              // as for a read'),
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
     '                sat_abort(tf, IDE_FAIL_BAD_END, st);',
     '            if (st & 0x08) {                                    // DRQ on a non-data command\n'
     '                ide_drain_sector();'),
    ('SAT non-data: no abort on timeout', 'ide.c',
     '            sat_abort(tf, IDE_FAIL_TIMEOUT, st);\n            return IDE_SAT_TIMEOUT;\n        }\n        busy_wait_us_32(10);\n    }\n}\n#endif',
     '            return IDE_SAT_TIMEOUT;\n        }\n        busy_wait_us_32(10);\n    }\n}\n#endif'),
    ('SAT non-data: ERR taken as success', 'ide.c',
     'return (st & 0x21) ? IDE_SAT_ATA_ERROR : IDE_SAT_OK;',
     'return IDE_SAT_OK;'),
    ('SAT non-data: registers read while BSY', 'ide.c',
     '        if (!(st & 0x80) && w.started) {                        // other bits only valid with BSY=0\n'
     '            sat_read_outputs(tf, st, regs);',
     '        if (w.started) {\n'
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
    # --- H1: the SAT policy gets the drive's own IDENTIFY words 82..84 ---
    # (the policy's own gates are mutated in test/run-sat-policy-tests.ps1)
    ('IDENTIFY words never marked valid', 'ide.c',
     '    id_words.valid = ok;\n', ''),
    ('a failed IDENTIFY keeps the last drive\'s words', 'ide.c',
     '    id_words.valid = ok;\n    if (ok) {', '    if (ok) {\n        id_words.valid = true;'),
    ('IDENTIFY words kept across a new probe', 'ide.c',
     '    id_words.valid = false;     // a new detection', '    // a new detection'),
    ('IDENTIFY words not tied to the device they came from', 'ide.c',
     'return id_words.valid && id_words.dev_base == dev_base; }', 'return id_words.valid; }'),
    ('word 83 kept as word 82', 'ide.c',
     'id_words.w82 = buf[82];', 'id_words.w82 = buf[83];'),
    ('word 83 kept as word 84', 'ide.c',
     'id_words.w84 = buf[84];', 'id_words.w84 = buf[83];'),
    # Not run, EQUIVALENT since ff9a3ed: 'policy told an IDENTIFY was captured
    # when none was' (sat.c in.id_captured = true). Every row that looks at
    # the words also goes through ide_id_words_verify(), which refuses when no
    # words are held, so the host sees the same refusal either way; the policy
    # is the first of two layers. The policy's own handling of "not captured"
    # is proven by the exhaustive policy test (run-sat-policy-tests.ps1).
    ('SAT: word 84 not passed to the policy', 'sat.c',
     '    in.id_w84 = id.w84;', '    in.id_w84 = 0x4001;'),
    # --- M1: a SAT command counts as ended only once it has visibly started ---
    ('SAT non-data: BSY clear taken as the end before the command started (M1 as found)', 'ide.c',
     '        if (!(st & 0x80) && w.started) {', '        if (!(st & 0x80)) {'),
    ('SAT PIO: a stale ERR taken as this command\'s', 'ide.c',
     '                if (w.started && (st & 0x21)) {', '                if (st & 0x21) {'),
    ('SAT: INTRQ never taken as evidence', 'ide.c',
     '    bool irq = w->irq_usable && gpio_get(IDE_INTRQ);   // before the status read releases it\n',
     '    bool irq = false;\n'),
    ('SAT: INTRQ sampled after the status read has released it', 'ide.c',
     '    bool irq = w->irq_usable && gpio_get(IDE_INTRQ);   // before the status read releases it\n'
     '    uint8_t st = ide_read_reg(7);\n',
     '    uint8_t st = ide_read_reg(7);\n'
     '    bool irq = w->irq_usable && gpio_get(IDE_INTRQ);\n'),
    ('SAT: INTRQ believed although it was high before the command', 'ide.c',
     '    w->irq_usable = !gpio_get(IDE_INTRQ);', '    w->irq_usable = true;'),
    ('SAT: BSY seen does not count as started', 'ide.c',
     '    if ((st & 0x80) || irq) w->started = true;', '    if (irq) w->started = true;'),
    ('SAT: no 1 us settle after the command (status read inside ATA\'s 400 ns)', 'ide.c',
     '    ide_write_reg(7, tf->command);\n    busy_wait_us_32(1);             // give the drive time to assert BSY (400 ns)\n',
     '    ide_write_reg(7, tf->command);\n'),
    ('SAT PIO: DRQ does not count as started', 'ide.c',
     '                if (st & 0x08) w.started = true;                // DRQ is this command\'s (sat_can_issue)\n', ''),
    # --- M2: DF (device fault) is an error on every SAT path ---
    ('SAT non-data: DF ignored, only ERR ends in error (review M2)', 'ide.c',
     'return (st & 0x21) ? IDE_SAT_ATA_ERROR : IDE_SAT_OK;', 'return (st & 0x01) ? IDE_SAT_ATA_ERROR : IDE_SAT_OK;'),
    ('SAT PIO: DF ignored at the end check (review M2)', 'ide.c',
     'if (st & 0x21) { sat_read_outputs(tf, st, regs); return IDE_SAT_ATA_ERROR; }',
     'if (st & 0x01) { sat_read_outputs(tf, st, regs); return IDE_SAT_ATA_ERROR; }'),
    ('SAT PIO: DF ignored while waiting for data', 'ide.c',
     '                if (w.started && (st & 0x21)) {', '                if (w.started && (st & 0x01)) {'),
    ('SAT: DF reported as a medium error (review M2)', 'sat.c',
     'else if (err == 0)   sense_with_regs(lun, SCSI_SENSE_HARDWARE_ERROR, 0x44, 0x00, &regs);',
     'else if (err == 0)   sense_with_regs(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00, &regs);'),
    # --- other properties from the 0.6f3p6 review's mutant list ---
    ('SAT PIO: registers read after the drain', 'ide.c',
     '                    sat_read_outputs(tf, st, regs);             // before the drain changes anything\n'
     '                    if (st & 0x08) ide_drain_sector();          // don\'t leave DRQ stranded',
     '                    if (st & 0x08) ide_drain_sector();          // don\'t leave DRQ stranded\n'
     '                    sat_read_outputs(tf, ide_read_reg(7), regs);'),
    ('sat_scsi does not forget a pending descriptor', 'sat.c',
     '    sat_sense_forget();\n\n    // tud_msc_scsi_cb()', '\n    // tud_msc_scsi_cb()'),
    ('SAT descriptor served more than once', 'sat.c',
     '    pending.valid = false;          // one delivery', '    // one delivery'),
    ('SAT descriptor ownership ignores ASC/ASCQ', 'sat.c',
     '(b[2] & 0x0F) == pending.key &&\n                b[12] == pending.asc && b[13] == pending.ascq;',
     '(b[2] & 0x0F) == pending.key;'),
    ('WRITE(10) leaves a pending SAT descriptor in place', 'usb.c',
     '    sat_sense_forget();     // this command may set sense of its own\n#endif\n'
     '    if (!is_mounted || config.drive_write_protected) return -1;',
     '#endif\n    if (!is_mounted || config.drive_write_protected) return -1;'),
    ('SAT: HOB bytes read with HOB clear', 'ide.c',
     '        ide_write_control(0x80);\n        r->hob_count', '        ide_write_control(0x00);\n        r->hob_count'),
    # --- gated SAT rows re-verify the drive (re-review M-A) ---
    ('gated rows skip the identity check', 'sat.c', '    if (v.needs_identity) {\n        int same', '    if (0) {\n        int same'),
    ('identity check ignores the serial', 'ide.c',
     '    for (int i = 0; same && i < 10; i++) same = buf[10 + i] == id_words.serial[i];\n', ''),
    ('identity check ignores the model', 'ide.c',
     '    for (int i = 0; same && i < 20; i++) same = buf[27 + i] == id_words.model[i];\n', ''),
    ('identity check ignores words 82..84', 'ide.c',
     'got > 0 && buf[82] == id_words.w82 &&\n                buf[83] == id_words.w83 && buf[84] == id_words.w84;',
     'got > 0;'),
    ('identity check issues IDENTIFY over stale DRQ', 'ide.c',
     '    if (!ide_wait_until_ready(1000) || (ide_read_reg(7) & 0x08)) return -1;\n', ''),
    ('identity check keeps the words on a mismatch', 'ide.c', '    if (!same) id_words.valid = false;\n', ''),
    ('not ready at the identity check answered as a refusal', 'sat.c', '        if (same < 0) {', '        if (0) {'),
    # Since 0.6f3p7 the rows come from the policy (needs_identity, review L-1);
    # each mutant exempts one command byte at the call site.
    ('READ SECTORS EXT not re-verified', 'sat.c', '    if (v.needs_identity) {', '    if (v.needs_identity && tf.command != 0x24) {'),
    ('READ NATIVE MAX not re-verified', 'sat.c', '    if (v.needs_identity) {', '    if (v.needs_identity && tf.command != 0xF8) {'),
    ('READ NATIVE MAX EXT not re-verified (review L-1)', 'sat.c', '    if (v.needs_identity) {', '    if (v.needs_identity && tf.command != 0x27) {'),
    ('SMART not re-verified (review L-1)', 'sat.c', '    if (v.needs_identity) {', '    if (v.needs_identity && tf.command != 0xB0) {'),
    # --- unmount forgets the IDENTIFY words (review follow-up to H1) ---
    ('unmount keeps the old drive IDENTIFY words', 'menus.c',
     '    ide_id_words_forget();          // only SAT builds keep the words\n', ''),
    # --- M3, L1, L3: firmware update mode (fwupdate.h, menus.c, usb.c) ---
    ('update key opens the prompt on any screen', 'fwupdate.h',
     'return on_main_menu && !mounted && key == FWUPDATE_KEY;', 'return !mounted && key == FWUPDATE_KEY;'),
    ('update key opens the prompt while mounted (review M3)', 'fwupdate.h',
     'return on_main_menu && !mounted && key == FWUPDATE_KEY;', 'return on_main_menu && key == FWUPDATE_KEY;'),
    ('update key back to B, which a split Down arrow gives (review L1)', 'fwupdate.h',
     '#define FWUPDATE_KEY        0x06', "#define FWUPDATE_KEY        'B'"),
    ('Y reboots while a drive is mounted (review M3)', 'fwupdate.h',
     '    if (mounted) return FWUPDATE_REFUSE_MOUNTED;\n', ''),
    ('Y reboots while a USB command is running (review L3)', 'fwupdate.h',
     '    if (!usb_busy) return FWUPDATE_GO;\n    return waited_ms', '    return FWUPDATE_GO;\n    return waited_ms'),
    ('Y waits for ever on a USB command that never ends', 'fwupdate.h',
     'return waited_ms < FWUPDATE_WAIT_MS ? FWUPDATE_WAIT : FWUPDATE_REFUSE_BUSY;', 'return FWUPDATE_WAIT;'),
    ('menus: update key taken on every screen', 'menus.c',
     'fwupdate_key_opens_prompt(current_screen == SCREEN_MAIN, is_mounted, k)', 'fwupdate_key_opens_prompt(true, is_mounted, k)'),
    ('menus: update key taken as if nothing were mounted (review M3)', 'menus.c',
     'fwupdate_key_opens_prompt(current_screen == SCREEN_MAIN, is_mounted, k)', 'fwupdate_key_opens_prompt(current_screen == SCREEN_MAIN, false, k)'),
    ('menus: the update prompt never opens', 'menus.c',
     '    current_screen = SCREEN_CONFIRM;\n    confirm_type = 5;\n    return true;', '    confirm_type = 5;\n    return true;'),
    ('menus: Y does not check mounted again (review M3)', 'menus.c',
     'fwupdate_step(is_mounted, usb_msc_ide_busy(), waited)', 'fwupdate_step(false, usb_msc_ide_busy(), waited)'),
    ('menus: Y ignores a running USB command (review L3)', 'menus.c',
     'fwupdate_step(is_mounted, usb_msc_ide_busy(), waited)', 'fwupdate_step(is_mounted, false, waited)'),
    ('menus: help row offers the old key', 'menus.c',
     'Ctrl+F: Firmware Update', 'B: Firmware Update'),
    ('usb: READ(10) not marked busy (review L3)', 'usb.c',
     '    msc_busy_begin();\n    host_cmd_enter(true, false, lba);', '    host_cmd_enter(true, false, lba);'),
    ('usb: WRITE(10) not marked busy (review L3)', 'usb.c',
     '    msc_busy_begin();\n    host_cmd_enter(true, true, lba);', '    host_cmd_enter(true, true, lba);'),
    ('usb: SAT pass-through not marked busy (review L3)', 'usb.c',
     '        msc_busy_begin();\n        host_cmd_enter(false, false, 0);', '        host_cmd_enter(false, false, 0);'),
    ('usb: busy flag never cleared', 'usb.c',
     '    msc_ide_busy = false;\n}', '}'),
    # --- 0.6f3p7: wall-clock limits (hardware finding 2026-09-24, ST380011A) ---
    ('write commit wait counts polls again (the defect as found)', 'ide.c',
     '        for (;;) {\n            if (ms_passed(start, limit)) break;   // write commit: still busy',
     '        for (uint32_t t = 0; ; t++) {\n            if (t >= 100000) break;   // write commit: still busy'),
    ('write DRQ wait counts polls again', 'ide.c',
     '        for (;;) {\n            if (ms_passed(start, limit)) break;   // write: no DRQ in time',
     '        for (uint32_t t = 0; ; t++) {\n            if (t >= 100000) break;   // write: no DRQ in time'),
    ('read DRQ wait counts polls again', 'ide.c',
     '        for (;;) {\n            if (ms_passed(start, limit)) goto read_timeout;',
     '        for (uint32_t t = 0; ; t++) {\n            if (t >= 100000) goto read_timeout;'),
    ('IDENTIFY wait counts polls again', 'ide.c',
     '    for (;;) {\n        if (ms_passed(start, limit)) return -1;',
     '    for (uint32_t t = 0; ; t++) {\n        if (t >= 100000) return -1;'),
    ('command limit 10 s instead of the budget less the reserve (15 s)', 'ide.c',
     '#define IDE_CMD_TIMEOUT_MS      (IDE_HOST_BUDGET_MS - IDE_RECOVERY_RESERVE_MS)', '#define IDE_CMD_TIMEOUT_MS      10000'),
    # Not run, EQUIVALENT in the firmware since review M-1: 'command limit 30 s
    # instead of 15 s'. Every READ or WRITE SECTORS the firmware sends goes out
    # inside a host command, where work_ms() cuts the limit to the budget less
    # the reserve (15 s at most) whatever IDE_CMD_TIMEOUT_MS says; core 1 sends
    # none. Only the host tests' direct calls would see it, and they measure
    # against the same macro. The budget itself is pinned by a check in
    # test_read's main (the mutant below).
    ('host command budget 30 s (the host\'s own timeout) instead of 20 s', 'ide.h',
     '#define IDE_HOST_BUDGET_MS        20000', '#define IDE_HOST_BUDGET_MS        30000'),
    ('IDENTIFY limit 5 s instead of 10 s', 'ide.c',
     '#define IDE_IDENTIFY_TIMEOUT_MS 10000', '#define IDE_IDENTIFY_TIMEOUT_MS 5000'),
    ('read limit restarts for every sector (per sector, not per command)', 'ide.c',
     '        for (;;) {\n            if (ms_passed(start, limit)) goto read_timeout;',
     '        start = ms_now();\n        for (;;) {\n            if (ms_passed(start, limit)) goto read_timeout;'),
    # --- 0.6f3p7: soft reset escalates once to a hardware reset ---
    ('no hardware reset when the soft reset fails', 'ide.c',
     '        if (r < 0) hw_reset_start();        // once per recovery: only from REC_SRST',
     '        if (r < 0) return recovery_end(false);'),
    ('hardware reset even when the soft reset worked', 'ide.c',
     '        if (r == 0) return 0;\n        if (r < 0) hw_reset_start();', '        if (r == 0) return 0;\n        hw_reset_start();'),
    ('hardware reset repeated until the drive answers (not once per recovery)', 'ide.c',
     '        if (r < 0) return recovery_end(false);\n        recal_start();', '        if (r < 0) { hw_reset_start(); return 0; }\n        recal_start();'),
    ('CHS geometry not restored after the hardware reset', 'ide.c',
     '    if (config.use_lba_mode) return recovery_end(true);', '    if (config.use_lba_mode || last_fail.hw_reset) return recovery_end(true);'),
    ('hardware reset not recorded', 'ide.c',
     '    last_fail.reset = true;\n    last_fail.hw_reset = true;\n}', '    last_fail.reset = true;\n}'),
    ('hardware reset carried into a later command\'s failure record', 'ide.c',
     '    last_fail.hw_reset = host.hw_resets > 0;', '    last_fail.hw_reset = last_fail.hw_reset || host.hw_resets > 0;'),
    ('RESET- pulse shorter than ATA\'s 25 us', 'ide.c',
     '    busy_wait_us_32(IDE_HW_RESET_LOW_US);\n', '    busy_wait_us_32(10);\n'),
    ('no RECALIBRATE after the hardware reset', 'ide.c',
     '    ide_write_reg(7, 0x10);\n    wait_after_command();\n    rec.stage = REC_RECAL;', '    rec.stage = REC_RECAL;'),
    ('device not selected again after the hardware reset (the finding: reads 0xFF)', 'ide.c',
     '        ide_write_reg(6, dev_base);\n        busy_wait_us_32(1);                 // 400 ns before status is valid',
     '        if (rec.stage != REC_HW) ide_write_reg(6, dev_base);\n        busy_wait_us_32(1);                 // 400 ns before status is valid'),
    ('SAT abort not recorded (review L-2)', 'ide.c',
     '    record_failure(kind, tf->command, st, sat_tf_lba(tf), 0, tf->sectors);\n    soft_reset_restore();', '    soft_reset_restore();'),
    ('SAT abort does not reset the drive', 'ide.c',
     '    record_failure(kind, tf->command, st, sat_tf_lba(tf), 0, tf->sectors);\n    soft_reset_restore();',
     '    record_failure(kind, tf->command, st, sat_tf_lba(tf), 0, tf->sectors);'),
    # --- 0.6f3p7: IDENTIFY believes ERR only once the command has started (review L-3) ---
    ('IDENTIFY: stale ERR taken as its answer', 'ide.c',
     '        if (w.started && (st & 0x01)) { if (st & 0x08) ide_drain_sector(); return 0; }',
     '        if (st & 0x01) { if (st & 0x08) ide_drain_sector(); return 0; }'),
    ('IDENTIFY: DRQ does not count as started (review R4)', 'ide.c',
     '        if (st & 0x08) w.started = true;\n        if (w.started && (st & 0x01))', '        if (w.started && (st & 0x01))'),
    # --- 0.6f3p7: SMART needs word 85 bit 0 (policy mutants in run-sat-policy-tests.ps1) ---
    ('word 85 not captured', 'ide.c', '        id_words.w85 = buf[85];', '        id_words.w85 = 0x4401;'),
    ('word 87 not captured', 'ide.c', '        id_words.w87 = buf[87];', '        id_words.w87 = 0x4000;'),
    ('SAT: word 85 not passed to the policy', 'sat.c', '    in.id_w85 = id.w85;', '    in.id_w85 = 0x4401;'),
    ('SAT: word 87 not passed to the policy', 'sat.c', '    in.id_w87 = id.w87;', '    in.id_w87 = 0x4000;'),
    ('identity check ignores words 85 and 87', 'ide.c',
     '    same = same && buf[85] == id_words.w85 && buf[87] == id_words.w87;\n', ''),
    # --- 0.6f3p7: core 1 waits for a running USB command (review L-5) ---
    ('Auto Detect does not wait for a USB command', 'menus.c',
     '    if (!bus_free_wait(false)) { needs_full_redraw = true; return false; }\n', ''),
    ('debug keys do not wait for a USB command', 'menus.c',
     '    if (!known || !bus_free_wait(true)) return;', '    if (!known) return;'),
    ('bus wait never gives up', 'menus.c',
     'while (usb_msc_ide_busy() && to_ms_since_boot(get_absolute_time()) - t0 < IDE_BUS_WAIT_MS)', 'while (usb_msc_ide_busy())'),
    ('bus wait goes on with the command after waiting (review L-1)', 'menus.c',
     '    bool still = usb_msc_ide_busy();', '    if (!usb_msc_ide_busy()) return true;\n    bool still = true;'),
    ('bus wait takes keys meanwhile (review L-1)', 'menus.c',
     '        sleep_ms(10);\n    bool still', '        (void)cdc_getchar_timeout_us(10000);\n    bool still'),
    ('bus wait shorter than a host command may run', 'menus.c',
     '#define IDE_BUS_WAIT_MS (IDE_HOST_BUDGET_MS + 5000u)', '#define IDE_BUS_WAIT_MS (IDE_HOST_BUDGET_MS / 2)'),
    ('Debug E does not show a pending reset (review M-1)', 'menus.c', '        if (f.pending)\n', '        if (0)\n'),
    # --- 0.6f3p7: Debug E and the version on the unit (review L-2) ---
    ('Debug E does not show the hardware reset', 'menus.c',
     'const char *rs = !f.reset ? "" : f.hw_reset ?', 'const char *rs = !f.reset ? "" : false ?'),
    ('SMART opt-in build shows the shipping banner', 'menus.c',
     '#if ATABOY_SAT && ATABOY_SAT_SMART_SAVES\n#define BANNER', '#if 0\n#define BANNER'),
    # --- review M-1 (0.6f3p7): one time budget per host command ---
    ('M-1: every callback taken as a new host command', 'usb.c',
     'if (!rw || !rw_cmd.open || rw_cmd.write != write || lba != rw_cmd.next_lba)\n        ide_host_cmd_begin();',
     'ide_host_cmd_begin();'),
    ('M-1: a READ(10) complete callback does not end the command', 'usb.c',
     'void tud_msc_read10_complete_cb(uint8_t lun) { (void)lun; rw_cmd.open = false; }',
     'void tud_msc_read10_complete_cb(uint8_t lun) { (void)lun; }'),
    ('M-1: a USB reset does not end the command', 'usb.c',
     'void tud_mount_cb(void) { rw_cmd.open = false; }', 'void tud_mount_cb(void) { }'),
    ('M-1: continuation not checked for direction', 'usb.c',
     '!rw_cmd.open || rw_cmd.write != write || lba', '!rw_cmd.open || lba'),
    ('M-1: continuation not checked for LBA', 'usb.c',
     ' || lba != rw_cmd.next_lba)', ')'),
    ('M-1: no budget (waits uncut, as in the 0.6f3p7 draft)', 'ide.c',
     '    if (!host.on) return UINT32_MAX;', '    return UINT32_MAX;'),
    ('M-1: work waits do not keep the recovery reserve', 'ide.c',
     '    left = left > IDE_RECOVERY_RESERVE_MS ? left - IDE_RECOVERY_RESERVE_MS : 0;\n', ''),
    ('M-1: recovery reserve 0', 'ide.h',
     '#define IDE_RECOVERY_RESERVE_MS   5000', '#define IDE_RECOVERY_RESERVE_MS   0'),
    ('M-1: recovery waits not cut to the budget', 'ide.c',
     '    uint32_t left = host_left_ms();\n    return want_ms < left ? want_ms : left;', '    return want_ms;'),
    ('M-1: a pending recovery not carried on first (commands sent over it)', 'ide.c',
     '    if (rec.stage == REC_NONE) return true;\n    return recovery_run() > 0;', '    return true;'),
    ('M-1: SAT sends with a recovery pending', 'ide.c',
     '    if (!recovery_gate() || !sat_time_to_send(tf)) return false;', '    if (!sat_time_to_send(tf)) return false;'),
    ('M-1: a command sent with no time left to reset after it', 'ide.c',
     '    if (work_ms(need) >= need) return true;', '    return true;'),
    ('M-1: no time recorded over the failure that caused it (issue #13 call)', 'ide.c',
     '    if (!host.recorded) record_failure(IDE_FAIL_NO_TIME, 0, ide_read_reg(7), lba, 0, count);',
     '    record_failure(IDE_FAIL_NO_TIME, 0, ide_read_reg(7), lba, 0, count);'),
    ('M-1: a re-detect leaves the recovery pending', 'ide.c',
     '    recovery_forget();          // the probe\'s own reset supersedes any recovery still waiting\n', ''),
    # --- review of 0.6f3p7 (MEDIUM): never send a command without a fair window ---
    ('MW: a command sent with any work time at all (the b073832 rule, E2 to E5)', 'ide.c',
     '    if (work_ms(need) >= need) return true;', '    if (work_ms(1) > 0) return true;'),
    ('MW: the window is n sectors, not n + 1', 'ide.c',
     '    uint32_t need = IDE_SECTOR_ALLOW_MS * (count + 1);', '    uint32_t need = IDE_SECTOR_ALLOW_MS * count;'),
    ('MW: 1 s a sector (the 0.6f3p6 polls without the status reads)', 'ide.c',
     '#define IDE_SECTOR_ALLOW_MS 1100', '#define IDE_SECTOR_ALLOW_MS 1000'),
    ('MW: read not checked again after the ready wait', 'ide.c',
     '    // Still a fair window after waiting for the drive to be ready?\n    if (!time_to_send(lba, count)) return -1;\n', ''),
    ('MW: write not checked again after the ready wait and 0x91', 'ide.c',
     '    if (!time_to_send(lba, count)) return -1;              // as for a read\n', ''),
    ('MW: the call again at a failed sector not given the time it took (E3)', 'ide.c',
     '        need += host.fail_ms - IDE_SECTOR_ALLOW_MS;     // issue #13: the failed sector again', '        ;'),
    ('MW: a failed sector time held against later host commands', 'ide.c',
     '    host.recorded = false;\n    host.failed = false;', '    host.recorded = false;'),
    ('MW: time of an ERR sector not kept', 'ide.c',
     '    note_failed_sector(lba + s, ms_now() - sector_start);\n    record_failure(IDE_FAIL_ERR,', '    record_failure(IDE_FAIL_ERR,'),
    ('X15: NO_TIME never recorded', 'ide.c',
     '    if (!host.recorded) record_failure(IDE_FAIL_NO_TIME, 0, ide_read_reg(7), lba, 0, count);\n', ''),
    ('X8: SAT sent with any work time (not its whole 10 s)', 'ide.c',
     '    if (work_ms(SAT_CMD_TIMEOUT_MS) >= SAT_CMD_TIMEOUT_MS) return true;', '    if (work_ms(1) > 0) return true;'),
    ('MW: SAT not checked again after the ready wait', 'ide.c',
     '    if (ide_read_reg(7) & 0x08) return false;   // stale DRQ\n    return sat_time_to_send(tf);',
     '    if (ide_read_reg(7) & 0x08) return false;   // stale DRQ\n    return true;'),
    ('MW: SAT refusal for time not recorded', 'ide.c',
     '    if (!host.recorded) record_failure(IDE_FAIL_NO_TIME, tf->command,', '    if (0) record_failure(IDE_FAIL_NO_TIME, tf->command,'),
    ('X4: identity check starts without time for a full IDENTIFY', 'ide.c',
     '    if (work_ms(IDE_IDCHECK_MS) < IDE_IDCHECK_MS) {', '    if (0) {'),
    # --- review of 0.6f3p7: LOW items and surviving mutants ---
    ('X1: ide_reset_drive does not end a pending recovery', 'ide.c',
     '    recovery_forget();          // this reset supersedes any recovery still waiting\n', ''),
    ('LOW: 0x91 sent with whatever is left (the LOW as found)', 'ide.c',
     '    if (recovery_ms(IDE_GEOMETRY_TIMEOUT_MS) < IDE_GEOMETRY_TIMEOUT_MS) return 0;\n', ''),
    ('LOW: an unmounted callback enters a host command (the LOW as found)', 'usb.c',
     '    if (!is_mounted) return;\n    if (!rw', '    if (!rw'),
    ('LOW: IORDY setting put on the pin while it is held', 'ide.c',
     '    if (!iordy_held) ide_set_iordy(config.iordy_enabled);\n}', '    ide_set_iordy(config.iordy_enabled);\n}'),
    ('LOW: a host command does not put the IORDY setting on the pin', 'ide.c',
     '    host.on = true;\n    ide_iordy_follow_config();', '    host.on = true;'),
    ('LOW: Features sets the IORDY pin itself (the LOW as found)', 'menus.c',
     '    if (!is_mounted && !usb_msc_ide_busy()) ide_iordy_follow_config();', '    ide_set_iordy(config.iordy_enabled);'),
    ('LOW: Features asks ide.c for the pin while mounted', 'menus.c',
     '    if (!is_mounted && !usb_msc_ide_busy()) ide_iordy_follow_config();', '    ide_iordy_follow_config();'),
    ('X10: write10_complete_cb does nothing', 'usb.c',
     'void tud_msc_write10_complete_cb(uint8_t lun) { (void)lun; rw_cmd.open = false; }',
     'void tud_msc_write10_complete_cb(uint8_t lun) { (void)lun; }'),
    ('X11: scsi_complete_cb does nothing', 'usb.c',
     '    (void)lun; (void)scsi_cmd;\n    rw_cmd.open = false;', '    (void)lun; (void)scsi_cmd;'),
    # Not run, EQUIVALENT (review of 0.6f3p7, the reviewer's X2, X14, X17, X18,
    # X19). Each cuts, or keeps open, something the rules above already make
    # unreachable; none changes what any caller can see.
    #  X2  'negative return leaves the command open' (usb.c host_cmd_leave).
    #      After a negative return TinyUSB fails the command and sends its CSW,
    #      and the complete callback then closes it anyway. Only a bulk-only
    #      reset between the two could tell, and then the next READ(10) at
    #      exactly that LBA inherits what is left of the budget, which is
    #      already what an abandoned command does by design (usb.c).
    #  X14 'SAT end wait uses SAT_END_TIMEOUT_MS uncut'. The end wait starts
    #      after the last block, which is read before the command's limit,
    #      the end of the work time; the recovery reserve (5 s) is then left,
    #      more than the 1 s wait.
    #  X17 'identify_run ready wait not cut'. Inside a host command only the
    #      identity check calls it, and only with 12 s of work time left.
    #  X18 'set_geometry wait not cut'. Inside a host command 0x91 goes out
    #      only from recovery_run with its whole 1 s left (the LOW above), or
    #      from chs_geometry_ok, which a read or write reaches with at least
    #      the 5 s reserve left.
    #  X19 'idle_after_error not cut'. ERR is only seen before the command's
    #      limit, the end of the work time, so the 5 s reserve is left, more
    #      than the 2 s wait. A check in test_read's main holds the reserve.
    # --- review 0.6f3p7 LOW items and surviving mutants R1 to R7 ---
    ('L-2: reset flags not kept for the whole host command', 'ide.c',
     '    last_fail.hw_reset = host.hw_resets > 0;', '    last_fail.hw_reset = false;'),
    ('R1: IORDY never believed again after RESET-', 'ide.c',
     '            iordy_release();\n            return true;', '            return true;'),
    ('R2: IORDY not ignored during RESET-', 'ide.c',
     'static void hw_reset_start(void) {\n    iordy_hold();\n', 'static void hw_reset_start(void) {\n'),
    ('L-3: IORDY believed again as soon as RESET- ends (the 0.6f3p7 draft)', 'ide.c',
     '    ide_write_control(0x00);                // nIEN=0, as the probe does\n    rec.stage = REC_HW;',
     '    ide_write_control(0x00);                // nIEN=0, as the probe does\n    iordy_release(); iordy_held = false; ide_set_iordy(config.iordy_enabled);\n    rec.stage = REC_HW;'),
    ('R3: RECALIBRATE completion not waited for', 'ide.c',
     'static int recal_wait(uint32_t budget_ms) {\n', 'static int recal_wait(uint32_t budget_ms) {\n    return 1;\n'),
    ('L-4: RECALIBRATE idle status taken before it has started', 'ide.c',
     '        bool over = rec.recal.started || ms_passed(rec.since, IDE_RECAL_GRACE_MS);', '        bool over = true;'),
    ('L-4: a RECALIBRATE still busy after 10 s taken as done', 'ide.c',
     '        if (ms_passed(start, limit)) return budget_ends_first ? 0 : -1;\n        busy_wait_us_32(10);\n    }\n}\n\n// The recovery is over',
     '        if (ms_passed(start, limit)) return budget_ends_first ? 0 : 1;\n        busy_wait_us_32(10);\n    }\n}\n\n// The recovery is over'),
    ('R5: RESET- held 30 us (ATA\'s minimum, not the probe\'s 50 ms)', 'ide.c',
     '#define IDE_HW_RESET_LOW_US 50000', '#define IDE_HW_RESET_LOW_US 30'),
    ('R6: no Device Control write after RESET-', 'ide.c',
     '    ide_write_control(0x00);                // nIEN=0, as the probe does\n    rec.stage = REC_HW;', '    rec.stage = REC_HW;'),
    ('R7: deadline a millisecond late (> instead of >=)', 'ide.c',
     'return ms_now() - start >= limit_ms;', 'return ms_now() - start > limit_ms;'),
    # --- 0.6f3p8: manual CHS with no IDENTIFY (Ctrl+G; test_manual_chs.cpp) ---
    ('mchs: key opens on any screen', 'manualchs.h',
     'return on_main_menu && !mounted && key == MANUAL_CHS_KEY;', 'return !mounted && key == MANUAL_CHS_KEY;'),
    ('mchs: key opens while mounted', 'manualchs.h',
     'return on_main_menu && !mounted && key == MANUAL_CHS_KEY;', 'return on_main_menu && key == MANUAL_CHS_KEY;'),
    ('mchs: key is a letter a split sequence can give', 'manualchs.h',
     '#define MANUAL_CHS_KEY      0x07', "#define MANUAL_CHS_KEY      'M'"),
    ('mchs: menus take the key as if nothing were mounted', 'menus.c',
     'manual_chs_key_opens(current_screen == SCREEN_MAIN, is_mounted, k)', 'manual_chs_key_opens(current_screen == SCREEN_MAIN, false, k)'),
    ('mchs: the key never reaches the entry', 'menus.c',
     '        if (manual_chs_key(k)) { if (run_manual_chs()) trigger_overlay = true; continue; }\n', ''),
    ('mchs: cylinders above 65535 accepted (wrap)', 'manualchs.h',
     '    if (c < 1 || c > MANUAL_CHS_MAX_CYLS) return 0;', '    if (c < 1) return 0;'),
    ('mchs: 0 cylinders accepted', 'manualchs.h',
     '    if (c < 1 || c > MANUAL_CHS_MAX_CYLS) return 0;', '    if (c > MANUAL_CHS_MAX_CYLS) return 0;'),
    ('mchs: heads above 16 accepted (cut to 4 bits)', 'manualchs.h',
     '    if (h < 1 || h > MANUAL_CHS_MAX_HEADS) return 1;', '    if (h < 1 || h > 255) return 1;'),
    ('mchs: 0 heads accepted', 'manualchs.h',
     '    if (h < 1 || h > MANUAL_CHS_MAX_HEADS) return 1;', '    if (h > MANUAL_CHS_MAX_HEADS) return 1;'),
    ('mchs: sectors above 255 accepted (wrap)', 'manualchs.h',
     '    if (s < 1 || s > MANUAL_CHS_MAX_SPT) return 2;', '    if (s < 1) return 2;'),
    ('mchs: 0 sectors accepted', 'manualchs.h',
     '    if (s < 1 || s > MANUAL_CHS_MAX_SPT) return 2;', '    if (s > MANUAL_CHS_MAX_SPT) return 2;'),
    ('mchs: head limit off by one', 'manualchs.h',
     '#define MANUAL_CHS_MAX_HEADS 16u', '#define MANUAL_CHS_MAX_HEADS 15u'),
    ('mchs: the range check is not used', 'menus.c',
     '        int bad = manual_chs_bad_field(v[0], v[1], v[2]);', '        int bad = -1;'),
    ('mchs: a refusal does not say why', 'menus.c',
     '        if (bad >= 0) { msg = manual_chs_refusal(bad); active = bad; continue; }', '        if (bad >= 0) { active = bad; continue; }'),
    ('mchs: no Y: sent as soon as the entry is in range', 'menus.c',
     '        draw_manual_chs(fields, -1, q, true);\n        int k;\n        do {', '        draw_manual_chs(fields, -1, q, true);\n        int k = \'Y\';\n        if (0) do {'),
    ('mchs: Esc at the question sends it', 'menus.c',
     '        if (k == KEY_ESC) return false;\n        if (k == \'n\'', '        if (k == \'n\''),
    ('mchs: N at the question sends it', 'menus.c',
     "        if (k == 'n' || k == 'N') { msg = \"\"; continue; }\n", ''),
    ('mchs: Esc while typing applies the fields (the old manual entry defect)', 'menus.c',
     '        if (ch == KEY_ESC) return false;                      // nothing sent, nothing changed', '        if (ch == KEY_ESC) break;'),
    ('mchs: a reset pending at entry is ignored', 'menus.c',
     '    needs_full_redraw = true;\n    if (mchs_pending_refused()) return false;\n', '    needs_full_redraw = true;\n'),
    ('mchs: a reset left pending while typing is ignored', 'menus.c',
     '    if (!bus_free_wait(false) || mchs_pending_refused()) return false;', '    if (!bus_free_wait(false)) return false;'),
    ('mchs: Y does not wait for a USB command', 'menus.c',
     '    if (!bus_free_wait(false) || mchs_pending_refused()) return false;', '    if (mchs_pending_refused()) return false;'),
    ('mchs: a refused 0x91 is shown as set (the defect in the older routes)', 'menus.c',
     '    int r = ide_manual_chs((uint8_t)v[1], (uint8_t)v[2], &st);\n    if (r == IDE_MCHS_OK) {', '    int r = ide_manual_chs((uint8_t)v[1], (uint8_t)v[2], &st);\n    if (1) {'),
    ('mchs: a failure keeps the geometry from before the reset', 'menus.c',
     '    cur_cyls = 0; cur_heads = 0; cur_spt = 0; use_lba_mode = false; total_lba_sectors = 0;\n    sync_to_config();\n    force_detect = false;',
     '    force_detect = false;'),
    ('mchs: a failure draws no result box', 'menus.c',
     '    force_detect = false;\n    show_detect_result = true;\n    return true;', '    force_detect = false;\n    return false;'),
    ('mchs: a failure offers F: Force', 'menus.c',
     '    force_detect = false;\n    show_detect_result = true;\n    return true;', '    force_detect = true;\n    show_detect_result = true;\n    return true;'),
    ('mchs: success does not name the route in Current HDD', 'menus.c',
     '        strcpy(hdd_model_raw, MCHS_MODEL);\n', ''),
    ('mchs: success saved to EEPROM', 'menus.c',
     '        strcpy(hdd_model_raw, MCHS_MODEL);\n        sync_to_config();\n', '        strcpy(hdd_model_raw, MCHS_MODEL);\n        sync_to_config(); config_save();\n'),
    ('mchs: help row does not offer Ctrl+G', 'menus.c',
     '"  Ctrl+G: Manual CHS     "', '"                         "'),
    ('mchs: Debug E shows the record as a sector failure', 'menus.c',
     '        if (f.kind == IDE_FAIL_MANUAL_CHS)      // Ctrl+G: no sector was asked for', '        if (0)'),
    ('ide mchs: IDENTIFY sent first (as Auto Detect and the force route do)', 'ide.c',
     '    iordy_release();                // past its power-on diagnostics\n', '    iordy_release();                // past its power-on diagnostics\n    { uint16_t idb[256]; (void)ide_identify(idb); }\n'),
    ('ide mchs: no RECALIBRATE', 'ide.c',
     '    ide_write_reg(7, 0x10);\n    wait_after_command();\n    if (!mchs_wait_end(', '    if (0) wait_after_command();\n    if (!mchs_wait_end('),
    ('ide mchs: 0x91 before RECALIBRATE', 'ide.c',
     '    sat_watch_arm(&w);              // the status reads above released INTRQ\n', '    (void)ide_set_geometry(heads, spt);\n    sat_watch_arm(&w);              // the status reads above released INTRQ\n'),
    ('ide mchs: no hardware reset', 'ide.c',
     '    gpio_put(IDE_RESET, 0);\n    sleep_ms(IDE_HW_RESET_LOW_US / 1000);\n    gpio_put(IDE_RESET, 1);\n    chs_geometry_lost',
     '    chs_geometry_lost'),
    ('ide mchs: RESET- pulse too short to count', 'ide.c',
     '    sleep_ms(IDE_HW_RESET_LOW_US / 1000);\n    gpio_put(IDE_RESET, 1);\n    chs_geometry_lost', '    busy_wait_us_32(10);\n    gpio_put(IDE_RESET, 1);\n    chs_geometry_lost'),
    ('ide mchs: waits for DRDY after the reset (a pre-ATA drive never sets it)', 'ide.c',
     '        if (!(st & 0x80)) break;\n        if (ms_passed(t0, IDE_MCHS_READY_MS)) { *status = st; return IDE_MCHS_BUSY; }',
     '        if (!(st & 0x80) && (st & 0x40)) break;\n        if (ms_passed(t0, IDE_MCHS_READY_MS)) { *status = st; return IDE_MCHS_BUSY; }'),
    ('ide mchs: the probe\'s 10 s for spin-up (the CP3044 may take 40)', 'ide.h',
     '#define IDE_MCHS_READY_MS   45000', '#define IDE_MCHS_READY_MS   10000'),
    ('ide mchs: a slave waits 45 s on an absent master (FFh on device 0)', 'ide.c',
     '    while ((st = ide_read_reg(7)) != 0xFF && (st & 0x80)) {', '    while ((st = ide_read_reg(7)) & 0x80) {'),
    ('ide mchs: FFh taken as a busy drive', 'ide.c',
     '        if (st == 0xFF) { *status = st; return IDE_MCHS_NO_DEVICE; }\n', ''),
    ('ide mchs: a stale ERR is 0x91\'s answer (no started rule)', 'ide.c',
     '        bool over = w->started || ms_passed(t0, IDE_RECAL_GRACE_MS);', '        bool over = true;'),
    ('ide mchs: an ERR from RECALIBRATE stops it', 'ide.c',
     '        if (over && !(*st & 0x88)) return true;', '        if (over && !(*st & 0x89)) return true;'),
    ('ide mchs: 0x91 ERR taken as accepted', 'ide.c',
     '    bool ok = ended && (st & 0x40) && !(st & 0x01);', '    bool ok = ended && (st & 0x40);'),
    ('ide mchs: 0x91 not ready taken as accepted', 'ide.c',
     '    bool ok = ended && (st & 0x40) && !(st & 0x01);', '    bool ok = !(st & 0x01);'),
    ('ide mchs: heads sent as they are, not heads - 1', 'ide.c',
     '    ide_write_reg(6, dev_base | ((heads - 1) & 0x0F));\n    ide_write_reg(2, spt);\n    sat_watch_arm(&w);',
     '    ide_write_reg(6, dev_base | (heads & 0x0F));\n    ide_write_reg(2, spt);\n    sat_watch_arm(&w);'),
    ('ide mchs: geometry still marked lost after 0x91 accepted', 'ide.c',
     '    chs_geometry_lost = !ok;\n    *status = st;', '    *status = st;'),
    ('ide mchs: a refused 0x91 not recorded for Debug E', 'ide.c',
     '        record_failure(IDE_FAIL_MANUAL_CHS, 0x91, st, 0, 0, 0);\n', ''),
    ('ide mchs: a RECALIBRATE that never ends not recorded', 'ide.c',
     '        record_failure(IDE_FAIL_MANUAL_CHS, 0x10, st, 0, 0, 0);\n', ''),
    ('ide mchs: IDENTIFY words from an earlier detection kept', 'ide.c',
     '    id_words.valid = false;         // this drive is never asked who it is\n', ''),
    ('ide mchs: bad heads not refused by ide.c', 'ide.c',
     '    if (heads < 1 || heads > 16 || spt < 1) return IDE_MCHS_BAD_ARGS;\n', ''),
    ('ide mchs: recovery pending never reported', 'ide.c',
     'bool ide_recovery_pending(void) { return rec.stage != REC_NONE; }', 'bool ide_recovery_pending(void) { return false; }'),
    # --- review of 0.6f3p8 ---
    # M-1: every reset marks the CHS geometry lost, so the next CHS transfer
    # sends 0x91 first (ide.c, chs_geometry_lost has the table). Not listed:
    # hw_reset_start()'s own line. It is an equivalent mutant: the recovery
    # only reaches a hardware reset from REC_SRST, after soft_reset_restore()
    # has set the flag, and nothing clears it in between (only a 0x91 the
    # drive accepts does, and none is sent before REC_GEO). The line is there
    # so that no reset path depends on another having run first.
    ('M-1: Debug R (ide_reset_drive) does not mark the geometry lost (review T21)', 'ide.c',
     '    chs_geometry_lost = true;   // RESET- drops 0x91 (review of 0.6f3p8, M-1)\n', ''),
    ('M-1: the probe does not mark the geometry lost', 'ide.c',
     '    chs_geometry_lost = true; // RESET- drops 0x91 (review of 0.6f3p8, M-1)\n', ''),
    ('M-1: geometry believed at power-up', 'ide.c',
     'static bool chs_geometry_lost = true;', 'static bool chs_geometry_lost = false;'),
    ('M-1: Ctrl+G RESET- does not mark the geometry lost (review N11)', 'ide.c',
     '    chs_geometry_lost = true;       // RESET- drops any geometry the drive had\n', ''),
    ('M-1: Auto Detect keeps the old geometry (review P2)', 'menus.c',
     '    cur_cyls = 0; cur_heads = 0; cur_spt = 0; use_lba_mode = false; total_lba_sectors = 0;\n    sync_to_config();\n    id_w49_known = false;',
     '    id_w49_known = false;'),
    ('M-1: Auto Detect clears the geometry on screen, not in config', 'menus.c',
     '    cur_cyls = 0; cur_heads = 0; cur_spt = 0; use_lba_mode = false; total_lba_sectors = 0;\n    sync_to_config();\n    id_w49_known = false;',
     '    cur_cyls = 0; cur_heads = 0; cur_spt = 0; use_lba_mode = false; total_lba_sectors = 0;\n    id_w49_known = false;'),
    ('M-1: 0x91 sent with no geometry at all', 'ide.c',
     '    if (config.heads == 0 || config.spt == 0) return false;\n', ''),
    # The review's surviving mutants, as it wrote them.
    ('Ctrl+G: Enter also confirms (review N3)', 'menus.c',
     "        while (k != 'y' && k != 'Y' && k != 'n' && k != 'N' && k != KEY_ESC);",
     "        while (k != 'y' && k != 'Y' && k != 'n' && k != 'N' && k != KEY_ESC && k != KEY_ENTER); if (k == KEY_ENTER) k = 'y';"),
    ('Ctrl+G: success leaves LBA mode (review N5)', 'menus.c',
     '        use_lba_mode = false; total_lba_sectors = 0;\n        cur_cyls = (uint16_t)v[0];', '        cur_cyls = (uint16_t)v[0];'),
    ('ide mchs: no IORDY hold over the reset (review N7)', 'ide.c',
     '    recovery_forget();              // RESET- supersedes any recovery (none is pending: menus.c)\n    iordy_hold();',
     '    recovery_forget();              // RESET- supersedes any recovery (none is pending: menus.c)'),
    ('write proceeds with a recovery pending (review T19)', 'ide.c',
     '    if (!recovery_gate()) return -1;                       // as for a read (review M-1)',
     '    (void)recovery_gate();'),
    # --- 0.6f3p9, A: the USB serial number is the board's own (test_usb_desc) ---
    ('serial: the upstream constant again', 'usb_descriptors.c',
     '        const char *str = index == 3 ? usb_serial_string() : string_desc_arr[index];',
     '        const char *str = index == 3 ? "654321" : string_desc_arr[index];'),
    ('serial: string 3 not replaced by the board id', 'usb_descriptors.c',
     '        const char *str = index == 3 ? usb_serial_string() : string_desc_arr[index];',
     '        const char *str = string_desc_arr[index];'),
    ('serial: board id cut to half its digits', 'usb_descriptors.c',
     '    pico_get_unique_board_id_string(usb_serial, sizeof usb_serial);',
     '    pico_get_unique_board_id_string(usb_serial, PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1);'),
    # --- 0.6f3p9, B: the IORDY advisory (test_manual_chs, test_tui) ---
    ('IORDY note: shown with IORDY off', 'menus.c',
     '    return known && config.iordy_enabled && !(w49 & 0x0800);', '    return known && !(w49 & 0x0800);'),
    ('IORDY note: bit 10 taken for bit 11', 'menus.c',
     '    return known && config.iordy_enabled && !(w49 & 0x0800);', '    return known && config.iordy_enabled && !(w49 & 0x0400);'),
    ('IORDY note: shown for a drive that declares IORDY', 'menus.c',
     '    return known && config.iordy_enabled && !(w49 & 0x0800);', '    return known && config.iordy_enabled;'),
    ('IORDY note: never on the main screen', 'menus.c',
     '                        : "\\033[20;3H F10: Save Current Setup to EEPROM  Ctrl+F: Firmware Update  Enter: Select");\n\n    draw_iordy_note(2, IORDY_NOTE_ROW);\n',
     '                        : "\\033[20;3H F10: Save Current Setup to EEPROM  Ctrl+F: Firmware Update  Enter: Select");\n\n'),
    ('IORDY note: never on the picker', 'menus.c',
     '    if (iordy_note_due(true, id[49]))', '    if (0)'),
    ('IORDY note: word 49 kept from before when IDENTIFY fails', 'menus.c',
     '    id_w49_known = false;           // IORDY advisory: only this detection\'s IDENTIFY counts\n', ''),
    ('IORDY note: Auto Detect never records word 49', 'menus.c',
     '    if (detected) { id_w49 = id_buf[49]; id_w49_known = true; }\n', ''),
    ('IORDY note: kept after Ctrl+G', 'menus.c',
     '    id_w49_known = false;           // no IDENTIFY: nothing to say about IORDY\n', ''),
    ('IORDY note: auto-mount does not record word 49', 'menus.c',
     '    id_w49 = id_buf[49]; id_w49_known = true;   // IORDY advisory\n', ''),
    # L-2: nothing through SAT to a drive set up by Ctrl+G.
    ('L-2: SAT not told about Ctrl+G', 'sat.c',
     '    in.manual_chs = ide_manual_chs_active();', '    in.manual_chs = false;'),
    ('L-2: an accepted Ctrl+G does not set the flag', 'ide.c',
     '    manual_chs_active = true;       // no pass-through to this drive (sat_policy.h)\n', ''),
    ('L-2: an IDENTIFY that answers does not clear the flag', 'ide.c',
     '    if (ok) manual_chs_active = false;\n', ''),
    ('L-2: an IDENTIFY that fails clears the flag too', 'ide.c',
     '    if (ok) manual_chs_active = false;', '    manual_chs_active = false;'),
    # L-1: F10 does not save a Ctrl+G geometry with Auto Mount on.
    ('L-1: F10 saves a Ctrl+G geometry with Auto Mount on', 'menus.c',
     '    if (config.auto_mount && ide_manual_chs_active()) {', '    if (0) {'),
    ('L-1: F10 refuses any Ctrl+G geometry', 'menus.c',
     '    if (config.auto_mount && ide_manual_chs_active()) {', '    if (ide_manual_chs_active()) {'),
    ('L-1: F10 refuses whenever Auto Mount is on', 'menus.c',
     '    if (config.auto_mount && ide_manual_chs_active()) {', '    if (config.auto_mount) {'),
]


# run.sh builds and runs these in order, each ending with "N checks, M failed".
# Mutants run with SAT on, where run.sh builds test_read and test_fwupdate
# twice each (with and without the SMART opt-in). This list said two programs
# while run.sh ran three from 0a6cb06 on; a surviving mutant would then have
# been reported as "not run" instead of SURVIVED (fixed in 0.6f3p7).
PROGRAMS = ['test_read', 'test_read_shipping', 'test_fwupdate', 'test_fwupdate_smart', 'test_manual_chs',
            'test_usb_desc']


def run_tests(srcdir, outdir):
    env = dict(os.environ, OUT=outdir)
    try:
        return subprocess.run(['sh', os.path.join(HERE, 'run.sh'), srcdir], env=env,
                              capture_output=True, text=True, timeout=900)
    except subprocess.TimeoutExpired:
        return None


def main():
    base = run_tests(SRC, tempfile.mkdtemp(prefix='ataboy-host-'))
    if base is None:
        print('the unmutated sources timed out'); return 2
    last = ' / '.join(l for l in base.stdout.splitlines() if 'checks,' in l) or base.stderr[-500:]
    print(f'unmutated: rc={base.returncode}  {last}')
    if base.returncode != 0:
        print('the unmutated sources must pass first'); return 2
    bad_setup, survived, killed = [], [], []
    words = sys.argv[1:]
    chosen = [m for m in MUTANTS if not words or any(w in m[0] for w in words)]
    for name, fname, old, new in chosen:
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
        if p is None:
            # A test that never finishes did not pass: the mutant is caught.
            killed.append(name)
            print(f'KILLED       {name}\n             timed out after 900 s')
            shutil.rmtree(tmp, ignore_errors=True); continue
        out = p.stdout.strip().splitlines()
        sums = [l for l in out if 'checks,' in l]
        summary = ' / '.join(sums)
        reported_failure = any(not l.endswith(' 0 failed') for l in sums)
        if p.returncode != 0 and not reported_failure:
            # run.sh stopped before every program reported. If the next one
            # was built, it crashed before its summary (a segfault, say): the
            # tests caught it, just not gracefully, so it counts as killed.
            # If it was not built, the mutant broke the build: not run.
            nxt = PROGRAMS[len(sums)] if len(sums) < len(PROGRAMS) else None
            built = nxt and any(os.path.exists(os.path.join(tmp, 'out', nxt + x)) for x in ('', '.exe'))
            if built:
                killed.append(name)
                print(f'KILLED       {name}\n             {nxt} crashed: {(p.stderr.strip().splitlines() or ["?"])[-1][:150]}')
            else:
                bad_setup.append((name, 'did not build: ' + (p.stderr.strip().splitlines() or ['?'])[-1]))
                print(f'NO RESULT    {name}')
            shutil.rmtree(tmp, ignore_errors=True); continue
        if p.returncode == 0 and len(sums) != len(PROGRAMS):
            bad_setup.append((name, f'{len(sums)} of {len(PROGRAMS)} test programs reported'))
            print(f'NO RESULT    {name}'); continue
        if p.returncode != 0:
            killed.append(name)
            first = next((l for l in out if l.startswith('FAIL')), '')
            print(f'KILLED       {name}\n             {summary}; first: {first[:150]}')
        else:
            survived.append(name)
            print(f'SURVIVED     {name}  ({summary})')
        shutil.rmtree(tmp, ignore_errors=True)
    print(f'\n{len(killed)} killed, {len(survived)} survived, {len(bad_setup)} not run, of {len(chosen)}'
          + (f' (filtered from {len(MUTANTS)})' if words else ''))
    for n, why in bad_setup:
        print(f'  not run: {n}: {why}')
    return 0 if not survived and not bad_setup else 1


if __name__ == '__main__':
    sys.exit(main())
