// A simulated ATA drive at register level, driven by the real ide.c through
// the PIO stand-ins below. It models only what the firmware relies on:
// BSY/DRDY/DRQ/ERR timing, PIO data in and out, soft reset, INITIALIZE
// DEVICE PARAMETERS, and a few ways a sector can fail. For the SAT path it
// also answers IDENTIFY DEVICE (0xEC), READ VERIFY (0x40), SMART (0xB0: D0,
// D1, D5, DA), READ NATIVE MAX ADDRESS (0xF8) and its EXT form (0x27), and
// reads back the HOB bytes when Device Control has HOB set.
//
// Timing and signals the SAT start-of-command rule depends on: a drive can
// take longer than ATA's 400 ns to raise BSY (t_bsy_delay), and it drives
// INTRQ (active high, only while selected and nIEN is 0) when a command ends
// or a data block is ready; reading Status (not Alternate Status), writing
// the command register, or SRST releases it. INTRQ can also be not wired or
// stuck high. A drive can ignore a command outright (ignore_cmd: no BSY, no
// INTRQ, status unchanged) or end one with DF (device fault) set.
//
// Long internal pauses (0.6f3p7): a sector can keep the drive busy for extra
// time while it is read or while a write to it is committed (pause), as the
// ST380011A did on a write. The RESET- line is modelled (hw_reset_line(),
// driven by gpio_put on GPIO 23): low puts the drive in reset, high starts
// its power-on diagnostics, t_hw_reset long. A drive can stay busy for ever
// after a soft reset (srst_wedges) and come back only from a hardware reset,
// or not even then (hw_reset_wedges).
//
// Review 0.6f3p7 (M-1 and the LOW items): a drive in its power-on diagnostics
// after RESET- holds IORDY low, and a PIO cycle with IORDY believed would
// then never end on the real board (iordy_stalls counts them). A drive can
// come out of RESET- with interrupts disabled (hw_reset_nien), take a while
// over RECALIBRATE (t_recal), or answer IDENTIFY with ERR and a block of
// data (identify_err_drq). The shortest RESET- pulse and the time of the last
// SRST are kept for the tests.
//
// Review of 0.6f3p7 (the MEDIUM on the least time a command is sent with):
// INITIALIZE DEVICE PARAMETERS can take a while (t_idp), and a drive can stay
// busy for a while after the last block of a read (busy_after), so that the
// next command finds it not ready yet.
// 0.6f3p8 (manual CHS, no IDENTIFY): every command byte is logged in order
// (cmd_log). A drive older than ATA can leave DRDY clear until it has had
// INITIALIZE DEVICE PARAMETERS (drdy_needs_idp: idle status 10h, not 50h),
// and can abort RECALIBRATE (abort_recal).
// Review of 0.6f3p8: a drive older than LBA takes no notice of the LBA bit
// in the device register and reads the address registers as CHS
// (lba_ignored), so an LBA address names some other sector, with good status.
// 0.6f3p9: IDENTIFY answers word 49 (id_w49; bit 11 is IORDY supported).
#pragma once
#include <stdint.h>
#include <map>
#include <vector>
#include <cstdio>

enum BadMode {
    BAD_ERR,          // UNC, ERR set, no data offered
    BAD_ERR_DRQ,      // UNC, ERR set and flawed data offered (DRQ=1), as older drives do
    BAD_ERR_DRQ_MORE, // like BAD_ERR_DRQ, but DRQ stays set after the drain (drive confused)
    BAD_HANG,         // BSY never clears for this sector (only SRST ends it)
    BAD_MARGINAL,     // fails (like BAD_ERR) for the first 'fails' attempts, then reads
};
struct Bad { BadMode mode; int fails; };

struct SimDrive {
    // geometry of the medium
    uint32_t nsect = 980u * 10 * 17;
    uint8_t  native_heads = 10, native_spt = 17;
    // current translation (set by 0x91; lost on SRST)
    bool     geo_valid = false;
    uint8_t  heads = 0, spt = 0;

    std::map<uint32_t, Bad> bad;
    std::map<uint32_t, std::vector<uint8_t>> written;
    std::map<uint32_t, int> attempts;       // media access attempts per sector

    // timing (ns)
    uint64_t t_sector = 200000;     // good sector
    uint64_t t_bad    = 600000000;  // failing sector (measured ~600 ms on the WD Caviar 280)
    uint64_t t_reset  = 2000000;
    bool     garbage_while_busy = false;  // status reads 0x81 while BSY (bits undefined)
    // With no INITIALIZE DEVICE PARAMETERS since power-on or SRST, a real drive
    // translates CHS with its own default geometry; it does not refuse. So a
    // lost geometry reads the WRONG sector with good status. false = IDNF.
    bool     default_translation = true;
    // This drive answers as device 1 (slave). Only the selected device (DEV
    // bit of register 6) drives status or acts on a command; SRST selects 0.
    bool     slave = false;
    // With slave set: a master is on the cable too. It is modelled only as
    // far as a soft reset goes. After SRST is released it stays busy for
    // master_t_reset (ATA: device 0 also waits for device 1), and while it
    // is selected and busy a register write is lost and counted as a
    // violation (the host must not write the command block while the
    // selected device is busy). Otherwise it reads DRDY and takes nothing.
    bool     master_present = false;
    uint64_t master_t_reset = 3000000000ull;
    uint64_t master_ready_at = 0;
    int      reset_writes = 0;              // task-file writes while in SRST
    bool     reject_idp = false;            // ABRT INITIALIZE DEVICE PARAMETERS
    // Extra busy time (ns) while reading this sector, or committing a write
    // to it. On a write the data is already in the drive (written[]) when
    // the pause starts, as on the ST380011A.
    std::map<uint32_t, uint64_t> pause;
    // Hardware reset (RESET-).
    bool     reset_low = false;             // the line is held low now
    uint64_t t_hw_reset = 500000000ull;     // power-on diagnostics after RESET- goes high
    bool     srst_wedges = false;           // after SRST, BSY until a hardware reset
    bool     hw_reset_wedges = false;       // ...and after a hardware reset too
    bool     wedged = false;
    int      hw_resets = 0;
    // ATA: RESET- must be held low at least 25 us. A shorter pulse is not
    // taken as a reset here: the drive goes on as it was.
    uint64_t reset_low_at = 0;
    bool     wedged_before_reset = false;
    int      short_resets = 0;
    uint64_t min_reset_low = ~0ull;         // shortest RESET- pulse seen (ns)
    bool     hw_post = false;               // in reset from RESET-: IORDY held low
    int      iordy_stalls = 0;              // PIO cycles with IORDY believed meanwhile
    bool     hw_reset_nien = false;         // comes out of RESET- with nIEN set
    uint64_t t_recal = 1000;                // RECALIBRATE (0x10) busy time
    bool     identify_err_drq = false;      // IDENTIFY ends at once with ERR and a block
    uint64_t srst_at = 0;                   // when SRST was last set
    // IDENTIFY takes this much longer than a sector (drive busy).
    uint64_t t_identify_extra = 0;
    uint64_t t_idp = 1000;                  // INITIALIZE DEVICE PARAMETERS (0x91) busy time
    // BSY for this long after the last block of a PIO data-in command, when
    // that block is this sector (IDENTIFY's counts as sector 0). The host
    // does not look at status after the last block of a read, so the next
    // command finds the drive busy.
    std::map<uint32_t, uint64_t> busy_after;
    bool     drdy_needs_idp = false;        // DRDY only once 0x91 has set a geometry
    bool     abort_recal = false;           // RECALIBRATE (0x10) ends at once with ABRT
    bool     lba_ignored = false;           // the LBA bit means nothing: every address is CHS
    uint16_t id_w49 = 0x0200;               // IDENTIFY word 49: LBA; bit 11 (IORDY) clear
    std::vector<uint8_t> cmd_log;           // every command byte, in order
    int      reg_writes = 0;                // any task file or Device Control write
    uint8_t idle_status() const { return (drdy_needs_idp && !geo_valid) ? 0x10 : 0x50; }

    // SAT-path features. nsect is the size the drive reports now; a larger
    // native_max models a Host Protected Area (0 = no HPA, same as nsect).
    uint64_t native_max = 0;
    bool     hpa_feature = true;            // answers READ NATIVE MAX (0xF8)
    bool     lba48 = true;                  // answers 48-bit commands (0x27)
    bool     smart_supported = true;        // else every SMART command ABRTs
    bool     smart_exceeded = false;        // SMART RETURN STATUS: threshold exceeded
    uint8_t  hang_cmd = 0;                  // this command byte never finishes (0 = none)
    bool     nondata_drq = false;           // a non-data command wrongly ends with DRQ
    uint64_t t_nondata = 1000000;           // non-data command that touches no sector
    // BSY rises this long after the 400 ns ATA allows (0 = at 400 ns). Until
    // then status reads as it did before the command.
    uint64_t t_bsy_delay = 0;
    uint8_t  ignore_cmd = 0;                // this command byte is ignored: no BSY, no INTRQ (0 = none)
    // Device fault: this command ends with DF (status 70h) and ERR clear,
    // error register 0; for a PIO data-in command after its last block, or,
    // with df_before_data, instead of the first block. 0 = none.
    uint8_t  df_cmd = 0;
    bool     df_before_data = false;
    // INTRQ. intrq is the drive's interrupt pending; the line is high when
    // it is, the drive is selected and nIEN (Device Control bit 1) is 0.
    bool     intrq = false;
    bool     intrq_wired = true;            // false: the line always reads low
    bool     intrq_stuck = false;           // true: the line always reads high
    // IDENTIFY DEVICE (0xEC). Words 82..84 are what the firmware keeps; the
    // defaults say what this drive answers: SMART (82.0), HPA (82.10),
    // 48-bit (83.10), SMART error logging (84.0), and the 01b signature in
    // bits 15:14 of words 83 and 84. identify_ok = false: IDENTIFY aborts.
    uint16_t id_w82 = 0x4401, id_w83 = 0x4400, id_w84 = 0x4001;
    // Word 85: enabled; SMART (bit 0) and HPA (bit 10) on, NOP (14). Word 87:
    // the 01b signature that makes 85..87 valid. A drive whose word 85 says
    // SMART is off aborts every SMART command, as the ST380011A did.
    uint16_t id_w85 = 0x4401, id_w87 = 0x4000;
    bool     identify_ok = true;
    const char *id_model = "SIMULATED ATA DRIVE";  // words 27..46
    const char *id_serial = "SIM0000001";           // words 10..19

    // registers
    uint8_t reg[8] = {0};
    uint8_t hob[8] = {0};           // previous value of regs 2..5 (LBA48)
    uint8_t status = 0x50, error = 0x01;
    uint8_t status_at_cmd = 0x50;
    uint64_t cmd_at = 0;

    enum Phase { IDLE, BUSY_IN, DRQ_IN, ERR_DRQ, HUNG, BUSY_OUT, DRQ_OUT, BUSY_COMMIT, IN_RESET, BUSY_ND };
    Phase phase = IDLE;
    uint64_t ready_at = 0;
    uint32_t cur = 0, left = 0;
    uint8_t  cmd = 0;
    uint16_t xfer[256];
    int      widx = 0;
    bool     more_after_drain = false;
    uint8_t  devctl = 0;
    // ATA: status is not valid for 2 ms after SRST is released; this drive
    // keeps showing what it showed before the reset during that window.
    uint8_t  status_before_srst = 0x50;
    uint64_t srst_released_at = 0;
    uint8_t  nd_status = 0x50;              // status a non-data command ends with
    // A non-data command's output registers appear when it ends, not when it
    // starts: until then the task file reads as the host wrote it.
    uint8_t  cmd_reg[8] = {0}, cmd_hob[8] = {0};    // as written, at the command
    uint8_t  nd_reg[8] = {0}, nd_hob[8] = {0}, nd_error = 0;

    // bookkeeping for the tests
    int srst = 0, init_params = 0, violations = 0, commands = 0;
    int data_reads = 0;                     // data register reads, any phase
    int id_data_reads = 0;                  // ...of which during IDENTIFY (0xEC)
    // Commands and data reads other than IDENTIFY: the firmware re-IDENTIFYs
    // the drive before a SAT command gated on IDENTIFY words (sat.c), and a
    // test of the command itself counts only the command.
    int non_id_commands() const { auto it = cmd_count.find(0xEC); return commands - (it == cmd_count.end() ? 0 : it->second); }
    int non_id_data_reads() const { return data_reads - id_data_reads; }
    int id_commands() const { auto it = cmd_count.find(0xEC); return it == cmd_count.end() ? 0 : it->second; }
    int hob_selects = 0;                    // Device Control writes with HOB set
    std::map<uint8_t, int> cmd_count;
    // Set by a test: called at every command; each command sent while it
    // returns false is counted.
    bool   (*busy_probe)() = nullptr;
    int      cmds_while_not_busy = 0;

    static uint8_t pattern(uint32_t lba, int i) {
        uint32_t x = lba * 2654435761u + (uint32_t)i * 40503u + 0x9E37u;
        x ^= x >> 13; x *= 0x5bd1e995u; x ^= x >> 15;
        return (uint8_t)x;
    }
    uint8_t byte_at(uint32_t lba, int i) const {
        auto w = written.find(lba);
        return w != written.end() ? w->second[i] : pattern(lba, i);
    }

    // ---- register decode --------------------------------------------------
    uint32_t addr_lba() const {
        if (cmd == 0x24) {
            return (uint32_t)reg[3] | ((uint32_t)reg[4] << 8) | ((uint32_t)reg[5] << 16) |
                   ((uint32_t)hob[3] << 24);
        }
        if ((reg[6] & 0x40) && !lba_ignored)
            return (uint32_t)reg[3] | ((uint32_t)reg[4] << 8) | ((uint32_t)reg[5] << 16) |
                   ((uint32_t)(reg[6] & 0x0F) << 24);
        uint32_t cyl = reg[4] | (reg[5] << 8), head = reg[6] & 0x0F, sec = reg[3];
        uint32_t H = heads, S = spt;
        if (!geo_valid) {
            if (!default_translation) return 0xFFFFFFFFu;
            H = native_heads; S = native_spt;
        }
        if (sec == 0 || sec > S || head >= H) return 0xFFFFFFFFu;
        return (cyl * H + head) * S + (sec - 1);
    }
    void set_addr_regs(uint32_t lba) {
        if (reg[6] & 0x40 || cmd == 0x24) {
            reg[3] = lba & 0xFF; reg[4] = (lba >> 8) & 0xFF; reg[5] = (lba >> 16) & 0xFF;
            if (cmd != 0x24) reg[6] = (reg[6] & 0xF0) | ((lba >> 24) & 0x0F);
            else { hob[3] = (lba >> 24) & 0xFF; hob[4] = 0; hob[5] = 0; }
        } else if (geo_valid) {
            uint32_t cyl = lba / (heads * spt), r = lba % (heads * spt);
            reg[4] = cyl & 0xFF; reg[5] = (cyl >> 8) & 0xFF;
            reg[6] = (reg[6] & 0xF0) | (r / spt); reg[3] = (r % spt) + 1;
        }
    }

    // advance the state machine to 'now'
    void tick(uint64_t now) {
        if (phase == IN_RESET) {
            if (!(devctl & 0x04) && !reset_low && !wedged && now >= ready_at) {
                geo_valid = false;              // SRST drops INITIALIZE DEVICE PARAMETERS
                phase = IDLE; status = idle_status(); error = 0x01; hw_post = false;
            }
            return;
        }
        if (phase == BUSY_IN && now >= ready_at) { resolve_sector(); if (phase != HUNG) intrq = true; }
        if (phase == BUSY_ND && now >= ready_at) {
            for (int i = 0; i < 8; i++) { reg[i] = nd_reg[i]; hob[i] = nd_hob[i]; }
            error = nd_error;
            if (nondata_drq && !(nd_status & 0x01)) {
                for (int i = 0; i < 256; i++) xfer[i] = 0xDEAD;
                widx = 0; left = 1; phase = DRQ_IN; status = 0x58;
            } else { phase = IDLE; status = nd_status; }
            intrq = true;
        }
        if (phase == BUSY_OUT && now >= ready_at) { phase = DRQ_OUT; widx = 0; status = 0x58; }   // no INTRQ for the first block out
        if (phase == BUSY_COMMIT && now >= ready_at) {
            if (left == 0) { phase = IDLE; status = 0x50; }
            else { phase = DRQ_OUT; widx = 0; status = 0x58; }
            intrq = true;
        }
        if (phase == IDLE && (status & 0x80) && now >= ready_at) { status = idle_status(); intrq = true; }
    }

    void fail_here(uint8_t err, bool offer) {
        set_addr_regs(cur);
        reg[2] = (uint8_t)left;
        error = err;
        if (offer) {
            for (int i = 0; i < 256; i++) xfer[i] = 0xDEAD;   // flawed data, never valid
            widx = 0; phase = ERR_DRQ; status = 0x59;
        } else { phase = IDLE; status = 0x51; }
    }

    // Synthetic sector numbers for SMART data, so the tests can check the bytes.
    static uint32_t smart_sector(uint8_t feature, uint8_t log, uint32_t i) {
        return 0xF0000000u | ((uint32_t)feature << 16) | ((uint32_t)log << 8) | i;
    }

    void fill_identify() {
        for (int i = 0; i < 256; i++) xfer[i] = 0;
        xfer[0] = 0x0040;                                   // fixed disk
        xfer[1] = (uint16_t)(nsect / (native_heads * native_spt));
        xfer[3] = native_heads; xfer[6] = native_spt;
        const char *model = id_model;
        const char *ser = id_serial;
        for (int i = 0; i < 10; i++) {
            char a = ser[0] ? *ser++ : ' ';
            char b = ser[0] ? *ser++ : ' ';
            xfer[10 + i] = (uint16_t)(((uint8_t)a << 8) | (uint8_t)b);
        }
        for (int i = 0; i < 20; i++) {
            char a = model[0] ? *model++ : ' ';
            char b = model[0] ? *model++ : ' ';
            xfer[27 + i] = (uint16_t)(((uint8_t)a << 8) | (uint8_t)b);
        }
        xfer[49] = id_w49;                                  // LBA (bit 9), IORDY (bit 11)
        xfer[60] = (uint16_t)nsect; xfer[61] = (uint16_t)(nsect >> 16);
        xfer[82] = id_w82; xfer[83] = id_w83; xfer[84] = id_w84;
        xfer[85] = id_w85; xfer[87] = id_w87;
        xfer[100] = (uint16_t)nsect; xfer[101] = (uint16_t)(nsect >> 16);
    }

    void resolve_sector() {
        if (df_cmd && cmd == df_cmd && df_before_data) { error = 0; phase = IDLE; status = 0x70; return; }
        if (cmd == 0xEC && identify_err_drq) {           // ABRT, with a block on offer
            for (int i = 0; i < 256; i++) xfer[i] = 0xDEAD;
            error = 0x04; widx = 0; phase = ERR_DRQ; status = 0x59; more_after_drain = false;
            return;
        }
        if (cmd == 0xEC) { fill_identify(); widx = 0; phase = DRQ_IN; status = 0x58; return; }
        if (cmd == 0xB0) {                  // SMART data: always readable
            for (int i = 0; i < 256; i++) xfer[i] = byte_at(cur, 2 * i) | (byte_at(cur, 2 * i + 1) << 8);
            widx = 0; phase = DRQ_IN; status = 0x58;
            return;
        }
        if (cur == 0xFFFFFFFFu || cur >= nsect) { fail_here(0x10, false); return; } // IDNF
        attempts[cur]++;
        auto b = bad.find(cur);
        if (b != bad.end()) {
            Bad &bd = b->second;
            switch (bd.mode) {
            case BAD_ERR: fail_here(0x40, false); return;
            case BAD_ERR_DRQ: fail_here(0x40, true); more_after_drain = false; return;
            case BAD_ERR_DRQ_MORE: fail_here(0x40, true); more_after_drain = true; return;
            case BAD_HANG: phase = HUNG; status = 0x80; return;
            case BAD_MARGINAL:
                if (bd.fails > 0) { bd.fails--; fail_here(0x40, false); return; }
                break;
            }
        }
        for (int i = 0; i < 256; i++) xfer[i] = byte_at(cur, 2 * i) | (byte_at(cur, 2 * i + 1) << 8);
        widx = 0; phase = DRQ_IN; status = 0x58;
    }

    uint64_t sector_delay(uint32_t lba) const {
        auto b = bad.find(lba);
        auto p = pause.find(lba);
        uint64_t extra = p != pause.end() ? p->second : 0;
        if (b != bad.end() && (b->second.mode != BAD_MARGINAL || b->second.fails > 0)) return t_bad + extra;
        return t_sector + extra;
    }

    bool selected() const { return ((reg[6] >> 4) & 1) == (slave ? 1 : 0); }
    bool master_busy(uint64_t now) const { return (devctl & 0x04) || now < master_ready_at; }

    // alt: Alternate Status (control block), which does not release INTRQ.
    uint8_t read_status(uint64_t now, bool alt = false) {
        if (!selected()) {
            if (slave && master_present) return master_busy(now) ? 0x80 : 0x50;
            return 0xFF;                           // no such device: the ATAboy bus floats high (ide.c probe filter)
        }
        if (!alt) intrq = false;                   // reading Status releases INTRQ
        if (phase == IN_RESET && !(devctl & 0x04) && now < srst_released_at + 2000000)
            return status_before_srst & ~0x80;     // stale during the 2 ms window
        // ATA: status is not valid for 400 ns after a command is written;
        // this drive keeps showing the pre-command status in that window,
        // and for t_bsy_delay more if it is slow to raise BSY.
        if (commands > 0 && now < cmd_at + 400 + t_bsy_delay) return status_at_cmd;
        tick(now);
        if (!alt) intrq = false;                   // ...including one raised by that tick
        if ((status & 0x80) && garbage_while_busy) return 0x81;
        return status;
    }

    bool intrq_line(uint64_t now) {
        if (!intrq_wired) return false;
        if (intrq_stuck) return true;
        tick(now);
        return intrq && selected() && !(devctl & 0x02);
    }

    uint16_t read_reg(int r, uint64_t now) {
        if (r == 7) return read_status(now);
        if (r == 1) return error;
        if ((devctl & 0x80) && r >= 2 && r <= 5) return hob[r];   // HOB: previous content
        return reg[r];
    }

    void write_reg(int r, uint8_t v, uint64_t now) {
        reg_writes++;
        tick(now);
        if (slave && master_present && !selected() && master_busy(now)) { violations++; return; }   // lost
        if (phase == IN_RESET) {
            // Once SRST is released the drives are still busy finishing the
            // reset. Selecting a device then is latched (Linux does it on a
            // slave-only cable); any other write is a protocol violation.
            // For a master it is one too: device 0 is the drive the host
            // waits on, so the host has not waited for it.
            if (r == 6 && !(devctl & 0x04)) { if (!slave) { violations++; reset_writes++; } reg[6] = v; return; }
            violations++; reset_writes++; return;
        }
        if (r == 7) {
            if (!selected()) { violations++; return; }  // command sent to the other device
            command(v, now); return;
        }
        if (status & 0x88) violations++;       // task file written while BSY or DRQ
        devctl &= ~0x80;                       // ATA: a register write clears HOB
        if (r >= 2 && r <= 5) hob[r] = reg[r];
        reg[r] = v;
    }

    void command(uint8_t c, uint64_t now) {
        commands++; cmd_count[c]++; cmd_log.push_back(c);
        if (busy_probe && !busy_probe()) cmds_while_not_busy++;
        if (status & 0x88) violations++;       // command written while BSY or DRQ
        status_at_cmd = status; cmd_at = now;
        intrq = false;                         // writing the command register releases INTRQ
        if (ignore_cmd && c == ignore_cmd) return;   // no BSY, no INTRQ, registers and status as they were
        cmd = c; error = 0;
        for (int i = 0; i < 8; i++) { cmd_reg[i] = reg[i]; cmd_hob[i] = hob[i]; }
        if (hang_cmd && c == hang_cmd) { phase = HUNG; status = 0x80; return; }
        switch (c) {
        case 0x20: case 0x21: case 0x24:
            left = reg[2] ? reg[2] : 256;
            cur = addr_lba();
            phase = BUSY_IN; status = 0x80; ready_at = now + sector_delay(cur);
            break;
        case 0x30: case 0x34:
            left = reg[2] ? reg[2] : 256;
            cur = addr_lba();
            phase = BUSY_OUT; status = 0x80; ready_at = now + 1000;
            break;
        case 0x91:
            if (reject_idp) { phase = IDLE; status = 0x51; error = 0x04; intrq = true; break; }
            heads = (reg[6] & 0x0F) + 1; spt = reg[2]; geo_valid = true; init_params++;
            phase = IDLE; status = 0x80; ready_at = now + t_idp;
            break;
        case 0x10:
            if (abort_recal) { abort_cmd(); break; }
            phase = IDLE; status = 0x80; ready_at = now + t_recal;
            break;
        case 0xEC:
            if (!identify_ok) { abort_cmd(); break; }
            left = 1; cur = 0;
            phase = BUSY_IN; status = 0x80; ready_at = now + t_sector + t_identify_extra;
            break;
        case 0x40: verify(now); break;
        case 0xB0: smart(now); break;
        case 0xF8: case 0x27: native_max_cmd(c, now); break;
        default:
            phase = IDLE; status = 0x51; error = 0x04; intrq = true;   // ABRT
        }
    }

    void abort_cmd() { phase = IDLE; status = 0x51; error = 0x04; intrq = true; }   // ABRT, at once

    void end_nondata(uint64_t now, uint64_t t, uint8_t st) {
        if (df_cmd && cmd == df_cmd) { st = 0x70; error = 0; }     // DF, ERR clear
        for (int i = 0; i < 8; i++) { nd_reg[i] = reg[i]; nd_hob[i] = hob[i]; reg[i] = cmd_reg[i]; hob[i] = cmd_hob[i]; }
        nd_error = error; error = 0;
        phase = BUSY_ND; status = 0x80; nd_status = st; ready_at = now + t;
    }

    // READ VERIFY SECTORS: reads each sector and returns no data. On a failure
    // the registers name the failing sector, as for READ SECTORS.
    void verify(uint64_t now) {
        if (!(reg[6] & 0x40)) { abort_cmd(); return; }
        uint32_t n = reg[2] ? reg[2] : 256;
        uint32_t lba = addr_lba();
        uint64_t t = 0;
        for (uint32_t i = 0; i < n; i++) {
            uint32_t s = lba + i;
            uint8_t err = 0;
            if (s >= nsect) err = 0x10;                                 // IDNF
            else {
                attempts[s]++;
                t += sector_delay(s);
                auto b = bad.find(s);
                if (b != bad.end()) {
                    Bad &bd = b->second;
                    if (bd.mode == BAD_HANG) { phase = HUNG; status = 0x80; return; }
                    if (bd.mode != BAD_MARGINAL) err = 0x40;             // UNC
                    else if (bd.fails > 0) { bd.fails--; err = 0x40; }
                }
            }
            if (err) {
                set_addr_regs(s); reg[2] = (uint8_t)(n - i); error = err;
                end_nondata(now, t + t_sector, 0x51);
                return;
            }
        }
        end_nondata(now, t + t_sector, 0x50);
    }

    void smart(uint64_t now) {
        if (!smart_supported || !(id_w85 & 1) || reg[4] != 0x4F || reg[5] != 0xC2) { abort_cmd(); return; }
        switch (reg[1]) {
        case 0xD0: case 0xD1:
            left = 1; cur = smart_sector(reg[1], 0, 0);
            phase = BUSY_IN; status = 0x80; ready_at = now + t_sector;
            break;
        case 0xD5:
            left = reg[2] ? reg[2] : 256; cur = smart_sector(0xD5, reg[3], 0);
            phase = BUSY_IN; status = 0x80; ready_at = now + t_sector;
            break;
        case 0xDA:
            reg[4] = smart_exceeded ? 0xF4 : 0x4F;
            reg[5] = smart_exceeded ? 0x2C : 0xC2;
            end_nondata(now, t_nondata, 0x50);
            break;
        default:
            abort_cmd();
        }
    }

    // READ NATIVE MAX ADDRESS (EXT): the last native LBA, low 24 bits in the
    // LBA registers, the rest in the device nibble (0xF8) or the HOB bytes (0x27).
    void native_max_cmd(uint8_t c, uint64_t now) {
        if (!(reg[6] & 0x40) || (c == 0xF8 && !hpa_feature) || (c == 0x27 && !lba48)) { abort_cmd(); return; }
        uint64_t last = (native_max ? native_max : nsect) - 1;
        if (c == 0xF8 && last > 0x0FFFFFFFull) last = 0x0FFFFFFF;
        reg[3] = (uint8_t)last; reg[4] = (uint8_t)(last >> 8); reg[5] = (uint8_t)(last >> 16);
        if (c == 0xF8) reg[6] = (uint8_t)((reg[6] & 0xF0) | ((last >> 24) & 0x0F));
        else { hob[3] = (uint8_t)(last >> 24); hob[4] = (uint8_t)(last >> 32); hob[5] = (uint8_t)(last >> 40); }
        reg[2] = 0; hob[2] = 0;
        end_nondata(now, t_nondata, 0x50);
    }

    void write_control(uint8_t v, uint64_t now) {
        reg_writes++;
        tick(now);
        if (v & 0x80) hob_selects++;
        if (v & 0x04) {
            intrq = false;
            if (!(devctl & 0x04)) { srst++; status_before_srst = status; srst_at = now; }
            phase = IN_RESET; status = 0x80;
            if (srst_wedges) wedged = true;
            reg[6] &= ~0x10;                       // SRST selects device 0
        } else if (devctl & 0x04) {
            ready_at = now + t_reset;
            srst_released_at = now;
            master_ready_at = now + master_t_reset;
        }
        devctl = v;
    }

    // RESET- (hardware reset). Low: every device on the cable is in reset,
    // device 0 selected, registers and Device Control cleared, CHS
    // translation lost. High: power-on diagnostics for t_hw_reset (a master
    // modelled for SRST also waits that long).
    void hw_reset_line(bool high, uint64_t now) {
        tick(now);
        if (!high) {
            if (reset_low) return;
            reset_low = true; hw_resets++;
            reset_low_at = now; wedged_before_reset = wedged;
            intrq = false; devctl = hw_reset_nien ? 0x02 : 0;
            hw_post = true;
            phase = IN_RESET; status = 0x80; error = 0x01;
            for (int i = 0; i < 8; i++) { reg[i] = 0; hob[i] = 0; }
            geo_valid = false;
            wedged = hw_reset_wedges;
            return;
        }
        if (!reset_low) return;                // already high: nothing happens
        reset_low = false;
        if (now - reset_low_at < min_reset_low) min_reset_low = now - reset_low_at;
        if (now - reset_low_at < 25000) {      // too short to count
            short_resets++;
            wedged = wedged_before_reset;
        }
        ready_at = now + t_hw_reset;
        srst_released_at = now;
        status_before_srst = 0x80;             // BSY through the 2 ms window
        master_ready_at = now + t_hw_reset;
    }

    uint16_t read_data(uint64_t now) {
        tick(now);
        data_reads++;
        if (cmd == 0xEC) id_data_reads++;
        if (phase == DRQ_IN || phase == ERR_DRQ) {
            uint16_t w = xfer[widx++];
            if (widx == 256) {
                if (phase == ERR_DRQ) {
                    phase = IDLE; status = more_after_drain ? 0x59 : 0x51;
                    if (more_after_drain) { phase = ERR_DRQ; widx = 0; }
                } else {
                    cur++; left--;
                    if (left == 0) {
                        phase = IDLE; status = (df_cmd && cmd == df_cmd) ? 0x70 : 0x50;
                        if (busy_after.count(cur - 1)) { status = 0x80; ready_at = now + busy_after[cur - 1]; }
                    }
                    else { phase = BUSY_IN; status = 0x80; ready_at = now + sector_delay(cur); }
                }
            }
            return w;
        }
        violations++;                              // data read without DRQ
        return 0xFFFF;
    }

    void write_data(uint16_t w, uint64_t now) {
        tick(now);
        if (phase != DRQ_OUT) { violations++; return; }
        xfer[widx++] = w;
        if (widx == 256) {
            std::vector<uint8_t> s(512);
            for (int i = 0; i < 256; i++) { s[2 * i] = xfer[i] & 0xFF; s[2 * i + 1] = xfer[i] >> 8; }
            written[cur] = s;
            auto p = pause.find(cur);
            uint64_t extra = p != pause.end() ? p->second : 0;
            cur++; left--;
            phase = BUSY_COMMIT; status = 0x80; ready_at = now + 1000 + extra;
        }
    }
};

extern SimDrive sim;

// ---- PIO stand-ins: decode CS0/CS1 and A0-A2 from the SIO output state ----
inline int bus_reg(bool &cs1) {
    bool cs0_low = !(mock_gpio_out & (1u << 24));
    bool cs1_low = !(mock_gpio_out & (1u << 25));
    cs1 = cs1_low && !cs0_low;
    if (cs0_low == cs1_low) { sim.violations++; return -1; }
    return (mock_gpio_out >> 20) & 7;
}
void ide_pio_init(void) {}
// A cycle with IORDY believed while the drive holds it low in its power-on
// diagnostics would never end on the board (the PIO program's wait has no
// timeout). Counted, and the cycle goes on, so a test can report it.
inline void iordy_check() {
    if (mock_iordy_inover == GPIO_OVERRIDE_NORMAL && sim.hw_post && sim.phase == SimDrive::IN_RESET) sim.iordy_stalls++;
}
void ide_pio_read(uint32_t count, uint16_t *buf) {
    iordy_check();
    bool cs1; int r = bus_reg(cs1);
    for (uint32_t i = 0; i < count; i++) {
        if (r < 0) { buf[i] = 0xFFFF; continue; }
        if (cs1) buf[i] = (r == 6) ? sim.read_status(mock_now_ns, true) : 0xFF;  // alt status
        else if (r == 0) buf[i] = sim.read_data(mock_now_ns);
        else buf[i] = sim.read_reg(r, mock_now_ns);
    }
}
void ide_pio_write(uint32_t count, const uint16_t *buf) {
    iordy_check();
    bool cs1; int r = bus_reg(cs1);
    for (uint32_t i = 0; i < count; i++) {
        if (r < 0) continue;
        if (cs1) { if (r == 6) sim.write_control((uint8_t)buf[i], mock_now_ns); }
        else if (r == 0) sim.write_data(buf[i], mock_now_ns);
        else sim.write_reg(r, (uint8_t)buf[i], mock_now_ns);
    }
}
bool mock_intrq(void) { return sim.intrq_line(mock_now_ns); }
void mock_reset_line(bool high) { sim.hw_reset_line(high, mock_now_ns); }
