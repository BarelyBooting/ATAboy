#include "ide.h"
#include "ide_pio.h"
#include "config.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

static uint8_t dev_base = 0xA0;   // 0xA0 = master, 0xB0 = slave

// True when the drive's CHS translation is not known to be ours: after a
// soft reset (which drops it) until INITIALIZE DEVICE PARAMETERS succeeds
// again. A CHS address would then go through the drive's own default
// translation and name a different sector, so CHS reads and writes refuse.
static bool chs_geometry_lost = false;

void ide_select_device(uint8_t base) { dev_base = base; }

#if ATABOY_SAT
// IDENTIFY words 82..84 kept for the SAT policy (ide.h, ide_id_words). Written
// only by ide_identify() and ide_probe_devices(), which run on core 1 while
// nothing is mounted; read by sat.c on core 0, which only issues anything
// while a drive is mounted. The policy checks "mounted" before it looks at
// these, so a read that races a detection is refused anyway.
static struct {
    bool     valid;
    uint8_t  dev_base;          // the device they came from
    uint16_t w82, w83, w84;
} id_words;

void ide_id_words_forget(void) { id_words.valid = false; }

void ide_id_words(ide_id_words_t *out) {
    out->valid = id_words.valid && id_words.dev_base == dev_base;
    out->w82 = id_words.w82;
    out->w83 = id_words.w83;
    out->w84 = id_words.w84;
}
#endif

// ---------------------------------------------------------------------------
//  Bus helpers — address, transceiver, and chip-select (all SIO-managed)
// ---------------------------------------------------------------------------

static void bus_idle(void) {
    sio_hw->gpio_set = (1 << IDE_CS0) | (1 << IDE_CS1);
    sio_hw->gpio_clr = (1 << IDE_DIR) | (1 << IDE_DIR1);   // DIR = read
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);     // OE disabled
}

static void set_address(uint8_t addr) {
    sio_hw->gpio_clr = ADDR_MASK;
    sio_hw->gpio_set = (uint32_t)(addr & 0x07) << IDE_A0;
}

static void xcvr_read(void) {
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);     // disable first
    sio_hw->gpio_clr = (1 << IDE_DIR) | (1 << IDE_DIR1);   // DIR = IDE -> Pico
    busy_wait_at_least_cycles(20);                            // 74HCT245 switch time
    sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);     // enable
}

static void xcvr_write(void) {
    sio_hw->gpio_set = (1 << IDE_OE) | (1 << IDE_OE1);     // disable first
    sio_hw->gpio_set = (1 << IDE_DIR) | (1 << IDE_DIR1);   // DIR = Pico -> IDE
    busy_wait_at_least_cycles(20);
    sio_hw->gpio_clr = (1 << IDE_OE) | (1 << IDE_OE1);     // enable
}

// ---------------------------------------------------------------------------
//  Register I/O — single 8-bit reads/writes via PIO strobes
// ---------------------------------------------------------------------------

void ide_write_reg(uint8_t reg, uint8_t val) {
    set_address(reg);
    xcvr_write();
    sio_hw->gpio_clr = (1 << IDE_CS0);

    uint16_t word = (uint16_t)val;
    ide_pio_write(1, &word);

    sio_hw->gpio_set = (1 << IDE_CS0);
    bus_idle();
}

uint8_t ide_read_reg(uint8_t reg) {
    set_address(reg);
    xcvr_read();
    sio_hw->gpio_clr = (1 << IDE_CS0);

    uint16_t val;
    ide_pio_read(1, &val);

    sio_hw->gpio_set = (1 << IDE_CS0);
    bus_idle();
    return (uint8_t)(val & 0xFF);
}

uint8_t ide_read_alt_status(void) {
    set_address(6);
    xcvr_read();
    sio_hw->gpio_clr = (1 << IDE_CS1);

    uint16_t val;
    ide_pio_read(1, &val);

    sio_hw->gpio_set = (1 << IDE_CS1);
    bus_idle();
    return (uint8_t)(val & 0xFF);
}

void ide_write_control(uint8_t val) {
    set_address(6);
    xcvr_write();
    sio_hw->gpio_clr = (1 << IDE_CS1);

    uint16_t word = (uint16_t)val;
    ide_pio_write(1, &word);

    sio_hw->gpio_set = (1 << IDE_CS1);
    bus_idle();
}

void ide_set_iordy(bool enabled) {
    gpio_set_inover(IDE_IORDY, enabled ? GPIO_OVERRIDE_NORMAL : GPIO_OVERRIDE_HIGH);
}

// ATA: the host must wait at least 400 ns after writing the command register
// before reading status. Until then the drive may not have raised BSY yet, so
// status can still show the end of the previous command, including its ERR.
//
// The READ(10) and WRITE(10) paths, INITIALIZE DEVICE PARAMETERS (0x91),
// RECALIBRATE and IDENTIFY all assume the drive has raised BSY by the time
// this 1 us is up: their first status read takes BSY clear as "finished"
// (the reads and writes then still need DRQ, which a stale status does not
// have, but a stale ERR would end them early). A drive slower than that to
// raise BSY breaks the assumption. Those paths are hardware-validated and
// left as they are; the SAT path does not rely on it (sat_watch_t below).
static void wait_after_command(void) {
    busy_wait_us_32(1);
}

// ---------------------------------------------------------------------------
//  Init / Reset / Polling
// ---------------------------------------------------------------------------

void ide_hw_init(void) {
    // SIO-controlled output pins
    uint32_t sio_out = (1 << IDE_DIR) | (1 << IDE_DIR1) |
                       (1 << IDE_OE)  | (1 << IDE_OE1)  |
                       ADDR_MASK |
                       (1 << IDE_RESET) |
                       (1 << IDE_CS0) | (1 << IDE_CS1);

    gpio_init_mask(sio_out);
    gpio_set_dir_out_masked(sio_out);

    // INTRQ — active-high from drive, idle low
    gpio_init(IDE_INTRQ);
    gpio_set_dir(IDE_INTRQ, GPIO_IN);
    gpio_pull_down(IDE_INTRQ);

    // IORDY — active-high when ready, pulled up as fallback
    gpio_init(IDE_IORDY);
    gpio_set_dir(IDE_IORDY, GPIO_IN);
    gpio_pull_up(IDE_IORDY);
    // Force IORDY HIGH so PIO wait instructions never block during init
    gpio_set_inover(IDE_IORDY, GPIO_OVERRIDE_HIGH);

    // Data bus pads: no pulls (PIO owns these pins after ide_pio_init)
    for (int i = 0; i < 16; i++) gpio_disable_pulls(i);

    // Bring up PIO state machines
    ide_pio_init();

    gpio_put(IDE_RESET, 1);
    bus_idle();

    // nIEN=0: allow INTRQ from the drive
    ide_write_control(0x00);

    // Restore IORDY based on config (override was HIGH for safe init)
    ide_set_iordy(config.iordy_enabled);
}

void ide_reset_drive(void) {
    // Force IORDY HIGH during reset — drive holds it LOW during POST
    ide_set_iordy(false);

    gpio_put(IDE_RESET, 0);
    sleep_ms(50);
    gpio_put(IDE_RESET, 1);
    sleep_ms(100);
    ide_write_control(0x00);
    ide_wait_until_ready(10000);

    // Recalibrate — seek heads to track 0 (required by some pre-ATA drives)
    ide_write_reg(6, dev_base);
    ide_write_reg(7, 0x10);
    wait_after_command();
    ide_wait_until_ready(10000);

    // Restore IORDY to config setting — drive is ready for normal operation
    ide_set_iordy(config.iordy_enabled);
}

uint8_t ide_probe_devices(void) {
#if ATABOY_SAT
    id_words.valid = false;     // a new detection: nothing is known until IDENTIFY answers
#endif
    ide_set_iordy(false);

    // Single hardware reset — both devices see it
    gpio_put(IDE_RESET, 0);
    sleep_ms(50);
    gpio_put(IDE_RESET, 1);
    sleep_ms(2000);           // generous POST delay (not status-based)
    ide_write_control(0x00);  // nIEN=0

    static const uint8_t addrs[] = {0xA0, 0xB0};
    for (int i = 0; i < 2; i++) {
        ide_write_reg(6, addrs[i]);       // select device
        busy_wait_us_32(50);              // let selection settle

        // Floating bus filter: 0xFF means no device (bus floats high)
        uint8_t st = ide_read_reg(7);
        if (st == 0xFF) continue;

        // Device present — wait for BSY to clear (not DRDY; some older
        // drives won't assert DRDY until after INITIALIZE DRIVE PARAMETERS)
        dev_base = addrs[i];
        uint32_t start = to_ms_since_boot(get_absolute_time());
        bool bsy_clear = false;
        while (to_ms_since_boot(get_absolute_time()) - start < 10000) {
            st = ide_read_reg(7);
            if (!(st & 0x80)) { bsy_clear = true; break; }
            busy_wait_us_32(10);
        }
        if (!bsy_clear) continue;

        // Recalibrate (required by some pre-ATA drives)
        ide_write_reg(6, addrs[i]);
        ide_write_reg(7, 0x10);
        wait_after_command();
        ide_wait_until_ready(10000);

        ide_set_iordy(config.iordy_enabled);
        return addrs[i];
    }

    ide_set_iordy(config.iordy_enabled);
    return 0;  // no device found
}

bool ide_wait_until_ready(uint32_t timeout_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - start < timeout_ms) {
        uint8_t st = ide_read_reg(7);
        if (!(st & 0x80) && (st & 0x40)) return true;   // BSY=0, DRDY=1
        busy_wait_us_32(10);
    }
    return false;
}

bool ide_set_geometry(uint8_t heads, uint8_t spt) {
    ide_write_reg(6, dev_base | ((heads - 1) & 0x0F));
    ide_write_reg(2, spt);
    ide_write_reg(7, 0x91);
    wait_after_command();
    // Only a clean completion counts. With ABRT the drive keeps its own
    // default translation, which is not the geometry we address it with.
    bool ok = ide_wait_until_ready(1000) && !(ide_read_reg(7) & 0x01);
    chs_geometry_lost = !ok;
    return ok;
}

// ---------------------------------------------------------------------------
//  IDENTIFY DEVICE (0xEC)
// ---------------------------------------------------------------------------

static bool identify_once(uint16_t *buf) {
    if (!ide_wait_until_ready(1000)) return false;
    if (ide_read_reg(7) & 0x08) ide_drain_sector();   // drain stranded DRQ before command
    ide_write_reg(6, dev_base);
    ide_write_reg(7, 0xEC);
    busy_wait_us_32(1);             // give drive time to assert BSY

    // Poll for DRQ — INTRQ provides early-exit if enabled, otherwise pure polling
    for (uint32_t t = 0; t < 100000; t++) {
        if (config.intrq_enabled && gpio_get(IDE_INTRQ)) ide_read_reg(7);  // clear INTRQ
        uint8_t st = ide_read_reg(7);
        if (st & 0x80) { busy_wait_us_32(50); continue; }  // BSY: other bits not valid yet
        if (st & 0x01) { if (st & 0x08) ide_drain_sector(); return false; }  // ERR — drain stranded DRQ
        if (st & 0x08) goto ready;                       // BSY=0, DRQ=1
        busy_wait_us_32(50);
    }
    return false;

ready:
    // Burst-read 256 words through PIO
    set_address(0);
    xcvr_read();
    sio_hw->gpio_clr = (1 << IDE_CS0);

    ide_pio_read(256, buf);

    sio_hw->gpio_set = (1 << IDE_CS0);
    bus_idle();
    return true;
}

bool ide_identify(uint16_t *buf) {
    bool ok = identify_once(buf);
#if ATABOY_SAT
    // Keep what this drive says it supports, for the SAT policy. A failed
    // IDENTIFY forgets it: whatever answered before may not be this drive.
    id_words.valid = ok;
    if (ok) {
        id_words.dev_base = dev_base;
        id_words.w82 = buf[82];
        id_words.w83 = buf[83];
        id_words.w84 = buf[84];
    }
#endif
    return ok;
}

// ---------------------------------------------------------------------------
//  Drain one sector of DRQ data (discard 256 words)
// ---------------------------------------------------------------------------

void ide_drain_sector(void) {
    uint16_t discard[256];
    set_address(0);
    xcvr_read();
    sio_hw->gpio_clr = (1 << IDE_CS0);
    ide_pio_read(256, discard);
    sio_hw->gpio_set = (1 << IDE_CS0);
    bus_idle();
}

// ---------------------------------------------------------------------------
//  Failure record and recovery for sector I/O
// ---------------------------------------------------------------------------

static ide_fail_t last_fail;

void ide_last_failure(ide_fail_t *out) { *out = last_fail; }

// Keep what the drive reported for a failed command. Must run before any
// drain or reset, since both change the registers.
static void record_failure(uint8_t kind, uint8_t cmd, uint8_t st,
                           uint32_t lba, uint32_t done, uint32_t count) {
    last_fail.kind    = kind;
    last_fail.command = cmd;
    last_fail.status  = st;
    last_fail.error   = ide_read_reg(1);
    for (int i = 0; i < 5; i++) last_fail.tf[i] = ide_read_reg(2 + i);
    last_fail.drained = false;
    last_fail.reset   = false;
    last_fail.reset_failed = false;
    last_fail.lba     = lba;
    last_fail.done    = done;
    last_fail.count   = count;
}

#define IDE_SRST_TIMEOUT_MS 31000   // ATA allows a device up to 31 s after SRST

// Soft reset, then select our device again and put the CHS translation back.
// True only if the drive came back ready and, in CHS mode, took the geometry.
// On false, chs_geometry_lost stays set and CHS transfers refuse until a
// later ide_set_geometry() succeeds.
static bool srst_and_restore(void) {
    ide_write_control(0x04);
    busy_wait_us_32(10);
    ide_write_control(0x00);
    chs_geometry_lost = true;               // SRST drops INITIALIZE DEVICE PARAMETERS
    busy_wait_us_32(2000);                  // ATA: 2 ms before status is valid
    // SRST leaves device 0 selected, and device 0 holds BSY until device 1
    // has finished too, so the host waits on device 0 before it selects
    // anything. That goes for a slave as well: a slave can be in use with a
    // master on the cable (auto-mount takes the saved dev_base without
    // probing, and the probe passes over a master that stays busy for more
    // than 10 s), and selecting it while the master is still busy is a
    // register write the master may drop, leaving the slave unselected.
    // The exception: an absent device 0 reads 0xFF on this bus (it floats
    // high), which looks like BSY for the whole timeout, so a slave stops
    // waiting on 0xFF. (Review finding L2 corrected an earlier comment here
    // that said a slave is only used when there is no usable master.)
    // Either way our device is selected again before anything else, even on
    // a timeout: left pointing at device 0, every later command would go to
    // the wrong drive or poll a floating bus until the next detect.
    // (A master that times out needs no reselect: device 0 is ours, and
    // writing registers to a drive still in reset would break the protocol.
    // A slave whose master never comes out of reset is selected anyway;
    // there is nothing better to do for it.)
    uint32_t start = to_ms_since_boot(get_absolute_time());
    if (dev_base == 0xA0) {
        while (ide_read_reg(7) & 0x80) {
            if (to_ms_since_boot(get_absolute_time()) - start >= IDE_SRST_TIMEOUT_MS)
                return false;
            busy_wait_us_32(10);
        }
    } else {
        uint8_t st;
        while (((st = ide_read_reg(7)) & 0x80) && st != 0xFF) {
            if (to_ms_since_boot(get_absolute_time()) - start >= IDE_SRST_TIMEOUT_MS)
                break;                      // master stuck in reset: select the slave anyway
            busy_wait_us_32(10);
        }
    }
    ide_write_reg(6, dev_base);
    busy_wait_us_32(1);                     // 400 ns before status is valid
    if (!ide_wait_until_ready(IDE_SRST_TIMEOUT_MS)) return false;
    if (config.use_lba_mode) return true;
    return ide_set_geometry(config.heads, config.spt);
}

// Soft reset, for a drive that is stuck or did not end the command cleanly.
static void soft_reset_restore(void) {
    last_fail.reset = true;
    last_fail.reset_failed = !srst_and_restore();
}

// CHS mode after a reset that could not restore the geometry: try once more
// (the drive may just have been slow). False if the drive still will not
// take it; the caller must then refuse the transfer.
static bool chs_geometry_ok(void) {
    if (config.use_lba_mode || !chs_geometry_lost) return true;
    return ide_set_geometry(config.heads, config.spt);
}

// After a command ends with ERR the drive should be idle again: BSY=0,
// DRQ=0, DRDY=1. True if it gets there within timeout_ms.
static bool idle_after_error(uint32_t timeout_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - start < timeout_ms) {
        uint8_t st = ide_read_reg(7);
        if (!(st & 0x80) && !(st & 0x08) && (st & 0x40)) return true;
        busy_wait_us_32(10);
    }
    return false;
}

// ---------------------------------------------------------------------------
//  Sector I/O — LBA28, single-sector loop
// ---------------------------------------------------------------------------

int32_t ide_read_sectors(uint32_t lba, uint32_t count, uint8_t *buf) {
    return ide_read_sectors_partial(lba, count, buf, 0);
}

// Reads count sectors into buf. On failure, *done (if not null) is the number
// of sectors already in buf, which are good data from the drive; the failing
// sector and anything after it are never written to buf.
//
// Error handling (issue #13): an ERR from the drive means it has finished the
// command, so no reset is needed. If it offers data for the bad sector along
// with ERR (older drives put the flawed data in the buffer), that data is
// drained and thrown away. A soft reset is only used when the drive does not
// end the command (timeout) or is not idle afterwards.
int32_t ide_read_sectors_partial(uint32_t lba, uint32_t count, uint8_t *buf,
                                 uint32_t *done) {
    uint32_t s = 0;
    uint8_t st = 0;
    uint8_t cmd = 0;
    if (done) *done = 0;
    if (count == 0) return -1;
    if (!ide_wait_until_ready(5000)) {
        record_failure(IDE_FAIL_NOT_READY, 0, ide_read_reg(7), lba, 0, count);
        return -1;
    }
    // If the drive is still offering data from an earlier command (a SAT
    // command or a debug seek test that gave up, say), issuing a new one
    // breaks the ATA protocol, and some drives would then hand that stale
    // block back as this read's result. This is not a drive ERR, so it must
    // not take the read_err path
    // (which deliberately skips the reset): record it, clear it with a soft
    // reset, and fail this read.
    st = ide_read_reg(7);
    if (st & 0x08) {
        record_failure(IDE_FAIL_STALE_DRQ, 0, st, lba, 0, count);
        soft_reset_restore();
        return -1;
    }
    if (!chs_geometry_ok()) {
        record_failure(IDE_FAIL_NO_GEOMETRY, 0x91, ide_read_reg(7), lba, 0, count);
        return -1;
    }

    bool use_lba48 = config.use_lba_mode && (config.lba_sectors > 0x0FFFFFFF);

    if (config.use_lba_mode) {
        if (use_lba48) {
            // LBA48: HOB (high bytes) first, then LOB (low bytes)
            ide_write_reg(2, 0);                               // sector count high
            ide_write_reg(3, (lba >> 24) & 0xFF);              // LBA 24-31
            ide_write_reg(4, 0);                               // LBA 32-39 (0 for uint32_t)
            ide_write_reg(5, 0);                               // LBA 40-47 (0 for uint32_t)
            ide_write_reg(2, (uint8_t)count);                  // sector count low
            ide_write_reg(3, lba & 0xFF);                      // LBA 0-7
            ide_write_reg(4, (lba >> 8) & 0xFF);               // LBA 8-15
            ide_write_reg(5, (lba >> 16) & 0xFF);              // LBA 16-23
            ide_write_reg(6, dev_base | 0x40);                 // LBA mode, no address bits
        } else {
            ide_write_reg(2, (uint8_t)count);
            ide_write_reg(3, lba & 0xFF);
            ide_write_reg(4, (lba >> 8) & 0xFF);
            ide_write_reg(5, (lba >> 16) & 0xFF);
            ide_write_reg(6, (dev_base | 0x40) | ((lba >> 24) & 0x0F));
        }
    } else {
        uint32_t tmp  = lba / config.spt;
        uint8_t  sec  = (lba % config.spt) + 1;           // 1-based
        uint8_t  head = tmp % config.heads;
        uint16_t cyl  = tmp / config.heads;
        ide_write_reg(2, (uint8_t)count);
        ide_write_reg(3, sec);
        ide_write_reg(4, cyl & 0xFF);
        ide_write_reg(5, (cyl >> 8) & 0xFF);
        ide_write_reg(6, dev_base | (head & 0x0F));
    }

    cmd = use_lba48 ? 0x24 : 0x20;
    ide_write_reg(7, cmd);                                 // READ SECTORS EXT / READ SECTORS
    wait_after_command();

    uint16_t *wbuf = (uint16_t *)buf;

    for (s = 0; s < count; s++) {
        // Poll for DRQ — INTRQ provides early-exit if enabled, otherwise pure polling
        for (uint32_t t = 0; t < 100000; t++) {
            if (config.intrq_enabled && gpio_get(IDE_INTRQ)) ide_read_reg(7);  // clear INTRQ
            st = ide_read_reg(7);
            // While BSY is set the other status bits are not valid (ATA), so a
            // leftover ERR must not end a command the drive is still working on.
            if (st & 0x80) { busy_wait_us_32(10); continue; }
            if (st & 0x01) goto read_err;
            if (st & 0x08) goto drq_read;
            busy_wait_us_32(10);
        }
        goto read_timeout;

    drq_read:
        set_address(0);
        xcvr_read();
        sio_hw->gpio_clr = (1 << IDE_CS0);

        ide_pio_read(256, wbuf + s * 256);

        sio_hw->gpio_set = (1 << IDE_CS0);
        bus_idle();
    }

    if (done) *done = count;
    return (int32_t)(count * 512);

read_err:
    // The drive ended the command with ERR at sector s. Sectors 0..s-1 are
    // already in buf and are good.
    record_failure(IDE_FAIL_ERR, cmd, st, lba + s, s, count);
    if (st & 0x08) {
        // Data offered for the failed sector. Not good data: discard it.
        ide_drain_sector();
        last_fail.drained = true;
    }
    if (!idle_after_error(2000)) soft_reset_restore();
    if (done) *done = s;
    return -1;

read_timeout:
    // No DRQ in time: the drive may still be retrying, so abort with SRST.
    record_failure(IDE_FAIL_TIMEOUT, cmd, st, lba + s, s, count);
    soft_reset_restore();
    if (done) *done = s;
    return -1;
}

int32_t ide_write_sectors(uint32_t lba, uint32_t count, const uint8_t *buf) {
    uint32_t s = 0;
    uint8_t st = 0;
    uint8_t fail_kind = IDE_FAIL_TIMEOUT;
    if (count == 0) return -1;
    if (!ide_wait_until_ready(5000)) {
        record_failure(IDE_FAIL_NOT_READY, 0, ide_read_reg(7), lba, 0, count);
        return -1;
    }
    if (!chs_geometry_ok()) {
        record_failure(IDE_FAIL_NO_GEOMETRY, 0x91, ide_read_reg(7), lba, 0, count);
        return -1;
    }

    bool use_lba48 = config.use_lba_mode && (config.lba_sectors > 0x0FFFFFFF);

    if (config.use_lba_mode) {
        if (use_lba48) {
            ide_write_reg(2, 0);                               // sector count high
            ide_write_reg(3, (lba >> 24) & 0xFF);              // LBA 24-31
            ide_write_reg(4, 0);                               // LBA 32-39
            ide_write_reg(5, 0);                               // LBA 40-47
            ide_write_reg(2, (uint8_t)count);                  // sector count low
            ide_write_reg(3, lba & 0xFF);                      // LBA 0-7
            ide_write_reg(4, (lba >> 8) & 0xFF);               // LBA 8-15
            ide_write_reg(5, (lba >> 16) & 0xFF);              // LBA 16-23
            ide_write_reg(6, dev_base | 0x40);                 // LBA mode, no address bits
        } else {
            ide_write_reg(2, (uint8_t)count);
            ide_write_reg(3, lba & 0xFF);
            ide_write_reg(4, (lba >> 8) & 0xFF);
            ide_write_reg(5, (lba >> 16) & 0xFF);
            ide_write_reg(6, (dev_base | 0x40) | ((lba >> 24) & 0x0F));
        }
    } else {
        uint32_t tmp  = lba / config.spt;
        uint8_t  sec  = (lba % config.spt) + 1;
        uint8_t  head = tmp % config.heads;
        uint16_t cyl  = tmp / config.heads;
        ide_write_reg(2, (uint8_t)count);
        ide_write_reg(3, sec);
        ide_write_reg(4, cyl & 0xFF);
        ide_write_reg(5, (cyl >> 8) & 0xFF);
        ide_write_reg(6, dev_base | (head & 0x0F));
    }

    uint8_t cmd = use_lba48 ? 0x34 : 0x30;
    ide_write_reg(7, cmd);                                 // WRITE SECTORS EXT / WRITE SECTORS
    wait_after_command();

    const uint16_t *wbuf = (const uint16_t *)buf;

    bool write_ok = true;

    for (s = 0; s < count; s++) {
        // Poll for DRQ — INTRQ not asserted for first sector of PIO write per ATA spec;
        // for s > 0 it provides early-exit if enabled, otherwise pure polling
        bool got_drq = false;
        for (uint32_t t = 0; t < 100000; t++) {
            if (s > 0 && config.intrq_enabled && gpio_get(IDE_INTRQ)) ide_read_reg(7);
            st = ide_read_reg(7);
            if (st & 0x80) { busy_wait_us_32(10); continue; }   // BSY: other bits not valid
            if (st & 0x01) { write_ok = false; fail_kind = IDE_FAIL_ERR; break; }
            if (st & 0x08) { got_drq = true; break; }
            busy_wait_us_32(10);
        }
        if (!write_ok || !got_drq) { write_ok = false; break; }

        set_address(0);
        xcvr_write();
        sio_hw->gpio_clr = (1 << IDE_CS0);

        ide_pio_write(256, wbuf + s * 256);

        sio_hw->gpio_set = (1 << IDE_CS0);
        bus_idle();
    }

    // Wait for drive to commit the last sector to media (BSY=0)
    if (write_ok) {
        for (uint32_t t = 0; t < 100000; t++) {
            if (config.intrq_enabled && gpio_get(IDE_INTRQ)) ide_read_reg(7);
            st = ide_read_reg(7);
            if (st & 0x80) { busy_wait_us_32(10); continue; }   // BSY: other bits not valid
            if (st & 0x01) { write_ok = false; fail_kind = IDE_FAIL_ERR; break; }
            return (int32_t)(count * 512);
        }
    }

    // Write errors still always reset, as before. Keep the registers first.
    // For a write, 'done' is the number of sectors sent to the drive.
    record_failure(fail_kind, cmd, st, lba + s, s, count);
    soft_reset_restore();
    return -1;
}

// ---------------------------------------------------------------------------
//  Diagnostics — task file snapshot and seek/read-one
// ---------------------------------------------------------------------------

void ide_read_taskfile(uint8_t tf[8]) {
    for (int i = 1; i <= 7; i++) tf[i] = ide_read_reg(i);
}

uint8_t ide_seek_read_one(uint32_t target, bool lba) {
    bool use_lba48 = lba && (config.lba_sectors > 0x0FFFFFFF);

    if (lba) {
        if (use_lba48) {
            ide_write_reg(2, 0);                                   // sector count high
            ide_write_reg(3, (target >> 24) & 0xFF);               // LBA 24-31
            ide_write_reg(4, 0);                                   // LBA 32-39
            ide_write_reg(5, 0);                                   // LBA 40-47
            ide_write_reg(2, 1);                                   // sector count low
            ide_write_reg(3, target & 0xFF);                       // LBA 0-7
            ide_write_reg(4, (target >> 8) & 0xFF);                // LBA 8-15
            ide_write_reg(5, (target >> 16) & 0xFF);               // LBA 16-23
            ide_write_reg(6, dev_base | 0x40);                     // LBA mode
        } else {
            ide_write_reg(2, 1);
            ide_write_reg(3, target & 0xFF);
            ide_write_reg(4, (target >> 8) & 0xFF);
            ide_write_reg(5, (target >> 16) & 0xFF);
            ide_write_reg(6, (dev_base | 0x40) | ((target >> 24) & 0x0F));
        }
    } else {
        uint16_t cyl = (uint16_t)target;
        ide_write_reg(2, 1);
        ide_write_reg(3, 1);                                   // sector 1 (1-based)
        ide_write_reg(4, cyl & 0xFF);
        ide_write_reg(5, (cyl >> 8) & 0xFF);
        ide_write_reg(6, dev_base);                            // head 0, CHS
    }
    ide_write_reg(7, use_lba48 ? 0x24 : 0x20);                // READ SECTORS EXT / READ SECTORS
    wait_after_command();

    // Wait for BSY to clear
    for (uint32_t t = 0; t < 10000; t++) {
        if (config.intrq_enabled && gpio_get(IDE_INTRQ)) ide_read_reg(7);  // clear INTRQ
        if (!(ide_read_reg(7) & 0x80)) break;
        busy_wait_us_32(10);
    }

    // Drain DRQ data if present
    if (ide_read_reg(7) & 0x08) ide_drain_sector();

    return ide_read_reg(7);
}

#if ATABOY_SAT
#include <string.h>

// ---------------------------------------------------------------------------
//  SAT pass-through: one command, task file exactly as given
// ---------------------------------------------------------------------------
//
// Used only by sat.c, for the commands sat_policy.c allows: PIO data-in
// (IDENTIFY, READ SECTORS, READ SECTORS EXT, SMART READ DATA / THRESHOLDS /
// LOG) and non-data (READ VERIFY, SMART RETURN STATUS, READ NATIVE MAX and
// its EXT form). Differs from ide_read_sectors() on purpose:
//  - it sends the ATA command byte the host chose (0x21 stays 0x21), and the
//    48-bit registers only when the host asked for a 48-bit command;
//  - on an error it does NOT soft-reset the drive or re-send INITIALIZE DRIVE
//    PARAMETERS. It stops, reads the drive's registers for the caller, and
//    reports the failure. The READ(10) path is unchanged.
//  - it refuses to start if the drive is busy or holds data from some earlier
//    command, instead of guessing what that state means.
// Timeouts are wall-clock, not loop counts.

#define SAT_READY_TIMEOUT_MS  5000    // same as ide_read_sectors()
#define SAT_CMD_TIMEOUT_MS    10000   // PIO: from issue to the last block; non-data: to completion
#define SAT_END_TIMEOUT_MS    1000    // after the last block, for BSY and DRQ to drop

// Abort whatever the drive is still doing for a SAT command we gave up on.
// Without this, a drive that finishes a slow read after our timeout raises
// DRQ with that sector's data, and the next command on the bus (a READ(10)
// from the host, say) can end up handed the stranded block as its own
// result. A soft reset ends the command, and srst_and_restore() selects the
// drive again and, in CHS mode, puts the geometry back. There is no error
// state worth keeping at this point: the command timed out or ended badly.
static void sat_abort(void) {
    (void)srst_and_restore();   // on failure, CHS reads refuse until restored
}

// Ready to take a new command: not busy, and not holding data from an
// earlier one (that is not ours to discard; the READ(10) path's own guard
// resets it). False means nothing was sent.
static bool sat_can_issue(void) {
    if (!ide_wait_until_ready(SAT_READY_TIMEOUT_MS)) return false;
    if (ide_read_reg(7) & 0x08) return false;   // stale DRQ
    return true;
}

// Has the drive visibly started the SAT command we just wrote? (Review
// finding M1.) Before it raises BSY, which ATA allows it 400 ns for and a slow
// drive can take longer over, status still reads as it did before the
// command: DRDY with BSY clear, and ERR if the last command failed. Taking
// that as the end would hand the host the registers it just wrote as the
// drive's answer (READ VERIFY "good", SMART "passed", READ NATIVE MAX 0), and
// a command the drive never started would never be aborted. So on the SAT
// path a status with BSY clear is believed only once the command is known to
// have started:
//  - BSY was seen set on some poll since the command was written; or
//  - INTRQ was high when sampled just before a status read. Reading Status
//    releases INTRQ, so it is sampled first. Writing the command register
//    released any earlier one, and a drive raises it only when the command
//    ends or a block is ready, so it cannot be left over from before. It is
//    believed only if it read low just before the command went out: a line
//    stuck high tells us nothing (and one never wired stays low, leaving BSY).
//  - (PIO data-in) DRQ: sat_can_issue() refused to start over a stale one.
// If neither shows before the timeout, the command is aborted with SRST and
// reported as a timeout, never as success.
// nIEN is 0 throughout (ide_hw_init, and every Device Control write in this
// file keeps it 0), so the selected drive drives INTRQ. The features menu's
// INTRQ switch only decides whether the READ(10) path uses INTRQ to cut its
// polling short; here INTRQ is read either way, as evidence, and it never
// shortens a wait.
typedef struct {
    bool irq_usable;    // INTRQ read low just before the command was written
    bool started;       // BSY or INTRQ (or DRQ) seen since
} sat_watch_t;

// Call after sat_can_issue() (whose status reads released INTRQ) and right
// before sat_write_taskfile().
static void sat_watch_arm(sat_watch_t *w) {
    w->irq_usable = !gpio_get(IDE_INTRQ);
    w->started = false;
}

static uint8_t sat_poll(sat_watch_t *w) {
    bool irq = w->irq_usable && gpio_get(IDE_INTRQ);   // before the status read releases it
    uint8_t st = ide_read_reg(7);
    if ((st & 0x80) || irq) w->started = true;
    return st;
}

static void sat_write_taskfile(const sat_taskfile_t *tf) {
    if (tf->ext) {
        // 48-bit: previous (HOB) value first, then current, per register
        ide_write_reg(1, tf->hob_feature);
        ide_write_reg(2, tf->hob_count);
        ide_write_reg(3, tf->hob_lba_low);
        ide_write_reg(4, tf->hob_lba_mid);
        ide_write_reg(5, tf->hob_lba_high);
    }
    ide_write_reg(1, tf->feature);
    ide_write_reg(2, tf->count);
    ide_write_reg(3, tf->lba_low);
    ide_write_reg(4, tf->lba_mid);
    ide_write_reg(5, tf->lba_high);
    ide_write_reg(6, dev_base | (tf->device & 0x4F));
    ide_write_reg(7, tf->command);
    busy_wait_us_32(1);             // give the drive time to assert BSY (400 ns)
}

// Read the output registers once the drive has ended the command (BSY=0).
// Registers 1-6 only: reading them has no side effect, and the data register
// is never touched. `st` is the status that ended the command.
static void sat_read_outputs(const sat_taskfile_t *tf, uint8_t st, ide_sat_regs_t *r) {
    r->status   = st;
    r->error    = ide_read_reg(1);
    r->count    = ide_read_reg(2);
    r->lba_low  = ide_read_reg(3);
    r->lba_mid  = ide_read_reg(4);
    r->lba_high = ide_read_reg(5);
    r->device   = ide_read_reg(6);
    bool aborted = (st & 0x01) && (r->error & 0x04);
    if (tf->ext && !aborted) {
        // HOB=1 in Device Control reads back the previous (high) bytes of
        // registers 2-5. nIEN stays 0, as everywhere else in this file.
        ide_write_control(0x80);
        r->hob_count    = ide_read_reg(2);
        r->hob_lba_low  = ide_read_reg(3);
        r->hob_lba_mid  = ide_read_reg(4);
        r->hob_lba_high = ide_read_reg(5);
        ide_write_control(0x00);
        r->hob = true;
    }
}

int ide_sat_pio_in(const sat_taskfile_t *tf, uint8_t *buf, ide_sat_regs_t *regs) {
    memset(regs, 0, sizeof(*regs));
    if (tf->protocol != SAT_PROTO_PIO_IN) return IDE_SAT_NOT_ISSUED;
    if (tf->sectors == 0 || tf->sectors > SAT_MAX_SECTORS) return IDE_SAT_NOT_ISSUED;
    if (!sat_can_issue()) return IDE_SAT_NOT_ISSUED;

    sat_watch_t w;
    sat_watch_arm(&w);
    sat_write_taskfile(tf);

    uint16_t *wbuf = (uint16_t *)buf;
    uint32_t start = to_ms_since_boot(get_absolute_time());

    for (uint32_t s = 0; s < tf->sectors; s++) {
        for (;;) {
            uint8_t st = sat_poll(&w);                          // also clears INTRQ
            if (!(st & 0x80)) {                                 // other bits only valid with BSY=0
                if (st & 0x08) w.started = true;                // DRQ is this command's (sat_can_issue)
                // ERR / DF before the command has started are the last
                // command's, not this one's (sat_watch_t): keep waiting.
                if (w.started && (st & 0x21)) {                 // ERR, or DF (device fault)
                    sat_read_outputs(tf, st, regs);             // before the drain changes anything
                    if (st & 0x08) ide_drain_sector();          // don't leave DRQ stranded
                    return IDE_SAT_ATA_ERROR;
                }
                if (st & 0x08) break;                           // DRQ: a block is ready
            }
            if (to_ms_since_boot(get_absolute_time()) - start >= SAT_CMD_TIMEOUT_MS) {
                sat_abort();
                return IDE_SAT_TIMEOUT;
            }
            busy_wait_us_32(10);
        }

        set_address(0);
        xcvr_read();
        sio_hw->gpio_clr = (1 << IDE_CS0);

        ide_pio_read(256, wbuf + s * 256);

        sio_hw->gpio_set = (1 << IDE_CS0);
        bus_idle();
    }

    // All blocks read. The drive must now end the command cleanly: not busy,
    // no error, and no more data on offer. DRQ can take a moment to drop
    // after the last word, so it gets a short window rather than one look.
    busy_wait_us_32(1);
    uint32_t end_start = to_ms_since_boot(get_absolute_time());
    for (;;) {
        uint8_t st = ide_read_reg(7);
        if (!(st & 0x80)) {
            if (st & 0x21) { sat_read_outputs(tf, st, regs); return IDE_SAT_ATA_ERROR; }
            if (!(st & 0x08)) return IDE_SAT_OK;
        }
        if (to_ms_since_boot(get_absolute_time()) - end_start >= SAT_END_TIMEOUT_MS) {
            sat_abort();
            return (st & 0x80) ? IDE_SAT_TIMEOUT : IDE_SAT_BAD_END;
        }
        busy_wait_us_32(10);
    }
}

// Non-data: issue, wait for BSY to drop, read the output registers. A drive
// that answers a non-data command with DRQ is confused; its data is never
// read (the data register is not touched on this path at all), and a soft
// reset ends the command so nothing is left stranded for the next one.
int ide_sat_nondata(const sat_taskfile_t *tf, ide_sat_regs_t *regs) {
    memset(regs, 0, sizeof(*regs));
    if (tf->protocol != SAT_PROTO_NON_DATA || tf->sectors != 0) return IDE_SAT_NOT_ISSUED;
    if (!sat_can_issue()) return IDE_SAT_NOT_ISSUED;

    sat_watch_t w;
    sat_watch_arm(&w);
    sat_write_taskfile(tf);

    uint32_t start = to_ms_since_boot(get_absolute_time());
    for (;;) {
        uint8_t st = sat_poll(&w);                              // also clears INTRQ
        // BSY clear ends the command only once it has started (sat_watch_t);
        // before that it is the status from before the command.
        if (!(st & 0x80) && w.started) {                        // other bits only valid with BSY=0
            sat_read_outputs(tf, st, regs);
            if (st & 0x08) {                                    // DRQ on a non-data command
                sat_abort();
                return IDE_SAT_BAD_END;
            }
            return (st & 0x21) ? IDE_SAT_ATA_ERROR : IDE_SAT_OK;   // ERR or DF
        }
        if (to_ms_since_boot(get_absolute_time()) - start >= SAT_CMD_TIMEOUT_MS) {
            sat_abort();
            return IDE_SAT_TIMEOUT;
        }
        busy_wait_us_32(10);
    }
}
#endif
