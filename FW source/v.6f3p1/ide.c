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
// IDENTIFY words 82..85 and 87 kept for the SAT policy (ide.h, ide_id_words).
// Who writes them, on which core:
//  - ide_identify() and ide_probe_devices() capture or clear them. They run
//    on core 1 (detection, auto-mount, the debug screen) while nothing is
//    mounted, and core 1 waits for any USB command still running before it
//    touches the bus (menus.c, review L-5).
//  - the identity check, ide_id_words_verify(), clears valid on core 0 while
//    a drive is mounted, when the drive no longer matches (review L-4
//    corrected an earlier comment here that said only core 1 writes them).
//  - ide_id_words_forget() clears valid on core 1 at unmount.
// sat.c reads them on core 0, and only issues anything while a drive is
// mounted. The policy checks "mounted" before it looks at these, so a read
// that races a detection is refused anyway. Only core 0 runs SAT code, and
// core 1 only writes them while unmounted, so the two never write at once.
// A hardware or soft reset does not forget them: a reset cannot change which
// drive is on the cable, and the identity check asks the drive again before
// every command the words let through.
static struct {
    bool     valid;
    uint8_t  dev_base;          // the device they came from
    uint16_t w82, w83, w84, w85, w87;
    uint16_t serial[10];        // words 10..19, to tell this drive from another
    uint16_t model[20];         // words 27..46
} id_words;

void ide_id_words_forget(void) { id_words.valid = false; }

// Words held, and taken from the device selected now. The one test of it,
// shared by the policy's view (ide_id_words) and the identity check.
static bool id_words_held(void) { return id_words.valid && id_words.dev_base == dev_base; }

void ide_id_words(ide_id_words_t *out) {
    out->valid = id_words_held();
    out->w82 = id_words.w82;
    out->w83 = id_words.w83;
    out->w84 = id_words.w84;
    out->w85 = id_words.w85;
    out->w87 = id_words.w87;
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
// and RECALIBRATE assume the drive has raised BSY by the time
// this 1 us is up: their first status read takes BSY clear as "finished"
// (the reads and writes then still need DRQ, which a stale status does not
// have, but a stale ERR would end them early). A drive slower than that to
// raise BSY breaks the assumption. Those paths are hardware-validated and
// left as they are; the SAT path and, since 0.6f3p7, IDENTIFY do not rely
// on it (sat_watch_t below).
static void wait_after_command(void) {
    busy_wait_us_32(1);
}

// ---------------------------------------------------------------------------
//  Wall-clock limits for sector I/O (0.6f3p7)
// ---------------------------------------------------------------------------
// Found on hardware 2026-09-24 (ST380011A, alone on the cable as a slave,
// 0.6f3p6): after about 90 single-sector writes at 10 to 20 ms each, one
// write kept the drive busy past the commit wait. That wait counted polls
// (100000 of them, 10 us apart, so about 1 s plus the register reads), not
// time. The firmware took it as a timeout and soft-reset the drive in the
// middle of its own work, and the drive did not come back until a hardware
// reset. The sector did hold the new data. Every wait on a sector command
// is now measured against the clock.
//
// IDE_CMD_TIMEOUT_MS runs from writing a READ or WRITE SECTORS command until
// its last block has moved and the drive has dropped BSY. One limit for the
// whole command, not a guess per poll or per sector, because the USB host
// times whole commands too.
//  - ATA puts no limit on a read or a write. A drive may retry, recalibrate
//    or flush internally for as long as it likes; ATA's only fixed number is
//    the 31 s a drive may take to come back after a reset.
//  - Linux gives a disk command 30 s (SD_TIMEOUT in drivers/scsi/sd.h), then
//    aborts it, which for usb-storage means resetting the USB device.
//  - Windows' disk class driver uses Services\Disk\TimeOutValue, or 10 s if
//    that is not set (Microsoft, "Registry Entries for SCSI Miniport
//    Drivers"). Many installs set it higher; the development PC has 65 s. A
//    pass-through caller sets its own (the palimpsest tools use 3 to 60 s).
//  - While a callback is blocked here, TinyUSB NAKs the bulk endpoint, so
//    the host simply waits until its own timeout. Giving up earlier gains
//    the host nothing; it only means resetting a drive that was still busy.
// So the firmware waits as long as a host will plausibly wait, and no longer:
// 30 s, the Linux figure and three times the Windows default. Past that the
// host has already failed the command, and core 0 is stuck in the callback
// (no console, no USB) for as long as the wait goes on. A drive still busy
// after 30 s on one command is soft-reset, as before.
#define IDE_CMD_TIMEOUT_MS      30000
// IDENTIFY DEVICE answers from the drive's own memory. 10 s is the SAT
// path's command limit; it used to be 100000 polls 50 us apart (5 s plus).
#define IDE_IDENTIFY_TIMEOUT_MS 10000
// The debug seek test only waits so as not to write the next command over a
// busy drive. It used to be 10000 polls (about 0.1 s); 1 s is kinder to a
// slow drive and still keeps the test moving.
#define IDE_SEEK_TIMEOUT_MS     1000

static uint32_t ms_now(void) { return to_ms_since_boot(get_absolute_time()); }
static bool ms_passed(uint32_t start, uint32_t limit_ms) { return ms_now() - start >= limit_ms; }

// Has the drive visibly started the command we just wrote? (Review
// finding M1, on the SAT path, where this began; IDENTIFY uses it too since
// 0.6f3p7, review L-3.) Before it raises BSY, which ATA allows it 400 ns for
// and a slow
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
    // Review L-3: ERR is believed only once this IDENTIFY has visibly
    // started (sat_watch_t). The status read above released INTRQ. A stale
    // ERR from an earlier command, seen before a slow drive raised BSY,
    // used to end IDENTIFY at once; for the identity check that meant
    // forgetting the words of a drive that was fine.
    sat_watch_t w;
    sat_watch_arm(&w);
    ide_write_reg(7, 0xEC);
    busy_wait_us_32(1);             // give drive time to assert BSY

    // Poll for DRQ. Status is read through sat_poll(), which samples INTRQ
    // first and releases it with the status read.
    uint32_t start = ms_now();
    for (;;) {
        if (ms_passed(start, IDE_IDENTIFY_TIMEOUT_MS)) return false;   // identify: no data in time
        uint8_t st = sat_poll(&w);
        if (st & 0x80) { busy_wait_us_32(50); continue; }  // BSY: other bits not valid yet
        if (st & 0x08) w.started = true;                 // DRQ is ours: a stale one was drained above
        if (w.started && (st & 0x01)) { if (st & 0x08) ide_drain_sector(); return false; }  // ERR: drain stranded DRQ
        if (st & 0x08) goto ready;                       // BSY=0, DRQ=1
        busy_wait_us_32(50);
    }

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
        id_words.w85 = buf[85];
        id_words.w87 = buf[87];
        for (int i = 0; i < 10; i++) id_words.serial[i] = buf[10 + i];
        for (int i = 0; i < 20; i++) id_words.model[i]  = buf[27 + i];
    }
#endif
    return ok;
}

#if ATABOY_SAT
// Is the drive on the cable still the one the words were captured from? Asks
// it for IDENTIFY again and compares serial, model and words 82..85 and 87.
// Called by sat.c right before a command the words let through (SMART, READ
// NATIVE MAX, READ SECTORS EXT: every row the policy marks needs_identity):
// a drive swapped after detection, mounted without a new
// one, would otherwise be judged by the old drive's words. Sends nothing when
// no words were captured, so a drive set up without IDENTIFY (manual CHS, as
// the CP3044 must be) never gets one from here. Any difference, or a failed
// IDENTIFY, forgets the words. Runs on core 0 while mounted, like all SAT I/O.
// Returns 1 same drive, 0 not (words forgotten) or no words held, -1 drive
// not ready or still offering data from an earlier command: then nothing is
// sent (identify_once would drain that data and issue IDENTIFY over it, which
// the SAT path refuses to do) and the words are kept.
// Word 85 can change without a drive swap (SMART or HPA switched on or off by
// another host); that also forgets the words, and the gated rows refuse until
// the next detection. That errs on the side of refusing.
// Residual gap (review L-4, accepted): between this check and the command
// itself only core 0 code runs, a few microseconds of it. A drive swapped or
// power-cycled inside that window is absent (status 0xFF) or still in its
// power-on reset (BSY, for far longer than the window), so the command that
// follows is refused or times out. No swap completes in a few microseconds.
int ide_id_words_verify(void) {
    if (!id_words_held()) return 0;
    if (!ide_wait_until_ready(1000) || (ide_read_reg(7) & 0x08)) return -1;
    uint16_t buf[256];
    bool same = identify_once(buf) && buf[82] == id_words.w82 &&
                buf[83] == id_words.w83 && buf[84] == id_words.w84;
    same = same && buf[85] == id_words.w85 && buf[87] == id_words.w87;
    for (int i = 0; same && i < 10; i++) same = buf[10 + i] == id_words.serial[i];
    for (int i = 0; same && i < 20; i++) same = buf[27 + i] == id_words.model[i];
    if (!same) id_words.valid = false;
    return same ? 1 : 0;
}
#endif

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
    last_fail.hw_reset = false;
    last_fail.lba     = lba;
    last_fail.done    = done;
    last_fail.count   = count;
}

#define IDE_SRST_TIMEOUT_MS 31000   // ATA allows a device up to 31 s after SRST
// RESET- low time. ATA asks for at least 25 us; the probe (ide_probe_devices)
// and ide_reset_drive() hold it for 50 ms, and so does the escalation below.
#define IDE_HW_RESET_LOW_US 50000

// After a soft or hardware reset: wait on device 0 where ATA says to, select
// our device again, and wait for it to be ready. True if it came back ready
// within IDE_SRST_TIMEOUT_MS.
static bool reselect_after_reset(void) {
    // A reset leaves device 0 selected, and device 0 holds BSY until device 1
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
    return ide_wait_until_ready(IDE_SRST_TIMEOUT_MS);
}

// Soft reset (SRST in Device Control). True if the drive came back ready.
static bool srst_and_reselect(void) {
    ide_write_control(0x04);
    busy_wait_us_32(10);
    ide_write_control(0x00);
    chs_geometry_lost = true;               // SRST drops INITIALIZE DEVICE PARAMETERS
    busy_wait_us_32(2000);                  // ATA: 2 ms before status is valid
    return reselect_after_reset();
}

// Hardware reset: RESET- on the cable, as the probe uses. It resets BOTH
// devices on the cable, not only ours; a master sharing the cable with a
// slave in use loses its own settings too (nothing here uses them). The drive
// then runs its power-on diagnostics, holding IORDY low meanwhile, so IORDY
// is ignored until it is ready. RECALIBRATE follows, as after every other
// hardware reset in this firmware (some pre-ATA drives need it before they
// will seek). Runs on core 0 inside a USB callback, so it busy-waits.
// The CHS geometry is already marked lost by the soft reset that came first.
static bool hw_reset_and_reselect(void) {
    ide_set_iordy(false);
    gpio_put(IDE_RESET, 0);
    busy_wait_us_32(IDE_HW_RESET_LOW_US);
    gpio_put(IDE_RESET, 1);
    busy_wait_us_32(2000);                  // as after SRST: 2 ms before status is valid
    ide_write_control(0x00);                // nIEN=0, as the probe does
    bool ok = reselect_after_reset();
    if (ok) {
        ide_write_reg(7, 0x10);             // RECALIBRATE, to the device just selected
        wait_after_command();
        ok = ide_wait_until_ready(10000);
    }
    ide_set_iordy(config.iordy_enabled);
    return ok;
}

// Reset a drive that is stuck or did not end a command cleanly, then put the
// CHS translation back. A soft reset first. If the drive does not come back
// ready from that, one hardware reset, and no more: found on hardware
// 2026-09-24, a lone slave ST380011A stayed busy through the soft reset and
// then read 0xFF until a re-detect pulsed RESET-. *hw_used says whether the
// hardware reset was needed. True only if the drive came back ready and, in
// CHS mode, took the geometry. On false, chs_geometry_lost stays set and CHS
// transfers refuse until a later ide_set_geometry() succeeds.
static bool reset_and_restore(bool *hw_used) {
    *hw_used = false;
    bool ready = srst_and_reselect();
    if (!ready) {
        *hw_used = true;
        ready = hw_reset_and_reselect();
    }
    if (!ready) return false;
    if (config.use_lba_mode) return true;
    return ide_set_geometry(config.heads, config.spt);
}

// Reset for the sector I/O paths, recorded in the failure record.
static void soft_reset_restore(void) {
    last_fail.reset = true;
    bool hw;
    last_fail.reset_failed = !reset_and_restore(&hw);
    last_fail.hw_reset = hw;
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
    uint32_t start = ms_now();                             // the whole command: IDE_CMD_TIMEOUT_MS

    uint16_t *wbuf = (uint16_t *)buf;

    for (s = 0; s < count; s++) {
        // Poll for DRQ — INTRQ provides early-exit if enabled, otherwise pure polling
        for (;;) {
            if (ms_passed(start, IDE_CMD_TIMEOUT_MS)) goto read_timeout;   // read: no DRQ in time
            if (config.intrq_enabled && gpio_get(IDE_INTRQ)) ide_read_reg(7);  // clear INTRQ
            st = ide_read_reg(7);
            // While BSY is set the other status bits are not valid (ATA), so a
            // leftover ERR must not end a command the drive is still working on.
            if (st & 0x80) { busy_wait_us_32(10); continue; }
            if (st & 0x01) goto read_err;
            if (st & 0x08) goto drq_read;
            busy_wait_us_32(10);
        }

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
    // No DRQ within IDE_CMD_TIMEOUT_MS: the drive may still be retrying, so
    // abort with SRST (and, if it does not come back, one hardware reset).
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
    uint32_t start = ms_now();                             // the whole command: IDE_CMD_TIMEOUT_MS

    const uint16_t *wbuf = (const uint16_t *)buf;

    bool write_ok = true;

    for (s = 0; s < count; s++) {
        // Poll for DRQ — INTRQ not asserted for first sector of PIO write per ATA spec;
        // for s > 0 it provides early-exit if enabled, otherwise pure polling
        bool got_drq = false;
        for (;;) {
            if (ms_passed(start, IDE_CMD_TIMEOUT_MS)) break;   // write: no DRQ in time
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

    // Wait for drive to commit the last sector to media (BSY=0). This is the
    // wait the ST380011A outlasted (IDE_CMD_TIMEOUT_MS above).
    if (write_ok) {
        for (;;) {
            if (ms_passed(start, IDE_CMD_TIMEOUT_MS)) break;   // write commit: still busy
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
    uint32_t start = ms_now();
    while (!ms_passed(start, IDE_SEEK_TIMEOUT_MS)) {
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
// result. A soft reset ends the command, and reset_and_restore() selects the
// drive again and, in CHS mode, puts the geometry back; a drive that does not
// come back from the soft reset gets one hardware reset, as on the READ(10)
// path. There is no error state worth keeping at this point: the command
// timed out or ended badly.
static void sat_abort(void) {
    bool hw;
    (void)reset_and_restore(&hw);   // on failure, CHS reads refuse until restored
}

// Ready to take a new command: not busy, and not holding data from an
// earlier one (that is not ours to discard; the READ(10) path's own guard
// resets it). False means nothing was sent.
static bool sat_can_issue(void) {
    if (!ide_wait_until_ready(SAT_READY_TIMEOUT_MS)) return false;
    if (ide_read_reg(7) & 0x08) return false;   // stale DRQ
    return true;
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
