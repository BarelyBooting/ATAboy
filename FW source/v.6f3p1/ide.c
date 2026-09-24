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

// IORDY held high (ignored) from a hardware reset until the drive is known to
// be back. A drive holds IORDY low through its power-on diagnostics, and the
// PIO program waits on IORDY with no timeout: a register read with IORDY
// believed, while a drive that never came back holds it low, would never
// finish, and core 0 would hang inside the callback for good (review L-3).
// So the configured setting comes back only when the drive has shown ready
// (ide_wait_until_ready), whenever that is.
static bool iordy_held = false;

static void iordy_hold(void) {
    iordy_held = true;
    ide_set_iordy(false);
}

static void iordy_release(void) {
    if (!iordy_held) return;
    iordy_held = false;
    ide_set_iordy(config.iordy_enabled);
}

// Put config.iordy_enabled on the pin, unless IORDY is held ignored after a
// hardware reset: then it waits for the drive to show ready, and
// iordy_release() puts the setting on the pin as it is at that moment.
// Review of 0.6f3p7 (LOW): the Features menu used to set the pin itself, so
// switching IORDY on while a hardware reset's recovery was still waiting for
// the drive undid iordy_hold(), and the next register read could hang core 0
// on a drive holding IORDY low in its power-on diagnostics. Called by the
// menu on core 1 only while nothing is mounted and no USB command is running
// (menus.c), and by core 0 at every host command (ide_host_cmd_enter), so
// only the core that owns the bus ever sets the pin.
void ide_iordy_follow_config(void) {
    if (!iordy_held) ide_set_iordy(config.iordy_enabled);
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
// How long a host waits. ATA puts no limit on a read or a write: a drive may
// retry, recalibrate or flush internally for as long as it likes, and ATA's
// only fixed number is the 31 s a drive may take to come back after a reset.
// The host does put a limit on the whole SCSI command:
//  - Linux gives a disk command 30 s (SD_TIMEOUT in drivers/scsi/sd.h), then
//    aborts it, which for usb-storage means a bulk-only reset or a USB port
//    reset of the whole device.
//  - The palimpsest tools send READ(10) and ATA PASS-THROUGH with their own
//    30 s pass-through timeout (tools/ataboy-read10.ps1,
//    tools/ataboy-donor-write.ps1).
//  - Windows' disk class driver uses Services\Disk\TimeOutValue, or 10 s if
//    that is not set (Microsoft, "Registry Entries for SCSI Miniport
//    Drivers"). Many installs set it higher; the development PC has 65 s.
//
// The first 0.6f3p7 draft waited 30 s per ATA command here and said that
// giving up earlier gains the host nothing. Review M-1 found that wrong: it
// loses the host everything. While a callback blocks here TinyUSB NAKs the
// bulk endpoint and the host's clock runs; the host's clock also started
// before ours (the CBW and any earlier chunks), and the issue #13 call again
// at the failing sector, and each reset, used to get time of its own on top
// (one 8-sector READ(10) with a hang at the fourth sector: 60 s and two soft
// resets, where 0.6f3p6 took 2 s). When the host gives up it resets the
// device while core 0 is still in the callback, so the good prefix that
// issue #13 exists to return is lost. The reviewer's reading of TinyUSB 0.18
// (dcd_rp2040.c, usbd.c; not measured) is that the bus reset also clears the
// endpoint state under the callback, which could leave USB wedged.
//
// So since the review, ONE budget covers everything done for one host
// command: IDE_HOST_BUDGET_MS (20 s, ide.h), counted from the first callback
// of the command (usb.c finds the command boundaries). That is 10 s under the
// 30 s floor of Linux and our tools, which leaves the USB transfer (at 12
// Mbit/s the largest READ(10) Linux sends, 120 KiB, takes about 0.1 s) and
// the host's own slack well inside. Windows' 10 s default cannot be met by a
// drive that legitimately retries for longer; FORK-README asks Windows users
// to set TimeOutValue to 30 or more.
// Inside the budget, waits for the drive to do the host's work end
// IDE_RECOVERY_RESERVE_MS (5 s) before the budget does, so a drive that hangs
// can still be soft-reset, and usually brought back, inside the same host
// command. Recovery may use everything that is left; a drive not back by then
// is carried on with by the next host command (recovery, below).
// IDE_CMD_TIMEOUT_MS runs from writing a READ or WRITE SECTORS command until
// its last block has moved and the drive has dropped BSY. One limit for the
// whole command, not a guess per poll or per sector, as the host times whole
// commands too. It is what a command sent at the start of a host command gets
// (20 s less the 5 s reserve), and what the functions get when called outside
// a host command (the host tests call them directly). A drive still busy when
// its time is up is soft-reset, as before.
#define IDE_CMD_TIMEOUT_MS      (IDE_HOST_BUDGET_MS - IDE_RECOVERY_RESERVE_MS)
// IDENTIFY DEVICE answers from the drive's own memory. 10 s is the SAT
// path's command limit; it used to be 100000 polls 50 us apart (5 s plus).
#define IDE_IDENTIFY_TIMEOUT_MS 10000
// The debug seek test only waits so as not to write the next command over a
// busy drive. It used to be 10000 polls (about 0.1 s); 1 s is kinder to a
// slow drive and still keeps the test moving.
#define IDE_SEEK_TIMEOUT_MS     1000

static uint32_t ms_now(void) { return to_ms_since_boot(get_absolute_time()); }
// The one deadline test. At exactly limit_ms the time is up (a boundary test
// in tests/host holds it there).
static bool ms_passed(uint32_t start, uint32_t limit_ms) { return ms_now() - start >= limit_ms; }

// The host command in progress (ide.h, ide_host_cmd_*). Written and read only
// on core 0, inside MSC callbacks: `on` is false everywhere else, so core 1's
// waits (detection, the debug keys, auto-mount) are never cut short by it.
// usb.c only enters a host command while a drive is mounted, so a callback
// that refuses because nothing is mounted leaves `on` alone while core 1 may
// be detecting a drive.
static struct {
    bool     on;            // core 0 is inside a callback of this command
    uint32_t start;         // when the command's first callback began (ms)
    uint8_t  resets;        // soft resets started during the command
    uint8_t  hw_resets;     // hardware resets during the command
    bool     recorded;      // the command has written a failure record
    bool     failed;        // a sector read failed in this command (issue #13):
    uint32_t fail_lba;      //   this sector,
    uint32_t fail_ms;       //   after the drive had worked on it this long
} host;

void ide_host_cmd_begin(void) {
    host.start = ms_now();
    host.resets = 0;
    host.hw_resets = 0;
    host.recorded = false;
    host.failed = false;
}
// The IORDY setting from the Features menu is put on the pin here, on core 0,
// while a drive is mounted (ide_iordy_follow_config).
void ide_host_cmd_enter(void) {
    host.on = true;
    ide_iordy_follow_config();
}
void ide_host_cmd_leave(void) { host.on = false; }

// What is left of the host command's time; no limit outside one.
static uint32_t host_left_ms(void) {
    if (!host.on) return UINT32_MAX;
    uint32_t used = ms_now() - host.start;
    return used >= IDE_HOST_BUDGET_MS ? 0 : IDE_HOST_BUDGET_MS - used;
}

// A wait for the drive to do the host's work: want_ms, or less, so that the
// recovery reserve is still there when it ends.
static uint32_t work_ms(uint32_t want_ms) {
    uint32_t left = host_left_ms();
    if (left == UINT32_MAX) return want_ms;
    left = left > IDE_RECOVERY_RESERVE_MS ? left - IDE_RECOVERY_RESERVE_MS : 0;
    return want_ms < left ? want_ms : left;
}

// A wait that is part of recovery: want_ms, or what is left of the command.
static uint32_t recovery_ms(uint32_t want_ms) {
    uint32_t left = host_left_ms();
    return want_ms < left ? want_ms : left;
}

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

static void recovery_forget(void);

void ide_reset_drive(void) {
    recovery_forget();          // this reset supersedes any recovery still waiting
    // Force IORDY HIGH during reset: the drive holds it LOW during POST. It
    // comes back to the configured setting once the drive shows ready.
    iordy_hold();

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
}

uint8_t ide_probe_devices(void) {
#if ATABOY_SAT
    id_words.valid = false;     // a new detection: nothing is known until IDENTIFY answers
#endif
    recovery_forget();          // the probe's own reset supersedes any recovery still waiting
    iordy_hold();

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

        iordy_release();          // BSY has cleared: the drive is past its POST
        return addrs[i];
    }

    // No device found. IORDY stays ignored until a drive shows ready (review
    // L-3): one still in its POST would hold it low and hang the next access.
    return 0;  // no device found
}

// Status is read at least once, even with no time to wait (a host command
// near the end of its time still sees a drive that is ready). A drive that
// shows ready is past any power-on diagnostics, so IORDY is believed again
// from here on (iordy_release).
bool ide_wait_until_ready(uint32_t timeout_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    for (;;) {
        uint8_t st = ide_read_reg(7);
        if (!(st & 0x80) && (st & 0x40)) {              // BSY=0, DRDY=1
            iordy_release();
            return true;
        }
        if (to_ms_since_boot(get_absolute_time()) - start >= timeout_ms) return false;
        busy_wait_us_32(10);
    }
}

// INITIALIZE DEVICE PARAMETERS: how long the drive is given to take it, as in
// 0.6f3p6. Inside a host command it is only sent with all of this left
// (recovery_run; a read or write reaches chs_geometry_ok() with at least the
// recovery reserve left), so the cut below does not bite there.
#define IDE_GEOMETRY_TIMEOUT_MS 1000

bool ide_set_geometry(uint8_t heads, uint8_t spt) {
    ide_write_reg(6, dev_base | ((heads - 1) & 0x0F));
    ide_write_reg(2, spt);
    ide_write_reg(7, 0x91);
    wait_after_command();
    // Only a clean completion counts. With ABRT the drive keeps its own
    // default translation, which is not the geometry we address it with.
    // Inside a host command the wait is cut to what is left of its time.
    bool ok = ide_wait_until_ready(recovery_ms(IDE_GEOMETRY_TIMEOUT_MS)) && !(ide_read_reg(7) & 0x01);
    chs_geometry_lost = !ok;
    return ok;
}

// ---------------------------------------------------------------------------
//  IDENTIFY DEVICE (0xEC)
// ---------------------------------------------------------------------------

// IDENTIFY, once. 1: the 256 words are in buf. 0: the drive was not ready,
// or ended the command with ERR. -1: IDENTIFY was sent and gave no data in
// time; the drive may still be working on it.
static int identify_run(uint16_t *buf) {
    if (!ide_wait_until_ready(work_ms(1000))) return 0;
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
    uint32_t limit = work_ms(IDE_IDENTIFY_TIMEOUT_MS);
    for (;;) {
        if (ms_passed(start, limit)) return -1;         // identify: no data in time
        uint8_t st = sat_poll(&w);
        if (st & 0x80) { busy_wait_us_32(50); continue; }  // BSY: other bits not valid yet
        // DRQ is this IDENTIFY's: a stale one was drained above. It counts as
        // the command having started, so ERR with DRQ (an abort that offers a
        // block, from a drive that neither showed BSY nor has INTRQ wired) is
        // an error, never 256 words of IDENTIFY (tests/host, review R4).
        if (st & 0x08) w.started = true;
        if (w.started && (st & 0x01)) { if (st & 0x08) ide_drain_sector(); return 0; }  // ERR: drain stranded DRQ
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
    return 1;
}

static bool identify_once(uint16_t *buf) { return identify_run(buf) > 0; }

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
// Time (review M-1): it runs inside the host command's budget. A drive whose
// reset from an earlier command is still being waited for gets nothing (-1,
// words kept). The check only starts with time for all of it, its two ready
// checks and a full IDENTIFY, so a drive is never judged by an IDENTIFY cut
// short (-1 otherwise, words kept). An IDENTIFY that gives no data in its
// 10 s is aborted with a reset and recorded, as the SAT path does with its
// own commands, so the drive is not left working on it.
#define IDE_IDCHECK_MS (IDE_IDENTIFY_TIMEOUT_MS + 2000)
static bool recovery_gate(void);
static void record_failure(uint8_t kind, uint8_t cmd, uint8_t st,
                           uint32_t lba, uint32_t done, uint32_t count);
static void soft_reset_restore(void);

int ide_id_words_verify(void) {
    if (!id_words_held()) return 0;
    if (!recovery_gate()) return -1;
    if (work_ms(IDE_IDCHECK_MS) < IDE_IDCHECK_MS) {         // nothing sent, recorded as for a SAT command
        if (!host.recorded) record_failure(IDE_FAIL_NO_TIME, 0xEC, ide_read_reg(7), 0, 0, 1);
        return -1;
    }
    if (!ide_wait_until_ready(1000) || (ide_read_reg(7) & 0x08)) return -1;
    uint16_t buf[256];
    int got = identify_run(buf);
    if (got < 0) {
        record_failure(IDE_FAIL_TIMEOUT, 0xEC, ide_read_reg(7), 0, 0, 1);
        soft_reset_restore();
    }
    bool same = got > 0 && buf[82] == id_words.w82 &&
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
// The reset flags cover the whole host command (review L-2): the issue #13
// call again at the failing sector writes a new record, and a reset the first
// call needed must not vanish with the old one. Outside a host command (the
// host tests call these functions directly) every call counts on its own.
static void record_failure(uint8_t kind, uint8_t cmd, uint8_t st,
                           uint32_t lba, uint32_t done, uint32_t count) {
    if (!host.on) { host.resets = 0; host.hw_resets = 0; }
    last_fail.kind    = kind;
    last_fail.command = cmd;
    last_fail.status  = st;
    last_fail.error   = ide_read_reg(1);
    for (int i = 0; i < 5; i++) last_fail.tf[i] = ide_read_reg(2 + i);
    last_fail.drained = false;
    last_fail.reset   = host.resets > 0 || host.hw_resets > 0;
    last_fail.reset_failed = false;
    last_fail.hw_reset = host.hw_resets > 0;
    last_fail.pending = false;
    last_fail.lba     = lba;
    last_fail.done    = done;
    last_fail.count   = count;
    host.recorded = true;
}

#define IDE_SRST_TIMEOUT_MS 31000   // ATA allows a device up to 31 s after SRST
// RESET- low time. ATA asks for at least 25 us; the probe (ide_probe_devices)
// and ide_reset_drive() hold it for 50 ms, and so does the escalation below.
// The probe's 50 ms is the only pulse proven on the project's own drives, so
// the escalation does not use a shorter one (a test holds it to that).
#define IDE_HW_RESET_LOW_US 50000
// RECALIBRATE after the hardware reset: how long it may take (as
// ide_reset_drive allows), and how long a drive that has shown no sign of
// starting it (no BSY seen, no INTRQ) must stay idle before it is taken as
// done. ATA gives a drive 400 ns to raise BSY; the slowest drive the tests
// model takes 5.4 us. 10 ms is far past both, and short enough that a fast
// drive with no INTRQ wired costs nothing noticeable.
#define IDE_RECAL_TIMEOUT_MS 10000
#define IDE_RECAL_GRACE_MS   10

// ---------------------------------------------------------------------------
//  Recovery: a soft reset, then at most one hardware reset, across host commands
// ---------------------------------------------------------------------------
// A drive that is stuck, or did not end a command cleanly, is soft-reset
// (SRST). ATA gives it 31 s to come back. If it has not, RESET- is pulsed
// once, and no more: found on hardware 2026-09-24, a lone slave ST380011A
// stayed busy through the soft reset and then read 0xFF until a re-detect
// pulsed RESET-. After the hardware reset (31 s again) RECALIBRATE, then, in
// CHS mode, the geometry. That is one recovery.
//
// Since review M-1 a recovery may take longer than the host command that
// started it: each step waits only as long as the host command has left
// (recovery_ms). If the drive is not back by then, the host is answered now
// (the command fails, the record says the recovery is pending, and Debug E
// shows it), and the next host command carries on where this one stopped,
// before anything else, and within its own time (recovery_gate): the rest of
// the 31 s after SRST, then the hardware reset if it is due, and so on.
// Meanwhile nothing new is sent to the drive. A WRITE is never sent again by
// the firmware: a host command that finds a recovery pending fails without
// sending anything, and the host decides whether to retry.
//
// Stages. `since` is when the stage began; ATA's windows count from there.
#define REC_NONE   0
#define REC_SRST   1        // SRST sent; waiting for the drive (31 s)
#define REC_HW     2        // RESET- pulsed; waiting for the drive (31 s)
#define REC_RECAL  3        // RECALIBRATE sent after RESET-; waiting for it (10 s)
#define REC_GEO    4        // back; CHS mode: INITIALIZE DEVICE PARAMETERS still to send
static struct {
    uint8_t     stage;
    bool        selected;   // our device selected again since the reset
    uint32_t    since;
    sat_watch_t recal;      // REC_RECAL: has the RECALIBRATE visibly started?
} rec;

static uint32_t stage_left_ms(uint32_t window_ms) {
    uint32_t used = ms_now() - rec.since;
    return used >= window_ms ? 0 : window_ms - used;
}

// A probe or ide_reset_drive() (core 1, nothing mounted) pulses RESET- itself
// and so ends any recovery in progress. The record keeps what happened, and
// no longer says pending.
static void recovery_forget(void) {
    rec.stage = REC_NONE;
    last_fail.pending = false;
}

// After a soft or hardware reset: wait on device 0 where ATA says to, select
// our device again, and wait for it to be ready. At most budget_ms now, and
// never past ATA's 31 s from the reset. 1: ready. 0: not yet, but the 31 s
// are not over (the next host command carries on). -1: the 31 s are over.
static int after_reset_wait(uint32_t budget_ms) {
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
    // A slave whose master never comes out of its 31 s is selected anyway;
    // there is nothing better to do for it.)
    uint32_t window = stage_left_ms(IDE_SRST_TIMEOUT_MS);
    bool budget_ends_first = budget_ms < window;
    uint32_t limit = budget_ends_first ? budget_ms : window;
    uint32_t start = ms_now();
    if (!rec.selected) {
        if (dev_base == 0xA0) {
            while (ide_read_reg(7) & 0x80) {
                if (ms_passed(start, limit)) return budget_ends_first ? 0 : -1;
                busy_wait_us_32(10);
            }
        } else {
            uint8_t st;
            while (((st = ide_read_reg(7)) & 0x80) && st != 0xFF) {
                if (ms_passed(start, limit)) {
                    if (budget_ends_first) return 0;
                    break;                  // master stuck in reset: select the slave anyway
                }
                busy_wait_us_32(10);
            }
        }
        ide_write_reg(6, dev_base);
        busy_wait_us_32(1);                 // 400 ns before status is valid
        rec.selected = true;
    }
    uint32_t used = ms_now() - start;
    if (ide_wait_until_ready(used < limit ? limit - used : 0)) return 1;
    return budget_ends_first ? 0 : -1;
}

// Hardware reset: RESET- on the cable, as the probe uses. It resets BOTH
// devices on the cable, not only ours; a master sharing the cable with a
// slave in use loses its own settings too (nothing here uses them). The drive
// then runs its power-on diagnostics, holding IORDY low meanwhile, so IORDY
// is ignored until it shows ready (iordy_hold, review L-3). Device Control
// is written with nIEN=0, as the probe does, rather than trusting what the
// drive comes out of reset with. Runs on core 0 inside a USB callback, so it
// busy-waits; the 52 ms it takes is part of the fixed margin over the host
// command's time (ide.h). The CHS geometry is already marked lost by the
// soft reset that came first.
static void hw_reset_start(void) {
    iordy_hold();
    gpio_put(IDE_RESET, 0);
    busy_wait_us_32(IDE_HW_RESET_LOW_US);
    gpio_put(IDE_RESET, 1);
    busy_wait_us_32(2000);                  // as after SRST: 2 ms before status is valid
    ide_write_control(0x00);                // nIEN=0, as the probe does
    rec.stage = REC_HW;
    rec.selected = false;
    rec.since = ms_now();
    host.hw_resets++;
    last_fail.reset = true;
    last_fail.hw_reset = true;
}

// RECALIBRATE, as after every other hardware reset in this firmware (some
// pre-ATA drives need it before they will seek), to the device just
// selected. Review L-4: its completion is checked before INITIALIZE DEVICE
// PARAMETERS goes out, and with the SAT path's rule (sat_watch_t): a status
// with BSY clear ends it only once it has visibly started, or once the drive
// has stayed idle IDE_RECAL_GRACE_MS, which is how a drive that finished it
// before the first poll, with no INTRQ wired, looks. ERR (a drive that does
// not know the command) ends it too: the drive is idle, which is all 0x91
// needs. It can go out with little of the host command's time left; its 10 s
// count from here and carry over to the next host command (recal_wait), so
// it always gets them, and is never reset for want of time.
static void recal_start(void) {
    sat_watch_arm(&rec.recal);              // the ready wait's status read released INTRQ
    ide_write_reg(7, 0x10);
    wait_after_command();
    rec.stage = REC_RECAL;
    rec.since = ms_now();
}

// 1: done. 0: not yet, time left in its 10 s. -1: still busy after 10 s.
static int recal_wait(uint32_t budget_ms) {
    uint32_t window = stage_left_ms(IDE_RECAL_TIMEOUT_MS);
    bool budget_ends_first = budget_ms < window;
    uint32_t limit = budget_ends_first ? budget_ms : window;
    uint32_t start = ms_now();
    for (;;) {
        uint8_t st = sat_poll(&rec.recal);
        bool over = rec.recal.started || ms_passed(rec.since, IDE_RECAL_GRACE_MS);
        if (over && !(st & 0x80) && !(st & 0x08) && (st & 0x40)) return 1;
        if (ms_passed(start, limit)) return budget_ends_first ? 0 : -1;
        busy_wait_us_32(10);
    }
}

// The recovery is over, one way or the other. The record says how.
static int recovery_end(bool ok) {
    rec.stage = REC_NONE;
    last_fail.pending = false;
    last_fail.reset_failed = !ok;
    return ok ? 1 : -1;
}

// Take the recovery as far as the host command's time allows. 1: the drive
// is back (and in CHS mode has its geometry). 0: not yet; the next host
// command carries on. -1: it did not come back, or did not take the geometry
// (the recovery is over; CHS transfers refuse until the geometry is set, and
// a drive that stayed busy is refused as not ready until it answers).
static int recovery_run(void) {
    if (rec.stage == REC_SRST) {
        int r = after_reset_wait(recovery_ms(IDE_SRST_TIMEOUT_MS));
        if (r == 0) return 0;
        if (r < 0) hw_reset_start();        // once per recovery: only from REC_SRST
    }
    if (rec.stage == REC_HW) {
        int r = after_reset_wait(recovery_ms(IDE_SRST_TIMEOUT_MS));
        if (r == 0) return 0;
        if (r < 0) return recovery_end(false);
        recal_start();
    }
    if (rec.stage == REC_RECAL) {
        int r = recal_wait(recovery_ms(IDE_RECAL_TIMEOUT_MS));
        if (r == 0) return 0;
        if (r < 0) return recovery_end(false);
    }
    if (rec.stage == REC_NONE) return 1;
    // Back from the reset. CHS mode: the translation the host's sector
    // numbers assume, or CHS reads and writes refuse (chs_geometry_ok).
    if (config.use_lba_mode) return recovery_end(true);
    // Review of 0.6f3p7 (LOW): 0x91 used to go out with whatever was left,
    // none at all if the drive came back as the host command's time ran out;
    // it then "failed" and the record said the reset had failed, when nothing
    // had. It is sent only with its whole second left; otherwise the
    // recovery stays pending and the next host command sends it first.
    rec.stage = REC_GEO;
    if (recovery_ms(IDE_GEOMETRY_TIMEOUT_MS) < IDE_GEOMETRY_TIMEOUT_MS) return 0;
    return recovery_end(ide_set_geometry(config.heads, config.spt));
}

// Before anything is sent to the drive for the host: a recovery an earlier
// host command left pending is carried on first, within this command's time.
// False: the drive is not back; this command fails, nothing new is sent, and
// the failure record stays the one that started the recovery.
static bool recovery_gate(void) {
    if (rec.stage == REC_NONE) return true;
    return recovery_run() > 0;
}

// Reset a drive that is stuck or did not end a command cleanly (the failure
// is already recorded), and take the recovery as far as time allows.
static void soft_reset_restore(void) {
    ide_write_control(0x04);
    busy_wait_us_32(10);
    ide_write_control(0x00);
    chs_geometry_lost = true;               // SRST drops INITIALIZE DEVICE PARAMETERS
    busy_wait_us_32(2000);                  // ATA: 2 ms before status is valid
    rec.stage = REC_SRST;
    rec.selected = false;
    rec.since = ms_now();
    host.resets++;
    last_fail.reset = true;
    last_fail.pending = true;
    (void)recovery_run();
}

// CHS mode after a reset that could not restore the geometry: try once more
// (the drive may just have been slow). False if the drive still will not
// take it; the caller must then refuse the transfer.
static bool chs_geometry_ok(void) {
    if (config.use_lba_mode || !chs_geometry_lost) return true;
    return ide_set_geometry(config.heads, config.spt);
}

bool ide_recovery_pending(void) { return rec.stage != REC_NONE; }

// ---------------------------------------------------------------------------
//  Manual CHS with no IDENTIFY (ide.h, ide_manual_chs)
// ---------------------------------------------------------------------------
// Runs on core 1, with nothing mounted, after menus.c has waited for any USB
// command still running and checked no recovery is pending. Outside a host
// command, so no budget cuts its waits (host.on is false).
//
// What is sent, and why each:
//  - RESET-: a clean start, as a PC/AT's controller reset gives. The probe's
//    50 ms and 2 s, the only timings proven on the project's own drives.
//  - Status reads, to wait for BSY to clear. Not DRDY: some drives older
//    than ATA do not set it until they have had INITIALIZE DEVICE PARAMETERS
//    (the probe's own note). A status of FFh is a floating bus: no drive.
//  - RECALIBRATE (0x10). Deliberately kept. It is in the IBM AT task file
//    command set (the WD1003's RESTORE) that the CP3044's manual says it
//    emulates, unlike IDENTIFY, which that command set never had; a PC/AT
//    BIOS sends it to a drive of any typed-in geometry. Every other path in
//    this firmware sends it after RESET- (probe, Debug R, and the recovery's
//    hardware reset, which will send it to this drive during imaging if a
//    read ever hangs), so leaving it out here would only mean the drive
//    first sees it in the middle of a recovery. A drive that does not know
//    it answers ERR; that ends it (the drive is idle, which is all 0x91
//    needs). One that stays busy 10 s gets nothing more.
//  - INITIALIZE DEVICE PARAMETERS (0x91), the operator's heads and sectors.
// Both commands are believed ended only once the drive has visibly started
// them (sat_watch_t: BSY or INTRQ seen) or has stayed idle IDE_RECAL_GRACE_MS,
// so an ERR left by a RECALIBRATE the drive did not know is never read as
// 0x91's answer.
#define IDE_MCHS_IDP_MS 1000        // 0x91 to ready, as ide_set_geometry allows

// Wait for a command written just now to end: BSY and DRQ clear, once it has
// visibly started or the grace time is over. The last status read, and
// whether it ended within limit_ms.
static bool mchs_wait_end(sat_watch_t *w, uint32_t limit_ms, uint8_t *st) {
    uint32_t t0 = ms_now();
    for (;;) {
        *st = sat_poll(w);
        bool over = w->started || ms_passed(t0, IDE_RECAL_GRACE_MS);
        if (over && !(*st & 0x88)) return true;
        if (ms_passed(t0, limit_ms)) return false;
        busy_wait_us_32(10);
    }
}

int ide_manual_chs(uint8_t heads, uint8_t spt, uint8_t *status) {
    uint8_t st = 0;
    *status = 0;
    if (heads < 1 || heads > 16 || spt < 1) return IDE_MCHS_BAD_ARGS;
#if ATABOY_SAT
    id_words.valid = false;         // this drive is never asked who it is
#endif
    recovery_forget();              // RESET- supersedes any recovery (none is pending: menus.c)
    iordy_hold();
    gpio_put(IDE_RESET, 0);
    sleep_ms(IDE_HW_RESET_LOW_US / 1000);
    gpio_put(IDE_RESET, 1);
    chs_geometry_lost = true;       // RESET- drops any geometry the drive had
    sleep_ms(2000);                 // as the probe
    ide_write_control(0x00);        // nIEN=0

    // Device 0 is selected after a reset and is the one to wait on (ATA);
    // nothing is written to a busy master. FFh: no device 0 (for a slave,
    // go on and look at it).
    uint32_t t0 = ms_now();
    while ((st = ide_read_reg(7)) != 0xFF && (st & 0x80)) {
        if (ms_passed(t0, IDE_MCHS_READY_MS)) { *status = st; return IDE_MCHS_BUSY; }
        busy_wait_us_32(10);
    }
    ide_write_reg(6, dev_base);
    busy_wait_us_32(50);            // let selection settle, as the probe
    for (;;) {
        st = ide_read_reg(7);
        if (st == 0xFF) { *status = st; return IDE_MCHS_NO_DEVICE; }
        if (!(st & 0x80)) break;
        if (ms_passed(t0, IDE_MCHS_READY_MS)) { *status = st; return IDE_MCHS_BUSY; }
        busy_wait_us_32(10);
    }
    iordy_release();                // past its power-on diagnostics

    sat_watch_t w;
    sat_watch_arm(&w);              // the status reads above released INTRQ
    ide_write_reg(7, 0x10);
    wait_after_command();
    if (!mchs_wait_end(&w, IDE_RECAL_TIMEOUT_MS, &st)) {
        record_failure(IDE_FAIL_MANUAL_CHS, 0x10, st, 0, 0, 0);
        *status = st;
        return IDE_MCHS_RECAL;
    }

    ide_write_reg(6, dev_base | ((heads - 1) & 0x0F));
    ide_write_reg(2, spt);
    sat_watch_arm(&w);
    ide_write_reg(7, 0x91);
    wait_after_command();
    bool ended = mchs_wait_end(&w, IDE_MCHS_IDP_MS, &st);
    bool ok = ended && (st & 0x40) && !(st & 0x01);
    chs_geometry_lost = !ok;
    *status = st;
    if (!ok) {
        record_failure(IDE_FAIL_MANUAL_CHS, 0x91, st, 0, 0, 0);
        return IDE_MCHS_REFUSED;
    }
    return IDE_MCHS_OK;
}

// ---------------------------------------------------------------------------
//  The least time a READ or WRITE SECTORS is sent with
// ---------------------------------------------------------------------------
// Review of 0.6f3p7 (MEDIUM): a command used to be sent whenever any work
// time at all was left. The chunks of one READ(10) or WRITE(10) share the
// host command's time, so a later chunk, or the issue #13 call again at a
// failing sector, could go out with a few milliseconds. The drive, healthy
// and working, could not finish in that, and was soft-reset for it. Measured
// in the host tests on b073832: a 128-sector READ(10) (what imagelba sends)
// from a drive reading 120 ms a sector was reset on every try and never read,
// where 0.6f3p6 read it with no reset; a 16-sector READ(10) with one sector
// that took 14.8 s to read well reset the drive over the healthy second
// chunk, on every retry; a sector that ended in ERR after 9 s was read again
// with 6 s and the drive reset, which ERR never needs (issue #13).
//
// So a READ or WRITE SECTORS of n sectors is sent inside a host command only
// if its work time (what is left less the recovery reserve) is at least
// IDE_SECTOR_ALLOW_MS * (n + 1). Otherwise NOTHING is sent: the host command
// fails, the failure is recorded as IDE_FAIL_NO_TIME, the drive is not
// touched, and the host may retry, best with a smaller command. A reset then
// only ever follows a command that was given a fair window and did not end
// in it.
//
// Why that much. 0.6f3p6, whose resets were proven on the project's drives,
// waited up to 100000 polls, 10 us apart plus a status read, for each DRQ
// of a read or write and for the BSY at the end of a write: at least 1 s
// each, a little more with the reads (not measured on the board; 1.1 s is
// taken). A command of n sectors has n such waits (read) or n + 1 (write),
// so n + 1 of them covers both. Every chunk the USB path sends is at most 8
// sectors (CFG_TUD_MSC_EP_BUFSIZE, 4 KiB), so its work time is at least
// 9.9 s: any command that then times out has had a sector take longer than
// 0.6f3p6 allowed, and 0.6f3p6 would have reset the drive too. No reset here
// is one that 0.6f3p6 would not have made. The first chunk of a host command
// has the full 15 s, unless a recovery carried over from an earlier host
// command used some of them first.
//
// The issue #13 call again at a sector that just failed: the drive will most
// likely take as long over it again as it took the first time, so that time
// replaces the one-sector allowance for it when it is longer (send_need_ms).
// A sector that ended in ERR after 9 s is not read again with 6 s left; the
// host gets the good sectors before it, and a clean failure.
//
// What this costs (the throughput floor). A READ(10) of 128 sectors is 16
// chunks of 8. The last one is sent only if the first 120 sectors took no
// more than 15 s less 9.9 s, 5.1 s: about 42 ms a sector on average, 12 KB/s.
// A drive slower than that (only one retrying on many sectors; a healthy
// 1990s drive reads a sector in a few milliseconds at most) fails such a
// command cleanly every time, with no reset. The host should then send
// smaller commands: a READ(10) of 8 sectors or fewer is one chunk and always
// has the whole 15 s. The same holds for a slow sector early in a command:
// one that takes more than about 5 s to read well makes the chunks after it
// refuse, and the command fails cleanly; the same sector read in a command
// of 8 sectors or fewer succeeds. A host test holds the floor to these
// numbers (test_min_window_floor).
#define IDE_SECTOR_ALLOW_MS 1100

static uint32_t send_need_ms(uint32_t lba, uint32_t count) {
    uint32_t need = IDE_SECTOR_ALLOW_MS * (count + 1);
    if (host.failed && lba == host.fail_lba && host.fail_ms > IDE_SECTOR_ALLOW_MS)
        need += host.fail_ms - IDE_SECTOR_ALLOW_MS;     // issue #13: the failed sector again
    return need;
}

// Inside a host command: is there a fair window for this command, and the
// recovery reserve after it? If not, nothing is sent. That is recorded,
// unless this host command has recorded a failure of its own already: the
// issue #13 call again at the failing sector must not hide the failure that
// made it. Outside a host command there is no limit (work_ms).
static bool time_to_send(uint32_t lba, uint32_t count) {
    uint32_t need = send_need_ms(lba, count);
    if (work_ms(need) >= need) return true;
    if (!host.recorded) record_failure(IDE_FAIL_NO_TIME, 0, ide_read_reg(7), lba, 0, count);
    return false;
}

// A sector read failed after the drive had worked on it for ms: kept for the
// issue #13 call again at it, in this host command only.
static void note_failed_sector(uint32_t lba, uint32_t ms) {
    host.failed = true;
    host.fail_lba = lba;
    host.fail_ms = ms;
}

// After a command ends with ERR the drive should be idle again: BSY=0,
// DRQ=0, DRDY=1. True if it gets there within timeout_ms (or what is left of
// the host command, if that is less).
static bool idle_after_error(uint32_t timeout_ms) {
    timeout_ms = recovery_ms(timeout_ms);
    uint32_t start = to_ms_since_boot(get_absolute_time());
    for (;;) {
        uint8_t st = ide_read_reg(7);
        if (!(st & 0x80) && !(st & 0x08) && (st & 0x40)) return true;
        if (to_ms_since_boot(get_absolute_time()) - start >= timeout_ms) return false;
        busy_wait_us_32(10);
    }
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
//
// Time (review M-1): inside a host command every wait is cut to the command's
// budget (work_ms, recovery_ms), and nothing is sent while a recovery from an
// earlier command is still pending (recovery_gate) or when the command would
// not have a fair window with the recovery reserve after it (time_to_send;
// checked before and after waiting for the drive to be ready).
int32_t ide_read_sectors_partial(uint32_t lba, uint32_t count, uint8_t *buf,
                                 uint32_t *done) {
    uint32_t s = 0;
    uint8_t st = 0;
    uint8_t cmd = 0;
    if (done) *done = 0;
    if (count == 0) return -1;
    if (!recovery_gate()) return -1;
    if (!time_to_send(lba, count)) return -1;
    if (!ide_wait_until_ready(work_ms(5000))) {
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
    // Still a fair window after waiting for the drive to be ready?
    if (!time_to_send(lba, count)) return -1;

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
    uint32_t start = ms_now();                             // the whole command: IDE_CMD_TIMEOUT_MS,
    uint32_t limit = work_ms(IDE_CMD_TIMEOUT_MS);          // or what the host command has left
    uint32_t sector_start = start;                         // when the drive began on sector s

    uint16_t *wbuf = (uint16_t *)buf;

    for (s = 0; s < count; s++) {
        // Poll for DRQ — INTRQ provides early-exit if enabled, otherwise pure polling
        for (;;) {
            if (ms_passed(start, limit)) goto read_timeout;   // read: no DRQ in time
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
        sector_start = ms_now();
    }

    if (done) *done = count;
    return (int32_t)(count * 512);

read_err:
    // The drive ended the command with ERR at sector s. Sectors 0..s-1 are
    // already in buf and are good.
    note_failed_sector(lba + s, ms_now() - sector_start);
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
    // No DRQ in the time allowed: the drive may still be retrying, so abort
    // with SRST (and, if it does not come back, one hardware reset). The
    // sectors before this one are good and go to the host (issue #13).
    note_failed_sector(lba + s, ms_now() - sector_start);
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
    if (!recovery_gate()) return -1;                       // as for a read (review M-1)
    if (!time_to_send(lba, count)) return -1;
    if (!ide_wait_until_ready(work_ms(5000))) {
        record_failure(IDE_FAIL_NOT_READY, 0, ide_read_reg(7), lba, 0, count);
        return -1;
    }
    if (!chs_geometry_ok()) {
        record_failure(IDE_FAIL_NO_GEOMETRY, 0x91, ide_read_reg(7), lba, 0, count);
        return -1;
    }
    if (!time_to_send(lba, count)) return -1;              // as for a read

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
    uint32_t start = ms_now();                             // the whole command: IDE_CMD_TIMEOUT_MS,
    uint32_t limit = work_ms(IDE_CMD_TIMEOUT_MS);          // or what the host command has left

    const uint16_t *wbuf = (const uint16_t *)buf;

    bool write_ok = true;

    for (s = 0; s < count; s++) {
        // Poll for DRQ — INTRQ not asserted for first sector of PIO write per ATA spec;
        // for s > 0 it provides early-exit if enabled, otherwise pure polling
        bool got_drq = false;
        for (;;) {
            if (ms_passed(start, limit)) break;   // write: no DRQ in time
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
            if (ms_passed(start, limit)) break;   // write commit: still busy
            if (config.intrq_enabled && gpio_get(IDE_INTRQ)) ide_read_reg(7);
            st = ide_read_reg(7);
            if (st & 0x80) { busy_wait_us_32(10); continue; }   // BSY: other bits not valid
            if (st & 0x01) { write_ok = false; fail_kind = IDE_FAIL_ERR; break; }
            return (int32_t)(count * 512);
        }
    }

    // Write errors still always reset, as before. Keep the registers first.
    // For a write, 'done' is the number of sectors sent to the drive. The
    // write is never sent again by the firmware, here or by a later host
    // command's recovery; retrying is the host's decision.
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
// Timeouts are wall-clock, not loop counts, and inside the host command's
// budget like the sector paths (review M-1): waits for the command end the
// recovery reserve before the budget does, and nothing is sent while a
// recovery is pending or when there is no time left to reset after it.

#define SAT_READY_TIMEOUT_MS  5000    // same as ide_read_sectors()
#define SAT_CMD_TIMEOUT_MS    10000   // PIO: from issue to the last block; non-data: to completion
#define SAT_END_TIMEOUT_MS    1000    // after the last block, for BSY and DRQ to drop

// Abort whatever the drive is still doing for a SAT command we gave up on.
// Without this, a drive that finishes a slow read after our timeout raises
// DRQ with that sector's data, and the next command on the bus (a READ(10)
// from the host, say) can end up handed the stranded block as its own
// result. A soft reset ends the command, and the recovery selects the drive
// again and, in CHS mode, puts the geometry back; a drive that does not come
// back from the soft reset gets one hardware reset, as on the READ(10) path,
// and one not back when the host command's time is up is carried on with by
// the next host command. The host gets no registers (a reset replaces them),
// but the failure record does (review L-2): the resets, the hardware one
// above all, which resets both devices on the cable, show in Debug E.
// `lba` is the task file's LBA, for the record.
static uint32_t sat_tf_lba(const sat_taskfile_t *tf) {
    uint32_t lba = (uint32_t)tf->lba_low | ((uint32_t)tf->lba_mid << 8) | ((uint32_t)tf->lba_high << 16);
    return lba | (tf->ext ? (uint32_t)tf->hob_lba_low << 24 : (uint32_t)(tf->device & 0x0F) << 24);
}

static void sat_abort(const sat_taskfile_t *tf, uint8_t kind, uint8_t st) {
    record_failure(kind, tf->command, st, sat_tf_lba(tf), 0, tf->sectors);
    soft_reset_restore();           // on failure, CHS reads refuse until restored
}

// A SAT command is sent only with its whole SAT_CMD_TIMEOUT_MS of work time
// left (review of 0.6f3p7, MEDIUM: it used to go out with any time at all,
// after a recovery carried over from an earlier host command had used most
// of this one, and was reset if the drive did not finish in the rest). A SAT
// command is a host command of its own, so it starts with 15 s and this only
// refuses after such a recovery. Refused: nothing is sent, NO_TIME is
// recorded (unless this host command recorded a failure already), and the
// host is told NOT READY, which it retries.
static bool sat_time_to_send(const sat_taskfile_t *tf) {
    if (work_ms(SAT_CMD_TIMEOUT_MS) >= SAT_CMD_TIMEOUT_MS) return true;
    if (!host.recorded) record_failure(IDE_FAIL_NO_TIME, tf->command, ide_read_reg(7), sat_tf_lba(tf), 0, tf->sectors);
    return false;
}

// Ready to take a new command: not busy, and not holding data from an
// earlier one (that is not ours to discard; the READ(10) path's own guard
// resets it), with the command's whole time ahead of it, also after waiting
// for the drive to be ready. False means nothing was sent.
static bool sat_can_issue(const sat_taskfile_t *tf) {
    if (!recovery_gate() || !sat_time_to_send(tf)) return false;
    if (!ide_wait_until_ready(work_ms(SAT_READY_TIMEOUT_MS))) return false;
    if (ide_read_reg(7) & 0x08) return false;   // stale DRQ
    return sat_time_to_send(tf);
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
    if (!sat_can_issue(tf)) return IDE_SAT_NOT_ISSUED;

    sat_watch_t w;
    sat_watch_arm(&w);
    sat_write_taskfile(tf);

    uint16_t *wbuf = (uint16_t *)buf;
    uint32_t start = to_ms_since_boot(get_absolute_time());
    uint32_t limit = work_ms(SAT_CMD_TIMEOUT_MS);

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
            if (to_ms_since_boot(get_absolute_time()) - start >= limit) {
                sat_abort(tf, IDE_FAIL_TIMEOUT, st);
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
    uint32_t end_limit = recovery_ms(SAT_END_TIMEOUT_MS);   // leaves the abort most of the reserve
    for (;;) {
        uint8_t st = ide_read_reg(7);
        if (!(st & 0x80)) {
            if (st & 0x21) { sat_read_outputs(tf, st, regs); return IDE_SAT_ATA_ERROR; }
            if (!(st & 0x08)) return IDE_SAT_OK;
        }
        if (to_ms_since_boot(get_absolute_time()) - end_start >= end_limit) {
            bool busy = (st & 0x80) != 0;
            sat_abort(tf, busy ? IDE_FAIL_TIMEOUT : IDE_FAIL_BAD_END, st);
            return busy ? IDE_SAT_TIMEOUT : IDE_SAT_BAD_END;
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
    if (!sat_can_issue(tf)) return IDE_SAT_NOT_ISSUED;

    sat_watch_t w;
    sat_watch_arm(&w);
    sat_write_taskfile(tf);

    uint32_t start = to_ms_since_boot(get_absolute_time());
    uint32_t limit = work_ms(SAT_CMD_TIMEOUT_MS);
    for (;;) {
        uint8_t st = sat_poll(&w);                              // also clears INTRQ
        // BSY clear ends the command only once it has started (sat_watch_t);
        // before that it is the status from before the command.
        if (!(st & 0x80) && w.started) {                        // other bits only valid with BSY=0
            sat_read_outputs(tf, st, regs);
            if (st & 0x08) {                                    // DRQ on a non-data command
                sat_abort(tf, IDE_FAIL_BAD_END, st);
                return IDE_SAT_BAD_END;
            }
            return (st & 0x21) ? IDE_SAT_ATA_ERROR : IDE_SAT_OK;   // ERR or DF
        }
        if (to_ms_since_boot(get_absolute_time()) - start >= limit) {
            sat_abort(tf, IDE_FAIL_TIMEOUT, st);
            return IDE_SAT_TIMEOUT;
        }
        busy_wait_us_32(10);
    }
}
#endif
