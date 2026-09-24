// Host tests for the READ(10) path: the real ide.c and usb.c, compiled for a
// PC, running against the simulated drive in sim.hpp. The READ(10) driver
// below follows TinyUSB 0.18's proc_read10_cmd (pico-sdk 2.2.0): it calls
// tud_msc_read10_cb with at most CFG_TUD_MSC_EP_BUFSIZE bytes at a time,
// sends whatever the callback returns, calls again for the rest, and fails
// the command (CSW failed, remaining data not sent) on a negative return.
// Once the CSW is out, passed or failed, it calls the complete callback, as
// TinyUSB does (review M-1: usb.c finds command boundaries by them).
//
// The host's timeout (review M-1). Every command sent through the helpers
// below (READ(10), WRITE(10), ATA PASS-THROUGH) is timed on the mock clock,
// each callback and the command as a whole, and a command that keeps core 0
// in its callbacks for longer than IDE_HOST_BUDGET_MS plus HOST_MARGIN_MS
// fails the test: a real host would have given up on it and reset the
// device under the callback. So every test that goes through the helpers
// also tests the budget.
//
// Build and run: see run.sh. Exit status is the number of failed checks.
#include "mock_pico.h"
#include "sim.hpp"

#include "ide.c"
#include "usb.c"
#if ATABOY_SAT
#include "sat_policy.c"
#include "sat.c"
#endif

#include <cstdio>
#include <vector>
#include <string>

uint64_t mock_now_ns = 0;
uint32_t mock_gpio_out = 0;
MockSio mock_sio;
SimDrive sim;
config_t config;
volatile bool is_mounted = false;
volatile bool media_changed_waiting = false;

static uint8_t last_key, last_asc, last_ascq;     // TinyUSB's stored sense
bool tud_msc_set_sense(uint8_t, uint8_t key, uint8_t asc, uint8_t ascq) {
    last_key = key; last_asc = asc; last_ascq = ascq; return true;
}

static int failures = 0, checks = 0;
static std::string current;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; \
    std::printf("FAIL [%s] %s:%d: %s -- ", current.c_str(), __FILE__, __LINE__, #cond); \
    std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

// The fixed margin over the budget that ide.h allows: the 52 ms of a RESET-
// pulse started near the end, polls in flight, and register reads.
static const uint64_t HOST_MARGIN_MS = 100;
static const uint64_t HOST_LIMIT_NS = (IDE_HOST_BUDGET_MS + HOST_MARGIN_MS) * 1000000ull;
// Worst times seen since the last reset_worst(): one callback, one command.
static uint64_t worst_cb_ns = 0, worst_cmd_ns = 0;
static void reset_worst() { worst_cb_ns = 0; worst_cmd_ns = 0; }
static void note_cmd(const char *what, uint64_t cmd_ns) {
    if (cmd_ns > worst_cmd_ns) worst_cmd_ns = cmd_ns;
    CHECK(cmd_ns <= HOST_LIMIT_NS, "%s kept core 0 in its callbacks for %llu ms: the host has given up",
          what, (unsigned long long)(cmd_ns / 1000000));
}
static void note_cb(uint64_t ns) { if (ns > worst_cb_ns) worst_cb_ns = ns; }

// ---- host side of READ(10) -------------------------------------------------
struct HostRead {
    bool ok = false;
    std::vector<uint8_t> data;   // bytes the host actually received
    int calls = 0;
    uint8_t key = 0, asc = 0;    // sense the firmware set (TinyUSB overwrites it today)
};

static void host_read10_body(HostRead &h, uint32_t lba, uint32_t nblocks) {
    uint32_t total = nblocks * 512, xferred = 0;
    static uint8_t epbuf[CFG_TUD_MSC_EP_BUFSIZE];
    last_key = last_asc = 0;
    int busy = 0;
    while (xferred < total) {
        uint32_t cur = lba + xferred / 512, off = xferred % 512;
        uint32_t n = total - xferred;
        if (n > CFG_TUD_MSC_EP_BUFSIZE) n = CFG_TUD_MSC_EP_BUFSIZE;
        memset(epbuf, 0xA5, sizeof(epbuf));        // poison: anything not written shows up
        uint64_t t = mock_now_ns;
        int32_t r = tud_msc_read10_cb(0, cur, off, epbuf, n);
        note_cb(mock_now_ns - t);
        h.calls++;
        if (r < 0) { h.key = last_key; h.asc = last_asc; return; }
        if (r == 0) { if (++busy > 100) return; continue; }
        if ((uint32_t)r > n) { std::printf("callback returned more than asked\n"); return; }
        h.data.insert(h.data.end(), epbuf, epbuf + r);
        xferred += r;
    }
    h.ok = true;
}

// A host asks for sense after a failed command (usb-storage and Windows both
// do), and TinyUSB will not run the next command's callback while sense is
// pending. Defined with the SAT helpers below.
static std::vector<uint8_t> request_sense(uint8_t alloc);

static HostRead host_read10(uint32_t lba, uint32_t nblocks) {
    HostRead h;
    uint64_t t0 = mock_now_ns;
    host_read10_body(h, lba, nblocks);
    note_cmd("READ(10)", mock_now_ns - t0);
    tud_msc_read10_complete_cb(0);                  // the CSW has gone out
    if (!h.ok) request_sense(18);
    return h;
}

// WRITE(10), as TinyUSB 0.18 hands it over: one callback per chunk of at
// most CFG_TUD_MSC_EP_BUFSIZE, the command failed at a negative return, and
// the complete callback once the CSW is out. Returns the bytes accepted, or
// -1 if the command failed.
static int32_t host_write10(uint32_t lba, const uint8_t *data, uint32_t nblocks) {
    static uint8_t epbuf[CFG_TUD_MSC_EP_BUFSIZE];
    uint32_t total = nblocks * 512, xferred = 0;
    uint64_t t0 = mock_now_ns;
    int32_t result = (int32_t)total;
    while (xferred < total) {
        uint32_t n = total - xferred;
        if (n > CFG_TUD_MSC_EP_BUFSIZE) n = CFG_TUD_MSC_EP_BUFSIZE;
        memcpy(epbuf, data + xferred, n);
        uint64_t t = mock_now_ns;
        int32_t r = tud_msc_write10_cb(0, lba + xferred / 512, 0, epbuf, n);
        note_cb(mock_now_ns - t);
        if (r < 0) { result = -1; break; }
        xferred += (uint32_t)r;
    }
    note_cmd("WRITE(10)", mock_now_ns - t0);
    tud_msc_write10_complete_cb(0);
    if (result < 0) request_sense(18);
    return result;
}

// Every byte the host received must be the medium's real content.
static bool matches_medium(const HostRead &h, uint32_t lba) {
    for (size_t i = 0; i < h.data.size(); i++)
        if (h.data[i] != sim.byte_at(lba + (uint32_t)(i / 512), (int)(i % 512))) return false;
    return true;
}
static bool contains_flawed(const HostRead &h) {
    for (size_t i = 0; i + 1 < h.data.size(); i += 2)
        if (h.data[i] == 0xAD && h.data[i + 1] == 0xDE) return true;
    return false;
}

// ---- setup -----------------------------------------------------------------
enum Mode { LBA, CHS };
static void setup(const char *name, Mode m, uint32_t medium_sectors = 0) {
    current = name;
    sim = SimDrive();
    mock_now_ns = 0;
    // A fresh start for the firmware's own state too: no recovery pending,
    // no host command open (tud_mount_cb, as after a USB reset), IORDY as
    // configured, no failure on record.
    memset(&rec, 0, sizeof rec);
    memset(&host, 0, sizeof host);
    memset(&last_fail, 0, sizeof last_fail);
    iordy_held = false;
    chs_geometry_lost = false;
    manual_chs_active = false;
    tud_mount_cb();
    reset_worst();
    mock_gpio_out = (1u << 24) | (1u << 25);      // CS0/CS1 idle high
    memset(&config, 0, sizeof(config));
    config.dev_base = 0xA0;
    config.use_lba_mode = (m == LBA);
    config.cyls = 980; config.heads = 10; config.spt = 17;
    config.lba_sectors = medium_sectors ? medium_sectors : sim.nsect;
    if (m == CHS && medium_sectors) config.cyls = (uint16_t)(medium_sectors / 170);
    ide_select_device(0xA0);
    ide_hw_init();
    uint16_t id[256];
    ide_identify(id);                               // as detection does: the SAT gate needs it
    if (m == CHS) ide_set_geometry(config.heads, config.spt);
    is_mounted = true;
    media_changed_waiting = false;
    last_key = last_asc = last_ascq = 0;           // no sense pending in TinyUSB
    sim.srst = 0; sim.init_params = 0; sim.violations = 0; sim.attempts.clear();
    sim.data_reads = 0; sim.id_data_reads = 0;      // the IDENTIFY above is not the test's
}

// ---- tests -----------------------------------------------------------------
static void test_clean_reads(Mode m) {
    setup(m == LBA ? "clean reads, LBA" : "clean reads, CHS", m);
    const uint32_t starts[] = {0, 1, 7, 8, 15, 1000, 33640, 166600 - 40};
    const uint32_t lens[] = {1, 2, 7, 8, 9, 16, 17, 40};
    for (uint32_t s : starts) for (uint32_t n : lens) {
        HostRead h = host_read10(s, n);
        CHECK(h.ok, "lba %u n %u", s, n);
        CHECK(h.data.size() == n * 512u, "lba %u n %u got %zu", s, n, h.data.size());
        CHECK(matches_medium(h, s), "lba %u n %u", s, n);
    }
    CHECK(sim.srst == 0, "srst %d", sim.srst);
    CHECK(sim.violations == 0, "violations %d", sim.violations);
}

// One bad sector at every position of a 16-sector read.
static void test_bad_at_each_position(Mode m, BadMode bm, const char *name) {
    setup(name, m);
    for (uint32_t k = 0; k < 16; k++) {
        uint32_t base = 2000 + k * 100, badlba = base + k;
        sim.bad.clear(); sim.bad[badlba] = {bm, 0};
        sim.attempts.clear();
        int srst0 = sim.srst;
        HostRead h = host_read10(base, 16);
        CHECK(!h.ok, "k=%u: read must fail", k);
        CHECK(h.data.size() == k * 512u, "k=%u: got %zu bytes, want %u", k, h.data.size(), k * 512);
        CHECK(matches_medium(h, base), "k=%u: delivered bytes are not the medium", k);
        CHECK(!contains_flawed(h), "k=%u: flawed data delivered", k);
        CHECK(h.key == SCSI_SENSE_MEDIUM_ERROR && h.asc == 0x11, "k=%u sense %u/%02x", k, h.key, h.asc);
        // ERR ends the command; no soft reset is needed or wanted.
        CHECK(sim.srst == srst0, "k=%u: %d soft resets", k, sim.srst - srst0);
        // The bad sector is tried once if it starts a TinyUSB chunk, twice if
        // it falls inside one (the chunk that stops there, then the retry).
        int want = (k % 8 == 0) ? 1 : 2;
        CHECK(sim.attempts[badlba] == want, "k=%u attempts %d want %d", k, sim.attempts[badlba], want);
        // Good sectors after the bad one must still read one at a time.
        for (uint32_t j = 0; j < 16; j++) {
            HostRead s1 = host_read10(base + j, 1);
            if (base + j == badlba) CHECK(!s1.ok && s1.data.empty(), "k=%u: bad sector single read", k);
            else CHECK(s1.ok && matches_medium(s1, base + j), "k=%u: good sector %u single read", k, j);
        }
    }
    CHECK(sim.violations == 0, "violations %d", sim.violations);
}

// The case from issue #13 and the hardware test plan: 16 sectors from 33640,
// bad sector 33648 (a WD Caviar 280 sector that failed 23 of 23 reads).
static void test_issue13_case(Mode m) {
    setup(m == LBA ? "issue 13 case 33640+16, LBA" : "issue 13 case 33640+16, CHS", m);
    sim.bad[33648] = {BAD_ERR_DRQ, 0};
    HostRead h = host_read10(33640, 16);
    CHECK(!h.ok, "must fail");
    CHECK(h.data.size() == 8 * 512, "got %zu", h.data.size());
    CHECK(matches_medium(h, 33640), "prefix is not the medium");
    CHECK(sim.srst == 0, "srst %d", sim.srst);
    // and a read that does not include it is unchanged
    HostRead g = host_read10(33649, 16);
    CHECK(g.ok && g.data.size() == 16 * 512 && matches_medium(g, 33649), "read after the bad sector");
    HostRead g2 = host_read10(33624, 16);
    CHECK(g2.ok && matches_medium(g2, 33624), "read before the bad sector");
    CHECK(sim.violations == 0, "violations %d", sim.violations);
}

// Drive never finishes the sector: timeout, soft reset, geometry restored.
static void test_hang(Mode m) {
    setup(m == LBA ? "hang -> timeout, LBA" : "hang -> timeout, CHS", m);
    sim.bad[5003] = {BAD_HANG, 0};
    int ip0 = sim.init_params;
    HostRead h = host_read10(5000, 8);
    CHECK(!h.ok, "must fail");
    CHECK(h.data.size() == 3 * 512, "got %zu", h.data.size());
    CHECK(matches_medium(h, 5000), "prefix");
    CHECK(sim.srst >= 1, "a hung drive must be reset");
    if (m == CHS) CHECK(sim.init_params > ip0, "CHS geometry must be restored after SRST");
    ide_fail_t f; ide_last_failure(&f);
    CHECK(f.kind == IDE_FAIL_TIMEOUT && f.reset, "kind %u reset %d", f.kind, f.reset);
    sim.bad.clear();
    HostRead g = host_read10(5000, 16);
    CHECK(g.ok && matches_medium(g, 5000), "drive usable after the reset");
}

// ERR + DRQ that does not go away after one drained sector: must reset.
static void test_err_drq_stuck() {
    setup("ERR with DRQ that stays set", LBA);
    sim.bad[7002] = {BAD_ERR_DRQ_MORE, 0};
    HostRead h = host_read10(7000, 4);
    CHECK(!h.ok && h.data.size() == 2 * 512 && matches_medium(h, 7000), "prefix only");
    CHECK(!contains_flawed(h), "flawed data delivered");
    CHECK(sim.srst >= 1, "must reset when DRQ stays after the drain");
    sim.bad.clear();
    HostRead g = host_read10(7000, 4);
    CHECK(g.ok && matches_medium(g, 7000), "usable afterwards");
    CHECK(sim.violations == 0, "violations %d", sim.violations);

    // The reset must happen inside the failing call, not be left to the next
    // command. With ATABOY_SAT the next read's stale DRQ guard would reset
    // too, which hides a missing reset from the checks above and overwrites
    // the record of the real failure.
    setup("ERR with DRQ that stays set, one call", LBA);
    sim.bad[7002] = {BAD_ERR_DRQ_MORE, 0};
    uint8_t buf[4 * 512];
    uint32_t done = 0;
    int32_t r = ide_read_sectors_partial(7000, 4, buf, &done);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(r < 0 && done == 2, "r %d done %u", r, done);
    CHECK(sim.srst >= 1, "the failing call itself must reset");
    CHECK(f.kind == IDE_FAIL_ERR && f.reset && f.drained, "kind %u reset %d drained %d", f.kind, f.reset, f.drained);
    CHECK(!(sim.read_status(mock_now_ns + 1000000000ull) & 0x08), "DRQ still set after the call");
}

// Status bits other than BSY are undefined while BSY=1. This drive shows ERR
// then; reads must still succeed.
static void test_garbage_while_busy() {
    setup("ERR bit set while BSY", LBA);
    sim.garbage_while_busy = true;
    HostRead h = host_read10(100, 40);
    CHECK(h.ok && h.data.size() == 40 * 512 && matches_medium(h, 100), "ok %d size %zu", h.ok, h.data.size());
    CHECK(sim.srst == 0, "srst %d", sim.srst);
}

// A failed command leaves ERR in status. With no reset afterwards, the next
// command must not read that stale status in the first 400 ns.
static void test_stale_status_after_error() {
    setup("next read right after an ERR", LBA);
    sim.bad[300] = {BAD_ERR, 0};
    HostRead h = host_read10(300, 1);
    CHECK(!h.ok, "bad sector fails");
    HostRead g = host_read10(301, 8);
    CHECK(g.ok && matches_medium(g, 301), "good read straight after a failure");
}

// Media bounds must hold as before (strict: never zeros past the end).
static void test_media_bounds() {
    const uint32_t max = 1000;
    setup("strict media bounds", LBA, max);
    HostRead a = host_read10(max - 16, 16);
    CHECK(a.ok && matches_medium(a, max - 16), "read ending at max");
    HostRead b = host_read10(max - 4, 16);
    CHECK(!b.ok && b.data.size() == 4 * 512 && matches_medium(b, max - 4), "straddle: %zu", b.data.size());
    CHECK(b.key == SCSI_SENSE_ILLEGAL_REQUEST && b.asc == 0x21, "straddle sense %u/%02x", b.key, b.asc);
    HostRead c = host_read10(max, 8);
    CHECK(!c.ok && c.data.empty() && c.key == SCSI_SENSE_ILLEGAL_REQUEST && c.asc == 0x21, "beyond");
    HostRead d = host_read10(0xFFFFFFF0u, 8);
    CHECK(!d.ok && d.data.empty(), "far beyond");
    // a bad sector just inside the end: prefix, then medium error (not 5/21)
    sim.bad[max - 2] = {BAD_ERR, 0};
    HostRead e = host_read10(max - 8, 16);
    CHECK(!e.ok && e.data.size() == 6 * 512 && matches_medium(e, max - 8), "bad near end: %zu", e.data.size());
    CHECK(e.key == SCSI_SENSE_MEDIUM_ERROR, "bad near end sense %u", e.key);
}

// The failure record must hold what the drive reported, not what a reset left.
static void test_failure_record(Mode m) {
    setup(m == LBA ? "failure record, LBA" : "failure record, CHS", m);
    const uint32_t badlba = 61627;
    sim.bad[badlba] = {BAD_ERR_DRQ, 0};
    HostRead h = host_read10(badlba - 3, 8);
    CHECK(!h.ok && h.data.size() == 3 * 512, "prefix %zu", h.data.size());
    ide_fail_t f; ide_last_failure(&f);
    CHECK(f.kind == IDE_FAIL_ERR, "kind %u", f.kind);
    CHECK(f.error == 0x40, "error reg %02x (want UNC 40)", f.error);
    CHECK((f.status & 0x81) == 0x01, "status %02x", f.status);
    CHECK(f.lba == badlba, "lba %u", f.lba);
    CHECK(f.drained && !f.reset, "drained %d reset %d", f.drained, f.reset);
    CHECK(f.command == 0x20, "command %02x", f.command);
    if (m == LBA) {
        uint32_t tl = f.tf[1] | (f.tf[2] << 8) | (f.tf[3] << 16) | ((f.tf[4] & 0x0F) << 24);
        CHECK(tl == badlba, "drive-reported LBA %u", tl);
    } else {
        uint32_t cyl = f.tf[2] | (f.tf[3] << 8), head = f.tf[4] & 0x0F, sec = f.tf[1];
        CHECK((cyl * 10 + head) * 17 + sec - 1 == badlba, "drive-reported CHS %u/%u/%u", cyl, head, sec);
    }
}

// A marginal sector that reads on a later attempt.
static void test_marginal() {
    setup("marginal sector", LBA);
    sim.bad[8004] = {BAD_MARGINAL, 1};     // fails once, then reads
    HostRead h = host_read10(8000, 8);     // inside the chunk: fails, then the retry reads it
    CHECK(h.ok && matches_medium(h, 8000), "ok %d size %zu", h.ok, h.data.size());
    CHECK(sim.attempts[8004] == 2, "attempts %d", sim.attempts[8004]);
    sim.bad[8008] = {BAD_MARGINAL, 1};     // at a chunk start: the host sees the failure
    HostRead a = host_read10(8008, 8);
    CHECK(!a.ok && a.data.empty(), "first try fails");
    HostRead b = host_read10(8008, 8);
    CHECK(b.ok && matches_medium(b, 8008), "second try reads");
}

// Two bad sectors in one request: the prefix stops at the first.
static void test_two_bad() {
    setup("two bad sectors", LBA);
    sim.bad[9003] = {BAD_ERR, 0};
    sim.bad[9010] = {BAD_ERR, 0};
    HostRead h = host_read10(9000, 16);
    CHECK(!h.ok && h.data.size() == 3 * 512 && matches_medium(h, 9000), "got %zu", h.data.size());
    HostRead g = host_read10(9004, 6);
    CHECK(g.ok && matches_medium(g, 9004), "between the two");
}

// Write path still works, and a write error still resets (unchanged policy).
static void test_writes() {
    setup("writes", LBA);
    config.drive_write_protected = false;
    std::vector<uint8_t> buf(4096);
    for (size_t i = 0; i < buf.size(); i++) buf[i] = (uint8_t)(i * 7 + 3);
    int32_t r = host_write10(400, buf.data(), 8);
    CHECK(r == 4096, "write returned %d", r);
    HostRead h = host_read10(400, 8);
    CHECK(h.ok && memcmp(h.data.data(), buf.data(), 4096) == 0, "read back");
    CHECK(sim.srst == 0 && sim.violations == 0, "srst %d violations %d", sim.srst, sim.violations);
}

static void test_not_ready() {
    setup("drive not ready", LBA);
    sim.phase = SimDrive::HUNG; sim.status = 0x80;
    uint8_t buf[1024]; memset(buf, 0x5A, sizeof(buf));
    uint32_t done = 99;
    int32_t r = ide_read_sectors_partial(10, 2, buf, &done);
    CHECK(r < 0 && done == 0, "r %d done %u", r, done);
    bool untouched = true;
    for (uint8_t b : buf) if (b != 0x5A) untouched = false;
    CHECK(untouched, "buffer written when nothing was read");
    ide_fail_t f; ide_last_failure(&f);
    CHECK(f.kind == IDE_FAIL_NOT_READY, "kind %u", f.kind);
}

// A drive still offering data from an earlier command (a SAT command, or a
// debug seek test, that ended after the bridge gave up on it). The next READ
// must not be issued over it: fail, record STALE_DRQ, reset, and never hand
// the stale block to the host.
static void test_stale_drq(Mode m) {
    setup(m == LBA ? "stale DRQ before a read, LBA" : "stale DRQ before a read, CHS", m);
    for (int i = 0; i < 256; i++) sim.xfer[i] = 0xDEAD;
    sim.widx = 0; sim.phase = SimDrive::DRQ_IN; sim.status = 0x58; sim.left = 1;
    int ip0 = sim.init_params;
    HostRead h = host_read10(300, 4);
    CHECK(!h.ok && h.data.empty(), "ok %d size %zu", h.ok, h.data.size());
    CHECK(!contains_flawed(h), "stale block delivered");
    CHECK(h.key == SCSI_SENSE_MEDIUM_ERROR && h.asc == 0x11, "sense %u/%02x", h.key, h.asc);
    CHECK(sim.srst >= 1, "stale DRQ must be cleared with a reset");
    if (m == CHS) CHECK(sim.init_params > ip0, "CHS geometry must be restored after SRST");
    ide_fail_t f; ide_last_failure(&f);
    CHECK(f.kind == IDE_FAIL_STALE_DRQ && f.reset && f.done == 0, "kind %u reset %d done %u", f.kind, f.reset, f.done);
    CHECK((f.status & 0x08) != 0, "status %02x", f.status);
    CHECK(sim.violations == 0, "violations %d", sim.violations);
    HostRead g = host_read10(300, 4);
    CHECK(g.ok && matches_medium(g, 300), "usable afterwards");
}

// ---- review findings: soft reset, CHS geometry, slave, host offsets ---------

// CHS with a geometry that differs from the drive's own default, so a lost
// INITIALIZE DEVICE PARAMETERS reads the wrong sector with good status.
static void chs_user_geometry() {
    config.heads = 5; config.spt = 34; config.cyls = 980;    // native is 10 x 17
    ide_set_geometry(config.heads, config.spt);
    sim.init_params = 0;
}

// Every byte the host got is the medium's, whether the read passed or failed.
static bool never_wrong(const HostRead &h, uint32_t lba) { return matches_medium(h, lba); }

// A soft reset that takes 3 s (ATA allows 31): the geometry must still be
// restored before any CHS read, and nothing may be written during the reset.
static void test_slow_reset_chs() {
    setup("slow soft reset, CHS user geometry", CHS);
    chs_user_geometry();
    sim.t_reset = 3000000000ull;
    sim.bad[5003] = {BAD_HANG, 0};
    HostRead h = host_read10(5000, 8);
    CHECK(!h.ok && h.data.size() == 3 * 512, "ok %d size %zu", h.ok, h.data.size());
    CHECK(never_wrong(h, 5000), "wrong sectors handed to the host");
    ide_fail_t f; ide_last_failure(&f);
    CHECK(f.kind == IDE_FAIL_TIMEOUT && f.reset && !f.reset_failed, "kind %u reset %d failed %d", f.kind, f.reset, f.reset_failed);
    CHECK(!f.hw_reset && sim.hw_resets == 0, "a 3 s soft reset is within ATA's 31 s: no hardware reset (%d)", sim.hw_resets);
    CHECK(sim.geo_valid && sim.heads == 5 && sim.spt == 34, "geometry %d %u x %u", sim.geo_valid, sim.heads, sim.spt);
    sim.bad.clear();
    HostRead g = host_read10(1234, 8);
    CHECK(g.ok && matches_medium(g, 1234), "read after the reset");
    CHECK(sim.violations == 0 && sim.reset_writes == 0, "violations %d reset writes %d", sim.violations, sim.reset_writes);
}

// A reset that outlasts the 31 s allowance: reads must refuse (never read
// through the default translation), and work again once the drive is back.
// Since 0.6f3p7 a soft reset that fails is followed by one hardware reset,
// so that has to be slow too for the drive to stay unready.
static void test_reset_never_ready_chs() {
    setup("soft reset never finishes in time, CHS", CHS);
    chs_user_geometry();
    sim.t_reset = 45000000000ull;                  // 45 s
    sim.t_hw_reset = 60000000000ull;               // 60 s
    sim.bad[5003] = {BAD_HANG, 0};
    // One call first, so the record read back is this failure's (the host
    // retry that follows fails "not ready" and records that instead).
    uint8_t buf[8 * 512]; uint32_t done = 0;
    int32_t r = ide_read_sectors_partial(5000, 8, buf, &done);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(r < 0 && done == 3, "r %d done %u", r, done);
    CHECK(f.kind == IDE_FAIL_TIMEOUT && f.reset && f.reset_failed, "kind %u reset %d failed %d", f.kind, f.reset, f.reset_failed);
    CHECK(f.hw_reset && sim.hw_resets == 1, "hardware reset %d, %d of them", f.hw_reset, sim.hw_resets);
    HostRead h = host_read10(5000, 8);             // drive still in reset
    CHECK(never_wrong(h, 5000) && !h.ok, "ok %d size %zu", h.ok, h.data.size());
    sim.bad.clear();
    HostRead g = host_read10(1234, 8);             // drive still in reset
    CHECK(!g.ok && g.data.empty(), "read while the drive is still resetting: ok %d size %zu", g.ok, g.data.size());
    mock_now_ns += 60000000000ull;                 // drive is back
    HostRead g2 = host_read10(1234, 8);
    CHECK(g2.ok && matches_medium(g2, 1234), "read once the drive is back (geometry restored on the way)");
    CHECK(sim.geo_valid && sim.heads == 5 && sim.spt == 34, "geometry %d %u x %u", sim.geo_valid, sim.heads, sim.spt);
    CHECK(sim.violations == 0 && sim.reset_writes == 0, "violations %d reset writes %d", sim.violations, sim.reset_writes);
}

// The drive comes back but refuses INITIALIZE DEVICE PARAMETERS: CHS reads
// AND writes must refuse, with the reason on record.
static void test_geometry_rejected_chs() {
    setup("drive refuses the geometry after a reset, CHS", CHS);
    chs_user_geometry();
    sim.bad[5003] = {BAD_HANG, 0};
    sim.reject_idp = true;
    HostRead h = host_read10(5000, 8);
    CHECK(never_wrong(h, 5000) && !h.ok, "ok %d size %zu", h.ok, h.data.size());
    sim.bad.clear();
    HostRead g = host_read10(1234, 8);
    CHECK(!g.ok && g.data.empty(), "read with no geometry: ok %d size %zu", g.ok, g.data.size());
    ide_fail_t f; ide_last_failure(&f);
    CHECK(f.kind == IDE_FAIL_NO_GEOMETRY && f.command == 0x91, "kind %u cmd %02X", f.kind, f.command);
    uint8_t w[512]; memset(w, 0x33, sizeof w);
    int32_t r = ide_write_sectors(1234, 1, w);
    CHECK(r < 0 && sim.written.empty(), "write with no geometry: r %d written %zu", r, sim.written.size());
    sim.reject_idp = false;                        // drive takes it now
    HostRead g2 = host_read10(1234, 8);
    CHECK(g2.ok && matches_medium(g2, 1234), "recovers once the drive takes the geometry");
    CHECK(sim.violations == 0, "violations %d", sim.violations);
}

// Slave drive: SRST selects device 0, so the reset must select ours again.
static void test_slave_after_reset(Mode m) {
    setup(m == LBA ? "slave after a soft reset, LBA" : "slave after a soft reset, CHS", m);
    sim.slave = true;
    ide_select_device(0xB0);
    if (m == CHS) ide_set_geometry(config.heads, config.spt);
    else { ide_write_reg(6, 0xB0); busy_wait_us_32(1); }
    sim.violations = 0;
    HostRead g0 = host_read10(100, 8);
    CHECK(g0.ok && matches_medium(g0, 100), "baseline slave read");
    sim.bad[5003] = {BAD_HANG, 0};
    HostRead h = host_read10(5000, 8);
    CHECK(never_wrong(h, 5000) && !h.ok, "ok %d size %zu", h.ok, h.data.size());
    ide_fail_t f; ide_last_failure(&f);
    CHECK(f.reset && !f.reset_failed, "reset %d failed %d", f.reset, f.reset_failed);
    sim.bad.clear();
    HostRead g = host_read10(100, 8);
    CHECK(g.ok && matches_medium(g, 100), "slave usable after the reset");
    CHECK(sim.violations == 0, "violations %d", sim.violations);

    // Found on hardware (ST380011A, slave-only, 2026-09-23): no master on the
    // cable, so device 0 reads 0xFF (bus floats high). The reset must not wait
    // on it, and must leave the slave selected, also when the slave is slow.
    sim.t_reset = 3000000000ull;
    sim.bad[5003] = {BAD_HANG, 0};
    uint64_t t0 = mock_now_ns;
    uint8_t buf[8 * 512]; uint32_t done = 0;
    int32_t r = ide_read_sectors_partial(5000, 8, buf, &done);
    ide_fail_t f2; ide_last_failure(&f2);
    CHECK(r < 0 && f2.reset && !f2.reset_failed, "slow slave reset: r %d reset %d failed %d", r, f2.reset, f2.reset_failed);
    // The hang itself takes IDE_CMD_TIMEOUT_MS to give up on; waiting on the
    // absent master would add up to 31 s on top of the slave's own 3 s.
    CHECK(mock_now_ns - t0 < (IDE_CMD_TIMEOUT_MS + 20000) * 1000000ull, "reset took %llu ms: waited on the absent master",
          (unsigned long long)((mock_now_ns - t0) / 1000000));
    CHECK(!f2.hw_reset && sim.hw_resets == 0, "a slave that comes back from SRST got a hardware reset");
    CHECK(((sim.reg[6] >> 4) & 1) == 1, "slave not selected after the reset (DH %02X)", sim.reg[6]);
    sim.bad.clear();
    HostRead g2 = host_read10(100, 8);
    CHECK(g2.ok && matches_medium(g2, 100), "slave usable after a slow reset");
    CHECK(sim.violations == 0, "violations %d", sim.violations);
}

// TinyUSB takes the block size from the host's CBW. A non-zero offset or a
// length that is not whole sectors is refused, read and write, with nothing
// copied, read or written.
static void test_host_offsets() {
    setup("malformed CBW offsets and lengths", LBA);
    config.drive_write_protected = false;
    static uint8_t b[CFG_TUD_MSC_EP_BUFSIZE];
    const uint32_t offs[] = {1, 256, 511, 512, 4096};
    for (uint32_t off : offs) {
        memset(b, 0xA5, sizeof b);
        int cmds = sim.commands;
        int32_t r = tud_msc_read10_cb(0, 1000, off, b, 4096);
        bool untouched = true; for (uint8_t x : b) if (x != 0xA5) untouched = false;
        CHECK(r < 0 && untouched && sim.commands == cmds, "read offset %u: r %d untouched %d", off, r, untouched);
        r = tud_msc_write10_cb(0, 1000, off, b, 4096);
        CHECK(r < 0 && sim.written.empty() && sim.commands == cmds, "write offset %u: r %d", off, r);
    }
    const uint32_t lens[] = {1, 500, 1000, 4095};
    for (uint32_t len : lens) {
        int cmds = sim.commands;
        int32_t r = tud_msc_read10_cb(0, 1000, 0, b, len);
        CHECK(r < 0 && sim.commands == cmds, "read length %u: r %d", len, r);
        r = tud_msc_write10_cb(0, 1000, 0, b, len);
        CHECK(r < 0 && sim.written.empty() && sim.commands == cmds, "write length %u: r %d", len, r);
    }
    int32_t r = tud_msc_read10_cb(0, 1000, 0, b, 4096);
    CHECK(r == 4096, "a normal read still works: r %d", r);
}

// ---- ATA PASS-THROUGH (SAT) ------------------------------------------------
//
// The host side below follows TinyUSB 0.18 (pico-sdk 2.2.0, msc_device.c) for
// a command that is not READ(10)/WRITE(10): the CBW arrives in the endpoint
// buffer, tud_msc_scsi_cb runs with the host's length cast to 16 bits (and is
// skipped while sense is pending), and a negative return fails the command.
// REQUEST SENSE is TinyUSB's own: 18 bytes of fixed sense from the stored
// key/ASC/ASCQ, then tud_msc_request_sense_cb over that buffer, the result cut
// to the host's allocation length, and the stored sense cleared.

static uint8_t ep[CFG_TUD_MSC_EP_BUFSIZE];

static std::vector<uint8_t> request_sense(uint8_t alloc) {
    static uint8_t b[CFG_TUD_MSC_EP_BUFSIZE];
    memset(b, 0, sizeof b);
    b[0] = 0xF0; b[2] = last_key & 0x0F; b[7] = 10; b[12] = last_asc; b[13] = last_ascq;
    int32_t n = 18;
#if ATABOY_SAT
    n = tud_msc_request_sense_cb(0, b, (uint16_t)sizeof b);
#endif
    last_key = last_asc = last_ascq = 0;
    if (n > alloc) n = alloc;
    return std::vector<uint8_t>(b, b + (n < 0 ? 0 : n));
}

struct SatResult {
    bool called = false;          // the app callback ran
    int32_t r = -1;               // what it returned
    std::vector<uint8_t> data;    // data the host received
    std::vector<uint8_t> sense;   // REQUEST SENSE result after a failure
    bool ok() const { return r >= 0; }
};

static SatResult sat_cmd(const std::vector<uint8_t> &cdb, uint32_t xfer, bool dir_in,
                         uint8_t alloc = 32, bool autosense = true) {
    SatResult s;
    uint8_t cb[16] = {0};
    memcpy(cb, cdb.data(), cdb.size());
    memset(ep, 0xA5, sizeof ep);
    const uint8_t cbw[15] = { 0x55, 0x53, 0x42, 0x43, 0x78, 0x56, 0x34, 0x12,
                              (uint8_t)xfer, (uint8_t)(xfer >> 8), (uint8_t)(xfer >> 16), (uint8_t)(xfer >> 24),
                              (uint8_t)(dir_in ? 0x80 : 0x00), 0x00, (uint8_t)cdb.size() };
    memcpy(ep, cbw, 15);
    memcpy(ep + 15, cb, 16);
    if (last_key == 0) {
        s.called = true;
        uint64_t t0 = mock_now_ns;
        s.r = tud_msc_scsi_cb(0, cb, ep, (uint16_t)xfer);
        note_cb(mock_now_ns - t0);
        note_cmd("ATA PASS-THROUGH", mock_now_ns - t0);
    }
    tud_msc_scsi_complete_cb(0, cb);
    if (s.r > 0) s.data.assign(ep, ep + ((uint32_t)s.r < xfer ? (uint32_t)s.r : xfer));
    if (s.r < 0 && autosense) s.sense = request_sense(alloc);
    return s;
}

// CDB builders. b1 carries PROTOCOL (and EXTEND for the 16-byte form).
static std::vector<uint8_t> pt12(uint8_t proto, uint8_t b2, uint8_t feat, uint8_t cnt,
                                 uint32_t lba24, uint8_t dev, uint8_t cmd) {
    return { 0xA1, (uint8_t)(proto << 1), b2, feat, cnt, (uint8_t)lba24, (uint8_t)(lba24 >> 8),
             (uint8_t)(lba24 >> 16), dev, cmd, 0, 0 };
}
static std::vector<uint8_t> pt16(uint8_t proto, bool ext, uint8_t b2, uint8_t feat, uint8_t cnt,
                                 uint64_t lba, uint8_t dev, uint8_t cmd) {
    return { 0x85, (uint8_t)((proto << 1) | (ext ? 1 : 0)), b2, 0, feat, 0, cnt,
             (uint8_t)(ext ? lba >> 24 : 0), (uint8_t)lba, (uint8_t)(ext ? lba >> 32 : 0), (uint8_t)(lba >> 8),
             (uint8_t)(ext ? lba >> 40 : 0), (uint8_t)(lba >> 16), dev, cmd, 0 };
}
static std::vector<uint8_t> smart_status12(uint8_t b2 = 0x20) { return pt12(3, b2, 0xDA, 0, 0xC24F00, 0xA0, 0xB0); }
static std::vector<uint8_t> native_max12() { return pt12(3, 0x20, 0, 0, 0, 0x40, 0xF8); }
static std::vector<uint8_t> native_max_ext16() { return pt16(3, true, 0x20, 0, 0, 0, 0x40, 0x27); }
static std::vector<uint8_t> verify12(uint8_t b2, uint8_t cnt, uint32_t lba28) {
    return pt12(3, b2, 0, cnt, lba28 & 0xFFFFFF, (uint8_t)(0xE0 | ((lba28 >> 24) & 0x0F)), 0x40);
}

// The ATA Status Return descriptor, as sat.c documents it.
static bool is_desc(const std::vector<uint8_t> &s) {
    return s.size() >= 22 && s[0] == 0x72 && s[7] == 0x0E && s[8] == 0x09 && s[9] == 0x0C;
}
static uint32_t desc_lba28(const std::vector<uint8_t> &s) {
    return s[15] | (s[17] << 8) | (s[19] << 16) | ((uint32_t)(s[20] & 0x0F) << 24);
}
static uint64_t desc_lba48(const std::vector<uint8_t> &s) {
    return s[15] | ((uint64_t)s[17] << 8) | ((uint64_t)s[19] << 16) |
           ((uint64_t)s[14] << 24) | ((uint64_t)s[16] << 32) | ((uint64_t)s[18] << 40);
}
static bool sense_is(const std::vector<uint8_t> &s, uint8_t key, uint8_t asc, uint8_t ascq) {
    if (s.size() >= 4 && s[0] == 0x72) return s[1] == key && s[2] == asc && s[3] == ascq;
    if (s.size() >= 14 && (s[0] & 0x7F) == 0x70) return (s[2] & 0x0F) == key && s[12] == asc && s[13] == ascq;
    return false;
}

// Review finding L2: a slave in use with a master on the cable (auto-mount
// uses the saved device without probing; the probe passes over a master
// busy for more than 10 s). After a soft reset the master stays busy for
// 3 s. The slave must not be selected before the master is done, or the
// select is lost and the slave is left unselected. Through the READ(10)
// path and through a SAT abort.
static void test_slave_with_master_after_reset(Mode m) {
    setup(m == LBA ? "slave with a master present, soft reset, LBA" : "slave with a master present, soft reset, CHS", m);
    sim.slave = true; sim.master_present = true;
    ide_select_device(0xB0);
    if (m == CHS) ide_set_geometry(config.heads, config.spt);
    else { ide_write_reg(6, 0xB0); busy_wait_us_32(1); }
    uint16_t id[256];
    CHECK(ide_identify(id), "slave IDENTIFY");
    sim.violations = 0;
    HostRead g0 = host_read10(100, 8);
    CHECK(g0.ok && matches_medium(g0, 100), "baseline slave read");
    sim.bad[5003] = {BAD_HANG, 0};
    uint8_t buf[8 * 512]; uint32_t done = 0;
    int32_t r = ide_read_sectors_partial(5000, 8, buf, &done);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(r < 0 && done == 3 && f.reset && !f.reset_failed, "r %d done %u reset %d failed %d", r, done, f.reset, f.reset_failed);
    CHECK(mock_now_ns >= sim.master_ready_at, "the reset finished before the master did");
    CHECK(((sim.reg[6] >> 4) & 1) == 1, "slave not selected after the reset (DH %02X)", sim.reg[6]);
    CHECK(sim.violations == 0, "violations %d (a register write while the master was busy)", sim.violations);
    sim.bad.clear();
    HostRead g = host_read10(100, 8);
    CHECK(g.ok && matches_medium(g, 100), "slave usable after the reset");
#if ATABOY_SAT
    sim.hang_cmd = 0xB0;                            // a SAT command the drive never finishes
    int srst0 = sim.srst;
    SatResult s = sat_cmd(smart_status12(), 0, false);
    CHECK(!s.ok() && sim.srst > srst0 && sense_is(s.sense, 0x0B, 0x00, 0x00), "SAT abort: r %d", s.r);
    CHECK(((sim.reg[6] >> 4) & 1) == 1 && sim.violations == 0, "after the SAT abort: DH %02X violations %d",
          sim.reg[6], sim.violations);
    sim.hang_cmd = 0;
    s = sat_cmd(smart_status12(), 0, false);
    CHECK(is_desc(s.sense) && sense_is(s.sense, 0x01, 0x00, 0x1D), "SAT after the abort");
#endif
    HostRead g2 = host_read10(200, 8);
    CHECK(g2.ok && matches_medium(g2, 200) && sim.violations == 0, "slave usable at the end, violations %d", sim.violations);
}

// ---- 0.6f3p7: long internal pauses, wall-clock limits, hardware reset -----
//
// Found on hardware 2026-09-24 (ST380011A alone on the cable as a slave,
// 0.6f3p6): one single-sector WRITE kept the drive busy past the commit
// wait, which counted polls (about 1 s) instead of time. The firmware
// soft-reset the drive in the middle of its own work; the drive stayed busy
// through that and read 0xFF until a re-detect pulsed RESET-. The sector held
// the written data.

static const char *lba_or_chs(Mode m) { return m == LBA ? "LBA" : "CHS"; }
static std::vector<uint8_t> fill512(size_t sectors, uint8_t seed) {
    std::vector<uint8_t> v(sectors * 512);
    for (size_t i = 0; i < v.size(); i++) v[i] = (uint8_t)(i * 13 + seed);
    return v;
}
static bool written_is(uint32_t lba, const uint8_t *p) {
    auto w = sim.written.find(lba);
    return w != sim.written.end() && memcmp(w->second.data(), p, 512) == 0;
}
static bool same_failure(const ide_fail_t &a, const ide_fail_t &b) { return memcmp(&a, &b, sizeof a) == 0; }
static uint64_t ms_since(uint64_t t0) { return (mock_now_ns - t0) / 1000000; }

// Pauses the old iteration-counted waits gave up on (3 s), and one well
// inside the command limit (12 s of IDE_CMD_TIMEOUT_MS, 15 s), are waited
// out: no reset, the data right, nothing recorded as a failure. Through the
// USB callbacks, so inside the host command's budget.
static void test_pause_outwaited(Mode m) {
    std::string n = std::string("drive pauses 3 s and 12 s, outwaited, ") + lba_or_chs(m);
    setup(n.c_str(), m);
    config.drive_write_protected = false;
    ide_fail_t f0; ide_last_failure(&f0);
    const uint32_t L = 123456;
    // WRITE(10), one sector, the drive busy 3 s committing it (the finding)
    std::vector<uint8_t> w = fill512(1, 1);
    sim.pause[L] = 3000000000ull;
    uint64_t t0 = mock_now_ns;
    int32_t r = host_write10(L, w.data(), 1);
    CHECK(r == 512 && ms_since(t0) >= 3000, "3 s commit: r %d after %llu ms", r, (unsigned long long)ms_since(t0));
    CHECK(written_is(L, w.data()), "3 s commit: sector content");
    // Eight sectors, the pause after the third: the wait for the fourth DRQ
    std::vector<uint8_t> w8 = fill512(8, 2);
    sim.pause.clear(); sim.pause[L + 12] = 3000000000ull;
    r = host_write10(L + 10, w8.data(), 8);
    bool all = true;
    for (int i = 0; i < 8; i++) all = all && written_is(L + 10 + i, w8.data() + i * 512);
    CHECK(r == 4096 && all, "8 sectors, 3 s after the third: r %d content %d", r, all);
    // READ(10), the drive busy 3 s before the fifth of eight sectors
    sim.pause.clear(); sim.pause[L + 104] = 3000000000ull;
    HostRead h = host_read10(L + 100, 8);
    CHECK(h.ok && matches_medium(h, L + 100), "read, 3 s before sector 5: ok %d size %zu", h.ok, h.data.size());
    // 12 s: past any poll count the old code could have meant, inside 15 s
    sim.pause.clear(); sim.pause[L + 200] = 12000000000ull;
    std::vector<uint8_t> w25 = fill512(1, 3);
    r = host_write10(L + 200, w25.data(), 1);
    CHECK(r == 512 && written_is(L + 200, w25.data()), "12 s commit: r %d", r);
    sim.pause.clear(); sim.pause[L + 300] = 12000000000ull;
    h = host_read10(L + 300, 1);
    CHECK(h.ok && matches_medium(h, L + 300), "read, 12 s: ok %d", h.ok);
    sim.pause.clear();
    ide_fail_t f; ide_last_failure(&f);
    CHECK(same_failure(f, f0), "a failure was recorded (kind %u cmd %02X)", f.kind, f.command);
    CHECK(sim.srst == 0 && sim.hw_resets == 0 && sim.violations == 0, "srst %d hw %d violations %d",
          sim.srst, sim.hw_resets, sim.violations);
}

// A pause longer than IDE_CMD_TIMEOUT_MS ends in a soft reset at the limit,
// recorded as a timeout; the drive comes back from the SRST, so no hardware
// reset. On a read the good prefix is still handed over (issue #13).
static void test_pause_too_long(Mode m) {
    std::string n = std::string("drive pauses 45 s, soft reset at the limit, ") + lba_or_chs(m);
    setup(n.c_str(), m);
    if (m == CHS) chs_user_geometry();
    config.drive_write_protected = false;
    const uint32_t L = 23456;
    std::vector<uint8_t> w = fill512(1, 4);
    sim.pause[L] = 45000000000ull;
    uint64_t t0 = mock_now_ns;
    int32_t r = ide_write_sectors(L, 1, w.data());
    uint64_t ms = ms_since(t0);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(r < 0, "write r %d", r);
    CHECK(ms >= IDE_CMD_TIMEOUT_MS && ms < IDE_CMD_TIMEOUT_MS + 1000, "write gave up after %llu ms", (unsigned long long)ms);
    CHECK(f.kind == IDE_FAIL_TIMEOUT && f.command == 0x30 && f.done == 1 && f.lba == L + 1,
          "kind %u cmd %02X done %u lba %u", f.kind, f.command, f.done, f.lba);
    CHECK(f.status & 0x80, "status %02X: the drive was busy when given up on", f.status);
    CHECK(f.reset && !f.hw_reset && !f.reset_failed && sim.srst == 1 && sim.hw_resets == 0,
          "reset %d hw %d failed %d srst %d hw resets %d", f.reset, f.hw_reset, f.reset_failed, sim.srst, sim.hw_resets);
    if (m == CHS) CHECK(sim.geo_valid && sim.heads == 5 && sim.spt == 34, "geometry %d %u x %u", sim.geo_valid, sim.heads, sim.spt);
    sim.pause.clear();
    sim.pause[L + 103] = 45000000000ull;
    uint8_t buf[8 * 512]; uint32_t done = 0;
    t0 = mock_now_ns;
    r = ide_read_sectors_partial(L + 100, 8, buf, &done);
    ms = ms_since(t0);
    ide_last_failure(&f);
    CHECK(r < 0 && done == 3 && f.kind == IDE_FAIL_TIMEOUT && f.reset && !f.hw_reset && f.lba == L + 103,
          "read r %d done %u kind %u lba %u", r, done, f.kind, f.lba);
    CHECK(ms >= IDE_CMD_TIMEOUT_MS && ms < IDE_CMD_TIMEOUT_MS + 1000, "read gave up after %llu ms", (unsigned long long)ms);
    bool prefix = true;
    for (uint32_t i = 0; i < 3 * 512; i++) prefix = prefix && buf[i] == sim.byte_at(L + 100 + i / 512, (int)(i % 512));
    CHECK(prefix, "the three sectors before the pause are not the medium");
    sim.pause.clear();
    HostRead g = host_read10(L + 100, 8);
    CHECK(g.ok && matches_medium(g, L + 100), "usable after the reset");
    // The limit is for the whole command, as the host's is: two 8 s pauses
    // in one 8-sector read end it at 15 s, after the fifth sector's wait began.
    sim.pause[L + 202] = 8000000000ull;
    sim.pause[L + 205] = 8000000000ull;
    done = 0;
    t0 = mock_now_ns;
    r = ide_read_sectors_partial(L + 200, 8, buf, &done);
    ms = ms_since(t0);
    CHECK(r < 0 && done == 5 && ms >= IDE_CMD_TIMEOUT_MS && ms < IDE_CMD_TIMEOUT_MS + 1000,
          "two 8 s pauses: r %d done %u after %llu ms (a limit per sector would have read all 8)", r, done, (unsigned long long)ms);
    sim.pause.clear();
    CHECK(sim.violations == 0 && sim.reset_writes == 0, "violations %d reset writes %d", sim.violations, sim.reset_writes);
}

// IDENTIFY is timed by the clock too: 7 s (past the old 100000 polls of
// 50 us) is waited out; 12 s (past IDE_IDENTIFY_TIMEOUT_MS) fails.
static void test_identify_slow() {
    setup("IDENTIFY takes 7 s, then 12 s", LBA);
    uint16_t id[256];
    sim.t_identify_extra = 7000000000ull;
    uint64_t t0 = mock_now_ns;
    CHECK(ide_identify(id) && id[27] == ('S' << 8 | 'I'), "7 s IDENTIFY failed");
    CHECK(ms_since(t0) >= 7000, "returned after %llu ms", (unsigned long long)ms_since(t0));
    sim.t_identify_extra = 12000000000ull;
    t0 = mock_now_ns;
    CHECK(!ide_identify(id), "12 s IDENTIFY taken");
    uint64_t ms = ms_since(t0);
    CHECK(ms + 1 >= IDE_IDENTIFY_TIMEOUT_MS && ms < IDE_IDENTIFY_TIMEOUT_MS + 1000, "gave up after %llu ms", (unsigned long long)ms);
    CHECK(sim.srst == 0 && sim.violations == 0, "srst %d violations %d", sim.srst, sim.violations);
}

// The finding itself: a lone slave that stays busy through the soft reset,
// and comes back only from a hardware reset. The firmware must use RESET-
// once, select the slave again, put the CHS geometry back, record that it
// did, and the drive must work afterwards. A drive that stays busy through
// the hardware reset too gets exactly one, and "reset FAILED".
// The failing commands are sent by calling ide.c directly, outside any host
// command, so each recovery runs to its end in one call; the same through
// USB, where it spans host commands, is test_budget_* below. Also here, from
// review 0.6f3p7: IORDY is ignored while the drive is in reset and believed
// again once it is back (R1, R2, L-3), the RESET- pulse is the probe's 50 ms
// (R5), Device Control gets nIEN=0 from the firmware whatever the drive came
// out of reset with (R6).
static void test_lone_slave_hw_reset(Mode m) {
    std::string n = std::string("lone slave, SRST does not bring it back, hardware reset does, ") + lba_or_chs(m);
    setup(n.c_str(), m);
    sim.slave = true;
    ide_select_device(0xB0);
    if (m == CHS) chs_user_geometry();
    else { ide_write_reg(6, 0xB0); busy_wait_us_32(1); }
    uint16_t id[256];
    CHECK(ide_identify(id), "slave IDENTIFY");
    config.drive_write_protected = false;
    config.iordy_enabled = true; ide_set_iordy(true);
    sim.hw_reset_nien = true;
    sim.violations = 0; sim.init_params = 0;
    sim.srst_wedges = true;
    const uint32_t L = 2816372 % 166600;            // the finding's LBA, folded into this medium
    std::vector<uint8_t> w = fill512(1, 5);
    sim.pause[L] = 45000000000ull;
    uint64_t t0 = mock_now_ns;
    int32_t r = ide_write_sectors(L, 1, w.data());
    ide_fail_t f; ide_last_failure(&f);
    CHECK(r < 0 && written_is(L, w.data()), "write r %d; the drive has the data", r);
    CHECK(f.kind == IDE_FAIL_TIMEOUT && f.command == 0x30 && f.done == 1, "kind %u cmd %02X done %u", f.kind, f.command, f.done);
    CHECK(f.reset && f.hw_reset && !f.reset_failed, "reset %d hw %d failed %d", f.reset, f.hw_reset, f.reset_failed);
    CHECK(sim.srst == 1 && sim.hw_resets == 1, "srst %d hardware resets %d", sim.srst, sim.hw_resets);
    CHECK(((sim.reg[6] >> 4) & 1) == 1, "slave not selected after the hardware reset (DH %02X)", sim.reg[6]);
    if (m == CHS)
        CHECK(sim.geo_valid && sim.heads == 5 && sim.spt == 34 && sim.init_params == 1,
              "geometry after the hardware reset: %d %u x %u, %d INITIALIZE", sim.geo_valid, sim.heads, sim.spt, sim.init_params);
    CHECK(sim.cmd_count[0x10] >= 1, "no RECALIBRATE after the hardware reset");
    CHECK(ms_since(t0) < IDE_CMD_TIMEOUT_MS + 2 * 31000 + 5000, "took %llu ms", (unsigned long long)ms_since(t0));
    CHECK(sim.iordy_stalls == 0, "%d register cycles with IORDY believed while the drive held it low", sim.iordy_stalls);
    CHECK(mock_iordy_inover == GPIO_OVERRIDE_NORMAL, "IORDY not believed again once the drive was back (%u)", mock_iordy_inover);
    CHECK(sim.min_reset_low >= 50000000ull, "RESET- held low %llu us, the probe holds it 50 ms",
          (unsigned long long)(sim.min_reset_low / 1000));
    CHECK(!(sim.devctl & 0x02), "nIEN left set after the hardware reset (Device Control %02X)", sim.devctl);
    sim.pause.clear();
    HostRead g = host_read10(100, 8);
    CHECK(g.ok && matches_medium(g, 100), "slave usable after the hardware reset");
    std::vector<uint8_t> w2 = fill512(1, 6);
    r = host_write10(L, w2.data(), 1);
    CHECK(r == 512 && written_is(L, w2.data()), "write after the hardware reset: r %d", r);
    // The same through a read that hangs: reset, hardware reset, prefix kept.
    sim.bad[5003] = {BAD_HANG, 0};
    uint8_t buf[8 * 512]; uint32_t done = 0;
    r = ide_read_sectors_partial(5000, 8, buf, &done);
    ide_last_failure(&f);
    CHECK(r < 0 && done == 3 && f.hw_reset && !f.reset_failed && sim.hw_resets == 2, "hang: r %d done %u hw %d hw resets %d",
          r, done, f.hw_reset, sim.hw_resets);
    sim.bad.clear();
#if ATABOY_SAT
    // A SAT command the drive never finishes: the abort escalates the same
    // way. Through USB, so it spans host commands: the one that timed out
    // (10 s, then the soft reset for the rest of its 20 s), a retry that
    // waits out most of the SRST's 31 s, and one that sends RESET- and gets
    // the drive back. Each is answered inside its budget; the retries are
    // refused as not ready, and nothing new reaches the drive until it is
    // back. The failure record says what happened (review L-2).
    sim.hang_cmd = 0xF8;
    int f8 = sim.cmd_count[0xF8];
    SatResult s = sat_cmd(native_max12(), 0, false);
    CHECK(!s.ok() && sense_is(s.sense, 0x0B, 0x00, 0x00) && sim.hw_resets == 2, "SAT abort: hw resets %d", sim.hw_resets);
    ide_last_failure(&f);
    CHECK(f.kind == IDE_FAIL_TIMEOUT && f.command == 0xF8 && f.reset && f.pending && !f.hw_reset,
          "SAT abort record: kind %u cmd %02X reset %d pending %d hw %d", f.kind, f.command, f.reset, f.pending, f.hw_reset);
    sim.hang_cmd = 0;
    int tries = 0;
    do { s = sat_cmd(native_max12(), 0, false); tries++; }
    while (tries < 5 && sense_is(s.sense, 0x02, 0x04, 0x00));
    CHECK(is_desc(s.sense) && sense_is(s.sense, 0x01, 0x00, 0x1D) && tries == 2, "SAT after the hardware reset, try %d", tries);
    CHECK(sim.hw_resets == 3 && sim.cmd_count[0xF8] == f8 + 2, "hw resets %d, F8 sent %d times (want 2: the hung one, the good one)",
          sim.hw_resets, sim.cmd_count[0xF8] - f8);
    ide_last_failure(&f);
    CHECK(f.command == 0xF8 && f.hw_reset && !f.pending && !f.reset_failed, "record after: cmd %02X hw %d pending %d failed %d",
          f.command, f.hw_reset, f.pending, f.reset_failed);
#endif
    CHECK(sim.violations == 0, "violations %d", sim.violations);
    // Stays busy through the hardware reset as well: one attempt, then FAILED.
    sim.hw_reset_wedges = true;
    sim.pause[L] = 45000000000ull;
    int hw = sim.hw_resets;
    r = ide_write_sectors(L, 1, w.data());
    ide_last_failure(&f);
    CHECK(r < 0 && f.reset && f.hw_reset && f.reset_failed, "wedged: reset %d hw %d failed %d", f.reset, f.hw_reset, f.reset_failed);
    CHECK(sim.hw_resets == hw + 1, "wedged: %d hardware resets, want exactly one", sim.hw_resets - hw);
    hw = sim.hw_resets;
    CHECK(sim.short_resets == 0, "RESET- held low too briefly %d times", sim.short_resets);
    // Review L-3: the drive never came back, so IORDY stays ignored; the
    // commands below read status from a drive still holding it low.
    CHECK(mock_iordy_inover == GPIO_OVERRIDE_HIGH, "IORDY believed again although the drive never came back");
    HostRead d = host_read10(100, 8);
    CHECK(!d.ok && d.data.empty() && sim.hw_resets == hw, "while wedged: ok %d, %d more hardware resets", d.ok, sim.hw_resets - hw);
    ide_last_failure(&f);
    CHECK(f.kind == IDE_FAIL_NOT_READY, "while wedged: kind %u", f.kind);
    CHECK(sim.iordy_stalls == 0, "%d register cycles with IORDY believed while the drive held it low", sim.iordy_stalls);
    config.iordy_enabled = false;
}

// A master that stays busy after SRST: the reset gives up on device 0 after
// 31 s, and the hardware reset brings it back.
static void test_master_hw_reset() {
    setup("master, SRST does not bring it back, hardware reset does", LBA);
    sim.srst_wedges = true;
    sim.bad[5003] = {BAD_HANG, 0};
    uint8_t buf[8 * 512]; uint32_t done = 0;
    int32_t r = ide_read_sectors_partial(5000, 8, buf, &done);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(r < 0 && done == 3 && f.reset && f.hw_reset && !f.reset_failed && sim.hw_resets == 1,
          "r %d done %u reset %d hw %d failed %d hw resets %d", r, done, f.reset, f.hw_reset, f.reset_failed, sim.hw_resets);
    sim.bad.clear();
    HostRead g = host_read10(5000, 8);
    CHECK(g.ok && matches_medium(g, 5000), "master usable after the hardware reset");
    // A later failure that needs no reset must not carry the old flag over.
    sim.bad[6000] = {BAD_ERR, 0};
    r = ide_read_sectors_partial(6000, 1, buf, &done);
    ide_last_failure(&f);
    CHECK(r < 0 && f.kind == IDE_FAIL_ERR && !f.reset && !f.hw_reset, "plain ERR after a hardware reset: reset %d hw %d", f.reset, f.hw_reset);
    sim.bad.clear();
    CHECK(sim.violations == 0 && sim.reset_writes == 0, "violations %d reset writes %d", sim.violations, sim.reset_writes);
}

// Review finding L3: while an MSC callback is using the drive, usb.c says so
// (usb_msc_ide_busy, which firmware update mode waits on), and it stops
// saying so when the callback returns. Every command the drive gets from a
// callback must come while the flag is set.
static void test_msc_busy_flag() {
    setup("MSC busy flag (firmware update waits on it)", LBA);
    config.drive_write_protected = false;
    sim.busy_probe = [] { return usb_msc_ide_busy(); };
    CHECK(!usb_msc_ide_busy(), "busy before any command");
    int c0 = sim.commands;
    HostRead h = host_read10(100, 16);
    CHECK(h.ok && !usb_msc_ide_busy(), "after READ(10): ok %d", h.ok);
    std::vector<uint8_t> w(4096, 0x3C);
    int32_t r = host_write10(500, w.data(), 8);
    CHECK(r == 4096 && !usb_msc_ide_busy(), "after WRITE(10): r %d", r);
    sim.bad[700] = {BAD_ERR, 0};
    h = host_read10(700, 1);
    CHECK(!h.ok && !usb_msc_ide_busy(), "after a failed READ(10)");
    request_sense(18);                              // the host's auto-sense for that failure
    sim.bad.clear();
#if ATABOY_SAT
    SatResult s = sat_cmd(pt12(4, 0x0E, 0, 2, 100, 0xE0, 0x20), 1024, true);
    CHECK(s.r == 1024 && !usb_msc_ide_busy(), "after a SAT read: r %d", s.r);
    s = sat_cmd(smart_status12(), 0, false);
    CHECK(is_desc(s.sense) && !usb_msc_ide_busy(), "after a SAT non-data command");
#endif
    CHECK(sim.commands >= c0 + 3 && sim.cmds_while_not_busy == 0, "%d commands, %d of them with the flag clear",
          sim.commands - c0, sim.cmds_while_not_busy);
    sim.busy_probe = nullptr;
    // Unmounted: the callbacks refuse without touching the drive.
    is_mounted = false;
    c0 = sim.commands;
    h = host_read10(100, 8);
    r = host_write10(500, w.data(), 8);
    CHECK(!h.ok && r < 0 && sim.commands == c0 && !usb_msc_ide_busy(), "unmounted: commands %d", sim.commands - c0);
    is_mounted = true;
}

#if ATABOY_SAT
static const char *mode_name(Mode m) { return m == LBA ? "LBA" : "CHS"; }
static std::string name2(const char *what, Mode m) { return std::string(what) + ", " + mode_name(m); }

// SMART RETURN STATUS: CK_COND, RECOVERED ERROR 00/1D, and 4F/C2 or F4/2C in
// LBA mid/high. Allowed in CHS mode. Both byte-2 forms (hdparm 0x20, smartctl 0x2C).
static void test_sat_smart_status(Mode m) {
    std::string n = name2("SAT SMART RETURN STATUS", m);
    setup(n.c_str(), m);
    for (int exceeded = 0; exceeded < 2; exceeded++) {
        sim.smart_exceeded = exceeded != 0;
        std::vector<uint8_t> forms[] = { smart_status12(0x20), smart_status12(0x2C),
                                         pt16(3, false, 0x2C, 0xDA, 0, 0xC24F00, 0x00, 0xB0) };
        for (auto &cdb : forms) {
            int cmds = sim.non_id_commands();
            SatResult s = sat_cmd(cdb, 0, false);
            CHECK(s.called && !s.ok(), "exceeded %d: r %d", exceeded, s.r);
            CHECK(sim.non_id_commands() == cmds + 1, "commands %d", sim.non_id_commands() - cmds);
            CHECK(is_desc(s.sense) && s.sense.size() == 22, "descriptor, size %zu", s.sense.size());
            if (!is_desc(s.sense)) continue;
            CHECK(sense_is(s.sense, 0x01, 0x00, 0x1D), "sense %02x/%02x/%02x", s.sense[1], s.sense[2], s.sense[3]);
            CHECK(s.sense[17] == (exceeded ? 0xF4 : 0x4F) && s.sense[19] == (exceeded ? 0x2C : 0xC2),
                  "exceeded %d: mid/high %02x/%02x", exceeded, s.sense[17], s.sense[19]);
            CHECK(s.sense[21] == 0x50 && s.sense[11] == 0x00 && s.sense[10] == 0x00,
                  "status %02x error %02x extend %02x", s.sense[21], s.sense[11], s.sense[10]);
            CHECK((s.sense[20] & 0x10) == 0, "device %02x", s.sense[20]);
        }
    }
    // CK_COND=0 is refused: the answer could not be returned.
    int cmds = sim.non_id_commands();
    SatResult s = sat_cmd(smart_status12(0x0C), 0, false);
    CHECK(!s.ok() && sense_is(s.sense, 0x05, 0x24, 0x00) && s.sense.size() == 18 && sim.non_id_commands() == cmds,
          "CK_COND=0: r %d commands %d", s.r, sim.non_id_commands() - cmds);
    CHECK(sim.non_id_data_reads() == 0 && sim.srst == 0 && sim.violations == 0 && sim.hob_selects == 0,
          "data reads %d srst %d violations %d hob %d", sim.non_id_data_reads(), sim.srst, sim.violations, sim.hob_selects);
}

// READ NATIVE MAX ADDRESS on a drive with a Host Protected Area. The numbers
// are the ST380011A donor's: 156,250,000 sectors reported, 156,301,488 in the
// model's specification.
static void test_sat_native_max(Mode m) {
    std::string n = name2("SAT READ NATIVE MAX, HPA", m);
    setup(n.c_str(), m);
    sim.nsect = 156250000; sim.native_max = 156301488;
    SatResult s = sat_cmd(native_max12(), 0, false);
    CHECK(!s.ok() && is_desc(s.sense) && sense_is(s.sense, 0x01, 0x00, 0x1D), "F8: r %d size %zu", s.r, s.sense.size());
    if (is_desc(s.sense)) {
        CHECK(desc_lba28(s.sense) == 156301487u, "F8 max LBA %u", desc_lba28(s.sense));
        CHECK(s.sense[10] == 0 && s.sense[21] == 0x50 && (s.sense[20] & 0x40), "extend %u status %02x device %02x",
              s.sense[10], s.sense[21], s.sense[20]);
    }
    CHECK(sim.hob_selects == 0, "28-bit command read HOB %d times", sim.hob_selects);
    // the same drive through the 48-bit form
    s = sat_cmd(native_max_ext16(), 0, false);
    CHECK(!s.ok() && is_desc(s.sense) && sense_is(s.sense, 0x01, 0x00, 0x1D), "27: r %d", s.r);
    if (is_desc(s.sense)) {
        CHECK(desc_lba48(s.sense) == 156301487ull, "27 max LBA %llu", (unsigned long long)desc_lba48(s.sense));
        CHECK(s.sense[10] == 0x01, "EXTEND %u", s.sense[10]);
    }
    // a native size past 2^32 only fits the 48-bit form
    sim.native_max = 0x123456789ABull + 1;
    s = sat_cmd(native_max_ext16(), 0, false);
    CHECK(is_desc(s.sense) && desc_lba48(s.sense) == 0x123456789ABull, "48-bit max %llx",
          is_desc(s.sense) ? (unsigned long long)desc_lba48(s.sense) : 0ull);
    s = sat_cmd(native_max12(), 0, false);
    CHECK(is_desc(s.sense) && desc_lba28(s.sense) == 0x0FFFFFFFu, "F8 caps at 0x0FFFFFFF: %x",
          is_desc(s.sense) ? desc_lba28(s.sense) : 0u);
    CHECK((sim.devctl & 0x80) == 0, "HOB left set in Device Control: %02x", sim.devctl);
    CHECK(sim.hob_selects == 2, "HOB selects %d (one per 0x27)", sim.hob_selects);
    CHECK(sim.non_id_data_reads() == 0 && sim.srst == 0 && sim.violations == 0, "data reads %d srst %d violations %d",
          sim.non_id_data_reads(), sim.srst, sim.violations);
    CHECK(sim.cmd_count[0xF9] == 0 && sim.cmd_count[0x37] == 0, "SET MAX issued");
    // and a read afterwards still sees the current size, nothing changed
    HostRead h = host_read10(1000, 8);
    CHECK(h.ok && matches_medium(h, 1000), "read after the queries");
}

// A drive without 48-bit support aborts 0x27. The firmware must not then set
// HOB in Device Control, which such a drive does not know.
static void test_sat_native_max_ext_abort() {
    setup("SAT READ NATIVE MAX EXT, drive without 48-bit", LBA);
    sim.lba48 = false;
    SatResult s = sat_cmd(native_max_ext16(), 0, false);
    CHECK(!s.ok() && is_desc(s.sense) && sense_is(s.sense, 0x0B, 0x00, 0x00), "r %d size %zu", s.r, s.sense.size());
    if (is_desc(s.sense))
        CHECK(s.sense[11] == 0x04 && s.sense[21] == 0x51 && s.sense[10] == 0, "error %02x status %02x extend %u",
              s.sense[11], s.sense[21], s.sense[10]);
    CHECK(sim.hob_selects == 0, "HOB selected on an aborted 48-bit command");
    sim.hpa_feature = false;
    s = sat_cmd(native_max12(), 0, false);
    CHECK(is_desc(s.sense) && sense_is(s.sense, 0x0B, 0x00, 0x00) && s.sense[11] == 0x04, "F8 unsupported");
    CHECK(sim.srst == 0 && sim.non_id_data_reads() == 0 && sim.violations == 0, "srst %d reads %d", sim.srst, sim.non_id_data_reads());
    HostRead h = host_read10(10, 8);
    CHECK(h.ok && matches_medium(h, 10), "usable afterwards");
}

// READ VERIFY: no data phase, GOOD with CK_COND=0, the failing LBA from the
// drive's own registers on an error, IDNF past the end, and LBA mode only.
static void test_sat_verify() {
    setup("SAT READ VERIFY", LBA);
    sim.nsect = 0x02000000;
    SatResult s = sat_cmd(verify12(0x00, 8, 1000), 0, false);
    CHECK(s.called && s.r == 0 && s.sense.empty() && last_key == 0, "good verify: r %d", s.r);
    for (uint32_t l = 1000; l < 1008; l++) CHECK(sim.attempts[l] == 1, "lba %u attempts %d", l, sim.attempts[l]);
    CHECK(sim.attempts[1008] == 0 && sim.attempts[999] == 0, "verified outside the range");
    s = sat_cmd(verify12(0x20, 8, 1000), 0, false);
    CHECK(!s.ok() && is_desc(s.sense) && sense_is(s.sense, 0x01, 0x00, 0x1D) && s.sense[21] == 0x50,
          "CK_COND=1: r %d", s.r);
    sim.attempts.clear();
    s = sat_cmd(verify12(0x0C, 0, 2000), 0, false);       // count 0 = 256 sectors; T_DIR, BYTE_BLOCK set
    CHECK(s.r == 0, "count 0: r %d sense %zu %02x %02x %02x", s.r, s.sense.size(), s.sense.size() > 13 ? s.sense[2] : 0, s.sense.size() > 13 ? s.sense[12] : 0, s.sense.size() > 13 ? s.sense[13] : 0);
    CHECK(sim.attempts[2000] == 1 && sim.attempts[2255] == 1 && sim.attempts[2256] == 0, "count 0 is 256 sectors");

    // a bad sector: UNC with its LBA, above 2^24 so the device nibble counts
    const uint32_t bad = 0x01234567;
    sim.bad[bad] = {BAD_ERR, 0};
    int srst0 = sim.srst;
    s = sat_cmd(verify12(0x00, 16, bad - 7), 0, false);
    CHECK(!s.ok() && is_desc(s.sense) && sense_is(s.sense, 0x03, 0x11, 0x00), "bad: r %d", s.r);
    if (is_desc(s.sense)) {
        CHECK(desc_lba28(s.sense) == bad, "failing LBA %x, want %x", desc_lba28(s.sense), bad);
        CHECK(s.sense[11] == 0x40 && (s.sense[21] & 0x81) == 0x01, "error %02x status %02x", s.sense[11], s.sense[21]);
    }
    CHECK(sim.srst == srst0, "no soft reset after an ATA error");
    // one that ERRs with data on offer: still no data read
    sim.bad[bad] = {BAD_ERR_DRQ, 0};
    s = sat_cmd(verify12(0x20, 1, bad), 0, false);
    CHECK(is_desc(s.sense) && desc_lba28(s.sense) == bad, "ERR+DRQ sector");
    // past the end: IDNF at the first missing sector
    s = sat_cmd(verify12(0x00, 8, sim.nsect - 4), 0, false);
    CHECK(is_desc(s.sense) && sense_is(s.sense, 0x03, 0x14, 0x01) && desc_lba28(s.sense) == sim.nsect,
          "past the end: %x", is_desc(s.sense) ? desc_lba28(s.sense) : 0u);
    // PIO-in protocol on 0x40 is refused before the drive sees anything
    int cmds = sim.commands;
    s = sat_cmd(pt12(4, 0x0E, 0, 1, 1000, 0xE0, 0x40), 512, true);
    CHECK(!s.ok() && sense_is(s.sense, 0x05, 0x24, 0x00) && sim.commands == cmds, "verify as PIO-in");
    CHECK(sim.data_reads == 0 && sim.violations == 0, "data reads %d violations %d", sim.data_reads, sim.violations);
    sim.bad.clear();
    HostRead h = host_read10(1000, 16);
    CHECK(h.ok && matches_medium(h, 1000), "usable afterwards");

    setup("SAT READ VERIFY refused in CHS mode", CHS);
    cmds = sim.commands;
    s = sat_cmd(verify12(0x20, 1, 100), 0, false);
    CHECK(!s.ok() && sense_is(s.sense, 0x05, 0x24, 0x00) && sim.commands == cmds, "CHS verify: r %d", s.r);
}

// The drive never ends the command: the path times out, aborts with a soft
// reset, and in CHS mode puts the geometry back before anything else runs.
static void test_sat_nondata_timeout(Mode m) {
    std::string n = name2("SAT non-data timeout", m);
    setup(n.c_str(), m);
    if (m == CHS) chs_user_geometry();
    sim.hang_cmd = 0xB0;
    uint64_t t0 = mock_now_ns;
    SatResult s = sat_cmd(smart_status12(), 0, false);
    uint64_t took = (mock_now_ns - t0) / 1000000;
    CHECK(!s.ok() && sense_is(s.sense, 0x0B, 0x00, 0x00) && s.sense.size() == 18 && s.sense[0] == 0xF0,
          "fixed sense, no descriptor: size %zu", s.sense.size());
    CHECK(took >= 10000 && took < 12000, "gave up after %llu ms", (unsigned long long)took);
    CHECK(sim.srst >= 1, "a hung command must be aborted");
    if (m == CHS)
        CHECK(sim.geo_valid && sim.heads == 5 && sim.spt == 34 && sim.init_params > 0,
              "geometry %d %u x %u", sim.geo_valid, sim.heads, sim.spt);
    CHECK(sim.non_id_data_reads() == 0, "data reads %d", sim.non_id_data_reads());
    sim.hang_cmd = 0;
    HostRead h = host_read10(1234, 8);
    CHECK(h.ok && matches_medium(h, 1234), "read after the abort");
    // the same through READ NATIVE MAX EXT, and a verify that hangs on a sector
    sim.hang_cmd = 0x27;
    int srst0 = sim.srst;
    s = sat_cmd(native_max_ext16(), 0, false);
    CHECK(sense_is(s.sense, 0x0B, 0x00, 0x00) && s.sense.size() == 18 && sim.srst > srst0, "0x27 timeout");
    sim.hang_cmd = 0;
    if (m == LBA) {
        sim.bad[3003] = {BAD_HANG, 0};
        srst0 = sim.srst;
        s = sat_cmd(verify12(0x20, 8, 3000), 0, false);
        CHECK(sense_is(s.sense, 0x0B, 0x00, 0x00) && s.sense.size() == 18 && sim.srst > srst0, "verify hang");
        sim.bad.clear();
    }
    HostRead g = host_read10(1234, 8);
    CHECK(g.ok && matches_medium(g, 1234), "read after the second abort");
    CHECK(sim.violations == 0 && sim.reset_writes == 0, "violations %d reset writes %d", sim.violations, sim.reset_writes);
}

// A drive that answers a non-data command with DRQ: the data is never read,
// and a soft reset ends the command.
static void test_sat_nondata_drq() {
    setup("SAT non-data command ends with DRQ", LBA);
    sim.nondata_drq = true;
    SatResult s = sat_cmd(smart_status12(), 0, false);
    CHECK(!s.ok() && sense_is(s.sense, 0x0B, 0x00, 0x00) && s.sense.size() == 18, "sense size %zu", s.sense.size());
    CHECK(sim.non_id_data_reads() == 0, "data register read %d times", sim.non_id_data_reads());
    CHECK(sim.srst >= 1, "must abort");
    sim.nondata_drq = false;
    HostRead h = host_read10(500, 8);
    CHECK(h.ok && matches_medium(h, 500) && !contains_flawed(h), "usable, nothing stranded");
    CHECK(sim.violations == 0, "violations %d", sim.violations);
}

// A drive still offering data from an earlier command: no SAT command is
// issued over it, whichever path.
static void test_sat_stale_drq(Mode m) {
    std::string n = name2("SAT refuses to issue over stale DRQ", m);
    setup(n.c_str(), m);
    for (int i = 0; i < 256; i++) sim.xfer[i] = 0xDEAD;
    sim.widx = 0; sim.phase = SimDrive::DRQ_IN; sim.status = 0x58; sim.left = 1;
    int cmds = sim.commands;
    SatResult a = sat_cmd(smart_status12(), 0, false);
    SatResult b = sat_cmd(native_max12(), 0, false);
    SatResult c = sat_cmd(pt12(4, 0x0E, 0xD0, 1, 0xC24F00, 0xA0, 0xB0), 512, true);
    CHECK(sense_is(a.sense, 0x02, 0x04, 0x00) && sense_is(b.sense, 0x02, 0x04, 0x00) && sense_is(c.sense, 0x02, 0x04, 0x00),
          "not ready: %zu %zu %zu", a.sense.size(), b.sense.size(), c.sense.size());
    CHECK(sim.commands == cmds, "%d commands issued over stale DRQ", sim.commands - cmds);
    // A row the IDENTIFY words do not gate (so no identity check runs first):
    // the SAT path's own stale DRQ check must refuse it.
    SatResult e = sat_cmd(pt12(4, 0x0E, 0, 1, 0, 0x00, 0xEC), 512, true);
    CHECK(sense_is(e.sense, 0x02, 0x04, 0x00), "IDENTIFY over stale DRQ: not ready expected");
    CHECK(sim.commands == cmds, "%d commands issued over stale DRQ (IDENTIFY row)", sim.commands - cmds);
    CHECK(sim.data_reads == 0 && sim.srst == 0, "data reads %d srst %d", sim.data_reads, sim.srst);
    HostRead h = host_read10(300, 4);       // the READ(10) guard clears it
    CHECK(!h.ok && !contains_flawed(h), "READ(10) guard");
    request_sense(18);                      // the host's auto-sense for that failure
    SatResult d = sat_cmd(smart_status12(), 0, false);
    CHECK(is_desc(d.sense) && sense_is(d.sense, 0x01, 0x00, 0x1D), "SAT works once cleared");
}

// SMART READ DATA / THRESHOLDS / LOG: PIO data-in, the drive's bytes exactly.
static void test_sat_smart_data(Mode m) {
    std::string n = name2("SAT SMART READ DATA / THRESHOLDS / LOG", m);
    setup(n.c_str(), m);
    auto same = [](const SatResult &s, uint32_t first, uint32_t count) {
        if (s.data.size() != count * 512) return false;
        for (uint32_t i = 0; i < s.data.size(); i++)
            if (s.data[i] != sim.byte_at(first + i / 512, (int)(i % 512))) return false;
        return true;
    };
    // smartread.ps1: A1 08 0E D0 01 00 4F C2 A0 B0 00 00
    SatResult s = sat_cmd(pt12(4, 0x0E, 0xD0, 1, 0xC24F00, 0xA0, 0xB0), 512, true);
    CHECK(s.r == 512 && same(s, SimDrive::smart_sector(0xD0, 0, 0), 1), "READ DATA: r %d", s.r);
    s = sat_cmd(pt16(4, false, 0x0E, 0xD1, 1, 0xC24F00, 0x00, 0xB0), 512, true);
    CHECK(s.r == 512 && same(s, SimDrive::smart_sector(0xD1, 0, 0), 1), "READ THRESHOLDS: r %d", s.r);
    s = sat_cmd(pt16(4, false, 0x0E, 0xD5, 3, 0xC24F06, 0xA0, 0xB0), 3 * 512, true);
    CHECK(s.r == 3 * 512 && same(s, SimDrive::smart_sector(0xD5, 0x06, 0), 3), "READ LOG 06h x3: r %d", s.r);
    s = sat_cmd(pt12(4, 0x0E, 0xD5, 8, 0xC24F80, 0xA0, 0xB0), 8 * 512, true);
    CHECK(s.r == 8 * 512 && same(s, SimDrive::smart_sector(0xD5, 0x80, 0), 8), "READ LOG 80h x8: r %d", s.r);
    CHECK(sim.srst == 0 && sim.violations == 0, "srst %d violations %d", sim.srst, sim.violations);
    // a drive without SMART: ABRT, with the registers
    sim.smart_supported = false;
    s = sat_cmd(pt12(4, 0x0E, 0xD0, 1, 0xC24F00, 0xA0, 0xB0), 512, true);
    CHECK(!s.ok() && is_desc(s.sense) && sense_is(s.sense, 0x0B, 0x00, 0x00), "no SMART: r %d", s.r);
    if (is_desc(s.sense)) CHECK(s.sense[11] == 0x04 && s.sense[21] == 0x51, "error %02x status %02x", s.sense[11], s.sense[21]);
    // SMART ENABLE (D8) is never sent
    int cmds = sim.commands;
    s = sat_cmd(pt12(3, 0x20, 0xD8, 0, 0xC24F00, 0xA0, 0xB0), 0, false);
    CHECK(!s.ok() && sense_is(s.sense, 0x05, 0x24, 0x00) && sim.commands == cmds, "D8 refused");
}

// Stage 1 reads now carry the drive's registers when they fail, including
// the 48-bit LBA through HOB for READ SECTORS EXT.
static void test_sat_read_error_registers() {
    setup("SAT READ SECTORS (EXT) error registers", LBA);
    sim.nsect = 0x20000000;
    const uint32_t bad = 0x12345662;
    sim.bad[bad] = {BAD_ERR, 0};
    SatResult s = sat_cmd(pt16(4, true, 0x0E, 0, 4, bad - 2, 0x40, 0x24), 4 * 512, true);
    CHECK(!s.ok() && s.data.empty() && is_desc(s.sense) && sense_is(s.sense, 0x03, 0x11, 0x00), "EXT: r %d", s.r);
    if (is_desc(s.sense)) {
        CHECK(desc_lba48(s.sense) == bad, "EXT failing LBA %llx", (unsigned long long)desc_lba48(s.sense));
        CHECK(s.sense[10] == 0x01 && s.sense[11] == 0x40, "extend %u error %02x", s.sense[10], s.sense[11]);
    }
    CHECK((sim.devctl & 0x80) == 0 && sim.hob_selects == 1, "devctl %02x hob selects %d", sim.devctl, sim.hob_selects);
    const uint32_t bad28 = 0x00ABCDE1;
    sim.bad[bad28] = {BAD_ERR_DRQ, 0};
    s = sat_cmd(pt12(4, 0x0E, 0, 4, (bad28 - 1) & 0xFFFFFF, 0xE0, 0x20), 4 * 512, true);
    CHECK(is_desc(s.sense) && desc_lba28(s.sense) == bad28 && s.sense[10] == 0, "28-bit failing LBA %x",
          is_desc(s.sense) ? desc_lba28(s.sense) : 0u);
    // STATUS is what ended the command (ERR with the flawed block on offer),
    // read before the drain, not what the drive shows after it.
    CHECK(is_desc(s.sense) && s.sense[21] == 0x59, "status %02x, want 59", is_desc(s.sense) ? s.sense[21] : 0);
    CHECK(sim.srst == 0 && sim.violations == 0, "srst %d violations %d", sim.srst, sim.violations);
    sim.bad.clear();
    s = sat_cmd(pt12(4, 0x0E, 0, 2, 100, 0xE0, 0x20), 1024, true);
    CHECK(s.r == 1024 && s.sense.empty(), "good read: r %d", s.r);
}

// How the descriptor reaches the host, and when it must not.
static void test_sat_sense_delivery() {
    setup("SAT sense delivery", LBA);
    // A host that asks for 18 bytes gets the first 18 of the descriptor.
    SatResult s = sat_cmd(smart_status12(), 0, false, 18);
    CHECK(s.sense.size() == 18 && s.sense[0] == 0x72 && s.sense[8] == 0x09 && s.sense[17] == 0x4F,
          "18-byte allocation: size %zu", s.sense.size());
    // Delivered once: the next REQUEST SENSE has nothing.
    std::vector<uint8_t> again = request_sense(32);
    CHECK(again.size() == 18 && again[0] == 0xF0 && (again[2] & 0x0F) == 0, "second REQUEST SENSE %zu", again.size());

    // A READ(10) between the SAT command and REQUEST SENSE sets its own sense
    // (the same 3/11/00 a failed verify sets): the host must get the READ(10)'s
    // fixed sense, never the SAT registers paired with it.
    sim.bad[4000] = {BAD_ERR, 0};
    s = sat_cmd(verify12(0x00, 1, 4000), 0, false, 32, false);
    CHECK(!s.ok() && last_key == 0x03 && last_asc == 0x11, "verify fails");
    static uint8_t b[4096];
    int32_t r = tud_msc_read10_cb(0, 4000, 0, b, 512);
    CHECK(r < 0 && last_key == 0x03, "READ(10) fails: r %d key %u", r, last_key);
    std::vector<uint8_t> rs = request_sense(32);
    CHECK(rs.size() == 18 && rs[0] == 0xF0 && (rs[2] & 0x0F) == 0x03, "READ(10) sense, fixed: size %zu", rs.size());
    // ...and a READ(10) that succeeds in between also drops it
    s = sat_cmd(verify12(0x20, 1, 10), 0, false, 32, false);
    CHECK(!s.ok() && last_key == 0x01, "CK_COND verify");
    r = tud_msc_read10_cb(0, 20, 0, b, 512);
    CHECK(r == 512, "READ(10) ok: r %d", r);
    rs = request_sense(32);
    CHECK(rs.size() == 18 && rs[0] == 0xF0 && (rs[2] & 0x0F) == 0x01, "stale descriptor after READ(10): size %zu", rs.size());
    // Sense replaced by something else (unit attention, say): fixed sense.
    s = sat_cmd(smart_status12(), 0, false, 32, false);
    tud_msc_set_sense(0, 0x06, 0x28, 0x00);
    rs = request_sense(32);
    CHECK(rs.size() == 18 && (rs[2] & 0x0F) == 0x06, "replaced sense: size %zu key %u", rs.size(), rs.size() > 2 ? rs[2] & 0x0F : 0);
    // Refusals are plain fixed sense.
    s = sat_cmd(pt12(3, 0x20, 0, 0, 0, 0x40, 0xF9), 0, false);
    CHECK(s.sense.size() == 18 && sense_is(s.sense, 0x05, 0x24, 0x00), "refusal: size %zu", s.sense.size());
    // With sense pending, TinyUSB does not call the callback at all.
    s = sat_cmd(smart_status12(), 0, false, 32, false);
    int cmds = sim.non_id_commands();
    SatResult t = sat_cmd(smart_status12(), 0, false, 32, false);
    CHECK(!t.called && sim.non_id_commands() == cmds, "callback ran with sense pending");
    request_sense(32);

    // The descriptor belongs to one command and one REQUEST SENSE. Below,
    // the test sets TinyUSB's sense directly, standing in for any other path
    // that sets the same or a similar sense later; the SAT registers must
    // never be paired with it.
    // Delivered once, even if the same key/ASC/ASCQ comes back.
    s = sat_cmd(smart_status12(), 0, false);            // 01/00/1D, delivered
    CHECK(is_desc(s.sense), "CK_COND descriptor");
    tud_msc_set_sense(0, 0x01, 0x00, 0x1D);
    rs = request_sense(32);
    CHECK(rs.size() == 18 && rs[0] == 0xF0, "descriptor served twice: size %zu", rs.size());
    // Same sense key, other ASC: not ours.
    sim.bad[4100] = {BAD_ERR, 0};
    s = sat_cmd(verify12(0x00, 1, 4100), 0, false, 32, false);   // 03/11/00 with registers, pending
    tud_msc_set_sense(0, 0x03, 0x0C, 0x00);
    rs = request_sense(32);
    CHECK(rs.size() == 18 && rs[0] == 0xF0 && rs[12] == 0x0C, "same key, other ASC: size %zu", rs.size());
    // TinyUSB dropped its sense without a REQUEST SENSE (a bulk-only reset,
    // say), then another SAT command ran and ended without sense: the old
    // descriptor must be gone by then.
    s = sat_cmd(verify12(0x00, 1, 4100), 0, false, 32, false);   // pending again
    last_key = last_asc = last_ascq = 0;
    SatResult id = sat_cmd(pt12(4, 0x0E, 0, 1, 0, 0xA0, 0xEC), 512, true);
    CHECK(id.r == 512, "IDENTIFY: r %d", id.r);
    tud_msc_set_sense(0, 0x03, 0x11, 0x00);
    rs = request_sense(32);
    CHECK(rs.size() == 18 && rs[0] == 0xF0, "descriptor outlived the next SAT command: size %zu", rs.size());
    // A WRITE(10) in between drops it, as READ(10) does.
    config.drive_write_protected = false;
    s = sat_cmd(verify12(0x20, 1, 10), 0, false, 32, false);     // 01/00/1D, pending
    std::vector<uint8_t> wb(512, 0x5A);
    r = host_write10(30, wb.data(), 1);
    CHECK(r == 512, "WRITE(10) ok: r %d", r);
    rs = request_sense(32);
    CHECK(rs.size() == 18 && rs[0] == 0xF0, "stale descriptor after WRITE(10): size %zu", rs.size());
    sim.bad.clear();
}

// The optional SAT commands run only when the drive's own IDENTIFY, captured
// by the firmware, says the drive has them (review finding H1). The drive
// below answers every one of them; whenever its IDENTIFY does not show one,
// the firmware must refuse it (5/24/00) without sending anything.
struct SatRow { const char *name; std::vector<uint8_t> cdb; uint32_t xfer; bool in; unsigned bit; };
enum { G_SMART = 1, G_LOG = 2, G_NMAX = 4, G_NMAX_EXT = 8, G_READ_EXT = 16, G_ALL = 31 };

static std::vector<SatRow> gated_rows() {
    return {
        { "SMART READ DATA",       pt12(4, 0x0E, 0xD0, 1, 0xC24F00, 0xA0, 0xB0), 512, true, G_SMART },
        { "SMART READ THRESHOLDS", pt16(4, false, 0x0E, 0xD1, 1, 0xC24F00, 0x00, 0xB0), 512, true, G_SMART },
        { "SMART RETURN STATUS",   smart_status12(), 0, false, G_SMART },
        { "SMART READ LOG",        pt12(4, 0x0E, 0xD5, 1, 0xC24F01, 0xA0, 0xB0), 512, true, G_LOG },
        { "READ NATIVE MAX",       native_max12(), 0, false, G_NMAX },
        { "READ NATIVE MAX EXT",   native_max_ext16(), 0, false, G_NMAX_EXT },
        { "READ SECTORS EXT",      pt16(4, true, 0x0E, 0, 2, 100, 0x40, 0x24), 1024, true, G_READ_EXT },
    };
}

// Run every gated row; those in `allowed` must reach the drive (once), the
// rest must be refused with 5/24/00 and reach nothing. The rows that are not
// gated must work throughout.
static void check_gate(const char *when, unsigned allowed, bool ungated = true) {
    for (auto &r : gated_rows()) {
        int cmds = sim.non_id_commands();
        SatResult s = sat_cmd(r.cdb, r.xfer, r.in);
        if (allowed & r.bit)
            CHECK(sim.non_id_commands() == cmds + 1, "%s: %s was not sent (r %d)", when, r.name, s.r);
        else
            CHECK(!s.ok() && sim.non_id_commands() == cmds && s.sense.size() == 18 && sense_is(s.sense, 0x05, 0x24, 0x00),
                  "%s: %s must be refused before the drive: r %d commands %d sense %zu", when, r.name, s.r,
                  sim.non_id_commands() - cmds, s.sense.size());
    }
    if (!ungated) return;
    SatResult a = sat_cmd(pt12(4, 0x0E, 0, 1, 0, 0xA0, 0xEC), 512, true);
    SatResult b = sat_cmd(pt12(4, 0x0E, 0, 2, 100, 0xE0, 0x20), 1024, true);
    SatResult v = sat_cmd(verify12(0x00, 4, 100), 0, false);
    CHECK(a.r == 512 && b.r == 1024 && v.r == 0, "%s: ungated rows: identify %d read %d verify %d", when, a.r, b.r, v.r);
    if (a.r == 512) {
        uint16_t w83 = (uint16_t)(a.data[166] | (a.data[167] << 8));
        CHECK(w83 == sim.id_w83, "%s: pass-through IDENTIFY word 83 %04x", when, w83);
    }
}

// Words 85 and 87 default to what the simulated drive answers: SMART and HPA
// enabled, 85..87 valid.
static void identify_with(uint16_t w82, uint16_t w83, uint16_t w84,
                          uint16_t w85 = 0x4401, uint16_t w87 = 0x4000) {
    sim.id_w82 = w82; sim.id_w83 = w83; sim.id_w84 = w84; sim.id_w85 = w85; sim.id_w87 = w87;
    uint16_t id[256];
    CHECK(ide_identify(id), "IDENTIFY %04x %04x %04x %04x %04x", w82, w83, w84, w85, w87);
}

// Re-review M-A: a drive swapped on the cable after detection and mounted
// without a new one (Mount only sets is_mounted) must not be judged by the
// old drive's IDENTIFY words. Before a gated command the firmware asks the
// drive for IDENTIFY again and compares serial, model and words 82..84.
struct IdRow { const char *name; std::vector<uint8_t> cdb; uint32_t xfer; bool dir_in; uint8_t op; bool lba_only; };
static std::vector<IdRow> id_check_rows() {
    // Every row the policy marks needs_identity, one of each command byte
    // (review L-1 found READ NATIVE MAX EXT missing here).
    return { { "SMART RETURN STATUS", smart_status12(), 0, false, 0xB0, false },
             { "READ NATIVE MAX", native_max12(), 0, false, 0xF8, false },
             { "READ NATIVE MAX EXT", native_max_ext16(), 0, false, 0x27, false },
             { "READ SECTORS EXT", pt16(4, true, 0x0E, 0, 1, 100, 0x40, 0x24), 512, true, 0x24, true } };
}
static void test_sat_identity_verified(Mode m) {
    std::string n = name2("SAT gated rows verify the drive is the detected one", m);
    setup(n.c_str(), m);
    auto reset_drive = [&]() {
        sim.id_model = "SIMULATED ATA DRIVE"; sim.id_serial = "SIM0000001";
        sim.id_w82 = 0x4401; sim.id_w83 = 0x4400; sim.id_w84 = 0x4001; sim.identify_ok = true;
        sim.id_w85 = 0x4401; sim.id_w87 = 0x4000;
        identify_with(0x4401, 0x4400, 0x4001);            // a detection of this drive
    };
    // want: 1 ran, 0 refused 5/24/00, 2 not ready 2/04/00
    auto run = [&](const IdRow &g, const char *what, int want, int want_ids) {
        int ids = sim.id_commands(), ops = sim.cmd_count[g.op];
        SatResult r = sat_cmd(g.cdb, g.xfer, g.dir_in);
        int got = r.ok() || (is_desc(r.sense) && sense_is(r.sense, 0x01, 0x00, 0x1D)) ? 1
                : sense_is(r.sense, 0x05, 0x24, 0x00) ? 0 : sense_is(r.sense, 0x02, 0x04, 0x00) ? 2 : -1;
        CHECK(got == want, "%s, %s: outcome %d, want %d", g.name, what, got, want);
        CHECK(sim.id_commands() - ids == want_ids, "%s, %s: %d IDENTIFY sent, want %d", g.name, what, sim.id_commands() - ids, want_ids);
        CHECK(sim.cmd_count[g.op] - ops == (want == 1 ? 1 : 0), "%s, %s: %d commands %02X reached the drive",
              g.name, what, sim.cmd_count[g.op] - ops, g.op);
    };
    for (const IdRow &g : id_check_rows()) {
        if (g.lba_only && m != LBA) continue;
        reset_drive();
        run(g, "same drive", 1, 1);
        sim.id_serial = "OTHER00002";                     // same model, another unit
        run(g, "other serial", 0, 1);
        run(g, "after that (words forgotten)", 0, 0);
        reset_drive();
        sim.id_model = "OTHER ATA DRIVE";                 // same serial text, other model
        run(g, "other model", 0, 1);
        reset_drive();
        sim.id_w82 = 0x4001;                              // same drive text, words changed
        run(g, "other words 82..84", 0, 1);
        reset_drive();
        sim.id_w85 = 0x4001;                              // HPA switched off since detection
        run(g, "other word 85", 0, 1);
        reset_drive();
        sim.id_w87 = 0x4001;                              // word 87 differs
        run(g, "other word 87", 0, 1);
        reset_drive();
        sim.id_model = "CONNER CFS1275A"; sim.id_serial = "CONNER0001";
        sim.id_w82 = 0; sim.id_w83 = 0; sim.id_w84 = 0;
        run(g, "CFS1275A-like drive", 0, 1);
        reset_drive();
        sim.identify_ok = false;                          // does not answer IDENTIFY
        run(g, "drive aborts IDENTIFY", 0, 1);
        reset_drive();
        ide_id_words_forget();                            // manual CHS: no words
        int cmds = sim.commands;
        run(g, "no words held", 0, 0);
        CHECK(sim.commands == cmds, "%s, no words: %d commands sent", g.name, sim.commands - cmds);
        // Busy for longer than the check waits, then ready: the command must
        // not go out unverified to whatever drive is there.
        reset_drive();
        sim.id_serial = "OTHER00002";
        sim.phase = SimDrive::IDLE; sim.status = 0x80; sim.ready_at = mock_now_ns + 1500000000ull;
        run(g, "busy 1.5 s during the check", 2, 0);
        mock_now_ns += 2000000000ull;
        run(g, "ready again, other serial", 0, 1);
    }
    // Rows the words do not gate never trigger the check (SAT data reads are
    // LBA mode only, so this part runs in LBA mode).
    reset_drive();
    if (m == LBA) {
        int ids = sim.id_commands();
        SatResult rd = sat_cmd(pt12(4, 0x0E, 0, 1, 100, 0xE0, 0x20), 512, true);
        CHECK(rd.ok() && sim.id_commands() == ids, "READ SECTORS: ok %d, %d IDENTIFY sent", rd.ok(), sim.id_commands() - ids);
    }
    CHECK(sim.violations == 0, "violations %d", sim.violations);
}

static void test_sat_identify_gate() {
    setup("SAT rows gated on the drive's IDENTIFY", LBA);
    check_gate("full IDENTIFY", G_ALL);

    // One feature missing at a time.
    identify_with(0x4400, 0x4400, 0x4001);  check_gate("no SMART (82.0)", G_NMAX | G_NMAX_EXT | G_READ_EXT);
    identify_with(0x4401, 0x4400, 0x4000);  check_gate("no SMART error log (84.0)", G_ALL & ~G_LOG);
    identify_with(0x4001, 0x4400, 0x4001);  check_gate("no HPA (82.10)", G_SMART | G_LOG | G_READ_EXT);
    identify_with(0x4401, 0x4000, 0x4001);  check_gate("no 48-bit (83.10)", G_SMART | G_LOG | G_NMAX);
    // SMART supported but switched off (word 85 bit 0), and word 85 not valid
    // (word 87 bits 15:14 not 01b): every SMART row refused, the rest run.
    identify_with(0x4401, 0x4400, 0x4001, 0x4400);          check_gate("SMART disabled (85.0)", G_NMAX | G_NMAX_EXT | G_READ_EXT);
    identify_with(0x4401, 0x4400, 0x4001, 0x4401, 0x0000);  check_gate("word 87 signature 00b", G_NMAX | G_NMAX_EXT | G_READ_EXT);
    identify_with(0x4401, 0x4400, 0x4001, 0x4401, 0xC000);  check_gate("word 87 signature 11b", G_NMAX | G_NMAX_EXT | G_READ_EXT);
    identify_with(0x4401, 0x4400, 0x4001, 0x4401, 0x8000);  check_gate("word 87 signature 10b", G_NMAX | G_NMAX_EXT | G_READ_EXT);
    identify_with(0x4401, 0x4400, 0x4001, 0xFFFF, 0xFFFF);  check_gate("words 85 and 87 FFFFh", G_NMAX | G_NMAX_EXT | G_READ_EXT);
    // The ST380011A donor's words 82 and 85 (346Bh, 3468h: SMART supported,
    // not enabled; HPA supported and enabled). Its words 83, 84 and 87 were not
    // recorded, so the simulated drive's valid ones stand in.
    identify_with(0x346B, 0x4400, 0x4001, 0x3468, 0x4000);  check_gate("ST380011A words 82 and 85", G_NMAX | G_NMAX_EXT | G_READ_EXT);
    // Words 82..84 not valid: nothing gated may run, whatever the bits say.
    identify_with(0x0401 | 0x0001, 0x0400, 0x0001);   check_gate("pre-ATA-4, no signature (CFS1275A-like)", 0);
    identify_with(0x4401, 0xC400, 0x4001);  check_gate("word 83 signature 11b", 0);
    identify_with(0x4401, 0x4400, 0x8001);  check_gate("word 84 signature 10b", 0);
    identify_with(0xFFFF, 0x4400, 0x4001);  check_gate("word 82 FFFFh", 0);
    identify_with(0x0000, 0x4400, 0x4001);  check_gate("word 82 0000h", 0);
    identify_with(0xFFFF, 0xFFFF, 0xFFFF);  check_gate("floating bus", 0);
    identify_with(0x4401, 0x4400, 0x4001);  check_gate("full again", G_ALL);

    // No IDENTIFY to trust: fail closed.
    sim.identify_ok = false;
    uint16_t id[256];
    CHECK(!ide_identify(id), "IDENTIFY should have failed");
    sim.identify_ok = true;
    // (check_gate also sends a pass-through IDENTIFY, which works now and
    // must not count: only the firmware's own IDENTIFY sets the words.)
    check_gate("after a failed IDENTIFY (forced manual geometry)", 0);
    identify_with(0x4401, 0x4400, 0x4001);  check_gate("IDENTIFY answers again", G_ALL);

    ide_probe_devices();                    // a new detection, before its IDENTIFY
    check_gate("after a probe, before IDENTIFY", 0);
    identify_with(0x4401, 0x4400, 0x4001);  check_gate("probe, then IDENTIFY", G_ALL);

    ide_select_device(0xB0);                // another device than the one identified
    check_gate("another device selected", 0, false);   // nothing on device 1 to read from
    ide_select_device(0xA0);
    check_gate("the identified device again", G_ALL);

    id_words.valid = false;                 // as at power-up: nothing captured yet
    check_gate("no IDENTIFY since power-up (auto-mount from saved config)", 0);
    identify_with(0x4401, 0x4400, 0x4001);

    // CHS mount: the gated rows that name no user sector still follow IDENTIFY.
    setup("SAT rows gated on the drive's IDENTIFY, CHS", CHS);
    for (auto &r : gated_rows()) {
        if (r.bit == G_READ_EXT) continue;          // refused in CHS mode anyway
        int cmds = sim.non_id_commands();
        sat_cmd(r.cdb, r.xfer, r.in);
        CHECK(sim.non_id_commands() == cmds + 1, "CHS, full IDENTIFY: %s not sent", r.name);
    }
    identify_with(0x0001, 0x0000, 0x0000, 0x0000, 0x0000);
    for (auto &r : gated_rows()) {
        int cmds = sim.non_id_commands();
        SatResult s = sat_cmd(r.cdb, r.xfer, r.in);
        CHECK(sim.non_id_commands() == cmds && sense_is(s.sense, 0x05, 0x24, 0x00), "CHS, pre-ATA-4: %s reached the drive", r.name);
    }
    CHECK(sim.violations == 0 && sim.srst == 0, "violations %d srst %d", sim.violations, sim.srst);
}

// ---- review finding M1: the SAT path must see the command start ----------
//
// Until a drive raises BSY, status reads as it did before the command. ATA
// allows 400 ns; this drive takes 5.4 us. The answers must be the drive's,
// never the registers the host just wrote read back (SMART "passed", native
// max 0, verify good), and a stale ERR must not end the next command. With
// INTRQ wired and without it (then only BSY tells).
static bool sat_data_is_medium(const SatResult &s, uint32_t lba) {
    for (size_t i = 0; i < s.data.size(); i++)
        if (s.data[i] != sim.byte_at(lba + (uint32_t)(i / 512), (int)(i % 512))) return false;
    return !s.data.empty();
}

static void test_sat_slow_bsy() {
    for (int wired = 1; wired >= 0; wired--) {
        std::string n = std::string("SAT, drive 5 us late raising BSY, INTRQ ") + (wired ? "wired" : "not wired");
        setup(n.c_str(), LBA);
        sim.intrq_wired = wired != 0;
        sim.t_bsy_delay = 5000;
        sim.nsect = 156250000; sim.native_max = 156301488;
        sim.smart_exceeded = true;                  // the host writes 4F/C2; the drive answers F4/2C
        // Each command below starts from an idle drive (settle()), so one that
        // was wrongly taken as finished cannot hide the next one's result.
        auto settle = [] { mock_now_ns += 5000000; };
        SatResult s = sat_cmd(smart_status12(), 0, false);
        CHECK(is_desc(s.sense) && sense_is(s.sense, 0x01, 0x00, 0x1D) && s.sense[17] == 0xF4 && s.sense[19] == 0x2C,
              "SMART RETURN STATUS: %02x/%02x (4F/C2 is what the host wrote)",
              is_desc(s.sense) ? s.sense[17] : 0, is_desc(s.sense) ? s.sense[19] : 0);
        settle();
        s = sat_cmd(native_max12(), 0, false);
        CHECK(is_desc(s.sense) && desc_lba28(s.sense) == 156301487u, "READ NATIVE MAX: %u",
              is_desc(s.sense) ? desc_lba28(s.sense) : 0u);
        settle();
        s = sat_cmd(native_max_ext16(), 0, false);
        CHECK(is_desc(s.sense) && desc_lba48(s.sense) == 156301487ull, "READ NATIVE MAX EXT: %llu",
              is_desc(s.sense) ? (unsigned long long)desc_lba48(s.sense) : 0ull);
        sim.bad[5000] = {BAD_ERR, 0};
        settle();
        s = sat_cmd(verify12(0x00, 8, 4996), 0, false);
        CHECK(!s.ok() && is_desc(s.sense) && sense_is(s.sense, 0x03, 0x11, 0x00) && desc_lba28(s.sense) == 5000,
              "verify over a bad sector: r %d", s.r);
        // The drive still shows ERR from that verify. Neither path may take
        // it as the next command's ending.
        mock_now_ns += 700000000;                   // the failed verify has ended (600 ms bad sector)
        s = sat_cmd(pt12(4, 0x0E, 0, 2, 100, 0xE0, 0x20), 1024, true);
        CHECK(s.r == 1024 && sat_data_is_medium(s, 100), "PIO read after an ERR: r %d", s.r);
        s = sat_cmd(verify12(0x00, 8, 4996), 0, false);
        CHECK(!s.ok(), "verify fails again");
        mock_now_ns += 700000000;
        s = sat_cmd(verify12(0x00, 4, 100), 0, false);
        CHECK(s.r == 0 && s.sense.empty(), "good verify after an ERR: r %d sense %zu", s.r, s.sense.size());
        // Review L-3: a gated command straight after an ERR. Its identity
        // check sends IDENTIFY, which must not take the stale ERR as its
        // answer (that would forget the words and refuse the command).
        s = sat_cmd(verify12(0x00, 8, 4996), 0, false);
        CHECK(!s.ok(), "verify fails again");
        mock_now_ns += 700000000;
        s = sat_cmd(native_max12(), 0, false);
        CHECK(is_desc(s.sense) && desc_lba28(s.sense) == 156301487u, "READ NATIVE MAX after an ERR: sense %zu %s", s.sense.size(),
              sense_is(s.sense, 0x05, 0x24, 0x00) ? "(refused: identity check took the stale ERR)" : "");
        CHECK(sim.srst == 0 && sim.violations == 0 && sim.non_id_data_reads() == 512, "srst %d violations %d data reads %d",
              sim.srst, sim.violations, sim.non_id_data_reads());
    }
}

// A drive that never starts the command (no BSY, no INTRQ, status as it was)
// must end in an abort with SRST, never in success; also with INTRQ stuck
// high, and on the PIO path with a stale ERR showing.
static void test_sat_never_starts() {
    setup("SAT, drive ignores the command", LBA);
    const uint8_t ignored[] = { 0xB0, 0xF8, 0x40 };
    const std::vector<uint8_t> cdbs[] = { smart_status12(), native_max12(), verify12(0x00, 1, 100) };
    for (int stuck = 0; stuck < 2; stuck++) {
        sim.intrq_stuck = stuck != 0;
        for (int i = 0; i < 3; i++) {
            sim.ignore_cmd = ignored[i];
            int srst0 = sim.srst;
            uint64_t t0 = mock_now_ns;
            SatResult s = sat_cmd(cdbs[i], 0, false);
            uint64_t ms = (mock_now_ns - t0) / 1000000;
            CHECK(!s.ok() && s.sense.size() == 18 && sense_is(s.sense, 0x0B, 0x00, 0x00) && sim.srst > srst0,
                  "INTRQ %s, %02X ignored: r %d sense %zu srst %d", stuck ? "stuck high" : "normal", ignored[i], s.r,
                  s.sense.size(), sim.srst - srst0);
            CHECK(ms >= 10000 && ms < 12000, "%02X: gave up after %llu ms", ignored[i], (unsigned long long)ms);
        }
    }
    sim.intrq_stuck = false;
    // PIO: a stale ERR, then a read the drive ignores
    sim.ignore_cmd = 0;
    sim.bad[5000] = {BAD_ERR, 0};
    sat_cmd(verify12(0x00, 1, 5000), 0, false);
    sim.ignore_cmd = 0x20;
    int srst0 = sim.srst;
    SatResult s = sat_cmd(pt12(4, 0x0E, 0, 1, 100, 0xE0, 0x20), 512, true);
    CHECK(!s.ok() && s.sense.size() == 18 && sense_is(s.sense, 0x0B, 0x00, 0x00) && sim.srst > srst0,
          "PIO, ignored after an ERR: r %d sense %zu (a descriptor here is the stale ERR)", s.r, s.sense.size());
    sim.ignore_cmd = 0;
    sim.bad.clear();
    HostRead h = host_read10(1000, 8);
    CHECK(h.ok && matches_medium(h, 1000), "usable after the aborts");
    CHECK(sim.violations == 0 && sim.non_id_data_reads() == 8 * 256, "violations %d data reads %d", sim.violations, sim.non_id_data_reads());
}

// A drive so quick the command is over before the first poll: BSY is never
// seen, but INTRQ says it ended, so the answer is taken. Without INTRQ the
// same command cannot be told from one never started: aborted, not GOOD.
static void test_sat_fast_command() {
    setup("SAT, command over before the first poll", LBA);
    sim.t_nondata = 0; sim.t_sector = 0;
    sim.smart_exceeded = true;
    SatResult s = sat_cmd(smart_status12(), 0, false);
    CHECK(is_desc(s.sense) && sense_is(s.sense, 0x01, 0x00, 0x1D) && s.sense[17] == 0xF4 && s.sense[19] == 0x2C,
          "fast SMART RETURN STATUS: size %zu", s.sense.size());
    sim.nsect = 156250000; sim.native_max = 156301488;
    s = sat_cmd(native_max12(), 0, false);
    CHECK(is_desc(s.sense) && desc_lba28(s.sense) == 156301487u, "fast READ NATIVE MAX");
    s = sat_cmd(pt12(4, 0x0E, 0, 4, 100, 0xE0, 0x20), 2048, true);
    CHECK(s.r == 2048 && sat_data_is_medium(s, 100), "fast PIO read: r %d", s.r);
    CHECK(sim.srst == 0, "srst %d", sim.srst);
    // A bad sector that fails at once with its data on offer (ERR+DRQ), no
    // BSY seen and no INTRQ: the DRQ shows the command started, so the ERR
    // is this command's, and the flawed block is never passed on as data.
    sim.intrq_wired = false; sim.t_bad = 0;
    sim.bad[3000] = {BAD_ERR_DRQ, 0};
    s = sat_cmd(pt12(4, 0x0E, 0, 2, 3000, 0xE0, 0x20), 1024, true);
    CHECK(!s.ok() && s.data.empty() && is_desc(s.sense) && sense_is(s.sense, 0x03, 0x11, 0x00),
          "instant ERR+DRQ: r %d sense %zu", s.r, s.sense.size());
    if (is_desc(s.sense)) CHECK(desc_lba28(s.sense) == 3000 && s.sense[21] == 0x59, "LBA %u status %02x",
                                desc_lba28(s.sense), s.sense[21]);
    sim.bad.clear();
    int srst0 = sim.srst;
    s = sat_cmd(smart_status12(), 0, false);
    CHECK(!s.ok() && s.sense.size() == 18 && sense_is(s.sense, 0x0B, 0x00, 0x00) && sim.srst > srst0,
          "fast, INTRQ not wired: must abort, r %d sense %zu", s.r, s.sense.size());
    CHECK(sim.violations == 0, "violations %d", sim.violations);
}

// ---- review finding M2: DF (device fault) ---------------------------------
// A command that ends with DF set and ERR clear is a failure: HARDWARE ERROR
// 4/44/00 with the drive's registers, on the non-data path and on the PIO
// path both after the data and instead of it.
static void test_sat_device_fault() {
    setup("SAT, device fault (DF)", LBA);
    sim.df_cmd = 0xB0;
    SatResult s = sat_cmd(smart_status12(), 0, false);
    CHECK(!s.ok() && is_desc(s.sense) && sense_is(s.sense, 0x04, 0x44, 0x00), "SMART RETURN STATUS with DF: r %d", s.r);
    if (is_desc(s.sense)) CHECK(s.sense[21] == 0x70 && s.sense[11] == 0x00, "status %02x error %02x", s.sense[21], s.sense[11]);
    sim.df_cmd = 0x40;
    s = sat_cmd(verify12(0x00, 4, 100), 0, false);
    CHECK(!s.ok() && is_desc(s.sense) && sense_is(s.sense, 0x04, 0x44, 0x00), "READ VERIFY with DF: r %d", s.r);
    sim.df_cmd = 0xB0;                                   // after the one block of SMART READ DATA
    s = sat_cmd(pt12(4, 0x0E, 0xD0, 1, 0xC24F00, 0xA0, 0xB0), 512, true);
    CHECK(!s.ok() && s.data.empty() && is_desc(s.sense) && sense_is(s.sense, 0x04, 0x44, 0x00),
          "SMART READ DATA ending with DF: r %d", s.r);
    sim.df_cmd = 0x20;                                   // after the last of 3 blocks
    s = sat_cmd(pt12(4, 0x0E, 0, 3, 100, 0xE0, 0x20), 3 * 512, true);
    CHECK(!s.ok() && s.data.empty() && is_desc(s.sense) && sense_is(s.sense, 0x04, 0x44, 0x00),
          "READ SECTORS ending with DF: r %d", s.r);
    sim.df_before_data = true;                           // instead of the first block
    uint64_t t0 = mock_now_ns;
    s = sat_cmd(pt12(4, 0x0E, 0, 3, 100, 0xE0, 0x20), 3 * 512, true);
    CHECK(!s.ok() && s.data.empty() && is_desc(s.sense) && sense_is(s.sense, 0x04, 0x44, 0x00),
          "READ SECTORS with DF before data: r %d", s.r);
    CHECK(mock_now_ns - t0 < 1000000000ull, "DF before data waited %llu ms (taken as a timeout)",
          (unsigned long long)((mock_now_ns - t0) / 1000000));
    sim.df_cmd = 0; sim.df_before_data = false;
    CHECK(sim.srst == 0 && sim.violations == 0, "srst %d violations %d", sim.srst, sim.violations);
    s = sat_cmd(pt12(4, 0x0E, 0, 3, 100, 0xE0, 0x20), 3 * 512, true);
    CHECK(s.r == 3 * 512 && sat_data_is_medium(s, 100), "read after DF: r %d", s.r);
}

// Mutating and out-of-list commands are refused and reach nothing.
static void test_sat_refusals_touch_nothing() {
    setup("SAT refusals touch nothing", LBA);
    struct { const char *name; std::vector<uint8_t> cdb; uint32_t xfer; bool in; } list[] = {
        { "SET MAX ADDRESS",          pt12(3, 0x20, 0, 0, 0, 0x40, 0xF9), 0, false },
        { "SET MAX ADDRESS EXT",      pt16(3, true, 0x20, 0, 0, 0, 0x40, 0x37), 0, false },
        { "DCO IDENTIFY",             pt12(4, 0x0E, 0xC2, 1, 0, 0xA0, 0xB1), 512, true },
        { "DCO FREEZE LOCK",          pt12(3, 0x20, 0xC1, 0, 0, 0xA0, 0xB1), 0, false },
        { "SMART ENABLE",             pt12(3, 0x20, 0xD8, 0, 0xC24F00, 0xA0, 0xB0), 0, false },
        { "SMART EXECUTE OFF-LINE",   pt12(3, 0x20, 0xD4, 0, 0xC24F01, 0xA0, 0xB0), 0, false },
        { "SECURITY FREEZE LOCK",     pt12(3, 0x20, 0, 0, 0, 0xA0, 0xF5), 0, false },
        { "WRITE SECTORS as non-data",pt12(3, 0x20, 0, 1, 0, 0xE0, 0x30), 0, false },
        { "READ NATIVE MAX, CK_COND=0", pt12(3, 0x00, 0, 0, 0, 0x40, 0xF8), 0, false },
        { "READ VERIFY, data-in CBW", verify12(0x20, 1, 0), 512, true },
    };
    for (auto &e : list) {
        int cmds = sim.commands;
        SatResult s = sat_cmd(e.cdb, e.xfer, e.in);
        CHECK(!s.ok() && s.sense.size() == 18 && sense_is(s.sense, 0x05, 0x24, 0x00) && sim.commands == cmds,
              "%s: r %d commands %d", e.name, s.r, sim.commands - cmds);
    }
    CHECK(sim.data_reads == 0 && sim.violations == 0, "data reads %d", sim.data_reads);
}
#else
static const char *mode_name(Mode m) { return m == LBA ? "LBA" : "CHS"; }

// With SAT built out, both pass-through opcodes are unknown commands and
// nothing reaches the drive.
static void test_no_sat() {
    setup("SAT off: pass-through refused", LBA);
    int cmds = sim.commands;
    SatResult a = sat_cmd(smart_status12(), 0, false);
    SatResult b = sat_cmd(pt12(4, 0x0E, 0, 1, 0, 0, 0xEC), 512, true);
    SatResult c = sat_cmd(native_max_ext16(), 0, false);
    CHECK(sense_is(a.sense, 0x05, 0x20, 0x00) && sense_is(b.sense, 0x05, 0x20, 0x00) &&
          sense_is(c.sense, 0x05, 0x20, 0x00), "sense sizes %zu %zu %zu", a.sense.size(), b.sense.size(), c.sense.size());
    CHECK(sim.commands == cmds && a.sense.size() == 18, "commands %d", sim.commands - cmds);
    (void)mode_name(LBA); (void)is_desc; (void)desc_lba28; (void)desc_lba48; (void)verify12; (void)native_max12;
}
#endif

// ---- review M-1: one time budget per host command -------------------------
//
// The first 0.6f3p7 draft gave every ATA command 30 s, the issue #13 call
// again at the failing sector its own 30 s, and every reset its own 31 s,
// inside one host command. The reviewer measured, on this simulator: a hang
// at the fourth sector of an 8-sector READ(10) kept core 0 in the callbacks
// for 60 s with two soft resets (0.6f3p6: 2 s); with SRST not bringing the
// drive back, 123 s and two hardware resets; a lone slave that comes back
// from neither, 92 s in one callback. A host gives up after 30 s and resets
// the device under the callback.
//
// Below, every failure the review listed goes through the USB callbacks as a
// host would send it, and the helpers fail any command that keeps core 0 past
// the budget (note_cmd). A host that is told a command failed may send it
// again: host_read_retry() does, up to `tries` commands in all, as Linux's sd
// driver retries a failed command. What is checked besides the time: the
// good prefix still reaches the host, a pending recovery sends nothing new to
// the drive, RESET- is used at most once per recovery, a WRITE is never sent
// to the drive a second time by the firmware, and the drive works afterwards.

struct Timing { std::string name; uint64_t cb_ms, cmd_ms; };
static std::vector<Timing> timings;
static void keep_timing(const std::string &name) {
    timings.push_back({ name, worst_cb_ns / 1000000, worst_cmd_ns / 1000000 });
}

// Sends READ(10) until it succeeds or `tries` commands have been sent.
// Returns the command that succeeded (1-based), or 0.
static int host_read_retry(uint32_t lba, uint32_t n, int tries, HostRead *out = nullptr) {
    for (int i = 1; i <= tries; i++) {
        HostRead h = host_read10(lba, n);
        if (out) *out = h;
        if (h.ok) return i;
    }
    return 0;
}

// Commands the drive got other than those of a recovery (RECALIBRATE,
// INITIALIZE DEVICE PARAMETERS) and IDENTIFY.
static int work_commands() {
    int n = sim.commands;
    for (uint8_t c : { 0x10, 0x91, 0xEC }) { auto it = sim.cmd_count.find(c); if (it != sim.cmd_count.end()) n -= it->second; }
    return n;
}

// A hang at every position of an 8-sector READ(10), the drive back from the
// soft reset: the command fails inside the budget with the good prefix, and
// the issue #13 call at the failing sector, which has no time left to send a
// command and still reset the drive, sends nothing (one READ reaches the
// drive, not two). The drive works afterwards.
static void test_budget_read_hang(Mode m) {
    for (uint32_t k = 0; k < 8; k++) {
        std::string n = std::string("budget: hang at sector ") + std::to_string(k) + " of 8, " + lba_or_chs(m);
        setup(n.c_str(), m);
        sim.bad[5000 + k] = {BAD_HANG, 0};
        int reads = sim.cmd_count[0x20];
        HostRead h = host_read10(5000, 8);
        CHECK(!h.ok && h.data.size() == k * 512 && matches_medium(h, 5000), "ok %d prefix %zu", h.ok, h.data.size());
        CHECK(sim.srst == 1 && sim.hw_resets == 0, "srst %d hw %d", sim.srst, sim.hw_resets);
        CHECK(sim.cmd_count[0x20] == reads + 1, "%d READs sent (the call at the failing sector had no time)",
              sim.cmd_count[0x20] - reads);
        ide_fail_t f; ide_last_failure(&f);
        CHECK(f.kind == IDE_FAIL_TIMEOUT && f.lba == 5000 + k && f.reset && !f.pending && !f.reset_failed,
              "record: kind %u lba %u reset %d pending %d", f.kind, f.lba, f.reset, f.pending);
        if (k == 3) keep_timing(std::string("hang at sector 3 of 8, SRST works (review A), ") + lba_or_chs(m));
        sim.bad.clear();
        HostRead g = host_read10(5000, 8);
        CHECK(g.ok && matches_medium(g, 5000), "usable afterwards");
        CHECK(sim.violations == 0, "violations %d", sim.violations);
    }
    // A single-sector READ(10) that hangs.
    setup((std::string("budget: single-sector read hangs, ") + lba_or_chs(m)).c_str(), m);
    sim.bad[777] = {BAD_HANG, 0};
    HostRead h = host_read10(777, 1);
    CHECK(!h.ok && h.data.empty() && sim.srst == 1, "ok %d srst %d", h.ok, sim.srst);
    keep_timing(std::string("single-sector read hangs, ") + lba_or_chs(m));
}

// WRITE(10): the drive busy past the limit committing a sector (the
// ST380011A finding), and one that never asks for the next sector. The
// command fails inside the budget, the write reaches the drive once, and
// the next write works.
static void test_budget_write_hang(Mode m) {
    setup((std::string("budget: write hangs, ") + lba_or_chs(m)).c_str(), m);
    config.drive_write_protected = false;
    const uint32_t L = 23456;
    std::vector<uint8_t> w = fill512(8, 7);
    sim.pause[L] = 45000000000ull;                  // commit of the only sector
    int32_t r = host_write10(L, w.data(), 1);
    CHECK(r < 0 && written_is(L, w.data()) && sim.cmd_count[0x30] == 1, "commit hang: r %d, %d WRITEs", r, sim.cmd_count[0x30]);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(f.kind == IDE_FAIL_TIMEOUT && f.command == 0x30 && f.reset && !f.pending, "record: kind %u cmd %02X", f.kind, f.command);
    sim.pause.clear();
    sim.pause[L + 102] = 45000000000ull;            // after the third of 8: no DRQ for the fourth
    r = host_write10(L + 100, w.data(), 8);
    CHECK(r < 0 && sim.cmd_count[0x30] == 2, "DRQ hang: r %d, %d WRITEs", r, sim.cmd_count[0x30]);
    keep_timing(std::string("write hangs, SRST works, ") + lba_or_chs(m));
    sim.pause.clear();
    r = host_write10(L + 100, w.data(), 8);
    bool all = true;
    for (int i = 0; i < 8; i++) all = all && written_is(L + 100 + i, w.data() + i * 512);
    CHECK(r == 4096 && all && sim.violations == 0, "write afterwards: r %d content %d", r, all);
}

// The reviewer's B: a hang at the fourth of 8 sectors, and the drive stays
// busy through the soft reset; RESET- brings it back. The first command
// fails with the prefix and the recovery pending; the host's next command
// waits out more of ATA's 31 s and fails too, sending nothing; the one after
// sends RESET- (once), gets the drive back, and reads. Meanwhile no command
// of any kind is sent to the drive: a SAT command and a WRITE(10) are
// refused without reaching it.
static void test_budget_srst_fails(Mode m) {
    setup((std::string("budget: SRST does not bring the drive back, ") + lba_or_chs(m)).c_str(), m);
    if (m == CHS) chs_user_geometry();
    config.drive_write_protected = false;
    sim.srst_wedges = true;
    sim.bad[5003] = {BAD_HANG, 0};
    HostRead h = host_read10(5000, 8);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(!h.ok && h.data.size() == 3 * 512 && matches_medium(h, 5000), "prefix %zu", h.data.size());
    CHECK(f.kind == IDE_FAIL_TIMEOUT && f.reset && f.pending && !f.hw_reset, "record: kind %u pending %d hw %d",
          f.kind, f.pending, f.hw_reset);
    sim.bad.clear();
    // The next command waits out 20 s more of the 31 s and is refused, with
    // nothing sent to the drive.
    int work = work_commands();
#if ATABOY_SAT
    // (SAT reads are LBA mode only; in CHS mode READ NATIVE MAX stands in.)
    SatResult s = m == LBA ? sat_cmd(pt12(4, 0x0E, 0, 1, 100, 0xE0, 0x20), 512, true) : sat_cmd(native_max12(), 0, false);
    CHECK(!s.ok() && sense_is(s.sense, 0x02, 0x04, 0x00), "SAT while the recovery is pending: r %d", s.r);
#else
    HostRead p = host_read10(100, 1);
    CHECK(!p.ok, "READ(10) while the recovery is pending");
#endif
    CHECK(work_commands() == work && sim.hw_resets == 0, "%d commands sent while the recovery was pending", work_commands() - work);
    // The one after sends RESET-, gets the drive back, and then does its own
    // work: a WRITE, sent once.
    std::vector<uint8_t> w = fill512(1, 9);
    int32_t r = host_write10(9000, w.data(), 1);
    CHECK(r == 512 && written_is(9000, w.data()) && sim.cmd_count[0x30] == 1, "write after the recovery: r %d", r);
    int got = host_read_retry(5000, 8, 5, &h);
    CHECK(got == 1 && h.ok && matches_medium(h, 5000), "read after the recovery: try %d", got);
    CHECK(sim.hw_resets == 1, "%d hardware resets for one recovery", sim.hw_resets);
    ide_last_failure(&f);
    CHECK(f.hw_reset && !f.pending && !f.reset_failed, "record: hw %d pending %d failed %d", f.hw_reset, f.pending, f.reset_failed);
    if (m == CHS) CHECK(sim.geo_valid && sim.heads == 5 && sim.spt == 34, "geometry %d %u x %u", sim.geo_valid, sim.heads, sim.spt);
    CHECK(sim.violations == 0 && sim.reset_writes == 0, "violations %d reset writes %d", sim.violations, sim.reset_writes);
    keep_timing(std::string("hang at sector 3, SRST fails, RESET- works (review B), ") + lba_or_chs(m));
}

// The fixed margin: RESET- is due 10 ms before the host command's time runs
// out (the SRST's 31 s end then). The 52 ms of the pulse and its settle time
// go past the budget; that is what HOST_MARGIN_MS allows, and no more.
static void test_budget_reset_at_the_end() {
    setup("budget: RESET- due in the last 10 ms of a command", LBA);
    sim.srst_wedges = true;
    sim.bad[5003] = {BAD_HANG, 0};
    HostRead h = host_read10(5000, 8);              // SRST at about 15 s, pending at 20 s
    sim.bad.clear();
    uint64_t srst = sim.srst_at;
    // The host's next command comes when the SRST's 31 s will end 10 ms
    // before that command's 20 s do.
    mock_now_ns = srst + (IDE_SRST_TIMEOUT_MS - IDE_HOST_BUDGET_MS + 10) * 1000000ull;
    reset_worst();
    h = host_read10(5000, 8);
    CHECK(sim.hw_resets == 1 && !h.ok, "hw resets %d ok %d", sim.hw_resets, h.ok);
    CHECK(worst_cmd_ns > IDE_HOST_BUDGET_MS * 1000000ull, "the pulse was not past the budget (%llu ms): the test missed the edge",
          (unsigned long long)(worst_cmd_ns / 1000000));
    keep_timing("RESET- due in the last 10 ms of a command");
    int got = host_read_retry(5000, 8, 3, &h);
    CHECK(got == 1 && h.ok && matches_medium(h, 5000) && sim.hw_resets == 1, "afterwards: try %d hw %d", got, sim.hw_resets);
}

// The reviewer's C: a lone slave that comes back from neither reset, IORDY
// enabled. Every command fails inside the budget, RESET- is used once, IORDY
// stays ignored while the drive holds it low (review L-3), and once the
// recovery is over the record says it failed and later commands are refused
// as not ready without another reset. When the drive does come back, it
// works, and IORDY is believed again.
static void test_budget_lone_slave_wedged() {
    setup("budget: lone slave, back from neither reset", LBA);
    sim.slave = true;
    ide_select_device(0xB0); ide_write_reg(6, 0xB0); busy_wait_us_32(1);
    uint16_t id[256];
    CHECK(ide_identify(id), "slave IDENTIFY");
    config.iordy_enabled = true; ide_set_iordy(true);
    sim.srst_wedges = true; sim.hw_reset_wedges = true;
    sim.bad[5000] = {BAD_HANG, 0};
    int reads = sim.cmd_count[0x20];
    int got = host_read_retry(5000, 1, 6);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(got == 0 && sim.hw_resets == 1 && sim.srst == 1, "try %d, hw resets %d, srst %d", got, sim.hw_resets, sim.srst);
    CHECK(sim.cmd_count[0x20] == reads + 1, "%d READs sent to a drive that never came back", sim.cmd_count[0x20] - reads);
    CHECK(f.kind == IDE_FAIL_NOT_READY || (f.hw_reset && f.reset_failed), "record: kind %u hw %d failed %d", f.kind, f.hw_reset, f.reset_failed);
    CHECK(sim.iordy_stalls == 0 && mock_iordy_inover == GPIO_OVERRIDE_HIGH, "IORDY: %d stalls, override %u",
          sim.iordy_stalls, mock_iordy_inover);
    keep_timing("lone slave, back from neither reset (review C)");
    // Six more commands: still nothing but not ready, and no second RESET-.
    got = host_read_retry(100, 8, 6);
    ide_last_failure(&f);
    CHECK(got == 0 && sim.hw_resets == 1 && f.kind == IDE_FAIL_NOT_READY, "later: try %d hw %d kind %u", got, sim.hw_resets, f.kind);
    // The drive comes back by itself.
    sim.srst_wedges = false; sim.hw_reset_wedges = false; sim.wedged = false; sim.bad.clear();
    HostRead h;
    got = host_read_retry(100, 8, 2, &h);
    CHECK(got == 1 && h.ok && matches_medium(h, 100), "once the drive is back: try %d", got);
    CHECK(mock_iordy_inover == GPIO_OVERRIDE_NORMAL && sim.iordy_stalls == 0, "IORDY not believed again, or stalled (%d)", sim.iordy_stalls);
    CHECK(sim.violations == 0, "violations %d", sim.violations);
    config.iordy_enabled = false;
}

// Data left over from an earlier command, and the soft reset that should
// clear it does not bring the drive back: the stale block is never handed
// over, the recovery spans host commands, and RESET- is used once.
static void test_budget_stale_drq() {
    setup("budget: stale DRQ, SRST does not bring the drive back", LBA);
    for (int i = 0; i < 256; i++) sim.xfer[i] = 0xDEAD;
    sim.widx = 0; sim.phase = SimDrive::DRQ_IN; sim.status = 0x58; sim.left = 1;
    sim.srst_wedges = true;
    HostRead h;
    int got = host_read_retry(300, 4, 5, &h);
    CHECK(got >= 2 && h.ok && matches_medium(h, 300) && sim.hw_resets == 1, "try %d, hw resets %d", got, sim.hw_resets);
    CHECK(sim.violations == 0, "violations %d", sim.violations);
    keep_timing("stale DRQ, SRST fails, RESET- works");
}

#if ATABOY_SAT
// SAT: a non-data command the drive never ends, and a PIO read that never
// gives data. Aborted inside the budget, recorded (review L-2), and the next
// command works.
static void test_budget_sat_timeout() {
    setup("budget: SAT commands time out", LBA);
    sim.hang_cmd = 0xB0;
    SatResult s = sat_cmd(smart_status12(), 0, false);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(!s.ok() && sense_is(s.sense, 0x0B, 0x00, 0x00) && sim.srst == 1, "r %d srst %d", s.r, sim.srst);
    CHECK(f.kind == IDE_FAIL_TIMEOUT && f.command == 0xB0 && f.reset && !f.pending, "record: kind %u cmd %02X", f.kind, f.command);
    sim.hang_cmd = 0x20;
    s = sat_cmd(pt12(4, 0x0E, 0, 2, 0x1234, 0xE0, 0x20), 1024, true);
    ide_last_failure(&f);
    CHECK(!s.ok() && s.data.empty() && f.command == 0x20 && f.lba == 0x1234 && f.count == 2,
          "PIO: r %d cmd %02X lba %x count %u", s.r, f.command, f.lba, f.count);
    sim.hang_cmd = 0;
    keep_timing("SAT non-data and PIO commands time out, SRST works");
    s = sat_cmd(smart_status12(), 0, false);
    CHECK(is_desc(s.sense) && sense_is(s.sense, 0x01, 0x00, 0x1D), "SAT afterwards");
    CHECK(sim.violations == 0, "violations %d", sim.violations);
}

// The identity check's IDENTIFY gets no answer: aborted after its 10 s with
// a reset and recorded, the gated command refused and never sent, all inside
// the budget; nothing is left stranded for the next READ(10).
static void test_budget_identity_timeout() {
    setup("budget: identity check IDENTIFY times out", LBA);
    sim.t_identify_extra = 60000000000ull;
    int b0 = sim.cmd_count[0xB0];
    SatResult s = sat_cmd(smart_status12(), 0, false);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(!s.ok() && sense_is(s.sense, 0x05, 0x24, 0x00) && sim.cmd_count[0xB0] == b0, "r %d, SMART sent %d times",
          s.r, sim.cmd_count[0xB0] - b0);
    CHECK(f.kind == IDE_FAIL_TIMEOUT && f.command == 0xEC && f.reset && sim.srst == 1, "record: kind %u cmd %02X srst %d",
          f.kind, f.command, sim.srst);
    keep_timing("identity check IDENTIFY times out");
    sim.t_identify_extra = 0;
    HostRead h = host_read10(1000, 8);
    CHECK(h.ok && matches_medium(h, 1000) && sim.violations == 0, "READ(10) afterwards: ok %d", h.ok);
}
#endif

// usb.c's command boundaries. The chunks of one READ(10) or WRITE(10) share
// one budget; the next command, after TinyUSB's complete callback, has its
// own; so does one after a USB reset (tud_mount_cb) that abandoned the last
// command mid-way, and one after another SCSI command's complete callback.
// (Since the review of 0.6f3p7 a second chunk with 3 s left is not sent at
// all, rather than sent and reset: test_min_window_*.)
static void test_budget_boundaries() {
    setup("budget: the chunks of one command share it", LBA);
    config.drive_write_protected = false;
    const uint32_t L = 40000;
    sim.pause[L + 2] = 12000000000ull;              // first chunk
    sim.pause[L + 10] = 12000000000ull;             // second chunk: 3 s left for it
    HostRead h = host_read10(L, 16);
    CHECK(!h.ok && h.data.size() == 8 * 512 && matches_medium(h, L) && sim.cmd_count[0x20] == 1 && sim.srst == 0,
          "READ(10) of 16: ok %d size %zu, %d READs, srst %d", h.ok, h.data.size(), sim.cmd_count[0x20], sim.srst);
    sim.pause.clear();
    std::vector<uint8_t> w = fill512(16, 3);
    sim.pause[L + 102] = 12000000000ull;            // WRITE: the commit of sector 2, then sector 10
    sim.pause[L + 110] = 12000000000ull;
    int32_t r = host_write10(L + 100, w.data(), 16);
    CHECK(r < 0 && sim.cmd_count[0x30] == 1 && sim.srst == 0, "WRITE(10) of 16: r %d, %d WRITEs, srst %d", r,
          sim.cmd_count[0x30], sim.srst);
    sim.pause.clear();

    // Two WRITE(10)s one after the other, each one chunk with a 12 s commit:
    // TinyUSB's complete callback ends the first, so the second, which starts
    // where the first ended, has its own time (review of 0.6f3p7, X10).
    setup("budget: a WRITE(10) complete callback ends the command", LBA);
    config.drive_write_protected = false;
    sim.pause[L + 7] = 12000000000ull;
    sim.pause[L + 15] = 12000000000ull;
    int32_t w1 = host_write10(L, w.data(), 8);
    int32_t w2 = host_write10(L + 8, w.data() + 8 * 512, 8);
    CHECK(w1 == 4096 && w2 == 4096 && sim.srst == 0, "two 12 s WRITE(10)s in a row: %d %d", w1, w2);

    setup("budget: each command has its own", LBA);
    sim.pause[L + 1] = 12000000000ull;
    sim.pause[L + 9] = 12000000000ull;
    HostRead a = host_read10(L, 8);
    HostRead b = host_read10(L + 8, 8);             // starts where the last one ended
    CHECK(a.ok && b.ok && matches_medium(b, L + 8), "two 12 s commands in a row: %d %d", a.ok, b.ok);

    setup("budget: a USB reset ends an abandoned command", LBA);
    static uint8_t buf[CFG_TUD_MSC_EP_BUFSIZE];
    sim.pause[L + 1] = 12000000000ull;
    sim.pause[L + 9] = 12000000000ull;
    int32_t r1 = tud_msc_read10_cb(0, L, 0, buf, 4096);    // first chunk of 16, then the host resets
    tud_mount_cb();                                         // SET_CONFIGURATION after the USB reset
    HostRead c = host_read10(L + 8, 8);
    CHECK(r1 == 4096 && c.ok, "after the reset: first %d, new command ok %d", r1, c.ok);
    // A bulk-only reset has no callback: a READ(10) that starts exactly where
    // the abandoned one stopped inherits what is left of its budget (usb.c).
    // It may fail for that, but never outlasts it.
    setup("budget: after a bulk-only reset (no callback)", LBA);
    sim.pause[L + 1] = 12000000000ull;
    sim.pause[L + 9] = 12000000000ull;
    uint64_t t0 = mock_now_ns;
    r1 = tud_msc_read10_cb(0, L, 0, buf, 4096);
    HostRead d = host_read10(L + 8, 8);
    CHECK(r1 == 4096 && mock_now_ns - t0 <= HOST_LIMIT_NS, "abandoned and next: %llu ms", (unsigned long long)ms_since(t0));
    (void)d;
    // ...but only one that starts exactly there, in the same direction. A
    // WRITE(10) there, or a READ(10) anywhere else, is a new command.
    setup("budget: after a bulk-only reset, a new command elsewhere", LBA);
    config.drive_write_protected = false;
    sim.pause[L + 1] = 12000000000ull;
    sim.pause[L + 8] = 12000000000ull;              // the commit of the write below
    sim.pause[L + 101] = 12000000000ull;
    sim.pause[L + 21] = 12000000000ull;             // the second abandoned READ(10) below
    r1 = tud_msc_read10_cb(0, L, 0, buf, 4096);
    std::vector<uint8_t> w8 = fill512(8, 4);
    int32_t wr = host_write10(L + 8, w8.data(), 8);
    CHECK(r1 == 4096 && wr == 4096, "WRITE(10) where an abandoned READ(10) stopped: r %d", wr);
    r1 = tud_msc_read10_cb(0, L + 20, 0, buf, 4096);
    HostRead e = host_read10(L + 100, 8);
    CHECK(r1 == 4096 && e.ok, "READ(10) elsewhere after an abandoned one: ok %d", e.ok);
    // A READ(10) abandoned by a bulk-only reset, then another SCSI command
    // (TEST UNIT READY, as a host sends after a reset): its complete callback
    // ends the abandoned command, so a READ(10) that starts exactly where
    // that one stopped has its own time (review of 0.6f3p7, X11).
    setup("budget: another SCSI command ends an abandoned READ(10)", LBA);
    sim.pause[L + 1] = 12000000000ull;
    sim.pause[L + 9] = 12000000000ull;
    r1 = tud_msc_read10_cb(0, L, 0, buf, 4096);
    uint8_t tur[16] = {0};
    int32_t tr = tud_msc_scsi_cb(0, tur, buf, 0);
    tud_msc_scsi_complete_cb(0, tur);
    HostRead g = host_read10(L + 8, 8);
    CHECK(r1 == 4096 && tr == 0 && g.ok && matches_medium(g, L + 8) && sim.srst == 0,
          "READ(10) after TEST UNIT READY: ok %d srst %d", g.ok, sim.srst);
}

// A detection (the probe pulses RESET- itself) ends a recovery in progress:
// the record no longer says pending, and the next command does not wait on it.
static void test_redetect_ends_recovery() {
    setup("a re-detect ends a pending recovery", LBA);
    sim.srst_wedges = true;
    sim.bad[5003] = {BAD_HANG, 0};
    HostRead h = host_read10(5000, 8);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(!h.ok && f.pending, "pending %d", f.pending);
    sim.bad.clear();
    is_mounted = false;                             // Auto Detect runs unmounted, on core 1
    CHECK(ide_probe_devices() == 0xA0, "probe");
    ide_last_failure(&f);
    CHECK(!f.pending && f.kind == IDE_FAIL_TIMEOUT, "after the probe: pending %d kind %u", f.pending, f.kind);
    is_mounted = true;
    uint64_t t0 = mock_now_ns;
    h = host_read10(5000, 8);
    CHECK(h.ok && matches_medium(h, 5000) && ms_since(t0) < 100 && sim.hw_resets == 1,
          "read after the probe: ok %d after %llu ms, RESET- %d (the probe's)", h.ok, (unsigned long long)ms_since(t0), sim.hw_resets);
}

// Review L-2: the reset flags cover the host command. The hardware reset
// here is sent by the command that finds a recovery pending; the read it
// then sends fails with a plain ERR (no reset of its own), and so does the
// issue #13 call at the failing sector. The record still says RESET- was
// used in this command. A later command's failure does not carry it.
static void test_budget_record_sticky() {
    setup("budget: reset flags cover the host command", LBA);
    sim.srst_wedges = true;
    sim.bad[5003] = {BAD_HANG, 0};
    HostRead h = host_read10(5000, 8);              // SRST, pending
    sim.bad.clear();
    h = host_read10(5000, 8);                       // most of the 31 s, still pending
    CHECK(!h.ok && sim.hw_resets == 0, "second command: ok %d hw %d", h.ok, sim.hw_resets);
    // The host comes back 1 s before the SRST's 31 s are over, so that after
    // the hardware reset its READ has a fair window (test_min_window_*).
    mock_now_ns = sim.srst_at + (IDE_SRST_TIMEOUT_MS - 1000) * 1000000ull;
    sim.bad[6003] = {BAD_ERR, 0};
    h = host_read10(6000, 8);                       // RESET-, back, then ERR at 6003 twice
    ide_fail_t f; ide_last_failure(&f);
    CHECK(!h.ok && h.data.size() == 3 * 512 && sim.hw_resets == 1 && sim.attempts[6003] == 2,
          "ok %d size %zu hw %d attempts %d", h.ok, h.data.size(), sim.hw_resets, sim.attempts[6003]);
    CHECK(f.kind == IDE_FAIL_ERR && f.lba == 6003 && f.count == 5 && f.reset && f.hw_reset && !f.reset_failed,
          "record of the call at the failing sector: kind %u lba %u count %u reset %d hw %d", f.kind, f.lba, f.count, f.reset, f.hw_reset);
    h = host_read10(6003, 1);
    ide_last_failure(&f);
    CHECK(!h.ok && f.kind == IDE_FAIL_ERR && !f.reset && !f.hw_reset, "next command: reset %d hw %d", f.reset, f.hw_reset);
}

// Review R7: the time is up at exactly the limit, not a millisecond later.
static void test_deadline_boundary() {
    setup("deadline boundary: a read gives up at exactly IDE_CMD_TIMEOUT_MS", LBA);
    mock_now_ns = (mock_now_ns / 1000000 + 1) * 1000000;    // on a millisecond
    sim.pause[900] = 45000000000ull;
    uint8_t buf[512]; uint32_t done = 0;
    int32_t r = ide_read_sectors_partial(900, 1, buf, &done);
    uint64_t gave_up = sim.srst_at - sim.cmd_at;
    CHECK(r < 0 && sim.srst == 1, "r %d srst %d", r, sim.srst);
    CHECK(gave_up >= (IDE_CMD_TIMEOUT_MS - 1) * 1000000ull && gave_up < IDE_CMD_TIMEOUT_MS * 1000000ull + 500000,
          "gave up %llu us after the command, want %u ms", (unsigned long long)(gave_up / 1000), (unsigned)IDE_CMD_TIMEOUT_MS);
}

// Review L-4 (and R3): RECALIBRATE after the hardware reset must have ended
// before INITIALIZE DEVICE PARAMETERS goes out. A drive that never ends it:
// no 0x91 over the busy drive, and the recovery is recorded as failed. A
// drive slow to raise BSY for it, with no INTRQ: its idle status before it
// started is not taken as the end. Called directly, so each recovery runs to
// its end in one call.
static void test_recal_after_hw_reset() {
    setup("RECALIBRATE after RESET-: never ends", CHS);
    chs_user_geometry();
    sim.srst_wedges = true;
    sim.bad[5003] = {BAD_HANG, 0};
    sim.hang_cmd = 0x10;
    uint8_t buf[8 * 512]; uint32_t done = 0;
    int32_t r = ide_read_sectors_partial(5000, 8, buf, &done);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(r < 0 && f.hw_reset && f.reset_failed && sim.cmd_count[0x10] == 1, "hw %d failed %d RECAL %d",
          f.hw_reset, f.reset_failed, sim.cmd_count[0x10]);
    CHECK(sim.init_params == 0 && sim.violations == 0, "0x91 sent %d times, violations %d", sim.init_params, sim.violations);

    setup("RECALIBRATE after RESET-: slow to start, no INTRQ", CHS);
    chs_user_geometry();
    sim.srst_wedges = true;
    sim.intrq_wired = false;
    sim.t_bsy_delay = 5000;
    sim.t_recal = 5000000;
    sim.bad[5003] = {BAD_HANG, 0};
    r = ide_read_sectors_partial(5000, 8, buf, &done);
    ide_last_failure(&f);
    CHECK(r < 0 && f.hw_reset && !f.reset_failed && sim.init_params == 1, "hw %d failed %d 0x91 %d", f.hw_reset, f.reset_failed,
          sim.init_params);
    CHECK(sim.violations == 0 && sim.geo_valid && sim.heads == 5, "violations %d (0x91 while RECALIBRATE ran)", sim.violations);
}

// Review R4: IDENTIFY that ends at once with ERR and a block of data, from a
// drive that shows no BSY and has no INTRQ wired. The DRQ shows the command
// started, so it is an error, never 256 words of IDENTIFY.
static void test_identify_err_drq() {
    setup("IDENTIFY ends at once with ERR and data", LBA);
    sim.identify_err_drq = true; sim.intrq_wired = false; sim.t_sector = 0;
    uint16_t id[256];
    CHECK(!ide_identify(id), "an aborted IDENTIFY taken as its answer");
    CHECK(!(sim.read_status(mock_now_ns + 1000000) & 0x08) && sim.violations == 0, "DRQ left, or violations %d", sim.violations);
}

// ---------------------------------------------------------------------------
//  Review of 0.6f3p7 (MEDIUM): never send a command without a fair window
// ---------------------------------------------------------------------------
// A READ or WRITE SECTORS goes out inside a host command only with
// IDE_SECTOR_ALLOW_MS * (n + 1) of work time (ide.c, time_to_send);
// otherwise nothing is sent, the command fails cleanly and NO_TIME is
// recorded. The reviewer's reproductions E2 to E5 (exp.cpp, against the real
// ide.c and usb.c) each soft-reset a healthy drive on b073832; here each must
// end with no reset of any kind, and the host's smaller commands must work.

static const uint32_t MW = 40000;                   // where these tests read and write

// No reset of either kind, and the drive's protocol kept.
static bool no_reset() { return sim.srst == 0 && sim.hw_resets == 0 && sim.violations == 0; }

// E2: a WRITE(10) of 16 whose first chunk's commit is slow. The second chunk
// (its own commit 1 s, as the ST380011A's was) used to be sent with what was
// left and reset mid-commit. Now it is not sent; the host's retry in 8-sector
// commands writes it.
static void test_min_window_e2() {
    for (uint64_t P : { 9000ull, 14000ull, 14900ull }) {
        std::string n = "min window E2: WRITE(10) of 16, chunk 1 commit " + std::to_string(P) + " ms";
        setup(n.c_str(), LBA);
        config.drive_write_protected = false;
        std::vector<uint8_t> w = fill512(16, 5);
        sim.pause[MW + 7] = P * 1000000ull;
        sim.pause[MW + 15] = 1000000000ull;
        int32_t r = host_write10(MW, w.data(), 16);
        ide_fail_t f; ide_last_failure(&f);
        CHECK(r < 0 && no_reset(), "r %d srst %d hw %d", r, sim.srst, sim.hw_resets);
        CHECK(sim.cmd_count[0x30] == 1 && !sim.written.count(MW + 8), "chunk 2 sent: %d WRITEs", sim.cmd_count[0x30]);
        CHECK(f.kind == IDE_FAIL_NO_TIME && f.lba == MW + 8 && f.count == 8 && !f.reset && !f.pending,
              "record: kind %u lba %u count %u reset %d", f.kind, f.lba, f.count, f.reset);
        if (P == 14900) keep_timing("min window E2: WRITE(10) of 16, chunk 1 commit 14.9 s");
        sim.pause.erase(MW + 7);
        int32_t r2 = host_write10(MW + 8, w.data() + 8 * 512, 8);
        bool all = true;
        for (int i = 0; i < 16; i++) all = all && written_is(MW + i, w.data() + i * 512);
        CHECK(r2 == 4096 && all && no_reset(), "the host's 8-sector retry: r %d content %d", r2, all);
    }
}

// E3: a sector that ends in ERR only after a long retry. The issue #13 call
// again at it used to be sent with the 6 s or so left, and the drive reset
// when it took as long again; ERR never needs a reset. Now the call again is
// sent only with that long again; the host gets the good prefix. A sector
// that fails faster still gets its call again (issue #13). At 8 s the call
// again would have had 7 s, more than the 6.6 s five sectors get, and not
// enough for the 8 s the sector takes. A new host command at the failed
// sector is not held to it: the sector that took 14 s is read again, once,
// by the host's next command.
static void test_min_window_e3() {
    for (uint64_t P : { 0ull, 3000ull, 7400ull, 9000ull, 12000ull, 13400ull }) {
        std::string n = "min window E3: 8-sector READ(10), ERR at sector 3 after " + std::to_string(P) + " ms more";
        setup(n.c_str(), LBA);
        sim.pause[MW + 3] = P * 1000000ull;
        sim.bad[MW + 3] = {BAD_ERR, 0};
        HostRead h = host_read10(MW, 8);
        ide_fail_t f; ide_last_failure(&f);
        int want = P <= 3000 ? 2 : 1;               // the call again goes out only with time for it
        CHECK(!h.ok && h.data.size() == 3 * 512 && matches_medium(h, MW) && no_reset(),
              "ok %d prefix %zu srst %d", h.ok, h.data.size(), sim.srst);
        CHECK(sim.attempts[MW + 3] == want, "the failing sector read %d times, want %d", sim.attempts[MW + 3], want);
        CHECK(f.kind == IDE_FAIL_ERR && f.lba == MW + 3 && !f.reset, "record: kind %u lba %u reset %d", f.kind, f.lba, f.reset);
        if (P == 12000) keep_timing("min window E3: ERR after 12.6 s, no call again");
        if (P != 13400) continue;
        h = host_read10(MW + 3, 1);
        CHECK(!h.ok && sim.attempts[MW + 3] == 2 && no_reset(), "the next command at the sector: read %d times in all",
              sim.attempts[MW + 3]);
    }
}

// E4: a 16-sector READ(10) with one slow but good sector in the first chunk
// and a healthy second chunk, 20 ms a sector. The second chunk used to go
// out with ~40 ms and the drive was reset, on every retry. Now it is not
// sent when the first chunk took more than about 5 s: the command fails
// cleanly, every time, and 8-sector commands read it all.
static void test_min_window_e4() {
    for (uint64_t P : { 4000ull, 6000ull, 14000ull, 14800ull }) {
        std::string n = "min window E4: READ(10) of 16, slow good sector " + std::to_string(P) + " ms";
        setup(n.c_str(), LBA);
        sim.t_sector = 20000000ull;
        sim.pause[MW + 2] = P * 1000000ull;
        bool fits = (P + 8 * 20) + IDE_SECTOR_ALLOW_MS * 9 <= IDE_CMD_TIMEOUT_MS;
        for (int i = 0; i < 3; i++) {
            HostRead h = host_read10(MW, 16);
            ide_fail_t f; ide_last_failure(&f);
            if (fits) {
                CHECK(h.ok && matches_medium(h, MW) && no_reset(), "try %d: ok %d srst %d", i, h.ok, sim.srst);
                continue;
            }
            CHECK(!h.ok && h.data.size() == 8 * 512 && matches_medium(h, MW) && no_reset(),
                  "try %d: ok %d got %zu srst %d", i, h.ok, h.data.size(), sim.srst);
            CHECK(sim.cmd_count[0x20] == i + 1, "try %d: %d READs sent", i, sim.cmd_count[0x20]);
            CHECK(f.kind == IDE_FAIL_NO_TIME && f.lba == MW + 8 && !f.reset, "try %d: record kind %u lba %u", i, f.kind, f.lba);
        }
        if (P == 14800) keep_timing("min window E4: READ(10) of 16, slow good sector 14.8 s");
        HostRead a = host_read10(MW, 8), b = host_read10(MW + 8, 8);
        CHECK(a.ok && b.ok && matches_medium(a, MW) && matches_medium(b, MW + 8) && no_reset(),
              "8-sector commands: %d %d", a.ok, b.ok);
    }
}

// How many 8-sector chunks of a READ(10) are sent when each sector takes
// t_ms: chunk k goes out while k * 8 * t_ms leaves it its fair window.
static uint32_t chunks_sent(uint32_t sectors, double t_ms) {
    uint32_t k = 0;
    while (k * 8 < sectors && k * 8 * t_ms + IDE_SECTOR_ALLOW_MS * 9 <= IDE_CMD_TIMEOUT_MS) k++;
    return k;
}

// E5: a slow but healthy drive, 120 ms a sector, and the 128-sector READ(10)
// imagelba sends. b073832 reset it on every try (0.6f3p6 read it). Now the
// command fails cleanly on every try with the chunks that fit, no reset,
// and what the host does next, smaller commands, reads it.
static void test_min_window_e5() {
    setup("min window E5: READ(10) of 128, 120 ms a sector", LBA);
    sim.t_sector = 120000000ull;
    uint32_t want = chunks_sent(128, 120) * 4096;
    for (int i = 0; i < 5; i++) {
        HostRead h = host_read10(MW, 128);
        ide_fail_t f; ide_last_failure(&f);
        CHECK(!h.ok && h.data.size() == want && matches_medium(h, MW) && no_reset(),
              "try %d: ok %d got %zu want %u srst %d", i, h.ok, h.data.size(), want, sim.srst);
        CHECK(f.kind == IDE_FAIL_NO_TIME && f.lba == MW + want / 512 && !f.reset && !f.pending,
              "try %d: record kind %u lba %u", i, f.kind, f.lba);
    }
    keep_timing("min window E5: READ(10) of 128, 120 ms a sector");
    HostRead a = host_read10(MW, 8), b = host_read10(MW + 64, 1);
    CHECK(a.ok && b.ok && matches_medium(a, MW) && matches_medium(b, MW + 64) && no_reset(),
          "the host's smaller commands: %d %d", a.ok, b.ok);
}

// The throughput floor ide.c documents: the slowest average sector time at
// which a 128-sector READ(10) still completes, derived from the constants.
// Just under it the command completes with no reset; just over it it fails
// cleanly with no reset.
static void test_min_window_floor() {
    double floor_ms = (double)(IDE_CMD_TIMEOUT_MS - IDE_SECTOR_ALLOW_MS * 9) / 120;
    current = "min window: the 128-sector floor";
    CHECK((int)floor_ms == 42, "floor %.1f ms a sector; ide.c says about 42", floor_ms);
    setup("min window: 128 sectors just under the floor", LBA);
    sim.t_sector = (uint64_t)(floor_ms * 0.97 * 1000000);
    HostRead h = host_read10(MW, 128);
    CHECK(h.ok && matches_medium(h, MW) && no_reset(), "ok %d srst %d", h.ok, sim.srst);
    setup("min window: 128 sectors just over the floor", LBA);
    sim.t_sector = (uint64_t)(floor_ms * 1.03 * 1000000);
    h = host_read10(MW, 128);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(!h.ok && h.data.size() == 15 * 4096 && no_reset() && f.kind == IDE_FAIL_NO_TIME,
          "ok %d got %zu srst %d kind %u", h.ok, h.data.size(), sim.srst, f.kind);
}

// The window is checked again after waiting for the drive to be ready: a
// drive busy for 4 s after the first chunk leaves the second less than its
// window, and it is not sent.
static void test_min_window_after_ready_wait() {
    setup("min window: checked again after the ready wait", LBA);
    sim.pause[MW + 2] = 2000000000ull;
    sim.busy_after[MW + 7] = 4000000000ull;
    HostRead h = host_read10(MW, 16);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(!h.ok && h.data.size() == 8 * 512 && sim.cmd_count[0x20] == 1 && no_reset(),
          "ok %d got %zu, %d READs", h.ok, h.data.size(), sim.cmd_count[0x20]);
    CHECK(f.kind == IDE_FAIL_NO_TIME && f.lba == MW + 8, "record kind %u lba %u", f.kind, f.lba);
}

// ...and after INITIALIZE DEVICE PARAMETERS, for a write. CHS mode with the
// geometry to send again, 0.9 s for the drive to take it, and 10.5 s of work
// time left when a later chunk of a WRITE(10) arrives: after the 0x91 the
// write no longer has its window, and is not sent. Called directly, inside a
// host command begun by hand, as usb.c would begin it.
static void test_min_window_after_geometry() {
    setup("min window: checked again after 0x91 (CHS write)", CHS);
    config.drive_write_protected = false;
    sim.t_idp = 900000000ull;
    std::vector<uint8_t> w = fill512(8, 6);
    ide_host_cmd_begin();
    ide_host_cmd_enter();
    mock_now_ns += (IDE_CMD_TIMEOUT_MS - 10500) * 1000000ull;  // the chunks before it took this long
    chs_geometry_lost = true;                                   // as after a reset
    int32_t r = ide_write_sectors(MW, 8, w.data());
    ide_host_cmd_leave();
    ide_fail_t f; ide_last_failure(&f);
    CHECK(r < 0 && sim.cmd_count[0x30] == 0 && sim.init_params == 1 && no_reset(), "r %d, %d WRITEs, 0x91 %d",
          r, sim.cmd_count[0x30], sim.init_params);
    CHECK(f.kind == IDE_FAIL_NO_TIME && f.lba == MW, "record kind %u lba %u", f.kind, f.lba);
}

#if ATABOY_SAT
// SAT: a command is sent only with its whole SAT_CMD_TIMEOUT_MS of work time,
// and the identity check only with time for all of it. After a recovery
// carried over from an earlier command leaves about 7 s of work, a READ
// SECTORS and a gated READ NATIVE MAX are refused NOT READY with nothing sent
// (no IDENTIFY either), NO_TIME is recorded, and the next command works.
static void test_sat_min_window() {
    for (int gated = 0; gated < 2; gated++) {
        setup(gated ? "min window: SAT gated row after a long recovery" : "min window: SAT read after a long recovery", LBA);
        sim.srst_wedges = true;
        sim.t_hw_reset = 7000000000ull;
        sim.bad[5003] = {BAD_HANG, 0};
        HostRead h = host_read10(5000, 8);          // SRST, pending
        sim.bad.clear();
        mock_now_ns = sim.srst_at + (IDE_SRST_TIMEOUT_MS - 1000) * 1000000ull;
        int work = work_commands(), ids = sim.id_commands();
        SatResult s = gated ? sat_cmd(native_max12(), 0, false) : sat_cmd(pt12(4, 0x0E, 0, 1, 100, 0xE0, 0x20), 512, true);
        ide_fail_t f; ide_last_failure(&f);
        CHECK(!s.ok() && sense_is(s.sense, 0x02, 0x04, 0x00) && sim.hw_resets == 1, "r %d hw %d", s.r, sim.hw_resets);
        CHECK(work_commands() == work && sim.id_commands() == ids, "%d commands and %d IDENTIFYs sent",
              work_commands() - work, sim.id_commands() - ids);
        CHECK(f.kind == IDE_FAIL_NO_TIME && f.hw_reset && !f.pending, "record kind %u hw %d pending %d", f.kind, f.hw_reset, f.pending);
        s = gated ? sat_cmd(native_max12(), 0, false) : sat_cmd(pt12(4, 0x0E, 0, 1, 100, 0xE0, 0x20), 512, true);
        CHECK(gated ? is_desc(s.sense) && sense_is(s.sense, 0x01, 0x00, 0x1D) : s.ok() && sat_data_is_medium(s, 100),
              "the next command: r %d", s.r);
        CHECK(sim.violations == 0 && sim.srst == 1, "violations %d srst %d", sim.violations, sim.srst);
        (void)h;
    }
    // Checked again after waiting for the drive to be ready: the identity
    // check's IDENTIFY takes 2 s and leaves the drive busy 4 s more, so READ
    // NATIVE MAX would go out with 9 s. It is not sent.
    setup("min window: SAT checked again after the ready wait", LBA);
    sim.t_identify_extra = 2000000000ull;
    sim.busy_after[0] = 4000000000ull;
    int nm = sim.cmd_count[0xF8];
    SatResult s = sat_cmd(native_max12(), 0, false);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(!s.ok() && sense_is(s.sense, 0x02, 0x04, 0x00) && sim.cmd_count[0xF8] == nm && no_reset(),
          "r %d, READ NATIVE MAX sent %d times", s.r, sim.cmd_count[0xF8] - nm);
    CHECK(f.kind == IDE_FAIL_NO_TIME && f.command == 0xF8, "record kind %u cmd %02X", f.kind, f.command);
}
#endif

// ---------------------------------------------------------------------------
//  Review of 0.6f3p7: the LOW items and surviving mutants
// ---------------------------------------------------------------------------

// ide_reset_drive() (the debug screen's reset, core 1) pulses RESET- itself,
// so it ends a recovery in progress, as the probe does (X1).
static void test_reset_drive_ends_recovery() {
    setup("ide_reset_drive ends a pending recovery", LBA);
    sim.srst_wedges = true;
    sim.bad[5003] = {BAD_HANG, 0};
    HostRead h = host_read10(5000, 8);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(!h.ok && f.pending, "pending %d", f.pending);
    sim.bad.clear();
    is_mounted = false;
    ide_reset_drive();
    ide_last_failure(&f);
    CHECK(!f.pending && rec.stage == REC_NONE, "after the reset: pending %d stage %u", f.pending, rec.stage);
    is_mounted = true;
    h = host_read10(5000, 8);
    CHECK(h.ok && matches_medium(h, 5000) && sim.hw_resets == 1, "read after it: ok %d hw %d", h.ok, sim.hw_resets);
}

// CHS: the drive comes back from RESET- with about 0.2 s of the host command
// left, and INITIALIZE DEVICE PARAMETERS takes 0.5 s. It is not sent then;
// the recovery stays pending, the record does not say the reset failed, and
// the next host command sends it with its whole second and reads.
static void test_geometry_waits_for_time() {
    setup("CHS: 0x91 not sent without its second", CHS);
    chs_user_geometry();
    sim.srst_wedges = true;
    sim.t_idp = 500000000ull;
    sim.t_hw_reset = (IDE_HOST_BUDGET_MS - 250) * 1000000ull;
    sim.bad[5003] = {BAD_HANG, 0};
    HostRead h = host_read10(5000, 8);              // SRST, pending
    sim.bad.clear();
    mock_now_ns = sim.srst_at + (IDE_SRST_TIMEOUT_MS - 1) * 1000000ull;
    int idp = sim.init_params;
    h = host_read10(5000, 8);                       // RESET-, back near the end
    ide_fail_t f; ide_last_failure(&f);
    CHECK(!h.ok && sim.hw_resets == 1 && sim.init_params == idp, "ok %d hw %d, 0x91 sent %d times", h.ok, sim.hw_resets,
          sim.init_params - idp);
    CHECK(f.pending && !f.reset_failed, "record: pending %d failed %d", f.pending, f.reset_failed);
    h = host_read10(5000, 8);
    ide_last_failure(&f);
    CHECK(h.ok && matches_medium(h, 5000) && sim.init_params == idp + 1, "next command: ok %d, 0x91 %d", h.ok, sim.init_params - idp);
    CHECK(!f.pending && !f.reset_failed && sim.geo_valid && sim.heads == 5 && sim.spt == 34, "after: pending %d failed %d geometry %d",
          f.pending, f.reset_failed, sim.geo_valid);
    CHECK(sim.violations == 0 && sim.hw_resets == 1, "violations %d hw %d", sim.violations, sim.hw_resets);
}

// usb.c: a READ(10) while nothing is mounted refuses without entering a host
// command, so core 1, which may be detecting a drive meanwhile, never has its
// waits cut to a host command's time. (Seen from here as the host command's
// state left exactly as it was.)
static void test_unmounted_callback_untimed() {
    setup("an unmounted READ(10) is not timed", LBA);
    is_mounted = false;
    host.start = 0xDEADBEEFu;
    static uint8_t buf[CFG_TUD_MSC_EP_BUFSIZE];
    int32_t r = tud_msc_read10_cb(0, 100, 0, buf, 4096);
    CHECK(r < 0 && !host.on && host.start == 0xDEADBEEFu, "r %d on %d start %x", r, host.on, host.start);
    is_mounted = true;
}

// ide.c's side of the Features IORDY switch: while IORDY is held ignored
// after a hardware reset (the drive back from neither reset), switching it
// on changes nothing on the pin, from the menu or at the next host command;
// once the drive is back the setting is believed. While mounted, a change
// reaches the pin at the next host command.
static void test_iordy_setting_deferred() {
    setup("IORDY switched on while RESET- recovery holds it", LBA);
    sim.srst_wedges = true; sim.hw_reset_wedges = true;
    sim.bad[5000] = {BAD_HANG, 0};
    host_read_retry(5000, 1, 3);
    CHECK(sim.hw_resets == 1 && iordy_held, "hw %d held %d", sim.hw_resets, iordy_held);
    config.iordy_enabled = true;
    ide_iordy_follow_config();
    CHECK(mock_iordy_inover == GPIO_OVERRIDE_HIGH, "the menu's switch undid the hold");
    host_read10(100, 1);
    CHECK(mock_iordy_inover == GPIO_OVERRIDE_HIGH && sim.iordy_stalls == 0, "a host command undid the hold (%d stalls)", sim.iordy_stalls);
    sim.srst_wedges = false; sim.hw_reset_wedges = false; sim.wedged = false; sim.bad.clear();
    HostRead h;
    int got = host_read_retry(100, 1, 2, &h);
    CHECK(got >= 1 && mock_iordy_inover == GPIO_OVERRIDE_NORMAL, "once back: try %d, IORDY believed %d", got,
          mock_iordy_inover == GPIO_OVERRIDE_NORMAL);
    config.iordy_enabled = false;
    h = host_read10(100, 1);
    CHECK(h.ok && mock_iordy_inover == GPIO_OVERRIDE_HIGH, "switched off while mounted: not on the pin at the next command");
}

#if ATABOY_SAT
// The shipping build refuses SMART READ DATA (D0) and RETURN STATUS (DA):
// on ATA-3 drives both save attribute values to the drive (sat_policy.h).
// Nothing may reach the drive for them, not even the identity check's
// IDENTIFY; the SMART rows without that wording, and READ NATIVE MAX, work.
// Built by run.sh without ATABOY_SAT_SMART_SAVES; everything else in this
// file runs in the build with it set.
static void test_sat_smart_saves_off(Mode m) {
    std::string n = name2("shipping build: SMART READ DATA and RETURN STATUS refused", m);
    setup(n.c_str(), m);
    identify_with(0x4401, 0x4400, 0x4001);              // the drive claims everything
    struct { const char *name; std::vector<uint8_t> cdb; uint32_t xfer; bool dir_in; } off[] = {
        { "SMART READ DATA, 12", pt12(4, 0x0E, 0xD0, 1, 0xC24F00, 0xA0, 0xB0), 512, true },
        { "SMART READ DATA, 16", pt16(4, false, 0x0E, 0xD0, 1, 0xC24F00, 0xA0, 0xB0), 512, true },
        { "SMART RETURN STATUS, 12", smart_status12(), 0, false },
        { "SMART RETURN STATUS, 12 (smartctl form)", smart_status12(0x2C), 0, false },
        { "SMART RETURN STATUS, 16", pt16(3, false, 0x20, 0xDA, 0, 0xC24F00, 0xA0, 0xB0), 0, false },
    };
    for (auto &o : off) {
        int cmds = sim.commands;
        SatResult r = sat_cmd(o.cdb, o.xfer, o.dir_in);
        CHECK(!r.ok() && sense_is(r.sense, 0x05, 0x24, 0x00), "%s: expected 5/24/00", o.name);
        CHECK(sim.commands == cmds, "%s: %d commands reached the drive", o.name, sim.commands - cmds);
    }
    int b0 = sim.cmd_count[0xB0];
    SatResult t = sat_cmd(pt12(4, 0x0E, 0xD1, 1, 0xC24F00, 0xA0, 0xB0), 512, true);
    CHECK(t.ok() && sim.cmd_count[0xB0] == b0 + 1, "SMART READ THRESHOLDS still works: ok %d", t.ok());
    SatResult nm = sat_cmd(native_max12(), 0, false);
    CHECK(is_desc(nm.sense) && sense_is(nm.sense, 0x01, 0x00, 0x1D), "READ NATIVE MAX still works");
    CHECK(sim.violations == 0, "violations %d", sim.violations);
}
#endif

// ---------------------------------------------------------------------------
//  Review of 0.6f3p8
// ---------------------------------------------------------------------------

// T19: a WRITE(10) while a reset from an earlier command is still pending is
// refused as a read is (recovery_gate): nothing is written when the recovery
// ends without the drive back. Here the drive stays busy after SRST, RESET-
// follows, and the drive is back one second after the recovery's 31 s are
// over, inside the 5 s a write would wait for it to be ready: a write that went
// on regardless would be written to it.
static void test_write_refused_while_pending() {
    setup("WRITE(10) refused while a reset is pending", LBA);
    sim.srst_wedges = true;
    sim.t_hw_reset = (IDE_SRST_TIMEOUT_MS + 1000) * 1000000ull;
    sim.bad[5003] = {BAD_HANG, 0};
    HostRead h = host_read10(5000, 8);              // SRST, still busy: pending
    sim.bad.clear();
    mock_now_ns = sim.srst_at + (IDE_SRST_TIMEOUT_MS - 1) * 1000000ull;
    h = host_read10(5000, 8);                       // the SRST's 31 s over: RESET-, pending
    CHECK(!h.ok && sim.hw_resets == 1 && rec.stage == REC_HW, "hw %d stage %u", sim.hw_resets, rec.stage);
    mock_now_ns = ((uint64_t)rec.since + IDE_SRST_TIMEOUT_MS - 100) * 1000000ull;
    int cmds = sim.commands;
    std::vector<uint8_t> data = fill512(8, 0x5A);
    int32_t r = host_write10(6000, data.data(), 8);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(r < 0 && sim.written.empty() && sim.commands == cmds, "write %d, %zu sectors written, %d commands",
          r, sim.written.size(), sim.commands - cmds);
    CHECK(!f.pending && f.reset_failed, "record: pending %d, reset failed %d", f.pending, f.reset_failed);
    // The recovery is over; the drive is back now, and the next write goes.
    mock_now_ns += 2000000000ull;
    r = host_write10(6000, data.data(), 8);
    CHECK(r == 8 * 512 && written_is(6000, data.data()), "write after the drive is back: %d", r);
}

// M-1: the state after each reset. Since hw_reset_start() marks the geometry
// lost as well, and every way out of a pending recovery either sends 0x91 or
// resets again, a soft reset that forgot to mark it would no longer read a
// wrong sector in any test; so the flag itself is checked while an SRST's
// recovery is still waiting for the drive (no hardware reset yet).
static void test_srst_marks_geometry_lost() {
    setup("SRST pending: geometry marked lost", CHS);
    chs_user_geometry();
    CHECK(!chs_geometry_lost, "lost before any reset");
    sim.t_reset = 20000000000ull;                   // back from SRST after 20 s: inside its 31 s
    sim.bad[5003] = {BAD_HANG, 0};
    HostRead h = host_read10(5000, 8);
    CHECK(!h.ok && rec.stage == REC_SRST && sim.hw_resets == 0, "stage %u hw %d", rec.stage, sim.hw_resets);
    CHECK(chs_geometry_lost, "SRST sent, geometry not marked lost");
    sim.bad.clear();
    mock_now_ns += 21000000000ull;
    h = host_read10(1234, 8);
    CHECK(h.ok && matches_medium(h, 1234) && sim.geo_valid && !chs_geometry_lost, "read once back: ok %d", h.ok);
}

#if ATABOY_SAT
// L-2: a drive set up by Ctrl+G (no IDENTIFY) gets nothing through ATA
// PASS-THROUGH, IDENTIFY first: refused 5/24/00 before anything reaches the
// drive, until an IDENTIFY the firmware sent itself has answered. READ(10)
// works throughout.
static void test_sat_manual_chs_refused() {
    setup("SAT refuses everything to a drive set up by Ctrl+G", CHS);
    is_mounted = false;
    uint8_t st = 0;
    CHECK(ide_manual_chs(10, 17, &st) == IDE_MCHS_OK && ide_manual_chs_active(), "Ctrl+G: st %02X", st);
    is_mounted = true;
    int cmds = sim.commands;
    struct { const char *name; std::vector<uint8_t> cdb; uint32_t xfer; bool in; } rows[] = {
        { "IDENTIFY (12)",      pt12(4, 0x0E, 0, 1, 0, 0xA0, 0xEC), 512, true },
        { "IDENTIFY (16)",      pt16(4, false, 0x0E, 0, 1, 0, 0x00, 0xEC), 512, true },
        { "READ SECTORS",       pt12(4, 0x0E, 0, 1, 100, 0xE0, 0x20), 512, true },
        { "READ VERIFY",        verify12(0x00, 4, 100), 0, false },
    };
    for (auto &e : rows) {
        SatResult s = sat_cmd(e.cdb, e.xfer, e.in);
        CHECK(!s.ok() && s.sense.size() == 18 && sense_is(s.sense, 0x05, 0x24, 0x00), "%s: r %d", e.name, s.r);
    }
    for (auto &g : gated_rows()) {
        SatResult s = sat_cmd(g.cdb, g.xfer, g.in);
        CHECK(!s.ok() && sense_is(s.sense, 0x05, 0x24, 0x00), "%s: r %d", g.name, s.r);
    }
    CHECK(sim.commands == cmds && sim.data_reads == 0, "reached the drive: %d commands", sim.commands - cmds);
    HostRead h = host_read10(100, 8);
    CHECK(h.ok && matches_medium(h, 100), "READ(10) after Ctrl+G");
    // Unmounted: NOT READY still comes first.
    is_mounted = false;
    SatResult u = sat_cmd(rows[0].cdb, 512, true);
    CHECK(!u.ok() && sense_is(u.sense, 0x02, 0x04, 0x00), "unmounted: not NOT READY");
    // Detection: the drive answers the firmware's own IDENTIFY. Allowed again.
    uint16_t id[256];
    CHECK(ide_identify(id) && !ide_manual_chs_active(), "flag kept after IDENTIFY answered");
    is_mounted = true;
    int ids = sim.id_commands();
    SatResult a = sat_cmd(rows[0].cdb, 512, true);
    CHECK(a.r == 512 && sim.id_commands() == ids + 1, "IDENTIFY after detection: r %d", a.r);
}
#endif

int main() {
    // The budget is a design number, not only what the helpers measure
    // against: 10 s under the 30 s that Linux and the project's tools give a
    // command (ide.c). A larger one would pass every test above and still
    // lose the host.
    current = "budget design";
    CHECK(IDE_HOST_BUDGET_MS <= 20000 && IDE_RECOVERY_RESERVE_MS >= 5000 && HOST_MARGIN_MS <= 100,
          "budget %u ms, reserve %u ms", (unsigned)IDE_HOST_BUDGET_MS, (unsigned)IDE_RECOVERY_RESERVE_MS);
#if ATABOY_SAT && !ATABOY_SAT_SMART_SAVES
    test_sat_smart_saves_off(LBA);
    test_sat_smart_saves_off(CHS);
    std::printf("%d checks, %d failed\n", checks, failures);
    return failures > 255 ? 255 : failures;
#endif
    test_clean_reads(LBA);
    test_clean_reads(CHS);
    test_bad_at_each_position(LBA, BAD_ERR, "bad at each position, ERR, LBA");
    test_bad_at_each_position(LBA, BAD_ERR_DRQ, "bad at each position, ERR+DRQ, LBA");
    test_bad_at_each_position(CHS, BAD_ERR_DRQ, "bad at each position, ERR+DRQ, CHS");
    test_issue13_case(LBA);
    test_issue13_case(CHS);
    test_hang(LBA);
    test_hang(CHS);
    test_err_drq_stuck();
    test_garbage_while_busy();
    test_stale_status_after_error();
    test_media_bounds();
    test_failure_record(LBA);
    test_failure_record(CHS);
    test_marginal();
    test_two_bad();
    test_writes();
    test_not_ready();
    test_stale_drq(LBA);
    test_stale_drq(CHS);
    test_slow_reset_chs();
    test_reset_never_ready_chs();
    test_geometry_rejected_chs();
    test_slave_after_reset(LBA);
    test_slave_after_reset(CHS);
    test_slave_with_master_after_reset(LBA);
    test_slave_with_master_after_reset(CHS);
    test_host_offsets();
    test_msc_busy_flag();
    test_pause_outwaited(LBA);
    test_pause_outwaited(CHS);
    test_pause_too_long(LBA);
    test_pause_too_long(CHS);
    test_lone_slave_hw_reset(LBA);
    test_lone_slave_hw_reset(CHS);
    test_master_hw_reset();
    test_identify_slow();
    // review M-1 and the LOW items of the 0.6f3p7 review
    test_budget_read_hang(LBA);
    test_budget_read_hang(CHS);
    test_budget_write_hang(LBA);
    test_budget_write_hang(CHS);
    test_budget_srst_fails(LBA);
    test_budget_srst_fails(CHS);
    test_budget_lone_slave_wedged();
    test_budget_reset_at_the_end();
    test_budget_stale_drq();
#if ATABOY_SAT
    test_budget_sat_timeout();
    test_budget_identity_timeout();
#endif
    test_budget_boundaries();
    test_redetect_ends_recovery();
    test_budget_record_sticky();
    test_deadline_boundary();
    test_recal_after_hw_reset();
    test_identify_err_drq();
    // review of 0.6f3p7: the least time a command is sent with, and the LOWs
    test_min_window_e2();
    test_min_window_e3();
    test_min_window_e4();
    test_min_window_e5();
    test_min_window_floor();
    test_min_window_after_ready_wait();
    test_min_window_after_geometry();
#if ATABOY_SAT
    test_sat_min_window();
#endif
    test_reset_drive_ends_recovery();
    test_geometry_waits_for_time();
    test_unmounted_callback_untimed();
    test_iordy_setting_deferred();
    test_write_refused_while_pending();
    test_srst_marks_geometry_lost();
#if ATABOY_SAT
    test_sat_smart_status(LBA);
    test_sat_smart_status(CHS);
    test_sat_native_max(LBA);
    test_sat_native_max(CHS);
    test_sat_native_max_ext_abort();
    test_sat_verify();
    test_sat_nondata_timeout(LBA);
    test_sat_nondata_timeout(CHS);
    test_sat_nondata_drq();
    test_sat_stale_drq(LBA);
    test_sat_stale_drq(CHS);
    test_sat_smart_data(LBA);
    test_sat_smart_data(CHS);
    test_sat_read_error_registers();
    test_sat_sense_delivery();
    test_sat_refusals_touch_nothing();
    test_sat_identify_gate();
    test_sat_identity_verified(LBA);
    test_sat_identity_verified(CHS);
    test_sat_slow_bsy();
    test_sat_never_starts();
    test_sat_fast_command();
    test_sat_device_fault();
    test_sat_manual_chs_refused();
#else
    test_no_sat();
#endif
    // Worst time core 0 spent in one callback, and in one host command, per
    // failure scenario (the report quotes these).
    for (auto &tm : timings)
        std::printf("budget: %-62s callback %6llu ms, command %6llu ms\n", tm.name.c_str(),
                    (unsigned long long)tm.cb_ms, (unsigned long long)tm.cmd_ms);
    std::printf("%d checks, %d failed\n", checks, failures);
    return failures > 255 ? 255 : failures;
}
