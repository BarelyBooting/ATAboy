// Host tests for the READ(10) path: the real ide.c and usb.c, compiled for a
// PC, running against the simulated drive in sim.hpp. The READ(10) driver
// below follows TinyUSB 0.18's proc_read10_cmd (pico-sdk 2.2.0): it calls
// tud_msc_read10_cb with at most CFG_TUD_MSC_EP_BUFSIZE bytes at a time,
// sends whatever the callback returns, calls again for the rest, and fails
// the command (CSW failed, remaining data not sent) on a negative return.
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

static uint8_t last_key, last_asc;
bool tud_msc_set_sense(uint8_t, uint8_t key, uint8_t asc, uint8_t) {
    last_key = key; last_asc = asc; return true;
}

static int failures = 0, checks = 0;
static std::string current;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; \
    std::printf("FAIL [%s] %s:%d: %s -- ", current.c_str(), __FILE__, __LINE__, #cond); \
    std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

// ---- host side of READ(10) -------------------------------------------------
struct HostRead {
    bool ok = false;
    std::vector<uint8_t> data;   // bytes the host actually received
    int calls = 0;
    uint8_t key = 0, asc = 0;    // sense the firmware set (TinyUSB overwrites it today)
};

static HostRead host_read10(uint32_t lba, uint32_t nblocks) {
    HostRead h;
    uint32_t total = nblocks * 512, xferred = 0;
    static uint8_t epbuf[CFG_TUD_MSC_EP_BUFSIZE];
    last_key = last_asc = 0;
    int busy = 0;
    while (xferred < total) {
        uint32_t cur = lba + xferred / 512, off = xferred % 512;
        uint32_t n = total - xferred;
        if (n > CFG_TUD_MSC_EP_BUFSIZE) n = CFG_TUD_MSC_EP_BUFSIZE;
        memset(epbuf, 0xA5, sizeof(epbuf));        // poison: anything not written shows up
        int32_t r = tud_msc_read10_cb(0, cur, off, epbuf, n);
        h.calls++;
        if (r < 0) { h.key = last_key; h.asc = last_asc; return h; }
        if (r == 0) { if (++busy > 100) return h; continue; }
        if ((uint32_t)r > n) { std::printf("callback returned more than asked\n"); return h; }
        h.data.insert(h.data.end(), epbuf, epbuf + r);
        xferred += r;
    }
    h.ok = true;
    return h;
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
    mock_gpio_out = (1u << 24) | (1u << 25);      // CS0/CS1 idle high
    memset(&config, 0, sizeof(config));
    config.dev_base = 0xA0;
    config.use_lba_mode = (m == LBA);
    config.cyls = 980; config.heads = 10; config.spt = 17;
    config.lba_sectors = medium_sectors ? medium_sectors : sim.nsect;
    if (m == CHS && medium_sectors) config.cyls = (uint16_t)(medium_sectors / 170);
    ide_select_device(0xA0);
    ide_hw_init();
    if (m == CHS) ide_set_geometry(config.heads, config.spt);
    is_mounted = true;
    media_changed_waiting = false;
    sim.srst = 0; sim.init_params = 0; sim.violations = 0; sim.attempts.clear();
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
    int32_t r = tud_msc_write10_cb(0, 400, 0, buf.data(), 4096);
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
    CHECK(sim.geo_valid && sim.heads == 5 && sim.spt == 34, "geometry %d %u x %u", sim.geo_valid, sim.heads, sim.spt);
    sim.bad.clear();
    HostRead g = host_read10(1234, 8);
    CHECK(g.ok && matches_medium(g, 1234), "read after the reset");
    CHECK(sim.violations == 0 && sim.reset_writes == 0, "violations %d reset writes %d", sim.violations, sim.reset_writes);
}

// A reset that outlasts the 31 s allowance: reads must refuse (never read
// through the default translation), and work again once the drive is back.
static void test_reset_never_ready_chs() {
    setup("soft reset never finishes in time, CHS", CHS);
    chs_user_geometry();
    sim.t_reset = 45000000000ull;                  // 45 s
    sim.bad[5003] = {BAD_HANG, 0};
    // One call first, so the record read back is this failure's (the host
    // retry that follows fails "not ready" and records that instead).
    uint8_t buf[8 * 512]; uint32_t done = 0;
    int32_t r = ide_read_sectors_partial(5000, 8, buf, &done);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(r < 0 && done == 3, "r %d done %u", r, done);
    CHECK(f.kind == IDE_FAIL_TIMEOUT && f.reset && f.reset_failed, "kind %u reset %d failed %d", f.kind, f.reset, f.reset_failed);
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
    CHECK(mock_now_ns - t0 < 20000000000ull, "reset took %llu ms: waited on the absent master", (unsigned long long)((mock_now_ns - t0) / 1000000));
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

int main() {
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
    test_host_offsets();
    std::printf("%d checks, %d failed\n", checks, failures);
    return failures > 255 ? 255 : failures;
}
