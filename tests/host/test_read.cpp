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

static uint8_t last_key, last_asc, last_ascq;     // TinyUSB's stored sense
bool tud_msc_set_sense(uint8_t, uint8_t key, uint8_t asc, uint8_t ascq) {
    last_key = key; last_asc = asc; last_ascq = ascq; return true;
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
    if (last_key == 0) { s.called = true; s.r = tud_msc_scsi_cb(0, cb, ep, (uint16_t)xfer); }
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
    int32_t r = tud_msc_write10_cb(0, 500, 0, w.data(), 4096);
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
    r = tud_msc_write10_cb(0, 500, 0, w.data(), 4096);
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
    r = tud_msc_write10_cb(0, 30, 0, wb.data(), 512);
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

static void identify_with(uint16_t w82, uint16_t w83, uint16_t w84) {
    sim.id_w82 = w82; sim.id_w83 = w83; sim.id_w84 = w84;
    uint16_t id[256];
    CHECK(ide_identify(id), "IDENTIFY %04x %04x %04x", w82, w83, w84);
}

// Re-review M-A: a drive swapped on the cable after detection and mounted
// without a new one (Mount only sets is_mounted) must not be judged by the
// old drive's IDENTIFY words. Before a gated command the firmware asks the
// drive for IDENTIFY again and compares serial, model and words 82..84.
struct IdRow { const char *name; std::vector<uint8_t> cdb; uint32_t xfer; bool dir_in; uint8_t op; bool lba_only; };
static std::vector<IdRow> id_check_rows() {
    return { { "SMART RETURN STATUS", smart_status12(), 0, false, 0xB0, false },
             { "READ NATIVE MAX", native_max12(), 0, false, 0xF8, false },
             { "READ SECTORS EXT", pt16(4, true, 0x0E, 0, 1, 100, 0x40, 0x24), 512, true, 0x24, true } };
}
static void test_sat_identity_verified(Mode m) {
    std::string n = name2("SAT gated rows verify the drive is the detected one", m);
    setup(n.c_str(), m);
    auto reset_drive = [&]() {
        sim.id_model = "SIMULATED ATA DRIVE"; sim.id_serial = "SIM0000001";
        sim.id_w82 = 0x4401; sim.id_w83 = 0x4400; sim.id_w84 = 0x4001; sim.identify_ok = true;
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
    identify_with(0x0001, 0x0000, 0x0000);
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
    test_slave_with_master_after_reset(LBA);
    test_slave_with_master_after_reset(CHS);
    test_host_offsets();
    test_msc_busy_flag();
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
#else
    test_no_sat();
#endif
    std::printf("%d checks, %d failed\n", checks, failures);
    return failures > 255 ? 255 : failures;
}
