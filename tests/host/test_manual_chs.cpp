// Host tests for manual CHS with no IDENTIFY (0.6f3p8, main menu Ctrl+G):
// menus.c's entry and ide.c's ide_manual_chs(), together, against the
// simulated drive in sim.hpp. Keys arrive on the mock clock as in
// test_fwupdate.cpp; everything the firmware sends the drive is counted by
// the simulator (cmd_log, reg_writes, hw_resets).
//
// What must hold:
//  - IDENTIFY (0xEC) is never sent, on any path.
//  - The drive gets nothing at all until Y: not for Esc at any point, not
//    for N, not for an entry out of range (each refusal says why).
//  - After Y: RESET-, then RECALIBRATE, then 0x91 with the operator's heads
//    and sectors. Nothing else.
//  - 0x91 accepted: the geometry and capacity are set. Refused, or anything
//    else going wrong: nothing is shown as set, capacity 0, Debug E has it.
//  - Not reachable while mounted, or while a reset is pending; a USB command
//    still running at Y stops it; the EEPROM is never written.
//
// Build and run: see run.sh. Exit status is the number of failed checks.
#include "mock_pico.h"
#include "sim.hpp"
#include "obs.c"
#include "ide.c"

#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <string>
#include <vector>

// ---- stand-ins menus.c needs ----------------------------------------------
typedef struct { int unused; } queue_t;
#define PICO_ERROR_TIMEOUT (-1)
struct Arrival { uint64_t at_us; char c; };
static std::vector<Arrival> rx;
static size_t rxi;
static std::string tty;
static uint64_t hang_limit_ns = ~0ull;
static int failures = 0, checks = 0;
// The main loop (core1_entry) never returns: once its keys are used up and
// idle_after_ns has passed, on_idle checks the result and ends the program.
static void (*on_idle)() = nullptr;
static uint64_t idle_after_ns = 0;
static inline void queue_add_blocking(queue_t *, const void *c) { tty.push_back(*(const char *)c); }
static inline bool queue_try_remove(queue_t *, void *out) {
    if (rxi < rx.size() && get_absolute_time() >= rx[rxi].at_us) { *(char *)out = rx[rxi++].c; return true; }
    if (on_idle && rxi == rx.size() && mock_now_ns > idle_after_ns) on_idle();
    // Waiting for a key that will never come: the code under test is stuck.
    if (mock_now_ns > hang_limit_ns) {
        printf("FAIL: waiting for a key that never comes (a hang)\n%d checks, %d failed\n", checks, failures + 1);
        exit(failures + 1);
    }
    return false;
}
static inline absolute_time_t make_timeout_time_us(uint64_t us) { return get_absolute_time() + us; }
static inline bool time_reached(absolute_time_t t) { return get_absolute_time() >= t; }

#include "menus.c"

// Holistic review: core 1's stack canary lives in ATAboy.c and the fault
// stop in fault.c, neither built here (fault.c is Arm assembly).
bool core1_stack_intact(void) { return true; }
void fault_stop(uint32_t code) { printf("FAIL: fault_stop(%08X) reached\n", (unsigned)code); exit(250); }

uint64_t mock_now_ns = 0;
uint32_t mock_gpio_out = 0;
MockSio mock_sio;
SimDrive sim;
config_t config;
int mock_usb_boot_requests = 0;
queue_t cdc_tx_queue, cdc_rx_queue;
volatile bool cdc_connected = true, is_mounted = false, media_changed_waiting = false;
bool tud_msc_set_sense(uint8_t, uint8_t, uint8_t, uint8_t) { return true; }
static int saves = 0;
void config_save(void) { saves++; }
void config_defaults(void) {}
// usb.c's busy flag as core 1 sees it. pend_at_ns: a USB command that ended
// at that time left a reset pending (ide.c's recovery), as one that timed
// out on the drive would.
static uint64_t busy_until_ns = 0, pend_at_ns = 0;
bool usb_msc_ide_busy(void) {
    if (pend_at_ns && mock_now_ns >= pend_at_ns) { rec.stage = REC_SRST; last_fail.pending = true; pend_at_ns = 0; }
    return mock_now_ns < busy_until_ns;
}

#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; \
    printf("FAIL [%s] %s:%d: %s -- ", current, __FILE__, __LINE__, #cond); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)
static const char *current = "";

static uint64_t now_us() { return mock_now_ns / 1000; }

// Keys, 20 ms apart (past get_input's 10 ms escape timeout), starting 1 ms
// from now. \x1b is a lone Esc, \r is Enter, \b is Backspace.
static void keys(const std::string &s, uint64_t start_after_ms = 1) {
    rx.clear(); rxi = 0;
    uint64_t t = now_us() + start_after_ms * 1000;
    for (char c : s) { rx.push_back({ t, c }); t += 20000; }
}

// The CP3044's manual geometry (physical), and a drive to go with it.
static const uint32_t C = 1045, H = 2, S = 40;

static void fresh(const char *name) {
    current = name;
    sim = SimDrive();
    sim.nsect = C * H * S; sim.native_heads = 4; sim.native_spt = 40;
    sim.identify_ok = false;                        // and never asked anyway
    mock_now_ns += 1000000000ull;
    memset(&config, 0, sizeof config);
    config.dev_base = 0xA0;
    ide_select_device(0xA0);
    hdd_model_raw[0] = 0; hdd_status_text[0] = 0;
    cur_cyls = cur_heads = cur_spt = 0; total_lba_sectors = 0; use_lba_mode = false;
    detect_cyls = 0; detect_heads = 0; detect_spt = 0;
    show_detect_result = false; force_detect = false;
    current_screen = SCREEN_MAIN; is_mounted = false;
    rec.stage = REC_NONE; memset(&last_fail, 0, sizeof last_fail);
    chs_geometry_lost = false;
    manual_chs_active = false; iordy_held = false;
    native_geo.on = false; idp_written = false; mchs_no_identify = false;
#ifdef MCHS_MODEL_CLEARED
    native_declared = false;
    memset(&refused, 0, sizeof refused);
#ifdef IDE_SENTINEL_SC
    memset(&self_resets, 0, sizeof self_resets); sentinel_armed = false; sentinel_proven = 0;
#endif
#endif
    sim_on_read = nullptr;
    saves = 0; busy_until_ns = 0; pend_at_ns = 0;
    tty.clear(); rx.clear(); rxi = 0;
    hang_limit_ns = mock_now_ns + 300000000000ull;  // 300 s of mock time
}

// Everything the menu shows or keeps about the drive, to check nothing changed.
struct Snap {
    uint16_t cc; uint8_t ch, cs; bool lba; uint64_t lbas;
    std::string model, status; bool shown, force;
    config_t cfg;
};
static Snap snap() {
    Snap s{ cur_cyls, cur_heads, cur_spt, use_lba_mode, total_lba_sectors,
            hdd_model_raw, hdd_status_text, show_detect_result, force_detect, config };
    return s;
}
static bool same(const Snap &a, const Snap &b) {
    return a.cc == b.cc && a.ch == b.ch && a.cs == b.cs && a.lba == b.lba && a.lbas == b.lbas &&
           a.model == b.model && a.status == b.status && a.shown == b.shown && a.force == b.force &&
           memcmp(&a.cfg, &b.cfg, sizeof a.cfg) == 0;
}
// Some setup already in place (a drive detected earlier), so "unchanged" means something.
static void earlier_setup() {
    strcpy(hdd_model_raw, "WDC AC280"); cur_cyls = 980; cur_heads = 10; cur_spt = 17;
    sync_to_config();
}
static bool nothing_sent() {
    return sim.reg_writes == 0 && sim.commands == 0 && sim.hw_resets == 0 && sim.srst == 0;
}
static bool no_identify() { return sim.cmd_count.find(0xEC) == sim.cmd_count.end(); }
static bool sent_exactly_recal_idp() {
    return sim.cmd_log == std::vector<uint8_t>{ 0x10, 0x91 } && sim.hw_resets == 1 && sim.srst == 0;
}
static bool has(const char *s) { return tty.find(s) != std::string::npos; }

// ---- 1. the pure decisions ---------------------------------------------------
static void test_decisions() {
    current = "decisions";
    CHECK(MANUAL_CHS_KEY == 0x07, "the key is Ctrl+G");
    const int extra[] = { -1, KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_F10, 0x107, -0xF9 };
    std::vector<int> ks;
    for (int k = 0; k < 256; k++) ks.push_back(k);
    for (int k : extra) ks.push_back(k);
    int opened = 0;
    for (int m = 0; m < 2; m++)
        for (int mo = 0; mo < 2; mo++)
            for (int k : ks) {
                bool want = m && !mo && k == 0x07;
                bool got = manual_chs_key_opens(m != 0, mo != 0, k);
                CHECK(got == want, "main %d mounted %d key %d: %d", m, mo, k, got);
                opened += got;
            }
    CHECK(opened == 1, "opened %d times", opened);
    // The key, on every screen of the real menu, mounted or not.
    const screen_t screens[] = { SCREEN_MAIN, SCREEN_FEATURES, SCREEN_CONFIRM, SCREEN_MOUNTED, SCREEN_DEBUG };
    for (screen_t sc : screens)
        for (int mo = 0; mo < 2; mo++) {
            current_screen = sc; is_mounted = mo != 0;
            CHECK(manual_chs_key(0x07) == (sc == SCREEN_MAIN && !mo), "screen %d mounted %d", (int)sc, mo);
            CHECK(!manual_chs_key('g') && !manual_chs_key('G') && !manual_chs_key('M') && !manual_chs_key(0x06),
                  "another key opened it on screen %d", (int)sc);
        }
    current_screen = SCREEN_MAIN; is_mounted = false;
    // Ranges: each boundary, each side.
    struct { uint32_t c, h, s; int bad; } r[] = {
        { 1, 1, 1, -1 }, { 65535, 16, 255, -1 }, { 1045, 2, 40, -1 },
        { 0, 2, 40, 0 }, { 65536, 2, 40, 0 }, { 99999, 2, 40, 0 },
        { 1045, 0, 40, 1 }, { 1045, 17, 40, 1 }, { 1045, 255, 40, 1 }, { 1045, 99999, 40, 1 },
        { 1045, 2, 0, 2 }, { 1045, 2, 256, 2 }, { 1045, 2, 99999, 2 },
        { 0, 0, 0, 0 }, { 1045, 0, 0, 1 }, { 65536, 17, 256, 0 },
    };
    for (auto &x : r) CHECK(manual_chs_bad_field(x.c, x.h, x.s) == x.bad, "%u/%u/%u: %d, want %d",
                            x.c, x.h, x.s, manual_chs_bad_field(x.c, x.h, x.s), x.bad);
    CHECK(strstr(manual_chs_refusal(0), "cylinders must be 1 to 65535") && strstr(manual_chs_refusal(1), "heads must be 1 to 16") &&
          strstr(manual_chs_refusal(2), "sectors per track must be 1 to 255") && !*manual_chs_refusal(-1),
          "refusal texts");
    for (int f = 0; f < 3; f++) CHECK(strstr(manual_chs_refusal(f), "Nothing sent."), "field %d: does not say nothing was sent", f);
}

// ---- 2. accepted ------------------------------------------------------------
static void test_accepted() {
    fresh("accepted");
    earlier_setup();
    keys("1045\r2\r40\ry");
    uint64_t t0 = mock_now_ns;
    bool overlay = run_manual_chs();
    CHECK(!overlay && !show_detect_result, "overlay %d shown %d", overlay, show_detect_result);
    CHECK(no_identify(), "IDENTIFY sent");
    CHECK(sent_exactly_recal_idp(), "commands %zu (first %02X), resets %d, SRST %d", sim.cmd_log.size(),
          sim.cmd_log.empty() ? 0 : sim.cmd_log[0], sim.hw_resets, sim.srst);
    CHECK(sim.short_resets == 0 && sim.min_reset_low >= 50000000ull, "RESET- pulse %llu ns", (unsigned long long)sim.min_reset_low);
    CHECK(sim.geo_valid && sim.heads == H && sim.spt == S, "drive geometry %d %u/%u", sim.geo_valid, sim.heads, sim.spt);
    CHECK(sim.violations == 0 && sim.iordy_stalls == 0, "violations %d, IORDY stalls %d", sim.violations, sim.iordy_stalls);
    CHECK(cur_cyls == C && cur_heads == H && cur_spt == S && !use_lba_mode, "shown %u/%u/%u lba %d", cur_cyls, cur_heads, cur_spt, use_lba_mode);
    CHECK(config.cyls == C && config.heads == H && config.spt == S && !config.use_lba_mode, "config %u/%u/%u", config.cyls, config.heads, config.spt);
    CHECK((uint64_t)config.cyls * config.heads * config.spt == (uint64_t)C * H * S, "capacity");
    CHECK(strcmp(hdd_model_raw, "Manual CHS (no IDENTIFY)") == 0, "Current HDD: \"%s\"", hdd_model_raw);
    CHECK(saves == 0, "EEPROM written %d times", saves);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(f.kind == IDE_FAIL_NONE, "a failure recorded: kind %u", f.kind);
    CHECK(has("Manual CHS (no IDENTIFY)") && has("Y: reset Master, send 1045/2/40 (91h)   N: edit"), "screens");
    CHECK(mock_now_ns - t0 < 5000000000ull, "took %llu ms", (unsigned long long)((mock_now_ns - t0) / 1000000));
#if ATABOY_SAT
    ide_id_words_t w; ide_id_words(&w);
    CHECK(!w.valid, "IDENTIFY words still held for this drive");
#endif
    // A read after it goes to the right sector, with no second 0x91.
    uint8_t buf[512];
    int32_t got = ide_read_sectors(1234, 1, buf);
    bool right = got >= 0;
    for (int i = 0; right && i < 512; i++) right = buf[i] == sim.byte_at(1234, i);
    CHECK(right, "read after manual CHS: %d", (int)got);
    // No 0x91 before the READ (review C round 2: once per mount).
    CHECK(sim.cmd_log == (std::vector<uint8_t>{ 0x10, 0x91, 0x20 }), "commands after the read: %zu", sim.cmd_log.size());
    // The main menu shows it.
    tty.clear(); update_main_menu();
    CHECK(has("Manual CHS (no IDENTIFY)") && has("1045 Cyl / 2 Hd / 40 SPT"), "main menu");
}

// The limits themselves are accepted, and reach the drive as they are.
static void test_limits_accepted() {
    fresh("65535/16/255");
    keys("65535\r16\r255\ry");
    run_manual_chs();
    CHECK(sent_exactly_recal_idp() && sim.heads == 16 && sim.spt == 255 && cur_cyls == 65535, "%u/%u/%u", cur_cyls, sim.heads, sim.spt);
    fresh("1/1/1");
    keys("1\r1\r1\ry");
    run_manual_chs();
    CHECK(sent_exactly_recal_idp() && sim.heads == 1 && sim.spt == 1 && cur_cyls == 1, "%u/%u/%u", cur_cyls, sim.heads, sim.spt);
}

// ---- 3. a drive older than ATA ------------------------------------------------
// No DRDY until it has a geometry, 30 s to spin up (under the CP3044's 40 s
// spin recovery), RECALIBRATE aborted, no INTRQ, and slow to raise BSY: the
// ERR left by RECALIBRATE must not be read as 0x91's answer.
static void test_pre_ata() {
    fresh("pre-ATA drive");
    sim.drdy_needs_idp = true; sim.t_hw_reset = 30000000000ull; sim.abort_recal = true;
    sim.intrq_wired = false; sim.t_bsy_delay = 3000;
    keys("1045\r2\r40\ry");
    bool overlay = run_manual_chs();
    CHECK(!overlay && sim.geo_valid && sim.heads == H && sim.spt == S, "overlay %d, drive %d %u/%u (%s)",
          overlay, sim.geo_valid, sim.heads, sim.spt, hdd_status_text);
    CHECK(sent_exactly_recal_idp() && no_identify(), "commands %zu", sim.cmd_log.size());
    CHECK(cur_cyls == C && cur_heads == H && cur_spt == S, "shown %u/%u/%u", cur_cyls, cur_heads, cur_spt);
}

// ---- 4. refused, and the other ways it can go wrong -------------------------
static void test_refused() {
    fresh("0x91 refused");
    earlier_setup();
    sim.reject_idp = true;
    keys("1045\r2\r40\ry");
    bool overlay = run_manual_chs();
    CHECK(overlay && show_detect_result && !force_detect, "overlay %d shown %d force %d", overlay, show_detect_result, force_detect);
    CHECK(sent_exactly_recal_idp() && no_identify(), "commands %zu", sim.cmd_log.size());
    CHECK(strstr(hdd_status_text, "Manual CHS: drive refused geometry (ST:51)") != nullptr, "\"%s\"", hdd_status_text);
    CHECK(cur_cyls == 0 && cur_heads == 0 && cur_spt == 0 && config.cyls == 0 && config.heads == 0 && config.spt == 0,
          "geometry still shown as set: %u/%u/%u", cur_cyls, cur_heads, cur_spt);
    CHECK((uint64_t)config.cyls * config.heads * config.spt == 0 && !config.use_lba_mode, "capacity not 0");
    CHECK(hdd_model_raw[0] == 0, "Current HDD still \"%s\"", hdd_model_raw);
    CHECK(saves == 0, "EEPROM written");
    ide_fail_t f; ide_last_failure(&f);
    CHECK(f.kind == IDE_FAIL_MANUAL_CHS && f.command == 0x91 && f.status == 0x51 && f.error == 0x04 && !f.pending,
          "record kind %u cmd %02X st %02X err %02X", f.kind, f.command, f.status, f.error);
    tty.clear(); run_debug_errors();
    CHECK(has("[Last Failed I/O] cmd 91, manual CHS: drive refused geometry"), "Debug E");
    CHECK(has("ST:51 ERR:04 SC:28"), "Debug E registers");
    tty.clear(); draw_error_box(hdd_status_text, force_detect);
    CHECK(has("Manual CHS: drive refused geometry (ST:51)") && !has("F: Force"), "error box");
    tty.clear(); update_main_menu();
    CHECK(has("0 Cyl / 0 Hd / 0 SPT"), "Current Geometry");
    // A read now refuses: CHS mode with the geometry lost.
    uint8_t buf[512];
    CHECK(ide_read_sectors(0, 1, buf) < 0, "read with no geometry");

    fresh("0x91 never ends");
    sim.hang_cmd = 0x91;
    keys("1045\r2\r40\ry");
    CHECK(run_manual_chs(), "no result box");
    CHECK(sent_exactly_recal_idp(), "commands %zu", sim.cmd_log.size());
    CHECK(strstr(hdd_status_text, "Manual CHS: drive refused geometry (ST:80)") != nullptr, "\"%s\"", hdd_status_text);
    CHECK(cur_cyls == 0 && config.cyls == 0, "geometry set by a drive that never answered");

    fresh("RECALIBRATE never ends");
    sim.hang_cmd = 0x10;
    keys("1045\r2\r40\ry");
    uint64_t t0 = mock_now_ns;
    CHECK(run_manual_chs(), "no result box");
    CHECK(sim.cmd_log == std::vector<uint8_t>{ 0x10 } && no_identify(), "commands %zu", sim.cmd_log.size());
    CHECK(mock_now_ns - t0 >= 12000000000ull, "gave up after %llu ms", (unsigned long long)((mock_now_ns - t0) / 1000000));
    CHECK(strstr(hdd_status_text, "Manual CHS: RECALIBRATE did not end (ST:80)") != nullptr, "\"%s\"", hdd_status_text);
    CHECK(cur_cyls == 0 && config.cyls == 0, "geometry set");
    ide_fail_t g; ide_last_failure(&g);
    CHECK(g.kind == IDE_FAIL_MANUAL_CHS && g.command == 0x10, "record kind %u cmd %02X", g.kind, g.command);
    CHECK(chs_geometry_lost, "RESET- sent, geometry not marked lost (M-1)");
    tty.clear(); run_debug_errors();
    CHECK(has("[Last Failed I/O] cmd 10, manual CHS: RECALIBRATE did not end"), "Debug E");

    fresh("busy for ever after the reset");
    earlier_setup();
    sim.hw_reset_wedges = true;
    keys("1045\r2\r40\ry");
    t0 = mock_now_ns;
    CHECK(run_manual_chs(), "no result box");
    uint64_t ms = (mock_now_ns - t0) / 1000000;
    CHECK(sim.commands == 0 && sim.hw_resets == 1, "commands %d", sim.commands);
    CHECK(ms >= 2000 + IDE_MCHS_READY_MS && ms < 2000 + IDE_MCHS_READY_MS + 1000, "gave up after %llu ms", (unsigned long long)ms);
    CHECK(strstr(hdd_status_text, "Manual CHS: drive still busy (ST:80)") != nullptr, "\"%s\"", hdd_status_text);
    CHECK(cur_cyls == 0 && config.cyls == 0 && hdd_model_raw[0] == 0, "old geometry kept after RESET-");
    CHECK(chs_geometry_lost, "RESET- sent, geometry not marked lost (M-1)");

    fresh("no drive");
    sim.slave = true;                               // nothing answers as device 0
    keys("1045\r2\r40\ry");
    t0 = mock_now_ns;
    CHECK(run_manual_chs(), "no result box");
    CHECK(sim.commands == 0 && no_identify(), "commands %d", sim.commands);
    // Review C, C-8: FFh is given the same 45 s as BSY before "no drive".
    uint64_t nd_ms = (mock_now_ns - t0) / 1000000;
    CHECK(nd_ms >= 2000 + IDE_MCHS_READY_MS && nd_ms < 2000 + IDE_MCHS_READY_MS + 1000, "took %llu ms", (unsigned long long)nd_ms);
    CHECK(strstr(hdd_status_text, "Manual CHS: no drive answered (ST:FF)") != nullptr, "\"%s\"", hdd_status_text);
    CHECK(chs_geometry_lost, "RESET- sent, geometry not marked lost (M-1)");

    fresh("slave alone");
    sim.slave = true;
    config.dev_base = 0xB0; ide_select_device(0xB0);
    keys("1045\r2\r40\ry");
    CHECK(!run_manual_chs(), "result box: %s", hdd_status_text);
    CHECK(sent_exactly_recal_idp() && sim.geo_valid && sim.violations == 0, "commands %zu violations %d", sim.cmd_log.size(), sim.violations);
    CHECK(has("Slave drive.") && has("Y: reset Slave, send 1045/2/40 (91h)"), "screens do not say Slave");

    // ide.c refuses bad arguments by itself, sending nothing.
    fresh("ide.c arguments");
    uint8_t st = 0x5A;
    int r0 = ide_manual_chs(0, 40, &st), r1 = ide_manual_chs(17, 40, &st), r2 = ide_manual_chs(2, 0, &st);
    CHECK(r0 == IDE_MCHS_BAD_ARGS && r1 == IDE_MCHS_BAD_ARGS && r2 == IDE_MCHS_BAD_ARGS && nothing_sent(),
          "%d %d %d, writes %d", r0, r1, r2, sim.reg_writes);
}

// ---- 5. backing out, and refusals: the drive gets nothing ------------------
static void test_nothing_sent() {
    const char *esc[] = { "\x1b", "10\x1b", "1045\r\x1b", "1045\r2\r4\x1b", "1045\r2\r40\r\x1b",
                          "1045\r2\r40\rn\x1b", "1045\r2\r40\r\x07\x06x\x1b", "\r\r\r\x1b",
                          "1045\r2\r40\r\r\x1b", "1045\r2\r40\r\t \x1b" };
    for (const char *e : esc) {
        fresh("Esc");
        earlier_setup();
        Snap before = snap();
        keys(e);
        bool overlay = run_manual_chs();
        CHECK(!overlay && nothing_sent(), "keys %zu: overlay %d, writes %d, commands %d, resets %d",
              strlen(e), overlay, sim.reg_writes, sim.commands, sim.hw_resets);
        CHECK(same(before, snap()), "keys %zu: something changed", strlen(e));
        CHECK(rxi == rx.size(), "keys %zu: not all keys read", strlen(e));
        CHECK(saves == 0, "EEPROM written");
    }
    struct { const char *k; int field; } bad[] = {
        { "0\r2\r40\r", 0 }, { "65536\r2\r40\r", 0 }, { "99999\r2\r40\r", 0 }, { "\r2\r40\r", 0 },
        { "1045\r0\r40\r", 1 }, { "1045\r17\r40\r", 1 }, { "1045\r\r40\r", 1 }, { "1045\r255\r40\r", 1 },
        { "1045\r2\r0\r", 2 }, { "1045\r2\r256\r", 2 }, { "1045\r2\r\r", 2 }, { "1045\r2\r99999\r", 2 },
        { "\r\r\r", 0 },
    };
    for (auto &b : bad) {
        fresh("refused entry");
        earlier_setup();
        Snap before = snap();
        keys(std::string(b.k) + "y\r\x1b");            // Y and Enter are not a way past it
        bool overlay = run_manual_chs();
        CHECK(!overlay && nothing_sent(), "\"%s\": sent %d writes", b.k, sim.reg_writes);
        CHECK(has(manual_chs_refusal(b.field)), "\"%s\": no \"%s\"", b.k, manual_chs_refusal(b.field));
        CHECK(!has("Y: reset"), "\"%s\": asked to send it", b.k);
        CHECK(same(before, snap()) && saves == 0, "\"%s\": something changed", b.k);
    }
    // Six digits: the sixth is not taken (99999, refused as cylinders).
    fresh("six digits");
    keys("999999\r2\r40\r\x1b");
    run_manual_chs();
    CHECK(has(manual_chs_refusal(0)) && nothing_sent() && has("99999") && !has("999999"), "999999");
    // A refusal, corrected: back at the field it names, then accepted.
    fresh("corrected");
    keys("1045\r17\r40\r\b\b2\r\ry");
    run_manual_chs();
    CHECK(has(manual_chs_refusal(1)) && sent_exactly_recal_idp() && sim.heads == 2 && cur_heads == 2, "heads %u", sim.heads);
    // N goes back to editing; Y then sends it.
    fresh("N then Y");
    keys("1045\r2\r40\rn\ry");
    run_manual_chs();
    CHECK(sent_exactly_recal_idp() && cur_cyls == C, "commands %zu", sim.cmd_log.size());
}

// ---- 6. mounted, pending, busy ----------------------------------------------
static void test_guards() {
    // A reset pending from a USB command: refused at once, nothing drawn but the refusal.
    fresh("pending at entry");
    earlier_setup();
    rec.stage = REC_SRST; last_fail.pending = true;
    Snap before = snap();
    keys("1045\r2\r40\ry");
    bool overlay = run_manual_chs();
    CHECK(!overlay && nothing_sent() && has("Reset still pending, nothing sent. Press a key"), "writes %d", sim.reg_writes);
    CHECK(!has("Manual CHS (no IDENTIFY)"), "the entry opened");
    CHECK(same(before, snap()) && ide_recovery_pending(), "state changed");

    // A USB command that ends while the operator types, leaving a reset pending.
    fresh("pending at Y");
    earlier_setup();
    before = snap();
    pend_at_ns = mock_now_ns + 50000000ull;
    keys("1045\r2\r40\ryx");
    overlay = run_manual_chs();
    CHECK(!overlay && nothing_sent() && has("Reset still pending, nothing sent. Press a key"), "writes %d", sim.reg_writes);
    CHECK(same(before, snap()), "state changed");

    // A USB command still running at Y: waited for, then nothing sent.
    fresh("busy at Y");
    earlier_setup();
    before = snap();
    busy_until_ns = mock_now_ns + 5000000000ull;
    keys("1045\r2\r40\ry");
    rx.push_back({ now_us() + 7000000, 'x' });
    overlay = run_manual_chs();
    CHECK(!overlay && nothing_sent() && has("A USB command was running, nothing sent. Press a key"), "writes %d", sim.reg_writes);
    CHECK(same(before, snap()), "state changed");
    // ...and pressed again, nothing running, it goes.
    busy_until_ns = 0; tty.clear();
    keys("1045\r2\r40\ry");
    run_manual_chs();
    CHECK(sent_exactly_recal_idp() && cur_cyls == C, "again: commands %zu", sim.cmd_log.size());
    CHECK(saves == 0, "EEPROM written");
}

// Auto Detect first (a drive that answers IDENTIFY), then manual CHS on
// the same cable: the words it left must not survive into the new setup.
static void test_after_auto_detect() {
    fresh("after Auto Detect");
    sim.identify_ok = true;
    keys("\x1b");                                    // Esc out of the picker
    run_auto_detect();
    int ids = sim.id_commands();
    CHECK(ids == 1, "Auto Detect sent %d IDENTIFY", ids);
#if ATABOY_SAT
    ide_id_words_t w; ide_id_words(&w);
    CHECK(w.valid, "Auto Detect kept no words (the next check means nothing)");
#endif
    tty.clear();
    keys("1045\r2\r40\ry");
    run_manual_chs();
    CHECK(sim.id_commands() == ids, "manual CHS sent IDENTIFY");
#if ATABOY_SAT
    ide_id_words(&w);
    CHECK(!w.valid, "the words from Auto Detect are still held");
    CHECK(ide_id_words_verify() == 0 && sim.id_commands() == ids, "the identity check IDENTIFYs the drive");
#endif
}

// ---- 8. review of 0.6f3p8 ----------------------------------------------------
// One sector read at lba outside a host command, as test_accepted does. True
// when it came back with the medium's own bytes.
static bool read_right(uint32_t lba, int32_t *got_out = nullptr) {
    uint8_t buf[512];
    int32_t got = ide_read_sectors(lba, 1, buf);
    if (got_out) *got_out = got;
    bool right = got >= 0;
    for (int i = 0; right && i < 512; i++) right = buf[i] == sim.byte_at(lba, i);
    return right;
}
static std::vector<uint8_t> cmds_since(size_t n) {
    return std::vector<uint8_t>(sim.cmd_log.begin() + (long)n, sim.cmd_log.end());
}
static void ctrl_g() {
    keys("1045\r2\r40\ry");
    run_manual_chs();
}
static void debug_reset() {
    current_screen = SCREEN_DEBUG;
    debug_key('R');                                 // ide_reset_drive(): RESET-, RECALIBRATE
    current_screen = SCREEN_MAIN;
}

// Power-up: nothing sent yet, so no geometry is believed. The first CHS read
// sends 0x91 before it names a sector. Runs first, on the state as the
// program starts (M-1: every reset marks the geometry lost, and power-up is one).
static void test_power_up() {
    current = "M-1: power-up";
    config.cyls = C; config.heads = H; config.spt = S; config.use_lba_mode = false;
    sim.nsect = C * H * S; sim.native_heads = 4; sim.native_spt = 40;
    CHECK(read_right(1234), "first read after power-up went to another sector");
    CHECK(!sim.cmd_log.empty() && sim.cmd_log[0] == 0x91, "0x91 not sent first: %zu commands", sim.cmd_log.size());
}

// M-1, the review's P1 (exp2.cpp): Ctrl+G, then Debug R, then a read. On
// bb94c26 the read came back GOOD from another sector: RESET- had dropped the
// drive's 0x91 and nothing marked it. Now the read sends 0x91 again first.
static void test_m1_debug_r() {
    fresh("M-1 P1: Ctrl+G, Debug R, read");
    ctrl_g();
    CHECK(read_right(1234), "read after Ctrl+G");
    debug_reset();
    CHECK(sim.hw_resets == 2 && !sim.geo_valid, "Debug R: resets %d, drive geometry %d", sim.hw_resets, sim.geo_valid);
    size_t m = sim.cmd_log.size();
    CHECK(read_right(1234), "read after Debug R went to another sector");
    CHECK(cmds_since(m) == (std::vector<uint8_t>{ 0x91, 0x20 }) && sim.heads == H && sim.spt == S,
          "0x91 not sent again first: %zu commands", sim.cmd_log.size() - m);
    CHECK(no_identify(), "IDENTIFY sent");

    // The drive refuses 0x91 this time: the read refuses, naming no sector.
    fresh("M-1 P1: Debug R, then 0x91 refused");
    ctrl_g();
    debug_reset();
    sim.reject_idp = true;
    m = sim.cmd_log.size();
    int32_t got = 0;
    read_right(1234, &got);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(got < 0 && cmds_since(m) == std::vector<uint8_t>{ 0x91 } && f.kind == IDE_FAIL_NO_GEOMETRY,
          "read %d, %zu commands, record %u", (int)got, sim.cmd_log.size() - m, f.kind);
}

// M-1, the review's P2: Ctrl+G, then Auto Detect (the drive answers IDENTIFY),
// then Esc in the picker. On bb94c26 the Ctrl+G geometry was kept against a
// drive the probe had reset, and a mount read the wrong sectors. Now the
// geometry is cleared at the probe, so there is nothing to mount; and ide.c
// on its own would have sent 0x91 again first.
static void test_m1_auto_detect_esc() {
    // Review C, C-3: Auto Detect after a Ctrl+G is refused outright now,
    // nothing sent, nothing changed.
    fresh("M-1 P2: Ctrl+G, then Auto Detect is refused");
    sim.identify_ok = true;
    ctrl_g();
    CHECK(cur_cyls == C && ide_manual_chs_active(), "Ctrl+G did not set it");
    size_t m0 = sim.cmd_log.size();
    int w0 = sim.reg_writes, hw0 = sim.hw_resets;
    tty.clear();
    keys("x\x1b\x1b");   // x for the refusal; Esc, Esc if Auto Detect ran
    CHECK(!run_auto_detect() && sim.id_commands() == 0 && sim.cmd_log.size() == m0 && sim.reg_writes == w0 && sim.hw_resets == hw0,
          "Auto Detect after Ctrl+G: %d IDENTIFY, %zu commands", sim.id_commands(), sim.cmd_log.size() - m0);
    CHECK(has(AUTODETECT_REFUSED_MCHS) && cur_cyls == C && ide_manual_chs_active(), "not refused, or state changed");
    // What the probe itself does is unchanged (the old test's ide.c half).
    fresh("M-1 P2: the probe clears nothing it should not");
    ctrl_g();
    ide_probe_devices();
    cur_cyls = 0; cur_heads = 0; cur_spt = 0; sync_to_config();     // as run_auto_detect clears it
    CHECK(config.cyls == 0 && config.heads == 0 && config.spt == 0, "config kept it: %u/%u/%u", config.cyls, config.heads, config.spt);
    // Mount Drive's own test (core1_entry, main menu item 1): nothing to mount.
    bool gv = (use_lba_mode && total_lba_sectors > 0) || (!use_lba_mode && cur_cyls > 0 && cur_heads > 0 && cur_spt > 0);
    CHECK(!gv && saves == 0, "a geometry to mount, or the EEPROM written");
    // A read regardless (usb.c would refuse it first: nothing is mounted)
    // sends nothing: there is no geometry to send.
    size_t m = sim.cmd_log.size();
    int32_t got = 0;
    read_right(1234, &got);
    CHECK(got < 0 && sim.cmd_log.size() == m, "read with no geometry: %d, %zu commands", (int)got, sim.cmd_log.size() - m);
    // ide.c's guard alone: the old geometry put back as bb94c26 kept it. The
    // probe's RESET- marked it lost, so 0x91 goes first and the read is right.
    config.cyls = C; config.heads = H; config.spt = S;
    m = sim.cmd_log.size();
    CHECK(read_right(1234), "read after Auto Detect went to another sector");
    CHECK(cmds_since(m) == (std::vector<uint8_t>{ 0x91, 0x20 }), "0x91 not sent again first: %zu commands",
          sim.cmd_log.size() - m);

    // The drive refuses IDENTIFY (the CP3044 case): the error box with F:
    // Force, and the Ctrl+G geometry is not kept for the forced entry either.
}

// N5: Ctrl+G after an Auto Detect that chose LBA must leave LBA mode, or the
// next reads send LBA addresses to a drive set up in CHS. This drive takes no
// notice of the LBA bit, as a drive older than LBA would, so an LBA read would
// come back GOOD from another sector.
static void test_mchs_after_lba_detect() {
    fresh("Ctrl+G after an LBA Auto Detect");
    sim.identify_ok = true;
    sim.lba_ignored = true;
    keys("\r");                                     // the picker offers LBA first for an LBA drive
    run_auto_detect();
    CHECK(use_lba_mode && config.use_lba_mode && total_lba_sectors > 0, "Auto Detect did not choose LBA");
    ctrl_g();
    CHECK(!use_lba_mode && !config.use_lba_mode && total_lba_sectors == 0 && config.lba_sectors == 0,
          "LBA mode kept: %d %d %llu", use_lba_mode, config.use_lba_mode, (unsigned long long)config.lba_sectors);
    CHECK(cur_cyls == C && config.heads == H && config.spt == S, "geometry %u/%u/%u", cur_cyls, config.heads, config.spt);
    size_t m = sim.cmd_log.size();
    CHECK(read_right(1234), "read went to another sector");
    CHECK(cmds_since(m) == std::vector<uint8_t>{ 0x20 } && !(sim.reg[6] & 0x40),
          "not a CHS read: device register %02X", sim.reg[6]);
}

// N3: Enter at the Y/N question is not Y (the same keys are in
// test_nothing_sent too; here with Tab and Space as well).
static void test_mchs_enter_not_yes() {
    fresh("Enter at the question");
    earlier_setup();
    Snap before = snap();
    keys("1045\r2\r40\r\r\t \x1b");
    CHECK(!run_manual_chs() && nothing_sent() && same(before, snap()) && rxi == rx.size(),
          "writes %d commands %d", sim.reg_writes, sim.commands);
}

// N7: with IORDY on in Features, it is ignored from Ctrl+G's RESET- until the
// drive is past its power-on diagnostics (a drive holds IORDY low through
// them, and a register cycle that believed it would never end), then believed
// again.
static void test_mchs_iordy_held() {
    fresh("Ctrl+G with IORDY on");
    sim.t_hw_reset = 5000000000ull;                 // diagnostics outlast the 2 s wait
    config.iordy_enabled = true;
    ide_iordy_follow_config();
    CHECK(mock_iordy_inover == GPIO_OVERRIDE_NORMAL, "IORDY not believed before");
    ctrl_g();
    CHECK(sent_exactly_recal_idp() && cur_cyls == C, "commands %zu", sim.cmd_log.size());
    CHECK(sim.iordy_stalls == 0, "%d register cycles with IORDY believed in the diagnostics", sim.iordy_stalls);
    // Review C, C-7: never believed again for a Ctrl+G drive.
    CHECK(mock_iordy_inover == GPIO_OVERRIDE_HIGH, "IORDY believed after Ctrl+G");
    ide_iordy_follow_config();
    ide_host_cmd_enter(); ide_host_cmd_leave();
    CHECK(mock_iordy_inover == GPIO_OVERRIDE_HIGH, "IORDY believed at a host command after Ctrl+G");
    config.iordy_enabled = false;
    mchs_no_identify = false;
    ide_iordy_follow_config();
}

// L-2's flag (ide_manual_chs_active): set by an accepted 0x91, kept over a
// reset that asks no IDENTIFY, cleared only by an IDENTIFY that answers.
static void test_mchs_flag() {
    fresh("Ctrl+G flag: set, kept by Debug R");
    CHECK(!ide_manual_chs_active(), "set before Ctrl+G");
    ctrl_g();
    CHECK(ide_manual_chs_active(), "not set by an accepted 0x91");
    debug_reset();
    CHECK(ide_manual_chs_active(), "cleared by Debug R");
    current_screen = SCREEN_DEBUG; debug_key('I'); current_screen = SCREEN_MAIN;   // IDENTIFY refused
    CHECK(ide_manual_chs_active(), "cleared by an IDENTIFY that failed");

    fresh("Ctrl+G flag: cleared by an IDENTIFY that answers (Auto Detect's)");
    sim.identify_ok = true;
    ctrl_g();
    current_screen = SCREEN_DEBUG; debug_key('i'); current_screen = SCREEN_MAIN;
    CHECK(ide_manual_chs_active() && sim.id_commands() == 0, "Debug I sent IDENTIFY after Ctrl+G (review of p10, M-1)");
    // Review C, C-3: Auto Detect is refused after a Ctrl+G, so only an
    // ATAboy power cycle ends it (fresh() here).
    keys("x\x1b\x1b");   // x for the refusal; Esc, Esc if Auto Detect ran
    run_auto_detect();
    CHECK(ide_manual_chs_active() && sim.id_commands() == 0, "Auto Detect ran after Ctrl+G");

    fresh("Ctrl+G flag: not set by a refused 0x91");
    sim.reject_idp = true;
    ctrl_g();
    CHECK(!ide_manual_chs_active(), "set by a refused 0x91");
}

// L-1: F10 does not save a Ctrl+G geometry with Auto Mount on (its power-up
// IDENTIFY would reach the drive), and does otherwise.
static void test_save_setup() {
    fresh("F10, Ctrl+G geometry, Auto Mount on");
    ctrl_g();
    config.auto_mount = true;
    tty.clear();
    keys("x");
    CHECK(!save_setup() && saves == 0 && has(SAVE_REFUSED_MCHS), "saved %d", saves);

    fresh("F10, Ctrl+G geometry, Auto Mount off");
    ctrl_g();
    CHECK(save_setup() && saves == 1 && config.cyls == C && config.heads == H && config.spt == S, "saves %d", saves);

    fresh("F10, Auto Mount on, after an Auto Detect that answered");
    sim.identify_ok = true;
    keys("\r");
    run_auto_detect();
    config.auto_mount = true;
    CHECK(save_setup() && saves == 1 && config.use_lba_mode, "saves %d", saves);
}

// ---- 10. 0.6f3p10: the power-up default mode (no 0x91 at all) -----------------
// Ctrl+G pressed again in the box selects it; the drive is then sent RESET-
// and RECALIBRATE and never INITIALIZE DEVICE PARAMETERS, at the mount or
// after any reset. The simulated drive's power-up default is its native
// translation (sim.native_heads/native_spt, used whenever no 0x91 is in
// force), here the CP3044's documented 522/4/40.
static const uint32_t NC = 522, NH = 4, NS = 40;
static int idp_sent() { auto it = sim.cmd_count.find(0x91); return it == sim.cmd_count.end() ? 0 : it->second; }
static void native_drive(const char *name) {
    fresh(name);
    sim.nsect = NC * NH * NS; sim.native_heads = NH; sim.native_spt = NS;
}
static void ctrl_g_native(const char *geo = "522\r4\r40\r") {
    keys(std::string("\x07") + geo + "y");
    run_manual_chs();
}
// A host command as usb.c brackets one (20 s budget), reading one sector.
static int32_t host_read(uint32_t lba, bool *right) {
    uint8_t buf[512];
    ide_host_cmd_begin(); ide_host_cmd_enter();
    int32_t got = ide_read_sectors(lba, 1, buf);
    ide_host_cmd_leave();
    bool ok = got >= 0;
    for (int i = 0; ok && i < 512; i++) ok = buf[i] == sim.byte_at(lba, i);
    if (right) *right = ok;
    return got;
}

static void test_native_accepted() {
    native_drive("native: accepted");
    earlier_setup();
    keys("\x07" "522\r4\r40\ry");
    bool overlay = run_manual_chs();
    CHECK(!overlay && !show_detect_result, "overlay %d (%s)", overlay, hdd_status_text);
    CHECK(sim.cmd_log == std::vector<uint8_t>{ 0x10 } && sim.hw_resets == 1 && sim.srst == 0,
          "commands %zu (first %02X), resets %d, SRST %d", sim.cmd_log.size(),
          sim.cmd_log.empty() ? 0 : sim.cmd_log[0], sim.hw_resets, sim.srst);
    CHECK(idp_sent() == 0 && sim.init_params == 0 && !sim.geo_valid, "0x91 sent: %d", idp_sent());
    CHECK(no_identify(), "IDENTIFY sent");
    CHECK(cur_cyls == NC && cur_heads == NH && cur_spt == NS && !use_lba_mode, "shown %u/%u/%u", cur_cyls, cur_heads, cur_spt);
    CHECK(config.cyls == NC && config.heads == NH && config.spt == NS && !config.use_lba_mode, "config %u/%u/%u",
          config.cyls, config.heads, config.spt);
    CHECK(strcmp(hdd_model_raw, "Manual CHS (no IDENTIFY) no 91h") == 0, "Current HDD: \"%s\"", hdd_model_raw);
    CHECK(strncmp(hdd_model_raw, MCHS_MODEL, strlen(MCHS_MODEL)) == 0, "Current HDD does not start with the old text");
    CHECK(ide_manual_chs_active() && ide_manual_chs_native_active(), "flags %d %d",
          ide_manual_chs_active(), ide_manual_chs_native_active());
    CHECK(saves == 0, "EEPROM written %d times", saves);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(f.kind == IDE_FAIL_NONE, "a failure recorded: kind %u", f.kind);
    CHECK(has("Manual CHS (no IDENTIFY) - power-up default, no 91h") && has("Ctrl+G: 91h mode") &&
          has("Only if C/H/S IS the drive's power-up default.") &&
          has("Y: reset Master, keep 522/4/40, no 91h   N: edit") &&
          has("Resetting the drive, then 10h only (no 91h). Up to a minute."), "native screens");
    CHECK(!has("(91h)   N: edit") && !has("then 10h and 91h"), "the old mode's question or working line shown");
#if ATABOY_SAT
    ide_id_words_t w; ide_id_words(&w);
    CHECK(!w.valid, "IDENTIFY words still held for this drive");
#endif
    size_t m = sim.cmd_log.size();
    CHECK(read_right(1234), "read after the native mount went to another sector");
    CHECK(cmds_since(m) == std::vector<uint8_t>{ 0x20 }, "commands for the read: %zu", sim.cmd_log.size() - m);
    tty.clear(); update_main_menu();
    CHECK(has("Manual CHS (no IDENTIFY) no 91h") && has("522 Cyl / 4 Hd / 40 SPT"), "main menu");
}

// The mode is chosen in the box and nowhere else: Ctrl+G twice is the old
// mode again, byte for byte; Esc in the new mode sends nothing; Ctrl+G at the
// question changes nothing.
static void test_native_selection() {
    native_drive("native: Ctrl+G twice is the old mode");
    keys("\x07" "52\x07" "2\r4\r40\ry");
    run_manual_chs();
    CHECK(sent_exactly_recal_idp() && sim.geo_valid && sim.heads == NH && sim.spt == NS, "commands %zu", sim.cmd_log.size());
    CHECK(strcmp(hdd_model_raw, MCHS_MODEL) == 0 && !ide_manual_chs_native_active(), "\"%s\"", hdd_model_raw);
    CHECK(has("Y: reset Master, send 522/4/40 (91h)   N: edit"), "old question not shown");

    native_drive("native: Esc sends nothing");
    earlier_setup();
    Snap before = snap();
    keys("\x07" "522\r4\r40\r\x1b");
    CHECK(!run_manual_chs() && nothing_sent() && same(before, snap()), "writes %d", sim.reg_writes);

    native_drive("native: Ctrl+G at the question does nothing");
    keys("522\r4\r40\r\x07y");
    run_manual_chs();
    CHECK(sent_exactly_recal_idp() && !ide_manual_chs_native_active(), "commands %zu", sim.cmd_log.size());

    native_drive("native: selected only for this Ctrl+G");
    ctrl_g_native();
    CHECK(ide_manual_chs_native_active(), "native not set");
    keys("522\r4\r40\ry");                          // the next Ctrl+G, not selected
    run_manual_chs();
    CHECK(cmds_since(1) == (std::vector<uint8_t>{ 0x10, 0x91 }) && !ide_manual_chs_native_active() &&
          strcmp(hdd_model_raw, MCHS_MODEL) == 0, "commands %zu", sim.cmd_log.size());
    // ...and the old mode's rule is back: after Debug R the read sends 0x91 first.
    debug_reset();
    size_t m = sim.cmd_log.size();
    CHECK(read_right(1234) && cmds_since(m) == (std::vector<uint8_t>{ 0x91, 0x20 }), "old mode after native: %zu",
          sim.cmd_log.size() - m);
}

// Every reset path, and 0x91 never goes out.
static void test_native_resets() {
    // Debug R (RESET-, RECALIBRATE), then reads: through chs_geometry_ok.
    native_drive("native: Debug R");
    ctrl_g_native();
    debug_reset();
    CHECK(sim.hw_resets == 2, "Debug R: %d resets", sim.hw_resets);
    size_t m = sim.cmd_log.size();
    CHECK(read_right(1234) && read_right(NC * NH * NS - 1), "read after Debug R went to another sector");
    CHECK(cmds_since(m) == (std::vector<uint8_t>{ 0x20, 0x20 }) && idp_sent() == 0, "commands %zu, 0x91 %d",
          sim.cmd_log.size() - m, idp_sent());
    CHECK(ide_manual_chs_native_active() && !chs_geometry_lost, "native %d lost %d", ide_manual_chs_native_active(), chs_geometry_lost);

    // A read that hangs: soft reset and recovery (soft_reset_restore, recovery_run).
    native_drive("native: soft reset after a hung read");
    ctrl_g_native();
    sim.bad[4321] = {BAD_HANG, 0};
    int32_t got = 0;
    read_right(4321, &got);
    ide_fail_t f; ide_last_failure(&f);
    CHECK(got < 0 && sim.srst == 1 && f.kind == IDE_FAIL_TIMEOUT && f.reset && !f.reset_failed && !f.pending,
          "read %d srst %d kind %u reset %d failed %d pending %d", (int)got, sim.srst, f.kind, f.reset, f.reset_failed, f.pending);
    CHECK(idp_sent() == 0 && !ide_recovery_pending(), "0x91 %d, pending %d", idp_sent(), ide_recovery_pending());
    CHECK(read_right(1234) && idp_sent() == 0, "read after the soft reset: 0x91 %d", idp_sent());

    // SRST does not bring it back, RESET- does (hw_reset_start, RECALIBRATE).
    native_drive("native: hardware reset in the recovery");
    ctrl_g_native();
    sim.srst_wedges = true;
    sim.bad[4321] = {BAD_HANG, 0};
    read_right(4321, &got);
    ide_last_failure(&f);
    CHECK(got < 0 && sim.srst == 1 && sim.hw_resets == 2 && f.hw_reset && !f.reset_failed,
          "srst %d resets %d hw %d failed %d", sim.srst, sim.hw_resets, f.hw_reset, f.reset_failed);
    CHECK(idp_sent() == 0 && !ide_recovery_pending(), "0x91 %d, pending %d", idp_sent(), ide_recovery_pending());
    CHECK(read_right(1234) && idp_sent() == 0, "read after the hardware reset: 0x91 %d", idp_sent());
    CHECK(sim.cmd_log.front() == 0x10 && std::count(sim.cmd_log.begin(), sim.cmd_log.end(), (uint8_t)0x10) == 2,
          "RECALIBRATE after the hardware reset missing");

    // The same inside host commands: the recovery outlasts the first
    // command's 20 s and is carried on by the next ones (recovery_gate).
    native_drive("native: recovery carried over host commands");
    ctrl_g_native();
    sim.srst_wedges = true;
    sim.bad[4321] = {BAD_HANG, 0};
    bool right = false;
    CHECK(host_read(4321, &right) < 0, "hung read succeeded");
    int tries = 0;
    do { got = host_read(1234, &right); tries++; } while (got < 0 && tries < 6);
    CHECK(got >= 0 && right && tries > 1, "read after the carried-over recovery: %d, right %d, %d commands", (int)got, right, tries);
    CHECK(idp_sent() == 0 && sim.hw_resets == 2 && !ide_recovery_pending(), "0x91 %d, resets %d", idp_sent(), sim.hw_resets);

    // The control for the three above: the old mode, same drive, same faults,
    // does send 0x91 after each (so the counts above can see one).
    native_drive("old mode control: soft reset");
    keys("522\r4\r40\ry"); run_manual_chs();
    int base = idp_sent();
    sim.bad[4321] = {BAD_HANG, 0};
    read_right(4321, &got);
    CHECK(idp_sent() == base + 1, "old mode: 0x91 not sent after the soft reset (%d)", idp_sent() - base);
    native_drive("old mode control: recovery over host commands");
    keys("522\r4\r40\ry"); run_manual_chs();
    base = idp_sent();
    sim.srst_wedges = true;
    sim.bad[4321] = {BAD_HANG, 0};
    host_read(4321, &right);
    tries = 0;
    do { got = host_read(1234, &right); tries++; } while (got < 0 && tries < 6);
    CHECK(got >= 0 && right && idp_sent() > base, "old mode: read %d, 0x91 %d", (int)got, idp_sent() - base);
}

// What the mode forgets it on: Auto Detect's probe, and anything that is not
// the geometry it was set with.
static void test_native_forgotten() {
    native_drive("native: forgotten by the probe");
    ctrl_g_native();
    ide_probe_devices();                            // Auto Detect's RESET- and RECALIBRATE
    CHECK(!ide_manual_chs_native_active(), "native kept over a probe");
    config.cyls = NC; config.heads = NH; config.spt = NS; // the same geometry put back by hand
    size_t m = sim.cmd_log.size();
    CHECK(read_right(1234) && cmds_since(m) == (std::vector<uint8_t>{ 0x91, 0x20 }), "0x91 not sent after the probe: %zu",
          sim.cmd_log.size() - m);

    native_drive("native: another geometry in use");
    ctrl_g_native();
    config.heads = 2;                               // not the one declared
    CHECK(!ide_manual_chs_native_active(), "native with other heads");
    config.heads = NH; config.spt = 17;             // review of p10, I-1
    CHECK(!ide_manual_chs_native_active(), "native with other sectors per track");
    debug_reset();
    size_t m2 = sim.cmd_log.size();
    int32_t got2 = 0;
    read_right(0, &got2);
#ifdef MCHS_MODEL_CLEARED
    // Re-review (N-1): still the drive declared native: refused, no 0x91.
    CHECK(got2 < 0 && idp_sent() == 0 && sim.cmd_log.size() == m2,
          "other sectors per track: %d, 0x91 %d", (int)got2, idp_sent());
#endif
    config.spt = NS;
    config.use_lba_mode = true;
    CHECK(!ide_manual_chs_native_active(), "native in LBA mode");
    config.use_lba_mode = false; ide_select_device(0xB0);
    CHECK(!ide_manual_chs_native_active(), "native on the other device");
    ide_select_device(0xA0);
    CHECK(ide_manual_chs_native_active(), "native not back with the same geometry");
}

// It can fail, and then nothing is set.
static void test_native_not_ready() {
    native_drive("native: DRDY only after 0x91");
    earlier_setup();
    sim.drdy_needs_idp = true;
    keys("\x07" "522\r4\r40\ry");
    CHECK(run_manual_chs(), "no result box");
    CHECK(sim.cmd_log == std::vector<uint8_t>{ 0x10 } && idp_sent() == 0, "commands %zu", sim.cmd_log.size());
    CHECK(strstr(hdd_status_text, "Manual CHS: drive not ready, no 91h (ST:10)") != nullptr, "\"%s\"", hdd_status_text);
    CHECK(cur_cyls == 0 && config.cyls == 0 && hdd_model_raw[0] == 0 && !ide_manual_chs_native_active() &&
          !ide_manual_chs_active(), "geometry set");
    CHECK(chs_geometry_lost, "geometry not lost");
    ide_fail_t f; ide_last_failure(&f);
    CHECK(f.kind == IDE_FAIL_MCHS_NOT_READY && f.command == 0x10 && f.status == 0x10, "record kind %u cmd %02X st %02X",
          f.kind, f.command, f.status);
    tty.clear(); run_debug_errors();
    CHECK(has("[Last Failed I/O] cmd 10, manual CHS: not ready after 10h (no 91h)"), "Debug E");

    native_drive("native: RECALIBRATE aborted");
    sim.abort_recal = true;
    keys("\x07" "522\r4\r40\ry");
    CHECK(run_manual_chs(), "no result box");
    CHECK(sim.cmd_log == std::vector<uint8_t>{ 0x10 } && idp_sent() == 0, "commands %zu", sim.cmd_log.size());
    CHECK(strstr(hdd_status_text, "Manual CHS: drive not ready, no 91h (ST:51)") != nullptr, "\"%s\"", hdd_status_text);

    native_drive("native: RECALIBRATE never ends");
    sim.hang_cmd = 0x10;
    keys("\x07" "522\r4\r40\ry");
    CHECK(run_manual_chs(), "no result box");
    CHECK(strstr(hdd_status_text, "Manual CHS: RECALIBRATE did not end (ST:80)") != nullptr, "\"%s\"", hdd_status_text);
    tty.clear(); run_debug_errors();
    CHECK(has("[Last Failed I/O] cmd 10, manual CHS: RECALIBRATE did not end"), "Debug E");
}

// The trade-off, as a fact about the firmware: nothing checks the operator's
// declaration. Declared 1045/2/40 on a drive whose power-up default is 4/40,
// a read comes back GOOD from another sector. (The old mode sends 2/40 as
// 0x91; what a CP3044 does with that is not known either: FORK-README 27.)
static void test_native_trusts_the_declaration() {
    native_drive("native: a wrong declaration is not caught");
    sim.nsect = 1045 * 2 * 40;
    ctrl_g_native("1045\r2\r40\r");
    CHECK(ide_manual_chs_native_active() && idp_sent() == 0, "native not set");
    int32_t got = 0;
    bool right = read_right(1234, &got);
    CHECK(got >= 0 && !right, "read %d right %d: the simulated drive should have answered from another sector",
          (int)got, right);
}

// ---- 11. review of 0.6f3p10 ----------------------------------------------------
static void debug(int k) { current_screen = SCREEN_DEBUG; debug_key(k); current_screen = SCREEN_MAIN; }

// M-1: after a Ctrl+G, either mode, accepted or not, Debug I and Debug S
// send nothing (the seek test starts with IDENTIFY); ide_identify() itself
// refuses. Auto Detect ends it.
static void test_p10_m1_no_identify() {
    const char *modes[] = { "522\r4\r40\ry", "\x07" "522\r4\r40\ry" };
    for (int native = 0; native < 2; native++)
        for (int fail = 0; fail < 2; fail++) {
            native_drive(native ? (fail ? "M-1: native, not ready" : "M-1: native") : (fail ? "M-1: old, refused" : "M-1: old"));
            sim.identify_ok = true;
            if (fail) { if (native) sim.drdy_needs_idp = true; else sim.reject_idp = true; }
            keys(modes[native]);
            run_manual_chs();
            CHECK(fail ? cur_cyls == 0 : cur_cyls == NC, "mount %u", cur_cyls);
            int w = sim.reg_writes, c = sim.commands;
            tty.clear(); debug('I'); debug('i'); debug('S'); debug('s');
            CHECK(sim.id_commands() == 0 && sim.reg_writes == w && sim.commands == c,
                  "Debug I/S sent %d IDENTIFY, %d writes", sim.id_commands(), sim.reg_writes - w);
            CHECK(has(DEBUG_REFUSED_MCHS), "no refusal shown");
            uint16_t id[256];
            CHECK(!ide_identify(id) && sim.id_commands() == 0 && sim.reg_writes == w, "ide_identify sent it");
            CHECK(ide_manual_chs_active() == !fail && ide_manual_chs_native_active() == (native && !fail),
                  "flags changed: %d %d", ide_manual_chs_active(), ide_manual_chs_native_active());
            // The other debug keys still work (R, T, E).
            debug('T'); debug('E');
            CHECK(sim.id_commands() == 0, "T or E sent IDENTIFY");
        }
    // Review C, C-3: Auto Detect does not end it any more; it is refused.
    native_drive("M-1: Auto Detect refused too");
    sim.identify_ok = true;
    ctrl_g_native();
    keys("x\x1b\x1b");   // x for the refusal; Esc, Esc if Auto Detect ran
    run_auto_detect();
    CHECK(sim.id_commands() == 0 && ide_mchs_no_identify(), "Auto Detect: %d", sim.id_commands());
    debug('I');
    CHECK(sim.id_commands() == 0, "Debug I after Auto Detect: %d", sim.id_commands());
}

// M-2: an IDENTIFY that answers (only Auto Detect's can, now) ends both
// Ctrl+G flags together, so native mode is never left on with READ LONG
// and pass-through unlocked. Forced here past M-1's refusal.
static void test_p10_m2_consistent() {
    native_drive("M-2: an IDENTIFY that answers ends native too");
    sim.identify_ok = true;
    ctrl_g_native();
    mchs_no_identify = false;                       // as if M-1 were bypassed
    uint16_t id[256];
    CHECK(ide_identify(id), "IDENTIFY did not answer");
    CHECK(!ide_manual_chs_active() && !ide_manual_chs_native_active(), "native %d with the Ctrl+G flag %d",
          ide_manual_chs_native_active(), ide_manual_chs_active());
    // ...and while M-1 holds, nothing unlocks: flag set, words and word 22 not held.
    native_drive("M-2: locked while the Ctrl+G mode is shown");
    sim.identify_ok = true;
    ctrl_g_native();
    debug('I');
    uint16_t w22 = 0;
    CHECK(ide_manual_chs_active() && ide_manual_chs_native_active() && !ide_read_long_word22(&w22), "unlocked");
#if ATABOY_SAT
    ide_id_words_t w; ide_id_words(&w);
    CHECK(!w.valid, "IDENTIFY words held");
#endif
}

// L-1: unmount ends native mode and its geometry; Current HDD still says
// Ctrl+G; Mount has nothing to mount; a read sends nothing.
static void test_p10_l1_unmount() {
    native_drive("L-1: unmount ends native mode");
    ctrl_g_native();
    is_mounted = true;
    menu_unmount();
    CHECK(!ide_manual_chs_native_active() && !native_geo.on, "native kept over unmount");
    CHECK(cur_cyls == 0 && cur_heads == 0 && cur_spt == 0 && config.heads == 0 && config.spt == 0, "geometry kept %u/%u/%u",
          cur_cyls, cur_heads, cur_spt);
    CHECK(strcmp(hdd_model_raw, MCHS_MODEL_NATIVE) == 0 && ide_manual_chs_active() && ide_mchs_no_identify(),
          "Ctrl+G marks lost: \"%s\"", hdd_model_raw);
    bool gv = (use_lba_mode && total_lba_sectors > 0) || (!use_lba_mode && cur_cyls > 0 && cur_heads > 0 && cur_spt > 0);
    CHECK(!gv, "something to mount");
    // Marked lost as well, or a read would be worked out with no geometry
    // (a division by zero on the host; on the RP2350 a wrong sector).
    CHECK(chs_geometry_lost, "geometry not marked lost at unmount");
    size_t m = sim.cmd_log.size();
    int32_t got = -1;
    if (chs_geometry_lost) read_right(1234, &got);
    CHECK(got < 0 && sim.cmd_log.size() == m && idp_sent() == 0, "read after unmount: %d, %zu commands",
          (int)got, sim.cmd_log.size() - m);
    // The old mode is unchanged by it (0.6f3p9 behaviour).
    native_drive("L-1: unmount leaves the old mode as it was");
    keys("522\r4\r40\ry"); run_manual_chs();
    is_mounted = true;
    menu_unmount();
    CHECK(cur_cyls == NC && config.heads == NH, "old-mode geometry cleared");
}

// H-1: after this firmware has sent 91h since boot, the power-up default
// mode is refused: in the box (not selected, reason shown), and by ide.c,
// which sends nothing.
static void test_p10_h1_idp_since_boot() {
    native_drive("H-1: native refused after an old-mode 91h");
    keys("522\r4\r40\ry"); run_manual_chs();             // old mode: 91h 4/40
    CHECK(ide_idp_sent_since_boot(), "0x91 sent, flag not set");
    size_t m = sim.cmd_log.size();
    int hw = sim.hw_resets;
    tty.clear();
    keys("\x07" "522\r4\r40\r\x1b");
    CHECK(!run_manual_chs() && sim.cmd_log.size() == m && sim.hw_resets == hw, "sent something");
    CHECK(has(MCHS_NATIVE_REFUSED) && !has("power-up default, no 91h"), "native offered after 91h");
    uint8_t st = 0;
    int r = ide_manual_chs_native(NH, NS, &st);
    CHECK(r == IDE_MCHS_IDP_SENT && sim.cmd_log.size() == m && sim.hw_resets == hw && ide_manual_chs_active(),
          "ide.c: %d, %zu commands", r, sim.cmd_log.size() - m);

    native_drive("H-1: a 91h after a reset counts too");
    ctrl_g_native();
    CHECK(!ide_idp_sent_since_boot(), "native mount set it");
    native_drive("H-1: a 91h after a reset counts too (saved geometry, no Ctrl+G)");
    config.cyls = NC; config.heads = 2; config.spt = 17;
    chs_geometry_lost = true;                       // as at power-up
    read_right(0);
    CHECK(ide_idp_sent_since_boot() && idp_sent() == 1, "0x91 from ide_set_geometry not counted");

    native_drive("H-1: native first is fine, and the old mode after it is not refused");
    ctrl_g_native();
    ctrl_g_native();
    CHECK(ide_manual_chs_native_active() && idp_sent() == 0, "second native refused");
    keys("522\r4\r40\ry"); run_manual_chs();
    CHECK(idp_sent() == 1 && !ide_manual_chs_native_active(), "old mode refused");
}

// ---- 12. re-review of 0.6f3p10 -------------------------------------------------
// N-1: an unmount while a USB read is still in its reset recovery. The read
// (core 0) is the one below; the unmount (core 1) is run from a register
// read once the soft reset has gone out, with usb.c's busy flag up.
static int n1_fired;
static void n1_unmount() {
    if (sim.srst < 1) return;
    sim_on_read = nullptr;
    n1_fired++;
    busy_until_ns = ~0ull;                          // the read is still running
    is_mounted = false;
    menu_unmount();
}
static void test_p10_n1_unmount_during_recovery() {
    native_drive("N-1: unmount during a read's recovery");
    ctrl_g_native();
    is_mounted = true;
    sim.srst_wedges = true;                         // SRST does not bring it back: RESET-, RECALIBRATE
    sim.bad[4321] = {BAD_HANG, 0};
    n1_fired = 0;
    sim_on_read = n1_unmount;
    int32_t got = 0;
    read_right(4321, &got);
    busy_until_ns = 0;
    CHECK(n1_fired == 1 && got < 0 && sim.srst == 1 && sim.hw_resets == 2, "fired %d, srst %d, resets %d",
          n1_fired, sim.srst, sim.hw_resets);
    CHECK(idp_sent() == 0 && !ide_idp_sent_since_boot(), "0x91 sent during the recovery: %d", idp_sent());
    // The unmount waited for the read: it could not end the mode or clear
    // config under it, and the recovery finished with the geometry intact.
    ide_fail_t f1; ide_last_failure(&f1);
    CHECK(ide_manual_chs_native_active() && config.heads == NH && config.spt == NS && !f1.reset_failed,
          "mode %d, config %u/%u, reset failed %d", ide_manual_chs_native_active(), config.heads, config.spt, f1.reset_failed);
    bool gv = (use_lba_mode && total_lba_sectors > 0) || (!use_lba_mode && cur_cyls > 0 && cur_heads > 0 && cur_spt > 0);
    CHECK(!gv, "something left to mount after the unmount");
    // Nothing more reaches the drive as 0x91 afterwards either.
    debug_reset();
    read_right(1234, &got);
    CHECK(idp_sent() == 0, "0x91 after the unmount: %d", idp_sent());

    // The same race, with the mode and geometry already cleared under the
    // recovery (as c17fc81's unmount did): ide.c alone must not send 0x91.
    native_drive("N-1: ide.c alone, mode and geometry cleared under a recovery");
    ctrl_g_native();
    sim.srst_wedges = true;
    sim.bad[4321] = {BAD_HANG, 0};
    bool right = false;
    host_read(4321, &right);                        // the recovery outlasts this command
    CHECK(ide_recovery_pending(), "no recovery pending to race");
    ide_manual_chs_native_forget();
    config.cyls = 0; config.heads = 0; config.spt = 0;
    for (int i = 0; i < 6 && ide_recovery_pending(); i++) host_read(1234, &right);
    CHECK(idp_sent() == 0, "0x91 sent with the geometry cleared: %d", idp_sent());
    native_drive("N-1: ide.c alone, mode cleared, geometry kept");
    ctrl_g_native();
    sim.srst_wedges = true;
    sim.bad[4321] = {BAD_HANG, 0};
    host_read(4321, &right);
    ide_manual_chs_native_forget();
    for (int i = 0; i < 6 && ide_recovery_pending(); i++) host_read(1234, &right);
    CHECK(idp_sent() == 0, "0x91 sent after the mode was cleared: %d", idp_sent());
#ifdef MCHS_MODEL_CLEARED
    // ...until a new Ctrl+G (old mode) or Auto Detect: then 91h as before.
    keys("522\r4\r40\ry"); run_manual_chs();
    CHECK(idp_sent() == 1 && !ide_native_drive(), "old-mode Ctrl+G after: 0x91 %d", idp_sent());
    // Never 0 heads or 0 sectors, in any mode.
    native_drive("N-1: no 0x91 with 0 heads or 0 sectors");
    CHECK(!ide_set_geometry(0, 40) && !ide_set_geometry(4, 0) && sim.cmd_log.empty(), "sent %zu", sim.cmd_log.size());
#endif
}

#ifdef MCHS_MODEL_CLEARED
// N-2: after a native unmount, Mount and the error box keep the Ctrl+G text
// on Current HDD until Auto Detect; ataboy_autodetect.ctrlg_refusal looks
// for text starting "Manual CHS (no IDENTIFY)", or "Manual CHS: ".
static void test_p10_n2_marker_kept() {
    native_drive("N-2: Mount after a native unmount");
    ctrl_g_native();
    is_mounted = true;
    menu_unmount();
    mount_nothing_to_mount();
    CHECK(strcmp(hdd_model_raw, MCHS_MODEL_CLEARED) == 0 && strstr(hdd_status_text, "Manual CHS: geometry cleared, Ctrl+G again"),
          "\"%s\" / \"%s\"", hdd_model_raw, hdd_status_text);
    tty.clear(); update_main_menu();
    CHECK(has("Manual CHS: geometry cleared"), "row 22 lost the Ctrl+G text");
    detect_result_dismiss();
    tty.clear(); update_main_menu();
    CHECK(has("Current HDD: ") && has(MCHS_MODEL_CLEARED) && has("0 Cyl / 0 Hd / 0 SPT"), "after dismissing");
    // A failed Ctrl+G (either mode) is marked the same once dismissed.
    native_drive("N-2: a failed Ctrl+G, dismissed");
    sim.reject_idp = true;
    keys("522\r4\r40\ry"); run_manual_chs();
    detect_result_dismiss();
    CHECK(strcmp(hdd_model_raw, MCHS_MODEL_CLEARED) == 0, "\"%s\"", hdd_model_raw);
    // With no Ctrl+G since the ATAboy started: as before.
    native_drive("N-2: not without a Ctrl+G");
    sim.identify_ok = false;
    keys("\r");
    run_auto_detect();
    detect_result_dismiss();
    CHECK(hdd_model_raw[0] == 0, "\"%s\" after Auto Detect", hdd_model_raw);
    mount_nothing_to_mount();
    CHECK(strstr(hdd_status_text, "Detect drive and set geometry first!") && hdd_model_raw[0] == 0, "\"%s\"", hdd_status_text);
}
#endif

#ifdef MCHS_MODEL_CLEARED
// ---- 13. each guard layer on its own (mutation run of 2fcc4aa) --------------------
// The layers hide each other when only the drive is watched, so each is
// checked by its own refusal count, or called directly.
static void test_p10_layers() {
    // ide_identify refuses by itself: the inner layer is never reached.
    native_drive("layers: ide_identify refuses before identify_run");
    ctrl_g_native();
    int w = sim.reg_writes;
    uint16_t id[256];
    CHECK(!ide_identify(id) && refused.identify_call == 1 && refused.identify_run == 0 && sim.reg_writes == w,
          "outer %u inner %u writes %d", refused.identify_call, refused.identify_run, sim.reg_writes - w);
    // identify_run, the last layer, refuses on its own (the SAT identity
    // check calls it directly).
    native_drive("layers: identify_run refuses on its own");
    mchs_no_identify = true;
    sim.identify_ok = true;
    CHECK(identify_run(id) == 0 && refused.identify_run == 1 && sim.id_commands() == 0 && sim.reg_writes == 0,
          "identify_run: inner %u, %d IDENTIFY", refused.identify_run, sim.id_commands());
    // The Debug keys refuse before either (screen text, test_p10_m1).
    native_drive("layers: Debug I refuses before ide_identify");
    ctrl_g_native();
    debug('I'); debug('S');
    CHECK(refused.identify_call == 0 && refused.identify_run == 0, "Debug reached ide_identify: %u %u",
          refused.identify_call, refused.identify_run);
    // chs_geometry_ok refuses no geometry by itself: ide_set_geometry's own
    // zero check is never reached.
    fresh("layers: chs_geometry_ok refuses no geometry by itself");
    config.cyls = 0; config.heads = 0; config.spt = 0; chs_geometry_lost = true;
    CHECK(!chs_geometry_ok(0) && refused.idp_zero == 0 && sim.cmd_log.empty(), "zero refusals %u, %zu commands",
          refused.idp_zero, sim.cmd_log.size());
    config.heads = 4;                               // 0 sectors only
    CHECK(!chs_geometry_ok(0) && refused.idp_zero == 0 && sim.cmd_log.empty(), "zero refusals %u (spt 0)", refused.idp_zero);
    // Mutation run of ab81985: since the holistic review it is the first
    // line of chs_geometry_ok (0 heads, over 16, or 0 sectors) that refuses
    // no geometry; the later zero check is never reached (mutate.py). That
    // line on its own: the geometry not marked lost, so nothing after it
    // would stop a CHS address being worked out with 0 heads or 0 sectors.
    fresh("layers: no geometry refused first, the geometry not marked lost");
    config.cyls = 0; config.heads = 0; config.spt = 40; chs_geometry_lost = false;
    CHECK(!chs_geometry_ok(0) && refused.idp_zero == 0 && sim.reg_writes == 0, "0 heads: zero refusals %u, %d writes",
          refused.idp_zero, sim.reg_writes);
    config.heads = 4; config.spt = 0;
    CHECK(!chs_geometry_ok(0) && refused.idp_zero == 0 && sim.reg_writes == 0, "0 sectors: zero refusals %u, %d writes",
          refused.idp_zero, sim.reg_writes);
    // ide_set_geometry's own zero check.
    fresh("layers: ide_set_geometry refuses 0 heads or 0 sectors");
    CHECK(!ide_set_geometry(0, 40) && !ide_set_geometry(4, 0) && refused.idp_zero == 2 && sim.cmd_log.empty(),
          "zero refusals %u", refused.idp_zero);
    // ...and its native-drive check, counted apart.
    native_drive("layers: ide_set_geometry refuses a native-mounted drive");
    ctrl_g_native();
    ide_manual_chs_native_forget();                 // the mode ended, the drive still marked
    CHECK(!ide_set_geometry(NH, NS) && refused.idp_native == 1 && refused.idp_zero == 0 && idp_sent() == 0,
          "native refusals %u", refused.idp_native);
}
#endif

#ifdef AUTODETECT_CONFIRM
// ---- 14. review C of 0.6f3p10 --------------------------------------------------
static void test_c3_c5_c8() {
    // C-3: Enter on Auto Detect asks first, and sends nothing to do so.
    fresh("C-3: Auto Detect asks first");
    sim.identify_ok = true;
    keys("\x1b");                                    // if it ran at once, Esc out of its picker
    CHECK(auto_detect_enter() && current_screen == SCREEN_CONFIRM && confirm_type == 6 && nothing_sent(),
          "screen %d confirm %d, writes %d", (int)current_screen, confirm_type, sim.reg_writes);
    current_screen = SCREEN_MAIN;
    // ...and after a Ctrl+G it is refused at once: no question, nothing sent.
    fresh("C-3: Auto Detect refused after a Ctrl+G");
    sim.identify_ok = true;
    ctrl_g_native();
    size_t m = sim.cmd_log.size();
    int w = sim.reg_writes;
    keys("x\x1b\x1b");   // x for the refusal; Esc, Esc if Auto Detect ran
    tty.clear();
    CHECK(!auto_detect_enter() && current_screen == SCREEN_MAIN && sim.cmd_log.size() == m && sim.reg_writes == w &&
          sim.id_commands() == 0 && has(AUTODETECT_REFUSED_MCHS), "screen %d, %zu commands", (int)current_screen, sim.cmd_log.size() - m);
    // C-5: the active-low outputs are high before they become outputs.
    fresh("C-5: no RESET- or CS pulse at boot");
    mock_gpio_out = 0;
    ide_hw_init();
    uint32_t need = (1u << IDE_RESET) | (1u << IDE_CS0) | (1u << IDE_CS1) | (1u << IDE_OE) | (1u << IDE_OE1);
    CHECK((mock_gpio_out_at_dir & need) == need, "latch at output enable %08X", (unsigned)mock_gpio_out_at_dir);
    // C-2: Debug E names the new failure kind.
    fresh("C-2: Debug E says no drive");
    last_fail.kind = IDE_FAIL_NO_DRIVE; last_fail.command = 0x20; last_fail.status = 0xFF; last_fail.lba = 5003;
    last_fail.done = 3; last_fail.count = 8;
    tty.clear(); run_debug_errors();
    CHECK(has("[Last Failed I/O] cmd 20 no drive at LBA 5003, 3 of 8 done"), "Debug E");
#ifdef IDE_FAIL_SELF_RESET
    // Review C round 2, C2-1: a self-reset in the power-up default mode: the
    // next command fails, no 91h, and Debug E shows it; the one after reads.
    native_drive("C2-1: native, the drive resets itself");
    ctrl_g_native();
    CHECK(read_right(1234), "before");
    sim.hw_reset_line(false, mock_now_ns); mock_now_ns += 1000000;
    sim.hw_reset_line(true, mock_now_ns); mock_now_ns += 2000000000ull;
    int32_t g = 0;
    read_right(1234, &g);
    ide_fail_t fr; ide_last_failure(&fr);
    ide_self_resets_t srs; ide_self_resets(&srs);
    CHECK(g < 0 && idp_sent() == 0 && fr.kind == IDE_FAIL_SELF_RESET && fr.lba == 1234 && srs.detected == 1 &&
          srs.native_failed == 1 && srs.idp_resent == 0, "read %d, 0x91 %d, kind %u, counts %u %u", (int)g, idp_sent(), fr.kind,
          srs.detected, srs.native_failed);
    tty.clear(); run_debug_errors();
    CHECK(has("[Last Failed I/O] drive reset itself before LBA 1234") && has("Self-resets 1, last LBA 1234") &&
          has("Self-resets: 91h again 0, failed 1"), "Debug E");
    CHECK(read_right(1234) && idp_sent() == 0, "the next read");

    // Round 3, C3-1: in the power-up default mode, a drive that does not keep
    // SC/SN must not fail every other command: the check is off, shown so.
    native_drive("C3-1: native, drive does not keep SC/SN");
    sim.clobber_after = 100000;
    ctrl_g_native();
    int good = 0;
    for (uint32_t l = 0; l < 20; l++) good += read_right(l * 8);
    ide_self_resets(&srs);
    CHECK(good == 20 && srs.detected == 0 && srs.native_failed == 0 && srs.unavailable && idp_sent() == 0,
          "%d of 20 reads, detected %u failed %u unavailable %d", good, srs.detected, srs.native_failed, srs.unavailable);
    tty.clear(); run_debug_errors();
    CHECK(has("Self-reset check OFF: drive does not keep SC/SN (x1)"), "Debug E does not say the check is off");
    // A new Ctrl+G tries again (a drive that keeps them).
    sim.clobber_after = 0;
    ctrl_g_native();
    ide_self_resets(&srs);
    CHECK(!srs.unavailable && srs.unheld == 1, "not tried again at the next mount");

    // Round 3, LOW: Debug E rows never run past the 68-column box.
    native_drive("C3: Debug E rows stay in the box");
    self_resets.detected = 4294967295u; self_resets.last_lba = 4294967295u;
    self_resets.idp_resent = 4294967295u; self_resets.native_failed = 4294967295u;
    self_resets.unavailable = true; self_resets.unheld = 4294967295u;
    last_fail.kind = IDE_FAIL_SELF_RESET; last_fail.lba = 4294967295u;
    tty.clear(); run_debug_errors();
    // Every number whole at its largest (the host reads them), and each row
    // whole: its text, padded to the box, then the reset (nothing was cut).
    auto whole = [](const std::string &row) { return has((row + std::string(68 - row.size(), ' ') + "\033[0m").c_str()); };
    CHECK(whole("[Last Failed I/O] drive reset itself before LBA 4294967295") &&
          whole("Self-resets 4294967295, last LBA 4294967295") &&
          whole("Self-resets: 91h again 4294967295, failed 4294967295") &&
          whole("Self-reset check OFF: drive does not keep SC/SN (x4294967295)"), "a Debug E self-reset row cut at the largest values");
    bool inbox = true;
    for (int row = 4; row <= 20; row++) {
        std::string at = "\033[" + std::to_string(row) + ";7H\033[40m";
        size_t p = tty.find(at);
        while (p != std::string::npos) {
            size_t e = tty.find("\033[0m", p + at.size());
            std::string seg = tty.substr(p + at.size(), e == std::string::npos ? std::string::npos : e - p - at.size());
            int vis = 0; bool esc = false;
            for (char c : seg) { if (c == '\033') esc = true; else if (esc) { if (c == 'm') esc = false; } else vis++; }
            if (vis > 68) inbox = false;
            p = tty.find(at, p + 1);
        }
    }
    CHECK(inbox, "a Debug E row is wider than the box");
    // A generic record can still run past 68 columns at its largest (a
    // 10-digit LBA and 3-digit counts): it is cut at the box, not spilled.
    native_drive("C3: a long generic Debug E record is cut at the box");
    last_fail.kind = IDE_FAIL_STALE_DRQ; last_fail.command = 0x20; last_fail.lba = 4294967295u;
    last_fail.done = 255; last_fail.count = 256;
    tty.clear(); run_debug_errors();
    std::string cut = "[Last Failed I/O] cmd 20 stale data at LBA 4294967295, 255 of 256 do";
    CHECK(cut.size() == 68 && has((cut + "\033[0m").c_str()), "the long record is not cut at 68 columns");
#endif
    // C-8: a drive that floats the bus (FFh) for 5 s after RESET- is waited for.
    native_drive("C-8: FFh for 5 s after RESET-");
    sim.t_float = 5000000000ull;
    keys("\x07" "522\r4\r40\ry");
    CHECK(!run_manual_chs() && cur_cyls == NC, "\"%s\"", hdd_status_text);
}
#endif

#ifdef IDE_SENTINEL_SC
// ---- 15. mutation run of fa3190a: coverage the review C changes removed ------------
static void test_fa3190a_gaps() {
    // Auto Detect starts by clearing whatever geometry was set, on screen
    // and in config (review P2 of 0.6f3p8), so leaving its picker with Esc
    // leaves nothing to mount. Since review C (C-3) Auto Detect is refused
    // after a Ctrl+G, so the geometry here is an earlier one (EEPROM).
    fresh("fa3190a: Auto Detect clears an earlier geometry");
    sim.identify_ok = true;
    earlier_setup();
    keys("\x1b");
    run_auto_detect();
    CHECK(sim.id_commands() == 1, "Auto Detect did not run");
    CHECK(cur_cyls == 0 && cur_heads == 0 && cur_spt == 0 && !use_lba_mode && total_lba_sectors == 0,
          "screen kept %u/%u/%u", cur_cyls, cur_heads, cur_spt);
    CHECK(config.cyls == 0 && config.heads == 0 && config.spt == 0 && !config.use_lba_mode && config.lba_sectors == 0,
          "config kept %u/%u/%u", config.cyls, config.heads, config.spt);
    // ide.c on its own: only an IDENTIFY that answers ends the Ctrl+G flag;
    // one that fails leaves it (the probe is how the no-IDENTIFY rule ends,
    // and it is only reached through Auto Detect, which the menu refuses
    // after a Ctrl+G: this is the rule underneath).
    fresh("fa3190a: an IDENTIFY that fails keeps the Ctrl+G flag (ide.c)");
    ctrl_g();
    ide_probe_devices();
    sim.identify_ok = false;
    uint16_t id[256];
    CHECK(!ide_identify(id) && ide_manual_chs_active(), "flag cleared by an IDENTIFY that failed");
    sim.identify_ok = true;
    CHECK(ide_identify(id) && !ide_manual_chs_active(), "flag kept after an IDENTIFY that answered");
}
#endif

#ifdef IDE_FAULT_MAGIC
// ---- holistic review (2026-09-26): menus, settings, boot, faults -----------------
static bool busy_now() { return true; }
static void test_holistic_menus() {
    // M-1: the picker's MANUAL row refuses 17 heads (it used to send 91h with 1).
    fresh("M-1: picker MANUAL, 17 heads");
    sim.identify_ok = true;
    keys("\t" "1045\r17\r40\r" "\x1b");
    run_auto_detect();
    CHECK(sim.cmd_count.count(0x91) == 0 && config.heads != 17, "0x91 %d, config heads %u",
          (int)(sim.cmd_count.count(0x91) ? sim.cmd_count[0x91] : 0), config.heads);
    fresh("M-1: picker MANUAL, 16 heads still taken");
    sim.identify_ok = true;
    keys("\t" "1045\r16\r40\r");
    run_auto_detect();
    CHECK(sim.cmd_count.count(0x91) == 1 && config.heads == 16, "16 heads not applied: %u", config.heads);

    // L-1: F10 and Load Defaults wait for a USB command, or do nothing.
    fresh("L-1: F10 with a USB command running");
    busy_until_ns = mock_now_ns + 60000000000ull;   // longer than the wait
    keys("x");
    tty.clear();
    CHECK(!save_setup() && saves == 0 && has("nothing sent"), "saved %d", saves);
    busy_until_ns = 0;

    // Stepping on the main screen.
    fresh("chip stepping shown");
    tty.clear(); update_main_menu();
    CHECK(has("Chip: RP2350 A4"), "no stepping on the main screen");

    // L-2: the settings sector.
    // (config.c is not built here: these are its defaults, written out.)
    fresh("L-2: the settings sector's checks");
    config_t c; memset(&c, 0, sizeof c);
    c.drive_write_protected = true; c.auto_mount = false; c.dev_base = 0xA0;
    c.magic = CONFIG_MAGIC; c.crc = config_crc(&c);
    CHECK(config_valid(&c) && c.drive_write_protected && !c.auto_mount, "defaults not valid, or not safe");
    config_t d = c; d.crc ^= 1;                      CHECK(!config_valid(&d), "a bad CRC taken");
    d = c; d.magic = 0x1DE45707; d.crc = config_crc(&d); CHECK(!config_valid(&d), "an old layout taken");
    d = c; d.heads = 17; d.cyls = 10; d.spt = 40; d.crc = config_crc(&d); CHECK(!config_valid(&d), "17 heads taken");
    d = c; d.dev_base = 0xE0; d.crc = config_crc(&d); CHECK(!config_valid(&d), "dev_base E0h taken");
    d = c; { uint8_t two = 2; memcpy(&d.auto_mount, &two, 1); } d.crc = config_crc(&d); CHECK(!config_valid(&d), "a bool of 2 taken");
    d = c; d.cyls = 522; d.heads = 4; d.spt = 0; d.crc = config_crc(&d); CHECK(!config_valid(&d), "0 sectors taken");
    d = c; d.cyls = 522; d.heads = 4; d.spt = 40; d.crc = config_crc(&d); CHECK(config_valid(&d), "522/4/40 refused");
    // F1 stage 2: Pipelined Reads in the CRC, a real bool, the cursor on row 5.
    d = c; d.pipelined_reads = true; CHECK(!config_valid(&d), "Pipelined Reads changed under the old CRC taken");
    d.crc = config_crc(&d); CHECK(config_valid(&d), "Pipelined Reads On refused");
    d = c; { uint8_t two = 2; memcpy(&d.pipelined_reads, &two, 1); } d.crc = config_crc(&d); CHECK(!config_valid(&d), "a Pipelined Reads of 2 taken");
    d = c; d.feat_selected = 5; d.crc = config_crc(&d); CHECK(config_valid(&d), "the cursor on Pipelined Reads refused");
    d = c; d.feat_selected = 6; d.crc = config_crc(&d); CHECK(!config_valid(&d), "a cursor past the last item taken");
    d = c; d.magic = 0x1DE45708; d.crc = config_crc(&d); CHECK(!config_valid(&d), "the layout before Pipelined Reads taken");
    d = c; d.auto_mount = true; d.crc = config_crc(&d); d.lba_sectors = 1; CHECK(!config_valid(&d), "a changed field under an old CRC taken");

    // B1: no active-low line dips at boot (warm or cold).
    fresh("B1: ide_hw_init lets no active-low line fall");
    mock_active_low_dips = 0;
    mock_gpio_out = 0; mock_gpio_oe = 0;            // SIO after a warm reset: latches and enables 0
    ide_hw_init();
    CHECK(mock_active_low_dips == 0, "%d dips", mock_active_low_dips);

    // The fault path: record, and the bus released.
    fresh("fault: record and bus release");
    mock_pio_stopped = false;
    mock_gpio_out = 0; mock_gpio_oe = 0xFFFFFFFFu;
    ide_fault_record(1, 0x10001234u, 0x10005678u, 0x00020000u);
    ide_fault_release_bus();
    uint32_t high = (1u << IDE_OE) | (1u << IDE_OE1) | (1u << IDE_CS0) | (1u << IDE_CS1) | (1u << IDE_RESET) |
                    (1u << IDE_DIOR) | (1u << IDE_DIOW);
    CHECK(mock_pio_stopped && (mock_gpio_out & high) == high && (mock_gpio_oe & high) == high &&
          !(mock_gpio_out & ((1u << IDE_DIR) | (1u << IDE_DIR1))) && !(mock_gpio_oe & 0xFFFFu) &&
          (mock_sio_pins & ((1u << IDE_DIOR) | (1u << IDE_DIOW) | 0xFFFFu)) == ((1u << IDE_DIOR) | (1u << IDE_DIOW) | 0xFFFFu),
          "out %08X oe %08X pio stopped %d", (unsigned)mock_gpio_out, (unsigned)mock_gpio_oe, mock_pio_stopped);
    ide_fault_t fr;
    CHECK(ide_fault_last(&fr) && fr.core == 1 && fr.pc == 0x10001234u && fr.cfsr == 0x00020000u, "record");
    unwitnessed_blocks = 42;
    tty.clear(); run_debug_errors();
    CHECK(has("Fault before this boot: core 1 PC 10001234 CFSR 00020000"), "Debug E does not show the fault");
    CHECK(has("Blocks taken with no drop seen before: 42"), "Debug E does not show the unwitnessed blocks");
    unwitnessed_blocks = 0;
    fault_rec.magic = 0;
    (void)busy_now;
}
#endif

// ---- 7. through the main loop: Ctrl+G reaches the entry ---------------------
// Last, since core1_entry never returns (on_idle ends the program).
static void core1_done() {
    CHECK(sent_exactly_recal_idp() && no_identify(), "commands %zu, resets %d", sim.cmd_log.size(), sim.hw_resets);
    CHECK(cur_cyls == C && cur_heads == H && cur_spt == S && config.cyls == C && !show_detect_result,
          "geometry %u/%u/%u", cur_cyls, cur_heads, cur_spt);
    CHECK(has("Ctrl+G: Manual CHS") && has("Manual CHS (no IDENTIFY)") && has("1045 Cyl / 2 Hd / 40 SPT"), "screens");
    CHECK(saves == 0, "EEPROM written %d times", saves);
    CHECK(has(SAVE_REFUSED_MCHS), "F10 with Auto Mount on: no refusal shown (L-1)");
    // F1 stage 1: Mount (Enter, Y) and Unmount (U, Y) wrote their markers.
    int mounts = 0, unmounts = 0;
    obs_log_entry_t mk{};
    for (uint32_t s = 1; s < log_next; s++) {
        obs_log_entry_t e = log_ring[s % OBS_LOG_ENTRIES];
        if (e.kind == OBS_EV_MOUNT) { mounts++; mk = e; }
        if (e.kind == OBS_EV_UNMOUNT && mounts == 1) unmounts++;
    }
    CHECK(mounts == 1 && unmounts == 1 && mk.value == OBS_MNT_MENU && mk.tf[0] == OBS_MODE_MCHS_91 &&
          mk.tf[1] == S && mk.tf[2] == H && (mk.tf[3] | (mk.tf[4] << 8)) == C && !is_mounted,
          "mounts %d unmounts %d, marker mode %u, mounted %d", mounts, unmounts, mk.tf[0], is_mounted);
    CHECK(config.pipelined_reads && current_screen == SCREEN_FEATURES && config.feat_selected == 5 &&
          has("Pipelined Reads"),
          "Pipelined Reads not switched on from Features: %d, screen %d, item %d", config.pipelined_reads,
          (int)current_screen, config.feat_selected);
    CHECK(has("Reads the next part"), "Pipelined Reads help not shown");
    printf("%d checks, %d failed\n", checks, failures);
    exit(failures > 255 ? 255 : failures);
}
static void test_core1() {
    fresh("main loop");
    config.auto_mount = true;                       // with no geometry saved, nothing mounts at start-up
    keys("\x07" "1045\r2\r40\ry", 500);             // once the menu is up
    // Then F10 (its escape sequence as a terminal sends it, 1 ms apart), Y,
    // and a key for the refusal (L-1).
    uint64_t t = rx.back().at_us + 20000;
    for (char c : std::string("\x1b[21~")) { rx.push_back({ t, c }); t += 1000; }
    rx.push_back({ t + 20000, 'y' });
    rx.push_back({ t + 40000, 'x' });
    // F1 stage 1: then Mount (down to it, Enter, Y) and Unmount (U, Y).
    t += 60000;
    for (char c : std::string("\x1b[B")) { rx.push_back({ t, c }); t += 1000; }
    rx.push_back({ t + 20000, '\r' });
    rx.push_back({ t + 40000, 'y' });
    rx.push_back({ t + 60000, 'u' });
    rx.push_back({ t + 80000, 'y' });
    // F1 stage 2: then Features (up, right, Enter), down to Pipelined Reads, Enter.
    t += 100000;
    for (const char *seq : { "\x1b[A", "\x1b[C" }) {
        for (const char *c = seq; *c; c++) { rx.push_back({ t, *c }); t += 1000; }
        t += 20000;
    }
    rx.push_back({ t, '\r' }); t += 20000;
    for (int i = 0; i < 5; i++) {
        for (const char *c = "\x1b[B"; *c; c++) { rx.push_back({ t, *c }); t += 1000; }
        t += 20000;
    }
    rx.push_back({ t, '\r' });
    obs_test_reset(0, 0);
    idle_after_ns = mock_now_ns + 20000000000ull;
    on_idle = core1_done;
    core1_entry();
}

// ---- F1 stage 1: proof from the log that no ECh and no 91h went out -----------
// What a host tool does with the log (READ BUFFER 4Ch; obs.c): take every
// entry from the last MOUNT marker to now. The proof holds only when that
// run is complete (no seq missing, none lost), and then it counts the ECh
// and 91h commands in it. Read here with obs_log_page, as usb.c answers.
struct Proof { bool mount = false, complete = false; int ec = 0, idp = 0, cmds = 0; };
static Proof proof_since_mount() {
    Proof p;
    std::vector<obs_log_entry_t> all;
    uint32_t want = 0;
    bool lost = false;
    static uint8_t pg[4096];
    for (int guard = 0; guard < 10000; guard++) {
        uint32_t r = obs_log_page(want & 0xFFFFFF, pg, sizeof pg);
        if (r < 32 || memcmp(pg, "ATBLOG01", 8) != 0) return p;
        if (pg[28] & 1) lost = true;
        uint32_t n = pg[12] | (pg[13] << 8);
        if (n == 0) break;
        uint32_t first = pg[8] | (pg[9] << 8) | (pg[10] << 16) | ((uint32_t)pg[11] << 24);
        if (want && first != want) lost = true;
        for (uint32_t i = 0; i < n; i++) { obs_log_entry_t e; memcpy(&e, pg + 32 + 32 * i, 32); all.push_back(e); }
        want = all.back().seq + 1;
    }
    size_t m = all.size();
    for (size_t i = 0; i < all.size(); i++) if (all[i].kind == OBS_EV_MOUNT) m = i;
    if (m == all.size()) return p;
    p.mount = true;
    // The mount marker must still be the oldest part of a gapless run: if
    // the ring had wrapped past it, the first page would have said "lost".
    p.complete = !lost;
    for (size_t i = m + 1; i < all.size(); i++) {
        if (all[i].seq != all[i - 1].seq + 1) p.complete = false;
        if (all[i].kind != OBS_EV_CMD) continue;
        p.cmds++;
        if (all[i].value == 0xEC) p.ec++;
        if (all[i].value == 0x91) p.idp++;
    }
    return p;
}
// The drive's own count of a command since a point in cmd_log.
static int sim_count_since(size_t from, uint8_t c) {
    int n = 0;
    for (size_t i = from; i < sim.cmd_log.size(); i++) if (sim.cmd_log[i] == c) n++;
    return n;
}

static void test_obs_proof() {
    // Native mode through everything that resets the drive: the log proves
    // no ECh and no 91h, and the drive agrees.
    native_drive("F1 proof: native mode, resets and recoveries");
    obs_test_reset(0, 0);
    ctrl_g_native();
    log_mount(OBS_EV_MOUNT, OBS_MNT_MENU);           // as Mount's Y does (menus.c)
    is_mounted = true;
    size_t m = sim.cmd_log.size();
    read_right(1234);
    debug_reset();
    read_right(1234);
    sim.bad[4321] = {BAD_HANG, 0};
    read_right(4321);
    sim.bad.clear();
    read_right(1234);
    sim.srst_wedges = true;
    sim.bad[4321] = {BAD_HANG, 0};
    read_right(4321);
    sim.srst_wedges = false; sim.bad.clear();
    bool right = false;
    for (int i = 0; i < 4; i++) host_read(1234 + i, &right);
    Proof p = proof_since_mount();
    CHECK(p.mount && p.complete && p.ec == 0 && p.idp == 0 && p.cmds >= 8, "mount %d complete %d ECh %d 91h %d cmds %d",
          p.mount, p.complete, p.ec, p.idp, p.cmds);
    CHECK(sim_count_since(m, 0xEC) == 0 && sim_count_since(m, 0x91) == 0 && p.cmds == (int)(sim.cmd_log.size() - m),
          "the drive: ECh %d 91h %d, commands %zu against the log's %d", sim_count_since(m, 0xEC),
          sim_count_since(m, 0x91), sim.cmd_log.size() - m, p.cmds);
    obs_log_entry_t mk{};
    for (uint32_t s = log_next - 1; s > 0; s--) if (log_ring[s % OBS_LOG_ENTRIES].kind == OBS_EV_MOUNT) { mk = log_ring[s % OBS_LOG_ENTRIES]; break; }
    // Every RESET- is in the log, with where it came from: Ctrl+G's, Debug R's, the recovery's.
    int rs[5] = {0, 0, 0, 0, 0};
    for (uint32_t s = 1; s < log_next; s++)
        if (log_ring[s % OBS_LOG_ENTRIES].kind == OBS_EV_RESET && log_ring[s % OBS_LOG_ENTRIES].value < 5) rs[log_ring[s % OBS_LOG_ENTRIES].value]++;
    CHECK(rs[OBS_RST_MANUAL_CHS] == 1 && rs[OBS_RST_RESET_DRIVE] == 1 && rs[OBS_RST_RECOVERY] == 1 &&
          rs[1] + rs[2] + rs[3] + rs[4] == sim.hw_resets, "RESET- in the log: Ctrl+G %d, Debug R %d, recovery %d; the drive saw %d",
          rs[OBS_RST_MANUAL_CHS], rs[OBS_RST_RESET_DRIVE], rs[OBS_RST_RECOVERY], sim.hw_resets);
    CHECK(mk.tf[0] == OBS_MODE_MCHS_NATIVE && mk.tf[1] == NS && mk.tf[2] == NH && (mk.tf[3] | (mk.tf[4] << 8)) == NC &&
          mk.tf[5] == 0xA0, "the marker: mode %u, %u/%u/%u", mk.tf[0], mk.tf[3] | (mk.tf[4] << 8), mk.tf[2], mk.tf[1]);

    // The other way: a 91h that does go out is seen. The old mode sends one
    // after a soft reset; and one planted straight on the bus in native mode.
    native_drive("F1 proof: the 91h mode shows its 91h");
    obs_test_reset(0, 0);
    keys("522\r4\r40\ry"); run_manual_chs();
    log_mount(OBS_EV_MOUNT, OBS_MNT_MENU);
    is_mounted = true;
    m = sim.cmd_log.size();
    sim.bad[4321] = {BAD_HANG, 0};
    read_right(4321);
    sim.bad.clear();
    p = proof_since_mount();
    CHECK(p.complete && p.idp >= 1 && p.idp == sim_count_since(m, 0x91), "91h mode: log %d, drive %d", p.idp,
          sim_count_since(m, 0x91));
    native_drive("F1 proof: a 91h planted in native mode");
    obs_test_reset(0, 0);
    ctrl_g_native();
    log_mount(OBS_EV_MOUNT, OBS_MNT_MENU);
    is_mounted = true;
    read_right(1234);
    ide_write_reg(6, 0xA0 | (NH - 1)); ide_write_reg(2, NS); ide_write_reg(7, 0x91);   // no firmware path does this
    mock_now_ns += 10000000;
    p = proof_since_mount();
    CHECK(p.complete && p.idp == 1, "planted 91h: log %d", p.idp);

    // No proof from a log that has lost the mount: the run is not complete.
    native_drive("F1 proof: the mount marker overwritten");
    obs_test_reset(0, 0);
    ctrl_g_native();
    log_mount(OBS_EV_MOUNT, OBS_MNT_MENU);
    is_mounted = true;
    for (int i = 0; i < OBS_LOG_ENTRIES + 10; i++) read_right(1234 + i % 40);
    p = proof_since_mount();
    CHECK(!p.mount || !p.complete, "a proof from a log that wrapped: mount %d complete %d", p.mount, p.complete);
    native_drive("F1 proof: no mount marker");
    obs_test_reset(0, 0);
    ctrl_g_native();
    read_right(1234);
    p = proof_since_mount();
    CHECK(!p.mount && !p.complete, "a proof with no mount");
}

// ---- 9. 0.6f3p9: the IORDY advisory --------------------------------------------
// Auto Detect against the simulated drive, Esc out of the picker, then the
// main and Features screens. The note shows only with IORDY on in Features
// and IDENTIFY word 49 bit 11 clear; the commands sent are the same either way.
static std::vector<uint8_t> iordy_detect(const char *name, bool iordy_on, uint16_t w49, bool *picker, bool *main_s, bool *feat) {
    fresh(name);
    sim.identify_ok = true;
    sim.id_w49 = w49;
    config.iordy_enabled = iordy_on;
    keys("\x1b");
    tty.clear();
    run_auto_detect();
    *picker = has(IORDY_NOTE);
    tty.clear(); update_main_menu(); *main_s = has(IORDY_NOTE);
    tty.clear(); update_features_menu(); *feat = has(IORDY_NOTE);
    return sim.cmd_log;
}

static void test_iordy_note() {
    struct { const char *name; bool on; uint16_t w49; bool want; } rows[] = {
        { "IORDY on, word 49 = 0000h (pre-ATA-2)",    true,  0x0000, true },
        { "IORDY on, word 49 = 0200h (LBA only)",     true,  0x0200, true },
        { "IORDY on, word 49 bit 10 only",            true,  0x0600, true },
        { "IORDY on, word 49 bit 11 (IORDY)",         true,  0x0800, false },
        { "IORDY on, word 49 = 0F00h",                true,  0x0F00, false },
        { "IORDY off, word 49 = 0000h",               false, 0x0000, false },
        { "IORDY off, word 49 = 0A00h",               false, 0x0A00, false },
    };
    std::vector<uint8_t> first;
    for (auto &r : rows) {
        bool pk = false, mn = false, ft = false;
        std::vector<uint8_t> log = iordy_detect(r.name, r.on, r.w49, &pk, &mn, &ft);
        CHECK(pk == r.want && mn == r.want && ft == r.want, "picker %d main %d features %d, want %d", pk, mn, ft, r.want);
        if (first.empty()) first = log;
        CHECK(log == first && log == (std::vector<uint8_t>{ 0x10, 0xEC }), "commands differ: %zu", log.size());
        CHECK(config.iordy_enabled == r.on, "the setting changed");
    }
    bool pk, mn, ft;
    // Switched off in Features afterwards: gone at the next redraw.
    iordy_detect("IORDY switched off after detection", true, 0x0000, &pk, &mn, &ft);
    CHECK(mn, "not shown to begin with");
    config.iordy_enabled = false;
    tty.clear(); update_main_menu();
    CHECK(!has(IORDY_NOTE), "still shown with IORDY off");
    config.iordy_enabled = true;
    tty.clear(); update_main_menu();
    CHECK(has(IORDY_NOTE), "not shown again with IORDY on");
    // Ctrl+G (no IDENTIFY): nothing is known about the drive's IORDY.
    ctrl_g();
    tty.clear(); update_main_menu();
    CHECK(!has(IORDY_NOTE) && cur_cyls == C, "shown after Ctrl+G");
    // A detection whose IDENTIFY fails forgets the last one's word 49.
    iordy_detect("IDENTIFY fails at the next detection", true, 0x0000, &pk, &mn, &ft);
    sim.identify_ok = false;
    keys("\r");
    run_auto_detect();
    tty.clear(); update_main_menu();
    CHECK(!has(IORDY_NOTE), "word 49 of the earlier detection kept");
    // The F: Force picker has no IDENTIFY, only zeros: no line (review of
    // 0.6f3p9, LOW-4). The same word 49 from a real IDENTIFY does show it.
    // (Heads and sectors are not zero here: the picker divides by the heads
    // in get_large_geometry, which traps on a PC. On the RP2350 a division by
    // zero gives 0, so the F: Force picker's all-zero words do not trap.)
    uint16_t zeros[256];
    memset(zeros, 0, sizeof zeros);
    zeros[1] = 1045; zeros[3] = 2; zeros[6] = 40;
    config.iordy_enabled = true;
    tty.clear(); draw_selection_menu_ex(zeros, 3, true);
    CHECK(!has(IORDY_NOTE), "shown on the F: Force picker");
    tty.clear(); draw_selection_menu_ex(zeros, 3, false);
    CHECK(has(IORDY_NOTE), "not shown for a real IDENTIFY with word 49 = 0");
    // Auto Mount at power-up sends its own IDENTIFY: its word 49 counts.
    fresh("auto-mount");
    sim.identify_ok = true; sim.id_w49 = 0x0000;
    config.iordy_enabled = true; config.auto_mount = true;
    config.use_lba_mode = true; config.lba_sectors = sim.nsect;
    try_auto_mount();
    tty.clear(); update_main_menu();
    CHECK(is_mounted && has(IORDY_NOTE), "auto-mount: mounted %d", is_mounted);
    is_mounted = false;
}

int main() {
    ide_hw_init();                                  // the bus idle, as at power-up
    test_power_up();                                // first: the state as the program starts
    test_decisions();
    test_accepted();
    test_limits_accepted();
    test_pre_ata();
    test_refused();
    test_nothing_sent();
    test_guards();
    test_after_auto_detect();
    test_m1_debug_r();
    test_m1_auto_detect_esc();
    test_mchs_after_lba_detect();
    test_mchs_enter_not_yes();
    test_mchs_iordy_held();
    test_mchs_flag();
    test_save_setup();
    test_iordy_note();
    test_native_accepted();
    test_native_selection();
    test_native_resets();
    test_native_forgotten();
    test_native_not_ready();
    test_native_trusts_the_declaration();
    test_p10_m1_no_identify();
    test_p10_m2_consistent();
    test_p10_l1_unmount();
    test_p10_h1_idp_since_boot();
    test_p10_n1_unmount_during_recovery();
#ifdef MCHS_MODEL_CLEARED
    test_p10_n2_marker_kept();
    test_p10_layers();
#endif
#ifdef AUTODETECT_CONFIRM
    test_c3_c5_c8();
#endif
#ifdef IDE_SENTINEL_SC
    test_fa3190a_gaps();
#endif
#ifdef IDE_FAULT_MAGIC
    test_holistic_menus();
#endif
    test_obs_proof();                               // F1 stage 1
    test_core1();                                   // does not return
    printf("%d checks, %d failed\n", checks, failures);
    return failures > 255 ? 255 : failures;
}
