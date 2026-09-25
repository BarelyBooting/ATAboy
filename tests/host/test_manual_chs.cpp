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
#include "ide.c"

#include <stdio.h>
#include <stdlib.h>
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
    CHECK(mock_now_ns - t0 < 4000000000ull, "took %llu ms", (unsigned long long)((mock_now_ns - t0) / 1000000));
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
    fresh("M-1 P2: Ctrl+G, Auto Detect, Esc");
    sim.identify_ok = true;
    ctrl_g();
    CHECK(cur_cyls == C && ide_manual_chs_active(), "Ctrl+G did not set it");
    keys("\x1b");
    run_auto_detect();
    CHECK(sim.id_commands() == 1 && !ide_manual_chs_active(), "Auto Detect: %d IDENTIFY, flag %d",
          sim.id_commands(), ide_manual_chs_active());
    CHECK(cur_cyls == 0 && cur_heads == 0 && cur_spt == 0 && !use_lba_mode && total_lba_sectors == 0,
          "the Ctrl+G geometry kept: %u/%u/%u", cur_cyls, cur_heads, cur_spt);
    CHECK(config.cyls == 0 && config.heads == 0 && config.spt == 0 && !config.use_lba_mode && config.lba_sectors == 0,
          "config kept it: %u/%u/%u", config.cyls, config.heads, config.spt);
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
    fresh("M-1 P2: Ctrl+G, Auto Detect, IDENTIFY refused");
    ctrl_g();
    CHECK(run_auto_detect() && force_detect, "no error box offering F: Force");
    CHECK(cur_cyls == 0 && cur_heads == 0 && config.cyls == 0 && config.heads == 0, "the Ctrl+G geometry kept");
    CHECK(ide_manual_chs_active(), "flag cleared by an IDENTIFY that failed");
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
    CHECK(mock_iordy_inover == GPIO_OVERRIDE_NORMAL, "IORDY not believed again after");
    config.iordy_enabled = false;
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

    fresh("Ctrl+G flag: cleared by an IDENTIFY that answers");
    sim.identify_ok = true;
    ctrl_g();
    current_screen = SCREEN_DEBUG; debug_key('i'); current_screen = SCREEN_MAIN;
    CHECK(!ide_manual_chs_active() && sim.id_commands() == 1, "kept after Debug I answered");

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
    ctrl_g();
    keys("\r");
    run_auto_detect();
    config.auto_mount = true;
    CHECK(save_setup() && saves == 1 && config.use_lba_mode, "saves %d", saves);
}

// ---- 7. through the main loop: Ctrl+G reaches the entry ---------------------
// Last, since core1_entry never returns (on_idle ends the program).
static void core1_done() {
    CHECK(sent_exactly_recal_idp() && no_identify(), "commands %zu, resets %d", sim.cmd_log.size(), sim.hw_resets);
    CHECK(cur_cyls == C && cur_heads == H && cur_spt == S && config.cyls == C && !show_detect_result,
          "geometry %u/%u/%u", cur_cyls, cur_heads, cur_spt);
    CHECK(has("Ctrl+G: Manual CHS") && has("Manual CHS (no IDENTIFY)") && has("1045 Cyl / 2 Hd / 40 SPT"), "screens");
    CHECK(saves == 0, "EEPROM written %d times", saves);
    CHECK(has(SAVE_REFUSED_MCHS), "F10 with Auto Mount on: no refusal shown (L-1)");
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
    idle_after_ns = mock_now_ns + 20000000000ull;
    on_idle = core1_done;
    core1_entry();
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
    test_core1();                                   // does not return
    printf("%d checks, %d failed\n", checks, failures);
    return failures > 255 ? 255 : failures;
}
