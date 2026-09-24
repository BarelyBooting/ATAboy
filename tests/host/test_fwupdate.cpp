// Host tests for firmware update mode (fwupdate.h and its use in menus.c):
// which key opens the prompt and where, what Y does while a drive is mounted
// or a USB command is still running, and that no split or damaged escape
// sequence from a terminal can produce the key (review findings M3, L1, L3).
//
// menus.c is compiled for the PC as in test_tui.cpp. Console input comes from
// a timed queue: each byte arrives at a set time on the mock clock, so a
// sequence can be split across get_input()'s 10 ms escape timeout.
//
// Build and run: see run.sh. Exit status is the number of failed checks.
#include "mock_pico.h"
#include <stdio.h>
#include <string>
#include <vector>

// ---- stand-ins menus.c needs ----------------------------------------------
typedef struct { int unused; } queue_t;
#define PICO_ERROR_TIMEOUT (-1)
struct Arrival { uint64_t at_us; char c; };
static std::vector<Arrival> rx;
static size_t rxi;
static std::string tty;
static inline void queue_add_blocking(queue_t *, const void *c) { tty.push_back(*(const char *)c); }
static inline bool queue_try_remove(queue_t *, void *out) {
    if (rxi < rx.size() && get_absolute_time() >= rx[rxi].at_us) { *(char *)out = rx[rxi++].c; return true; }
    return false;
}
static inline absolute_time_t make_timeout_time_us(uint64_t us) { return get_absolute_time() + us; }
static inline bool time_reached(absolute_time_t t) { return get_absolute_time() >= t; }

#include "menus.c"

uint64_t mock_now_ns = 0;
int mock_usb_boot_requests = 0;
int mock_id_words_forgotten = 0;
void ide_id_words_forget(void) { mock_id_words_forgotten++; }
uint32_t mock_gpio_out = 0;
MockSio mock_sio;
bool mock_intrq(void) { return false; }
void mock_reset_line(bool) {}
bool tud_msc_set_sense(uint8_t, uint8_t, uint8_t, uint8_t) { return true; }
queue_t cdc_tx_queue, cdc_rx_queue;
volatile bool cdc_connected = true, is_mounted = false, media_changed_waiting = false;
config_t config;
void config_defaults(void) {}
void config_save(void) {}
// usb.c's busy flag, as core 1 sees it: busy until a set time (mock clock).
static uint64_t busy_until_ns = 0;
static int busy_reads = 0;
bool usb_msc_ide_busy(void) { busy_reads++; return mock_now_ns < busy_until_ns; }

// Every stand-in that would put a cycle on the IDE bus counts itself, and
// counts it again if the USB side still had the bus at that moment (review
// L-5: core 1 must not drive the bus while a USB command runs on core 0).
static int bus_uses = 0, bus_uses_while_busy = 0;
static int mock_iordy_pin_writes = 0, mock_iordy_follows = 0;
static void bus_use() { bus_uses++; if (mock_now_ns < busy_until_ns) bus_uses_while_busy++; }
void ide_select_device(uint8_t) {}
uint8_t ide_probe_devices(void) { bus_use(); return 0; }
void ide_reset_drive(void) { bus_use(); }
bool ide_wait_until_ready(uint32_t) { bus_use(); return true; }
uint8_t ide_read_reg(uint8_t) { bus_use(); return 0x50; }
void ide_set_iordy(bool) { mock_iordy_pin_writes++; }
// ide.c decides when the Features setting reaches the pin (review of 0.6f3p7, LOW).
void ide_iordy_follow_config(void) { mock_iordy_follows++; }
bool ide_identify(uint16_t *) { bus_use(); return false; }
bool ide_set_geometry(uint8_t, uint8_t) { bus_use(); return true; }
void ide_read_taskfile(uint8_t tf[8]) { bus_use(); for (int i = 0; i < 8; i++) tf[i] = 0; tf[7] = 0x50; }
uint8_t ide_seek_read_one(uint32_t, bool) { bus_use(); return 0x50; }
static ide_fail_t stub_fail;
void ide_last_failure(ide_fail_t *out) { *out = stub_fail; }
// Manual CHS (Ctrl+G) runs against ide.c itself in test_manual_chs.cpp.
bool ide_recovery_pending(void) { return false; }
int ide_manual_chs(uint8_t, uint8_t, uint8_t *st) { bus_use(); *st = 0x50; return IDE_MCHS_OK; }

static int failures = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; \
    printf("FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)

static void feed(std::vector<Arrival> seq) { rx = seq; rxi = 0; }
static uint64_t now_us() { return mock_now_ns / 1000; }

// ---- 1. the pure decisions ---------------------------------------------------
static void test_decisions() {
    const int extra[] = { -1, KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_F10, 0x106, -0xFA };
    std::vector<int> keys;
    for (int k = 0; k < 256; k++) keys.push_back(k);
    for (int k : extra) keys.push_back(k);
    int opened = 0;
    for (int main = 0; main < 2; main++)
        for (int m = 0; m < 2; m++)
            for (int k : keys) {
                bool want = main && !m && k == 0x06;
                bool got = fwupdate_key_opens_prompt(main != 0, m != 0, k);
                CHECK(got == want, "main %d mounted %d key %d: %d", main, m, k, got);
                opened += got;
            }
    CHECK(opened == 1, "opened %d times", opened);
    CHECK(FWUPDATE_KEY == 0x06, "the key is Ctrl+F");

    const uint32_t waits[] = { 0, 1, 5000, 59999, 60000, 60001, 0xFFFFFFFFu };
    for (int m = 0; m < 2; m++)
        for (int b = 0; b < 2; b++)
            for (uint32_t w : waits) {
                fwupdate_step_t want = m ? FWUPDATE_REFUSE_MOUNTED : !b ? FWUPDATE_GO
                                     : w < 60000u ? FWUPDATE_WAIT : FWUPDATE_REFUSE_BUSY;
                fwupdate_step_t got = fwupdate_step(m != 0, b != 0, w);
                CHECK(got == want, "mounted %d busy %d waited %u: %d want %d", m, b, w, (int)got, (int)want);
            }
}

// ---- 2. menus.c: the key, on every screen -----------------------------------
static void test_key_in_menus() {
    const screen_t screens[] = { SCREEN_MAIN, SCREEN_FEATURES, SCREEN_CONFIRM, SCREEN_MOUNTED, SCREEN_DEBUG };
    const int keys[] = { 0x06, 'B', 'b', 'F', 'f', 'U', KEY_DOWN, KEY_ENTER, KEY_ESC, 12 };
    for (screen_t sc : screens)
        for (int m = 0; m < 2; m++)
            for (int k : keys) {
                current_screen = sc; confirm_type = 1; is_mounted = m != 0;
                bool want = sc == SCREEN_MAIN && !m && k == 0x06;
                bool got = fwupdate_key(k);
                CHECK(got == want, "screen %d mounted %d key %d: %d", (int)sc, m, k, got);
                if (want) CHECK(current_screen == SCREEN_CONFIRM && confirm_type == 5, "prompt state %d/%d",
                                (int)current_screen, confirm_type);
                else CHECK(current_screen == sc && confirm_type == 1, "state changed without the prompt");
            }
    is_mounted = false;
    current_screen = SCREEN_MAIN;
}

// ---- 3. get_input(): Ctrl+F passes through; no escape sequence yields it -----
// Every sequence a terminal sends for the keys this menu reads (and a few it
// doesn't), split every possible way across the 10 ms escape timeout, and
// with any one byte lost. Returns every key get_input() produced.
static std::vector<int> run_input(const std::vector<Arrival> &seq) {
    feed(seq);
    std::vector<int> out;
    uint64_t end = (seq.empty() ? 0 : seq.back().at_us) + 500000;
    while (now_us() < end || rxi < rx.size()) {
        int k = get_input();
        if (k != -1) out.push_back(k);
    }
    return out;
}

static void test_escape_sequences() {
    mock_now_ns = 0;
    std::vector<int> o = run_input({ { now_us(), 0x06 } });
    CHECK(o.size() == 1 && o[0] == 0x06, "Ctrl+F through get_input: %zu keys, first %d", o.size(), o.empty() ? -2 : o[0]);
    o = run_input({ { now_us(), 12 } });
    CHECK(o.size() == 1 && o[0] == 12, "Ctrl+L through get_input");
    o = run_input({ { now_us(), 0x07 } });
    CHECK(o.size() == 1 && o[0] == MANUAL_CHS_KEY, "Ctrl+G through get_input");

    const char *seqs[] = {
        "\x1b[A", "\x1b[B", "\x1b[C", "\x1b[D",          // arrows, CSI
        "\x1bOA", "\x1bOB", "\x1bOC", "\x1bOD",          // arrows, SS3 (application cursor mode)
        "\x1b[21~", "\x1b[5~", "\x1b[6~", "\x1b[2~", "\x1b[3~",   // F10, PgUp, PgDn, Ins, Del
        "\x1b[1;5B", "\x1b[1;2A", "\x1bOP", "\x1b[H", "\x1b[F", "\x1b[Z",
    };
    long runs = 0, seen_bare_B = 0;
    for (const char *s : seqs) {
        std::string q(s);
        size_t n = q.size();
        for (int lost = -1; lost < (int)n; lost++) {        // -1: nothing lost
            std::string v = q;
            if (lost >= 0) v.erase((size_t)lost, 1);
            size_t gaps = v.size() > 0 ? v.size() - 1 : 0;
            for (unsigned mask = 0; mask < (1u << gaps); mask++) {
                // a set bit: the next byte arrives 15 ms later (past the 10 ms timeout)
                std::vector<Arrival> a;
                uint64_t t = now_us() + 1000;
                for (size_t i = 0; i < v.size(); i++) {
                    if (i > 0 && (mask >> (i - 1)) & 1u) t += 15000;
                    a.push_back({ t, v[i] });
                }
                std::vector<int> keys = run_input(a);
                runs++;
                for (int k : keys) {
                    CHECK(k != FWUPDATE_KEY, "sequence %s (lost %d, split %x) gave the update key",
                          q.c_str() + 1, lost, mask);
                    CHECK(!fwupdate_key_opens_prompt(true, false, k), "sequence %s opened the prompt", q.c_str() + 1);
                    CHECK(!manual_chs_key_opens(true, false, k), "sequence %s opened manual CHS", q.c_str() + 1);
                    if (k == 'B') seen_bare_B++;
                }
            }
        }
    }
    // The harness can see the old fault: with B as the key, a split or
    // damaged Down arrow does hand back a bare 'B'.
    CHECK(seen_bare_B > 0, "no split sequence gave a bare 'B' (the harness would miss the old key)");
    printf("escape sequences: %ld runs, bare 'B' from %ld of them\n", runs, seen_bare_B);
}

// ---- 4. Y: reboot, wait, or refuse ------------------------------------------
static void confirm_with(bool mounted, uint64_t busy_for_ns, std::vector<Arrival> keys) {
    tty.clear();
    mock_usb_boot_requests = 0; busy_reads = 0;
    is_mounted = mounted;
    busy_until_ns = mock_now_ns + busy_for_ns;
    feed(keys);
    fwupdate_confirmed();
}

static void test_confirm() {
    mock_now_ns = 1000000000ull;
    confirm_with(false, 0, {});
    CHECK(mock_usb_boot_requests == 1 && tty.find("Rebooting") != std::string::npos, "idle: reboot %d", mock_usb_boot_requests);
    CHECK(busy_reads >= 1, "busy flag not consulted");

    uint64_t t0 = mock_now_ns;                          // a USB command still running for 5 s
    confirm_with(false, 5000000000ull, {});
    CHECK(mock_usb_boot_requests == 1, "busy 5 s: reboot %d", mock_usb_boot_requests);
    CHECK(mock_now_ns - t0 >= 5000000000ull, "rebooted after %llu ms, while the command ran",
          (unsigned long long)((mock_now_ns - t0) / 1000000));
    CHECK(tty.find("Waiting for a USB command") != std::string::npos, "no waiting message");
    CHECK(tty.find("Esc") == std::string::npos, "the waiting message promises Esc (review L-1)");

    t0 = mock_now_ns;                                   // never finishes: give up, do not reboot
    // (the Esc at 90 s ends a Y that would wait for ever, so the test ends too)
    confirm_with(false, 1000000000000ull, { { mock_now_ns / 1000 + 61000000, 'x' }, { mock_now_ns / 1000 + 90000000, 27 } });
    uint64_t ms = (mock_now_ns - t0) / 1000000;
    CHECK(mock_usb_boot_requests == 0, "busy forever: reboot %d", mock_usb_boot_requests);
    CHECK(ms >= 60000 && ms < 70000, "gave up after %llu ms", (unsigned long long)ms);
    CHECK(tty.find("USB still busy") != std::string::npos, "no refusal message");

    t0 = mock_now_ns;                                   // Esc while waiting cancels
    confirm_with(false, 1000000000000ull, { { mock_now_ns / 1000 + 2000000, 27 } });
    CHECK(mock_usb_boot_requests == 0 && (mock_now_ns - t0) < 3000000000ull, "Esc: reboot %d after %llu ms",
          mock_usb_boot_requests, (unsigned long long)((mock_now_ns - t0) / 1000000));

    confirm_with(true, 0, { { mock_now_ns / 1000 + 1000, 'x' } });   // mounted meanwhile
    CHECK(mock_usb_boot_requests == 0 && tty.find("Drive mounted") != std::string::npos, "mounted: reboot %d",
          mock_usb_boot_requests);
    is_mounted = false;
}

// ---- 5. the help row offers the key only when nothing is mounted -----------
static void test_help_row() {
    memset(&config, 0, sizeof(config));
    config.dev_base = 0xA0;
    current_screen = SCREEN_MAIN;
    is_mounted = false; tty.clear();
    update_main_menu();
    CHECK(tty.find("Ctrl+F: Firmware Update") != std::string::npos, "unmounted: not offered");
    CHECK(tty.find("B: Firmware") == std::string::npos, "old key still offered");
    CHECK(tty.find("\033[19;3H ESC: Quit to Main Menu  Ctrl+G: Manual CHS     \xe2\x86\x91") != std::string::npos,
          "unmounted: Ctrl+G not offered, or the arrows moved");
    is_mounted = true; tty.clear();
    update_main_menu();
    CHECK(tty.find("Firmware Update") == std::string::npos, "mounted: offered");
    CHECK(tty.find("Manual CHS") == std::string::npos, "mounted: Ctrl+G offered");
    CHECK(tty.find("\033[19;3H ESC: Quit to Main Menu                         \xe2\x86\x91") != std::string::npos,
          "mounted: row 19 is not the 0.6f3p7 row");
    is_mounted = false;
}

// Unmount forgets the captured IDENTIFY words, so a drive swapped on the
// cable is not judged by the previous drive's SMART / HPA support bits.
static void test_unmount_forgets_identify() {
    is_mounted = true; media_changed_waiting = false; mock_id_words_forgotten = 0;
    menu_unmount();
    CHECK(!is_mounted && media_changed_waiting, "unmount state %d %d", is_mounted, media_changed_waiting);
#if ATABOY_SAT
    CHECK(mock_id_words_forgotten == 1, "IDENTIFY words forgotten %d times", mock_id_words_forgotten);
#else
    CHECK(mock_id_words_forgotten == 0, "SAT=0 keeps no IDENTIFY words, forgotten %d times", mock_id_words_forgotten);
#endif
}

// ---- 6. review L-5 and L-1: core 1 and a running USB command ---------------
// Auto Detect and every debug key that touches the bus wait for the USB side
// to let go of it, up to IDE_BUS_WAIT_MS. Review L-1: on the board the wait
// can neither show a message nor see a key, since both travel through core
// 0, which is inside the callback. So it is silent, it cannot be cancelled
// (a key typed meanwhile is left for the menu, not taken by the wait), and
// having waited, it does NOT do what was asked: it says a USB command was
// running and nothing was sent. With nothing running, the key does its work.
// (Here the mock queue delivers keys whenever they are due, which the board
// would not; the checks below hold either way.)
static void test_bus_wait() {
    const int dkeys[] = { 'i', 'I', 't', 'T', 'e', 'E', 's', 'S', 'r', 'R' };
    CHECK(IDE_BUS_WAIT_MS >= IDE_HOST_BUDGET_MS + 1000, "the wait (%u ms) is shorter than a USB command may run",
          (unsigned)IDE_BUS_WAIT_MS);
    // Each debug key, with a USB command running for another 5 s.
    for (int k : dkeys) {
        mock_now_ns += 1000000000ull;
        tty.clear(); bus_uses = 0; bus_uses_while_busy = 0;
        busy_until_ns = mock_now_ns + 5000000000ull;
        current_screen = SCREEN_DEBUG;
        feed({ { now_us() + 2000000, 27 } });            // Esc typed during the wait
        uint64_t t0 = mock_now_ns;
        debug_key(k);
        uint64_t ms = (mock_now_ns - t0) / 1000000;
        CHECK(bus_uses == 0, "debug %c: %d bus uses after waiting for the USB command", k, bus_uses);
        CHECK(ms >= 5000 && ms < 5100, "debug %c: waited %llu ms for a command that ran 5 s", k, (unsigned long long)ms);
        CHECK(tty.find("A USB command was running, nothing sent. Try again") != std::string::npos, "debug %c: no message", k);
        CHECK(tty.find("Esc") == std::string::npos && tty.find("Waiting") == std::string::npos,
              "debug %c: promises what the board cannot do", k);
        CHECK(rxi == 0, "debug %c: the wait took a key", k);
        // Pressed again, nothing running: it does its work.
        feed({}); tty.clear(); bus_uses = 0;
        debug_key(k);
        CHECK(bus_uses > 0 && bus_uses_while_busy == 0, "debug %c again: %d bus uses", k, bus_uses);
    }
    // Never lets go: nothing sent, and a message.
    feed({});
    tty.clear(); bus_uses = 0;
    busy_until_ns = mock_now_ns + 1000000000000ull;
    uint64_t t0 = mock_now_ns;
    debug_key('t');
    uint64_t ms = (mock_now_ns - t0) / 1000000;
    CHECK(bus_uses == 0 && ms >= IDE_BUS_WAIT_MS && ms < IDE_BUS_WAIT_MS + 1000, "busy for ever: %d bus uses after %llu ms",
          bus_uses, (unsigned long long)ms);
    CHECK(tty.find("USB still busy") != std::string::npos, "busy for ever: no message");
    // Other keys wait for nothing and send nothing.
    int br = busy_reads; bus_uses = 0;
    debug_key('x'); debug_key(KEY_ENTER);
    CHECK(bus_uses == 0 && busy_reads == br, "unknown debug keys: %d bus uses, %d busy reads", bus_uses, busy_reads - br);
    // Auto Detect: waits, then does not probe; says so, and a key goes on.
    mock_now_ns += 1000000000ull;
    tty.clear(); bus_uses = 0; bus_uses_while_busy = 0;
    busy_until_ns = mock_now_ns + 5000000000ull;
    current_screen = SCREEN_MAIN; show_detect_result = false;
    feed({ { now_us() + 7000000, 'x' } });
    t0 = mock_now_ns;
    bool overlay = run_auto_detect();
    CHECK(bus_uses == 0 && !overlay && !show_detect_result && mock_now_ns - t0 >= 5000000000ull,
          "auto detect after a wait: %d bus uses, overlay %d", bus_uses, overlay);
    CHECK(tty.find("A USB command was running, nothing sent. Press a key") != std::string::npos, "auto detect: no message");
    // Pressed again, nothing running: it probes, and shows the result.
    feed({}); tty.clear(); bus_uses = 0;
    overlay = run_auto_detect();
    CHECK(bus_uses > 0 && bus_uses_while_busy == 0 && overlay && show_detect_result,
          "auto detect, nothing running: %d bus uses, overlay %d shown %d", bus_uses, overlay, show_detect_result);
    show_detect_result = false;
    // Auto Detect, never lets go: no probe, a message, and a key to go on.
    tty.clear(); bus_uses = 0;
    busy_until_ns = mock_now_ns + 1000000000000ull;
    feed({ { now_us() + (IDE_BUS_WAIT_MS + 1000) * 1000ull, 'x' } });
    overlay = run_auto_detect();
    CHECK(bus_uses == 0 && !overlay && !show_detect_result && tty.find("USB still busy") != std::string::npos,
          "auto detect, busy for ever: %d bus uses, overlay %d", bus_uses, overlay);
    busy_until_ns = 0;
}

// ---- 7. Debug E: how the reset went, in 68 columns --------------------------
static void test_debug_errors_row() {
    struct { bool reset, hw, failed; const char *tail; } cases[] = {
        { false, false, false, "DH:A9  drained" },
        { true,  false, false, "DH:A9  drained  reset" },
        { true,  false, true,  "DH:A9  drained  reset FAILED" },
        { true,  true,  false, "DH:A9  drained  HW reset" },
        { true,  true,  true,  "DH:A9  drained  HW reset FAILED" },
    };
    for (auto &c : cases) {
        memset(&stub_fail, 0, sizeof stub_fail);
        stub_fail.kind = IDE_FAIL_TIMEOUT; stub_fail.command = 0x30; stub_fail.status = 0xD0;
        stub_fail.tf[4] = 0xA9; stub_fail.drained = true;
        stub_fail.reset = c.reset; stub_fail.hw_reset = c.hw; stub_fail.reset_failed = c.failed;
        tty.clear();
        run_debug_errors();
        std::string want = std::string("ST:D0 ERR:00 SC:00 SN:00 CL:00 CH:00 ") + c.tail;
        size_t at = tty.find(want);
        CHECK(at != std::string::npos, "row for reset %d hw %d failed %d not found: want \"%s\"", c.reset, c.hw, c.failed, want.c_str());
        // The row after it must not run on: the next byte is padding or the colour reset.
        if (at != std::string::npos) {
            char nx = tty[at + want.size()];
            CHECK(nx == ' ' || nx == '\033', "row runs on after \"%s\": %d", c.tail, nx);
        }
        CHECK(want.size() <= 68, "row is %zu columns", want.size());
        CHECK(tty.find("Reset still running") == std::string::npos, "pending shown for a finished reset");
    }
    // Review M-1: the drive was not back when the USB command's time ran out.
    memset(&stub_fail, 0, sizeof stub_fail);
    stub_fail.kind = IDE_FAIL_TIMEOUT; stub_fail.command = 0x20; stub_fail.reset = true; stub_fail.pending = true;
    tty.clear();
    run_debug_errors();
    CHECK(tty.find("Reset still running: the next USB command waits for it") != std::string::npos, "pending not shown");
    // The kinds added in 0.6f3p7: no time left, and a SAT command that ended badly.
    struct { uint8_t kind; const char *word; } kinds[] = { { IDE_FAIL_NO_TIME, "cmd 20 no time at" },
                                                           { IDE_FAIL_BAD_END, "cmd 20 bad end at" } };
    for (auto &k : kinds) {
        stub_fail.kind = k.kind; stub_fail.pending = false;
        tty.clear();
        run_debug_errors();
        CHECK(tty.find(k.word) != std::string::npos, "kind %u: \"%s\" not shown", k.kind, k.word);
    }
    memset(&stub_fail, 0, sizeof stub_fail);
}

// ---- 8. the banner names the build ------------------------------------------
static void test_banner() {
    tty.clear();
    draw_bios_frame();
#if ATABOY_SAT && ATABOY_SAT_SMART_SAVES
    CHECK(tty.find("\033[1;4H\033[37;1mATAboy Setup Utility v0.6f3p8 (fork+smartsaves) - (C) 2026 obsoletetech.us") != std::string::npos,
          "SMART opt-in build: banner does not say so");
#else
    CHECK(tty.find("\033[1;10H\033[37;1mATAboy Setup Utility v0.6f3p8 (fork) - (C) 2026 obsoletetech.us") != std::string::npos,
          "banner is not v0.6f3p8 (fork)");
    CHECK(tty.find("smartsaves") == std::string::npos, "shipping build banner claims SMART saves");
#endif
}

// ---- Features, IORDY: the menu never sets the pin itself -------------------
// Review of 0.6f3p7 (LOW): switching IORDY on used to set the pin at once,
// undoing ide.c's hold while a drive comes back from a hardware reset. The
// menu now leaves the pin to ide.c: it asks for it at once only when nothing
// is mounted and no USB command is running (core 0 puts the setting on the
// pin at the next USB command otherwise). ide.c's side, the hold itself, is
// tested in test_read (test_iordy_setting_deferred).
static void test_iordy_toggle() {
    const struct { bool mounted, busy, now; } cases[] = {
        { false, false, true }, { true, false, false }, { false, true, false }, { true, true, false } };
    for (auto &c : cases) {
        is_mounted = c.mounted;
        busy_until_ns = c.busy ? mock_now_ns + 1000000000ull : 0;
        bool was = config.iordy_enabled;
        int w0 = mock_iordy_pin_writes, f0 = mock_iordy_follows;
        features_toggle_iordy();
        CHECK(config.iordy_enabled == !was, "mounted %d busy %d: setting not switched", c.mounted, c.busy);
        CHECK(mock_iordy_pin_writes == w0, "mounted %d busy %d: the menu set the pin itself", c.mounted, c.busy);
        CHECK(mock_iordy_follows - f0 == (c.now ? 1 : 0), "mounted %d busy %d: asked ide.c %d times, want %d",
              c.mounted, c.busy, mock_iordy_follows - f0, c.now ? 1 : 0);
    }
    is_mounted = false;
    busy_until_ns = 0;
    config.iordy_enabled = false;
}

int main() {
    test_decisions();
    test_key_in_menus();
    test_escape_sequences();
    test_confirm();
    test_help_row();
    test_unmount_forgets_identify();
    test_bus_wait();
    test_debug_errors_row();
    test_banner();
    test_iordy_toggle();
    printf("%d checks, %d failed\n", checks, failures);
    return failures > 255 ? 255 : failures;
}
