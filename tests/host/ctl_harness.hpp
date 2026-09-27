// The ATAboy Control API (palimpsest api/SPEC-v1.md) on the PC: the real
// ctl*.c, menus.c, ide.c and obs.c, against the simulated drive (sim.hpp), a
// stand-in for TinyUSB's CDC instance 1 (the Control port), and a
// deterministic core 1 that comes to its idle point when the test says.
// Shared by test_ctl.cpp (the tests, and "serve" for the end-to-end run with
// the real host client) and ctl_fuzz.cpp. Built with ATABOY_CTL=1 only.
#pragma once
#include "mock_pico.h"
#include "sim.hpp"
#include "obs.c"
#include "ide.c"

#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#if !ATABOY_CTL
#error "the Control API's tests need -DATABOY_CTL=1"
#endif

// ---- stand-ins menus.c needs (as test_manual_chs.cpp) ----------------------
typedef struct { int unused; } queue_t;
#define PICO_ERROR_TIMEOUT (-1)
struct Arrival { uint64_t at_us; char c; };
static std::vector<Arrival> rx;
static size_t rxi;
static std::string tty;                     // everything the TUI sent to interface 0
static void (*on_idle)() = nullptr;
static uint64_t idle_after_ns = 0;
// R-API-1 finding 1 (SPEC-v1 5.3, I-23): a job body must never touch the
// TUI's queues. Each use while a job runs is counted; a read then gets 'N'
// at once, so a job that would wait for a key ends instead of hanging, and
// the count fails the test.
static int job_tui_io = 0;
static bool job_running_now();
static inline void queue_add_blocking(queue_t *, const void *c) {
    if (job_running_now()) job_tui_io++;
    tty.push_back(*(const char *)c);
}
// test_ctl core1-tui: core 1 is its real loop, and core 0's passes (and the
// test's timed checks) come from its input polls (core0_tick, below).
static bool core0_in_loop = false;
static void core0_tick();
static inline bool queue_try_remove(queue_t *, void *out) {
    if (job_running_now()) { job_tui_io++; *(char *)out = 'N'; return true; }
    if (core0_in_loop) core0_tick();
    if (rxi < rx.size() && get_absolute_time() >= rx[rxi].at_us) { *(char *)out = rx[rxi++].c; return true; }
    if (on_idle && rxi == rx.size() && mock_now_ns > idle_after_ns) on_idle();
    return false;
}
static inline absolute_time_t make_timeout_time_us(uint64_t us) { return get_absolute_time() + us; }
static inline bool time_reached(absolute_time_t t) { return get_absolute_time() >= t; }

#include "menus.c"
#include "ctl_json.c"
#include "ctl_sha256.c"
#include "ctl_policy.c"
#include "ctl.c"
#include "ctl_job.c"
#if ATABOY_BOARD_JUMPERS
#include "board_jumpers.c"          // stage 3: JP1 and JP2 against mock_pico.h's pad register file
#endif

static bool job_running_now() { return ctl_job.state == CTL_JOB_RUNNING; }
bool core1_stack_intact(void) { return true; }
void fault_stop(uint32_t code) { printf("FAIL: fault_stop(%08X) reached\n", (unsigned)code); exit(250); }
// A test that drives the tripwire on purpose (identify.forbid, I-22) sets
// trip_expected and comes back to its setjmp; anywhere else it is a failure.
static bool trip_expected = false;
static jmp_buf trip_jb;
void ide_trip_stop(void) {
    if (trip_expected) longjmp(trip_jb, 1);
    printf("FAIL: the tripwire fired\n"); exit(251);
}

uint64_t mock_now_ns = 0;
uint32_t mock_gpio_out = 0;
MockSio mock_sio;
SimDrive sim;
config_t config;
int mock_usb_boot_requests = 0;
queue_t cdc_tx_queue, cdc_rx_queue;
volatile bool cdc_connected = false, is_mounted = false, media_changed_waiting = false;
bool tud_msc_set_sense(uint8_t, uint8_t, uint8_t, uint8_t) { return true; }
static int saves = 0;
void config_save(void) { saves++; }
// Load Setup Defaults' effect on what the API reports (config.c's values).
void config_defaults(void) {
    config.drive_write_protected = true; config.pipelined_reads = false; config.auto_mount = false;
    config.use_lba_mode = false; config.cyls = 0; config.heads = 0; config.spt = 0; config.lba_sectors = 0;
    config.dev_base = 0xA0;
}
static uint64_t busy_until_ns = 0;
bool usb_msc_ide_busy(void) { return mock_now_ns < busy_until_ns; }

// The settings sector as XIP would read it, and what config_load() found.
static config_t flash_sector;
static bool loaded_from_flash_stub = false;
void config_flash_copy(config_t *out) { *out = flash_sector; }
bool config_loaded_from_flash(void) { return loaded_from_flash_stub; }

static uint64_t rand_value = 0x7c1e2a90d4b3f001ull;
static uint64_t rand_calls = 0;     // the first call (dev.boot) is rand_value; each later one differs (lease tokens)
uint64_t get_rand_64(void) { return rand_value + 0x9E3779B97F4A7C15ull * rand_calls++; }
static const char *board_serial = "E6614103E7A1B22C";
void pico_get_unique_board_id_string(char *id_out, unsigned len) {
    snprintf(id_out, len, "%s", board_serial);
}

// ---- the Control port: TinyUSB's CDC instance 1 ----------------------------
// RX: what the host sent and the firmware has not read. TX: TinyUSB's 512-byte
// FIFO, which the host empties only while `reading`.
struct CtlPort {
    std::deque<uint8_t> rx;
    std::string fifo, got;                  // got: every byte the host has read
    bool dtr = true, reading = true;
    uint32_t tx_cap = 512;
    uint32_t rx_cap = 256;                  // CFG_TUD_CDC_RX_BUFSIZE: what one pass can find there
    uint32_t rx_now = 0;                    // bytes in TinyUSB's RX FIFO this pass
    int calls_this_task = 0;
    int clears = 0;
} port;

static void calls_guard() {
    // A writer that loops waiting for room (a blocking one) never returns:
    // stop it here instead of hanging the run (mutant "writer blocks").
    if (++port.calls_this_task > 100000) { printf("FAIL: ctl_task never returns (a blocking loop)\n"); exit(252); }
}
// TinyUSB's RX FIFO holds 256 bytes; the host's other bytes wait (NAKed)
// until a tud_task() pass moves them in, which happens between ctl_task()
// passes, never during one.
bool tud_cdc_n_connected(uint8_t itf) { calls_guard(); return itf == 1 && port.dtr; }
uint32_t tud_cdc_n_available(uint8_t itf) { calls_guard(); return itf == 1 ? port.rx_now : 0; }
uint32_t tud_cdc_n_read(uint8_t itf, void *b, uint32_t n) {
    calls_guard();
    uint32_t k = 0;
    while (itf == 1 && k < n && port.rx_now) { ((uint8_t *)b)[k++] = port.rx.front(); port.rx.pop_front(); port.rx_now--; }
    return k;
}
void tud_cdc_n_read_flush(uint8_t itf) {
    if (itf != 1) return;
    while (port.rx_now) { port.rx.pop_front(); port.rx_now--; }
}
uint32_t tud_cdc_n_write(uint8_t itf, void const *b, uint32_t n) {
    calls_guard();
    if (itf != 1) return 0;
    uint32_t room = port.tx_cap - (uint32_t)port.fifo.size();
    if (n > room) n = room;
    port.fifo.append((const char *)b, n);
    return n;
}
uint32_t tud_cdc_n_write_flush(uint8_t) { calls_guard(); return 0; }
uint32_t tud_cdc_n_write_available(uint8_t itf) {
    calls_guard();
    return itf == 1 ? port.tx_cap - (uint32_t)port.fifo.size() : 0;
}
bool tud_cdc_n_write_clear(uint8_t itf) { if (itf == 1) { port.fifo.clear(); port.clears++; } return true; }

// ---- time, core 0 and core 1 ------------------------------------------------
static uint8_t core1_screen = CTL_TUI_MAIN;     // what core 1 is showing at its idle point
static bool core1_runs = true;                  // false: core 1 is inside a dialog, never at its top
static uint64_t task_max_ns = 0;                // the longest ctl_task() pass, in modelled time

static void (*after_task)() = nullptr;      // ctl_fuzz: its oracle, after every pass
static void one_task() {
    port.calls_this_task = 0;
    port.rx_now = port.rx.size() < port.rx_cap ? (uint32_t)port.rx.size() : port.rx_cap;   // tud_task() filled the FIFO
    uint64_t t0 = mock_now_ns;
    ctl_task();
    if (after_task) after_task();
    if (mock_now_ns - t0 > task_max_ns) task_max_ns = mock_now_ns - t0;
    if (port.reading) { port.got += port.fifo; port.fifo.clear(); }
}

// core1-tui: a pass of core 0 every 1 ms of core 1's polling, unless core 0
// is held (inside a long USB callback, say), and the timed checks.
static bool core0_hold = false;
static uint64_t core0_last_ns = 0;
static std::vector<std::pair<uint64_t, std::function<void()>>> timeline;   // at us, run
static size_t tli = 0;
static void core0_tick() {
    while (tli < timeline.size() && get_absolute_time() >= timeline[tli].first) { auto f = timeline[tli++].second; f(); }
    if (!core0_hold && mock_now_ns - core0_last_ns >= 1000000) { core0_last_ns = mock_now_ns; one_task(); }
}

// ms of time: each 1 ms, a pass of core 0's loop and, when it runs, one of
// core 1's (its idle point).
static void pump(uint32_t ms = 20) {
    for (uint32_t i = 0; i < ms; i++) {
        mock_now_ns += 1000000;
        one_task();
        if (core1_runs) ctl_core1_idle(core1_screen);
    }
}

static void host_send(const std::string &s) { for (char ch : s) port.rx.push_back((uint8_t)ch); }

// The lines the host has read so far, each without its \n; a last partial
// line stays in port.got.
static std::vector<std::string> host_lines() {
    std::vector<std::string> out;
    size_t i;
    while ((i = port.got.find('\n')) != std::string::npos) { out.push_back(port.got.substr(0, i)); port.got.erase(0, i + 1); }
    return out;
}

#if ATABOY_BOARD_JUMPERS
static bool power_up_fitted[2];             // shunts already on at power-up (JP1, JP2)
#endif
// Back to power-up: the API's state, the drive, the menus, the log.
static void power_up() {
    sim = SimDrive();
    mock_now_ns += 1000000000ull;
    memset(&config, 0, sizeof config);
    config.dev_base = 0xA0;
    config.drive_write_protected = true;
    flash_sector = config;
    flash_sector.magic = CONFIG_MAGIC;
    flash_sector.crc = config_crc(&flash_sector);
    loaded_from_flash_stub = true;
    ide_select_device(0xA0);
    hdd_model_raw[0] = 0; hdd_status_text[0] = 0;
    cur_cyls = cur_heads = cur_spt = 0; total_lba_sectors = 0; use_lba_mode = false;
    show_detect_result = false; force_detect = false;
    current_screen = SCREEN_MAIN; is_mounted = false;
    rec.stage = REC_NONE; memset(&last_fail, 0, sizeof last_fail);
    chs_geometry_lost = false; manual_chs_active = false; iordy_held = false;
    native_geo.on = false; idp_written = false; mchs_no_identify = false; native_declared = false;
    identify_forbidden = false; memset(&rules_seen, 0, sizeof rules_seen);
    memset(&refused, 0, sizeof refused);
    memset(&self_resets, 0, sizeof self_resets);
    memset(&trip, 0, sizeof trip);
    memset(&idc, 0, sizeof idc);
    busy_until_ns = 0;
    obs_test_reset(0, 0);
    memset(&c, 0, sizeof c);
    rand_calls = 0;
    pres_text[0] = 0; ctl_lease_gen = 0; ctl_lease_by = 0;
    job_tui_io = 0;
    memset(&ctl_job, 0, sizeof ctl_job);
    ctl_tui_state = CTL_TUI_BOOT;
    ctl_epoch = 0; ctl_wp_last = CTL_WP_BOOT; ctl_setup_src = CTL_SRC_NONE; ctl_setup_at_ms = 0;
    ctl_mount_seq = 0; ctl_mount_log_seq = 0;
    fail_detach = false;
    dtr_dropped = false; usb_mounted = false;
    memset(ctl_test_op, 0, sizeof ctl_test_op);
    port = CtlPort();
    sim_reset_hook = nullptr;
#if ATABOY_BOARD_JUMPERS
    // The pads at their reset state, both shunts off, then set up as main()
    // does before tusb_init() (ATAboy.c).
    mock_pads_reset();
    mock_gpio_oe &= ~((1u << 30) | (1u << 31));
    mock_jumper_fitted[0] = power_up_fitted[0];
    mock_jumper_fitted[1] = power_up_fitted[1];
    mock_floating_high = false;
    mock_jumper_let_low = 0;
    memset(&bj, 0, sizeof bj);
    board_jumpers_init();
#endif
    tty.clear(); rx.clear(); rxi = 0;
    core1_screen = CTL_TUI_MAIN; core1_runs = true;
    ctl_init();
    pump(2);
#if ATABOY_BOARD_JUMPERS
    // JP1 read OPEN 20 times over 10 ms: released (UNKNOWN to OPEN, one
    // epoch count), as a few ms after every power-up on the unit.
    pump(25);
#endif
}

// ---- a strict reader for the responses (SPEC-v1 3.1's response grammar) ----
struct J {
    enum T { OBJ, ARR, STR, INT, BOOL, NUL } t = NUL;
    std::vector<std::pair<std::string, std::shared_ptr<J>>> obj;
    std::vector<std::shared_ptr<J>> arr;
    std::string s;
    uint64_t n = 0;
    bool b = false;
    const J *get(const std::string &k) const {
        for (auto &m : obj) if (m.first == k) return m.second.get();
        return nullptr;
    }
    const J &operator[](const std::string &k) const {
        static J none;
        const J *p = get(k);
        return p ? *p : none;
    }
    bool is(const char *str) const { return t == STR && s == str; }
};

struct JParse {
    const std::string &p; size_t i = 0; std::string err;
    explicit JParse(const std::string &s) : p(s) {}
    bool fail(const char *e) { if (err.empty()) err = e; return false; }
    bool str(std::string &out) {
        if (p[i] != '"') return fail("not a string");
        i++;
        while (i < p.size() && p[i] != '"') {
            if (p[i] == '\\') {
                if (i + 1 >= p.size() || (p[i + 1] != '"' && p[i + 1] != '\\')) return fail("an escape other than \\\" or \\\\");
                out.push_back(p[i + 1]); i += 2; continue;
            }
            out.push_back(p[i++]);
        }
        if (i >= p.size()) return fail("unterminated string");
        i++;
        return true;
    }
    bool val(J &v, int depth) {
        if (depth > 8) return fail("nesting deeper than 8");
        if (i >= p.size()) return fail("ended early");
        char ch = p[i];
        if (ch == '{') {
            v.t = J::OBJ; i++;
            if (p[i] == '}') { i++; return true; }
            for (;;) {
                std::string k;
                if (!str(k)) return false;
                for (auto &m : v.obj) if (m.first == k) return fail("duplicate key");
                if (p[i++] != ':') return fail("no colon");
                auto sub = std::make_shared<J>();
                if (!val(*sub, depth + 1)) return false;
                v.obj.push_back({ k, sub });
                if (p[i] == ',') { i++; continue; }
                if (p[i] == '}') { i++; return true; }
                return fail("no comma or brace");
            }
        }
        if (ch == '[') {
            v.t = J::ARR; i++;
            if (p[i] == ']') { i++; return true; }
            for (;;) {
                auto sub = std::make_shared<J>();
                if (!val(*sub, depth + 1)) return false;
                v.arr.push_back(sub);
                if (p[i] == ',') { i++; continue; }
                if (p[i] == ']') { i++; return true; }
                return fail("no comma or bracket");
            }
        }
        if (ch == '"') { v.t = J::STR; return str(v.s); }
        if (ch >= '0' && ch <= '9') {
            size_t s0 = i;
            while (i < p.size() && p[i] >= '0' && p[i] <= '9') i++;
            if (p[s0] == '0' && i - s0 > 1) return fail("leading zero");
            if (i - s0 > 16) return fail("integer too long");
            v.t = J::INT; v.n = strtoull(p.substr(s0, i - s0).c_str(), 0, 10);
            if (v.n > 9007199254740991ull) return fail("integer over 2^53-1");
            if (i < p.size() && (p[i] == '.' || p[i] == 'e' || p[i] == 'E')) return fail("a float");
            return true;
        }
        if (p.compare(i, 4, "true") == 0) { v.t = J::BOOL; v.b = true; i += 4; return true; }
        if (p.compare(i, 5, "false") == 0) { v.t = J::BOOL; v.b = false; i += 5; return true; }
        if (p.compare(i, 4, "null") == 0) { v.t = J::NUL; i += 4; return true; }
        return fail("not a value");
    }
};

// One response line (without its \n) as the grammar allows, or an error.
static bool parse_response(const std::string &line, J &out, std::string *why = nullptr) {
    for (unsigned char ch : line)
        if (ch < 0x20 || ch > 0x7E) { if (why) *why = "a byte outside 0x20-0x7E"; return false; }
    if (line.size() + 1 > CTL_MAX_RESPONSE) { if (why) *why = "over 16,384 bytes"; return false; }
    JParse jp(line);
    out = J();
    if (!jp.val(out, 1) || out.t != J::OBJ) { if (why) *why = jp.err.empty() ? "not an object" : jp.err; return false; }
    if (jp.i != line.size()) { if (why) *why = "trailing bytes"; return false; }
    return true;
}

// The envelope every response has (SPEC-v1 3.3): exactly these fields.
static std::string envelope_problem(const J &r) {
    static const char *allowed[] = { "v", "id", "ok", "outcome", "result", "retry", "err", "dev", "replayed" };
    for (auto &m : r.obj) {
        bool known = false;
        for (auto a : allowed) if (m.first == a) known = true;
        if (!known) return "unknown envelope field " + m.first;
    }
    if (r["v"].t != J::INT || r["v"].n != 1) return "v";
    if (!r.get("id") || (r["id"].t != J::INT && r["id"].t != J::NUL)) return "id";
    const J &d = r["dev"];
    if (d.t != J::OBJ || d.obj.size() != 5 || !d["serial"].is(board_serial) || d["boot"].t != J::STR ||
        d["boot"].s.size() != 16 || d["up_ms"].t != J::INT || d["seq"].t != J::INT || d["epoch"].t != J::INT)
        return "dev";
    std::string o = r["outcome"].s;
    if (o == "DONE") {
        if (r["ok"].t != J::BOOL || !r["ok"].b || r["result"].t != J::OBJ || r.get("err") || r.get("retry")) return "DONE shape";
    } else if (o == "NOT_EXECUTED" || o == "FAILED" || o == "UNKNOWN") {
        if (r["ok"].t != J::BOOL || r["ok"].b || r["err"].t != J::OBJ || r["retry"].t != J::STR) return "error shape";
        bool code_known = false;
        for (int e = 1; e < CTL_E_COUNT; e++) if (r["err"]["code"].s == ctl_err_name[e]) code_known = true;
        if (!code_known) return "err.code " + r["err"]["code"].s;
    } else {
        return "outcome " + o;
    }
    return "";
}
