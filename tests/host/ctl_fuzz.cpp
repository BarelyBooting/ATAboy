// A deterministic mutational fuzzer for the ATAboy Control API's parser, its
// envelope checks and the line layer (palimpsest api/SPEC-v1.md 11.3), run
// through the real ctl*.c on the host (ctl_harness.hpp). libFuzzer needs a
// clang with -fsanitize=fuzzer, which this host does not have, and the
// suite's msys64 g++ has no AddressSanitizer on Windows; so this is the
// spec's fallback: fixed seeds, a fixed iteration count, reproducible, built
// with UndefinedBehaviorSanitizer in trap mode (an out-of-bounds index into
// a fixed array, an overflow, stops the run), stack protector on every
// function, and the C++ library's own assertions.
//
//   ctl_fuzz <corpus dir> [iterations] [seed]
//
// Inputs: the corpus (one line per text line, as hex: the spec's examples,
// the lines the real host client sends (ctl_e2e.py writes them), noise such
// as modem probes and TUI keystrokes), each then mutated grammar-aware (one
// field of a valid envelope changed) or byte by byte (flips, every byte
// value, truncation, splices, overlong lines). Between lines, events at
// random: DTR drops, USB resets, a core 1 away from its idle point, clock
// gaps over 50 ms.
//
// Oracle, per line (SPEC-v1 11.3): with DTR high, exactly one response for a
// non-empty line and none for an empty one; every response is one line of
// the response grammar with the closed envelope; a response other than DONE
// leaves the state as it was (I-10, I-27: the epoch included); replayed:true
// only for a line byte-identical to an earlier one with its id, and then a
// response that id got before (I-18); a mutating request never runs twice
// for one id (its API entries in the command log, 5.4, count the runs); and
// no state change is ever missed by the epoch's hooks (epoch_poll's
// cross-check). Stage 2: the ratchet's five ops are real, with the human's
// own changes between lines (a TUI mount, write protect or Pipelined Reads
// switched back), so wp.on, unmount and the rest have something to change.
// Exit status: 0, or the number of violations (at most 255).
#include "ctl_harness.hpp"
#include <algorithm>
#include <dirent.h>
#include <set>

static uint64_t rng = 0x9E3779B97F4A7C15ull;
static uint64_t next() { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return rng; }
static uint32_t below(uint32_t n) { return n ? (uint32_t)(next() % n) : 0; }

static int violations = 0;
static uint64_t lines_sent = 0, responses = 0, done_n = 0, replays = 0;
static std::map<std::string, int> codes, done_ops;
static void violation(const char *what, const std::string &line, const std::string &resp) {
    violations++;
    if (violations <= 20) {
        printf("VIOLATION %s\n  line (%zu bytes): ", what, line.size());
        for (size_t i = 0; i < line.size() && i < 200; i++) printf("%02x", (unsigned char)line[i]);
        printf("\n  response: %.300s\n", resp.c_str());
    }
}

static std::vector<std::string> corpus;
static void load_corpus(const char *dir) {
    DIR *d = opendir(dir);
    if (!d) return;
    std::vector<std::string> names;
    while (struct dirent *e = readdir(d)) {
        std::string n = e->d_name;
        if (n.size() > 4 && n.substr(n.size() - 4) == ".hex") names.push_back(n);
    }
    closedir(d);
    std::sort(names.begin(), names.end());          // the same order every run
    for (auto &n : names) {
        FILE *f = fopen((std::string(dir) + "/" + n).c_str(), "rb");
        if (!f) continue;
        char buf[8192];
        while (fgets(buf, sizeof buf, f)) {
            std::string h(buf), line;
            while (!h.empty() && (h.back() == '\n' || h.back() == '\r')) h.pop_back();
            if (h.empty() || h[0] == '#') continue;
            for (size_t i = 0; i + 1 < h.size(); i += 2) line.push_back((char)strtoul(h.substr(i, 2).c_str(), 0, 16));
            corpus.push_back(line);
        }
        fclose(f);
    }
}

static const char *ops[] = { "hello", "status", "identify.cached", "config.get", "taskfile.snapshot", "log.read",
                             "hashes.read", "debug.e", "selftest.status", "chip.info", "job.last", "wp.on",
                             "pipelined.off", "identify.forbid", "unmount", "detach", "session.open", "session.close",
                             "setup.ctrlg", "mount", "wp.off", "setup.detect", "setup.geometry", "debug.r", "" };
static const char *vals[] = { "0", "1", "2", "9007199254740991", "9007199254740992", "18446744073709551616", "-1", "1.5",
                              "1e3", "01", "true", "false", "null", "\"\"", "\"x\"", "{}", "[]", "{\"a\":{\"b\":{}}}",
                              "\"00112233445566778899aabbccddeeff\"", "\"a b\"", "\"a\\\"b\"", "2147483647",
                              "2147483648", "16777215", "16777216", "128", "129", "96", "97" };
static const char *keys[] = { "v", "id", "op", "args", "token", "cursor", "max_entries", "include_data", "expect_epoch",
                              "expect_mount_seq", "x", "", "V", "ID" };
// A small number, or the current one now and then, so both a match and a
// mismatch come up (expect_mount_seq, expect_epoch).
static std::string near(uint32_t cur) { return std::to_string(below(3) ? cur : (uint32_t)below(4)); }

// A valid envelope, then one field changed.
static std::string grammar_line() {
    // Half the time an id above every mutating id so far, so mutating
    // requests run (and can be resent byte for byte); else a small one.
    uint32_t id = below(2) ? c.replay.max_id + 1 + below(2) : 1 + below(40);
    std::string op = ops[below(sizeof ops / sizeof ops[0])];
    std::string args = "{}";
    if (op == "log.read" || op == "hashes.read") args = "{\"cursor\":" + std::to_string(below(50)) + ",\"max_entries\":" + std::to_string(1 + below(96)) + "}";
    if (op == "debug.e" && below(2)) args = "{\"include_data\":true}";
    if ((op == "wp.on" || op == "pipelined.off" || op == "identify.forbid") && below(3) == 0)
        args = "{\"expect_epoch\":" + near(ctl_epoch) + "}";
    if (op == "unmount" || op == "detach") {
        args = "{\"expect_mount_seq\":" + near(ctl_mount_seq);
        if (below(3) == 0) args += ",\"expect_epoch\":" + near(ctl_epoch);
        args += "}";
    }
    std::vector<std::pair<std::string, std::string>> f = {
        { "v", "1" }, { "id", std::to_string(id) }, { "op", "\"" + op + "\"" }, { "args", args } };
    switch (below(10)) {
    case 0: f.erase(f.begin() + below((uint32_t)f.size())); break;
    case 1: f[below((uint32_t)f.size())].second = vals[below(sizeof vals / sizeof vals[0])]; break;
    case 2: f.push_back({ keys[below(sizeof keys / sizeof keys[0])], vals[below(sizeof vals / sizeof vals[0])] }); break;
    case 3: { auto x = f[below((uint32_t)f.size())]; f.insert(f.begin() + below((uint32_t)f.size()), x); break; }
    case 4: std::swap(f[below((uint32_t)f.size())], f[below((uint32_t)f.size())]); break;
    default: break;                                 // valid as it is
    }
    std::string s = "{";
    for (size_t i = 0; i < f.size(); i++) s += (i ? "," : "") + std::string("\"") + f[i].first + "\":" + f[i].second;
    s += "}";
    if (below(8) == 0) s.insert(below((uint32_t)s.size() + 1), std::string(1 + below(3), ' '));
    return s;
}

static std::string byte_mutant(std::string s) {
    int n = 1 + (int)below(4);
    for (int k = 0; k < n; k++) {
        switch (below(7)) {
        case 0: if (!s.empty()) s[below((uint32_t)s.size())] ^= (char)(1u << below(8)); break;
        case 1: s.insert(s.begin() + below((uint32_t)s.size() + 1), (char)below(256)); break;
        case 2: if (!s.empty()) s.erase(below((uint32_t)s.size()), 1 + below(8)); break;
        case 3: s = s.substr(0, below((uint32_t)s.size() + 1)); break;
        case 4: if (!corpus.empty()) { const std::string &o = corpus[below((uint32_t)corpus.size())];
                    s = s.substr(0, below((uint32_t)s.size() + 1)) + o.substr(below((uint32_t)o.size() + 1)); } break;
        case 5: s += std::string(below(4) ? 1000 + below(60) : 20000, (char)(0x20 + below(95))); break;
        case 6: s += std::string(1 + below(40), ' '); break;
        }
    }
    return s;
}

// What a line other than a DONE must leave as it was (I-10). dev.seq, the
// parse and abandon counters, the rate window and the replay table are the
// protocol's own bookkeeping and are left out.
static std::string digest() {
    std::string s((const char *)&config, sizeof config);
    uint8_t h[OBS_PAGE_HEADER];
    obs_log_page(0, h, sizeof h); s.append((const char *)h + 16, 8);
    obs_hash_page(0, h, sizeof h); s.append((const char *)h + 16, 8);
    s += std::to_string(is_mounted) + std::to_string(mchs_no_identify) + std::to_string(identify_forbidden) + "/" +
         std::to_string(sim.reg_writes) + "/" + std::to_string(sim.cmd_log.size()) + "/" + std::to_string(c.jl_present) +
         "/" + std::to_string(ctl_epoch) + "/" + std::to_string(ctl_mount_seq) + "/" + std::to_string(current_screen) +
         "/" + std::to_string(ctl_setup_src) + std::to_string(ctl_wp_last) + hdd_model_raw;
    return s;
}

// Runs per id, from the command log: a core-0 op's one DONE entry, a job's
// API_BEGIN (SPEC-v1 5.4). Counted from the last entry already looked at.
static std::map<uint32_t, int> runs;
static uint32_t runs_scanned = 1;
static void count_runs() {
    for (; runs_scanned < log_next; runs_scanned++) {
        const obs_log_entry_t &e = log_ring[runs_scanned % OBS_LOG_ENTRIES];
        if (e.seq != runs_scanned || e.kind != OBS_EV_API) continue;
        bool job = e.value == CTL_OP_UNMOUNT || e.value == CTL_OP_DETACH || e.value == CTL_OP_TASKFILE_SNAPSHOT;
        if (job ? e.outcome != OBS_API_BEGIN : e.outcome != OBS_API_DONE) continue;
        if (!ctl_op_mutating(e.value)) continue;
        runs[e.tf[0] | (e.tf[1] << 8) | (e.tf[2] << 16) | ((uint32_t)e.tf[3] << 24)]++;
    }
}
// The human, between lines: what the ratchet then has to change.
static void human_event() {
    switch (below(4)) {
    case 0:                                         // Auto Detect's set-up, then Mount (confirm-3)
        if (is_mounted) break;
        use_lba_mode = false; cur_cyls = 980; cur_heads = 10; cur_spt = 17; total_lba_sectors = 0;
        strcpy(hdd_model_raw, "FUZZ"); sync_to_config(); ctl_setup_noted(CTL_SRC_TUI_DETECT_NORMAL);
        log_mount(OBS_EV_MOUNT, OBS_MNT_MENU); is_mounted = true; current_screen = SCREEN_MOUNTED; CTL_EPOCH_BUMP();
        break;
    case 1:                                         // Features, Write Protect to Disabled
#if ATABOY_BOARD_JUMPERS
        if (jp1_locked()) break;                    // refused while JP1 is fitted (menus.c)
#endif
        config.drive_write_protected = false; ctl_wp_last = CTL_WP_TUI; CTL_EPOCH_BUMP(); break;
    case 2: config.pipelined_reads = true; CTL_EPOCH_BUMP(); break;
    case 3: if (is_mounted) { menu_unmount(); current_screen = SCREEN_MAIN; } break;
    }
}

#if ATABOY_BOARD_JUMPERS
// Stage 3 (SPEC-v1 11.3, I-4, I-25): JP1 and JP2 timelines between lines, and
// an oracle after every core-0 pass that works from the shunts and the pad
// registers themselves, not from board_jumpers.c's own state.
//  - I-4: while JP1 is fitted or a pad is set up wrong, write protect is on
//    (ide_write_protected, and config.drive_write_protected after the pass).
//  - I-25: a fit event only after an OPEN of 20 equal samples over 10 ms or
//    more, starting at least 200 ms before the event's edge, with no gap over
//    50 ms, no pad set up wrong, no debounced FITTED and no earlier event
//    since; and the edge the first sample of an unbroken FITTED run of at
//    least 200 ms up to the event.
static const unsigned jp_pins[2] = { 30, 31 };
static bool phys_pads_ok() {
    for (unsigned p : jp_pins) {
        uint32_t pad = mock_pads.io[p], ctrl = mock_io.io[p].ctrl;
        if (!(pad & 0x8u) || (pad & 0x4u) || !(pad & 0x40u) || (pad & 0x100u)) return false;
        if ((ctrl & 0x1Fu) != 5u || (ctrl & 0x3F000u)) return false;
        if (mock_gpio_oe & (1u << p)) return false;
    }
    return true;
}
static struct {
    bool have;
    uint32_t last, R;               // the last pass; since when an OPEN may count
    bool run_fit; uint32_t run_start, run_n;
    bool open_ok; uint32_t open_start;
    uint32_t events;
} jo;
static uint64_t jp_passes = 0, jp_events = 0, jp_locks = 0, jp_pad_faults = 0, jp_gaps = 0;
static void jo_reset(uint32_t t) { jo.R = t; jo.run_n = 0; jo.open_ok = false; }
static void jp_oracle() {
    uint32_t t = to_ms_since_boot(get_absolute_time());
    jp_passes++;
    bool ok = phys_pads_ok();
    // I-4
    if ((mock_jumper_fitted[0] || !ok) && (!config.drive_write_protected || !ide_write_protected())) {
        violation("JP1 fitted or a pad wrong, and write protect off (I-4)", "", "");
    }
    if (jo.have && t - jo.last > 50) { jo_reset(t); jp_gaps++; }
    jo.have = true;
    jo.last = t;
    if (!ok) { jo_reset(t); }
    else {
        bool fit = mock_jumper_fitted[1];
        if (jo.run_n && jo.run_fit == fit) jo.run_n++;
        else {
            if (jo.run_n && jo.run_fit && jo.run_n >= 20) jo_reset(t);     // a debounced FITTED ended
            jo.run_fit = fit; jo.run_start = t; jo.run_n = 1;
        }
        if (!fit && !jo.open_ok && jo.run_n >= 20 && t - jo.run_start >= 10 && jo.run_start >= jo.R) {
            jo.open_ok = true; jo.open_start = jo.run_start;
        }
    }
    uint32_t ev = bj_fit_events();
    if (ev != jo.events) {
        jp_events += ev - jo.events;
        uint32_t edge = bj.edge;
        bool a = ev == jo.events + 1 && ok && jo.run_fit && jo.run_start == edge && t - edge >= 200;
        bool b = jo.open_ok && jo.open_start >= jo.R && edge - jo.open_start >= 200;
        if (!a) violation("a fit event without an unbroken FITTED of 200 ms from its edge (I-25)", "", "");
        if (!b) violation("a fit event without an observed OPEN of 200 ms before it (I-25)", "", "");
        jo.events = ev;
        jo.R = t; jo.open_ok = false;                           // the next event needs a new OPEN
    }
}
static void jo_power_up() { memset(&jo, 0, sizeof jo); jo.events = bj_fit_events(); jo.R = to_ms_since_boot(get_absolute_time()); }

// A pad set up wrong, one of the ways test_ctl's table has.
static void pad_fault(unsigned p) {
    switch (below(8)) {
    case 0: mock_pads.io[p] = MOCK_PAD_RESET; mock_io.io[p].ctrl = MOCK_CTRL_RESET; break;
    case 1: mock_pads.io[p] = (mock_pads.io[p] & ~0x8u) | 0x4u; break;
    case 2: mock_pads.io[p] &= ~0x40u; break;
    case 3: mock_pads.io[p] |= 0x100u; break;
    case 4: mock_io.io[p].ctrl = (mock_io.io[p].ctrl & ~0x1Fu) | 6u; break;
    case 5: mock_io.io[p].ctrl |= (1u + below(3)) << (12 + 2 * below(3)); break;
    case 6: mock_gpio_out &= ~(1u << p); mock_gpio_oe |= 1u << p; break;
    case 7: mock_pads.io[p] &= ~0xCu; break;
    }
}
static void pads_repair() {
    for (unsigned p : jp_pins) { mock_gpio_oe &= ~(1u << p); gpio_pull_up(p); gpio_init(p); }
}
static void jumper_event() {
    switch (below(9)) {
    case 0: mock_jumper_fitted[1] = !mock_jumper_fitted[1]; break;
    case 1:                                         // an act, of random lengths either side of 200 ms
        mock_jumper_fitted[1] = false; pump(150 + below(150));
        mock_jumper_fitted[1] = true; pump(150 + below(150));
        break;
    case 2:                                         // bounce
        for (int i = 0, n = 1 + (int)below(6); i < n; i++) { mock_jumper_fitted[1] = !mock_jumper_fitted[1]; pump(1 + below(8)); }
        break;
    case 3: mock_jumper_fitted[0] = !mock_jumper_fitted[0]; if (mock_jumper_fitted[0]) jp_locks++; break;
    case 4: pad_fault(jp_pins[below(2)]); jp_pad_faults++; break;
    case 5: pads_repair(); break;
    case 6:                                         // JP1 on and off between two passes, read live by core 1
        mock_jumper_fitted[0] = true; mock_core_num = 1; (void)jp1_locked(); mock_core_num = 0;
        mock_jumper_fitted[0] = false; jp_locks++;
        break;
    case 7:                                         // core 0 away, JP2 moved meanwhile
        mock_now_ns += (uint64_t)(20 + below(80)) * 1000000ull;
        mock_jumper_fitted[1] = !mock_jumper_fitted[1];
        mock_now_ns += (uint64_t)(below(80)) * 1000000ull;
        break;
    case 8: mock_jumper_fitted[1] = false; pump(200 + below(400)); break;
    }
    // The passes that see it, and a debounced release of JP1 (20 samples
    // over 10 ms), before the next line takes its digest (I-10).
    pump(30);
}
#endif

// One line in, the answers out, the oracle applied.
static std::map<uint32_t, std::vector<std::pair<std::string, std::string>>> answered;   // id -> (line digest, response)
static void one_line(std::string body, bool count_oracle) {
    for (char &ch : body) if (ch == '\n') ch = ' ';     // one line exactly
    std::string line = body + "\n";
    std::string d0 = digest();
    host_send(line);
    lines_sent++;
    // Until the line is all read (256 bytes a pass at most) and answered.
    for (int i = 0; i < 2000 && (!port.rx.empty() || c.resp_busy); i++) pump(1);
    pump(20);
    // A job may be waiting for core 1 (or for its 30 s, on a busy screen):
    // let it end and be answered before the next line.
    for (int i = 0; i < 1600 && ctl_job.state != CTL_JOB_EMPTY; i++) pump(20);
    pump(5);
    std::vector<std::string> got = host_lines();
    std::string stripped = body;
    if (!stripped.empty() && stripped.back() == '\r') stripped.pop_back();
    bool empty = stripped.empty();
    if (count_oracle && got.size() != (empty ? 0u : 1u)) violation("not exactly one response for the line", line, got.empty() ? "" : got[0]);
    for (auto &r : got) {
        responses++;
        J j;
        std::string why;
        if (!parse_response(r, j, &why)) { violation(("response not in the grammar: " + why).c_str(), line, r); continue; }
        std::string e = envelope_problem(j);
        if (!e.empty()) { violation(("envelope: " + e).c_str(), line, r); continue; }
        codes[j["outcome"].is("DONE") ? std::string("DONE") : j["err"]["code"].s]++;
        bool done = j["outcome"].is("DONE");
        if (done && !j.get("replayed")) {
            static ctlj_doc_t pd;
            if (ctlj_parse(stripped.data(), stripped.size(), &pd) == CTLJ_OK) {
                int o = ctlj_find(&pd, 0, "op");
                for (int k = CTL_OP_WP_ON; k <= CTL_OP_DETACH; k++) if (ctlj_str_is(&pd, o, ctl_ops[k].name)) done_ops[ctl_ops[k].name]++;
            }
        }
        done_n += done;
        if (!done && digest() != d0) violation("a refused line changed the state (I-10)", line, r);
        uint8_t dg[32];
        ctl_sha256(stripped.data(), stripped.size(), dg);
        std::string dgs((const char *)dg, 32);
        if (j.get("replayed")) {
            replays++;
            bool found = false, same = false;
            if (j["id"].t == J::INT)
                for (auto &pr : answered[(uint32_t)j["id"].n])
                    if (pr.first == dgs) { found = true; same |= r == pr.second.substr(0, pr.second.size() - 1) + ",\"replayed\":true}"; }
            if (!found) violation("replayed for a line that was not sent before with its id (I-18)", line, r);
            else if (!same) violation("a replay that is not a response the line got before", line, r);
        } else if (j["id"].t == J::INT) {
            answered[(uint32_t)j["id"].n].push_back({ dgs, r });
        }
    }
    count_runs();
    for (auto &kv : runs) if (kv.second > 1) { violation("a mutating id ran twice", line, ""); kv.second = 1; }
    if (c.epoch_missed) { violation("a state change no epoch hook counted", line, ""); c.epoch_missed = 0; }
}

int main(int argc, char **argv) {
    const char *dir = argc > 1 ? argv[1] : "ctl_corpus";
    uint64_t iters = argc > 2 ? strtoull(argv[2], 0, 10) : 20000;
    rng ^= argc > 3 ? strtoull(argv[3], 0, 10) : 1;
    ide_hw_init();
    load_corpus(dir);
    if (corpus.empty()) { printf("COULD NOT RUN: no corpus in %s\n", dir); return 98; }
    power_up();
    runs_scanned = log_next;
#if ATABOY_BOARD_JUMPERS
    after_task = jp_oracle;
    jo_power_up();
#endif
    // 1. The corpus as it is: the count oracle holds for every line.
    for (auto &l : corpus) one_line(l, true);
    // 2. Mutants, with events between them.
    for (uint64_t it = 0; it < iters; it++) {
        uint32_t kind = below(12);
        static std::vector<std::string> history;    // earlier lines, resent byte for byte (replays)
        std::string l = kind < 4 ? grammar_line() : kind < 9 ? byte_mutant(corpus[below((uint32_t)corpus.size())])
                      : kind < 10 ? byte_mutant(grammar_line())
                      : history.empty() ? grammar_line() : history[below((uint32_t)history.size())];
        if (history.size() < 64) history.push_back(l); else history[below(64)] = l;
        if (below(6) == 0) human_event();
#if ATABOY_BOARD_JUMPERS
        if (below(5) == 0) jumper_event();
#endif
        bool events = below(12) == 0;
        if (events) {
            switch (below(5)) {
            case 0: tud_cdc_line_state_cb(1, false, false); pump(2); tud_cdc_line_state_cb(1, true, false); break;
            case 1: ctl_on_usb_mount(); break;
            case 2: core1_runs = !core1_runs; break;
            case 3: mock_now_ns += (uint64_t)(51 + below(5000)) * 1000000ull; break;
            case 4: core1_screen = (uint8_t)below(CTL_TUI_MODAL + 1); break;
            }
        }
        // The count oracle needs core 1 able to take a job before its 30 s.
        core1_runs = true;
        if (below(50) == 0) pump(1000);             // the rate window empties now and then
        one_line(l, true);
        if (it % 5000 == 4999) {
#if ATABOY_BOARD_JUMPERS
            after_task = nullptr;                   // power_up's own passes: a new boot, the oracle starts after it
#endif
            power_up(); answered.clear(); runs.clear(); runs_scanned = log_next;
#if ATABOY_BOARD_JUMPERS
            jo_power_up();
            after_task = jp_oracle;
#endif
        }
    }
    printf("fuzz: seed %llu, %zu corpus lines, %llu mutants, %llu lines, %llu responses (%llu DONE, %llu replayed); codes:",
           (unsigned long long)(argc > 3 ? strtoull(argv[3], 0, 10) : 1), corpus.size(), (unsigned long long)iters,
           (unsigned long long)lines_sent, (unsigned long long)responses, (unsigned long long)done_n,
           (unsigned long long)replays);
    for (auto &kv : codes) printf(" %s=%d", kv.first.c_str(), kv.second);
    printf("; ratchet DONE:");
    for (auto &kv : done_ops) printf(" %s=%d", kv.first.c_str(), kv.second);
#if ATABOY_BOARD_JUMPERS
    printf("\njumpers: %llu passes checked (I-4, I-25), %llu fit events, %llu JP1 fits, %llu pad faults, %llu gaps",
           (unsigned long long)jp_passes, (unsigned long long)jp_events, (unsigned long long)jp_locks,
           (unsigned long long)jp_pad_faults, (unsigned long long)jp_gaps);
#endif
    printf("\n%llu checks, %d failed\n", (unsigned long long)lines_sent, violations);
    return violations > 255 ? 255 : violations;
}
