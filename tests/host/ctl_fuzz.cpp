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
// leaves the state as it was (I-10); replayed:true only for a line
// byte-identical to an earlier one with its id, and then the stored response
// itself (I-18); a mutating stand-in never runs twice for one id.
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
static std::map<std::string, int> codes;
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

// A valid envelope, then one field changed.
static std::string grammar_line() {
    uint32_t id = 1 + below(40);
    std::string op = ops[below(sizeof ops / sizeof ops[0])];
    std::string args = "{}";
    if (op == "log.read" || op == "hashes.read") args = "{\"cursor\":" + std::to_string(below(50)) + ",\"max_entries\":" + std::to_string(1 + below(96)) + "}";
    if (op == "debug.e" && below(2)) args = "{\"include_data\":true}";
    if (op == "wp.on" && below(3) == 0) args = "{\"expect_epoch\":" + std::to_string(below(4)) + "}";
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
    s += std::to_string(is_mounted) + std::to_string(mchs_no_identify) + std::to_string(sim.reg_writes) + "/" +
         std::to_string(sim.cmd_log.size()) + "/" + std::to_string(c.jl_present) + std::to_string(c.epoch);
    return s;
}

static int executed_ids[64];
static int stand_in(ctlw_t *w, const ctlj_doc_t *d, int) {
    int id = (int)d->node[ctlj_find(d, 0, "id")].num;
    if (id >= 0 && id < 64) executed_ids[id]++;
    ctlw_bool(w, "changed", true);
    return CTL_E_NONE;
}

// One line in, the answers out, the oracle applied.
static std::map<uint32_t, std::pair<std::string, std::string>> first_mutating;   // id -> (line digest, response)
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
        done_n += done;
        if (!done && digest() != d0) violation("a refused line changed the state (I-10)", line, r);
        if (j.get("replayed")) {
            replays++;
            uint8_t dg[32];
            ctl_sha256(stripped.data(), stripped.size(), dg);
            std::string dgs((const char *)dg, 32);
            auto it = j["id"].t == J::INT ? first_mutating.find((uint32_t)j["id"].n) : first_mutating.end();
            if (it == first_mutating.end() || it->second.first != dgs)
                violation("replayed for a line that was not sent before with its id (I-18)", line, r);
            else if (r != it->second.second.substr(0, it->second.second.size() - 1) + ",\"replayed\":true}")
                violation("a replay that is not the stored response", line, r);
        } else if (done && j["id"].t == J::INT) {
            static ctlj_doc_t pd;
            if (ctlj_parse(stripped.data(), stripped.size(), &pd) == CTLJ_OK && ctlj_str_is(&pd, ctlj_find(&pd, 0, "op"), "wp.on")) {
                uint8_t dg[32];
                ctl_sha256(stripped.data(), stripped.size(), dg);
                first_mutating[(uint32_t)j["id"].n] = { std::string((const char *)dg, 32), r };
            }
        }
    }
    for (int i = 0; i < 64; i++) if (executed_ids[i] > 1) { violation("a mutating id ran twice", line, ""); executed_ids[i] = 1; }
}

int main(int argc, char **argv) {
    const char *dir = argc > 1 ? argv[1] : "ctl_corpus";
    uint64_t iters = argc > 2 ? strtoull(argv[2], 0, 10) : 20000;
    rng ^= argc > 3 ? strtoull(argv[3], 0, 10) : 1;
    ide_hw_init();
    load_corpus(dir);
    if (corpus.empty()) { printf("COULD NOT RUN: no corpus in %s\n", dir); return 98; }
    power_up();
    ctl_test_op[CTL_OP_WP_ON] = stand_in;           // a mutating op, so replays are exercised (stage 2 is not built)
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
        if (it % 5000 == 4999) { power_up(); ctl_test_op[CTL_OP_WP_ON] = stand_in; first_mutating.clear(); memset(executed_ids, 0, sizeof executed_ids); }
    }
    printf("fuzz: seed %llu, %zu corpus lines, %llu mutants, %llu lines, %llu responses (%llu DONE, %llu replayed); codes:",
           (unsigned long long)(argc > 3 ? strtoull(argv[3], 0, 10) : 1), corpus.size(), (unsigned long long)iters,
           (unsigned long long)lines_sent, (unsigned long long)responses, (unsigned long long)done_n,
           (unsigned long long)replays);
    for (auto &kv : codes) printf(" %s=%d", kv.first.c_str(), kv.second);
    printf("\n%llu checks, %d failed\n", (unsigned long long)lines_sent, violations);
    return violations > 255 ? 255 : violations;
}
