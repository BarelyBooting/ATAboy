// Host tests for the ATAboy Control API, stages 1 and 2 (palimpsest
// api/SPEC-v1.md, section 12.1): the parser and writer, the op table against
// a hand-written copy of SPEC-v1 4.0, the envelope and its order of checks,
// the line layer (limits, CR, DTR, the partial-line timeout), the rate
// window, the non-blocking writer, the replay table (digest-keyed,
// E_ID_REUSED, E_STALE_ID, max_mutating_id), every Tier R op, the
// taskfile.snapshot job on core 1, the hooks in menus.c, and stage 2's
// ratchet (Tier S): wp.on, pipelined.off and identify.forbid on core 0,
// unmount and detach as jobs naming the mount, identify.forbid's guards,
// and the epoch counted where the state changes. Everything runs the real
// ctl*.c, menus.c, ide.c and obs.c (ctl_harness.hpp).
//
//   test_ctl            the tests; exit status is the number of failed checks
//   test_ctl core1      core 1's real loop (core1_entry) takes a job at its idle
//                       point and not in a confirm box; run.sh runs it after
//   test_ctl core1-tui  core 1's real loop with keys: the Features toggles and
//                       their epoch (a change and its reversal inside one
//                       core-0 pass), F: Force after identify.forbid, and an
//                       API unmount from the MOUNTED screen
//   test_ctl serve      the firmware's Control port on stdin/stdout, for
//                       ctl_e2e.py and the real host client (api/ataboy_ctl.py)
//
// Build and run: see run.sh (with ATABOY_CTL=1).
#include "ctl_harness.hpp"
#include <algorithm>
#include <functional>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

static int failures = 0, checks = 0;
static const char *current = "";
#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; \
    printf("FAIL [%s] %s:%d: %s -- ", current, __FILE__, __LINE__, #cond); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)

// The epoch's cross-check (ctl.c epoch_poll) must have found nothing the
// hooks missed in the test that just ended.
static void fresh(const char *name) {
    CHECK(c.epoch_missed == 0, "%u state change(s) with no epoch hook in [%s]", (unsigned)c.epoch_missed, current);
    current = name;
    power_up();
}

// A state change the firmware counts on a path these tests do not run (the
// key handling inside core 1's loop, which test_ctl core1-tui drives): the
// store, then the bump, as the hook there does.
static void hooked() { CTL_EPOCH_BUMP(); }
static void human_wp(bool on) { config.drive_write_protected = on; ctl_wp_last = CTL_WP_TUI; hooked(); }
static void human_pipelined(bool on) { config.pipelined_reads = on; hooked(); }
static void set_mounted(bool m) { is_mounted = m; hooked(); }
// The TUI's set-up (the picker's first row) and mount (confirm-3), through
// menus.c's own functions where there are any.
static void tui_setup_chs(uint16_t cyls, uint8_t heads, uint8_t spt) {
    use_lba_mode = false; cur_cyls = cyls; cur_heads = heads; cur_spt = spt; total_lba_sectors = 0;
    strcpy(hdd_model_raw, "SIM DRIVE");
    sync_to_config();
    ctl_setup_noted(CTL_SRC_TUI_DETECT_NORMAL);
}
static void tui_mount() {
    log_mount(OBS_EV_MOUNT, OBS_MNT_MENU); is_mounted = true; media_changed_waiting = true;
    current_screen = SCREEN_MOUNTED; hooked();
}
// API entries in the command log for one op and id (SPEC-v1 5.4).
static int api_entries(uint8_t op, uint32_t id, int outcome = -1) {
    int n = 0;
    uint32_t lo = log_next > OBS_LOG_ENTRIES ? log_next - OBS_LOG_ENTRIES : 1;
    for (uint32_t q = lo; q < log_next; q++) {
        const obs_log_entry_t &e = log_ring[q % OBS_LOG_ENTRIES];
        uint32_t eid = e.tf[0] | (e.tf[1] << 8) | (e.tf[2] << 16) | ((uint32_t)e.tf[3] << 24);
        if (e.seq == q && e.kind == OBS_EV_API && e.value == op && eid == id && (outcome < 0 || e.outcome == outcome)) n++;
    }
    return n;
}

// Send one request line, run the loop, and return every response read.
static std::vector<std::string> ask(const std::string &line, uint32_t ms = 60) {
    host_send(line);
    pump(ms);
    return host_lines();
}
// Exactly one response, parsed and checked for the envelope; `ok` false if not.
static J ask1(const std::string &line, bool *ok = nullptr, uint32_t ms = 60) {
    std::vector<std::string> r = ask(line, ms);
    J j;
    std::string why;
    bool good = r.size() == 1 && parse_response(r[0], j, &why) && envelope_problem(j).empty();
    if (!good) printf("  (response problem: %zu lines, %s %s)\n", r.size(), why.c_str(),
                      r.size() == 1 ? envelope_problem(j).c_str() : "");
    if (ok) *ok = good;
    return j;
}
static std::string req(uint32_t id, const char *op, const char *args = "{}") {
    char b[1100];
    snprintf(b, sizeof b, "{\"v\":1,\"id\":%u,\"op\":\"%s\",\"args\":%s}\n", id, op, args);
    return b;
}
static bool is_refusal(const J &r, const char *code) {
    return r["outcome"].is("NOT_EXECUTED") && r["err"]["code"].is(code);
}

// ---- 1. the parser (ctl_json.c, SPEC-v1 3.2) -------------------------------
static int parse(const std::string &s) {
    static ctlj_doc_t d;
    return ctlj_parse(s.data(), s.size(), &d);
}
static std::string keys_n(int n) {
    std::string s = "{";
    for (int i = 0; i < n; i++) s += (i ? ",\"k" : "\"k") + std::to_string(i) + "\":1";
    return s + "}";
}
static void test_parser() {
    current = "parser";
    const char *good[] = {
        "{}", " { } ", "{\"a\":0}", "{\"a\":9007199254740991}", "{\"a\":true,\"b\":false}",
        "{\"a\":{\"b\":{\"c\":1}}}", "{\"a\":{}}", "{\"a\":\"\"}", "{ \"a\" : \"x.y:z-_9\" , \"b\" : 1 }",
        "{\"v\":1,\"id\":17,\"op\":\"status\",\"args\":{}}",
        "{\"a\":{\"b\":1},\"c\":{\"b\":1}}",                // the same key in two objects
    };
    for (auto s : good) CHECK(parse(s) == CTLJ_OK, "refused: %s", s);
    CHECK(parse("{\"a\":\"" + std::string(64, 'x') + "\"}") == CTLJ_OK, "64-character string refused");
    CHECK(parse("{\"" + std::string(24, 'k') + "\":1}") == CTLJ_OK, "24-character key refused");
    CHECK(parse(keys_n(32)) == CTLJ_OK, "32 keys refused");
    const char *bad[] = {
        "", " ", "[]", "\"x\"", "1", "{", "}", "{\"a\"}", "{\"a\":}", "{\"a\":1,}", "{,}", "{\"a\" 1}",
        "{\"a\":null}", "{\"a\":[1]}", "{\"a\":1.5}", "{\"a\":1e3}", "{\"a\":-1}", "{\"a\":+1}", "{\"a\":01}",
        "{\"a\":9007199254740992}", "{\"a\":18446744073709551616}", "{\"a\":99999999999999999999}",
        "{\"a\":tru}", "{\"a\":truex}", "{\"a\":True}", "{\"a\":\"x y\"}", "{\"a\":\"x\\\"y\"}", "{\"a\":\"x\\\\\"}",
        "{\"a\":\"\\u0041\"}", "{\"a\":\"x/y\"}", "{a:1}", "{'a':1}", "{\"a\":1}x", "{\"a\":1} {}",
        "{\"a\":{\"b\":{\"c\":{\"d\":1}}}}", "{\"a\":1\t}", "{\"a\":\"unterminated}",
    };
    for (auto s : bad) CHECK(parse(s) == CTLJ_E_PARSE, "accepted or misjudged: %s (%d)", s, parse(s));
    CHECK(parse("{\"a\":\"" + std::string(65, 'x') + "\"}") == CTLJ_E_PARSE, "65-character string accepted");
    CHECK(parse("{\"" + std::string(25, 'k') + "\":1}") == CTLJ_E_PARSE, "25-character key accepted");
    CHECK(parse(keys_n(33)) == CTLJ_E_PARSE, "33 keys accepted");
    // 33 values in 32 keys cannot happen; 33 objects cannot either (depth 3): the
    // key cap holds them. Duplicate keys: E_SCHEMA, at any depth.
    CHECK(parse("{\"a\":1,\"a\":2}") == CTLJ_E_DUP, "duplicate key not reported");
    CHECK(parse("{\"a\":{\"b\":1,\"b\":1}}") == CTLJ_E_DUP, "nested duplicate key not reported");
    // Every byte value in a string: only the token characters pass.
    int passed = 0;
    for (int b = 0; b < 256; b++) {
        std::string s = "{\"a\":\"x";
        s.push_back((char)b);
        s += "\"}";
        bool tok = (b >= 'A' && b <= 'Z') || (b >= 'a' && b <= 'z') || (b >= '0' && b <= '9') ||
                   b == '_' || b == '.' || b == ':' || b == '-';
        int r = parse(s);
        if (tok) passed += r == CTLJ_OK;
        else CHECK(r == CTLJ_E_PARSE || b == '"', "byte %02X accepted in a string", b);
    }
    CHECK(passed == 66, "%d token characters accepted, not 66", passed);
    // Values read back as written.
    static ctlj_doc_t d;
    std::string s = "{\"v\":1,\"id\":2147483647,\"op\":\"log.read\",\"args\":{\"cursor\":16777215,\"flag\":true}}";
    CHECK(ctlj_parse(s.data(), s.size(), &d) == CTLJ_OK, "envelope");
    int a = ctlj_find(&d, 0, "args"), cur = ctlj_find(&d, a, "cursor"), fl = ctlj_find(&d, a, "flag");
    CHECK(a > 0 && d.node[a].type == CTLJ_OBJ && cur > 0 && d.node[cur].num == 16777215 && fl > 0 &&
          d.node[fl].type == CTLJ_BOOL && d.node[fl].num == 1 && ctlj_str_is(&d, ctlj_find(&d, 0, "op"), "log.read") &&
          d.node[ctlj_find(&d, 0, "id")].num == 2147483647u && ctlj_find(&d, a, "v") < 0, "values");
}

// ---- 2. the writer and SHA-256 ----------------------------------------------
static std::string hex(const uint8_t *p, size_t n) {
    static const char h[] = "0123456789abcdef";
    std::string s;
    for (size_t i = 0; i < n; i++) { s.push_back(h[p[i] >> 4]); s.push_back(h[p[i] & 15]); }
    return s;
}
static void test_writer_sha() {
    current = "writer and SHA-256";
    char buf[256];
    ctlw_t w;
    ctlw_init(&w, buf, sizeof buf);
    ctlw_obj(&w, 0);
    ctlw_str(&w, "s", "a\"b\\c\x01\x80");
    ctlw_uint(&w, "n", 9007199254740991ull);
    ctlw_arr(&w, "a"); ctlw_uint(&w, 0, 0); ctlw_bool(&w, 0, true); ctlw_null(&w, 0); ctlw_end_arr(&w);
    const uint8_t hb[] = { 0x00, 0xAB, 0xFF };
    ctlw_hex(&w, "h", hb, 3);
    ctlw_end_obj(&w);
    std::string got(buf, w.len);
    CHECK(!w.overflow && got == "{\"s\":\"a\\\"b\\\\c??\",\"n\":9007199254740991,\"a\":[0,true,null],\"h\":\"00abff\"}",
          "wrote %s", got.c_str());
    // ctlw_raw with nothing to write leaves a value, not a bare key (review A LOW-5):
    // job.last's response when the job's answer was over its 1,024 bytes.
    ctlw_init(&w, buf, sizeof buf);
    ctlw_obj(&w, 0); ctlw_raw(&w, "response", "", 0); ctlw_raw(&w, "r", "{\"a\":1}", 7); ctlw_end_obj(&w);
    CHECK(!w.overflow && std::string(buf, w.len) == "{\"response\":null,\"r\":{\"a\":1}}", "ctlw_raw wrote %s",
          std::string(buf, w.len).c_str());
    ctlw_init(&w, buf, 10);
    ctlw_obj(&w, 0); ctlw_str(&w, "key", "a long value");
    CHECK(w.overflow && w.len <= 10, "overflow not caught (%u bytes)", w.len);
    uint8_t d[32];
    ctl_sha256("", 0, d);
    CHECK(hex(d, 32) == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", "sha256('')");
    ctl_sha256("abc", 3, d);
    CHECK(hex(d, 32) == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", "sha256('abc')");
    const char *m = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    ctl_sha256(m, strlen(m), d);
    CHECK(hex(d, 32) == "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1", "sha256(56 bytes)");
}

// ---- 3. the op table, by hand (SPEC-v1 4.0; I-14) ---------------------------
static void test_op_table() {
    current = "op table (I-14)";
    // Written out from the spec's table, not read from ctl_policy.c.
    struct { const char *name; uint8_t tier; bool job; uint32_t max_ms; bool built; } want[] = {
        { "hello", CTL_TIER_R, false, 5, true }, { "status", CTL_TIER_R, false, 5, true },
        { "identify.cached", CTL_TIER_R, false, 5, true }, { "config.get", CTL_TIER_R, false, 5, true },
        { "taskfile.snapshot", CTL_TIER_R, true, 100, true }, { "log.read", CTL_TIER_R, false, 20, true },
        { "hashes.read", CTL_TIER_R, false, 20, true }, { "debug.e", CTL_TIER_R, false, 5, true },
        { "selftest.status", CTL_TIER_R, false, 5, true }, { "chip.info", CTL_TIER_R, false, 5, true },
        { "job.last", CTL_TIER_R, false, 5, true },
        { "wp.on", CTL_TIER_S, false, 5, true }, { "pipelined.off", CTL_TIER_S, false, 5, true },
        { "identify.forbid", CTL_TIER_S, false, 5, true }, { "unmount", CTL_TIER_S, true, 30000, true },
        { "detach", CTL_TIER_S, true, 30000, true }, { "session.open", CTL_TIER_SESSION, false, 5, true },
        { "session.close", CTL_TIER_SESSION, false, 5, true }, { "setup.ctrlg", CTL_TIER_M, true, 75000, true },
        { "mount", CTL_TIER_M, true, 1000, true },
    };
    CHECK(CTL_OP_COUNT == 20 && sizeof want / sizeof want[0] == 20, "%d ops", CTL_OP_COUNT);
    for (int i = 0; i < CTL_OP_COUNT && i < 20; i++)
        CHECK(!strcmp(ctl_ops[i].name, want[i].name) && ctl_ops[i].tier == want[i].tier && ctl_ops[i].job == want[i].job &&
              ctl_ops[i].max_ms == want[i].max_ms && ctl_ops[i].built == want[i].built,
              "row %d: %s tier %d job %d max %u built %d", i, ctl_ops[i].name, ctl_ops[i].tier, ctl_ops[i].job,
              (unsigned)ctl_ops[i].max_ms, ctl_ops[i].built);
    const char *human[] = { "wp.off", "geometry.manual", "geometry.force", "geometry.large", "config.save",
                            "config.defaults", "debug.i", "debug.s", "debug.r", "fw.update", "auto_mount",
                            "iordy", "intrq", "dev_base", "pipelined.on" };
    CHECK(CTL_HUMAN_ONLY_COUNT == 15, "%d human-only names", CTL_HUMAN_ONLY_COUNT);
    for (auto h : human) {
        bool listed = false, in_table = false;
        for (int i = 0; i < CTL_HUMAN_ONLY_COUNT; i++) listed |= !strcmp(ctl_human_only[i], h);
        for (int i = 0; i < CTL_OP_COUNT; i++) in_table |= !strcmp(ctl_ops[i].name, h);
        CHECK(listed && !in_table, "%s: listed %d, in the op table %d", h, listed, in_table);
    }
    for (auto h : { "setup.detect", "setup.geometry" }) {
        bool listed = false, in_table = false;
        for (int i = 0; i < CTL_NOT_IN_V1_COUNT; i++) listed |= !strcmp(ctl_not_in_v1[i], h);
        for (int i = 0; i < CTL_OP_COUNT; i++) in_table |= !strcmp(ctl_ops[i].name, h);
        CHECK(listed && !in_table && CTL_NOT_IN_V1_COUNT == 2, "%s: listed %d, in the op table %d", h, listed, in_table);
    }
    // An op number outside the table is never offered (X40): every v1 op is
    // built from stage 4 on, so hello's walk over the table no longer meets it.
    for (int op : { -1, (int)CTL_OP_COUNT, (int)CTL_OP_COUNT + 7 }) {
        uint8_t st = 0xFF;
        const char *why = nullptr;
        ctl_op_cap(op, &st, &why);
        CHECK(st == CTL_CAP_ABSENT && why && !strcmp(why, "NOT_BUILT"), "ctl_op_cap(%d): state %u why %s", op, st, why ? why : "-");
    }
    // Mutating: Tier S, the session ops and Tier M; tokens: Tier M and session.close.
    for (int i = 0; i < CTL_OP_COUNT; i++) {
        CHECK(ctl_op_mutating(i) == (i >= CTL_OP_WP_ON), "%s mutating %d", ctl_ops[i].name, ctl_op_mutating(i));
        CHECK(ctl_op_needs_token(i) == (i == CTL_OP_SESSION_CLOSE || i == CTL_OP_SETUP_CTRLG || i == CTL_OP_MOUNT),
              "%s token %d", ctl_ops[i].name, ctl_op_needs_token(i));
    }
}

// ---- 4. hello and the capability listing (SPEC-v1 3.4, 4.2) ------------------
static void test_hello() {
    fresh("hello");
    bool ok;
    J r = ask1(req(1, "hello"), &ok);
    const J &res = r["result"];
    CHECK(ok && r["outcome"].is("DONE") && r["id"].n == 1, "hello not DONE");
    CHECK(res["proto"].t == J::ARR && res["proto"].arr.size() == 1 && res["proto"].arr[0]->n == 1, "proto");
    CHECK(res["serial"].is(board_serial) && r["dev"]["serial"].is(board_serial) && r["dev"]["boot"].is("7c1e2a90d4b3f001"),
          "serial or boot");
    CHECK(res["fw"]["version"].is("0.6f4-palimpsest") && res["fw"]["banner"].is("v0.6f4 (fork+api)"),
          "version %s banner %s", res["fw"]["version"].s.c_str(), res["fw"]["banner"].s.c_str());
    CHECK(res["fw"]["git"].s.size() == 40 && res["fw"]["dirty"].t == J::BOOL, "git or dirty");
    const J &bf = res["fw"]["build_flags"];
#if ATABOY_BOARD_JUMPERS
    CHECK(bf["ctl"].b && bf["sat"].t == J::BOOL && bf["tripwire"].b && bf["board_jumpers"].t == J::BOOL &&
          bf["board_jumpers"].b && bf.obj.size() == 6, "build flags");
#else
    CHECK(bf["ctl"].b && bf["sat"].t == J::BOOL && bf["tripwire"].b && !bf["board_jumpers"].b && bf.obj.size() == 6, "build flags");
#endif
    const J &caps = res["caps"];
    CHECK(caps.obj.size() == CTL_OP_COUNT + CTL_HUMAN_ONLY_COUNT + CTL_NOT_IN_V1_COUNT, "%zu caps", caps.obj.size());
    for (int i = 0; i < CTL_OP_COUNT; i++) {
        const J &cap = caps[ctl_ops[i].name];
        if (ctl_ops[i].tier != CTL_TIER_M)  // Tier R, Tier S (stage 2), the session ops (stage 4)
            CHECK(cap["state"].is("OPERATIONAL") && !cap.get("why"), "%s not OPERATIONAL", ctl_ops[i].name);
        else                                // stage 4: no lease held; without the jumpers never usable
#if ATABOY_BOARD_JUMPERS
            CHECK(cap["state"].is("LOCKED") && cap["why"].is("NO_LEASE"), "%s: %s %s", ctl_ops[i].name,
                  cap["state"].s.c_str(), cap["why"].s.c_str());
#else
            CHECK(cap["state"].is("LOCKED") && cap["why"].is("JUMPERS_UNSUPPORTED"), "%s: %s %s", ctl_ops[i].name,
                  cap["state"].s.c_str(), cap["why"].s.c_str());
#endif
    }
    for (int i = 0; i < CTL_HUMAN_ONLY_COUNT; i++)
        CHECK(caps[ctl_human_only[i]]["state"].is("ABSENT") && caps[ctl_human_only[i]]["why"].is("HUMAN_ONLY"),
              "%s", ctl_human_only[i]);
    CHECK(caps["setup.detect"]["why"].is("NOT_IN_V1") && caps["setup.geometry"]["why"].is("NOT_IN_V1") &&
          caps["setup.geometry"]["state"].is("ABSENT"), "not in v1");
    const J &lim = res["limits"];
    struct { const char *k; uint64_t v; } want[] = {
        { "max_request_line", 1024 }, { "max_response_line", 16384 }, { "max_log_entries_per_read", 128 },
        { "max_hash_entries_per_read", 96 }, { "rate_per_s", 20 }, { "presence_window_s", 120 },
        { "mount_credit_s", 60 }, { "lease_idle_s_max", 600 }, { "replay_slots", 16 }, { "max_mutating_id", 0 } };
    CHECK(lim.obj.size() == 10, "%zu limits", lim.obj.size());
    for (auto &x : want) CHECK(lim[x.k].t == J::INT && lim[x.k].n == x.v, "limits.%s = %llu", x.k, (unsigned long long)lim[x.k].n);
    CHECK(res["board"]["die"].is("RP2350") && res["board"]["stepping"].n == 4 && res["board"]["package"].is("UNKNOWN"), "board");
    // Every op listed ABSENT is refused as unknown, and a human-only one too.
    for (const char *op : { "wp.off", "debug.r", "setup.detect", "setup.geometry", "no.such" })
        CHECK(is_refusal(ask1(req(2, op)), "E_UNKNOWN_OP"), "%s not E_UNKNOWN_OP", op);
}

// ---- 5. the envelope and the order of checks (SPEC-v1 3.3, 3.5) -------------
static void test_envelope() {
    fresh("envelope");
    J r;
    r = ask1("{\"id\":5,\"op\":\"hello\",\"args\":{}}\n");
    CHECK(is_refusal(r, "E_PROTO_VERSION") && r["id"].n == 5, "no v");
    r = ask1("{\"v\":2,\"id\":6,\"op\":\"hello\",\"args\":{}}\n");
    CHECK(is_refusal(r, "E_PROTO_VERSION") && r["id"].n == 6, "v 2");
    r = ask1("{\"v\":\"1\",\"id\":6,\"op\":\"hello\",\"args\":{}}\n");
    CHECK(is_refusal(r, "E_PROTO_VERSION"), "v as a string");
    r = ask1("{\"v\":1,\"id\":7,\"op\":\"hello\",\"args\":{},\"extra\":1}\n");
    CHECK(is_refusal(r, "E_SCHEMA") && r["id"].n == 7, "unknown field");
    for (const char *id : { "0", "2147483648", "\"7\"", "true" }) {
        r = ask1(std::string("{\"v\":1,\"id\":") + id + ",\"op\":\"hello\",\"args\":{}}\n");
        CHECK(is_refusal(r, "E_SCHEMA") && r["id"].t == J::NUL && r["err"]["field"].is("id"), "id %s", id);
    }
    r = ask1("{\"v\":1,\"id\":2147483647,\"op\":\"hello\",\"args\":{}}\n");
    CHECK(r["outcome"].is("DONE") && r["id"].n == 2147483647u, "the largest id");
    r = ask1("{\"v\":1,\"id\":8,\"args\":{}}\n");
    CHECK(is_refusal(r, "E_SCHEMA"), "no op");
    r = ask1("{\"v\":1,\"id\":8,\"op\":1,\"args\":{}}\n");
    CHECK(is_refusal(r, "E_SCHEMA"), "op not a string");
    r = ask1("{\"v\":1,\"id\":9,\"op\":\"hello\"}\n");
    CHECK(is_refusal(r, "E_SCHEMA") && r["err"]["field"].is("args"), "no args");
    r = ask1("{\"v\":1,\"id\":9,\"op\":\"hello\",\"args\":1}\n");
    CHECK(is_refusal(r, "E_SCHEMA"), "args not an object");
    r = ask1("{\"v\":1,\"id\":10,\"op\":\"hello\",\"token\":\"00112233445566778899aabbccddeeff\",\"args\":{}}\n");
    CHECK(is_refusal(r, "E_SCHEMA") && r["err"]["field"].is("token"), "a token where none belongs");
    r = ask1(req(11, "hello", "{\"x\":1}"));
    CHECK(is_refusal(r, "E_SCHEMA"), "unknown argument");
    r = ask1(req(12, "log.read", "{\"cursor\":0}"));
    CHECK(is_refusal(r, "E_SCHEMA") && r["err"]["field"].is("max_entries"), "missing argument");
    r = ask1(req(13, "log.read", "{\"cursor\":\"0\",\"max_entries\":1}"));
    CHECK(is_refusal(r, "E_SCHEMA") && r["err"]["field"].is("cursor"), "argument of the wrong type");
    r = ask1(req(14, "log.read", "{\"cursor\":0,\"max_entries\":129}"));
    CHECK(is_refusal(r, "E_RANGE") && r["err"]["field"].is("max_entries"), "129 log entries");
    r = ask1(req(15, "hashes.read", "{\"cursor\":0,\"max_entries\":97}"));
    CHECK(is_refusal(r, "E_RANGE"), "97 hash entries");
    r = ask1(req(16, "log.read", "{\"cursor\":16777216,\"max_entries\":1}"));
    CHECK(is_refusal(r, "E_RANGE"), "cursor over 24 bits");
    r = ask1(req(17, "debug.e", "{\"include_data\":1}"));
    CHECK(is_refusal(r, "E_SCHEMA"), "include_data not a boolean");
    r = ask1("{\"v\":1,\"id\":18,\"id\":19,\"op\":\"hello\",\"args\":{}}\n");
    CHECK(is_refusal(r, "E_SCHEMA"), "duplicate id key");
    // A repeated argument: every name in it is known, so only the parser's
    // duplicate check can refuse it.
    r = ask1(req(26, "log.read", "{\"cursor\":0,\"cursor\":1,\"max_entries\":1}"));
    CHECK(is_refusal(r, "E_SCHEMA") && r["id"].n == 26, "duplicate argument accepted");
    r = ask1("{\"v\":2,\"v\":1,\"id\":18,\"op\":\"hello\",\"args\":{}}\n");
    CHECK(is_refusal(r, "E_PROTO_VERSION"), "duplicate v, the first 2: v is checked first");
    r = ask1(req(20, "hello", "{\"a\":{\"b\":1}}"));
    CHECK(is_refusal(r, "E_SCHEMA"), "nested unknown argument");
    // The order: v before the envelope, the envelope before the op, the op before its arguments.
    r = ask1("{\"v\":2,\"id\":21,\"op\":\"nope\",\"args\":1,\"x\":1}\n");
    CHECK(is_refusal(r, "E_PROTO_VERSION"), "v first");
    r = ask1("{\"v\":1,\"id\":22,\"op\":\"nope\",\"args\":1}\n");
    CHECK(is_refusal(r, "E_SCHEMA"), "the envelope before the op");
    r = ask1("{\"v\":1,\"id\":23,\"op\":\"nope\",\"args\":{\"x\":1}}\n");
    CHECK(is_refusal(r, "E_UNKNOWN_OP"), "the op before its arguments");
    // Every refusal says NOT_EXECUTED, retry SAFE, and nothing changed.
    CHECK(r["retry"].is("SAFE") && !r["ok"].b, "retry");
    // dev.seq counts responses.
    J a = ask1(req(24, "chip.info")), b = ask1(req(25, "chip.info"));
    CHECK(b["dev"]["seq"].n == a["dev"]["seq"].n + 1, "seq %llu then %llu", (unsigned long long)a["dev"]["seq"].n,
          (unsigned long long)b["dev"]["seq"].n);
}

// ---- 6. the line layer (SPEC-v1 3.1) -----------------------------------------
static std::string padded_hello(uint32_t id, size_t total) {       // total: bytes with the \n
    std::string s = "{\"v\":1,\"id\":" + std::to_string(id) + ",\"op\":\"hello\",\"args\":{}";
    s += std::string(total - s.size() - 2, ' ');
    return s + "}\n";
}
static void test_lines() {
    fresh("lines");
    std::vector<std::string> r;
    // Exactly one answer a line; none for an empty one.
    r = ask("\n\r\n\n");
    CHECK(r.empty(), "%zu answers to empty lines", r.size());
    r = ask(req(1, "chip.info") + req(2, "chip.info") + "\n" + req(3, "chip.info"), 60);
    CHECK(r.size() == 3, "%zu answers to three lines", r.size());
    // 1,024 bytes with the \n: accepted; one more: E_TOO_LARGE once, id null.
    std::string l1024 = padded_hello(4, 1024);
    CHECK(l1024.size() == 1024, "fixture");
    J j = ask1(l1024);
    CHECK(j["outcome"].is("DONE") && j["id"].n == 4, "a 1,024-byte line refused");
    std::string l1025 = padded_hello(5, 1025);
    r = ask(l1025, 40);
    J k;
    CHECK(r.size() == 1 && parse_response(r[0], k) && is_refusal(k, "E_TOO_LARGE") && k["id"].t == J::NUL,
          "a 1,025-byte line: %zu answers", r.size());
    // 1,023 bytes and a \r\n: the \r counts, and is taken off (1,024 with it).
    std::string crlf = padded_hello(6, 1024);
    crlf.insert(crlf.size() - 1, "\r");
    r = ask(crlf, 40);
    CHECK(r.size() == 1 && parse_response(r[0], k) && is_refusal(k, "E_TOO_LARGE"), "1,025 bytes with \\r\\n accepted");
    crlf = padded_hello(7, 1023);
    crlf.insert(crlf.size() - 1, "\r");
    j = ask1(crlf);
    CHECK(j["outcome"].is("DONE") && j["id"].n == 7, "\\r\\n ending refused");
    // A very long line is answered once, and the next line is fine.
    r = ask(std::string(5000, 'x') + "\n" + req(8, "chip.info"), 200);
    CHECK(r.size() == 2 && parse_response(r[0], k) && is_refusal(k, "E_TOO_LARGE") && parse_response(r[1], k) &&
          k["outcome"].is("DONE") && k["id"].n == 8, "long line then a request: %zu answers", r.size());
    // Bytes outside 0x20-0x7E, and a \r anywhere but last: E_PARSE, id null.
    for (int b : { 0x00, 0x09, 0x0D, 0x1B, 0x7F, 0x80, 0xC3, 0xFF }) {
        std::string s = req(9, "chip.info");
        s.insert(10, 1, (char)b);
        r = ask(s);
        // Refused by the line layer itself, before the parser (which would
        // refuse it too): each layer is checked on its own.
        CHECK(r.size() == 1 && parse_response(r[0], k) && is_refusal(k, "E_PARSE") && k["id"].t == J::NUL &&
              k["err"]["msg"].is("a byte outside 0x20-0x7E"), "byte %02X: %zu answers", b, r.size());
    }
    // No \n: nothing runs, and after 2 s of quiet it is E_PARSE.
    uint32_t seq0 = c.seq;
    r = ask(req(10, "chip.info").substr(0, 30), 1900);
    CHECK(r.empty() && c.seq == seq0, "a partial line answered before 2 s");
    r = ask("", 200);
    CHECK(r.size() == 1 && parse_response(r[0], k) && is_refusal(k, "E_PARSE"), "partial line not ended at 2 s");
    r = ask(req(11, "chip.info"));
    CHECK(r.size() == 1 && parse_response(r[0], k) && k["id"].n == 11, "the line after a timed-out one");
    // A gap of over 50 ms between passes (core 0 in a long USB callback) is
    // not the host's silence: the 2 s start again.
    host_send("{\"v\":1,");
    pump(1500);
    mock_now_ns += 5000000000ull;                   // a 5 s callback
    pump(1);
    pump(1000);
    r = host_lines();
    CHECK(r.empty(), "a partial line ended by a long callback (%zu answers)", r.size());
    r = ask("\"id\":12,\"op\":\"chip.info\",\"args\":{}}\n");
    CHECK(r.size() == 1 && parse_response(r[0], k) && k["outcome"].is("DONE") && k["id"].n == 12,
          "the line resumed after the callback");
    // DTR low: nothing read, nothing answered; a rise starts a clean line.
    port.dtr = false;
    tud_cdc_line_state_cb(1, false, false);
    r = ask(req(13, "chip.info"), 50);
    CHECK(r.empty() && port.rx.empty(), "answered with DTR low (%zu)", r.size());
    host_send("{\"v\":1,\"id\":14,");
    pump(5);
    port.dtr = true;
    tud_cdc_line_state_cb(1, true, false);
    r = ask(req(15, "chip.info"));
    CHECK(r.size() == 1 && parse_response(r[0], k) && k["id"].n == 15, "after DTR rose: %zu answers", r.size());
    // DTR dropped in the middle of a line: the part before is gone.
    host_send("{\"v\":1,\"id\":16,\"op\":\"chip.info\"");
    pump(2);
    tud_cdc_line_state_cb(1, false, false);
    pump(2);
    tud_cdc_line_state_cb(1, true, false);
    r = ask(",\"args\":{}}\n" + req(17, "chip.info"), 40);
    CHECK(r.size() == 2 && parse_response(r[0], k) && is_refusal(k, "E_PARSE") && parse_response(r[1], k) && k["id"].n == 17,
          "a line cut by a DTR drop: %zu answers", r.size());
    // DTR dropped on the TUI's port (interface 0) is not this port's: the
    // line goes on.
    host_send("{\"v\":1,\"id\":19,\"op\":\"chip.info\"");
    pump(2);
    tud_cdc_line_state_cb(0, false, false);
    r = ask(",\"args\":{}}\n", 20);
    CHECK(r.size() == 1 && parse_response(r[0], k) && k["outcome"].is("DONE") && k["id"].n == 19,
          "a DTR drop on interface 0 cut the Control port's line");
    // USB reset (SET_CONFIGURATION): the same.
    host_send("{\"v\":1,\"id\":18,\"op\":\"chip.info\"");
    pump(2);
    ctl_on_usb_mount();
    r = ask(",\"args\":{}}\n", 20);
    CHECK(r.size() == 1 && parse_response(r[0], k) && is_refusal(k, "E_PARSE"), "a line cut by a USB reset");
    // ctl_task never takes modelled time (no sleeps, no busy waits).
    CHECK(task_max_ns == 0, "ctl_task took %llu ns of modelled time", (unsigned long long)task_max_ns);
}

// ---- 7. the rate window (SPEC-v1 3.5) ----------------------------------------
static void test_rate() {
    fresh("rate");
    ctl_rate_t rt;
    ctl_rate_init(&rt);
    int admitted = 0;
    for (int i = 0; i < 20; i++) admitted += ctl_rate_admit(&rt, 1000 + i);
    CHECK(admitted == 20, "%d of the first 20", admitted);
    CHECK(!ctl_rate_admit(&rt, 1999), "a 21st within 1 s of the first");
    CHECK(ctl_rate_admit(&rt, 2000), "a 21st 1,000 ms after the first");
    CHECK(!ctl_rate_admit(&rt, 2000), "a 22nd at the same time (the second was at 1001)");
    CHECK(ctl_rate_admit(&rt, 2001), "a 22nd at 2001");
    // Through the port: 20 answered DONE in a burst, the 21st E_RATE; later fine.
    std::string burst;
    for (int i = 1; i <= 21; i++) burst += req(i, "chip.info");
    std::vector<std::string> r = ask(burst, 200);
    int done = 0, rate = 0;
    long long rate_id = -1;
    for (auto &l : r) {
        J k;
        if (parse_response(l, k)) { done += k["outcome"].is("DONE"); if (is_refusal(k, "E_RATE")) { rate++; rate_id = k["id"].t == J::INT ? (long long)k["id"].n : 0; } }
    }
    CHECK(r.size() == 21 && done == 20 && rate == 1, "%zu answers, %d DONE, %d E_RATE", r.size(), done, rate);
    // Review delta-B R7: an over-limit well-formed request is answered under ITS
    // OWN id (21), never id null: a client reads a null id as a protocol
    // violation (DESYNC), and E_RATE is a retryable refusal.
    CHECK(rate_id == 21, "the E_RATE answer to request 21 carries id %lld (0 = null)", rate_id);
    pump(1000);
    J k = ask1(req(30, "chip.info"));
    CHECK(k["outcome"].is("DONE"), "after the window");
    // Owner ruling 2026-09-29 on review A LOW-8, option (b): a malformed line
    // (E_PARSE: a byte outside 0x20-0x7E, or outside the grammar) COUNTS toward
    // the 20 a second and is still ANSWERED, once. Inside the limit it is E_PARSE
    // as before; over it, E_RATE with id null (the answer a well-formed
    // over-limit request with no readable id gets), never silence.
    fresh("rate: malformed lines count and are answered");
    std::string noise;
    for (int i = 0; i < 30; i++) noise += (i % 2) ? std::string("AT\r\n") : std::string("{\"v\":1,\"id\":1,\"op\":\"status\"\x01}\n");
    r = ask(noise, 200);
    int pe = 0, re = 0, nulls = 0, first_rate = -1, safe = 0;
    for (size_t i = 0; i < r.size(); i++) {
        J q;
        if (!parse_response(r[i], q)) continue;
        safe += q["retry"].is("SAFE");                  // review delta-B R2
        pe += is_refusal(q, "E_PARSE");
        if (is_refusal(q, "E_RATE")) { re++; if (first_rate < 0) first_rate = (int)i; }
        nulls += q["id"].t == J::NUL;
    }
    CHECK(r.size() == 30 && pe == 20 && re == 10 && first_rate == 20 && nulls == 30,
          "%zu answers to 30 malformed lines: %d E_PARSE, %d E_RATE (first at %d), %d with id null", r.size(), pe, re, first_rate, nulls);
    // Every one of them, the over-limit E_RATE included, says retry SAFE (the
    // line changed nothing; RESPONSE "A LOW-8"), not AFTER_STATUS or UNSAFE.
    CHECK(safe == 30, "%d of 30 answers to malformed lines say retry SAFE", safe);
    k = ask1(req(1, "chip.info"));
    CHECK(is_refusal(k, "E_RATE"), "a well-formed request in the same second as 20 malformed lines: %s", k["outcome"].s.c_str());
    pump(1000);
    k = ask1(req(2, "chip.info"));
    CHECK(k["outcome"].is("DONE"), "after the window: %s", k["outcome"].s.c_str());
    // The over-limit answer is a refusal like the others: not a parse error, and
    // it changes nothing but the counters.
    k = ask1(req(3, "status"));
    CHECK(k["outcome"].is("DONE") && k["result"]["counters"]["parse_errors"].n == 20, "parse_errors %lld, not 20", (long long)k["result"]["counters"]["parse_errors"].n);
    // A malformed line after the window has emptied is E_PARSE again.
    pump(1000);
    r = ask(std::string("AT\r\n"));
    CHECK(r.size() == 1 && parse_response(r[0], k) && is_refusal(k, "E_PARSE"), "a malformed line in an empty window");
    // Malformed lines and requests share one window: 15 + 5 admitted, the 21st refused.
    fresh("rate: one window for both");
    std::string mix;
    for (int i = 0; i < 15; i++) mix += "AT\r\n";
    for (int i = 1; i <= 6; i++) mix += req(i, "chip.info");
    r = ask(mix, 200);
    done = 0; rate = 0; pe = 0;
    rate_id = -1;
    for (auto &l : r) {
        J q;
        if (parse_response(l, q)) {
            done += q["outcome"].is("DONE"); pe += is_refusal(q, "E_PARSE");
            if (is_refusal(q, "E_RATE")) { rate++; rate_id = q["id"].t == J::INT ? (long long)q["id"].n : 0; }
        }
    }
    CHECK(r.size() == 21 && pe == 15 && done == 5 && rate == 1, "%zu answers: %d E_PARSE, %d DONE, %d E_RATE", r.size(), pe, done, rate);
    CHECK(rate_id == 6, "the over-limit request (id 6) is answered E_RATE under id %lld (0 = null)", rate_id);
    // Empty lines are not requests: no answer and no count.
    fresh("rate: empty lines do not count");
    std::string blanks;
    for (int i = 0; i < 40; i++) blanks += (i % 2) ? "\r\n" : "\n";      // review delta-B R5: a bare CR line is empty too
    r = ask(blanks, 200);
    k = ask1(req(1, "chip.info"));
    CHECK(r.empty() && k["outcome"].is("DONE"), "%zu answers to empty lines, then %s", r.size(), k["outcome"].s.c_str());
}

// ---- 8. the writer never blocks (SPEC-v1 3.1, I-17) --------------------------
static void test_writer_never_blocks() {
    fresh("writer never blocks");
    port.reading = false;                           // a host that never reads
    host_send(req(1, "hello"));
    pump(10);
    CHECK(port.fifo.size() == port.tx_cap && c.resp_busy, "fifo %zu", port.fifo.size());
    pump(5100);                                     // past 5 s: abandoned
    CHECK(!c.resp_busy && c.tx_abandoned == 1, "busy %d, abandoned %u", c.resp_busy, (unsigned)c.tx_abandoned);
    port.fifo.clear();
    port.reading = true;
    J k = ask1(req(2, "status"));
    CHECK(k["outcome"].is("DONE") && k["result"]["counters"]["tx_abandoned"].n == 1, "tx_abandoned not counted");
    CHECK(task_max_ns == 0, "ctl_task took modelled time");
    // A response trickles out 64 bytes a pass and arrives whole.
    fresh("writer: a slow host");
    port.tx_cap = 64;
    J h = ask1(req(3, "hello"), nullptr, 400);
    CHECK(h["outcome"].is("DONE") && h["result"]["caps"].obj.size() == 37, "hello through a 64-byte FIFO");
}

// ---- 9. the replay table (SPEC-v1 3.5, R-API-1 finding 4) ---------------------
static void test_replay_table() {
    current = "replay table (pure)";
    ctl_replay_t t;
    ctl_replay_init(&t);
    uint8_t da[32], db[32];
    memset(da, 0xAA, 32); memset(db, 0xBB, 32);
    int slot;
    CHECK(ctl_replay_check(&t, 5, da, &slot) == CTL_RPV_NEW, "new");
    int s5 = ctl_replay_begin(&t, 5, da);
    CHECK(s5 >= 0 && t.max_id == 5, "begin");
    CHECK(ctl_replay_check(&t, 5, da, &slot) == CTL_RPV_INFLIGHT && slot == s5, "pending: in flight");
    CHECK(ctl_replay_check(&t, 5, db, &slot) == CTL_RPV_REUSED, "pending, other line: reused");
    ctl_replay_finish(&t, s5, "{\"x\":1}", 7);
    CHECK(ctl_replay_check(&t, 5, da, &slot) == CTL_RPV_REPLAY && slot == s5, "ended: replay");
    CHECK(ctl_replay_check(&t, 5, db, &slot) == CTL_RPV_REUSED, "ended, other line: reused");
    CHECK(ctl_replay_check(&t, 4, da, &slot) == CTL_RPV_STALE, "below max, not held: stale");
    CHECK(ctl_replay_check(&t, 6, da, &slot) == CTL_RPV_NEW, "above max: new");
    // 16 slots; the oldest ended one goes first; a pending one never.
    ctl_replay_init(&t);
    int first = ctl_replay_begin(&t, 1, da);            // stays pending
    for (uint32_t id = 2; id <= 16; id++) ctl_replay_finish(&t, ctl_replay_begin(&t, id, da), "{}", 2);
    CHECK(ctl_replay_begin(&t, 17, da) >= 0, "17th");
    CHECK(ctl_replay_check(&t, 1, da, &slot) == CTL_RPV_INFLIGHT && slot == first, "the pending entry was evicted");
    CHECK(ctl_replay_check(&t, 2, da, &slot) == CTL_RPV_STALE, "id 2 (the oldest ended) not evicted");
    CHECK(ctl_replay_check(&t, 3, da, &slot) == CTL_RPV_REPLAY, "id 3 evicted too early");
    ctl_replay_init(&t);
    for (uint32_t id = 1; id <= 16; id++) ctl_replay_begin(&t, id, da);
    CHECK(ctl_replay_begin(&t, 17, da) == -1, "a slot taken from a pending entry");
    // A response that does not fit is never replayed half.
    ctl_replay_init(&t);
    s5 = ctl_replay_begin(&t, 5, da);
    static char big[CTL_REPLAY_BYTES + 1];
    memset(big, 'x', sizeof big);
    ctl_replay_finish(&t, s5, big, sizeof big);
    CHECK(ctl_replay_check(&t, 5, da, &slot) == CTL_RPV_LOST, "an oversize response replayed");
}

// The replay path, through the real wp.on (stage 1 drove it through a
// stand-in): each run writes one API entry for its id, so the entries count
// the executions.
static int executed_ids(std::initializer_list<uint32_t> ids) {
    int n = 0;
    for (uint32_t id : ids) n += api_entries(CTL_OP_WP_ON, id);
    return n;
}
static void test_replay_dispatch() {
    fresh("replay through the dispatcher");
    int executed = 0;
    std::string l7 = req(7, "wp.on");
    J a = ask1(l7);
    executed = executed_ids({ 7 });
    CHECK(a["outcome"].is("DONE") && executed == 1 && !a.get("replayed"), "first send");
    std::vector<std::string> r = ask(l7);
    executed = executed_ids({ 7 });
    J b;
    CHECK(r.size() == 1 && parse_response(r[0], b) && executed == 1 && b["replayed"].t == J::BOOL && b["replayed"].b,
          "a byte-identical resend ran again, or was not marked (executed %d)", executed);
    // The stored response as sent, dev block and seq included, plus the mark.
    std::string first_line;
    {
        uint8_t dg[32]; ctl_sha256(l7.data(), l7.size() - 1, dg);
        int slot; ctl_replay_check(&c.replay, 7, dg, &slot);
        first_line.assign(c.replay.slot[slot].resp, c.replay.slot[slot].len);
    }
    CHECK(r[0] == first_line.substr(0, first_line.size() - 1) + ",\"replayed\":true}", "replay is not the stored response");
    CHECK(b["dev"]["seq"].n == a["dev"]["seq"].n, "a replay took a new seq");
    // A \r\n resend is the same line (the digest is over the line without \r\n).
    std::string l7crlf = l7; l7crlf.insert(l7crlf.size() - 1, "\r");
    J b2 = ask1(l7crlf);
    executed = executed_ids({ 7 });
    CHECK(b2["replayed"].b && executed == 1, "\\r\\n resend not replayed");
    // The same id with another line: E_ID_REUSED, UNSAFE, not run, and never the other's response.
    J c1 = ask1(req(7, "wp.on", "{\"expect_epoch\":0}"));
    executed = executed_ids({ 7 });
    CHECK(is_refusal(c1, "E_ID_REUSED") && c1["retry"].is("UNSAFE") && executed == 1 && !c1.get("replayed"),
          "same id, other line: %s", c1["err"]["code"].s.c_str());
    // A lower id not in the table: E_STALE_ID; a higher one runs.
    J d = ask1(req(6, "wp.on"));
    executed = executed_ids({ 6, 7 });
    // retry stays SAFE, the default SPEC-v1 gives a refusal (owner ruling 2026-09-29, review A LOW-1).
    CHECK(is_refusal(d, "E_STALE_ID") && executed == 1 && d["err"]["msg"].s.find("may have run") != std::string::npos &&
          d["retry"].is("SAFE"), "stale id: %s, retry %s", d["err"]["msg"].s.c_str(), d["retry"].s.c_str());
    J e = ask1(req(8, "wp.on"));
    executed = executed_ids({ 6, 7, 8 });
    CHECK(e["outcome"].is("DONE") && executed == 2, "a higher id");
    J h = ask1(req(3, "hello"));
    CHECK(h["result"]["limits"]["max_mutating_id"].n == 8, "hello max_mutating_id %llu", (unsigned long long)h["result"]["limits"]["max_mutating_id"].n);
    J s = ask1(req(4, "status"));
    CHECK(s["result"]["ids"]["max_mutating"].n == 8, "status ids.max_mutating");
    // A read-only request reusing a mutating id is simply answered again.
    J h2 = ask1(req(8, "hello"));
    CHECK(h2["outcome"].is("DONE") && !h2.get("replayed"), "a read with a used id");
    // The replay check comes after the envelope: a resend with a schema error is refused as such.
    J f = ask1("{\"v\":1,\"id\":7,\"op\":\"wp.on\",\"args\":{},\"x\":1}\n");
    executed = executed_ids({ 6, 7, 8 });
    CHECK(is_refusal(f, "E_SCHEMA") && executed == 2, "schema before replay");
    // Refused requests of a mutating op keep their answer too (not run twice).
    J g = ask1(req(9, "wp.on", "{\"expect_epoch\":\"x\"}"));
    CHECK(is_refusal(g, "E_SCHEMA"), "bad arg");
    // Past 16 entries the oldest ended one is forgotten: its resend is E_STALE_ID.
    for (uint32_t id = 20; id < 40; id++) { ask1(req(id, "wp.on")); pump(60); }
    J old = ask1(l7);
    CHECK(is_refusal(old, "E_STALE_ID") && old["err"]["msg"].s.find("may have run") != std::string::npos && old["retry"].is("SAFE"),
          "an evicted id: %s, retry %s", old["err"]["code"].s.c_str(), old["retry"].s.c_str());
}

// ---- 10. the Tier R ops -----------------------------------------------------
static bool all_keys(const J &o, std::initializer_list<const char *> ks) {
    if (o.t != J::OBJ || o.obj.size() != ks.size()) return false;
    for (auto k : ks) if (!o.get(k)) return false;
    return true;
}
static void test_status() {
    fresh("status");
    J s = ask1(req(1, "status"));
    const J &r = s["result"];
    CHECK(s["outcome"].is("DONE") &&
          all_keys(r, { "mounted", "mount_seq", "mount_log_seq", "geometry", "setup", "device", "wp", "rules", "features",
                        "bus", "tui", "jumpers", "presence", "lease", "job", "ids", "log", "hashes", "counters", "epoch", "up_ms" }),
          "status fields (%zu)", r.obj.size());
    CHECK(all_keys(r["geometry"], { "mode", "cyls", "heads", "spt", "lba_sectors", "total_sectors" }) &&
          all_keys(r["wp"], { "effective", "config", "forced_by", "last_change" }) &&
          all_keys(r["rules"], { "no_identify_ctrlg", "identify_forbidden", "identify_barred", "manual_chs_active",
                                 "native_active", "native_declared", "idp_sent_since_boot" }) &&
          all_keys(r["presence"], { "waiting", "op", "id", "request_sha256", "ms_left" }) &&
          all_keys(r["job"], { "running", "id", "queued" }) &&
          all_keys(r["counters"], { "unwitnessed_blocks", "self_resets", "tx_abandoned", "parse_errors" }), "sub-objects");
    CHECK(!r["mounted"].b && r["mount_seq"].n == 0 && r["mount_log_seq"].t == J::NUL && r["device"].is("MASTER") &&
          r["wp"]["effective"].b && r["wp"]["last_change"].is("BOOT") && r["wp"]["forced_by"].arr.empty() &&
          r["tui"]["state"].is("MAIN") && !r["presence"]["waiting"].b &&
#if ATABOY_BOARD_JUMPERS
          r["jumpers"]["jp1"].is("OPEN") && r["jumpers"]["jp2"].is("OPEN") &&
#else
          r["jumpers"]["jp1"].is("NOT_BUILT") && r["jumpers"]["jp2"].is("NOT_BUILT") &&
#endif
          r["presence"]["op"].is("NONE") && !r["lease"]["held"].b && r["job"]["running"].is("NONE") &&
          r["job"]["id"].t == J::NUL && r["setup"]["source"].is("NONE") && r["setup"]["at_ms"].n == 0 &&
          r["hashes"]["engine"].is("SOFTWARE") && !r["rules"]["identify_forbidden"].b && !r["rules"]["identify_barred"].b &&
          r["log"]["next_seq"].n >= 2, "values at power-up");
    uint64_t e0 = r["epoch"].n;
    // The human turns write protect off in the TUI: epoch moves, last_change TUI.
    human_wp(false);
    pump(2);
    J s2 = ask1(req(2, "status"));
    CHECK(s2["result"]["epoch"].n == e0 + 1 && s2["result"]["wp"]["last_change"].is("TUI") && !s2["result"]["wp"]["effective"].b &&
          s2["dev"]["epoch"].n == e0 + 1, "epoch %llu after a WP change", (unsigned long long)s2["result"]["epoch"].n);
    // Nothing changed: the epoch stays.
    J s3 = ask1(req(3, "status"));
    CHECK(s3["result"]["epoch"].n == e0 + 1, "epoch moved with nothing changed");
    // Mounted through the menus' own path: mount_seq and the marker's log seq.
    tui_setup_chs(980, 10, 17);
    tui_mount();
    J m = ask1(req(4, "status"));
    CHECK(m["result"]["setup"]["source"].is("TUI_DETECT_NORMAL") && m["result"]["setup"]["at_ms"].n > 0, "setup.source %s",
          m["result"]["setup"]["source"].s.c_str());
    CHECK(m["result"]["mounted"].b && m["result"]["mount_seq"].n == 1 && m["result"]["mount_log_seq"].n == obs_last_marker_seq() &&
          m["result"]["geometry"]["total_sectors"].n == 980u * 10 * 17 && m["result"]["epoch"].n > e0 + 1, "after a mount");
    log_mount(OBS_EV_UNMOUNT, OBS_UNM_MENU); set_mounted(false);
    log_mount(OBS_EV_MOUNT, OBS_MNT_MENU); set_mounted(true);
    m = ask1(req(5, "status"));
    CHECK(m["result"]["mount_seq"].n == 2, "mount_seq %llu after a second mount", (unsigned long long)m["result"]["mount_seq"].n);
    // A Ctrl+G since power-up forces write protect (review C, C-4).
    set_mounted(false);
    mchs_no_identify = true; hooked();
    m = ask1(req(6, "status"));
    CHECK(m["result"]["wp"]["effective"].b && m["result"]["wp"]["forced_by"].arr.size() == 1 &&
          m["result"]["wp"]["forced_by"].arr[0]->is("CTRLG") && m["result"]["rules"]["no_identify_ctrlg"].b &&
          m["result"]["rules"]["identify_barred"].b, "Ctrl+G forcing WP not shown");
}

static void test_config_get() {
    fresh("config.get");
    J g = ask1(req(1, "config.get"));
    const J &r = g["result"];
    CHECK(g["outcome"].is("DONE") && all_keys(r, { "ram", "flash", "dirty", "loaded_from" }) && r["ram"].obj.size() == 13 &&
          r["flash"].obj.size() == 14 && r["flash"]["valid"].b && !r["dirty"].b && r["loaded_from"].is("FLASH"),
          "a clean state: valid %d dirty %d", r["flash"]["valid"].b, r["dirty"].b);
    human_pipelined(true);
    g = ask1(req(2, "config.get"));
    CHECK(g["result"]["dirty"].b && g["result"]["ram"]["pipelined_reads"].b && !g["result"]["flash"]["pipelined_reads"].b, "RAM changed");
    flash_sector.crc ^= 1;
    loaded_from_flash_stub = false;
    g = ask1(req(3, "config.get"));
    CHECK(!g["result"]["flash"]["valid"].b && g["result"]["dirty"].b && g["result"]["loaded_from"].is("DEFAULTS"), "a bad sector");
    CHECK(saves == 0, "config.get saved the settings");
}

static void test_pages() {
    fresh("log.read and hashes.read");
    for (int i = 0; i < 40; i++) obs_marker(OBS_EV_MOUNT, 9, 0, 0, 0, (uint16_t)i, 0xA0);
    for (uint32_t cur : { 0u, 1u, 7u, 30u, 16777215u }) {
        for (uint32_t max : { 1u, 5u, 128u }) {
            char a[80];
            snprintf(a, sizeof a, "{\"cursor\":%u,\"max_entries\":%u}", cur, max);
            J r = ask1(req(1, "log.read", a));
            // The READ BUFFER 4Ch page for the same cursor and room, as usb.c builds it.
            static uint8_t want[OBS_PAGE_HEADER + 128 * 32];
            uint32_t n = obs_log_page(cur, want, OBS_PAGE_HEADER + max * 32);
            CHECK(r["outcome"].is("DONE") && r["result"]["page_hex"].s == hex(want, n) &&
                  r["result"]["entries"].n == (uint64_t)(want[12] | (want[13] << 8)) && r["result"]["entries"].n <= max &&
                  r["result"]["next_seq"].n == (uint64_t)le32(want + 16) && r["result"]["first_seq"].n == (uint64_t)le32(want + 8),
                  "log.read cursor %u max %u", cur, max);
        }
    }
    J r = ask1(req(2, "log.read", "{\"cursor\":1,\"max_entries\":2}"));
    CHECK(r["result"]["lost"].t == J::BOOL && !r.get("engine") && !r["result"].get("engine"), "log.read has no engine");
    uint8_t blk[512];
    for (int i = 0; i < 512; i++) blk[i] = (uint8_t)i;
    for (uint32_t i = 0; i < 100; i++) obs_hash_sector(1, i, 0, 0, blk);
    obs_hash_flush();
    r = ask1(req(3, "hashes.read", "{\"cursor\":0,\"max_entries\":96}"));
    static uint8_t want[OBS_PAGE_HEADER + 96 * 48];
    uint32_t n = obs_hash_page(0, want, sizeof want);
    CHECK(r["outcome"].is("DONE") && r["result"]["page_hex"].s == hex(want, n) && r["result"]["entries"].n == 96 &&
          r["result"]["engine"].is("SOFTWARE"), "hashes.read");
    // The largest answers fit the 16 KB line with room to spare.
    std::vector<std::string> lines = ask(req(4, "hashes.read", "{\"cursor\":0,\"max_entries\":96}"));
    CHECK(lines.size() == 1 && lines[0].size() + 1 < CTL_MAX_RESPONSE, "%zu bytes", lines.empty() ? 0 : lines[0].size());
}

static void test_identify_cached() {
    fresh("identify.cached");
    J r = ask1(req(1, "identify.cached"));
    CHECK(r["outcome"].is("DONE") && !r["result"]["present"].b && r["result"]["why"].is("NEVER"), "at power-up");
    // The TUI's Auto Detect, with its picker: Enter takes the first row.
    sim.nsect = 980u * 10 * 17;
    rx.clear(); rxi = 0;
    rx.push_back({ mock_now_ns / 1000 + 100000000ull, '\r' });      // long after the reset and IDENTIFY
    run_auto_detect();
    r = ask1(req(2, "identify.cached"));
    const J &x = r["result"];
    CHECK(x["present"].b && !x["stale"].b && x["origin"].is("AUTO_DETECT") && x["device"].is("MASTER") &&
          x["words_hex"].s.size() == 1024 && x["w49"].n == sim.id_w49, "after Auto Detect: %s %s", x["origin"].s.c_str(),
          x["words_hex"].s.substr(0, 16).c_str());
    // The bytes are the drive's own, low byte first; their SHA-256 beside them.
    std::string wh = x["words_hex"].s;
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i + 1 < wh.size(); i += 2) bytes.push_back((uint8_t)strtoul(wh.substr(i, 2).c_str(), 0, 16));
    uint8_t dg[32];
    ctl_sha256(bytes.data(), bytes.size(), dg);
    CHECK(x["sha256"].s == hex(dg, 32) && bytes.size() == 512 && (bytes[2] | (bytes[3] << 8)) == 980, "hex or digest (cyls %u)",
          bytes.size() == 512 ? (unsigned)(bytes[2] | (bytes[3] << 8)) : 0);
    // Unmount: stale. A new probe: gone (PROBE). A Ctrl+G: gone (CTRLG).
    set_mounted(true);
    menu_unmount();
    r = ask1(req(3, "identify.cached"));
    CHECK(r["result"]["present"].b && r["result"]["stale"].b, "not stale after unmount");
    (void)ide_probe_devices();
    r = ask1(req(4, "identify.cached"));
    CHECK(!r["result"]["present"].b && r["result"]["why"].is("PROBE"), "after a probe: %s", r["result"]["why"].s.c_str());
    uint16_t w[256] = { 0 };
    ide_idcache_put(w, IDE_IDC_DEBUG_I);
    uint8_t st;
    (void)ide_manual_chs(2, 40, &st);
    r = ask1(req(5, "identify.cached"));
    CHECK(!r["result"]["present"].b && r["result"]["why"].is("CTRLG"), "after a Ctrl+G: %s", r["result"]["why"].s.c_str());
    CHECK(sim.cmd_count[0xEC] == 1, "IDENTIFY sent %d times (only Auto Detect's)", sim.cmd_count[0xEC]);
}

static void test_debug_e_selftest_chip() {
    fresh("debug.e, selftest.status, chip.info");
    last_fail.kind = IDE_FAIL_ERR; last_fail.command = 0x20; last_fail.status = 0x51; last_fail.error = 0x40;
    last_fail.lba = 1234; last_fail.count = 8; last_fail.tf[2] = 0xC5;
    salvage.valid = true; salvage.seq = 3; salvage.lba = 1234;
    for (int i = 0; i < 512; i++) salvage.data[i] = (uint8_t)(i * 7);
    refused.identify_call = 2; refused.idp_zero = 5;
    J r = ask1(req(1, "debug.e"));
    const J &x = r["result"];
    uint8_t dg[32];
    ctl_sha256(salvage.data, 512, dg);
    CHECK(r["outcome"].is("DONE") && x["last_failure"]["kind"].is("ERR") && x["last_failure"]["command"].n == 0x20 &&
          x["last_failure"]["lba"].n == 1234 && x["last_failure"]["tf"].arr.size() == 5 && x["last_failure"]["tf"].arr[2]->n == 0xC5 &&
          x["salvage"]["valid"].b && x["salvage"]["seq"].n == 3 && x["salvage"]["data_sha256"].s == hex(dg, 32) &&
          !x["salvage"].get("data_hex") && x["refused"]["identify_call"].n == 2 && x["refused"]["idp_zero"].n == 5 &&
          x.get("unwitnessed_blocks") && x["self_resets"].obj.size() == 6 && !x.get("fault_before_boot"), "debug.e fields");
    r = ask1(req(2, "debug.e", "{\"include_data\":true}"));
    CHECK(r["result"]["salvage"]["data_hex"].s == hex(salvage.data, 512), "debug.e data_hex");
    r = ask1(req(3, "selftest.status"));
    CHECK(r["outcome"].is("DONE") && all_keys(r["result"], { "hash_engine", "core1_canary", "fault_before_boot", "config",
                                                              "image_sha256", "jumpers" }) &&
          r["result"]["hash_engine"].is("SOFTWARE") && r["result"]["core1_canary"].is("INTACT") &&
          r["result"]["fault_before_boot"].t == J::NUL && r["result"]["image_sha256"].is("NOT_COMPUTED") &&
#if ATABOY_BOARD_JUMPERS
          r["result"]["jumpers"].is("OK") &&
#else
          r["result"]["jumpers"].is("NOT_BUILT") &&
#endif
          r["result"]["config"].is("FLASH"), "selftest.status");
    r = ask1(req(4, "chip.info"));
    CHECK(all_keys(r["result"], { "die", "stepping", "unique_id", "package" }) && r["result"]["unique_id"].is(board_serial) &&
          r["result"]["stepping"].n == 4, "chip.info");
    r = ask1(req(5, "job.last"));
    CHECK(r["outcome"].is("DONE") && all_keys(r["result"], { "present" }) && !r["result"]["present"].b, "job.last at power-up");
}

// ---- 11. taskfile.snapshot: a job on core 1 (SPEC-v1 4.6, 5.3) ----------------
static bool sim_intrq_pending() { return sim.intrq; }
static void test_taskfile() {
    fresh("taskfile.snapshot");
    sim.error = 0x44; sim.reg[2] = 0x12; sim.reg[3] = 0x34; sim.reg[4] = 0x56; sim.reg[5] = 0x78; sim.reg[6] = 0xA0;
    sim.status = 0x51;
    sim.intrq = true;                               // a pending interrupt, which a register 7 read would clear
    int writes0 = sim.reg_writes;
    size_t cmds0 = sim.cmd_log.size();
    uint32_t log0 = log_next;
    std::string tty0 = tty;
    J r = ask1(req(40, "taskfile.snapshot"), nullptr, 50);
    const J &x = r["result"];
    CHECK(r["outcome"].is("DONE") && r["id"].n == 40 && x["error"].n == 0x44 && x["count"].n == 0x12 && x["sector"].n == 0x34 &&
          x["cyl_lo"].n == 0x56 && x["cyl_hi"].n == 0x78 && x["device"].n == 0xA0 && x["alt_status"].n == 0x51,
          "registers: error %llu alt %llu", (unsigned long long)x["error"].n, (unsigned long long)x["alt_status"].n);
    CHECK(x["err_bits"].arr.size() == 2 && x["err_bits"].arr[0]->is("UNC") && x["err_bits"].arr[1]->is("ABRT"), "err_bits");
    CHECK(sim.reg_writes == writes0 && sim.cmd_log.size() == cmds0, "the job wrote to the drive");
    CHECK(sim_intrq_pending(), "register 7 was read (INTRQ cleared)");
    CHECK(tty == tty0, "the job wrote to the TUI's queue (I-23)");
    // Its API markers, with the id, around nothing but its register reads.
    obs_log_entry_t b = log_ring[log0 % OBS_LOG_ENTRIES], e = log_ring[(log0 + 1) % OBS_LOG_ENTRIES];
    CHECK(log_next == log0 + 2 && b.kind == OBS_EV_API && b.outcome == OBS_API_BEGIN && b.value == CTL_OP_TASKFILE_SNAPSHOT &&
          (b.tf[0] | (b.tf[1] << 8)) == 40 && e.kind == OBS_EV_API && e.outcome == OBS_API_DONE && x["log_seq_before"].n == b.seq,
          "API markers: %u entries", (unsigned)(log_next - log0));
    J jl = ask1(req(41, "job.last"));
    uint8_t dg[32];
    std::string l40 = req(40, "taskfile.snapshot");
    ctl_sha256(l40.data(), l40.size() - 1, dg);
    J inner;
    std::string resp_text;
    CHECK(jl["result"]["present"].b && jl["result"]["op"].is("taskfile.snapshot") && jl["result"]["id"].n == 40 &&
          jl["result"]["request_sha256"].s == hex(dg, 32) && jl["result"]["log_seq_begin"].n == b.seq &&
          jl["result"]["log_seq_end"].n == e.seq && jl["result"]["response"]["result"]["error"].n == 0x44 &&
          jl["result"]["response"]["dev"]["seq"].n == r["dev"]["seq"].n, "job.last");
    // Not at the idle point: it waits; after 30 s it is withdrawn, nothing run.
    fresh("taskfile.snapshot: TUI busy");
    core1_screen = CTL_TUI_CONFIRM;
    log0 = log_next;
    std::vector<std::string> rr = ask(req(42, "taskfile.snapshot"), 29000);
    J st = ask1(req(43, "status"));
    CHECK(rr.empty() && st["result"]["job"]["queued"].is("taskfile.snapshot") && st["result"]["job"]["id"].n == 42 &&
          st["result"]["tui"]["state"].is("CONFIRM"), "waiting: %zu answers", rr.size());
    J busy = ask1(req(44, "taskfile.snapshot"));
    CHECK(is_refusal(busy, "E_INFLIGHT"), "a second job while one is queued");
    rr = ask("", 1100);
    J k;
    CHECK(rr.size() == 1 && parse_response(rr[0], k) && is_refusal(k, "E_TUI_BUSY") && k["id"].n == 42 &&
          k["err"]["why"].is("CONFIRM") && log_next == log0, "withdrawn at 30 s: %zu answers", rr.size());
    core1_screen = CTL_TUI_MAIN;
    pump(20);
    J jl2 = ask1(req(45, "job.last"));
    CHECK(!jl2["result"]["present"].b, "a withdrawn job kept as job.last");
    // Core 1 inside a dialog (never at its top): the same.
    fresh("taskfile.snapshot: core 1 in a dialog");
    core1_runs = false;
    rr = ask(req(46, "taskfile.snapshot"), 31000);
    CHECK(rr.size() == 1 && parse_response(rr[0], k) && is_refusal(k, "E_TUI_BUSY"), "core 1 away");
    // Mounted: refused at arrival, and at pickup.
    fresh("taskfile.snapshot: mounted");
    set_mounted(true);
    core1_runs = false;                             // refused at arrival: core 1 never sees it
    CHECK(is_refusal(ask1(req(47, "taskfile.snapshot")), "E_STATE") && ctl_job.state == CTL_JOB_EMPTY, "mounted at arrival");
    set_mounted(false);
    host_send(req(48, "taskfile.snapshot"));
    pump(5);
    set_mounted(true);                              // the human mounted while it waited
    core1_runs = true;
    rr = ask("", 20);
    CHECK(rr.size() == 1 && parse_response(rr[0], k) && is_refusal(k, "E_STATE") && sim.reg_writes == 0, "mounted at pickup");
    J jl3 = ask1(req(55, "job.last"));
    CHECK(!jl3["result"]["present"].b, "a job refused at pickup kept as job.last (it never ran)");
    // A USB command still running, or a reset it left pending: refused, not waited for.
    fresh("taskfile.snapshot: bus busy");
    core1_runs = false;                             // refused at arrival: core 1 never sees it
    busy_until_ns = mock_now_ns + 10000000000ull;
    CHECK(is_refusal(ask1(req(49, "taskfile.snapshot")), "E_BUS_BUSY") && ctl_job.state == CTL_JOB_EMPTY, "busy at arrival");
    busy_until_ns = 0;
    rec.stage = REC_SRST;
    CHECK(is_refusal(ask1(req(50, "taskfile.snapshot")), "E_BUS_BUSY") && ctl_job.state == CTL_JOB_EMPTY, "recovery pending");
    rec.stage = REC_NONE;
    host_send(req(51, "taskfile.snapshot"));
    pump(5);
    busy_until_ns = mock_now_ns + 10000000000ull;
    core1_runs = true;
    rr = ask("", 20);
    CHECK(rr.size() == 1 && parse_response(rr[0], k) && is_refusal(k, "E_BUS_BUSY"), "busy at pickup");
    busy_until_ns = 0;
    // The port closed while it was queued: withdrawn, never run, no answer.
    fresh("taskfile.snapshot: port closed");
    core1_runs = false;
    log0 = log_next;
    host_send(req(52, "taskfile.snapshot"));
    pump(5);
    tud_cdc_line_state_cb(1, false, false);
    pump(2);
    core1_runs = true;
    pump(20);
    tud_cdc_line_state_cb(1, true, false);
    rr = ask(req(53, "status"));
    CHECK(rr.size() == 1 && parse_response(rr[0], k) && k["id"].n == 53 && k["result"]["job"]["queued"].is("NONE") &&
          log_next == log0 && ctl_job.state == CTL_JOB_EMPTY, "a queued job survived the port closing");
    // Past its deadline (a bus held up): it stops reading and ends UNKNOWN.
    fresh("taskfile.snapshot: deadline");
    sim_reg_cost_ns = 30000000;                     // 30 ms a register read: 4 fit in the 100 ms, no 5th
    r = ask1(req(54, "taskfile.snapshot"), nullptr, 400);
    sim_reg_cost_ns = 0;
    CHECK(r["outcome"].is("UNKNOWN") && r["retry"].is("AFTER_STATUS") && ctl_job.regs_read <= 4,
          "past the deadline: %s after %u reads", r["outcome"].s.c_str(), ctl_job.regs_read);
}

// ---- 12. no answer while the port is closed, and I-10 on garbage -------------
static std::string state_digest() {
    // What an invalid line must not change: the config, the log, the hashes,
    // the replay table, is_mounted, the rules.
    std::string s((const char *)&config, sizeof config);
    uint8_t h[OBS_PAGE_HEADER];
    obs_log_page(0, h, sizeof h); s.append((const char *)h, 24);
    obs_hash_page(0, h, sizeof h); s.append((const char *)h, 24);
    s += std::to_string(c.replay.max_id) + "/" + std::to_string(is_mounted) + std::to_string(mchs_no_identify) +
         std::to_string(ctl_job.state) + "/" + std::to_string(sim.reg_writes);
    return s;
}
static void test_garbage_changes_nothing() {
    fresh("I-10: garbage changes nothing");
    std::string d0 = state_digest();
    const char *noise[] = { "AT\r\n", "ATZ\r\n", "\x1b[A\n", "{\"v\":1}\n", "{}\n", "null\n", "{\"v\":1,\"id\":1,\"op\":\"wp.off\",\"args\":{}}\n",
                            "{\"v\":1,\"id\":1,\"op\":\"setup.ctrlg\",\"args\":{}}\n", "\x07\x06\x02y\n", "+++\n" };
    size_t answers = 0;
    for (auto n : noise) { answers += ask(n, 10).size(); }
    CHECK(answers == sizeof noise / sizeof noise[0] && state_digest() == d0, "%zu answers; state changed %d", answers, state_digest() != d0);
}

// ---- 13. core 1's real loop takes a job at its idle point (its own run) -------
#if ATABOY_BOARD_JUMPERS
static void core1_trip_stage();
#endif
static void core1_done() {
    CHECK(ctl_job.state == CTL_JOB_EMPTY && c.jl_present && c.jl_id == 61, "the job did not run from core1_entry (state %u)",
          (unsigned)ctl_job.state);
    CHECK(ctl_tui_state == CTL_TUI_MAIN, "published state %u", ctl_tui_state);
#if ATABOY_BOARD_JUMPERS
    core1_trip_stage();                             // the tripwire screen takes over; does not return
#endif
    printf("%d checks, %d failed\n", checks, failures);
    exit(failures > 255 ? 255 : failures);
}
static void test_core1_loop() {
    fresh("core 1's loop");
    core1_runs = false;                             // core 1 is its real loop below, not the stepper
    host_send(req(61, "taskfile.snapshot"));
    pump(5);                                        // queued (core 0), core 1 not running yet
    CHECK(ctl_job.state == CTL_JOB_QUEUED, "not queued");
    // Core 1's loop from here: core 0 answers from its idle hook below.
    struct Hook { static void idle() { one_task(); core1_done(); } };
    idle_after_ns = mock_now_ns + 500000000ull;
    on_idle = Hook::idle;
    // The loop never returns. With a terminal, get_input polls the queue,
    // which ends the run from the hook above once the clock is past.
    cdc_connected = true;
    core1_entry();
}
static void test_core1_confirm() {
    fresh("core 1's loop: a confirm box open");
    current_screen = SCREEN_CONFIRM; confirm_type = 3;
    core1_runs = false;
    host_send(req(62, "taskfile.snapshot"));
    pump(5);
    struct Hook { static void idle() {
        CHECK(ctl_job.state == CTL_JOB_QUEUED && !c.jl_present, "a job ran with a confirm box open (state %u)", (unsigned)ctl_job.state);
        CHECK(ctl_tui_state == CTL_TUI_CONFIRM, "published state %u", ctl_tui_state);
        printf("%d checks, %d failed\n", checks, failures);
        exit(failures > 255 ? 255 : failures);
    } };
    idle_after_ns = mock_now_ns + 500000000ull;
    on_idle = Hook::idle;
    cdc_connected = true;
    core1_entry();
}

// ---- 14. stage 2: the ratchet on core 0 (SPEC-v1 4.12, I-8, I-19) ------------
// What a refusal must leave as it was (I-10, I-27): everything but the
// protocol's own bookkeeping (dev.seq, the replay table, max_mutating_id).
static std::string state2() {
    std::string s((const char *)&config, sizeof config);
    uint8_t h[OBS_PAGE_HEADER];
    obs_log_page(0, h, sizeof h); s.append((const char *)h + 16, 8);
    obs_hash_page(0, h, sizeof h); s.append((const char *)h + 16, 8);
    char b[256];
    snprintf(b, sizeof b, "/e%u m%d s%d ms%u src%u f%d n%d a%d nd%d idc%d%d wl%u jl%d%u rw%d cmd%zu tty%zu",
             (unsigned)ctl_epoch, (int)is_mounted, (int)current_screen, (unsigned)ctl_mount_seq, (unsigned)ctl_setup_src,
             (int)identify_forbidden, (int)mchs_no_identify, (int)manual_chs_active, (int)native_declared,
             (int)idc.present, (int)idc.stale, (unsigned)ctl_wp_last, (int)c.jl_present, (unsigned)c.jl_id,
             sim.reg_writes, sim.cmd_log.size(), tty.size());
    return s + b + hdd_model_raw;
}
static uint32_t epoch_now() { return ctl_epoch; }
static bool log_entry_is(uint32_t seq, uint8_t op, uint32_t id, uint8_t outcome) {
    const obs_log_entry_t &e = log_ring[seq % OBS_LOG_ENTRIES];
    return e.seq == seq && e.kind == OBS_EV_API && e.value == op && e.outcome == outcome &&
           (e.tf[0] | (e.tf[1] << 8) | (e.tf[2] << 16) | ((uint32_t)e.tf[3] << 24)) == id;
}
static std::string eargs(const char *k, uint64_t v) { return "{\"" + std::string(k) + "\":" + std::to_string(v) + "}"; }

static void test_wp_on() {
    fresh("wp.on");
    human_wp(false);                                // the human turned write protect off in Features
    J s0 = ask1(req(1, "status"));
    CHECK(!s0["result"]["wp"]["effective"].b && s0["result"]["wp"]["last_change"].is("TUI"), "setup");
    uint32_t e0 = epoch_now(), log0 = log_next;
    J r = ask1(req(10, "wp.on"));
    const J &x = r["result"];
    CHECK(r["outcome"].is("DONE") && all_keys(x, { "wp", "changed", "log_seq" }) &&
          all_keys(x["wp"], { "effective", "config", "forced_by", "last_change" }) && x["wp"]["effective"].b &&
          x["wp"]["config"].b && x["wp"]["last_change"].is("API") && x["changed"].b, "wp.on result");
    CHECK(config.drive_write_protected && ide_write_protected(), "write protect not on in RAM");
    CHECK(epoch_now() == e0 + 1 && r["dev"]["epoch"].n == e0 + 1, "epoch %u, was %u", (unsigned)epoch_now(), (unsigned)e0);
    CHECK(log_next == log0 + 1 && x["log_seq"].n == log0 && log_entry_is(log0, CTL_OP_WP_ON, 10, OBS_API_DONE),
          "one API entry, its seq in the result (%u entries)", (unsigned)(log_next - log0));
    CHECK(saves == 0, "wp.on saved the settings (5.5)");
    // Again: nothing changes but the log; changed false, epoch kept, last_change stays API.
    r = ask1(req(11, "wp.on"));
    CHECK(r["outcome"].is("DONE") && !r["result"]["changed"].b && r["result"]["wp"]["effective"].b &&
          epoch_now() == e0 + 1 && r["result"]["wp"]["last_change"].is("API") && log_entry_is(log0 + 1, CTL_OP_WP_ON, 11, OBS_API_DONE),
          "wp.on twice: changed %d epoch %u", r["result"]["changed"].b, (unsigned)epoch_now());
    // Never a toggle (X2), and nothing on the bus.
    for (uint32_t id = 12; id < 16; id++) ask1(req(id, "wp.on"));
    CHECK(config.drive_write_protected && sim.reg_writes == 0 && sim.cmd_log.empty(), "wp.on toggled or touched the bus");
    J st = ask1(req(2, "status"));
    CHECK(st["result"]["wp"]["effective"].b && st["result"]["wp"]["last_change"].is("API") && st["result"]["epoch"].n == e0 + 1,
          "status after wp.on");
    // expect_epoch (4.3): the wrong one is E_STALE and changes nothing; the right one runs.
    human_wp(false);
    std::string d0 = state2();
    r = ask1(req(20, "wp.on", eargs("expect_epoch", epoch_now() + 1).c_str()));
    CHECK(is_refusal(r, "E_STALE") && r["retry"].is("AFTER_STATUS") && r["err"]["field"].is("expect_epoch") && state2() == d0 &&
          api_entries(CTL_OP_WP_ON, 20) == 0, "a stale expect_epoch: %s", r["err"]["code"].s.c_str());
    r = ask1(req(21, "wp.on", eargs("expect_epoch", epoch_now() - 1).c_str()));
    CHECK(is_refusal(r, "E_STALE") && state2() == d0, "an older expect_epoch");
    r = ask1(req(22, "wp.on", eargs("expect_epoch", epoch_now()).c_str()));
    CHECK(r["outcome"].is("DONE") && r["result"]["changed"].b && config.drive_write_protected, "the current expect_epoch refused");
    // A Ctrl+G's forced write protect shows in wp.on's result too.
    mchs_no_identify = true; hooked();
    r = ask1(req(23, "wp.on"));
    CHECK(r["result"]["wp"]["forced_by"].arr.size() == 1 && r["result"]["wp"]["forced_by"].arr[0]->is("CTRLG"), "forced_by");
    // Allowed while a job waits for the TUI (4.12: no lease, jobs do not block it).
    fresh("wp.on while a job waits");
    human_wp(false);
    core1_screen = CTL_TUI_CONFIRM;
    host_send(req(30, "taskfile.snapshot"));
    pump(5);
    r = ask1(req(31, "wp.on"));
    CHECK(r["outcome"].is("DONE") && config.drive_write_protected && ctl_job.state == CTL_JOB_QUEUED, "wp.on refused while a job waits");
    core1_screen = CTL_TUI_MAIN;
    pump(50);
    host_lines();
}

static void test_pipelined_off() {
    fresh("pipelined.off");
    human_pipelined(true);
    uint32_t e0 = epoch_now(), log0 = log_next;
    J r = ask1(req(10, "pipelined.off"));
    CHECK(r["outcome"].is("DONE") && all_keys(r["result"], { "pipelined_reads", "changed" }) &&
          !r["result"]["pipelined_reads"].b && r["result"]["changed"].b && !config.pipelined_reads, "pipelined.off result");
    CHECK(epoch_now() == e0 + 1 && log_next == log0 + 1 && log_entry_is(log0, CTL_OP_PIPELINED_OFF, 10, OBS_API_DONE),
          "epoch %u log %u", (unsigned)(epoch_now() - e0), (unsigned)(log_next - log0));
    for (uint32_t id = 11; id < 15; id++) {
        r = ask1(req(id, "pipelined.off"));
        CHECK(r["outcome"].is("DONE") && !r["result"]["changed"].b && !config.pipelined_reads, "pipelined.off %u toggled", id);
    }
    CHECK(epoch_now() == e0 + 1 && saves == 0 && sim.reg_writes == 0, "a no-op moved the epoch, saved, or touched the bus");
    human_pipelined(true);
    std::string d0 = state2();
    r = ask1(req(20, "pipelined.off", eargs("expect_epoch", epoch_now() + 5).c_str()));
    CHECK(is_refusal(r, "E_STALE") && r["err"]["field"].is("expect_epoch") && state2() == d0 && config.pipelined_reads,
          "a stale expect_epoch");
    r = ask1(req(21, "pipelined.off", eargs("expect_epoch", epoch_now()).c_str()));
    CHECK(r["outcome"].is("DONE") && !config.pipelined_reads, "the current expect_epoch");
}

// identify.forbid (4.12) and every guard it adds (1.4; I-1, I-22; R-API-1
// finding 6). Each guard is driven both ways: IDENTIFY goes out before it,
// and after it nothing is sent.
static void test_identify_forbid() {
    fresh("identify.forbid");
    sim.nsect = 980u * 10 * 17;
    uint16_t buf[256];
    // Before: every path sends IDENTIFY.
    CHECK(ide_identify(buf) && sim.cmd_count[0xEC] == 1, "IDENTIFY before the forbid (%d)", sim.cmd_count[0xEC]);
    uint32_t e0 = epoch_now(), log0 = log_next;
    J r = ask1(req(10, "identify.forbid"));
    const J &x = r["result"];
    CHECK(r["outcome"].is("DONE") && all_keys(x, { "identify_forbidden", "changed", "log_seq" }) && x["identify_forbidden"].b &&
          x["changed"].b && x["log_seq"].n == log0 && log_entry_is(log0, CTL_OP_IDENTIFY_FORBID, 10, OBS_API_DONE) &&
          epoch_now() == e0 + 1, "identify.forbid result");
    J st = ask1(req(1, "status"));
    CHECK(st["result"]["rules"]["identify_forbidden"].b && st["result"]["rules"]["identify_barred"].b &&
          !st["result"]["rules"]["no_identify_ctrlg"].b,
          "status.rules after identify.forbid");
    CHECK(!mchs_no_identify, "identify.forbid set Ctrl+G's rule (whose screen texts are about a Ctrl+G)");
    r = ask1(req(11, "identify.forbid"));
    CHECK(r["outcome"].is("DONE") && !r["result"]["changed"].b && epoch_now() == e0 + 1, "twice");
    // ide_identify (every IDENTIFY the firmware sends for itself) refuses, nothing sent.
    uint32_t call0 = refused.identify_call, run0 = refused.identify_run;
    int reg0 = sim.reg_writes;
    CHECK(!ide_identify(buf) && sim.cmd_count[0xEC] == 1 && refused.identify_call == call0 + 1 && sim.reg_writes == reg0,
          "ide_identify sent after identify.forbid");
    // identify_send, the last layer before the strobe, refuses on its own
    // (without it the tripwire below would stop the firmware: a failed check
    // here, not a stop, so the mutation run sees which layer went).
    int got = -1;
    trip_expected = true;
    if (!setjmp(trip_jb)) got = identify_run(buf);
    trip_expected = false;
    CHECK(got == 0 && !trip.tripped && refused.identify_run == run0 + 1 && sim.cmd_count[0xEC] == 1,
          "identify_run sent or reached the tripwire (returned %d, tripped %d)", got, trip.tripped);
    // Auto Detect's probe clears Ctrl+G's rule, never this one.
    (void)ide_probe_devices();
    CHECK(ide_identify_forbidden() && !ide_identify(buf) && sim.cmd_count[0xEC] == 1, "the probe cleared identify.forbid");
    // The tripwire (I-22), with every layer above it taken away: ECh strobed
    // straight at ide_write_reg stops, before the strobe.
    int ec0 = sim.cmd_count[0xEC];
    trip_expected = true;
    if (!setjmp(trip_jb)) {
        ide_write_reg(7, 0xEC);
        CHECK(false, "the tripwire let ECh through after identify.forbid");
    }
    trip_expected = false;
    CHECK(trip.tripped && trip.opcode == 0xEC && trip.rule == IDE_TRIP_FORBIDDEN && sim.cmd_count[0xEC] == ec0,
          "tripwire record: rule %u, ECh sent %d", trip.rule, sim.cmd_count[0xEC] - ec0);
    // Both ways: without identify.forbid the same strobe goes out.
    fresh("identify.forbid: the tripwire lets ECh through without it");
    trip_expected = true;
    if (!setjmp(trip_jb)) ide_write_reg(7, 0xEC);
    trip_expected = false;
    CHECK(!trip.tripped && sim.cmd_count[0xEC] == 1, "ECh refused with no rule set");
    // A stale expect_epoch changes nothing.
    std::string d0 = state2();
    r = ask1(req(12, "identify.forbid", eargs("expect_epoch", epoch_now() + 1).c_str()));
    CHECK(is_refusal(r, "E_STALE") && !ide_identify_forbidden() && state2() == d0, "a stale expect_epoch");
}

// The TUI's own pre-checks after identify.forbid (R-API-1 finding 6): Debug
// I and S, Auto Detect's Enter and its Y refuse before anything is sent,
// with the new text. Both ways.
static void test_forbid_tui_prechecks() {
    fresh("identify.forbid: the TUI's pre-checks, before");
    sim.nsect = 980u * 10 * 17;
    // Before: Enter on Auto Detect asks Y/N (nothing sent yet), Debug I sends IDENTIFY.
    CHECK(auto_detect_enter() && current_screen == SCREEN_CONFIRM && sim.hw_resets == 0, "Auto Detect's Enter before");
    current_screen = SCREEN_DEBUG;
    debug_key('i');
    CHECK(sim.cmd_count[0xEC] == 1, "Debug I did not send IDENTIFY before the forbid");
    fresh("identify.forbid: the TUI's pre-checks, after");
    sim.nsect = 980u * 10 * 17;
    ask1(req(10, "identify.forbid"));
    int reg0 = sim.reg_writes, hw0 = sim.hw_resets;
    for (int k : { 'i', 'I', 's', 'S' }) {
        size_t t0 = tty.size();
        current_screen = SCREEN_DEBUG;
        debug_key(k);
        std::string out = tty.substr(t0);
        CHECK(out.find(API_FORBIDDEN_TEXT) != std::string::npos && out.find("Nothing sent.") != std::string::npos &&
              sim.reg_writes == reg0 && sim.cmd_count[0xEC] == 0, "Debug %c after identify.forbid", k);
    }
    // Enter on Auto Detect: refused at once (no Y/N), the box says why, nothing sent.
    current_screen = SCREEN_MAIN;
    rx.clear(); rxi = 0;
    rx.push_back({ mock_now_ns / 1000 + 1000, '\r' });     // the key that closes the box
    size_t t0 = tty.size();
    CHECK(!auto_detect_enter() && current_screen == SCREEN_MAIN, "Enter on Auto Detect asked Y/N after identify.forbid");
    CHECK(tty.substr(t0).find(API_FORBIDDEN_BOX) != std::string::npos && sim.hw_resets == hw0 && sim.reg_writes == reg0 &&
          !force_detect && !show_detect_result, "Auto Detect's Enter: the box, and no RESET- (%d resets)", sim.hw_resets - hw0);
    // Y at the question (the question was open before the forbid came).
    rx.clear(); rxi = 0;
    rx.push_back({ mock_now_ns / 1000 + 1000, '\r' });
    t0 = tty.size();
    CHECK(!run_auto_detect() && tty.substr(t0).find(API_FORBIDDEN_BOX) != std::string::npos && sim.hw_resets == hw0 &&
          sim.reg_writes == reg0 && !force_detect, "Auto Detect's Y after identify.forbid");
    // With a Ctrl+G's rule as well, the Ctrl+G text comes first, as before.
    mchs_no_identify = true; hooked();
    current_screen = SCREEN_DEBUG;
    t0 = tty.size();
    debug_key('i');
    CHECK(tty.substr(t0).find(DEBUG_REFUSED_MCHS) != std::string::npos && sim.reg_writes == reg0, "both rules: the Ctrl+G text");
    // The box texts fit their boxes (50 columns; the Debug screen 68).
    CHECK(strlen(API_FORBIDDEN_BOX) <= 50 && strlen(API_FORBIDDEN_TEXT) <= 68 && visible_strlen(API_FORBIDDEN_FORCE) <= 50,
          "a refusal text is wider than its box");
}

// ---- 15. stage 2: unmount and detach, naming the mount (4.12; I-24, I-27, I-28) ----
static std::string named(uint32_t ms) { return eargs("expect_mount_seq", ms); }
static std::string named_e(uint32_t ms, uint32_t ep) {
    return "{\"expect_mount_seq\":" + std::to_string(ms) + ",\"expect_epoch\":" + std::to_string(ep) + "}";
}
static void test_unmount() {
    fresh("unmount");
    sim.nsect = 980u * 10 * 17;
    // expect_mount_seq is required (X66), for both.
    J r = ask1(req(10, "unmount"));
    CHECK(is_refusal(r, "E_SCHEMA") && r["err"]["field"].is("expect_mount_seq"), "unmount without expect_mount_seq");
    r = ask1(req(11, "detach"));
    CHECK(is_refusal(r, "E_SCHEMA") && r["err"]["field"].is("expect_mount_seq"), "detach without expect_mount_seq");
    tui_setup_chs(980, 10, 17);
    tui_mount();
    CHECK(ctl_mount_seq == 1 && is_mounted, "mounted");
    // One lower, one higher: refused at arrival, nothing changes (I-27, X68).
    for (uint32_t bad : { 0u, 2u, 7u }) {
        std::string d0 = state2();
        uint32_t lg = log_next;
        r = ask1(req(20 + bad, "unmount", named(bad).c_str()));
        CHECK(is_refusal(r, "E_STALE") && r["retry"].is("AFTER_STATUS") && r["err"]["field"].is("expect_mount_seq") &&
              state2() == d0 && log_next == lg && is_mounted, "unmount naming mount %u of 1", bad);
    }
    // Refused at arrival, at once, never queued: with core 1 away the answer
    // still comes (a pickup check alone would wait 30 s for the TUI).
    core1_runs = false;
    r = ask1(req(28, "unmount", named(0).c_str()));
    CHECK(is_refusal(r, "E_STALE") && r["err"]["field"].is("expect_mount_seq") && ctl_job.state == CTL_JOB_EMPTY,
          "a stale mount name not refused at arrival");
    r = ask1(req(29, "detach", named_e(1, epoch_now() + 3).c_str()));
    CHECK(is_refusal(r, "E_STALE") && r["err"]["field"].is("expect_epoch") && ctl_job.state == CTL_JOB_EMPTY,
          "a stale expect_epoch not refused at arrival");
    core1_runs = true;
    // The current mount: DONE, the confirm-4 path, the main menu.
    uint32_t e0 = epoch_now(), lg = log_next;
    std::string tty0 = tty;
    int regs0 = sim.reg_writes;
    r = ask1(req(30, "unmount", named(1).c_str()));
    const J &x = r["result"];
    CHECK(r["outcome"].is("DONE") && all_keys(x, { "mounted", "changed", "native_mode_ended" }) && !x["mounted"].b &&
          x["changed"].b && !x["native_mode_ended"].b, "unmount result");
    CHECK(!is_mounted && current_screen == SCREEN_MAIN && needs_full_redraw, "the screen does not follow (I-24): %d", current_screen);
    CHECK(epoch_now() > e0 && ctl_mount_seq == 1, "epoch or mount_seq");
    // API_BEGIN, the UNMOUNT marker, API_END; nothing on the bus, nothing to the TUI.
    CHECK(log_next == lg + 3 && log_entry_is(lg, CTL_OP_UNMOUNT, 30, OBS_API_BEGIN) &&
          log_ring[(lg + 1) % OBS_LOG_ENTRIES].kind == OBS_EV_UNMOUNT && log_entry_is(lg + 2, CTL_OP_UNMOUNT, 30, OBS_API_DONE),
          "log: %u entries", (unsigned)(log_next - lg));
    CHECK(sim.reg_writes == regs0 && tty == tty0, "unmount touched the bus or the TUI's queue");
    J jl = ask1(req(2, "job.last"));
    CHECK(jl["result"]["op"].is("unmount") && jl["result"]["id"].n == 30 && jl["result"]["response"]["result"]["changed"].b, "job.last");
    // Unmounted, naming that mount: DONE, nothing changed.
    std::string d1 = state2();
    lg = log_next;
    r = ask1(req(31, "unmount", named(1).c_str()));
    CHECK(r["outcome"].is("DONE") && !r["result"]["changed"].b && !r["result"]["mounted"].b && log_next == lg + 2 &&
          api_entries(CTL_OP_UNMOUNT, 31) == 2, "unmount while unmounted");
    // A byte-identical resend is the stored answer, not a second run.
    std::string l30 = req(30, "unmount", named(1).c_str());
    std::vector<std::string> rr = ask(l30);
    J k;
    CHECK(rr.size() == 1 && parse_response(rr[0], k) && k["replayed"].b && api_entries(CTL_OP_UNMOUNT, 30) == 2, "resend of unmount");
    // expect_epoch as well: stale at arrival refused, current runs.
    tui_mount();
    r = ask1(req(40, "unmount", named_e(2, epoch_now() + 1).c_str()));
    CHECK(is_refusal(r, "E_STALE") && r["err"]["field"].is("expect_epoch") && is_mounted, "unmount with a stale expect_epoch");
    r = ask1(req(41, "unmount", named_e(2, epoch_now()).c_str()));
    CHECK(r["outcome"].is("DONE") && !is_mounted, "unmount with the current epoch");
}

// A mount between arrival and pickup (X67): caught at pickup, nothing changes.
static void test_unmount_pickup() {
    fresh("unmount: a new mount before pickup");
    tui_setup_chs(980, 10, 17);
    tui_mount();
    core1_screen = CTL_TUI_CONFIRM;                 // the human has a box open: the job waits
    host_send(req(10, "unmount", named(1).c_str()));
    pump(20);
    CHECK(ctl_job.state == CTL_JOB_QUEUED, "not queued");
    menu_unmount(); current_screen = SCREEN_MAIN;   // the human's own unmount, then mount again
    tui_mount();
    CHECK(ctl_mount_seq == 2, "mount_seq %u", (unsigned)ctl_mount_seq);
    std::string d0 = state2();
    uint32_t lg = log_next;
    core1_screen = CTL_TUI_MOUNTED;
    std::vector<std::string> rr = ask("", 40);
    J k;
    CHECK(rr.size() == 1 && parse_response(rr[0], k) && is_refusal(k, "E_STALE") && k["err"]["field"].is("expect_mount_seq") &&
          k["retry"].is("AFTER_STATUS") && is_mounted && log_next == lg, "the newer mount was cut (%zu answers)", rr.size());
    CHECK(state2().substr(0, sizeof config) == d0.substr(0, sizeof config) && !c.jl_present, "state or job.last changed");
    // The epoch named, changed while it waited (a WP toggle): caught at pickup too.
    fresh("unmount: the epoch moves before pickup");
    tui_setup_chs(980, 10, 17);
    tui_mount();
    core1_screen = CTL_TUI_CONFIRM;
    host_send(req(11, "unmount", named_e(1, epoch_now()).c_str()));
    pump(20);
    human_wp(false);
    core1_screen = CTL_TUI_MOUNTED;
    rr = ask("", 40);
    CHECK(rr.size() == 1 && parse_response(rr[0], k) && is_refusal(k, "E_STALE") && k["err"]["field"].is("expect_epoch") &&
          is_mounted, "a stale epoch at pickup");
    // And both ways: with nothing changed meanwhile, the queued one runs.
    fresh("unmount: queued, then run");
    tui_setup_chs(980, 10, 17);
    tui_mount();
    core1_screen = CTL_TUI_CONFIRM;
    host_send(req(12, "unmount", named_e(1, epoch_now()).c_str()));
    pump(20);
    core1_screen = CTL_TUI_MOUNTED;
    rr = ask("", 40);
    CHECK(rr.size() == 1 && parse_response(rr[0], k) && k["outcome"].is("DONE") && !is_mounted, "the queued unmount did not run");
    // Another job in flight: E_INFLIGHT, the queued one untouched.
    fresh("unmount: another job in flight");
    core1_screen = CTL_TUI_CONFIRM;
    host_send(req(13, "taskfile.snapshot"));
    pump(5);
    J r = ask1(req(14, "unmount", named(0).c_str()));
    CHECK(is_refusal(r, "E_INFLIGHT") && ctl_job.state == CTL_JOB_QUEUED && ctl_job.id == 13, "unmount with a job queued");
    core1_screen = CTL_TUI_MAIN;
    pump(50);
    host_lines();
}

// Ctrl+G's power-up default mode (native): the unmount waits for a USB read
// as confirm-4 does, and says whether the mode ended.
static void test_unmount_native() {
    for (int busy = 0; busy < 2; busy++) {
        fresh(busy ? "unmount, native, a USB read still running" : "unmount, native");
        uint8_t st;
        CHECK(ide_manual_chs_native(10, 17, &st) == IDE_MCHS_OK, "native Ctrl+G");
        use_lba_mode = false; cur_cyls = 980; cur_heads = 10; cur_spt = 17; total_lba_sectors = 0;
        strcpy(hdd_model_raw, MCHS_MODEL_NATIVE);
        sync_to_config();
        ctl_setup_noted(CTL_SRC_TUI_CTRLG_NATIVE);
        tui_mount();
        if (busy) busy_until_ns = mock_now_ns + 60000000000ull;     // longer than the 25 s wait
        J r = ask1(req(10, "unmount", named(1).c_str()), nullptr, busy ? 26000 : 60);
        CHECK(r["outcome"].is("DONE") && !r["result"]["mounted"].b && r["result"]["changed"].b &&
              r["result"]["native_mode_ended"].b == !busy, "native_mode_ended %d", r["result"]["native_mode_ended"].b);
        CHECK(ide_native_drive() && ide_mchs_no_identify(), "the no-91h or no-IDENTIFY rule ended");
        J s = ask1(req(1, "status"));
        CHECK(s["result"]["setup"]["source"].is("NONE") && s["result"]["rules"]["native_active"].b == (busy != 0),
              "status after: source %s native_active %d", s["result"]["setup"]["source"].s.c_str(), s["result"]["rules"]["native_active"].b);
        busy_until_ns = 0;
    }
}

static void test_detach() {
    fresh("detach");
    uint8_t st;
    CHECK(ide_manual_chs_native(10, 17, &st) == IDE_MCHS_OK, "native Ctrl+G");
    use_lba_mode = false; cur_cyls = 980; cur_heads = 10; cur_spt = 17; total_lba_sectors = 0;
    strcpy(hdd_model_raw, MCHS_MODEL_NATIVE);
    sync_to_config();
    ctl_setup_noted(CTL_SRC_TUI_CTRLG_NATIVE);
    tui_mount();
    uint16_t w[256] = { 0 };
    ide_idcache_put(w, IDE_IDC_DEBUG_I);            // an IDENTIFY held from before (to be dropped)
    std::string d0 = state2();
    J r = ask1(req(9, "detach", named(0).c_str()));
    CHECK(is_refusal(r, "E_STALE") && state2() == d0, "a stale detach changed something");
    uint32_t lg = log_next;
    int regs0 = sim.reg_writes;
    std::string tty0 = tty;
    r = ask1(req(10, "detach", named(1).c_str()));
    CHECK(r["outcome"].is("DONE") && all_keys(r["result"], { "detached" }) && r["result"]["detached"].b, "detach result");
    CHECK(!is_mounted && config.cyls == 0 && config.heads == 0 && config.spt == 0 && cur_cyls == 0 && !ide_manual_chs_native_active(),
          "the geometry is still set");
    CHECK(!strcmp(hdd_model_raw, MCHS_MODEL_CLEARED) && current_screen == SCREEN_MAIN && !idc.present && idc.why == IDE_IDC_WHY_DETACH,
          "model %s, screen %d, identify.cached present %d", hdd_model_raw, current_screen, idc.present);
    CHECK(ide_native_drive() && ide_mchs_no_identify(), "detach ended a sticky rule");
    CHECK(sim.reg_writes == regs0 && tty == tty0, "detach touched the bus or the TUI queue");
    CHECK(log_entry_is(lg, CTL_OP_DETACH, 10, OBS_API_BEGIN) && log_entry_is(log_next - 1, CTL_OP_DETACH, 10, OBS_API_DONE),
          "API markers");
    J s = ask1(req(1, "status"));
    CHECK(s["result"]["setup"]["source"].is("NONE") && s["result"]["geometry"]["total_sectors"].n == 0 &&
          s["result"]["mount_seq"].n == 1, "status after detach");
    // Nothing can mount it again without a new set-up: Mount finds nothing.
    J ic = ask1(req(2, "identify.cached"));
    CHECK(!ic["result"]["present"].b && ic["result"]["why"].is("DETACH"), "identify.cached after detach");
    // Unmounted, a set-up made, no mount yet: detach names the last mount (0 here).
    fresh("detach: a set-up, never mounted");
    tui_setup_chs(980, 10, 17);
    r = ask1(req(11, "detach", named(1).c_str()));
    CHECK(is_refusal(r, "E_STALE") && config.cyls == 980, "a detach naming a mount that never was");
    r = ask1(req(12, "detach", named(0).c_str()));
    CHECK(r["outcome"].is("DONE") && config.cyls == 0 && hdd_model_raw[0] == 0 && ctl_setup_src == CTL_SRC_NONE, "detach of a set-up");
}

// A USB read still running after the wait: FAILED, E_BUS_BUSY, and nothing
// cleared (R-API-1 finding 8, X57).
static void test_detach_busy() {
    fresh("detach: a USB read still running");
    tui_setup_chs(980, 10, 17);
    tui_mount();
    busy_until_ns = mock_now_ns + 60000000000ull;
    uint32_t lg = log_next;
    J r = ask1(req(10, "detach", named(1).c_str()), nullptr, 26000);
    CHECK(r["outcome"].is("FAILED") && r["err"]["code"].is("E_BUS_BUSY") && r["retry"].is("AFTER_STATUS") &&
          r["result"].t == J::OBJ && all_keys(r["result"], { "detached", "mounted" }) && !r["result"]["detached"].b &&
          !r["result"]["mounted"].b, "detach while busy: %s %s", r["outcome"].s.c_str(), r["err"]["code"].s.c_str());
    CHECK(!is_mounted && config.cyls == 980 && config.heads == 10 && config.spt == 17 && cur_cyls == 980 &&
          ctl_setup_src == CTL_SRC_TUI_DETECT_NORMAL, "detach cleared the geometry under a running read");
    CHECK(log_entry_is(log_next - 1, CTL_OP_DETACH, 10, OBS_API_FAILED) && log_entry_is(lg, CTL_OP_DETACH, 10, OBS_API_BEGIN),
          "API_END FAILED");
    J jl = ask1(req(1, "job.last"));
    CHECK(jl["result"]["present"].b && jl["result"]["response"]["outcome"].is("FAILED"), "job.last of the FAILED detach");
    busy_until_ns = 0;
    // The read ended: the same detach again (a new id) clears it.
    r = ask1(req(11, "detach", named(1).c_str()));
    CHECK(r["outcome"].is("DONE") && config.cyls == 0, "detach after the read ended");
    // A read that ends within the wait: detach waits for it (every mode,
    // not only native), then clears.
    fresh("detach: waits for a USB read to end");
    tui_setup_chs(980, 10, 17);
    tui_mount();
    busy_until_ns = mock_now_ns + 10000000000ull;  // 10 s more
    uint64_t t0 = mock_now_ns;
    r = ask1(req(12, "detach", named(1).c_str()), nullptr, 12000);
    CHECK(r["outcome"].is("DONE") && config.cyls == 0 && mock_now_ns - t0 >= 10000000000ull, "detach did not wait: %s",
          r["outcome"].s.c_str());
    busy_until_ns = 0;
}

// No terminal on interface 0 (R-API-1 finding 1): the TUI's queue full and
// nobody reading. The jobs end within their time and put nothing in it.
static void test_ratchet_no_terminal() {
    fresh("the ratchet with no terminal");
    cdc_connected = false;
    tty.assign(4096, 'q');
    tui_setup_chs(980, 10, 17);
    tui_mount();
    std::string tty0 = tty;
    uint64_t t0 = mock_now_ns;
    J r = ask1(req(10, "unmount", named(1).c_str()));
    CHECK(r["outcome"].is("DONE") && tty == tty0 && mock_now_ns - t0 < 30000000000ull, "unmount with no terminal");
    r = ask1(req(11, "detach", named(1).c_str()));
    CHECK(r["outcome"].is("DONE") && tty == tty0, "detach with no terminal");
    for (const char *op : { "wp.on", "pipelined.off", "identify.forbid" }) {
        static uint32_t id = 20;
        r = ask1(req(id++, op));
        CHECK(r["outcome"].is("DONE") && tty == tty0, "%s with no terminal", op);
    }
    cdc_connected = false;
}

// ---- 16. stage 2: the epoch, counted where the state changes (4.3, ctl_state.h) ----
static void test_epoch_hooks() {
    fresh("epoch hooks");
    sim.nsect = 980u * 10 * 17;
    struct Step { const char *what; std::function<void()> act; bool moves; };
    uint8_t st;
    uint16_t buf[256];
    std::vector<Step> steps = {
        { "sync_to_config, a new geometry", [] { cur_cyls = 500; cur_heads = 4; cur_spt = 20; sync_to_config(); }, true },
        { "sync_to_config, the same", [] { sync_to_config(); }, false },
        { "ide_select_device, another device", [] { ide_select_device(0xB0); }, true },
        { "ide_select_device, the same", [] { ide_select_device(0xB0); }, false },
        { "back to the master", [] { ide_select_device(0xA0); }, true },
        { "a native Ctrl+G (no 91h since boot)", [&] { (void)ide_manual_chs_native(4, 20, &st); }, true },
        { "the TUI's set-up source", [] { ctl_setup_noted(CTL_SRC_TUI_CTRLG_NATIVE); }, true },
        { "a clear, then a clear of nothing", [] { ctl_setup_noted(CTL_SRC_NONE); ctl_setup_noted(CTL_SRC_NONE); }, true },
        { "a clear of nothing", [] { ctl_setup_noted(CTL_SRC_NONE); }, false },
        { "mount_nothing_to_mount (unmounted)", [] { mount_nothing_to_mount(); show_detect_result = false; }, true },
        { "detect_result_dismiss", [] { detect_result_dismiss(); }, true },
        { "Auto Detect's probe (native and the rule ended)", [] { (void)ide_probe_devices(); }, true },
        { "ide_set_geometry, the first 91h since boot", [] { (void)ide_set_geometry(4, 20); }, true },
        { "ide_set_geometry, a second 91h", [] { (void)ide_set_geometry(4, 20); }, false },
        { "a Ctrl+G (the no-IDENTIFY rule)", [&] { (void)ide_manual_chs(4, 20, &st); }, true },
        { "IDENTIFY refused after it", [&] { (void)ide_identify(buf); }, false },
        { "the probe (the rule ended)", [] { (void)ide_probe_devices(); }, true },
        { "an IDENTIFY that answers (Ctrl+G's pass-through bar ends)", [&] { (void)ide_identify(buf); }, true },
        { "an IDENTIFY that answers, nothing to end", [&] { (void)ide_identify(buf); }, false },
    };
    for (auto &s : steps) {
        uint32_t e0 = epoch_now();
        s.act();
        uint32_t e1 = epoch_now();                  // at once, before any core-0 pass
        CHECK(s.moves ? e1 > e0 : e1 == e0, "%s: epoch %u -> %u", s.what, (unsigned)e0, (unsigned)e1);
        pump(2);
        CHECK(c.epoch_missed == 0, "%s: a change no hook counted", s.what);
    }
    // try_auto_mount at boot: a mount and its source.
    fresh("epoch hooks: auto-mount");
    sim.nsect = 980u * 10 * 17;
    config.auto_mount = true; config.use_lba_mode = false; config.cyls = 980; config.heads = 10; config.spt = 17;
    hooked();
    uint32_t e0 = epoch_now();
    try_auto_mount();
    CHECK(is_mounted && epoch_now() > e0 && ctl_setup_src == CTL_SRC_AUTO_MOUNT && ctl_mount_seq == 1, "auto-mount");
    pump(2);
    // A change and its reversal inside one core-0 pass (one long USB
    // callback): stage 1's polling saw nothing; the hooks count both.
    fresh("epoch hooks: a change and its reversal in one pass");
    e0 = epoch_now();
    cur_cyls = 500; cur_heads = 4; cur_spt = 20; sync_to_config();
    cur_cyls = 0; cur_heads = 0; cur_spt = 0; sync_to_config();
    pump(2);
    J s = ask1(req(1, "status"));
    CHECK(s["result"]["epoch"].n == e0 + 2 && c.epoch_missed == 0, "epoch %llu after a change and its reversal, was %u",
          (unsigned long long)s["result"]["epoch"].n, (unsigned)e0);
    // The cross-check can fail: a store with no hook is counted, and moves
    // the epoch late rather than never.
    fresh("epoch hooks: the cross-check");
    e0 = epoch_now();
    config.dev_base = 0xB0;                         // a store no firmware path makes without its hook
    pump(2);
    CHECK(c.epoch_missed == 1 && epoch_now() == e0 + 1, "the cross-check missed a store: missed %u, epoch %u",
          (unsigned)c.epoch_missed, (unsigned)(epoch_now() - e0));
    c.epoch_missed = 0;                             // seen: not this test's failure
}

// Inside a Ctrl+G (a minute of it on a slow drive), each rule is counted as
// it is stored, not when the Ctrl+G returns: at its RESET- the no-IDENTIFY
// rule is already counted, and each later store once more (ctl_state.h).
static uint32_t pulse_epoch;
static bool pulse_seen, pulse_noid;
static void at_pulse(bool high) {
    if (high || pulse_seen) return;
    pulse_seen = true; pulse_epoch = ctl_epoch; pulse_noid = mchs_no_identify;
}
static void test_epoch_inside_ctrlg() {
    for (int native = 0; native < 2; native++) {
        fresh(native ? "epoch inside a native Ctrl+G" : "epoch inside a Ctrl+G");
        sim.nsect = 980u * 10 * 17;
        pulse_seen = false;
        sim_reset_hook = at_pulse;
        uint32_t e0 = epoch_now();
        uint8_t st;
        int r = native ? ide_manual_chs_native(10, 17, &st) : ide_manual_chs(10, 17, &st);
        sim_reset_hook = nullptr;
        CHECK(r == IDE_MCHS_OK && pulse_seen && pulse_noid && pulse_epoch > e0,
              "at RESET-: rule %d, epoch %u (was %u)", pulse_noid, (unsigned)pulse_epoch, (unsigned)e0);
        // After RESET-: the 91h mode stores idp_written, then manual_chs_active;
        // the native mode stores its declaration once.
        CHECK(epoch_now() == pulse_epoch + (native ? 1 : 2), "after RESET-: epoch +%u", (unsigned)(epoch_now() - pulse_epoch));
        pump(2);
    }
}

// status.setup through the TUI's own Auto Detect (the picker's first row).
static void test_setup_source() {
    fresh("setup.source from Auto Detect");
    sim.nsect = 980u * 10 * 17;
    rx.clear(); rxi = 0;
    rx.push_back({ mock_now_ns / 1000 + 100000000ull, '\r' });
    run_auto_detect();                              // the picker starts on the LBA row for an LBA drive
    J s = ask1(req(1, "status"));
    CHECK(s["result"]["setup"]["source"].is("TUI_DETECT_LBA") && s["result"]["geometry"]["mode"].is("LBA"), "after Auto Detect: %s",
          s["result"]["setup"]["source"].s.c_str());
    fresh("setup.source from Auto Detect, the first row");
    sim.nsect = 980u * 10 * 17;
    rx.clear(); rxi = 0;
    uint64_t t0 = mock_now_ns / 1000 + 100000000ull;
    for (char ch : std::string("\x1b[A\x1b[A\r")) { rx.push_back({ t0, ch }); t0 += 1000; }
    run_auto_detect();
    s = ask1(req(1, "status"));
    CHECK(s["result"]["setup"]["source"].is("TUI_DETECT_NORMAL") && s["result"]["geometry"]["cyls"].n == 980, "the first row: %s",
          s["result"]["setup"]["source"].s.c_str());
    // Auto Detect's probe clears the old set-up: an IDENTIFY that then fails
    // leaves none (NONE), not the one from before.
    fresh("setup.source: Auto Detect that fails");
    sim.nsect = 980u * 10 * 17;
    tui_setup_chs(980, 10, 17);
    sim.identify_err_drq = true;
    run_auto_detect();
    s = ask1(req(1, "status"));
    CHECK(s["result"]["setup"]["source"].is("NONE") && show_detect_result && force_detect, "after a failed Auto Detect: %s",
          s["result"]["setup"]["source"].s.c_str());
    show_detect_result = false; force_detect = false;
    // The picker's MANUAL row (Tab, then the three numbers): TUI_MANUAL.
    fresh("setup.source from the picker's MANUAL row");
    sim.nsect = 980u * 10 * 17;
    rx.clear(); rxi = 0;
    t0 = mock_now_ns / 1000 + 100000000ull;
    for (char ch : std::string("\t980\r10\r17\r")) { rx.push_back({ t0, ch }); t0 += 1000; }
    run_auto_detect();
    s = ask1(req(1, "status"));
    CHECK(s["result"]["setup"]["source"].is("TUI_MANUAL") && s["result"]["geometry"]["heads"].n == 10, "the MANUAL row: %s",
          s["result"]["setup"]["source"].s.c_str());
    // The Ctrl+G box's Y: TUI_CTRLG_IDP; a failure: NONE.
    fresh("setup.source from Ctrl+G");
    rx.clear(); rxi = 0;
    uint64_t t = mock_now_ns / 1000 + 1000;
    for (char ch : std::string("980\r10\r17\ry")) { rx.push_back({ t, ch }); t += 1000; }
    run_manual_chs();
    s = ask1(req(2, "status"));
    CHECK(s["result"]["setup"]["source"].is("TUI_CTRLG_IDP") && s["result"]["rules"]["no_identify_ctrlg"].b, "after Ctrl+G: %s",
          s["result"]["setup"]["source"].s.c_str());
    fresh("setup.source after a Ctrl+G that failed");
    sim.reject_idp = true;
    rx.clear(); rxi = 0;
    t = mock_now_ns / 1000 + 1000;
    for (char ch : std::string("980\r10\r17\ry")) { rx.push_back({ t, ch }); t += 1000; }
    ctl_setup_noted(CTL_SRC_TUI_DETECT_NORMAL);     // a set-up from before
    run_manual_chs();
    s = ask1(req(3, "status"));
    CHECK(s["result"]["setup"]["source"].is("NONE"), "after a failed Ctrl+G: %s", s["result"]["setup"]["source"].s.c_str());
}

#if ATABOY_BOARD_JUMPERS
// ---- 18. stage 3: JP1 LOCK and JP2 PERMIT (SPEC-v1 8; I-4, I-25; R-API-1 findings 5, 12, 13) ----
// board_jumpers.c against mock_pico.h's register file: the pads of GPIO30
// and GPIO31 as the RP2350 has them (reset value 0x116: pull-down, isolated,
// input off), set up by the SDK calls board_jumpers_init() makes, and the
// shunts (mock_jumper_fitted) tying a pin to GND. One core-0 pass a ms
// (pump); away() is core 0 inside a USB callback, with no pass at all.
static void jp1_fit(bool f) { mock_jumper_fitted[0] = f; }
static void jp2_fit(bool f) { mock_jumper_fitted[1] = f; }
static void hold2(bool f, uint32_t ms) { jp2_fit(f); pump(ms); }
static void away(uint32_t ms) { mock_now_ns += (uint64_t)ms * 1000000ull; }
static uint32_t t_ms() { return to_ms_since_boot(get_absolute_time()); }
// Pump to the moment `to` on the millisecond clock, when it is ahead by less
// than 10 minutes; otherwise fail and pump nothing (a moment already passed,
// under a mutant, would otherwise wrap to a 49-day pump and hang the run).
static bool pump_to(uint32_t to) {
    uint32_t d = to - t_ms();
    CHECK(d < 600000u, "pump_to: %u ms ahead (a moment already passed?)", (unsigned)d);
    if (d >= 600000u) return false;
    pump(d);
    return true;
}
static J status_now(uint32_t id) { return ask1(req(id, "status"), nullptr, 10); }
static std::string jstate(const J &s, const char *which) { return s["result"]["jumpers"][which].s; }

// The pads, and the start (8.1).
static void test_jp_pads_boot() {
    fresh("stage 3: the pads at power-up");
    CHECK(bj_pads_ok() && mock_jumper_let_low == 0, "pads ok %d, let go low %d times", bj_pads_ok(), mock_jumper_let_low);
    for (unsigned p = 30; p < 32; p++)
        CHECK((mock_pads.io[p] & 0x1CCu) == 0x48u && (mock_io.io[p].ctrl & 0x3F01Fu) == 5 && !(mock_gpio_oe & (1u << p)),
              "GPIO%u: pad %03X ctrl %05X", p, (unsigned)mock_pads.io[p], (unsigned)mock_io.io[p].ctrl);
    J s = status_now(1);
    CHECK(jstate(s, "jp1") == "OPEN" && jstate(s, "jp2") == "OPEN" && s["result"]["jumpers"]["mount_credit_ms_left"].n == 0 &&
          s["result"]["wp"]["forced_by"].arr.empty(), "status.jumpers %s %s", jstate(s, "jp1").c_str(), jstate(s, "jp2").c_str());
    J t = ask1(req(2, "selftest.status"));
    CHECK(t["result"]["jumpers"].is("OK"), "selftest.status.jumpers %s", t["result"]["jumpers"].s.c_str());
    // The first passes after power-up: JP1 UNKNOWN until 20 OPEN samples over
    // 10 ms, then OPEN, counted once; JP1 read open is no lock meanwhile.
    human_wp(false);
    pump(2);
    memset(&bj, 0, sizeof bj);
    mock_pads_reset();
    fingerprint(&c.fp); c.fp_epoch = ctl_epoch;     // JP1's state reset by the test itself, not a change to count
    uint32_t e0 = ctl_epoch;
    CHECK(jp1_locked() && !config.drive_write_protected && ctl_epoch == e0,
          "before board_jumpers_init: locked, nothing forced or counted");
    board_jumpers_init();
    CHECK(bj_jp1() == BJ_UNKNOWN && bj_jp2() == BJ_UNKNOWN && !jp1_locked() && !bj_tier_m_open(), "just set up");
    pump(10);
    CHECK(bj_jp1() == BJ_UNKNOWN && ctl_epoch == e0 && !config.drive_write_protected, "10 samples: still UNKNOWN");
    pump(15);
    CHECK(bj_jp1() == BJ_OPEN && bj_jp2() == BJ_OPEN && ctl_epoch == e0 + 1 && bj_tier_m_open() &&
          !config.drive_write_protected, "25 samples: jp1 %u jp2 %u epoch +%u", bj_jp1(), bj_jp2(), (unsigned)(ctl_epoch - e0));
    // Both ways for the order of set-up: the pull-up after gpio_init lets
    // the pin go to its pull-down for a moment (the mock counts it).
    mock_pads_reset();
    gpio_init(31); gpio_pull_up(31);
    CHECK(mock_jumper_let_low == 1, "gpio_init first: let go low %d times", mock_jumper_let_low);
    mock_jumper_let_low = 0;
}

// Every way a pad can be set up wrong (finding 13): each makes both jumpers
// UNREADABLE, which is LOCK (write protect forced on) and no PERMIT.
struct PadFault { const char *name; void (*make)(unsigned pin); };
static const PadFault pad_faults[] = {
    { "reset state (pull-down, isolated, input off, no function)",
      [](unsigned p) { mock_pads.io[p] = MOCK_PAD_RESET; mock_io.io[p].ctrl = MOCK_CTRL_RESET; } },
    { "pull-down, no pull-up", [](unsigned p) { mock_pads.io[p] = (mock_pads.io[p] & ~0x8u) | 0x4u; } },
    { "both pulls", [](unsigned p) { mock_pads.io[p] |= 0x4u; } },
    { "no pull", [](unsigned p) { mock_pads.io[p] &= ~0xCu; } },
    { "input off", [](unsigned p) { mock_pads.io[p] &= ~0x40u; } },
    { "isolated", [](unsigned p) { mock_pads.io[p] |= 0x100u; } },
    { "function PIO0", [](unsigned p) { mock_io.io[p].ctrl = (mock_io.io[p].ctrl & ~0x1Fu) | 6u; } },
    { "no function", [](unsigned p) { mock_io.io[p].ctrl |= 0x1Fu; } },
    { "OE override: invert", [](unsigned p) { mock_io.io[p].ctrl |= 1u << 14; } },
    { "OE override: disable", [](unsigned p) { mock_io.io[p].ctrl |= 2u << 14; } },
    { "OE override: enable", [](unsigned p) { mock_io.io[p].ctrl |= 3u << 14; } },
    { "OUT override: invert", [](unsigned p) { mock_io.io[p].ctrl |= 1u << 12; } },
    { "OUT override: low", [](unsigned p) { mock_io.io[p].ctrl |= 2u << 12; } },
    { "OUT override: high", [](unsigned p) { mock_io.io[p].ctrl |= 3u << 12; } },
    { "IN override: invert", [](unsigned p) { mock_io.io[p].ctrl |= 1u << 16; } },
    { "IN override: low", [](unsigned p) { mock_io.io[p].ctrl |= 2u << 16; } },
    { "IN override: high", [](unsigned p) { mock_io.io[p].ctrl |= 3u << 16; } },
    { "SIO output enabled, driving low", [](unsigned p) { mock_gpio_out &= ~(1u << p); mock_gpio_oe |= 1u << p; } },
    { "SIO output enabled, driving high", [](unsigned p) { mock_gpio_out |= 1u << p; mock_gpio_oe |= 1u << p; } },
};
static void test_jp_pad_faults() {
    int faked_fitted = 0, cases = 0;
    for (unsigned pin = 30; pin < 32; pin++) {
        for (const PadFault &f : pad_faults) {
            fresh("stage 3: a pad set up wrong");
            human_wp(false);
            pump(2);
            uint32_t e0 = ctl_epoch, ev0 = bj_fit_events();
            f.make(pin);
            cases++;
            if (!mock_pin_level(pin)) faked_fitted++;     // what a read without the read-back would take for a shunt
            bool live = jp1_locked();
            CHECK(live && config.drive_write_protected && bj_jp1() == BJ_UNREADABLE && ide_write_protected() && !bj_pads_ok() &&
                  !bj_tier_m_open(), "GPIO%u %s: live lock %d, wp %d, jp1 %u", pin, f.name, live, config.drive_write_protected, bj_jp1());
            CHECK(ctl_epoch == e0 + 1, "GPIO%u %s: epoch +%u (the forcing and the state, counted once)", pin, f.name,
                  (unsigned)(ctl_epoch - e0));
            // 400 ms of it, then: status, selftest, and no fit event (JP2 had
            // been open for over 200 ms: a pad that reads low would look like
            // a fit).
            pump(400);
            J s = status_now(1);
            J t = ask1(req(2, "selftest.status"), nullptr, 10);
            CHECK(jstate(s, "jp1") == "UNREADABLE" && jstate(s, "jp2") == "UNREADABLE" && s["result"]["wp"]["effective"].b &&
                  s["result"]["wp"]["config"].b && s["result"]["wp"]["forced_by"].arr.size() == 1 &&
                  s["result"]["wp"]["forced_by"].arr[0]->is("JP1") && t["result"]["jumpers"].is("PAD_CONFIG_MISMATCH") &&
                  bj_fit_events() == ev0, "GPIO%u %s: status %s/%s selftest %s events +%u", pin, f.name, jstate(s, "jp1").c_str(),
                  jstate(s, "jp2").c_str(), t["result"]["jumpers"].s.c_str(), (unsigned)(bj_fit_events() - ev0));
            // Set up right again: released after a debounced OPEN; write
            // protect stays on (8.2), and JP2 needs a new observed OPEN.
            mock_gpio_oe &= ~(1u << pin);
            gpio_pull_up(pin); gpio_init(pin);
            pump(5);
            CHECK(bj_jp1() == BJ_UNREADABLE && jp1_locked(), "GPIO%u %s: released before 20 samples", pin, f.name);
            pump(25);
            J s2 = status_now(3);
            CHECK(jstate(s2, "jp1") == "OPEN" && jstate(s2, "jp2") == "OPEN" && s2["result"]["wp"]["config"].b &&
                  s2["result"]["wp"]["forced_by"].arr.empty() && !jp1_locked(), "GPIO%u %s: after the repair %s/%s wp %d", pin,
                  f.name, jstate(s2, "jp1").c_str(), jstate(s2, "jp2").c_str(), (int)config.drive_write_protected);
        }
    }
    // The fixture drives the danger both ways: most of these read low on the
    // pin, which is "fitted" to a read that does not check the pad.
    CHECK(faked_fitted >= 20, "%d of %d wrong set-ups read low", faked_fitted, cases);
    printf("stage 3 pads: %d wrong set-ups (%zu per pin), %d of them read low (a fake shunt) without the read-back\n",
           cases, sizeof pad_faults / sizeof pad_faults[0], faked_fitted);
}

// JP1: LOCK (8.2, I-4, finding 12).
static void test_jp1_lock() {
    fresh("stage 3: JP1 fitted");
    human_wp(false);
    pump(2);
    uint32_t e0 = ctl_epoch;
    jp1_fit(true);
    pump(1);
    CHECK(bj_jp1() == BJ_FITTED && config.drive_write_protected && ctl_epoch == e0 + 1 && !bj_tier_m_open(),
          "one pass: jp1 %u wp %d epoch +%u", bj_jp1(), config.drive_write_protected, (unsigned)(ctl_epoch - e0));
    J s = status_now(1);
    const J &wp = s["result"]["wp"];
    CHECK(jstate(s, "jp1") == "FITTED" && wp["effective"].b && wp["config"].b && wp["forced_by"].arr.size() == 1 &&
          wp["forced_by"].arr[0]->is("JP1") && wp["last_change"].is("TUI"), "status while JP1 is fitted");
    // Tier S is not the jumpers' (8.2): wp.on still DONE (nothing to change).
    J w = ask1(req(2, "wp.on"));
    CHECK(w["outcome"].is("DONE") && !w["result"]["changed"].b && w["result"]["wp"]["forced_by"].arr.size() == 1, "wp.on under JP1");
    J h = ask1(req(3, "hello"));
    CHECK(h["result"]["caps"]["wp.on"]["state"].is("OPERATIONAL") && h["result"]["caps"]["unmount"]["state"].is("OPERATIONAL") &&
          h["result"]["caps"]["setup.ctrlg"]["state"].is("LOCKED") && h["result"]["caps"]["setup.ctrlg"]["why"].is("JP1_FITTED"),
          "caps under JP1");
    // Taken off: still locked until 20 OPEN samples over 10 ms (8.1), and
    // write protect stays on: the "Disabled" of before does not come back.
    e0 = ctl_epoch;
    jp1_fit(false);
    pump(5);
    CHECK(bj_jp1() == BJ_FITTED && jp1_locked() && ctl_epoch == e0, "released after 5 samples");
    pump(25);
    J s2 = status_now(4);
    CHECK(jstate(s2, "jp1") == "OPEN" && ctl_epoch == e0 + 1 && s2["result"]["wp"]["config"].b && s2["result"]["wp"]["effective"].b &&
          s2["result"]["wp"]["forced_by"].arr.empty() && !jp1_locked() && bj_tier_m_open(), "after removal: %s, wp %d, epoch +%u",
          jstate(s2, "jp1").c_str(), config.drive_write_protected, (unsigned)(ctl_epoch - e0));
    // A shunt that keeps touching (a low sample every 15 ms) is never released.
    jp1_fit(true); pump(1);
    for (int i = 0; i < 20; i++) { jp1_fit(false); pump(14); jp1_fit(true); pump(1); }
    jp1_fit(false);
    CHECK(bj_jp1() == BJ_FITTED && jp1_locked(), "released while it bounced");
    pump(25);
    CHECK(bj_jp1() == BJ_OPEN, "not released after it stopped");

    // Read live (finding 12): core 0 in a 20 s callback, no pass; the shunt
    // goes on; core 1 (the TUI) and the write check see it at once.
    fresh("stage 3: JP1 fitted while core 0 is away");
    human_wp(false);
    pump(2);
    e0 = ctl_epoch;
    away(5000);
    jp1_fit(true);
    mock_core_num = 1;
    bool locked = jp1_locked();
    mock_core_num = 0;
    CHECK(locked && config.drive_write_protected && bj_jp1() == BJ_FITTED && ctl_epoch == e0 + 1 && ide_write_protected(),
          "live read: locked %d wp %d", locked, config.drive_write_protected);
    away(15000);
    pump(2);
    CHECK(bj_jp1() == BJ_FITTED && ctl_epoch == e0 + 1, "after the callback");
    // A store of false racing the lock (the TUI's toggle between its check
    // and its store) is undone by the next read while still locked.
    jp1_fit(false);
    config.drive_write_protected = false;
    e0 = ctl_epoch;
    CHECK(jp1_locked() && config.drive_write_protected && ctl_epoch == e0 + 1, "a racing false not undone");
    pump(30);
}

// The debounce (SPEC-v1 8.1, README item 35): a level counts only after 20
// samples in a row over at least 10 ms (review C MED-4). Both bounds are driven
// on their own, with the other one satisfied: 19 samples over a long time and
// 20 over a short one are not enough, 20 over 10 ms is. Core 0's passes are
// placed on the millisecond clock, n of them, the first now and the last
// span_ms later (a 1 ms cadence would satisfy the time bound for any n).
static void samples_over(unsigned n, unsigned span_ms) {
    uint64_t t0 = mock_now_ns;
    for (unsigned i = 0; i < n; i++) {
        mock_now_ns = t0 + (uint64_t)(n > 1 ? (i * span_ms) / (n - 1) : 0) * 1000000ull;
        one_task();
    }
    mock_now_ns += 1000000ull;
}
static void test_debounce() {
    struct Case { unsigned n, span; bool settles; };
    const Case cases[] = { { 19, 40, false }, { 19, 10, false }, { 20, 9, false }, { 20, 10, true }, { 20, 19, true }, { 21, 30, true },
                           { 1, 0, false } };
    size_t ran = 0;
    for (const Case &k : cases) {
        char nm[80];
        // JP1's release: locked at once, released only by the debounced OPEN.
        snprintf(nm, sizeof nm, "stage 3: JP1 release after %u samples over %u ms", k.n, k.span);
        fresh(nm);
        pump(30);
        jp1_fit(true); pump(1);
        CHECK(bj_jp1() == BJ_FITTED, "JP1 fitted");
        jp1_fit(false);
        samples_over(k.n, k.span);
        CHECK(k.settles ? (bj_jp1() == BJ_OPEN && !jp1_locked()) : (bj_jp1() == BJ_FITTED && jp1_locked()),
              "JP1 %u samples over %u ms: state %u (released: %d, wanted %d)", k.n, k.span, bj_jp1(), bj_jp1() == BJ_OPEN, k.settles);
        // JP2's level (8.3), both levels: UNKNOWN after a gap until the run settles.
        for (int fitted = 0; fitted < 2; fitted++) {
            snprintf(nm, sizeof nm, "stage 3: JP2 %s level after %u samples over %u ms", fitted ? "FITTED" : "OPEN", k.n, k.span);
            fresh(nm);
            pump(30);
            jp2_fit(fitted != 0);
            away(60);                                       // a gap: JP2 is UNKNOWN again, its run starts over
            samples_over(k.n, k.span);
            uint8_t want = k.settles ? (fitted ? BJ_FITTED : BJ_OPEN) : BJ_UNKNOWN;
            CHECK(bj_jp2() == want, "JP2 %s, %u samples over %u ms: state %u, wanted %u", fitted ? "fitted" : "open", k.n, k.span,
                  bj_jp2(), want);
        }
        ran++;
    }
    CHECK(ran == sizeof cases / sizeof cases[0] && ran >= 6, "the debounce table ran %zu cases", ran);
    // A level change with NO gap (review delta-B LOW-6, D3): every case above
    // starts its run from n = 0 (after a gap), so a run count that is carried
    // over a level change instead of restarting was invisible here. JP2 settled
    // at one level for 20 samples, then the other for 19 samples over 15 ms: the
    // 19 are a fresh run of 19, not 20 + 19, and settle nothing.
    for (int fitted = 0; fitted < 2; fitted++) {
        char nm[96];
        snprintf(nm, sizeof nm, "stage 3: JP2 %s after 20 settled samples of the other level", fitted ? "FITTED" : "OPEN");
        fresh(nm);
        pump(30);
        jp2_fit(fitted == 0);
        away(60);                                           // a gap: its run starts from nothing
        samples_over(20, 19);
        uint8_t first = fitted ? BJ_OPEN : BJ_FITTED;
        CHECK(bj_jp2() == first, "JP2 first level settled: state %u, wanted %u", bj_jp2(), first);
        jp2_fit(fitted != 0);                               // the change, no gap
        samples_over(19, 15);
        CHECK(bj_jp2() == first, "JP2 %s after 19 samples over 15 ms following a settled run of the other: state %u, wanted still %u",
              fitted ? "fitted" : "open", bj_jp2(), first);
        samples_over(20, 19);                               // and a full run of 20 does settle it
        CHECK(bj_jp2() == (fitted ? BJ_FITTED : BJ_OPEN), "JP2 after a fresh 20: state %u", bj_jp2());
    }
}

// JP2: the fit event (8.3, I-25, finding 5), timelines both ways.
static uint32_t ev_base;
static uint32_t evs() { return bj_fit_events() - ev_base; }
static void jp2_start(const char *name, bool fitted_at_power_up = false) {
    power_up_fitted[1] = fitted_at_power_up;
    fresh(name);
    power_up_fitted[1] = false;
    ev_base = bj_fit_events();
}
static void test_jp2_events() {
    // A clean act: OPEN 1 s, then FITTED; the event at edge + 200 ms, once.
    jp2_start("stage 3: JP2, a clean act");
    hold2(false, 1000);
    uint32_t t0 = t_ms();
    hold2(true, 200);
    CHECK(evs() == 0, "event before the FITTED 200 ms");
    pump(1);
    CHECK(evs() == 1 && bj.edge == t0 + 1, "one event, edge %u (fitted at %u)", (unsigned)bj.edge, (unsigned)t0);
    hold2(true, 10000);
    CHECK(evs() == 1, "a shunt left on made %u events", evs());
    J s = status_now(1);
    CHECK(jstate(s, "jp2") == "FITTED" && jstate(s, "jp1") == "OPEN", "status.jumpers %s", jstate(s, "jp2").c_str());

    // The OPEN before it: 199 ms is not enough, 200 is (from a standing fit).
    for (uint32_t open_ms : { 199u, 200u }) {
        jp2_start("stage 3: JP2, the OPEN's length", true);
        hold2(true, 1000);
        CHECK(evs() == 0 && bj_jp2() == BJ_FITTED, "a shunt on at power-up: %u events", evs());
        hold2(false, open_ms);
        hold2(true, 400);
        CHECK(evs() == (open_ms >= 200 ? 1u : 0u), "OPEN %u ms: %u events", open_ms, evs());
    }
    // The FITTED: 200 passes are not enough, 201 (edge + 200 ms) are.
    for (uint32_t fit_ms : { 200u, 201u }) {
        jp2_start("stage 3: JP2, the FITTED's length");
        hold2(false, 1000);
        hold2(true, fit_ms);
        hold2(false, 1000);
        CHECK(evs() == (fit_ms > 200 ? 1u : 0u), "FITTED %u ms: %u events", fit_ms, evs());
    }
    // Bounce as the shunt goes on: the debounced OPEN goes on through it and
    // the edge is the first sample of the steady FITTED run.
    jp2_start("stage 3: JP2, bounce going on");
    hold2(false, 1000);
    for (int i = 0; i < 3; i++) { hold2(true, 2); hold2(false, 2); }
    uint32_t t1 = t_ms();
    hold2(true, 400);
    CHECK(evs() == 1 && bj.edge == t1 + 1, "bounce: %u events, edge %u, steady from %u", evs(), (unsigned)bj.edge, (unsigned)t1 + 1);
    // A FITTED blip (5 ms) in a long OPEN: not a fit, and the OPEN stands.
    jp2_start("stage 3: JP2, a blip while open");
    hold2(false, 1000);
    hold2(true, 5);
    hold2(false, 100);
    hold2(true, 400);
    CHECK(evs() == 1, "a blip then a fit: %u events", evs());
    // An OPEN sample inside the FITTED 200 ms: no event; the human refits.
    jp2_start("stage 3: JP2, a gap in the FITTED");
    hold2(false, 1000);
    hold2(true, 100);
    hold2(false, 1);
    hold2(true, 1000);
    CHECK(evs() == 0, "a break in the FITTED made %u events", evs());
    hold2(false, 300);
    hold2(true, 300);
    CHECK(evs() == 1, "the refit made %u events", evs());
    // Fit then remove inside the 200 ms, many times: nothing.
    jp2_start("stage 3: JP2, taps");
    hold2(false, 1000);
    for (int i = 0; i < 20; i++) { hold2(true, 150); hold2(false, 250); }
    CHECK(evs() == 0, "taps made %u events", evs());
    // Three proper acts: three events.
    for (int i = 0; i < 3; i++) { hold2(false, 300); hold2(true, 300); }
    CHECK(evs() == 3, "three acts made %u events", evs());

    // Gaps between passes (a USB callback): 49 ms away keeps the OPEN, 50
    // (a 51 ms gap between passes) forgets it.
    for (uint32_t gap : { 49u, 50u }) {
        jp2_start("stage 3: JP2, a gap");
        hold2(false, 1000);
        jp2_fit(true);
        away(gap);
        hold2(true, 400);
        CHECK(evs() == (gap < 50 ? 1u : 0u), "core 0 away %u ms, fitted meanwhile: %u events", gap, evs());
    }
    // A fit during a 20 s MSC callback: never counted; after it, a refit is.
    jp2_start("stage 3: JP2, fitted inside a 20 s callback");
    hold2(false, 1000);
    uint32_t gaps0 = bj.gaps;
    away(5000); jp2_fit(true); away(15000);
    pump(1);
    CHECK(bj.gaps == gaps0 + 1 && bj_jp2() == BJ_UNKNOWN, "after the callback: gaps +%u, jp2 %u", (unsigned)(bj.gaps - gaps0), bj_jp2());
    J s3 = status_now(2);
    CHECK(jstate(s3, "jp2") == "UNKNOWN", "status.jumpers.jp2 %s right after the gap", jstate(s3, "jp2").c_str());
    hold2(true, 2000);
    CHECK(evs() == 0, "the act inside the callback made %u events", evs());
    hold2(false, 300); hold2(true, 300);
    CHECK(evs() == 1, "the refit after it made %u events", evs());
    // Open and fitted both inside one callback: nothing either.
    jp2_start("stage 3: JP2, a whole act inside a callback");
    hold2(false, 1000);
    away(3000); jp2_fit(true); away(3000);
    hold2(true, 400);
    CHECK(evs() == 0, "%u events", evs());

    // The pads unreadable for 5 ms in the OPEN: the OPEN is forgotten.
    jp2_start("stage 3: JP2, pads unreadable in the OPEN");
    hold2(false, 1000);
    mock_pads.io[31] |= 0x4u;                       // both pulls: UNREADABLE
    pump(5);
    gpio_pull_up(31);
    hold2(false, 50);
    hold2(true, 400);
    CHECK(evs() == 0, "OPEN before the pads went wrong counted: %u events", evs());
    hold2(false, 300); hold2(true, 300);
    CHECK(evs() == 1, "%u events after a new OPEN", evs());
    // The pad at its reset state (reads low) or driven low, after a long
    // OPEN (findings 13, X7 and X54): never an event.
    jp2_start("stage 3: JP2, a pad at reset reads low");
    hold2(false, 1000);
    mock_pads.io[31] = MOCK_PAD_RESET; mock_io.io[31].ctrl = MOCK_CTRL_RESET;
    CHECK(!mock_pin_level(31), "the reset state does not read low");
    pump(1000);
    CHECK(evs() == 0, "a pad at its reset state made %u events", evs());
    jp2_start("stage 3: JP2, GPIO31 driven low");
    hold2(false, 1000);
    mock_gpio_out &= ~(1u << 31); mock_gpio_oe |= 1u << 31;
    CHECK(!mock_pin_level(31), "driven low does not read low");
    pump(1000);
    mock_gpio_oe &= ~(1u << 31);
    CHECK(evs() == 0, "an output driving low made %u events", evs());

    // What bj_task hands back (stage 4 compares its edge with the waiting
    // request's arrival): the edge, the first FITTED sample, and the pass
    // that ended the 200 ms; only from that one pass. Driven by hand, one
    // pass a ms, with no ctl_task in between.
    jp2_start("stage 3: JP2, the event's record");
    bj_event_t ev = { 0, 0 };
    int reported = 0;
    uint32_t t = t_ms(), fit_at = 0, rep_edge = 0, rep_at = 0, rep_pass = 0;
    for (uint32_t i = 1; i <= 1500; i++) {
        if (i == 600) { jp2_fit(true); fit_at = t + i; }
        if (bj_task(t + i, &ev)) { reported++; rep_edge = ev.edge_ms; rep_at = ev.at_ms; rep_pass = t + i; }
    }
    mock_now_ns = (uint64_t)(t + 1500) * 1000000ull;
    CHECK(reported == 1 && rep_edge == fit_at && rep_at == fit_at + 200 && rep_pass == rep_at,
          "event record: %d reported, edge %u (fitted %u), at %u, pass %u", reported, (unsigned)rep_edge,
          (unsigned)fit_at, (unsigned)rep_at, (unsigned)rep_pass);
}

// A shunt fitted and taken off between two of core 0's passes, seen only by
// a live read (the TUI's, on core 1): the release still needs 20 OPEN
// samples over 10 ms after it (R-API-1 finding 12).
static void test_jp1_between_passes() {
    fresh("stage 3: JP1 seen between two passes");
    pump(30);
    jp1_fit(true);
    mock_core_num = 1;
    bool locked = jp1_locked();
    mock_core_num = 0;
    jp1_fit(false);
    CHECK(locked && bj_jp1() == BJ_FITTED, "the live read: %d", locked);
    pump(5);
    CHECK(bj_jp1() == BJ_FITTED && jp1_locked(), "released 5 samples after a live-only lock");
    pump(25);
    CHECK(bj_jp1() == BJ_OPEN && !jp1_locked(), "not released after 30 samples");

    // A gap (core 0 away) inside the OPEN run: the samples before it do not
    // count toward the release (the shunt may have been back meanwhile).
    fresh("stage 3: JP1 released across a gap");
    pump(30);
    jp1_fit(true); pump(1); jp1_fit(false);
    pump(15);
    away(60);
    pump(5);
    CHECK(bj_jp1() == BJ_FITTED && jp1_locked(), "released on samples from before a gap");
    pump(20);
    CHECK(bj_jp1() == BJ_OPEN, "not released 25 samples after the gap");

    // A pad set up wrong between the read-back and the read of the level:
    // the read-back after the read catches it (finding 13).
    fresh("stage 3: a pad set up wrong during the read");
    human_wp(false);
    pump(2);
    struct Race { static void at_read() {
        mock_pads.io[31] = MOCK_PAD_RESET; mock_io.io[31].ctrl = MOCK_CTRL_RESET; mock_on_gpio_in = nullptr;
    } };
    mock_on_gpio_in = Race::at_read;
    bool locked2 = jp1_locked();
    mock_on_gpio_in = nullptr;
    CHECK(locked2 && bj_jp1() == BJ_UNREADABLE && config.drive_write_protected,
          "a pad changed during the read: locked %d, jp1 %u", locked2, bj_jp1());
    gpio_pull_up(31); gpio_init(31);
    pump(30);
}

// An event with no lease (stage 3 has none): counted, and nothing else
// happens (8.3: an event with no request waiting approves nothing); JP2's
// level is not part of the epoch (4.3).
static void test_jp2_changes_nothing() {
    jp2_start("stage 3: JP2 changes nothing");
    hold2(false, 1000);
    std::string d0 = state2();
    uint32_t e0 = ctl_epoch;
    J before = status_now(1);
    hold2(true, 400);
    CHECK(evs() == 1 && ctl_epoch == e0 && state2() == d0, "an event moved the state (epoch +%u)", (unsigned)(ctl_epoch - e0));
    J after = status_now(2);
    CHECK(after["result"]["presence"]["op"].is("NONE") && !after["result"]["presence"]["waiting"].b &&
          !after["result"]["lease"]["held"].b && jstate(after, "jp2") == "FITTED" && jstate(before, "jp2") == "OPEN" &&
          after["result"]["epoch"].n == before["result"]["epoch"].n, "status after an event");
    hold2(false, 400);
    CHECK(ctl_epoch == e0 && state2() == d0, "JP2 taken off moved the state");
    // The request's expect_epoch taken before the act still holds after it.
    J w = ask1(req(3, "wp.on", eargs("expect_epoch", e0).c_str()));
    CHECK(w["outcome"].is("DONE"), "expect_epoch from before the act: %s", w["outcome"].s.c_str());
}

// Tier M's capability (3.4) from the jumpers: the function, and hello with
// stand-ins for stage 4's two Tier M ops.
// Tier M's capability (3.4) from the jumpers: the function stage 4's built
// Tier M ops will take their state from, and hello as stage 3 builds it
// (Tier M not built: ABSENT whatever the jumpers; Tier R and S OPERATIONAL
// whatever the jumpers). hello with a built Tier M op is stage 4's test.
static void test_tier_m_caps() {
    fresh("stage 3: Tier M's capability");
    struct { uint8_t jp1; bool lease; uint8_t st; const char *why; } t[] = {
        { CTL_JP1_OPEN, false, CTL_CAP_LOCKED, "NO_LEASE" }, { CTL_JP1_OPEN, true, CTL_CAP_OPERATIONAL, nullptr },
        { CTL_JP1_FITTED, false, CTL_CAP_LOCKED, "JP1_FITTED" }, { CTL_JP1_FITTED, true, CTL_CAP_LOCKED, "JP1_FITTED" },
        { CTL_JP1_UNREADABLE, true, CTL_CAP_LOCKED, "JUMPERS_UNREADABLE" }, { CTL_JP1_UNKNOWN, true, CTL_CAP_LOCKED, "JUMPERS_UNREADABLE" },
        { CTL_JP1_UNREADABLE, false, CTL_CAP_LOCKED, "JUMPERS_UNREADABLE" }, { CTL_JP1_UNKNOWN, false, CTL_CAP_LOCKED, "JUMPERS_UNREADABLE" },
    };
    for (auto &x : t) {
        uint8_t st = 0xFF; const char *why = "x";
        ctl_tier_m_cap(x.jp1, x.lease, &st, &why);
        CHECK(st == x.st && ((!why && !x.why) || (why && x.why && !strcmp(why, x.why))), "jp1 %u lease %d: %u %s",
              x.jp1, x.lease, st, why ? why : "-");
    }
    // hello, JP1 open, fitted, and a pad wrong: Tier R, S and the session ops
    // OPERATIONAL each time; Tier M LOCKED, why from the jumpers first.
    static const char *const tier_m_why[3] = { "NO_LEASE", "JP1_FITTED", "JUMPERS_UNREADABLE" };
    for (int k = 0; k < 3; k++) {
        if (k == 1) { jp1_fit(true); pump(1); }
        if (k == 2) { jp1_fit(false); pump(30); mock_pads.io[30] &= ~0x40u; pump(1); }
        J h = ask1(req(1 + k, "hello"), nullptr, 20);
        const J &caps = h["result"]["caps"];
        for (int op = 0; op < CTL_OP_COUNT; op++) {
            bool rs = ctl_ops[op].tier != CTL_TIER_M;
            CHECK(rs ? caps[ctl_ops[op].name]["state"].is("OPERATIONAL") && !caps[ctl_ops[op].name].get("why")
                     : caps[ctl_ops[op].name]["state"].is("LOCKED") && caps[ctl_ops[op].name]["why"].is(tier_m_why[k]),
                  "hello (jumpers %d): %s %s %s", k, ctl_ops[op].name, caps[ctl_ops[op].name]["state"].s.c_str(),
                  caps[ctl_ops[op].name]["why"].s.c_str());
        }
    }
}
#endif

// ---- 19. stage 4: the lease and Tier M (SPEC-v1 4.13, 4.14, 6; I-7, I-26 to I-28) ----
// Requests with a token, the lease, an act (JP2 OPEN 250 ms then FITTED 250
// ms: one fit event), and what reached the simulated drive.
static std::string reqt(uint32_t id, const char *op, const std::string &token, const std::string &args) {
    char b[1100];
    snprintf(b, sizeof b, "{\"v\":1,\"id\":%u,\"op\":\"%s\",\"token\":\"%s\",\"args\":%s}\n", id, op, token.c_str(), args.c_str());
    return b;
}
static std::string lease_token;
// A session in these tests is one with a terminal open on the setup screen's
// port, the TUI on the main menu: a guarded request waits for its act only
// then (owner ruling 2026-09-29, review A MED-3 (2)). The tests of "no
// terminal" close it afterwards.
static J open_lease(uint32_t id, unsigned idle_s = 300, const char *client = "test") {
    cdc_connected = true;
    char a[96];
    snprintf(a, sizeof a, "{\"client\":\"%s\",\"tui_lock\":false,\"idle_s\":%u}", client, idle_s);
    J r = ask1(req(id, "session.open", a));
    if (r["outcome"].is("DONE")) lease_token = r["result"]["token"].s;
    return r;
}
static std::string ctrlg_args(unsigned c = 980, unsigned h = 10, unsigned s = 17, const char *mode = "idp",
                              const char *dev = "master", const char *extra = "") {
    char b[256];
    snprintf(b, sizeof b, "{\"cyls\":%u,\"heads\":%u,\"spt\":%u,\"mode\":\"%s\",\"device\":\"%s\",\"drive_power_cycled\":true%s}",
             c, h, s, mode, dev, extra);
    return b;
}
static std::string mount_args(const char *src = "API_CTRLG_IDP", unsigned c = 980, unsigned h = 10, unsigned s = 17,
                              const char *extra = "") {
    char b[256];
    snprintf(b, sizeof b, "{\"expect\":{\"source\":\"%s\",\"mode\":\"CHS\",\"device\":\"MASTER\",\"cyls\":%u,\"heads\":%u,\"spt\":%u}%s}",
             src, c, h, s, extra);
    return b;
}
// The act: JP2 open 250 ms, then fitted 250 ms (an event 200 ms after the edge).
#if ATABOY_BOARD_JUMPERS
static void act() { hold2(false, 250); hold2(true, 250); hold2(false, 30); }
#endif
// A line sent, no answer expected yet (a Tier M request waits).
static void send_only(const std::string &line, uint32_t ms = 5) { host_send(line); pump(ms); }
static std::string sha_hex(const std::string &line) {
    std::string l = line;
    while (!l.empty() && (l.back() == '\n' || l.back() == '\r')) l.pop_back();
    uint8_t dg[32];
    ctl_sha256(l.data(), l.size(), dg);
    return hex(dg, 32);
}
static J one_answer(bool *ok = nullptr) {
    std::vector<std::string> r = host_lines();
    J j;
    std::string why;
    bool good = r.size() == 1 && parse_response(r[0], j, &why) && envelope_problem(j).empty();
    if (!good) printf("  (answer problem: %zu lines %s)\n", r.size(), why.c_str());
    if (ok) *ok = good;
    return j;
}
static J status4(uint32_t id) { return ask1(req(id, "status"), nullptr, 10); }
static void power_drive() { sim.nsect = 980u * 10 * 17; }

// detach needs the lease token while someone else's lease is held and a
// set-up is in place (owner ruling 2026-09-29 on review A MED-2, option "A's
// minimum"); unmount stays tokenless, the emergency stop. Both builds.
static void test_detach_lease() {
    fresh("stage 4: detach under a held lease");
    tui_setup_chs(980, 10, 17);
    open_lease(1);
    const std::string T = lease_token;
    CHECK(c.lease.held && ctl_setup_src != CTL_SRC_NONE, "the lease and the set-up are in place");
    std::string d0 = state2();
    // Tokenless: refused, the lease holder's name and idle time given, nothing changed, nothing queued.
    J r = ask1(req(2, "detach", named(0).c_str()));
    CHECK(is_refusal(r, "E_LEASE_HELD") && r["retry"].is("SAFE") && r["err"]["client"].is("test") &&
          r["err"]["idle_ms_left"].n > 0 && r["err"]["field"].is("token") && r["err"]["msg"].s.size() > 0,
          "tokenless detach: %s outcome %s", r["err"]["code"].s.c_str(), r["outcome"].s.c_str());
    CHECK(state2() == d0 && config.cyls == 980 && ctl_setup_src == CTL_SRC_TUI_DETECT_NORMAL && ctl_job.state == CTL_JOB_EMPTY &&
          c.lease.held, "the refused detach changed something");
    // The refusal is what the replay table keeps.
    std::vector<std::string> rr = ask(req(2, "detach", named(0).c_str()));
    J rj;
    CHECK(rr.size() == 1 && parse_response(rr[0], rj) && rj["replayed"].b && rj["err"]["code"].is("E_LEASE_HELD") &&
          config.cyls == 980, "the resent refusal");
    // A wrong token, a token one digit short, and a token where unmount takes none.
    std::string bad = T;
    bad[31] = bad[31] == '0' ? '1' : '0';
    r = ask1(reqt(3, "detach", bad, named(0)));
    CHECK(is_refusal(r, "E_LEASE_INVALID") && config.cyls == 980, "a wrong token: %s", r["err"]["code"].s.c_str());
    r = ask1(reqt(4, "detach", T.substr(0, 31), named(0)));
    CHECK(is_refusal(r, "E_LEASE_INVALID") && config.cyls == 980, "a short token: %s", r["err"]["code"].s.c_str());
    r = ask1(reqt(5, "unmount", T, named(0)));
    CHECK(is_refusal(r, "E_SCHEMA") && r["err"]["field"].is("token"), "unmount takes no token: %s", r["err"]["code"].s.c_str());
    // The token still names the mount: a stale one is E_STALE, not a clear.
    r = ask1(reqt(6, "detach", T, named(9)));
    CHECK(is_refusal(r, "E_STALE") && config.cyls == 980, "a right token, a wrong mount: %s", r["err"]["code"].s.c_str());
    // The holder's own detach, with the token.
    r = ask1(reqt(7, "detach", T, named(0)));
    CHECK(r["outcome"].is("DONE") && r["result"]["detached"].b && config.cyls == 0 && ctl_setup_src == CTL_SRC_NONE,
          "detach with the token: %s %s", r["outcome"].s.c_str(), r["err"]["code"].s.c_str());
    // unmount stays tokenless (the emergency stop), lease held, mounted.
    fresh("stage 4: unmount under a held lease");
    tui_setup_chs(980, 10, 17);
    tui_mount();
    open_lease(1);
    r = ask1(req(2, "unmount", named(1).c_str()));
    CHECK(r["outcome"].is("DONE") && !is_mounted && c.lease.held, "tokenless unmount: %s %s", r["outcome"].s.c_str(),
          r["err"]["code"].s.c_str());
    // A mounted set-up is a set-up in place: detach is refused tokenless there too.
    fresh("stage 4: detach, mounted, under a held lease");
    tui_setup_chs(980, 10, 17);
    tui_mount();
    open_lease(1);
    r = ask1(req(2, "detach", named(1).c_str()));
    CHECK(is_refusal(r, "E_LEASE_HELD") && is_mounted && config.cyls == 980, "mounted, tokenless detach: %s", r["err"]["code"].s.c_str());
    // No set-up in place: nothing for detach to clear, so no token is asked for.
    fresh("stage 4: detach, lease held, no set-up");
    open_lease(1);
    CHECK(c.lease.held && ctl_setup_src == CTL_SRC_NONE, "the lease, no set-up");
    r = ask1(req(2, "detach", named(0).c_str()));
    CHECK(r["outcome"].is("DONE"), "detach with nothing set up: %s %s", r["outcome"].s.c_str(), r["err"]["code"].s.c_str());
    // No lease: as before, tokenless. A token when none is held is E_NO_LEASE; the ended lease's is E_LEASE_REVOKED.
    fresh("stage 4: detach, no lease");
    tui_setup_chs(980, 10, 17);
    r = ask1(req(2, "detach", named(0).c_str()));
    CHECK(r["outcome"].is("DONE") && config.cyls == 0, "tokenless detach, no lease: %s %s", r["outcome"].s.c_str(), r["err"]["code"].s.c_str());
    tui_setup_chs(980, 10, 17);
    open_lease(3);
    const std::string T2 = lease_token;
    ask1(reqt(4, "session.close", T2, "{}"));
    CHECK(!c.lease.held, "the lease is closed");
    r = ask1(reqt(5, "detach", T2, named(0)));
    CHECK(is_refusal(r, "E_LEASE_REVOKED") && config.cyls == 980, "the ended lease's token: %s", r["err"]["code"].s.c_str());
    r = ask1(reqt(6, "detach", std::string(32, 'a'), named(0)));
    CHECK(is_refusal(r, "E_NO_LEASE") && config.cyls == 980, "a token, no lease: %s", r["err"]["code"].s.c_str());
    r = ask1(req(7, "detach", named(0).c_str()));
    CHECK(r["outcome"].is("DONE") && config.cyls == 0, "tokenless detach after the lease ended: %s", r["err"]["code"].s.c_str());
}

#if ATABOY_BOARD_JUMPERS
// The lease (4.13, 6.1), both ways.
static void test_lease() {
    fresh("stage 4: the lease");
    uint32_t e0 = ctl_epoch;
    J r = open_lease(1, 300, "palimpsest");
    CHECK(r["outcome"].is("DONE") && all_keys(r["result"], { "token", "idle_s", "tui_lock" }) && lease_token.size() == 32 &&
          lease_token.find_first_not_of("0123456789abcdef") == std::string::npos && r["result"]["idle_s"].n == 300 &&
          !r["result"]["tui_lock"].b && ctl_epoch == e0 + 1, "session.open: %s", r["outcome"].s.c_str());
    J s = status4(2);
    CHECK(s["result"]["lease"]["held"].b && s["result"]["lease"]["client"].is("palimpsest") &&
          s["result"]["lease"]["idle_ms_left"].n > 290000 && !s["result"]["lease"]["tui_lock"].b, "status.lease");
    std::vector<std::string> raw = ask(req(22, "status"), 10);
    CHECK(raw.size() == 1 && raw[0].find(lease_token) == std::string::npos, "status shows the token");
    // A second client: E_LEASE_HELD with who and how long, never the token.
    J r2 = open_lease(3, 60, "other");
    CHECK(is_refusal(r2, "E_LEASE_HELD") && r2["err"]["client"].is("palimpsest") && r2["err"]["idle_ms_left"].t == J::INT &&
          !r2.get("result"), "a second open: %s", r2["err"]["code"].s.c_str());
    // tui_lock is not built in this stage: refused, nothing opened.
    J r3 = ask1(req(4, "session.open", "{\"client\":\"x\",\"tui_lock\":true,\"idle_s\":60}"));
    CHECK(is_refusal(r3, "E_PRECONDITION") && r3["err"]["why"].is("TUI_LOCK_NOT_BUILT"), "tui_lock true: %s",
          r3["err"]["code"].s.c_str());
    // session.close: its token only.
    std::string good = lease_token;
    J c1 = ask1(req(5, "session.close"));
    CHECK(is_refusal(c1, "E_SCHEMA") && c1["err"]["field"].is("token"), "close without a token");
    J c2 = ask1(reqt(6, "session.close", "00112233445566778899aabbccddeeff", "{}"));
    CHECK(is_refusal(c2, "E_LEASE_INVALID"), "close with another token: %s", c2["err"]["code"].s.c_str());
    e0 = ctl_epoch;
    J c3 = ask1(reqt(7, "session.close", good, "{}"));
    CHECK(c3["outcome"].is("DONE") && c3["result"]["closed"].b && !c.lease.held && ctl_epoch == e0 + 1, "close");
    J c4 = ask1(reqt(8, "session.close", good, "{}"));
    CHECK(is_refusal(c4, "E_LEASE_REVOKED") && c4["err"]["by"].is("CLOSED"), "the old token: %s", c4["err"]["code"].s.c_str());
    J c5 = ask1(reqt(9, "session.close", "00112233445566778899aabbccddeeff", "{}"));
    CHECK(is_refusal(c5, "E_NO_LEASE"), "close with no lease: %s", c5["err"]["code"].s.c_str());
    // A new lease has another token.
    open_lease(10);
    CHECK(lease_token != good, "the same token twice");
    // Idle: 30 s with nothing, revoked (by IDLE); a request with the token renews.
    fresh("stage 4: the lease's idle timer");
    open_lease(1, 30);
    pump(20000);
    J k = ask1(reqt(2, "setup.ctrlg", lease_token, ctrlg_args(980, 10, 17, "idp", "slave")));   // refused, but renews
    CHECK(is_refusal(k, "E_PRECONDITION") && c.lease.held, "a refused request with the token: %s", k["err"]["code"].s.c_str());
    pump(20000);
    CHECK(c.lease.held, "renewed lease ended after 20 s");
    pump(11000);
    J k2 = ask1(reqt(3, "session.close", lease_token, "{}"));
    CHECK(!c.lease.held && is_refusal(k2, "E_LEASE_REVOKED") && k2["err"]["by"].is("IDLE"), "idle: %s %s",
          k2["err"]["code"].s.c_str(), k2["err"]["by"].s.c_str());
    // The port closing and a USB reset end it too.
    fresh("stage 4: the lease and the port");
    open_lease(1);
    std::string t1 = lease_token;
    tud_cdc_line_state_cb(1, false, false); pump(3); tud_cdc_line_state_cb(1, true, false); pump(3);
    J p1 = ask1(reqt(2, "session.close", t1, "{}"));
    CHECK(is_refusal(p1, "E_LEASE_REVOKED") && p1["err"]["by"].is("PORT_CLOSED"), "DTR drop: %s", p1["err"]["by"].s.c_str());
    open_lease(3);
    std::string t2 = lease_token;
    ctl_on_usb_mount(); pump(3);
    J p2 = ask1(reqt(4, "session.close", t2, "{}"));
    CHECK(is_refusal(p2, "E_LEASE_REVOKED") && p2["err"]["by"].is("USB_RESET"), "USB reset: %s", p2["err"]["by"].s.c_str());
    // hello: Tier M OPERATIONAL while a lease is held (3.4).
    open_lease(5);
    J h = ask1(req(6, "hello"), nullptr, 20);
    CHECK(h["result"]["caps"]["setup.ctrlg"]["state"].is("OPERATIONAL") && h["result"]["caps"]["mount"]["state"].is("OPERATIONAL"),
          "hello with a lease: %s", h["result"]["caps"]["mount"]["state"].s.c_str());
}

// The lease token is 128 bits, both halves, in order (review C HIGH-1). The
// mock's RNG normally counts up, which makes "the token differs" true of any
// token at all; here it is pinned to a known sequence instead, so a token built
// from one half twice, or from 32 bits, or with a digit out of place, is wrong.
static std::string token_of(uint64_t r0, uint64_t r1) {
    char b[40];
    snprintf(b, sizeof b, "%016llx%016llx", (unsigned long long)r0, (unsigned long long)r1);
    return b;
}
static void test_token_entropy() {
    fresh("stage 4: the token is the RNG's 128 bits, in order");
    // B has no structural relation to A (review delta-B LOW-8): with B == ~A and
    // B ending in 0 a token built from r1 = ~r0, or ending in a constant digit,
    // passed this first check and only the nibble loop below caught it.
    const uint64_t A = 0x0123456789abcdefull, B = 0x9e3779b97f4a7c15ull;
    uint32_t id = 1;
    rand_script = { A, B };
    J r = open_lease(id++);
    CHECK(r["outcome"].is("DONE") && lease_token == "0123456789abcdef9e3779b97f4a7c15" && rand_script.empty(),
          "pinned draws A then B gave token %s", lease_token.c_str());
    // Every one of the 32 digits follows its own nibble: flip that nibble in
    // the draw, and exactly that digit of the token changes.
    int wrong = 0;
    for (int i = 0; i < 32; i++) {
        J cl = ask1(reqt(id++, "session.close", lease_token, "{}"));
        uint64_t r0 = A, r1 = B;
        if (i < 16) r0 ^= 0xFull << (60 - 4 * i); else r1 ^= 0xFull << (60 - 4 * (i - 16));
        rand_script = { r0, r1 };
        J o = open_lease(id++);
        std::string want = token_of(r0, r1);
        if (!cl["outcome"].is("DONE") || !o["outcome"].is("DONE") || lease_token != want) {
            wrong++;
            printf("  digit %d: token %s, wanted %s\n", i, lease_token.c_str(), want.c_str());
        }
    }
    CHECK(wrong == 0, "%d of 32 digit positions did not follow their nibble", wrong);
}

// A token that is nearly the token is not the token (review C HIGH-2): the last
// digit, the first, one in the middle, a prefix, a length of 31 or 33, and
// upper case, each to session.close and to every Tier M op, and against the
// last lease's token after it ended. All refused, the lease untouched, nothing
// queued and nothing sent to the drive.
static std::vector<std::pair<std::string, std::string>> near_misses(const std::string &tk) {
    auto other = [](char ch) { return ch == '0' ? '1' : '0'; };
    std::vector<std::pair<std::string, std::string>> v;
    std::string s = tk;
    s[31] = other(s[31]); v.push_back({ "last digit changed", s });
    s = tk; s[0] = other(s[0]); v.push_back({ "first digit changed", s });
    s = tk; s[16] = other(s[16]); v.push_back({ "digit 16 changed", s });
    v.push_back({ "length 31", tk.substr(0, 31) });
    v.push_back({ "length 33", tk + "0" });
    v.push_back({ "first 8 chars", tk.substr(0, 8) });
    v.push_back({ "first 16 chars", tk.substr(0, 16) });
    v.push_back({ "last 16 chars", tk.substr(16) });
    std::string up = tk;
    for (char &ch : up) if (ch >= 'a' && ch <= 'f') ch = (char)(ch - 32);
    if (up != tk) v.push_back({ "upper case", up });
    return v;
}
static std::string tier_m_args(const char *op) {
    return !strcmp(op, "setup.ctrlg") ? ctrlg_args() : !strcmp(op, "mount") ? mount_args() : "{}";
}
static void test_token_near_miss() {
    fresh("stage 4: a near-miss token");
    power_drive();
    rand_script = { 0x0123456789abcdefull, 0xfedcba9876543210ull };
    open_lease(1);
    std::string good = lease_token;
    CHECK(good.size() == 32, "no token to take near misses of (%zu characters)", good.size());
    if (good.size() != 32) return;              // a build that cannot open a session fails here, not by an exception
    std::vector<const char *> tier_m;
    for (int op = 0; op < CTL_OP_COUNT; op++) if (ctl_ops[op].tier == CTL_TIER_M) tier_m.push_back(ctl_ops[op].name);
    std::vector<std::pair<std::string, std::string>> nm = near_misses(good);
    CHECK(tier_m.size() >= 2 && nm.size() >= 9, "population: %zu Tier M ops, %zu near-misses", tier_m.size(), nm.size());
    uint32_t id = 10;
    int sent = sim.commands;
    int bad = 0;
    auto expect = [&](const char *what, const std::string &tok, const char *op, const char *code) {
        J r = ask1(reqt(id++, op, tok, !strcmp(op, "session.close") ? "{}" : tier_m_args(op)));
        bool ok = is_refusal(r, code) && !c.pres.waiting && ctl_job.state == CTL_JOB_EMPTY && sim.commands == sent;
        if (!ok) { bad++; printf("  %s to %s: %s %s\n", what, op, r["outcome"].s.c_str(), r["err"]["code"].s.c_str()); }
    };
    for (auto &m : nm) {
        expect(m.first.c_str(), m.second, "session.close", "E_LEASE_INVALID");
        for (const char *op : tier_m) expect(m.first.c_str(), m.second, op, "E_LEASE_INVALID");
    }
    CHECK(bad == 0 && c.lease.held, "%d near-miss refusals wrong (or the lease was lost)", bad);
    // The real token still works, so the refusals above were the token's, not another gate's.
    J ok = ask1(reqt(id++, "session.close", good, "{}"));
    CHECK(ok["outcome"].is("DONE"), "the real token: %s", ok["err"]["code"].s.c_str());
    // Against the old token: with no lease held (E_NO_LEASE), then with a new
    // one (E_LEASE_INVALID). Never E_LEASE_REVOKED, which is the old token's alone.
    bad = 0;
    for (auto &m : nm) {
        expect(m.first.c_str(), m.second, "session.close", "E_NO_LEASE");
        for (const char *op : tier_m) expect(m.first.c_str(), m.second, op, "E_NO_LEASE");
    }
    CHECK(bad == 0, "%d near-misses of the old token wrong with no lease", bad);
    rand_script = { 0x1122334455667788ull, 0x99aabbccddeeff00ull };
    open_lease(id++);
    CHECK(lease_token != good, "the new lease has another token");
    for (auto &m : nm) {
        expect(m.first.c_str(), m.second, "session.close", "E_LEASE_INVALID");
        for (const char *op : tier_m) expect(m.first.c_str(), m.second, op, "E_LEASE_INVALID");
    }
    CHECK(bad == 0 && c.lease.held, "%d near-misses of the old token wrong under a new lease", bad);
}

// session.open's arguments at their bounds (review C HIGH-3). client is copied
// into a 17-byte array and idle_s is multiplied by 1000 in 32 bits, so the
// schema's limits are all that stand between a request and a smashed lease.
static void test_session_open_bounds() {
    fresh("stage 4: session.open at its bounds");
    uint32_t id = 1;
    auto open_with = [&](const std::string &client, const std::string &idle) {
        char b[200];
        snprintf(b, sizeof b, "{\"client\":\"%s\",\"tui_lock\":false,\"idle_s\":%s}", client.c_str(), idle.c_str());
        J r = ask1(req(id++, "session.open", b));
        if (r["outcome"].is("DONE")) lease_token = r["result"]["token"].s;
        return r;
    };
    auto close_it = [&] { ask1(reqt(id++, "session.close", lease_token, "{}")); };
    // client: 1 and 16 characters open and are reported back; 0, 17 and 64 are refused.
    // A 16-character name comes BEFORE the shorter ones on purpose (review
    // delta-B LOW-1, S4): the array is reused between leases, and a name that
    // does not end with its own NUL shows the previous name's tail.
    struct { size_t len; bool ok; } cl[] = { { 0, false }, { 16, true }, { 1, true }, { 6, true }, { 16, true }, { 2, true }, { 17, false }, { 64, false } };
    int wrong = 0;
    for (auto &k : cl) {
        pump(1000);                                               // a fresh rate window: each row is up to four requests
        std::string name(k.len, 'c');
        if (k.len >= 2) name[k.len - 1] = 'z';                    // the last byte is the one an overrun would cost
        char before[sizeof c.lease];
        memcpy(before, &c.lease, sizeof before);
        J r = open_with(name, "300");
        bool held = c.lease.held;
        bool good = k.ok ? (r["outcome"].is("DONE") && held && !strcmp(c.lease.client, name.c_str()) &&
                            c.lease.idle_ms == 300000u && strlen(c.lease.token) == 32 && r["result"]["idle_s"].n == 300)
                         : (is_refusal(r, "E_RANGE") && !held && !memcmp(before, &c.lease, sizeof before));
        if (good && k.ok) {
            J s = status4(id++);
            good = s["result"]["lease"]["client"].is(name.c_str());
        }
        if (!good) { wrong++; printf("  client of %zu chars: %s %s\n", k.len, r["outcome"].s.c_str(), r["err"]["code"].s.c_str()); }
        if (held) close_it();
    }
    CHECK(wrong == 0, "%d client lengths handled wrongly", wrong);
    // idle_s: 30 and 600 open and come back as given; 0, 29, 601 and the
    // values that wrap in 32 bits are refused, with nothing changed.
    struct { const char *v; bool ok; uint32_t s; } id_s[] = {
        { "0", false, 0 }, { "29", false, 0 }, { "30", true, 30 }, { "31", true, 31 }, { "599", true, 599 }, { "600", true, 600 },
        { "601", false, 0 }, { "4294967", false, 0 }, { "4294968", false, 0 }, { "4294967296", false, 0 },
        { "9007199254740991", false, 0 },
        // Review delta-B MED-2 (S5): values that TRUNCATE, in 32 bits, to a valid
        // idle_s (2^32 + 30 -> 30, 2^32 + 600 -> 600) and their neighbours; the
        // three old rows above all truncate to something out of range as well, so
        // they could not tell a 64-bit compare from a 32-bit one.
        { "4294967295", false, 0 },     // 2^32 - 1
        { "4294967297", false, 0 },     // 2^32 + 1
        { "4294967325", false, 0 },     // 2^32 + 29
        { "4294967326", false, 0 },     // 2^32 + 30
        { "4294967327", false, 0 },     // 2^32 + 31
        { "4294967895", false, 0 },     // 2^32 + 599
        { "4294967896", false, 0 },     // 2^32 + 600
        { "4294967897", false, 0 },     // 2^32 + 601
        { "8589934622", false, 0 } };   // 2^33 + 30
    wrong = 0;
    for (auto &k : id_s) {
        pump(1000);
        char before[sizeof c.lease];
        memcpy(before, &c.lease, sizeof before);
        J r = open_with("bounds", k.v);
        bool held = c.lease.held;
        bool good = k.ok ? (r["outcome"].is("DONE") && held && r["result"]["idle_s"].n == k.s && c.lease.idle_ms == k.s * 1000u)
                         : (is_refusal(r, "E_RANGE") && r["err"]["field"].is("idle_s") && !held && !memcmp(before, &c.lease, sizeof before));
        if (!good) { wrong++; printf("  idle_s %s: %s %s\n", k.v, r["outcome"].s.c_str(), r["err"]["code"].s.c_str()); }
        if (held) close_it();
    }
    CHECK(wrong == 0, "%d idle_s values handled wrongly", wrong);
}

// job.last when the job's own answer was too long to keep (jl_len 0): valid
// JSON with a null response, not `"response":` and nothing (review A LOW-5).
static void test_job_last_no_response() {
    fresh("stage 4: job.last with no kept response");
    c.jl_present = true; c.jl_op = CTL_OP_UNMOUNT; c.jl_id = 5; c.jl_len = 0;
    std::vector<std::string> raw = ask(req(1, "job.last"), 20);
    J r;
    std::string why;
    bool parsed = raw.size() == 1 && parse_response(raw[0], r, &why);
    CHECK(parsed && r["outcome"].is("DONE") && r["result"]["present"].b && r["result"]["response"].t == J::NUL,
          "job.last: %s (%s)", raw.empty() ? "no answer" : raw[0].c_str(), why.c_str());
    c.jl_present = false;
}

// The token appears in session.open's answer, and in nothing else (README item
// 36; review C LOW-2): every Tier R op (with no arguments and with valid ones),
// the refusals a token can be in the way of, a job's answer, job.last, the log
// and the status, before and after a set-up job ran. The one deliberate
// exception, a byte-identical resend of the open, is test_session_open_replay.
static void test_token_confinement() {
    fresh("stage 4: the token appears only in session.open's answer");
    power_drive();
    rand_script = { 0xC0FFEE0123456789ull, 0xDEADBEEF87654321ull };
    open_lease(1);
    const std::string T = lease_token;
    CHECK(T == "c0ffee0123456789deadbeef87654321", "the pinned token: %s", T.c_str());
    uint32_t id = 10;
    size_t answers = 0;
    int leaks = 0;
    auto sweep = [&](const std::string &line, const char *what) {
        std::vector<std::string> r = ask(line, 60);
        answers += r.size();
        for (auto &l : r) {
            std::string low = l;
            for (char &ch : low) ch = (char)tolower((unsigned char)ch);
            if (l.find(T) != std::string::npos || low.find(T) != std::string::npos) { leaks++; printf("  the token is in the answer to %s\n", what); }
        }
    };
    auto sweep_all = [&](const char *when) {
        for (int op = 0; op < CTL_OP_COUNT; op++) {
            if (ctl_ops[op].tier != CTL_TIER_R) continue;
            sweep(req(id++, ctl_ops[op].name), (std::string(ctl_ops[op].name) + " " + when).c_str());
        }
        sweep(req(id++, "log.read", "{\"cursor\":0,\"max_entries\":128}"), "log.read cursor 0");
        sweep(req(id++, "hashes.read", "{\"cursor\":0,\"max_entries\":96}"), "hashes.read cursor 0");
        sweep(req(id++, "debug.e", "{\"include_data\":true}"), "debug.e with data");
        sweep(reqt(id++, "session.close", "00112233445566778899aabbccddeeff", "{}"), "a close with another token");
        sweep(reqt(id++, "setup.ctrlg", "00112233445566778899aabbccddeeff", ctrlg_args()), "a set-up with another token");
        sweep(reqt(id++, "mount", "00112233445566778899aabbccddeeff", mount_args()), "a mount with another token");
        sweep(reqt(id++, "mount", T.substr(0, 31), mount_args()), "a mount with a token one short");
        sweep(req(id++, "session.open", "{\"client\":\"other\",\"tui_lock\":false,\"idle_s\":60}"), "a second open");
        sweep(req(id++, "session.open", "{\"client\":\"x\",\"tui_lock\":true,\"idle_s\":60}"), "an open with tui_lock");
        sweep(req(id++, "wp.on", "{\"expect_epoch\":\"x\"}"), "a schema error");
    };
    sweep_all("before a job");
    hold2(false, 400);
    send_only(reqt(id++, "setup.ctrlg", T, ctrlg_args()));
    sweep(req(id++, "status"), "status with a request waiting");
    hold2(true, 250);
    pump(50);
    for (auto &l : host_lines()) {
        answers++;
        if (l.find(T) != std::string::npos) { leaks++; printf("  the token is in a job's answer\n"); }
    }
    sweep_all("after a job");
    sweep(reqt(id++, "session.close", T, "{}"), "the close");
    sweep_all("after the close");
    CHECK(leaks == 0 && answers >= 40, "%d answers carry the token, of %zu swept", leaks, answers);
    // Not in the state the host can read back either: the log's entries carry no token bytes.
    bool in_log = false;
    for (uint32_t q = 1; q < log_next; q++) {
        const uint8_t *e = (const uint8_t *)&log_ring[q % OBS_LOG_ENTRIES];
        for (size_t k = 0; k + 8 <= sizeof(obs_log_entry_t); k++) {
            uint64_t v = 0;
            memcpy(&v, e + k, 8);
            // Both draws, in either byte order (review delta-B LOW-10: the byte-reversed
            // constant here was mistyped, and the second draw was never looked for).
            for (uint64_t draw : { 0xC0FFEE0123456789ull, 0xDEADBEEF87654321ull })
                if (v == draw || v == __builtin_bswap64(draw)) in_log = true;
        }
    }
    CHECK(!in_log, "the token's bytes are in the command log");
}

// A byte-identical resend of session.open is answered from the replay table
// (SPEC-v1 3.5: a client that lost the answer may resend safely), and that
// answer carries the token again, while the lease lives and after it has
// ended (review C LOW-1, review A LOW-2; README item 36 says so). After the
// lease ended the replayed token is dead.
static void test_session_open_replay() {
    fresh("stage 4: a resent session.open");
    rand_script = { 0x1111222233334444ull, 0x5555666677778888ull };
    std::string L = req(1, "session.open", "{\"client\":\"replay\",\"tui_lock\":false,\"idle_s\":300}");
    J a = ask1(L);
    const std::string T = a["result"]["token"].s;
    CHECK(a["outcome"].is("DONE") && T == "11112222333344445555666677778888" && !a.get("replayed"), "the first open");
    J b = ask1(L);
    CHECK(b["outcome"].is("DONE") && b["replayed"].b && b["result"]["token"].is(T.c_str()) && c.lease.held, "the resend while it lives: %s",
          b["outcome"].s.c_str());
    // Another client repeating the id with another line is not handed it.
    J o = ask1(req(1, "session.open", "{\"client\":\"other\",\"tui_lock\":false,\"idle_s\":300}"));
    CHECK(is_refusal(o, "E_ID_REUSED") && !o.get("result"), "same id, another line: %s", o["err"]["code"].s.c_str());
    J cl = ask1(reqt(2, "session.close", T, "{}"));
    CHECK(cl["outcome"].is("DONE") && !c.lease.held, "close");
    J d = ask1(L);
    CHECK(d["outcome"].is("DONE") && d["replayed"].b && d["result"]["token"].is(T.c_str()) && !c.lease.held,
          "the resend after the close is the stored answer, and opens nothing");
    J e = ask1(reqt(3, "session.close", T, "{}"));
    CHECK(is_refusal(e, "E_LEASE_REVOKED") && e["err"]["by"].is("CLOSED"), "the replayed token is dead: %s", e["err"]["code"].s.c_str());
    // A lease opened afterwards has its own token, and the resend still gives the dead one.
    rand_script = { 0x9999aaaabbbbccccull, 0xddddeeeeffff0000ull };
    J n = ask1(req(4, "session.open", "{\"client\":\"replay\",\"tui_lock\":false,\"idle_s\":300}"));
    J f = ask1(L);
    CHECK(n["outcome"].is("DONE") && n["result"]["token"].is("9999aaaabbbbccccddddeeeeffff0000") && f["replayed"].b &&
          f["result"]["token"].is(T.c_str()) && c.lease.held, "a later lease: the resend still gives the first token");
    J g = ask1(reqt(5, "session.close", T, "{}"));
    CHECK(is_refusal(g, "E_LEASE_INVALID") || is_refusal(g, "E_LEASE_REVOKED"), "the first token under a later lease: %s",
          g["err"]["code"].s.c_str());
}

// What the presence row says a JP2 act approves (review A MED-3): a
// setup.ctrlg sent with then_mount also lets a mount follow without a second
// act, so its row says "ctrlg+mount", not "setup.ctrlg". Same width, so the
// row still fits its 37 cells at the longest id (presence_rows.py).
static void test_presence_label() {
    struct Case { const char *op; std::string args; const char *label; } cases[] = {
        { "setup.ctrlg", ctrlg_args(), "setup.ctrlg" },
        { "setup.ctrlg", ctrlg_args(980, 10, 17, "idp", "master", ",\"then_mount\":true"), "ctrlg+mount" },
        { "setup.ctrlg", ctrlg_args(980, 10, 17, "idp", "master", ",\"then_mount\":false"), "setup.ctrlg" },
        { "mount", "", "mount" },
    };
    for (auto &k : cases) {
        fresh("stage 4: the presence row's label");
        power_drive();
        open_lease(1);
        hold2(false, 400);
        std::string args = k.args;
        if (!strcmp(k.op, "mount")) { tui_setup_chs(980, 10, 17); args = mount_args("TUI_DETECT_NORMAL"); }
        const uint32_t id = CTL_MAX_ID;                  // the longest id: the row at its widest
        std::string line = reqt(id, k.op, lease_token, args);
        send_only(line);
        char t[CTL_PRESENCE_TEXT];
        char want[64];
        snprintf(want, sizeof want, "JP2? %s #%u %.8s", k.label, (unsigned)id, sha_hex(line).c_str());
        bool shown = ctl_presence_text(t, sizeof t);
        CHECK(c.pres.waiting && shown && !strcmp(t, want) && strlen(t) <= 37, "the row for %s: '%s' wanted '%s'", k.op, t, want);
    }
}

// The request first, then the act (4.14; owner, R-API-1 finding 3; I-26).
static void test_presence() {
    fresh("stage 4: request first, then the act");
    power_drive();
    open_lease(1);
    hold2(false, 400);                                  // JP2 off, long enough
    std::string line = reqt(2, "setup.ctrlg", lease_token, ctrlg_args());
    uint32_t e0 = ctl_epoch;
    int n91 = sim.cmd_count[0x91], nhw = sim.hw_resets;
    send_only(line);
    CHECK(host_lines().empty() && c.pres.waiting, "no answer while it waits");
    J s = status4(3);
    const J &p = s["result"]["presence"];
    CHECK(p["waiting"].b && p["op"].is("setup.ctrlg") && p["id"].n == 2 && p["request_sha256"].is(sha_hex(line).c_str()) &&
          p["ms_left"].n > 119000 && p["ms_left"].n <= 120000 && s["result"]["job"]["queued"].is("NONE") && ctl_epoch == e0,
          "status.presence: %s id %llu", p["op"].s.c_str(), (unsigned long long)p["id"].n);
    char t[CTL_PRESENCE_TEXT];
    CHECK(ctl_presence_text(t, sizeof t) && !strncmp(t, "JP2? setup.ctrlg #2 ", 20) && !strncmp(t + 20, sha_hex(line).c_str(), 8) &&
          strlen(t) == 28, "the presence row: %s", t);
    pump(60000);                                        // ms_left counts down (60 s later)
    J s60 = status4(5);
    CHECK(s60["result"]["presence"]["ms_left"].n > 59000 && s60["result"]["presence"]["ms_left"].n <= 60000,
          "ms_left 60 s after the arrival: %llu", (unsigned long long)s60["result"]["presence"]["ms_left"].n);
    CHECK(sim.cmd_count[0x91] == n91 && sim.hw_resets == nhw, "something sent while it waits");
    uint32_t lg = log_next;
    hold2(true, 250);                                   // the act
    bool ok;
    J a = one_answer(&ok);
    CHECK(ok && a["outcome"].is("DONE") && a["id"].n == 2 && a["result"]["status"].t == J::INT && a["result"]["mode"].is("idp") &&
          a["result"]["source"].is("API_CTRLG_IDP") && a["result"]["device"].is("MASTER") &&
          a["result"]["geometry"]["cyls"].n == 980 && a["result"]["geometry"]["heads"].n == 10 &&
          a["result"]["geometry"]["spt"].n == 17 && a["result"]["geometry"]["total_sectors"].n == 980u * 10 * 17 &&
          !a["result"]["mount_credit"].b, "setup.ctrlg after the act: %s", a["outcome"].s.c_str());
    CHECK(sim.cmd_count[0x91] == n91 + 1 && sim.hw_resets == nhw + 1 && sim.cmd_count[0xEC] == 0 && ide_mchs_no_identify() &&
          ctl_setup_src == CTL_SRC_API_CTRLG_IDP && !c.pres.waiting && !ctl_presence_text(t, sizeof t),
          "the drive: 91h +%d, RESET- +%d, ECh %d", sim.cmd_count[0x91] - n91, sim.hw_resets - nhw, sim.cmd_count[0xEC]);
    // Every bus event of it between its API_BEGIN and API_END (I-13).
    int begin = -1, end = -1, bus_out = 0;
    for (uint32_t q = lg; q < log_next; q++) {
        const obs_log_entry_t &e = log_ring[q % OBS_LOG_ENTRIES];
        if (e.kind == OBS_EV_API && e.value == CTL_OP_SETUP_CTRLG) { if (e.outcome == OBS_API_BEGIN) begin = (int)q; else end = (int)q; }
        else if ((e.kind == OBS_EV_CMD || e.kind == OBS_EV_CTRL || e.kind == OBS_EV_RESET) && (begin < 0 || end >= 0)) bus_out++;
    }
    CHECK(begin > 0 && end > begin && bus_out == 0, "API markers %d..%d, %d bus events outside", begin, end, bus_out);
    J jl = ask1(req(4, "job.last"));
    CHECK(jl["result"]["op"].is("setup.ctrlg") && jl["result"]["id"].n == 2 && jl["result"]["request_sha256"].is(sha_hex(line).c_str()),
          "job.last");

    // An act before the request: nothing; the request waits for its own.
    fresh("stage 4: an act before the request");
    power_drive();
    open_lease(1);
    act();
    uint32_t ev0 = bj_fit_events();
    CHECK(ev0 >= 1, "the act made no event");
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    hold2(true, 1000);                                  // left fitted: no event
    CHECK(c.pres.waiting && host_lines().empty() && sim.cmd_count[0x91] == 0, "an earlier act approved it");
    // 120 s: E_PRESENCE_REQUIRED, nothing sent, the slot free.
    pump(117500);
    CHECK(c.pres.waiting, "withdrawn before 120 s");
    pump(1500);
    J w = one_answer();
    CHECK(is_refusal(w, "E_PRESENCE_REQUIRED") && w["retry"].is("SAFE") && !c.pres.waiting && sim.cmd_count[0x91] == 0 &&
          sim.hw_resets == 0, "at 120 s: %s", w["err"]["code"].s.c_str());
    // The same id again: the replay table holds that answer.
    J w2 = ask1(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    CHECK(w2.get("replayed") && is_refusal(w2, "E_PRESENCE_REQUIRED"), "resent: not replayed");

    // An act done at 119.9 s runs; its OPEN may have begun before the arrival.
    fresh("stage 4: the OPEN before the arrival, the edge after");
    power_drive();
    open_lease(1);
    hold2(false, 1000);                                 // JP2 off since before the request
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    pump(119700 - 250);
    hold2(true, 250);                                   // the edge at about 119.45 s, the event 200 ms later
    J late = one_answer();
    CHECK(late["outcome"].is("DONE"), "an act inside the 120 s: %s %s", late["outcome"].s.c_str(), late["err"]["code"].s.c_str());

    // A second request while one waits.
    fresh("stage 4: one waits; a second is E_INFLIGHT");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    std::string l1 = reqt(2, "setup.ctrlg", lease_token, ctrlg_args());
    send_only(l1);
    J second = ask1(reqt(3, "setup.ctrlg", lease_token, ctrlg_args(990)));
    CHECK(is_refusal(second, "E_INFLIGHT") && c.pres.waiting && c.pres.id == 2, "a second request: %s",
          second["err"]["code"].s.c_str());
    J s2 = status4(4);
    CHECK(s2["result"]["presence"]["request_sha256"].is(sha_hex(l1).c_str()), "the waiting request changed");
    J mnt = ask1(reqt(5, "mount", lease_token, mount_args()));
    CHECK(is_refusal(mnt, "E_INFLIGHT"), "a mount while one waits: %s", mnt["err"]["code"].s.c_str());
    // taskfile.snapshot too (SPEC-v1 3.5): the mailbox stays free for the act.
    J tfs = ask1(req(7, "taskfile.snapshot"));
    CHECK(is_refusal(tfs, "E_INFLIGHT") && ctl_job.state == CTL_JOB_EMPTY && c.pres.waiting && c.pres.id == 2,
          "taskfile.snapshot while one waits: %s", tfs["err"]["code"].s.c_str());
    hold2(true, 250);
    J first = one_answer();
    CHECK(first["outcome"].is("DONE") && first["id"].n == 2, "the first after the act");
    // The second, resent as a new id, waits for its own act (one act, one request).
    send_only(reqt(6, "setup.ctrlg", lease_token, ctrlg_args(990)));
    hold2(true, 1000);
    CHECK(c.pres.waiting && c.pres.id == 6 && host_lines().empty(), "one act ran two requests");
    hold2(false, 250); hold2(true, 250);
    J third = one_answer();
    CHECK(third["outcome"].is("DONE") && third["id"].n == 6, "the second's own act: %s", third["outcome"].s.c_str());
}

// Withdrawal (4.14): each NOT_EXECUTED, nothing sent, the slot free.
static void test_withdrawal() {
    struct Case { const char *name; void (*make)(); const char *code; const char *by; void (*before)(); } cases[] = {
        { "session.close", [] {
            host_send(reqt(90, "session.close", lease_token, "{}")); pump(10);
        }, "E_LEASE_REVOKED", "CLOSED", nullptr },
        { "JP1 fitted", [] { jp1_fit(true); pump(3); }, "E_LOCKED", nullptr, nullptr },
        { "a pad set up wrong", [] { mock_pads.io[31] |= 0x4u; pump(3); }, "E_LOCKED", nullptr, nullptr },
        { "the TUI turning write protect off", [] {
            config.drive_write_protected = false; ctl_wp_last = CTL_WP_TUI; hooked(); pump(3);
        }, "E_STALE", nullptr, nullptr },
        // A wp.on from another client (write protect was off when the request came).
        { "wp.on", [] { host_send(req(93, "wp.on")); pump(10); }, "E_STALE", nullptr,
          [] { config.drive_write_protected = false; ctl_wp_last = CTL_WP_TUI; hooked(); pump(3); } },
        { "identify.forbid", [] { host_send(req(91, "identify.forbid")); pump(10); }, "E_STALE", nullptr, nullptr },
        { "detach", [] { host_send(req(92, "detach", named(0).c_str())); pump(40); }, "E_STALE", nullptr, nullptr },
        // An unmount naming the current mount, with nothing mounted: it changes
        // no state (no epoch), and still withdraws it (SPEC-v1 3.5, the ratchet wins).
        { "unmount, nothing mounted", [] { host_send(req(94, "unmount", named(0).c_str())); pump(40); }, "E_STALE", nullptr,
          nullptr },
    };
    for (auto &k : cases) {
        fresh("stage 4: a waiting request withdrawn");
        power_drive();
        config.drive_write_protected = true;
        open_lease(1);
        hold2(false, 400);
        if (k.before) k.before();
        send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
        CHECK(c.pres.waiting, "%s: not waiting", k.name);
        k.make();
        std::vector<std::string> got = host_lines();
        J w;
        bool found = false, order = true;
        for (size_t i = 0; i < got.size(); i++) {
            J x;
            if (!parse_response(got[i], x) || x["id"].n != 2) continue;
            w = x; found = true;
            if (k.by && !strcmp(k.by, "CLOSED")) order = i + 1 < got.size();     // before the close's own DONE
        }
        CHECK(found && w["retry"].is(!strcmp(k.code, "E_STALE") ? "AFTER_STATUS" : "SAFE"), "%s: retry %s", k.name,
              w["retry"].s.c_str());
        CHECK(found && is_refusal(w, k.code) && (!k.by || w["err"]["by"].is(k.by)) && order && !c.pres.waiting &&
              sim.cmd_count[0x91] == 0 && sim.hw_resets == 0, "%s: %s %s (%zu lines)", k.name, w["err"]["code"].s.c_str(),
              w["err"]["by"].s.c_str(), got.size());
        // And the act after it runs nothing.
        jp1_fit(false); mock_pads.io[31] &= ~0x4u; pump(30);
        act();
        CHECK(sim.cmd_count[0x91] == 0 && host_lines().size() <= 1, "%s: the act after the withdrawal ran it", k.name);
    }
    // The port closing, and a USB reset: the request withdrawn (no answer
    // sent: the port is gone), its answer kept for a byte-identical resend
    // (3.5), with how the lease ended.
    for (int usb = 0; usb < 2; usb++) {
        fresh(usb ? "stage 4: a waiting request and a USB reset" : "stage 4: a waiting request and the port closing");
        power_drive();
        open_lease(1);
        hold2(false, 400);
        std::string line = reqt(2, "setup.ctrlg", lease_token, ctrlg_args());
        send_only(line);
        CHECK(c.pres.waiting, "not waiting");
        if (usb) { ctl_on_usb_mount(); pump(3); }
        else { tud_cdc_line_state_cb(1, false, false); pump(3); tud_cdc_line_state_cb(1, true, false); pump(3); }
        CHECK(!c.pres.waiting && host_lines().empty(), "%s: still waiting", usb ? "USB reset" : "the port closed");
        act();
        CHECK(sim.cmd_count[0x91] == 0 && sim.hw_resets == 0, "an act after the %s ran it", usb ? "USB reset" : "port closed");
        J again = ask1(line);
        CHECK(again.get("replayed") && is_refusal(again, "E_LEASE_REVOKED") && again["err"]["by"].is(usb ? "USB_RESET" : "PORT_CLOSED"),
              "the kept answer: %s by %s", again["err"]["code"].s.c_str(), again["err"]["by"].s.c_str());
    }
    // The lease's idle timer is paused while its request waits (finding 14).
    fresh("stage 4: the idle timer while a request waits");
    open_lease(1, 30);
    hold2(false, 400);
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    pump(100000);
    CHECK(c.lease.held && c.pres.waiting, "a 30 s lease lapsed while its request waited");
    pump(25000);
    J w = one_answer();
    CHECK(is_refusal(w, "E_PRESENCE_REQUIRED") && c.lease.held, "at 120 s");
    pump(31000);
    CHECK(!c.lease.held, "the timer did not restart after the wait");
}

// Arrival refusals (4.14): each answered at once, nothing waits, nothing sent.
static void test_tier_m_arrival() {
    struct Case { const char *name; std::string line; const char *code; void (*before)(); };
    fresh("stage 4: arrival refusals");
    power_drive();
    open_lease(1);
    std::string tk = lease_token;
    Case cases[] = {
        { "no token", req(10, "setup.ctrlg", ctrlg_args().c_str()), "E_SCHEMA", nullptr },
        { "another token", reqt(11, "setup.ctrlg", "00112233445566778899aabbccddeeff", ctrlg_args()), "E_LEASE_INVALID", nullptr },
        { "heads 17", reqt(12, "setup.ctrlg", tk, ctrlg_args(980, 17)), "E_RANGE", nullptr },
        { "cyls 0", reqt(13, "setup.ctrlg", tk, ctrlg_args(0)), "E_RANGE", nullptr },
        { "spt 0", reqt(14, "setup.ctrlg", tk, ctrlg_args(980, 10, 0)), "E_RANGE", nullptr },
        { "mode", reqt(15, "setup.ctrlg", tk, ctrlg_args(980, 10, 17, "lba")), "E_SCHEMA", nullptr },
        { "device", reqt(16, "setup.ctrlg", tk, ctrlg_args(980, 10, 17, "idp", "both")), "E_SCHEMA", nullptr },
        { "drive_power_cycled false", reqt(17, "setup.ctrlg", tk,
          "{\"cyls\":980,\"heads\":10,\"spt\":17,\"mode\":\"idp\",\"device\":\"master\",\"drive_power_cycled\":false}"), "E_SCHEMA", nullptr },
        { "slave, the device master", reqt(18, "setup.ctrlg", tk, ctrlg_args(980, 10, 17, "idp", "slave")), "E_PRECONDITION", nullptr },
        { "stale expect_epoch", reqt(19, "setup.ctrlg", tk, ctrlg_args(980, 10, 17, "idp", "master", ",\"expect_epoch\":1")),
          "E_STALE", nullptr },
        { "mounted", reqt(20, "setup.ctrlg", tk, ctrlg_args()), "E_STATE", [] { tui_setup_chs(980, 10, 17); tui_mount(); } },
        { "a USB command running", reqt(21, "setup.ctrlg", tk, ctrlg_args()), "E_BUS_BUSY",
          [] { busy_until_ns = mock_now_ns + 5000000000ull; } },
        { "JP1 fitted", reqt(22, "setup.ctrlg", tk, ctrlg_args()), "E_LOCKED", [] { jp1_fit(true); pump(2); } },
        { "native after a 91h", reqt(23, "setup.ctrlg", tk, ctrlg_args(980, 10, 17, "native")), "E_PRECONDITION",
          [] { idp_written = true; hooked(); } },
        { "a job in flight", reqt(24, "setup.ctrlg", tk, ctrlg_args()), "E_INFLIGHT",
          [] { core1_runs = false; host_send(req(99, "taskfile.snapshot")); pump(5); } },
        { "mount of a settings-only geometry", reqt(25, "mount", tk, mount_args("TUI_DETECT_NORMAL")), "E_PRECONDITION",
          [] { config.cyls = 980; config.heads = 10; config.spt = 17; hooked(); } },
        { "mount expecting another geometry", reqt(26, "mount", tk, mount_args("TUI_DETECT_NORMAL", 981)), "E_STALE",
          [] { tui_setup_chs(980, 10, 17); } },
        { "mount expecting another source", reqt(27, "mount", tk, mount_args("TUI_MANUAL")), "E_STALE",
          [] { tui_setup_chs(980, 10, 17); } },
        { "mount, native geometry gone", reqt(28, "mount", tk, mount_args("API_CTRLG_NATIVE")), "E_PRECONDITION",
          [] { use_lba_mode = false; cur_cyls = 980; cur_heads = 10; cur_spt = 17; sync_to_config();
               ctl_setup_noted(CTL_SRC_API_CTRLG_NATIVE); } },
        { "mount, no such source", reqt(29, "mount", tk, mount_args("NONE")), "E_SCHEMA", nullptr },
        { "mount naming a device that is not one", reqt(37, "mount", tk, "{\"expect\":{\"source\":\"TUI_DETECT_NORMAL\",\"mode\":\"CHS\",\"device\":\"BOTH\",\"cyls\":980,\"heads\":10,\"spt\":17}}"), "E_SCHEMA",
          [] { tui_setup_chs(980, 10, 17); } },
        { "mount expecting other spt", reqt(33, "mount", tk, mount_args("TUI_DETECT_NORMAL", 980, 10, 18)), "E_STALE",
          [] { tui_setup_chs(980, 10, 17); } },
        { "mount expecting other heads", reqt(34, "mount", tk, mount_args("TUI_DETECT_NORMAL", 980, 9, 17)), "E_STALE",
          [] { tui_setup_chs(980, 10, 17); } },
        { "mount expecting the slave", reqt(35, "mount", tk, "{\"expect\":{\"source\":\"TUI_DETECT_NORMAL\",\"mode\":\"CHS\",\"device\":\"SLAVE\",\"cyls\":980,\"heads\":10,\"spt\":17}}"), "E_STALE",
          [] { tui_setup_chs(980, 10, 17); } },
        { "mount expecting other LBA sectors", reqt(36, "mount", tk, "{\"expect\":{\"source\":\"TUI_DETECT_LBA\",\"mode\":\"LBA\",\"device\":\"MASTER\",\"lba_sectors\":166601}}"),
          "E_STALE", [] { use_lba_mode = true; cur_cyls = 980; cur_heads = 10; cur_spt = 17; total_lba_sectors = 166600; strcpy(hdd_model_raw, "SIM DRIVE"); sync_to_config(); ctl_setup_noted(CTL_SRC_TUI_DETECT_LBA); } },
        { "a reset left pending by a USB command", reqt(31, "setup.ctrlg", tk, ctrlg_args()), "E_BUS_BUSY", [] { rec.stage = REC_SRST; } },
        { "mount of a set-up whose geometry is gone", reqt(32, "mount", tk, mount_args("TUI_DETECT_NORMAL")), "E_PRECONDITION",
          [] { use_lba_mode = false; cur_cyls = 0; cur_heads = 0; cur_spt = 0; total_lba_sectors = 0; sync_to_config();
               ctl_setup_noted(CTL_SRC_TUI_DETECT_NORMAL); } },
        { "mount expecting CHS of an LBA set-up with the same numbers", reqt(30, "mount", tk, mount_args("TUI_DETECT_LBA")), "E_STALE",
          [] { use_lba_mode = true; cur_cyls = 980; cur_heads = 10; cur_spt = 17; total_lba_sectors = 980u * 10 * 17;
               strcpy(hdd_model_raw, "SIM DRIVE"); sync_to_config(); ctl_setup_noted(CTL_SRC_TUI_DETECT_LBA); } },
    };
    for (auto &k : cases) {
        fresh("stage 4: arrival refusals");
        power_drive();
        open_lease(1);
        std::string line = k.line;
        size_t at = line.find(tk);
        if (at != std::string::npos) line.replace(at, tk.size(), lease_token);
        if (k.before) k.before();
        std::string d0 = state2();
        uint32_t e0 = ctl_epoch;
        int n91 = sim.cmd_count[0x91], nhw = sim.hw_resets;
        host_send(line);
        pump(1);                                        // the pass that reads it: refused there, never in the slot
        bool entered = c.pres.waiting;
        pump(59);
        std::vector<std::string> got = host_lines();
        J r;
        if (got.size() == 1) parse_response(got[0], r);
        CHECK(got.size() == 1 && !entered, "%s: %zu answers, entered the slot %d", k.name, got.size(), (int)entered);
        CHECK(is_refusal(r, k.code) && !c.pres.waiting && ctl_epoch == e0 && state2() == d0 && sim.cmd_count[0x91] == n91 &&
              sim.hw_resets == nhw, "%s: %s (%s %s)", k.name, r["err"]["code"].s.c_str(), r["err"]["why"].s.c_str(),
              r["err"]["field"].s.c_str());
        core1_runs = true; busy_until_ns = 0; jp1_fit(false);
    }
}

// The declared pair (owner Q3): one act, the mount on its credit.
static void pair_setup(uint32_t id, bool then_mount = true) {
    power_drive();
    hold2(false, 400);
    send_only(reqt(id, "setup.ctrlg", lease_token, ctrlg_args(980, 10, 17, "idp", "master", then_mount ? ",\"then_mount\":true" : "")));
    hold2(true, 250);
}
static void test_pair() {
    fresh("stage 4: the declared pair");
    open_lease(1);
    pair_setup(2);
    J a = one_answer();
    CHECK(a["outcome"].is("DONE") && a["result"]["mount_credit"].b, "the set-up: %s credit %d", a["outcome"].s.c_str(),
          a["result"]["mount_credit"].b);
    J s = status4(3);
    CHECK(s["result"]["jumpers"]["mount_credit_ms_left"].n > 55000, "credit %llu ms", (unsigned long long)s["result"]["jumpers"]["mount_credit_ms_left"].n);
    pump(30000);
    uint32_t ms0 = ctl_mount_seq;
    J m = ask1(reqt(4, "mount", lease_token, mount_args()), nullptr, 20);
    CHECK(m["outcome"].is("DONE") && m["result"]["used_credit"].b && m["result"]["mounted"].b &&
          m["result"]["mount_seq"].n == ms0 + 1 && m["result"]["total_sectors"].n == 980u * 10 * 17 && is_mounted &&
          current_screen == SCREEN_MOUNTED && !c.pres.waiting, "the mount on the credit: %s %s", m["outcome"].s.c_str(),
          m["err"]["code"].s.c_str());
    // The MOUNT marker says the API mounted (5.4).
    CHECK(log_ring[ctl_mount_log_seq % OBS_LOG_ENTRIES].kind == OBS_EV_MOUNT &&
          log_ring[ctl_mount_log_seq % OBS_LOG_ENTRIES].value == OBS_MNT_API, "marker value");
    // Used: after an unmount, the same mount waits for its own act.
    J u = ask1(req(5, "unmount", named(ctl_mount_seq).c_str()));
    CHECK(u["outcome"].is("DONE") && !is_mounted, "unmount");
    send_only(reqt(6, "mount", lease_token, mount_args()));
    CHECK(c.pres.waiting && c.pres.op == CTL_OP_MOUNT, "a second mount on a spent credit");
    hold2(false, 250); hold2(true, 250);
    J m2 = one_answer();
    CHECK(m2["outcome"].is("DONE") && !m2["result"]["used_credit"].b, "the mount's own act: %s", m2["outcome"].s.c_str());

    // The credit's edge, read ONCE (review A LOW-1): the clock moves on with every
    // read here, so a decision made from two reads of it can straddle the 60 s.
    // With the terminal closed a mount on the credit is not asked for a terminal
    // and one not on it is refused NO_TERMINAL; it must never WAIT for an act
    // that would be withdrawn anyway (the arrival check skipped, the credit gone).
    {
        int on_credit = 0, refused = 0, waited = 0;
        for (int step = 0; step <= 60; step++) {
            fresh("stage 4: the credit's edge, read once");
            open_lease(1);
            pair_setup(2);
            (void)one_answer();
            cdc_connected = false;
            pump(3);
            uint64_t edge_ns = ((uint64_t)c.credit.end_ms + 60000u) * 1000000ull;
            uint64_t now_ns = mock_now_ns;
            if (edge_ns > now_ns + 5000000ull) pump((uint32_t)((edge_ns - now_ns) / 1000000ull) - 5);
            mock_now_ns = edge_ns - 400000ull + (uint64_t)step * 20000ull;        // -0.4 ms .. +0.8 ms around the edge
            mock_read_step_ns = 250000;                                        // each read of the clock is 0.25 ms later
            host_send(reqt(3, "mount", lease_token, mount_args()));
            one_task();
            mock_read_step_ns = 0;
            task_max_ns = 0;
            std::vector<std::string> got = host_lines();
            J r;
            bool refusal = got.size() == 1 && parse_response(got[0], r) && is_refusal(r, "E_PRECONDITION") && r["err"]["why"].is("NO_TERMINAL");
            if (c.pres.waiting) waited++;
            else if (refusal) refused++;
            else on_credit++;
            pump(50);
        }
        CHECK(waited == 0, "%d mounts arrived on a credit that lapsed between two reads of the clock and WAITED with no terminal", waited);
        CHECK(on_credit > 0 && refused > 0, "the sweep did not straddle the edge: %d on the credit, %d refused", on_credit, refused);
    }

    // The 60 s: at 59 s it runs on the credit, at 61 s it waits for an act.
    for (uint32_t after : { 59000u, 61000u }) {
        fresh("stage 4: the credit's 60 s");
        open_lease(1);
        pair_setup(2);
        (void)one_answer();
        pump(after - 30);
        host_send(reqt(3, "mount", lease_token, mount_args()));
        pump(20);
        bool on_credit = after < 60000;
        CHECK(on_credit ? (is_mounted && !c.pres.waiting) : (!is_mounted && c.pres.waiting), "mount at %u ms: mounted %d waiting %d",
              after, (int)is_mounted, (int)c.pres.waiting);
    }
    // No credit: without then_mount, after a failed set-up, after an epoch
    // change, under another lease, for another geometry.
    struct Case { const char *name; void (*between)(); bool then_mount; bool fail; } cases[] = {
        { "without then_mount", nullptr, false, false },
        { "a set-up that failed", nullptr, true, true },
        { "an epoch change (wp.on)", [] { config.drive_write_protected = false; hooked(); pump(3);
                                          host_send(req(80, "wp.on")); pump(5); host_lines(); }, true, false },
        { "JP1 fitted and off again", [] { jp1_fit(true); pump(3); jp1_fit(false); pump(30); }, true, false },
        { "another lease", [] { host_send(reqt(81, "session.close", lease_token, "{}")); pump(5); host_lines();
                                open_lease(82); }, true, false },
    };
    for (auto &k : cases) {
        fresh("stage 4: no credit");
        open_lease(1);
        if (k.fail) { sim.reject_idp = true; media_changed_waiting = false; strcpy(hdd_status_text, "old"); }
        pair_setup(2, k.then_mount);
        J a2 = one_answer();
        sim.reject_idp = false;
        CHECK(k.fail ? a2["outcome"].is("FAILED") && a2["err"]["code"].is("E_DRIVE") && a2["err"]["why"].is("REFUSED")
                     : a2["outcome"].is("DONE") && a2["result"]["mount_credit"].b == k.then_mount,
              "%s: the set-up %s %s", k.name, a2["outcome"].s.c_str(), a2["err"]["why"].s.c_str());
        if (k.between) k.between();
        CHECK(!credit_live(now_ms()), "%s: a credit is live", k.name);
        if (k.fail) {
            // The failed set-up left what a dismissed error box leaves.
            CHECK(ctl_setup_src == CTL_SRC_NONE && config.cyls == 0 && !strcmp(hdd_model_raw, MCHS_MODEL_CLEARED) &&
                  !show_detect_result && job_tui_io == 0 && media_changed_waiting && !hdd_status_text[0],
                  "%s: the state after it (media change %d, status text '%s')", k.name, (int)media_changed_waiting, hdd_status_text);
            continue;
        }
        send_only(reqt(90, "mount", lease_token, mount_args()), 20);
        CHECK(!is_mounted && c.pres.waiting && c.pres.op == CTL_OP_MOUNT, "%s: a mount without an act (mounted %d)", k.name,
              (int)is_mounted);
    }
    // The credit's 60 s counted to the pickup (Q3): queued at 59 s, picked up
    // after 60 s, refused (E_PRESENCE_REQUIRED).
    fresh("stage 4: the credit lapses between the queueing and the pickup");
    open_lease(1);
    pair_setup(2);
    (void)one_answer();
    pump(59000);
    core1_runs = false;
    host_send(reqt(3, "mount", lease_token, mount_args()));
    pump(5);
    CHECK(ctl_job.state == CTL_JOB_QUEUED && ctl_job.via_credit, "not queued on the credit");
    pump(2000);
    core1_runs = true;
    pump(20);
    {
        J r = one_answer();
        CHECK(is_refusal(r, "E_PRESENCE_REQUIRED") && !is_mounted, "a credit past 60 s at pickup: %s", r["err"]["code"].s.c_str());
    }
    // A change after the mount was queued on the credit, before its pickup,
    // ends the credit there too (Q3): refused, not mounted.
    fresh("stage 4: a change between the credit mount's queueing and its pickup");
    open_lease(1);
    pair_setup(2);
    (void)one_answer();
    core1_runs = false;
    host_send(reqt(3, "mount", lease_token, mount_args()));
    pump(5);
    CHECK(ctl_job.state == CTL_JOB_QUEUED && ctl_job.via_credit, "not queued on the credit");
    human_wp(false);
    pump(3);
    core1_runs = true;
    pump(20);
    {
        J r = one_answer();
        CHECK(is_refusal(r, "E_PRESENCE_REQUIRED") && !is_mounted, "a change before the pickup: %s", r["err"]["code"].s.c_str());
    }
    // One credit, one mount, even one that never ran: a mount queued on it and
    // withdrawn unpicked (E_TUI_BUSY at 30 s, no state change) spent it.
    fresh("stage 4: the credit spent by a mount that never ran");
    open_lease(1);
    pair_setup(2);
    (void)one_answer();
    core1_runs = false;
    host_send(reqt(3, "mount", lease_token, mount_args()));
    pump(5);
    CHECK(ctl_job.state == CTL_JOB_QUEUED && ctl_job.via_credit, "not queued on the credit");
    pump(30500);
    {
        J r = one_answer();
        CHECK(is_refusal(r, "E_TUI_BUSY"), "the unpicked mount: %s", r["err"]["code"].s.c_str());
    }
    core1_runs = true;
    send_only(reqt(4, "mount", lease_token, mount_args()), 20);
    CHECK(c.pres.waiting && c.pres.id == 4 && !is_mounted, "a second mount on one credit (mounted %d, waiting %d)", (int)is_mounted,
          (int)c.pres.waiting);
    // A mount expecting another geometry never uses the credit (E_STALE at arrival).
    fresh("stage 4: the credit and another geometry");
    open_lease(1);
    pair_setup(2);
    (void)one_answer();
    J other = ask1(reqt(3, "mount", lease_token, mount_args("API_CTRLG_IDP", 979)));
    CHECK(is_refusal(other, "E_STALE") && !is_mounted && credit_live(now_ms()), "another geometry: %s", other["err"]["code"].s.c_str());
    // An unmount or detach naming another mount changes nothing (I-27): the credit stays (X68).
    J u1 = ask1(req(4, "unmount", named(ctl_mount_seq + 1).c_str()));
    // (detach carries the lease's token here: a tokenless one is refused for the lease first, test_detach_lease.)
    J d1 = ask1(reqt(5, "detach", lease_token, named(ctl_mount_seq + 2)));
    CHECK(is_refusal(u1, "E_STALE") && is_refusal(d1, "E_STALE") && credit_live(now_ms()), "a mount-name mismatch: %s %s, credit %d",
          u1["err"]["code"].s.c_str(), d1["err"]["code"].s.c_str(), (int)credit_live(now_ms()));
}

// At pickup (5.3, 4.14; findings 9 and 14): the act was spent, the request
// refused if its lease, JP1 or a precondition changed meanwhile.
static void test_pickup() {
    struct Case { const char *name; void (*between)(); const char *code; } cases[] = {
        { "JP1 fitted", [] { jp1_fit(true); pump(3); }, "E_LOCKED" },
        { "the TUI mounted", [] { tui_setup_chs(980, 10, 17); tui_mount(); }, "E_STATE" },
        { "a USB command started", [] { busy_until_ns = mock_now_ns + 5000000000ull; }, "E_BUS_BUSY" },
    };
    for (auto &k : cases) {
        fresh("stage 4: at pickup");
        power_drive();
        open_lease(1);
        hold2(false, 400);
        send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
        core1_runs = false;                             // core 1 in a dialog: the approved job waits
        hold2(true, 250);
        CHECK(ctl_job.state == CTL_JOB_QUEUED && !c.pres.waiting, "%s: not queued", k.name);
        k.between();
        core1_runs = true;
        pump(20);
        std::vector<std::string> got = host_lines();
        J r;
        for (auto &l : got) { J x; if (parse_response(l, x) && x["id"].n == 2) r = x; }
        CHECK(is_refusal(r, k.code) && sim.cmd_count[0x91] == 0 && sim.hw_resets == 0 && !c.jl_present,
              "%s: %s", k.name, r["err"]["code"].s.c_str());
        jp1_fit(false); busy_until_ns = 0;
    }
    // expect_epoch compared again at pickup: a change while the approved job
    // waits for core 1 refuses it (E_STALE), nothing sent.
    fresh("stage 4: at pickup, expect_epoch");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    {
        std::string ep = ",\"expect_epoch\":" + std::to_string(ctl_epoch);
        send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args(980, 10, 17, "idp", "master", ep.c_str())));
    }
    core1_runs = false;
    hold2(true, 250);
    CHECK(ctl_job.state == CTL_JOB_QUEUED, "expect_epoch: not queued");
    human_wp(false);                                    // the TUI changes something meanwhile
    pump(3);
    core1_runs = true;
    pump(20);
    {
        J r;
        for (auto &l : host_lines()) { J x; if (parse_response(l, x) && x["id"].n == 2) r = x; }
        CHECK(is_refusal(r, "E_STALE") && r["err"]["field"].is("expect_epoch") && sim.cmd_count[0x91] == 0 && sim.hw_resets == 0,
              "expect_epoch at pickup: %s %s", r["err"]["code"].s.c_str(), r["err"]["field"].s.c_str());
    }
    // The lease closed while the approved job waits for core 1: withdrawn
    // (finding 9), answered before the close's own DONE.
    fresh("stage 4: a queued job and the lease's end");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    core1_runs = false;
    hold2(true, 250);
    host_send(reqt(3, "session.close", lease_token, "{}"));
    pump(10);
    std::vector<std::string> got = host_lines();
    J w, cl;
    if (got.size() == 2) { parse_response(got[0], w); parse_response(got[1], cl); }
    CHECK(got.size() == 2 && w["id"].n == 2 && is_refusal(w, "E_LEASE_REVOKED") && w["err"]["by"].is("CLOSED") &&
          cl["id"].n == 3 && cl["outcome"].is("DONE") && ctl_job.state == CTL_JOB_EMPTY, "%zu lines", got.size());
    core1_runs = true;
    pump(50);
    CHECK(sim.cmd_count[0x91] == 0 && sim.hw_resets == 0, "a withdrawn job ran");
    // The idle timer paused while its job is queued (finding 14): a 30 s
    // lease, core 1 away 40 s, the job still runs.
    fresh("stage 4: the idle timer while its job is queued");
    power_drive();
    open_lease(1, 30);
    hold2(false, 400);
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    core1_runs = false;
    hold2(true, 250);
    pump(29000);
    core1_runs = true;
    pump(20);
    J done = one_answer();
    CHECK(done["outcome"].is("DONE") && c.lease.held, "after 29 s queued: %s, lease %d", done["outcome"].s.c_str(), (int)c.lease.held);
}

// No terminal on interface 0 (R-API-1 finding 1): the job neither draws nor
// reads a key, ends within its time, and the TUI's queue is as it was. Since
// 2026-09-29 a request is only left waiting while a terminal is open on the
// main menu (test_no_terminal_refusal) and is only approved by an act that
// finds one still there (test_no_terminal_at_act), so here the terminal closes
// AFTER the act and before the job runs: the job must still run without it.
static void test_tier_m_no_terminal() {
    fresh("stage 4: Tier M with no terminal");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args(980, 10, 17, "idp", "master", ",\"then_mount\":true")));
    CHECK(c.pres.waiting, "the pair's set-up is not waiting");
    tty.assign(4096, 'q');
    std::string tty0 = tty;
    core1_runs = false;
    hold2(true, 250);
    cdc_connected = false;                               // closed after the act, before the job
    core1_runs = true;
    pump(20);
    J a = one_answer();
    J m = ask1(reqt(3, "mount", lease_token, mount_args()), nullptr, 20);
    CHECK(a["outcome"].is("DONE") && m["outcome"].is("DONE") && tty == tty0 && job_tui_io == 0,
          "with no terminal: %s %s, TUI queue used %d times", a["outcome"].s.c_str(), m["outcome"].s.c_str(), job_tui_io);
    // A drive that does not answer: the job ends (FAILED), within 75 s, and draws nothing.
    fresh("stage 4: Tier M with no terminal, no drive");
    sim.hw_reset_wedges = true;                          // BSY for ever after RESET-
    open_lease(1);
    hold2(false, 400);
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    tty.assign(4096, 'q');
    tty0 = tty;
    core1_runs = false;
    hold2(true, 250);
    cdc_connected = false;                               // closed after the act, before the job
    core1_runs = true;
    pump(80000);
    J f = one_answer();
    CHECK(f["outcome"].is("FAILED") && f["err"]["code"].is("E_DRIVE") && tty == tty0 && job_tui_io == 0 &&
          c.jl_present && c.jl_ended - c.jl_started < 75000 && ctl_job.state == CTL_JOB_EMPTY,
          "no drive: %s %s tty %d io %d t %llu state %u", f["outcome"].s.c_str(), f["err"]["why"].s.c_str(), tty == tty0, job_tui_io, (unsigned long long)(c.jl_ended - c.jl_started), (unsigned)ctl_job.state);
    // Refused at pickup (a USB command started after the act): nothing
    // sent, nothing drawn, nothing waiting afterwards.
    fresh("stage 4: Tier M with no terminal, refused at pickup");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    tty.assign(4096, 'q');
    tty0 = tty;
    core1_runs = false;
    hold2(true, 250);
    cdc_connected = false;                               // closed after the act, before the job
    busy_until_ns = mock_now_ns + 5000000000ull;
    core1_runs = true;
    pump(20);
    J rf = one_answer();
    CHECK(is_refusal(rf, "E_BUS_BUSY") && sim.commands == 0 && sim.hw_resets == 0 && tty == tty0 && job_tui_io == 0 &&
          ctl_job.state == CTL_JOB_EMPTY && !c.pres.waiting && !c.jl_present,
          "refused at pickup: %s, commands %d, resets %d, tty %d, io %d", rf["err"]["code"].s.c_str(), sim.commands,
          sim.hw_resets, tty == tty0, job_tui_io);
    busy_until_ns = 0;
    cdc_connected = false;
}

// A guarded request is refused, not left waiting, unless the person can see
// the presence row: the TUI on the main menu with a terminal connected (owner
// ruling 2026-09-29 on review A MED-3 (2)). E_PRECONDITION, why NO_TERMINAL,
// nothing queued, nothing sent, the act that follows approves nothing.
static void test_no_terminal_refusal() {
    struct Case { const char *name; bool connected; uint8_t screen; } cases[] = {
        { "no terminal", false, CTL_TUI_MAIN },
        { "terminal, boot", true, CTL_TUI_BOOT },
        { "terminal, features", true, CTL_TUI_FEATURES },
        { "terminal, debug", true, CTL_TUI_DEBUG },
        { "terminal, mounted screen", true, CTL_TUI_MOUNTED },
        { "terminal, confirm box", true, CTL_TUI_CONFIRM },
        { "terminal, error box", true, CTL_TUI_ERROR_BOX },
        { "terminal, modal", true, CTL_TUI_MODAL },
        { "no terminal, features", false, CTL_TUI_FEATURES },
    };
    for (const char *op : { "setup.ctrlg", "mount" }) {
        for (auto &k : cases) {
            fresh("stage 4: no terminal, no wait");
            power_drive();
            config.drive_write_protected = true;
            open_lease(1);
            if (!strcmp(op, "mount")) tui_setup_chs(980, 10, 17);
            hold2(false, 400);
            cdc_connected = k.connected;
            core1_screen = k.screen;
            if (k.screen == CTL_TUI_BOOT) { core1_runs = false; ctl_tui_state = CTL_TUI_BOOT; }
            pump(3);
            std::string line = !strcmp(op, "mount") ? reqt(2, "mount", lease_token, mount_args("TUI_DETECT_NORMAL"))
                                                    : reqt(2, "setup.ctrlg", lease_token, ctrlg_args());
            std::string d0 = state2();
            host_send(line);
            pump(20);
            J r = one_answer();
            CHECK(is_refusal(r, "E_PRECONDITION") && r["err"]["why"].is("NO_TERMINAL") && r["retry"].is("SAFE") &&
                  !c.pres.waiting && ctl_job.state == CTL_JOB_EMPTY && sim.commands == 0 && state2() == d0,
                  "%s %s: %s %s waiting %d", op, k.name, r["err"]["code"].s.c_str(), r["err"]["why"].s.c_str(), (int)c.pres.waiting);
            // The act approves nothing: no request waits, nothing is queued, nothing is sent.
            hold2(true, 250);
            hold2(false, 30);
            CHECK(!c.pres.waiting && ctl_job.state == CTL_JOB_EMPTY && sim.commands == 0 && host_lines().empty() && !is_mounted,
                  "%s %s: the act after the refusal did something", op, k.name);
            // The refusal is what the replay table keeps.
            host_send(line);
            pump(20);
            J rp = one_answer();
            CHECK(rp["replayed"].b && rp["err"]["why"].is("NO_TERMINAL") && !c.pres.waiting, "%s %s: the resend", op, k.name);
            // status says nothing waits.
            J st = status4(3);
            CHECK(!st["result"]["presence"]["waiting"].b, "%s %s: status.presence", op, k.name);
        }
    }
    // With a terminal on the main menu the same request waits, and the act runs it.
    for (const char *op : { "setup.ctrlg", "mount" }) {
        fresh("stage 4: a terminal on the main menu: the wait");
        power_drive();
        config.drive_write_protected = true;
        open_lease(1);
        if (!strcmp(op, "mount")) tui_setup_chs(980, 10, 17);
        hold2(false, 400);
        cdc_connected = true;
        pump(3);
        CHECK(ctl_tui_state == CTL_TUI_MAIN, "the published state %u", ctl_tui_state);
        send_only(!strcmp(op, "mount") ? reqt(2, "mount", lease_token, mount_args("TUI_DETECT_NORMAL"))
                                        : reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
        CHECK(c.pres.waiting && c.pres.id == 2, "%s: not waiting with a terminal on the main menu", op);
        hold2(true, 250);
        J ok = one_answer();
        CHECK(ok["outcome"].is("DONE"), "%s: %s %s", op, ok["outcome"].s.c_str(), ok["err"]["code"].s.c_str());
        cdc_connected = false;
    }
    // The refusal comes after every other check, so a request that fails for another reason keeps its own answer.
    fresh("stage 4: no terminal, another refusal first");
    power_drive();
    config.drive_write_protected = true;
    open_lease(1);
    tui_setup_chs(980, 10, 17);
    cdc_connected = false;
    pump(3);
    J st = ask1(reqt(2, "mount", lease_token, mount_args("TUI_DETECT_NORMAL", 979)));
    CHECK(is_refusal(st, "E_STALE"), "a mount naming another geometry, no terminal: %s %s", st["err"]["code"].s.c_str(), st["err"]["why"].s.c_str());
    J bad = ask1(reqt(3, "setup.ctrlg", std::string(32, '0'), ctrlg_args()));
    CHECK(is_refusal(bad, "E_LEASE_INVALID"), "a wrong token, no terminal: %s", bad["err"]["code"].s.c_str());
    J wr = ask1(reqt(4, "setup.ctrlg", lease_token, ctrlg_args(980, 10, 17, "idp", "slave")));
    CHECK(is_refusal(wr, "E_PRECONDITION") && wr["err"]["why"].is("NOT_THE_DEVICE"), "another device, no terminal: %s %s",
          wr["err"]["code"].s.c_str(), wr["err"]["why"].s.c_str());
    // ...and the other refusals that come before it (review delta-B LOW-2, A5):
    // a stale expect_epoch, JP1, and above all E_INFLIGHT while another request
    // waits (the terminal closed after it began to wait), each with no terminal.
    J se = ask1(reqt(5, "setup.ctrlg", lease_token, ctrlg_args(980, 10, 17, "idp", "master", (",\"expect_epoch\":" + std::to_string(epoch_now() + 1)).c_str())));
    CHECK(is_refusal(se, "E_STALE") && se["err"]["field"].is("expect_epoch"), "a stale expect_epoch, no terminal: %s %s",
          se["err"]["code"].s.c_str(), se["err"]["why"].s.c_str());
    fresh("stage 4: no terminal, E_INFLIGHT first");
    power_drive();
    config.drive_write_protected = true;
    open_lease(1);
    hold2(false, 400);
    pump(3);
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    CHECK(c.pres.waiting && c.pres.id == 2, "the first request does not wait");
    cdc_connected = false;                      // the terminal closes; the wait itself is not cut short
    pump(3);
    J inf = ask1(reqt(3, "setup.ctrlg", lease_token, ctrlg_args()));
    CHECK(is_refusal(inf, "E_INFLIGHT") && c.pres.waiting && c.pres.id == 2, "a second request while one waits, no terminal: %s %s",
          inf["err"]["code"].s.c_str(), inf["err"]["why"].s.c_str());
    fresh("stage 4: no terminal, E_LOCKED first");
    power_drive();
    config.drive_write_protected = true;
    open_lease(1);
    hold2(false, 400);
    cdc_connected = false;
    jp1_fit(true);
    pump(3);
    J lk = ask1(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    CHECK(is_refusal(lk, "E_LOCKED"), "JP1 fitted, no terminal: %s %s", lk["err"]["code"].s.c_str(), lk["err"]["why"].s.c_str());
    jp1_fit(false);
    // A mount on the pair's credit is not a wait: it needs no terminal (the credit was earned by one act,
    // under a terminal, and taking it waits for nothing).
    fresh("stage 4: the pair's mount, the terminal closed after the set-up");
    cdc_connected = true;
    open_lease(1);
    pair_setup(2);
    J a = one_answer();
    CHECK(a["outcome"].is("DONE") && a["result"]["mount_credit"].b, "the set-up under a terminal: %s", a["outcome"].s.c_str());
    cdc_connected = false;
    pump(3);
    J m = ask1(reqt(3, "mount", lease_token, mount_args()), nullptr, 20);
    CHECK(m["outcome"].is("DONE") && is_mounted, "the credit's mount with the terminal closed: %s %s", m["outcome"].s.c_str(),
          m["err"]["code"].s.c_str());
    cdc_connected = false;
}

// The same condition at the act (owner ruling 2026-09-29, review A MED-3 (2)
// gap): a request that waits is approved only if the terminal is connected and
// the TUI is on the main menu when the JP2 act arrives; if the row cannot be
// shown then, the request is withdrawn, E_PRECONDITION, why NO_TERMINAL, and
// the act approves nothing. Only refuses more: the wait itself is not cut short
// by a screen that passes through a modal box.
static void test_no_terminal_at_act() {
    struct Case { const char *name; bool connected; uint8_t screen; } cases[] = {
        { "terminal closed", false, CTL_TUI_MAIN },
        { "terminal closed, on features", false, CTL_TUI_FEATURES },
        { "features screen", true, CTL_TUI_FEATURES },
        { "debug screen", true, CTL_TUI_DEBUG },
        { "confirm box", true, CTL_TUI_CONFIRM },
        { "error box", true, CTL_TUI_ERROR_BOX },
        { "modal", true, CTL_TUI_MODAL },
    };
    for (const char *op : { "setup.ctrlg", "mount" }) {
        for (auto &k : cases) {
            fresh("stage 4: NO_TERMINAL at the act");
            power_drive();
            config.drive_write_protected = true;
            open_lease(1);
            if (!strcmp(op, "mount")) tui_setup_chs(980, 10, 17);
            hold2(false, 400);
            cdc_connected = true;
            pump(3);
            std::string line = !strcmp(op, "mount") ? reqt(2, "mount", lease_token, mount_args("TUI_DETECT_NORMAL"))
                                                    : reqt(2, "setup.ctrlg", lease_token, ctrlg_args());
            send_only(line);
            CHECK(c.pres.waiting && c.pres.id == 2, "%s %s: not waiting on arrival", op, k.name);
            cdc_connected = k.connected;                // the row can no longer be shown
            core1_screen = k.screen;
            pump(3);
            CHECK(c.pres.waiting, "%s %s: the wait itself was cut short before any act", op, k.name);
            hold2(true, 250);
            J r = one_answer();
            CHECK(is_refusal(r, "E_PRECONDITION") && r["err"]["why"].is("NO_TERMINAL") && r["retry"].is("SAFE") && r["id"].n == 2 &&
                  !c.pres.waiting && ctl_job.state == CTL_JOB_EMPTY && sim.commands == 0 && !is_mounted,
                  "%s %s: %s %s waiting %d", op, k.name, r["outcome"].s.c_str(), r["err"]["why"].s.c_str(), (int)c.pres.waiting);
            // Nothing runs later either, and the refusal is what the replay table keeps.
            hold2(false, 30);
            hold2(true, 250);
            pump(50);
            CHECK(ctl_job.state == CTL_JOB_EMPTY && sim.commands == 0 && !is_mounted && host_lines().empty(), "%s %s: it ran after the refusal", op, k.name);
            host_send(line);
            pump(20);
            J rp = one_answer();
            CHECK(rp["replayed"].b && rp["err"]["why"].is("NO_TERMINAL"), "%s %s: the resend", op, k.name);
            J st = status4(3);
            CHECK(!st["result"]["presence"]["waiting"].b, "%s %s: status.presence", op, k.name);
        }
    }
    // A terminal that is there at the act: approved. The screen may have passed
    // through a modal box while it waited (only the act's moment is asked), and
    // a terminal closed and reopened before the act is the same as one never closed.
    for (const char *op : { "setup.ctrlg", "mount" }) {
        fresh("stage 4: NO_TERMINAL at the act: the row shown again");
        power_drive();
        config.drive_write_protected = true;
        open_lease(1);
        if (!strcmp(op, "mount")) tui_setup_chs(980, 10, 17);
        hold2(false, 400);
        cdc_connected = true;
        pump(3);
        send_only(!strcmp(op, "mount") ? reqt(2, "mount", lease_token, mount_args("TUI_DETECT_NORMAL")) : reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
        CHECK(c.pres.waiting, "%s: not waiting", op);
        cdc_connected = false;
        core1_screen = CTL_TUI_MODAL;
        pump(10);
        cdc_connected = true;
        core1_screen = CTL_TUI_MAIN;
        pump(10);
        CHECK(c.pres.waiting, "%s: withdrawn by a screen that passed", op);
        hold2(true, 250);
        J ok = one_answer();
        CHECK(ok["outcome"].is("DONE"), "%s: the row shown at the act, not approved: %s %s", op, ok["outcome"].s.c_str(), ok["err"]["why"].s.c_str());
        cdc_connected = false;
    }
    // After a withdrawal at the act, a new request with the terminal back waits and is approved.
    fresh("stage 4: NO_TERMINAL at the act: then the next request");
    power_drive();
    config.drive_write_protected = true;
    open_lease(1);
    hold2(false, 400);
    cdc_connected = true;
    pump(3);
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    cdc_connected = false;
    pump(3);
    hold2(true, 250);
    J w = one_answer();
    CHECK(is_refusal(w, "E_PRECONDITION") && w["err"]["why"].is("NO_TERMINAL"), "the first: %s", w["err"]["why"].s.c_str());
    hold2(false, 400);
    cdc_connected = true;
    pump(3);
    send_only(reqt(3, "setup.ctrlg", lease_token, ctrlg_args()));
    CHECK(c.pres.waiting && c.pres.id == 3, "the next request does not wait");
    hold2(true, 250);
    J ok2 = one_answer();
    CHECK(ok2["outcome"].is("DONE"), "the next request: %s %s", ok2["outcome"].s.c_str(), ok2["err"]["why"].s.c_str());
    cdc_connected = false;
}

// mount refuses while write protect is off (review A MED-1): a recovery drive
// is never host-writable through the API. Arrival and pickup, both ways; the
// pair's mount goes through because a Ctrl+G drive is write protected whatever
// Features says (ide_write_protected).
static void test_mount_wp_off() {
    fresh("stage 4: mount with write protect off, at arrival");
    power_drive();
    open_lease(1);
    tui_setup_chs(980, 10, 17);
    hold2(false, 400);
    human_wp(false);
    pump(2);
    J r = ask1(reqt(2, "mount", lease_token, mount_args("TUI_DETECT_NORMAL")));
    CHECK(is_refusal(r, "E_PRECONDITION") && r["err"]["why"].is("WP_OFF") && !c.pres.waiting && !is_mounted &&
          ctl_job.state == CTL_JOB_EMPTY, "WP off at arrival: %s %s", r["err"]["code"].s.c_str(), r["err"]["why"].s.c_str());
    // Write protect on: the same request waits for its act and mounts.
    human_wp(true);
    pump(2);
    send_only(reqt(3, "mount", lease_token, mount_args("TUI_DETECT_NORMAL")));
    CHECK(c.pres.waiting && c.pres.op == CTL_OP_MOUNT, "WP on: the mount waits");
    hold2(true, 250);
    J ok = one_answer();
    CHECK(ok["outcome"].is("DONE") && is_mounted && ide_write_protected(), "WP on: mounted, %s", ok["err"]["code"].s.c_str());

    // Turned off after the request was approved, before its pickup.
    fresh("stage 4: mount with write protect off, at pickup");
    power_drive();
    open_lease(1);
    tui_setup_chs(980, 10, 17);
    hold2(false, 400);
    send_only(reqt(2, "mount", lease_token, mount_args("TUI_DETECT_NORMAL")));
    core1_runs = false;
    hold2(true, 250);
    CHECK(ctl_job.state == CTL_JOB_QUEUED, "the mount not queued");
    human_wp(false);
    pump(3);
    core1_runs = true;
    pump(20);
    J p = one_answer();
    CHECK(is_refusal(p, "E_PRECONDITION") && p["err"]["why"].is("WP_OFF") && !is_mounted && ctl_job.state == CTL_JOB_EMPTY,
          "WP off at pickup: %s %s", p["err"]["code"].s.c_str(), p["err"]["why"].s.c_str());

    // The pair with Features' write protect off: the Ctrl+G drive is protected anyway.
    fresh("stage 4: the pair with write protect off in Features");
    open_lease(1);
    human_wp(false);
    pump(2);
    pair_setup(2);
    J a = one_answer();
    CHECK(a["outcome"].is("DONE") && a["result"]["mount_credit"].b && !config.drive_write_protected && ide_write_protected(),
          "the set-up: %s", a["outcome"].s.c_str());
    J m = ask1(reqt(3, "mount", lease_token, mount_args()), nullptr, 20);
    CHECK(m["outcome"].is("DONE") && m["result"]["used_credit"].b && is_mounted && ide_write_protected(), "the pair's mount: %s %s",
          m["outcome"].s.c_str(), m["err"]["why"].s.c_str());
}

// The lease ends while its set-up runs (Q3, X51): the set-up still ends
// (a running job is not cancelled, 6.1), and earns no credit.
static void test_credit_lease_ends_during_setup() {
    fresh("stage 4: the lease ends during the set-up");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args(980, 10, 17, "idp", "master", ",\"then_mount\":true")));
    // core 0 reads a session.close while core 1 is in the RESET- pulse
    sim_reset_hook = [](bool high) { if (!high) { host_send(reqt(3, "session.close", lease_token, "{}")); one_task(); one_task(); } };
    hold2(true, 250);
    sim_reset_hook = nullptr;
    pump(20);
    J a, cl;
    for (auto &l : host_lines()) { J x; if (parse_response(l, x)) { if (x["id"].n == 2) a = x; if (x["id"].n == 3) cl = x; } }
    CHECK(a["outcome"].is("DONE") && !a["result"]["mount_credit"].b && cl["outcome"].is("DONE") && !c.credit.live,
          "the lease ended during the set-up: %s, mount_credit %d, close %s", a["outcome"].s.c_str(), (int)a["result"]["mount_credit"].b,
          cl["outcome"].s.c_str());
}

// ide.c refusing the job's Ctrl+G by itself, nothing sent (a 91h seen
// between the pickup checks and the call): NOT_EXECUTED, never FAILED, and
// the state as it was.
static void test_ctrlg_refused_by_ide_job() {
    fresh("stage 4: setup.ctrlg refused by ide.c after its pickup");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args(980, 10, 17, "native")));
    ctl_test_before_ctrlg = [] { idp_written = true; CTL_EPOCH_BUMP(); };
    hold2(true, 250);
    ctl_test_before_ctrlg = nullptr;
    J r = one_answer();
    CHECK(is_refusal(r, "E_PRECONDITION") && r["err"]["why"].is("IDP_SENT_SINCE_BOOT") && sim.hw_resets == 0 && sim.commands == 0 &&
          ctl_setup_src == CTL_SRC_NONE, "ide.c's own refusal: %s %s %s", r["outcome"].s.c_str(), r["err"]["code"].s.c_str(),
          r["err"]["why"].s.c_str());
}

// menus_api_ctrlg when ide.c refuses before sending anything (bad heads;
// the power-up default mode after a 91h): the state is left as it was. The
// job's own checks never let either through; this is the layer under them.
static void test_api_ctrlg_refused_by_ide() {
    fresh("stage 4: menus_api_ctrlg refused by ide.c");
    power_drive();
    tui_setup_chs(980, 10, 17);
    for (int k = 0; k < 2; k++) {
        if (k) idp_written = true;
        std::string st0 = state2();
        uint8_t src0 = ctl_setup_src, st = 0;
        int n = sim.commands, hw = sim.hw_resets;
        int r = menus_api_ctrlg(980, k ? 10 : 17, 17, k == 1, &st);
        CHECK(r == (k ? IDE_MCHS_IDP_SENT : IDE_MCHS_BAD_ARGS) && config.cyls == 980 && config.heads == 10 && ctl_setup_src == src0 &&
              !strcmp(hdd_model_raw, "SIM DRIVE") && sim.commands == n && sim.hw_resets == hw,
              "%s: rc %d, geometry %u/%u, source %u", k ? "native after a 91h" : "heads 17", r, config.cyls, config.heads, ctl_setup_src);
        (void)st0;
    }
    idp_written = false;
    hooked();
}

// An LBA set-up (the TUI's picker) mounted over the API: its own act, the
// expect in LBA terms, the whole LBA size.
static void test_mount_lba() {
    fresh("stage 4: an LBA mount");
    power_drive();
    open_lease(1);
    ([] { use_lba_mode = true; cur_cyls = 980; cur_heads = 10; cur_spt = 17; total_lba_sectors = 166600; strcpy(hdd_model_raw, "SIM DRIVE"); sync_to_config(); ctl_setup_noted(CTL_SRC_TUI_DETECT_LBA); })();
    media_changed_waiting = false;
    hold2(false, 400);
    send_only(reqt(2, "mount", lease_token, "{\"expect\":{\"source\":\"TUI_DETECT_LBA\",\"mode\":\"LBA\",\"device\":\"MASTER\",\"lba_sectors\":166600}}"));
    CHECK(c.pres.waiting && c.pres.op == CTL_OP_MOUNT, "the LBA mount does not wait for its act");
    hold2(true, 250);
    J r = one_answer();
    CHECK(media_changed_waiting, "the API mount did not flag the media change for the host");
    CHECK(r["outcome"].is("DONE") && r["result"]["total_sectors"].n == 166600 && !r["result"]["used_credit"].b && is_mounted &&
          current_screen == SCREEN_MOUNTED, "the LBA mount: %s, %llu sectors", r["outcome"].s.c_str(),
          (unsigned long long)r["result"]["total_sectors"].n);
    // Its API_BEGIN and API_END around the MOUNT marker (5.4, X36).
    CHECK(api_entries(CTL_OP_MOUNT, 2, OBS_API_BEGIN) == 1 && api_entries(CTL_OP_MOUNT, 2, OBS_API_DONE) == 1,
          "the mount's API entries: begin %d, end %d", api_entries(CTL_OP_MOUNT, 2, OBS_API_BEGIN), api_entries(CTL_OP_MOUNT, 2, OBS_API_DONE));
    // A mount approved and then refused at its pickup (JP1 fitted meanwhile):
    // not mounted, no marker.
    fresh("stage 4: a mount refused at its pickup");
    power_drive();
    open_lease(1);
    tui_setup_chs(980, 10, 17);
    hold2(false, 400);
    send_only(reqt(2, "mount", lease_token, mount_args("TUI_DETECT_NORMAL")));
    core1_runs = false;
    hold2(true, 250);
    CHECK(ctl_job.state == CTL_JOB_QUEUED, "the mount not queued");
    uint32_t ms0 = ctl_mount_seq;
    jp1_fit(true);
    pump(3);
    core1_runs = true;
    pump(20);
    J p2 = one_answer();
    CHECK(is_refusal(p2, "E_LOCKED") && !is_mounted && ctl_mount_seq == ms0 && api_entries(CTL_OP_MOUNT, 2, OBS_API_BEGIN) == 0,
          "a mount at its pickup with JP1 fitted: %s, mounted %d", p2["err"]["code"].s.c_str(), (int)is_mounted);
    jp1_fit(false);
}

// A mount-name mismatch leaves a waiting request waiting (I-27, X68).
static void test_name_mismatch_leaves_waiting() {
    fresh("stage 4: a mount-name mismatch and a waiting request");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    J u = ask1(req(3, "unmount", named(ctl_mount_seq + 1).c_str()));
    J d = ask1(req(4, "detach", named(ctl_mount_seq + 1).c_str()));
    CHECK(is_refusal(u, "E_STALE") && is_refusal(d, "E_STALE") && c.pres.waiting && c.pres.id == 2,
          "a mismatch touched the waiting request: %s %s, waiting %d", u["err"]["code"].s.c_str(), d["err"]["code"].s.c_str(),
          (int)c.pres.waiting);
}

// A set-up that ends past its 75 s (SPEC-v1 5.3, 6.3: an overrun is a
// firmware defect): UNKNOWN, never DONE, and no credit even with then_mount.
// ide.c's own waits end it within about 58 s, so the clock is moved 80 s
// when RESET- is released.
static void test_ctrlg_deadline() {
    fresh("stage 4: setup.ctrlg past its max time");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args(980, 10, 17, "idp", "master", ",\"then_mount\":true")));
    sim_reset_hook = [](bool high) { if (high) mock_now_ns += 80000000000ull; };
    hold2(true, 250);
    sim_reset_hook = nullptr;
    J r = one_answer();
    CHECK(r["outcome"].is("UNKNOWN") && r["retry"].is("AFTER_STATUS") && !credit_live(now_ms()) && c.jl_present &&
          c.jl_ended - c.jl_started > 75000, "past 75 s: %s, credit %d, %llu ms", r["outcome"].s.c_str(), (int)credit_live(now_ms()),
          (unsigned long long)(c.jl_ended - c.jl_started));
}

// After an API mount (R-API-1 finding 2, I-24): the screen is MOUNTED, and
// the TUI's bus keys refuse while mounted, nothing sent.
static void test_mounted_keys() {
    fresh("stage 4: the TUI's bus keys while mounted");
    open_lease(1);
    pair_setup(2);
    (void)one_answer();
    J m = ask1(reqt(3, "mount", lease_token, mount_args()), nullptr, 20);
    CHECK(m["outcome"].is("DONE") && current_screen == SCREEN_MOUNTED, "screen %d",
          current_screen);
    // 3. With no mount the keys below are not the test's (Debug S would wait
    //    for a key for ever): stop here, the check above has failed.
    if (!is_mounted) return;
    int cmds = sim.commands, hw = sim.hw_resets, writes = sim.reg_writes;
    for (int k : { 'R', 'I', 'T', 'E', 'S' }) {
        current_screen = SCREEN_DEBUG;
        debug_key(k);
    }
    CHECK(sim.commands == cmds && sim.hw_resets == hw && sim.reg_writes == writes, "Debug keys while mounted: %d commands, %d resets",
          sim.commands - cmds, sim.hw_resets - hw);
    // Auto Detect's Enter and Y, and Load Defaults' Y: refused with a box.
    size_t t0 = tty.size();
    rx.push_back({ get_absolute_time(), ' ' });
    current_screen = SCREEN_MAIN;
    CHECK(!auto_detect_enter() && sim.hw_resets == hw && tty.find("Drive mounted", t0) != std::string::npos, "Auto Detect's Enter");
    current_screen = SCREEN_MOUNTED;
}

#endif
#if ATABOY_BOARD_JUMPERS
// An act whose FITTED edge came before the request arrived, its event
// completing while the request waits: approves nothing (X60, I-26).
static void test_banked_act() {
    fresh("stage 4: an edge before the arrival");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    jp2_fit(true);
    pump(100);                                          // the edge is 100 ms old
    uint32_t ev0 = bj_fit_events();
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    pump(300);                                          // the event completes now, the request waiting
    CHECK(bj_fit_events() == ev0 + 1 && c.pres.waiting && sim.cmd_count[0x91] == 0 && host_lines().empty(),
          "an edge from before the arrival approved it (events +%u, waiting %d)", bj_fit_events() - ev0, (int)c.pres.waiting);
    hold2(false, 250); hold2(true, 250);
    J r = one_answer();
    CHECK(r["outcome"].is("DONE"), "the act after the arrival: %s", r["outcome"].s.c_str());
    // ...and with the terminal closed while that old edge's event completes
    // (review delta-B LOW-3, P2): an edge from before the request approves
    // nothing, so it must not withdraw it either; NO_TERMINAL is asked at an act
    // that WOULD approve, not at any fit event.
    fresh("stage 4: an edge before the arrival, the terminal closed at its event");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    jp2_fit(true);
    pump(100);
    ev0 = bj_fit_events();
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    CHECK(c.pres.waiting, "not waiting on arrival");
    cdc_connected = false;
    pump(300);                                          // the old edge's event completes, no terminal
    CHECK(bj_fit_events() == ev0 + 1 && c.pres.waiting && host_lines().empty() && sim.cmd_count[0x91] == 0,
          "an edge from before the arrival withdrew it with the terminal closed (events +%u, waiting %d, %zu lines)",
          bj_fit_events() - ev0, (int)c.pres.waiting, host_lines().size());
    cdc_connected = true;
    hold2(false, 250); hold2(true, 250);
    r = one_answer();
    CHECK(r["outcome"].is("DONE"), "the act after the terminal returned: %s %s", r["outcome"].s.c_str(), r["err"]["why"].s.c_str());
    cdc_connected = false;
    // The edge sampled in the pass that reads the request: the same
    // millisecond, not after it (X60).
    fresh("stage 4: an edge in the arrival's own millisecond");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    jp2_fit(true);
    host_send(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    pump(300);
    CHECK(bj.edge == c.pres.arrival_ms && c.pres.waiting && sim.cmd_count[0x91] == 0 && host_lines().empty(),
          "an edge in the arrival's millisecond approved it (edge %u, arrival %u, waiting %d)", (unsigned)bj.edge,
          (unsigned)c.pres.arrival_ms, (int)c.pres.waiting);
}

// The 120 s from both sides (survivor: withdrawn a millisecond late): an
// event 119 999 ms after the arrival runs, one at 120 000 ms is too late.
static void test_presence_120s() {
    for (uint32_t k = 0; k < 2; k++) {
        fresh(k ? "stage 4: an event 120 000 ms after the arrival" : "stage 4: an event 119 999 ms after the arrival");
        power_drive();
        open_lease(1);
        hold2(false, 400);
        send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()), 1);
        CHECK(c.pres.waiting, "not waiting");
        if (!c.pres.waiting) continue;
        uint32_t A = c.pres.arrival_ms;
        if (!pump_to(A + 119798 + k)) continue;         // fitted here: the edge 1 ms on, the event 201 ms on
        jp2_fit(true);
        pump(210);
        J r = one_answer();
        CHECK(k ? is_refusal(r, "E_PRESENCE_REQUIRED") && sim.cmd_count[0x91] == 0 : r["outcome"].is("DONE") && sim.cmd_count[0x91] == 1,
              "an event %u ms after the arrival: %s %s (edge %u)", 119999 + k, r["outcome"].s.c_str(), r["err"]["code"].s.c_str(),
              (unsigned)(bj.edge - A));
    }
}

// The idle timer is paused only by the lease's own request or job (6.1):
// another's job in the mailbox does not keep it alive.
static void test_idle_other_job() {
    fresh("stage 4: the idle timer and a job of no lease");
    power_drive();
    open_lease(1, 30);
    pump(5000);
    core1_runs = false;
    host_send(req(2, "taskfile.snapshot"));
    pump(5);
    CHECK(ctl_job.state == CTL_JOB_QUEUED, "not queued");
    pump(26000);                                        // 31 s since the lease was last used
    CHECK(!c.lease.held, "another's job paused the lease's idle timer");
    core1_runs = true;
    pump(20);
    host_lines();
}

// The answers owed come before any new line's (4.13): a session.close and
// a status sent back to back while a request waits.
static void test_owed_order() {
    fresh("stage 4: the answers owed before a new line's");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    host_send(reqt(3, "session.close", lease_token, "{}") + req(4, "status"));
    pump(30);
    std::vector<uint64_t> ids;
    for (auto &l : host_lines()) { J x; if (parse_response(l, x)) ids.push_back(x["id"].n); }
    CHECK(ids.size() == 3 && ids[0] == 2 && ids[1] == 3 && ids[2] == 4, "%zu answers, in the order %llu %llu %llu", ids.size(),
          ids.size() > 0 ? (unsigned long long)ids[0] : 0ull, ids.size() > 1 ? (unsigned long long)ids[1] : 0ull,
          ids.size() > 2 ? (unsigned long long)ids[2] : 0ull);
}

// Core 0 ends the lease between core 1's pickup and its checks (R-API-1
// finding 9, X55): the lease check at pickup refuses it, nothing is sent.
static void test_lease_end_at_pickup() {
    fresh("stage 4: the lease ends between the pickup and its checks");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    core1_runs = false;
    hold2(true, 250);                                   // approved, queued; core 1 away
    CHECK(ctl_job.state == CTL_JOB_QUEUED, "not queued");
    host_send(reqt(3, "session.close", lease_token, "{}"));
    ctl_test_after_cas = [] { one_task(); one_task(); };   // core 0 takes the close meanwhile
    ctl_core1_idle(core1_screen);
    ctl_test_after_cas = nullptr;
    core1_runs = true;
    pump(20);
    J w, cl;
    for (auto &l : host_lines()) { J x; if (parse_response(l, x)) { if (x["id"].n == 2) w = x; if (x["id"].n == 3) cl = x; } }
    CHECK(is_refusal(w, "E_LEASE_REVOKED") && w["err"]["by"].is("CLOSED") && cl["outcome"].is("DONE") &&
          sim.cmd_count[0x91] == 0 && sim.hw_resets == 0, "a lease ended at the pickup: %s by %s, close %s, 91h %d",
          w["err"]["code"].s.c_str(), w["err"]["by"].s.c_str(), cl["outcome"].s.c_str(), sim.cmd_count[0x91]);
}

// The same two rules where the millisecond clock wraps (every 2^32 ms, 49.7
// days up): an edge just before the wrap is not after an arrival just past
// it, and an edge just past the wrap is after an arrival just before it.
static void test_presence_wrap() {
    const uint64_t wrap_ns = 4294967296ull * 1000000ull;
    // A banked act: the edge 100 ms before the wrap, the request 50 ms after it.
    mock_now_ns = wrap_ns - 3000000000ull;              // power_up adds 1 s: 2 s before the wrap
    fresh("stage 4: an edge before the clock's wrap, the arrival after it");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    if (!pump_to(0u - 100u)) return;                    // to 100 ms before the wrap
    jp2_fit(true);
    pump(150);                                           // 50 ms past it
    CHECK(t_ms() < 1000, "not past the wrap (%u)", (unsigned)t_ms());
    uint32_t ev0 = bj_fit_events();
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    pump(300);                                           // the event completes, the request waiting
    CHECK(bj_fit_events() == ev0 + 1 && c.pres.waiting && sim.cmd_count[0x91] == 0 && host_lines().empty(),
          "an edge from before the wrap approved a request from after it (events +%u, waiting %d)", bj_fit_events() - ev0,
          (int)c.pres.waiting);
    // A true act: the request 50 ms before the wrap, the edge 100 ms after it.
    mock_now_ns = wrap_ns - 3000000000ull;
    fresh("stage 4: an arrival before the clock's wrap, the edge after it");
    power_drive();
    open_lease(1);
    hold2(false, 400);
    if (!pump_to(0u - 60u)) return;                     // to 60 ms before the wrap
    send_only(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()), 10);
    CHECK(c.pres.waiting && c.pres.arrival_ms > 0xFFFF0000u, "no request just before the wrap (%u)", (unsigned)c.pres.arrival_ms);
    hold2(false, 150);                                   // 100 ms past the wrap
    hold2(true, 250);
    J r = one_answer();
    CHECK(r["outcome"].is("DONE") && sim.cmd_count[0x91] == 1, "an act after the wrap for a request before it: %s %s",
          r["outcome"].s.c_str(), r["err"]["code"].s.c_str());
}

// The credit is the mount's only (X50): a second set-up with a credit live
// waits for its own act.
static void test_credit_mount_only() {
    fresh("stage 4: the credit is for a mount only");
    open_lease(1);
    pair_setup(2);
    J a = one_answer();
    CHECK(a["result"]["mount_credit"].b && credit_live(now_ms()), "no credit");
    send_only(reqt(3, "setup.ctrlg", lease_token, ctrlg_args(990, 10, 17)));
    CHECK(c.pres.waiting && c.pres.op == CTL_OP_SETUP_CTRLG && ctl_job.state == CTL_JOB_EMPTY && host_lines().empty(),
          "a set-up ran on the mount's credit");
}

// The layers of setup.ctrlg's ranges (X17): the schema refuses (E_RANGE),
// and ctl_mreq_check refuses on its own a request the schema never saw.
static void test_ctrlg_layers() {
    fresh("stage 4: setup.ctrlg's ranges, each layer");
    ctlj_doc_t d;
    for (const char *args : { "{\"cyls\":0,\"heads\":10,\"spt\":17,\"mode\":\"idp\",\"device\":\"master\",\"drive_power_cycled\":true}",
                              "{\"cyls\":980,\"heads\":17,\"spt\":17,\"mode\":\"idp\",\"device\":\"master\",\"drive_power_cycled\":true}",
                              "{\"cyls\":980,\"heads\":10,\"spt\":0,\"mode\":\"idp\",\"device\":\"master\",\"drive_power_cycled\":true}" }) {
        std::string line = reqt(1, "setup.ctrlg", "00112233445566778899aabbccddeeff", args);
        line.pop_back();
        ctl_req_t r;
        CHECK(ctlj_parse(line.data(), line.size(), &d) == CTLJ_OK && ctl_check_request(&d, &r) == CTL_E_RANGE,
              "schema: %s", args);
    }
    struct { uint16_t cy; uint8_t h, s; const char *field; } bad[] = { { 0, 10, 17, "cyls" }, { 980, 17, 17, "heads" },
                                                                        { 980, 0, 17, "heads" }, { 980, 10, 0, "spt" } };
    for (auto &b : bad) {
        ctl_mreq_t m = {};
        m.cyls = b.cy; m.heads = b.h; m.spt = b.s; m.dev_base = 0xA0;
        const char *field, *why;
        int e = ctl_mreq_check(CTL_OP_SETUP_CTRLG, &m, &field, &why);
        CHECK(e == CTL_E_RANGE && field && !strcmp(field, b.field), "ctl_mreq_check %u/%u/%u: %d %s", b.cy, b.h, b.s, e,
              field ? field : "-");
    }
    ctl_mreq_t ok = {};
    ok.cyls = 980; ok.heads = 16; ok.spt = 255; ok.dev_base = 0xA0;
    const char *f, *w;
    CHECK(ctl_mreq_check(CTL_OP_SETUP_CTRLG, &ok, &f, &w) == CTL_E_NONE, "980/16/255 refused");
}
#endif
#if ATABOY_BOARD_JUMPERS
// Without jumpers there is no Tier M (8.1): every Tier M request E_LOCKED.
#else
static void test_tier_m_no_jumpers() {
    fresh("stage 4: no jumpers, no Tier M");
    power_drive();
    open_lease(1);
    J r = ask1(reqt(2, "setup.ctrlg", lease_token, ctrlg_args()));
    CHECK(is_refusal(r, "E_LOCKED") && r["err"]["why"].is("JUMPERS_UNSUPPORTED") && !c.pres.waiting &&
          sim.cmd_count[0x91] == 0, "setup.ctrlg: %s %s", r["err"]["code"].s.c_str(), r["err"]["why"].s.c_str());
    tui_setup_chs(980, 10, 17);
    J m = ask1(reqt(3, "mount", lease_token, mount_args("TUI_DETECT_NORMAL")));
    CHECK(is_refusal(m, "E_LOCKED") && !is_mounted, "mount: %s", m["err"]["code"].s.c_str());
    // hello says the same: every Tier M op LOCKED for the missing jumpers, with a lease held.
    J h = ask1(req(4, "hello"), nullptr, 20);
    int seen = 0, wrong = 0;
    for (int op = 0; op < CTL_OP_COUNT; op++) {
        if (ctl_ops[op].tier != CTL_TIER_M) continue;
        seen++;
        const J &cp = h["result"]["caps"][ctl_ops[op].name];
        if (!cp["state"].is("LOCKED") || !cp["why"].is("JUMPERS_UNSUPPORTED")) wrong++;
    }
    CHECK(seen >= 2 && wrong == 0, "hello without jumpers: %d Tier M ops, %d not LOCKED for JUMPERS_UNSUPPORTED", seen, wrong);
    // Nothing waited, nothing was queued, nothing reached the drive.
    CHECK(!c.pres.waiting && ctl_job.state == CTL_JOB_EMPTY && sim.cmd_count[0x91] == 0 && sim.hw_resets == 0, "nothing sent");
}
#endif

// ---- 13b. after a tripwire stop core 1 shows the trip screen, not the presence row ----
// (review A LOW-4). Runs at the end of the "core1" run above: the loop had published
// MAIN, then trip_screen_forever() takes over and never returns to the loop's idle
// point, which is the only place ctl_tui_state was ever updated. It must say MODAL
// from then on, or a guarded request passes the NO_TERMINAL check while the person
// sees the trip screen and no row (and the approved job is never picked up).
#if ATABOY_BOARD_JUMPERS
static void core1_trip_hook() {
    CHECK(ctl_tui_state == CTL_TUI_MODAL, "after the trip screen took over, ctl_tui_state is %u, not MODAL (%u)", ctl_tui_state, CTL_TUI_MODAL);
    (void)host_lines();                             // the answer to the job that ran before the trip screen
    power_drive();
    config.drive_write_protected = true;
    open_lease(70);
    hold2(false, 400);
    J r = ask1(reqt(71, "setup.ctrlg", lease_token, ctrlg_args()));
    CHECK(is_refusal(r, "E_PRECONDITION") && r["err"]["why"].is("NO_TERMINAL") && !c.pres.waiting && ctl_job.state == CTL_JOB_EMPTY,
          "a guarded request with the trip screen shown: %s %s, waiting %d", r["outcome"].s.c_str(), r["err"]["why"].s.c_str(), (int)c.pres.waiting);
    J st = status4(72);
    CHECK(st["result"]["tui"]["state"].is("MODAL"), "status.tui.state after the trip screen: %s", st["result"]["tui"]["state"].s.c_str());
    printf("%d checks, %d failed\n", checks, failures);
    exit(failures > 255 ? 255 : failures);
}
static void core1_trip_stage() {
    cdc_connected = true;
    on_idle = core1_trip_hook;
    idle_after_ns = 0;
    trip_screen_forever();
}
#endif

// ---- 17. core 1's real loop with keys (its own run: test_ctl core1-tui) -----
// The hooks inside core1_entry's key handling (the Features toggles,
// confirm-3), the F: Force refusal after identify.forbid, and an API
// unmount taken at the MOUNTED screen's idle point. Keys arrive on the mock
// clock; core 0's passes come from core 1's own input polls (core0_tick),
// except while core0_hold says core 0 is inside a long USB callback.
static uint64_t T(double s) { return (uint64_t)(s * 1e6); }        // us of the run
static void key_at(double s, const char *k) {
    uint64_t t = T(s);
    for (const char *p = k; *p; p++) { rx.push_back({ t, *p }); t += 200; }
}
static void at(double s, std::function<void()> f) { timeline.push_back({ T(s), f }); }
static uint32_t mark_e;
static int mark_91, mark_hw;
static J last_answer(bool *ok) {
    std::vector<std::string> r = host_lines();
    J j;
    *ok = r.size() == 1 && parse_response(r[0], j) && envelope_problem(j).empty();
    return j;
}
static void core1_tui_done() {
    CHECK(c.epoch_missed == 0, "core1-tui: %u change(s) with no hook", (unsigned)c.epoch_missed);
    CHECK(rxi == rx.size(), "core1-tui: %zu keys left unread", rx.size() - rxi);
    printf("%d checks, %d failed\n", checks, failures);
    exit(failures > 255 ? 255 : failures);
}
static void test_core1_tui() {
    setvbuf(stdout, nullptr, _IONBF, 0);            // a run that stops early still shows how far it got
    fresh("core 1's loop with keys");
    sim.nsect = 980u * 10 * 17;
    core1_runs = false;                             // core 1 is its real loop, not the stepper
    cdc_connected = true;
    config.drive_write_protected = true;            // the settings' default
    fingerprint(&c.fp); c.fp_epoch = ctl_epoch;
    uint64_t base = mock_now_ns / 1000;
    auto B = [base](double s) { return (base + T(s)) / 1e6; };      // seconds, absolute
    // 1. Features, Write Protect: Enter turns it off (a TUI change: epoch, last_change).
    key_at(B(1.0), "\x1b[C");                       // main menu: to Features
    key_at(B(1.5), "\r");
    at(B(1.9), [] { mark_e = ctl_epoch; CHECK(current_screen == SCREEN_FEATURES && config.feat_selected == 0, "not on Features"); });
    key_at(B(2.0), "\r");
    at(B(2.5), [] {
        CHECK(!config.drive_write_protected && ctl_wp_last == CTL_WP_TUI && ctl_epoch == mark_e + 1,
              "WP toggle: wp %d last %u epoch +%u", config.drive_write_protected, ctl_wp_last, (unsigned)(ctl_epoch - mark_e));
    });
    // 2. On and off again while core 0 is away (a 1 s USB callback): the
    // cross-check sees nothing move, the hooks count both.
    at(B(2.9), [] { core0_hold = true; mark_e = ctl_epoch; });
    key_at(B(3.0), "\r");
    key_at(B(3.1), "\r");
    at(B(3.6), [] {
        CHECK(ctl_epoch == mark_e + 2 && !config.drive_write_protected, "a change and its reversal: epoch +%u",
              (unsigned)(ctl_epoch - mark_e));
        core0_hold = false;
    });
    at(B(3.8), [] { CHECK(c.epoch_missed == 0 && ctl_epoch == mark_e + 2, "after core 0 came back: +%u", (unsigned)(ctl_epoch - mark_e)); });
    // 3. Pipelined Reads (the sixth row): on, then off.
    key_at(B(4.0), "\x1b[B\x1b[B\x1b[B\x1b[B\x1b[B");
    at(B(4.4), [] { mark_e = ctl_epoch; CHECK(config.feat_selected == 5, "row %u", config.feat_selected); });
    key_at(B(4.5), "\r");
    at(B(4.9), [] { CHECK(config.pipelined_reads && ctl_epoch == mark_e + 1, "Pipelined Reads on: epoch +%u", (unsigned)(ctl_epoch - mark_e)); });
    key_at(B(5.0), "\r");
    at(B(5.4), [] { CHECK(!config.pipelined_reads && ctl_epoch == mark_e + 2, "Pipelined Reads off"); });
    key_at(B(5.5), "\x1b");                         // back to the main menu (on Features)
    // 4. Load Setup Defaults: write protect back on, by the TUI.
    key_at(B(6.0), "\x1b[B");                       // Features -> Load Defaults
    key_at(B(6.3), "\r");
    at(B(6.6), [] {
        mark_e = ctl_epoch;
        CHECK(current_screen == SCREEN_CONFIRM && !config.drive_write_protected, "no Load Defaults question");
        ctl_wp_last = CTL_WP_BOOT;                  // so the check below sees Load Defaults set it
    });
    key_at(B(6.7), "y");
    at(B(7.2), [] {
        CHECK(config.drive_write_protected && ctl_wp_last == CTL_WP_TUI && ctl_epoch == mark_e + 1 && saves == 1,
              "Load Defaults: wp %d last %u epoch +%u", config.drive_write_protected, ctl_wp_last, (unsigned)(ctl_epoch - mark_e));
    });
    key_at(B(7.5), "\x1b[A\x1b[D");                 // Load Defaults -> Features -> Auto Detect
    // 5. Auto Detect on a drive whose IDENTIFY fails: the error box offers
    // F: Force. The API forbids IDENTIFY while it is open. (The Force entry
    // itself is not driven here: its picker computes the LARGE row from the
    // zero IDENTIFY words, 0 heads, and x86 traps on the division where the
    // RP2350 gives 0; menus.c get_large_geometry.)
    at(B(7.9), [] { sim.identify_err_drq = true; CHECK(current_screen == SCREEN_MAIN && config.main_selected == 0, "not on Auto Detect"); });
    key_at(B(8.0), "\r");
    key_at(B(8.5), "y");
    at(B(15.0), [] {
        CHECK(show_detect_result && force_detect, "no IDENTIFY-failed box with F: Force (%d %d)", show_detect_result, force_detect);
        host_send(req(50, "identify.forbid"));
        mark_91 = sim.cmd_count[0x91]; mark_hw = sim.hw_resets;
    });
    at(B(15.5), [] {
        bool ok;
        J r = last_answer(&ok);
        CHECK(ok && r["outcome"].is("DONE") && ide_identify_forbidden(), "identify.forbid during the box");
    });
    // F now: refused, and the box says why (R-API-1 finding 6).
    key_at(B(16.0), "f");
    at(B(16.5), [] {
        CHECK(show_detect_result && !force_detect && strstr(hdd_status_text, "Forbidden by ATAboy Control") &&
              strcmp(hdd_model_raw, "Manually Forced Drive") && ctl_setup_src != CTL_SRC_TUI_FORCE && sim.cmd_count[0x91] == mark_91,
              "F: Force after identify.forbid: force %d, text %s", force_detect, hdd_status_text);
    });
    key_at(B(17.0), "\r");                          // dismiss
    // 6. Auto Detect's Enter now: refused at once, the new box, no RESET-.
    key_at(B(18.0), "\r");
    at(B(18.5), [] {
        CHECK(tty.find(API_FORBIDDEN_BOX) != std::string::npos && current_screen == SCREEN_MAIN && sim.hw_resets == mark_hw,
              "Auto Detect after identify.forbid: resets %d", sim.hw_resets - mark_hw);
    });
    key_at(B(19.0), " ");                           // the box's key
    // 7. Mount through confirm-3 (the hook there), then an API unmount taken
    // at the MOUNTED screen's idle point: the TUI follows to the main menu.
    at(B(20.0), [] {
        use_lba_mode = false; cur_cyls = 980; cur_heads = 10; cur_spt = 17; total_lba_sectors = 0;
        strcpy(hdd_model_raw, "SIM DRIVE"); sync_to_config(); ctl_setup_noted(CTL_SRC_TUI_DETECT_NORMAL);
    });
    key_at(B(20.5), "\x1b[B");                      // to Mount
    key_at(B(21.0), "\r");
    at(B(21.4), [] { mark_e = ctl_epoch; CHECK(current_screen == SCREEN_CONFIRM, "Mount did not ask"); });
    key_at(B(21.5), "y");
    at(B(22.0), [] {
        CHECK(is_mounted && current_screen == SCREEN_MOUNTED && ctl_mount_seq == 1 && ctl_epoch == mark_e + 1,
              "confirm-3: mounted %d, epoch +%u", (int)is_mounted, (unsigned)(ctl_epoch - mark_e));
        host_send(req(60, "unmount", named(1).c_str()));
    });
    at(B(23.0), [] {
        bool ok;
        J r = last_answer(&ok);
        CHECK(ok && r["outcome"].is("DONE") && r["result"]["changed"].b && !is_mounted && current_screen == SCREEN_MAIN,
              "API unmount from the MOUNTED screen: %s, screen %d", r["outcome"].s.c_str(), current_screen);
        CHECK(ctl_tui_state == CTL_TUI_MAIN, "published state %u", ctl_tui_state);
    });
#if ATABOY_BOARD_JUMPERS
    // 8. Stage 3, JP1 and the TUI's Write Protect toggle (SPEC-v1 8.2): open,
    // the toggle works both ways; fitted, Enter shows the box and changes
    // nothing; taken off, "Disabled" does not come back by itself; fitted
    // while core 0 is in a long callback, the TUI still sees it (finding 12).
    key_at(B(24.0), "\x1b[A\x1b[C");               // Mount -> Auto Detect -> Features
    key_at(B(24.3), "\r");
    at(B(24.8), [] {
        CHECK(current_screen == SCREEN_FEATURES && config.feat_selected == 0 && config.drive_write_protected,
              "JP1: not on Features' Write Protect (screen %d row %u)", current_screen, config.feat_selected);
    });
    key_at(B(25.0), "\r");                          // Enabled -> Disabled, JP1 open
    at(B(25.4), [] {
        CHECK(!config.drive_write_protected && ctl_wp_last == CTL_WP_TUI, "JP1 open: the toggle did not work");
        mark_e = ctl_epoch;
        mock_jumper_fitted[0] = true;                   // the owner fits JP1
    });
    at(B(25.8), [] {
        CHECK(config.drive_write_protected && bj_jp1() == BJ_FITTED && ctl_epoch == mark_e + 1,
              "JP1 fitted: wp %d jp1 %u epoch +%u", config.drive_write_protected, bj_jp1(), (unsigned)(ctl_epoch - mark_e));
        mark_91 = (int)tty.size(); mark_e = ctl_epoch;
    });
    key_at(B(26.0), "\r");                          // Enabled -> Disabled: refused
    at(B(26.4), [] {
        CHECK(tty.find(JP1_LOCKED_BOX, (size_t)mark_91) != std::string::npos && config.drive_write_protected && ctl_epoch == mark_e,
              "JP1 fitted: toggle not refused (wp %d, box %d)", config.drive_write_protected,
              tty.find(JP1_LOCKED_BOX, (size_t)mark_91) != std::string::npos);
    });
    key_at(B(26.5), " ");                           // the box's key
    at(B(27.0), [] { mock_jumper_fitted[0] = false; });
    at(B(27.5), [] {
        CHECK(bj_jp1() == BJ_OPEN && config.drive_write_protected && current_screen == SCREEN_FEATURES,
              "JP1 off: jp1 %u, wp %d (Disabled came back)", bj_jp1(), config.drive_write_protected);
    });
    key_at(B(28.0), "\r");                          // the human can now
    at(B(28.4), [] {
        CHECK(!config.drive_write_protected, "JP1 off: the toggle refused");
        core0_hold = true;                              // core 0 in a long USB callback
        mock_jumper_fitted[0] = true;
        mark_91 = (int)tty.size();
    });
    key_at(B(29.0), "\r");
    at(B(29.4), [] {
        CHECK(tty.find(JP1_LOCKED_BOX, (size_t)mark_91) != std::string::npos && config.drive_write_protected &&
              bj_jp1() == BJ_FITTED, "JP1 fitted with core 0 away: the TUI did not see it (wp %d)", config.drive_write_protected);
    });
    key_at(B(29.5), " ");
    at(B(30.0), [] { core0_hold = false; mock_jumper_fitted[0] = false; });
    at(B(31.0), [] { CHECK(bj_jp1() == BJ_OPEN && config.drive_write_protected, "JP1 off again"); });
    // 9. Stage 4, the presence row (SPEC-v1 4.14): a guarded request waiting
    // for its act shows on the main menu, row 13 from column 43, drawn by the
    // TUI's loop; withdrawn (session.close), the row is blanked.
    key_at(B(31.2), "\x1b");                        // Features -> the main menu
    at(B(31.5), [] { host_send(req(70, "session.open", "{\"client\":\"tui\",\"tui_lock\":false,\"idle_s\":300}")); });
    at(B(32.0), [] {
        bool ok;
        J r = last_answer(&ok);
        CHECK(ok && r["outcome"].is("DONE") && current_screen == SCREEN_MAIN, "presence row: session.open %s, screen %d",
              r["outcome"].s.c_str(), current_screen);
        lease_token = r["result"]["token"].s;
        mark_91 = (int)tty.size();
        host_send(reqt(71, "setup.ctrlg", lease_token, ctrlg_args()));
    });
    at(B(33.0), [] {
        char want[64];
        snprintf(want, sizeof want, "\033[13;43H" FG_YELLOW "JP2? setup.ctrlg #71 %.8s", c.pres.waiting ? hex(c.pres.digest, 4).c_str() : "none");
        CHECK(c.pres.waiting && tty.find(want, (size_t)mark_91) != std::string::npos, "presence row not drawn");
        mark_91 = (int)tty.size();
    });
    key_at(B(33.2), "\x0c");                        // Ctrl+L: a full frame blanks row 13, the row comes back
    at(B(33.5), [] {
        char want[64];
        snprintf(want, sizeof want, "\033[13;43H" FG_YELLOW "JP2? setup.ctrlg #71 %.8s", c.pres.waiting ? hex(c.pres.digest, 4).c_str() : "none");
        CHECK(c.pres.waiting && tty.find(want, (size_t)mark_91) != std::string::npos, "presence row not drawn again after Ctrl+L");
        mark_91 = (int)tty.size();
        host_send(reqt(72, "session.close", lease_token, "{}"));
    });
    at(B(34.0), [] {
        std::vector<std::string> r = host_lines();
        std::string blank = "\033[13;43H" FG_YELLOW + std::string(37, ' ');
        CHECK(r.size() == 2 && !c.pres.waiting && tty.find(blank, (size_t)mark_91) != std::string::npos,
              "presence row: %zu answers, row blanked %d", r.size(), tty.find(blank, (size_t)mark_91) != std::string::npos);
    });
    const double t10 = 35.0;
#else
    const double t10 = 25.0;
#endif
    // 10. Stage 4, R-API-1 finding 2 (SPEC-v1 5.3, I-24): Load Defaults' Y
    // and Auto Detect's Y with a drive mounted (a question left open when a
    // mount came: no path today makes one, so the state is set here) are
    // refused with the box, nothing saved, nothing sent, the screen MOUNTED.
    // (identify.forbid from part 5 is still on: Auto Detect's own refusal
    // would say so with another box and leave the main menu.)
    static int saves10, hw10, ec10;
    at(B(t10), [] {
        is_mounted = true; hooked();
        current_screen = SCREEN_CONFIRM; confirm_type = 0; needs_full_redraw = true;
        saves10 = saves; hw10 = sim.hw_resets; ec10 = sim.cmd_count[0xEC]; mark_91 = (int)tty.size();
    });
    key_at(B(t10 + 0.3), "y");
    key_at(B(t10 + 0.7), " ");                      // the box's key
    at(B(t10 + 1.0), [] {
        CHECK(tty.find(API_MOUNTED_BOX, (size_t)mark_91) != std::string::npos && saves == saves10 && sim.hw_resets == hw10 &&
              current_screen == SCREEN_MOUNTED && is_mounted, "Load Defaults' Y while mounted: saves +%d, screen %d",
              saves - saves10, current_screen);
        current_screen = SCREEN_CONFIRM; confirm_type = 6; needs_full_redraw = true;
        mark_91 = (int)tty.size();
    });
    key_at(B(t10 + 1.3), "y");
    key_at(B(t10 + 1.7), " ");
    at(B(t10 + 2.0), [] {
        CHECK(tty.find(API_MOUNTED_BOX, (size_t)mark_91) != std::string::npos && sim.hw_resets == hw10 &&
              sim.cmd_count[0xEC] == ec10 && current_screen == SCREEN_MOUNTED && is_mounted,
              "Auto Detect's Y while mounted: resets +%d, screen %d", sim.hw_resets - hw10, current_screen);
        is_mounted = false; hooked();
        current_screen = SCREEN_MAIN; needs_full_redraw = true;
    });
    // 11. Every Debug key on the Debug screen with a drive mounted, through
    // the real loop (review B M-2): the one place the API build's setup screen
    // differs from the old one on a key the old one ran (README items 33, 34,
    // 36). A drive can be mounted under an open Debug screen only by a race
    // (an API mount leaves the screen on MOUNTED), so the state is set here,
    // as in part 10. Each key says so and sends nothing.
    static int cmds11, hw11, regs11;
    static size_t mark11;
    at(B(t10 + 2.2), [] {
        is_mounted = true; hooked();
        current_screen = SCREEN_DEBUG; needs_full_redraw = true;
        cmds11 = sim.commands; hw11 = sim.hw_resets; regs11 = sim.reg_writes; mark11 = tty.size();
    });
    key_at(B(t10 + 2.6), "r");
    key_at(B(t10 + 3.0), "i");
    key_at(B(t10 + 3.4), "t");
    key_at(B(t10 + 3.8), "e");
    key_at(B(t10 + 4.2), "s");
    at(B(t10 + 4.6), [] {
        int shown = 0;
        for (size_t p = tty.find(API_MOUNTED_TEXT, mark11); p != std::string::npos; p = tty.find(API_MOUNTED_TEXT, p + 1)) shown++;
        CHECK(shown == 5 && sim.commands == cmds11 && sim.hw_resets == hw11 && sim.reg_writes == regs11 && current_screen == SCREEN_DEBUG &&
              is_mounted, "Debug keys while mounted: refusal shown %d times, %d commands, %d resets, %d register writes, screen %d", shown,
              sim.commands - cmds11, sim.hw_resets - hw11, sim.reg_writes - regs11, current_screen);
        is_mounted = false; hooked();
        current_screen = SCREEN_MAIN; needs_full_redraw = true;
    });
    idle_after_ns = (base + T(t10 + 5.0)) * 1000;
    on_idle = core1_tui_done;
    core0_in_loop = true;
    core1_entry();
}

// ---- serve: the Control port on stdin/stdout, for ctl_e2e.py ------------------
// jp (stage 3, ctl_e2e.py): 1, JP1 fitted after the mount; 2, GPIO31's pad
// set up wrong (both pulls on) after it; 3 (stage 4), nothing mounted and
// nothing set up. A line "#ACT" (stage 4) is not the port's: it is the
// human's JP2 act (open 250 ms, fitted 250 ms), taken here, answered with
// nothing.
static void serve(int jp = 0) {
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    power_up();
    cdc_connected = true;       // a terminal on the main menu: a guarded request waits for its act only then (2026-09-29)
    port.got.clear();
    // A mounted drive's worth of log, so log.read has something to return.
    for (int i = 0; i < 5; i++) obs_marker(OBS_EV_MOUNT, 9, 0, 0, 0, (uint16_t)i, 0xA0);
    // Stage 2: a drive set up and mounted through the menus' own path, so
    // unmount and detach have something to end (mount_seq 1).
    sim.nsect = 980u * 10 * 17;
    config.drive_write_protected = false; ctl_wp_last = CTL_WP_TUI; hooked();
    config.pipelined_reads = true; hooked();
    if (jp != 3) {
        tui_setup_chs(980, 10, 17);
        tui_mount();
    }
    pump(2);
#if ATABOY_BOARD_JUMPERS
    if (jp == 1) { mock_jumper_fitted[0] = true; pump(2); }
    if (jp == 2) { mock_pads.io[31] |= 0x4u; pump(2); }
#else
    (void)jp;
#endif
    static char buf[4096];
    std::string cur;
    for (;;) {
        size_t n = fread(buf, 1, 1, stdin);        // one byte at a time: never waits for more than the host sent
        if (n == 0) break;
        cur.push_back(buf[0]);
        if (buf[0] != '\n') continue;
        if (cur == "#ACT\n") {
#if ATABOY_BOARD_JUMPERS
            mock_jumper_fitted[1] = false; pump(250);
            mock_jumper_fitted[1] = true; pump(250);
            mock_jumper_fitted[1] = false; pump(30);
#endif
        } else {
            for (char ch : cur) port.rx.push_back((uint8_t)ch);
            pump(30);
        }
        cur.clear();
        if (!port.got.empty()) { fwrite(port.got.data(), 1, port.got.size(), stdout); fflush(stdout); port.got.clear(); }
    }
}

int main(int argc, char **argv) {
    ide_hw_init();
    if (argc > 1 && !strcmp(argv[1], "serve")) { serve(); return 0; }
    if (argc > 1 && !strcmp(argv[1], "serve-jp1")) { serve(1); return 0; }
    if (argc > 1 && !strcmp(argv[1], "serve-pads")) { serve(2); return 0; }
    if (argc > 1 && !strcmp(argv[1], "serve-idle")) { serve(3); return 0; }
    if (argc > 1 && !strcmp(argv[1], "core1")) test_core1_loop();          // does not return
    if (argc > 1 && !strcmp(argv[1], "core1-confirm")) test_core1_confirm(); // does not return
    if (argc > 1 && !strcmp(argv[1], "core1-tui")) test_core1_tui();       // does not return
    test_parser();
    test_writer_sha();
    test_op_table();
    test_hello();
    test_envelope();
    test_lines();
    test_rate();
    test_writer_never_blocks();
    test_replay_table();
    test_replay_dispatch();
    test_status();
    test_config_get();
    test_pages();
    test_identify_cached();
    test_debug_e_selftest_chip();
    test_taskfile();
    test_garbage_changes_nothing();
    test_wp_on();
    test_pipelined_off();
    test_identify_forbid();
    test_forbid_tui_prechecks();
    test_unmount();
    test_unmount_pickup();
    test_unmount_native();
    test_detach();
    test_detach_busy();
    test_ratchet_no_terminal();
    test_epoch_hooks();
    test_epoch_inside_ctrlg();
    test_setup_source();
#if ATABOY_BOARD_JUMPERS
    test_jp_pads_boot();
    test_jp_pad_faults();
    test_jp1_lock();
    test_debounce();
    test_jp1_between_passes();
    test_jp2_events();
    test_jp2_changes_nothing();
    test_tier_m_caps();
#endif
#if ATABOY_BOARD_JUMPERS
    test_lease();
    test_token_entropy();
    test_token_near_miss();
    test_session_open_bounds();
    test_job_last_no_response();
    test_token_confinement();
    test_session_open_replay();
    test_presence_label();
    test_presence();
    test_withdrawal();
    test_tier_m_arrival();
    test_pair();
    test_pickup();
    test_tier_m_no_terminal();
    test_ctrlg_deadline();
    test_credit_lease_ends_during_setup();
    test_name_mismatch_leaves_waiting();
    test_mount_lba();
    test_api_ctrlg_refused_by_ide();
    test_ctrlg_refused_by_ide_job();
    test_mounted_keys();
    test_banked_act();
    test_presence_120s();
    test_idle_other_job();
    test_owed_order();
    test_lease_end_at_pickup();
    test_presence_wrap();
    test_credit_mount_only();
    test_mount_wp_off();
    test_no_terminal_refusal();
    test_no_terminal_at_act();
    test_ctrlg_layers();
#else
    test_tier_m_no_jumpers();
#endif
    test_detach_lease();
    fresh("end");                                   // the last test's epoch cross-check
    printf("%d checks, %d failed\n", checks, failures);
    return failures > 255 ? 255 : failures;
}
