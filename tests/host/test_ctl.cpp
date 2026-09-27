// Host tests for the ATAboy Control API, stage 1 (palimpsest api/SPEC-v1.md,
// section 12.1): the parser and writer, the op table against a hand-written
// copy of SPEC-v1 4.0, the envelope and its order of checks, the line layer
// (limits, CR, DTR, the partial-line timeout), the rate window, the
// non-blocking writer, the replay table (digest-keyed, E_ID_REUSED,
// E_STALE_ID, max_mutating_id), every Tier R op, the taskfile.snapshot job
// on core 1, and the hooks in menus.c. Everything runs the real ctl*.c,
// menus.c, ide.c and obs.c (ctl_harness.hpp).
//
//   test_ctl            the tests; exit status is the number of failed checks
//   test_ctl core1      core 1's real loop (core1_entry) takes a job at its idle
//                       point and not in a confirm box; run.sh runs it after
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

static void fresh(const char *name) { current = name; power_up(); }

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
        { "wp.on", CTL_TIER_S, false, 5, false }, { "pipelined.off", CTL_TIER_S, false, 5, false },
        { "identify.forbid", CTL_TIER_S, false, 5, false }, { "unmount", CTL_TIER_S, true, 30000, false },
        { "detach", CTL_TIER_S, true, 30000, false }, { "session.open", CTL_TIER_SESSION, false, 5, false },
        { "session.close", CTL_TIER_SESSION, false, 5, false }, { "setup.ctrlg", CTL_TIER_M, true, 75000, false },
        { "mount", CTL_TIER_M, true, 1000, false },
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
    CHECK(bf["ctl"].b && bf["sat"].t == J::BOOL && bf["tripwire"].b && !bf["board_jumpers"].b && bf.obj.size() == 6, "build flags");
    const J &caps = res["caps"];
    CHECK(caps.obj.size() == CTL_OP_COUNT + CTL_HUMAN_ONLY_COUNT + CTL_NOT_IN_V1_COUNT, "%zu caps", caps.obj.size());
    for (int i = 0; i < CTL_OP_COUNT; i++) {
        const J &cap = caps[ctl_ops[i].name];
        if (i <= CTL_OP_JOB_LAST)
            CHECK(cap["state"].is("OPERATIONAL") && !cap.get("why"), "%s not OPERATIONAL", ctl_ops[i].name);
        else
            CHECK(cap["state"].is("ABSENT") && cap["why"].is("NOT_BUILT"), "%s not ABSENT/NOT_BUILT", ctl_ops[i].name);
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
    for (const char *op : { "wp.on", "unmount", "session.open", "setup.ctrlg", "mount", "wp.off", "debug.r",
                            "setup.detect", "setup.geometry", "no.such" })
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
    for (auto &l : r) { J k; if (parse_response(l, k)) { done += k["outcome"].is("DONE"); rate += is_refusal(k, "E_RATE"); } }
    CHECK(r.size() == 21 && done == 20 && rate == 1, "%zu answers, %d DONE, %d E_RATE", r.size(), done, rate);
    pump(1000);
    J k = ask1(req(30, "chip.info"));
    CHECK(k["outcome"].is("DONE"), "after the window");
    // A line refused before the rate check (E_PARSE) does not count.
    fresh("rate: parse errors not counted");
    std::string noise;
    for (int i = 0; i < 30; i++) noise += "AT\r\n";
    r = ask(noise, 200);
    k = ask1(req(1, "chip.info"));
    CHECK(r.size() == 30 && k["outcome"].is("DONE"), "%zu answers to noise, then %s", r.size(), k["outcome"].s.c_str());
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

// Stage 1 has no mutating op, so these drive the dispatcher's replay path
// through a host-test stand-in for wp.on: each run counts one execution.
static int executed = 0;
static int stand_in(ctlw_t *w, const ctlj_doc_t *, int) { executed++; ctlw_bool(w, "changed", true); return CTL_E_NONE; }
static void test_replay_dispatch() {
    fresh("replay through the dispatcher");
    ctl_test_op[CTL_OP_WP_ON] = stand_in;
    executed = 0;
    std::string l7 = req(7, "wp.on");
    J a = ask1(l7);
    CHECK(a["outcome"].is("DONE") && executed == 1 && !a.get("replayed"), "first send");
    std::vector<std::string> r = ask(l7);
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
    CHECK(b2["replayed"].b && executed == 1, "\\r\\n resend not replayed");
    // The same id with another line: E_ID_REUSED, UNSAFE, not run, and never the other's response.
    J c1 = ask1(req(7, "wp.on", "{\"expect_epoch\":0}"));
    CHECK(is_refusal(c1, "E_ID_REUSED") && c1["retry"].is("UNSAFE") && executed == 1 && !c1.get("replayed"),
          "same id, other line: %s", c1["err"]["code"].s.c_str());
    // A lower id not in the table: E_STALE_ID; a higher one runs.
    J d = ask1(req(6, "wp.on"));
    CHECK(is_refusal(d, "E_STALE_ID") && executed == 1, "stale id");
    J e = ask1(req(8, "wp.on"));
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
    CHECK(is_refusal(f, "E_SCHEMA") && executed == 2, "schema before replay");
    // Refused requests of a mutating op keep their answer too (not run twice).
    J g = ask1(req(9, "wp.on", "{\"expect_epoch\":\"x\"}"));
    CHECK(is_refusal(g, "E_SCHEMA"), "bad arg");
    // Past 16 entries the oldest ended one is forgotten: its resend is E_STALE_ID.
    for (uint32_t id = 20; id < 40; id++) { ask1(req(id, "wp.on")); pump(60); }
    J old = ask1(l7);
    CHECK(is_refusal(old, "E_STALE_ID"), "an evicted id: %s", old["err"]["code"].s.c_str());
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
          r["tui"]["state"].is("MAIN") && r["jumpers"]["jp1"].is("NOT_BUILT") && !r["presence"]["waiting"].b &&
          r["presence"]["op"].is("NONE") && !r["lease"]["held"].b && r["job"]["running"].is("NONE") &&
          r["job"]["id"].t == J::NUL && r["setup"]["source"].is("UNKNOWN") && r["hashes"]["engine"].is("SOFTWARE") &&
          r["log"]["next_seq"].n >= 2, "values at power-up");
    uint64_t e0 = r["epoch"].n;
    // The human turns write protect off in the TUI: epoch moves, last_change TUI.
    config.drive_write_protected = false;
    pump(2);
    J s2 = ask1(req(2, "status"));
    CHECK(s2["result"]["epoch"].n == e0 + 1 && s2["result"]["wp"]["last_change"].is("TUI") && !s2["result"]["wp"]["effective"].b &&
          s2["dev"]["epoch"].n == e0 + 1, "epoch %llu after a WP change", (unsigned long long)s2["result"]["epoch"].n);
    // Nothing changed: the epoch stays.
    J s3 = ask1(req(3, "status"));
    CHECK(s3["result"]["epoch"].n == e0 + 1, "epoch moved with nothing changed");
    // Mounted through the menus' own path: mount_seq and the marker's log seq.
    cur_cyls = 980; cur_heads = 10; cur_spt = 17; sync_to_config();
    log_mount(OBS_EV_MOUNT, OBS_MNT_MENU); is_mounted = true;
    J m = ask1(req(4, "status"));
    CHECK(m["result"]["mounted"].b && m["result"]["mount_seq"].n == 1 && m["result"]["mount_log_seq"].n == obs_last_marker_seq() &&
          m["result"]["geometry"]["total_sectors"].n == 980u * 10 * 17 && m["result"]["epoch"].n > e0 + 1, "after a mount");
    log_mount(OBS_EV_UNMOUNT, OBS_UNM_MENU); is_mounted = false;
    log_mount(OBS_EV_MOUNT, OBS_MNT_MENU); is_mounted = true;
    m = ask1(req(5, "status"));
    CHECK(m["result"]["mount_seq"].n == 2, "mount_seq %llu after a second mount", (unsigned long long)m["result"]["mount_seq"].n);
    // A Ctrl+G since power-up forces write protect (review C, C-4).
    is_mounted = false;
    mchs_no_identify = true;
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
    config.pipelined_reads = true;
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
    is_mounted = true;
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
          r["result"]["jumpers"].is("NOT_BUILT") && r["result"]["config"].is("FLASH"), "selftest.status");
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
    is_mounted = true;
    core1_runs = false;                             // refused at arrival: core 1 never sees it
    CHECK(is_refusal(ask1(req(47, "taskfile.snapshot")), "E_STATE") && ctl_job.state == CTL_JOB_EMPTY, "mounted at arrival");
    is_mounted = false;
    host_send(req(48, "taskfile.snapshot"));
    pump(5);
    is_mounted = true;                              // the human mounted while it waited
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
static void core1_done() {
    CHECK(ctl_job.state == CTL_JOB_EMPTY && c.jl_present && c.jl_id == 61, "the job did not run from core1_entry (state %u)",
          (unsigned)ctl_job.state);
    CHECK(ctl_tui_state == CTL_TUI_MAIN, "published state %u", ctl_tui_state);
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

// ---- serve: the Control port on stdin/stdout, for ctl_e2e.py ------------------
static void serve() {
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    power_up();
    port.got.clear();
    // A mounted drive's worth of log, so log.read has something to return.
    for (int i = 0; i < 5; i++) obs_marker(OBS_EV_MOUNT, 9, 0, 0, 0, (uint16_t)i, 0xA0);
    static char buf[4096];
    for (;;) {
        size_t n = fread(buf, 1, 1, stdin);        // one byte at a time: never waits for more than the host sent
        if (n == 0) break;
        port.rx.push_back((uint8_t)buf[0]);
        if (buf[0] != '\n') continue;
        pump(30);
        if (!port.got.empty()) { fwrite(port.got.data(), 1, port.got.size(), stdout); fflush(stdout); port.got.clear(); }
    }
}

int main(int argc, char **argv) {
    ide_hw_init();
    if (argc > 1 && !strcmp(argv[1], "serve")) { serve(); return 0; }
    if (argc > 1 && !strcmp(argv[1], "core1")) test_core1_loop();          // does not return
    if (argc > 1 && !strcmp(argv[1], "core1-confirm")) test_core1_confirm(); // does not return
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
    printf("%d checks, %d failed\n", checks, failures);
    return failures > 255 ? 255 : failures;
}
