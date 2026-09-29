// ATAboy Control, protocol v1: the op table, the envelope and argument
// checks, the replay table and the rate window (ctl_policy.h). Pure C.
//
// The tests compare this table with a hand-written copy of SPEC-v1 4.0's
// (tests/host/test_ctl.cpp, I-14), never with anything generated from here.

#include "ctl_policy.h"
#include <string.h>

// name, tier, job (core 1), max time in ms, built in this stage
const ctl_op_t ctl_ops[CTL_OP_COUNT] = {
    { "hello",             CTL_TIER_R,       false,     5, true  },
    { "status",            CTL_TIER_R,       false,     5, true  },
    { "identify.cached",   CTL_TIER_R,       false,     5, true  },
    { "config.get",        CTL_TIER_R,       false,     5, true  },
    { "taskfile.snapshot", CTL_TIER_R,       true,    100, true  },
    { "log.read",          CTL_TIER_R,       false,    20, true  },
    { "hashes.read",       CTL_TIER_R,       false,    20, true  },
    { "debug.e",           CTL_TIER_R,       false,     5, true  },
    { "selftest.status",   CTL_TIER_R,       false,     5, true  },
    { "chip.info",         CTL_TIER_R,       false,     5, true  },
    { "job.last",          CTL_TIER_R,       false,     5, true  },
    // Stage 2: the ratchet (Tier S).
    { "wp.on",             CTL_TIER_S,       false,     5, true  },
    { "pipelined.off",     CTL_TIER_S,       false,     5, true  },
    { "identify.forbid",   CTL_TIER_S,       false,     5, true  },
    { "unmount",           CTL_TIER_S,       true,  30000, true  },
    { "detach",            CTL_TIER_S,       true,  30000, true  },
    // Stage 4: the lease and Tier M.
    { "session.open",      CTL_TIER_SESSION, false,     5, true  },
    { "session.close",     CTL_TIER_SESSION, false,     5, true  },
    { "setup.ctrlg",       CTL_TIER_M,       true,  75000, true  },
    { "mount",             CTL_TIER_M,       true,   1000, true  },
};

const char *const ctl_src_name[CTL_SRC_COUNT] = {
    "NONE", "TUI_DETECT_NORMAL", "TUI_DETECT_LARGE", "TUI_DETECT_LBA", "TUI_MANUAL", "TUI_FORCE",
    "TUI_CTRLG_IDP", "TUI_CTRLG_NATIVE", "API_CTRLG_IDP", "API_CTRLG_NATIVE", "AUTO_MOUNT",
};

// SPEC-v1 3.4 (client ambiguity A11): never offered to automation.
const char *const ctl_human_only[CTL_HUMAN_ONLY_COUNT] = {
    "wp.off", "geometry.manual", "geometry.force", "geometry.large", "config.save",
    "config.defaults", "debug.i", "debug.s", "debug.r", "fw.update",
    "auto_mount", "iordy", "intrq", "dev_base", "pipelined.on",
};
// Owner Q2 and R-API-1 finding 20: not in v1.
const char *const ctl_not_in_v1[CTL_NOT_IN_V1_COUNT] = { "setup.detect", "setup.geometry" };

const char *const ctl_err_name[CTL_E_COUNT] = {
    "",
    "E_PARSE", "E_TOO_LARGE", "E_PROTO_VERSION", "E_UNKNOWN_OP", "E_SCHEMA",
    "E_RATE", "E_BUSY_STATE", "E_STALE", "E_STATE", "E_BUS_BUSY", "E_TUI_BUSY",
    "E_INFLIGHT", "E_LEASE_HELD", "E_NO_LEASE", "E_LEASE_INVALID", "E_LEASE_REVOKED",
    "E_LOCKED", "E_PRESENCE_REQUIRED", "E_PRECONDITION", "E_RANGE", "E_DRIVE",
    "E_RULE_NO_IDENTIFY", "E_ID_REUSED", "E_STALE_ID",
};
const char *const ctl_outcome_name[4] = { "DONE", "NOT_EXECUTED", "FAILED", "UNKNOWN" };
const char *const ctl_retry_name[3] = { "SAFE", "UNSAFE", "AFTER_STATUS" };

int ctl_op_find(const ctlj_doc_t *d, int s) {
    for (int i = 0; i < CTL_OP_COUNT; i++)
        if (ctlj_str_is(d, s, ctl_ops[i].name)) return i;
    return -1;
}

bool ctl_op_mutating(int op) {
    return op >= 0 && op < CTL_OP_COUNT && ctl_ops[op].tier != CTL_TIER_R;
}

bool ctl_op_needs_token(int op) {
    return op == CTL_OP_SESSION_CLOSE || (op >= 0 && op < CTL_OP_COUNT && ctl_ops[op].tier == CTL_TIER_M);
}

void ctl_op_cap(int op, uint8_t *state, const char **why) {
    *why = 0;
    if (op < 0 || op >= CTL_OP_COUNT || !ctl_ops[op].built) {
        *state = CTL_CAP_ABSENT; *why = "NOT_BUILT";
    } else if (ctl_ops[op].tier == CTL_TIER_M) {
        // Stage 3 and 4 decide this from the jumpers and the lease (3.4).
        // A build without jumper support has no Tier M at all (8.1).
        *state = CTL_CAP_LOCKED; *why = "JUMPERS_UNSUPPORTED";
    } else {
        *state = CTL_CAP_OPERATIONAL;
    }
}

#if ATABOY_BOARD_JUMPERS
void ctl_tier_m_cap(uint8_t jp1, bool lease, uint8_t *state, const char **why) {
    *why = 0;
    if (jp1 == CTL_JP1_FITTED) {
        *state = CTL_CAP_LOCKED; *why = "JP1_FITTED";               // LOCK wins (8.2)
    } else if (jp1 != CTL_JP1_OPEN) {
        *state = CTL_CAP_LOCKED; *why = "JUMPERS_UNREADABLE";       // fail closed (8.1)
    } else if (!lease) {
        *state = CTL_CAP_LOCKED; *why = "NO_LEASE";
    } else {
        *state = CTL_CAP_OPERATIONAL;
    }
}
#endif

// ---------------------------------------------------------------------------
//  Arguments, per op (SPEC-v1 4.x)
// ---------------------------------------------------------------------------
typedef struct {
    const char *name;
    uint8_t     type;       // CTLJ_INT, CTLJ_BOOL, CTLJ_STR (a token; lo..hi its length)
    bool        required;
    uint64_t    lo, hi;
} ctl_arg_t;

static const ctl_arg_t args_none[] = { { 0, 0, false, 0, 0 } };
static const ctl_arg_t args_log[] = {
    { "cursor", CTLJ_INT, true, 0, 16777215 },
    { "max_entries", CTLJ_INT, true, 1, CTL_LOG_PER_READ },
    { 0, 0, false, 0, 0 } };
static const ctl_arg_t args_hashes[] = {
    { "cursor", CTLJ_INT, true, 0, 16777215 },
    { "max_entries", CTLJ_INT, true, 1, CTL_HASH_PER_READ },
    { 0, 0, false, 0, 0 } };
static const ctl_arg_t args_debug_e[] = {
    { "include_data", CTLJ_BOOL, false, 0, 1 },
    { 0, 0, false, 0, 0 } };
// Stage 2 and 4, kept here with the rest of the boundary.
static const ctl_arg_t args_ratchet[] = {
    { "expect_epoch", CTLJ_INT, false, 0, 0xFFFFFFFFu },
    { 0, 0, false, 0, 0 } };
static const ctl_arg_t args_named_mount[] = {
    { "expect_mount_seq", CTLJ_INT, true, 0, 0xFFFFFFFFu },
    { "expect_epoch", CTLJ_INT, false, 0, 0xFFFFFFFFu },
    { 0, 0, false, 0, 0 } };
static const ctl_arg_t args_session_open[] = {
    { "client", CTLJ_STR, true, 1, 16 },
    { "tui_lock", CTLJ_BOOL, true, 0, 1 },
    { "idle_s", CTLJ_INT, true, 30, CTL_LEASE_IDLE_MAX },
    { 0, 0, false, 0, 0 } };
// setup.ctrlg (4.14): the ranges are manual_chs_bad_field's (manualchs.h);
// mode, device and drive_power_cycled are checked for their values below.
static const ctl_arg_t args_setup_ctrlg[] = {
    { "cyls", CTLJ_INT, true, 1, 65535 },
    { "heads", CTLJ_INT, true, 1, 16 },
    { "spt", CTLJ_INT, true, 1, 255 },
    { "mode", CTLJ_STR, true, 1, 64 },
    { "device", CTLJ_STR, true, 1, 64 },
    { "drive_power_cycled", CTLJ_BOOL, true, 0, 1 },
    { "then_mount", CTLJ_BOOL, false, 0, 1 },
    { "expect_epoch", CTLJ_INT, false, 0, 0xFFFFFFFFu },
    { 0, 0, false, 0, 0 } };
static const ctl_arg_t args_mount[] = {
    { "expect", CTLJ_OBJ, true, 0, 0 },
    { "expect_epoch", CTLJ_INT, false, 0, 0xFFFFFFFFu },
    { 0, 0, false, 0, 0 } };
// mount's expect: CHS (cyls, heads, spt) or LBA (lba_sectors), never both.
static const ctl_arg_t args_expect_chs[] = {
    { "source", CTLJ_STR, true, 1, 64 },
    { "mode", CTLJ_STR, true, 1, 64 },
    { "device", CTLJ_STR, true, 1, 64 },
    { "cyls", CTLJ_INT, true, 1, 65535 },
    { "heads", CTLJ_INT, true, 1, 16 },
    { "spt", CTLJ_INT, true, 1, 255 },
    { 0, 0, false, 0, 0 } };
static const ctl_arg_t args_expect_lba[] = {
    { "source", CTLJ_STR, true, 1, 64 },
    { "mode", CTLJ_STR, true, 1, 64 },
    { "device", CTLJ_STR, true, 1, 64 },
    { "lba_sectors", CTLJ_INT, true, 1, 281474976710655ull },   // 2^48 - 1
    { 0, 0, false, 0, 0 } };

static const ctl_arg_t *op_args(int op) {
    switch (op) {
    case CTL_OP_LOG_READ:       return args_log;
    case CTL_OP_HASHES_READ:    return args_hashes;
    case CTL_OP_DEBUG_E:        return args_debug_e;
    case CTL_OP_WP_ON: case CTL_OP_PIPELINED_OFF: case CTL_OP_IDENTIFY_FORBID:
                                return args_ratchet;
    case CTL_OP_UNMOUNT: case CTL_OP_DETACH:
                                return args_named_mount;
    case CTL_OP_SESSION_OPEN:   return args_session_open;
    case CTL_OP_SETUP_CTRLG:    return args_setup_ctrlg;
    case CTL_OP_MOUNT:          return args_mount;
    default:                    return args_none;
    }
}

static int check_members(const ctlj_doc_t *d, const ctl_arg_t *a, int args, const char **field) {
    for (int i = 1; i < d->n; i++) {                // every member must be known
        if (d->node[i].parent != args) continue;
        bool known = false;
        for (const ctl_arg_t *s = a; s->name; s++) if (ctlj_key_is(d, i, s->name)) known = true;
        if (!known) { *field = "args"; return CTL_E_SCHEMA; }
    }
    for (const ctl_arg_t *s = a; s->name; s++) {
        int n = ctlj_find(d, args, s->name);
        *field = s->name;
        if (n < 0) { if (s->required) return CTL_E_SCHEMA; continue; }
        const ctlj_node_t *v = &d->node[n];
        if (v->type != s->type) return CTL_E_SCHEMA;
        if (s->type == CTLJ_INT && (v->num < s->lo || v->num > s->hi)) return CTL_E_RANGE;
        if (s->type == CTLJ_STR && (v->str_len < s->lo || v->str_len > s->hi)) return CTL_E_RANGE;
    }
    *field = 0;
    return CTL_E_NONE;
}

// A string member that must be one of a list: its index, or -1.
static int one_of(const ctlj_doc_t *d, int obj, const char *key, const char *const *names, int n) {
    int v = ctlj_find(d, obj, key);
    if (v < 0) return -1;
    for (int i = 0; i < n; i++) if (ctlj_str_is(d, v, names[i])) return i;
    return -1;
}

static const char *const mode_names[] = { "idp", "native" };
static const char *const dev_names[] = { "master", "slave" };
static const char *const dev_names_up[] = { "MASTER", "SLAVE" };
static const char *const geo_names[] = { "CHS", "LBA" };

static int check_args(const ctlj_doc_t *d, int op, int args, const char **field) {
    const ctl_arg_t *a = op_args(op);
    int e = check_members(d, a, args, field);
    if (e != CTL_E_NONE) return e;
    if (op == CTL_OP_SETUP_CTRLG) {
        // Values the schema's types cannot say (4.14). drive_power_cycled is
        // the operator's declaration: false is not a request.
        if (one_of(d, args, "mode", mode_names, 2) < 0) { *field = "mode"; return CTL_E_SCHEMA; }
        if (one_of(d, args, "device", dev_names, 2) < 0) { *field = "device"; return CTL_E_SCHEMA; }
        if (!d->node[ctlj_find(d, args, "drive_power_cycled")].num) { *field = "drive_power_cycled"; return CTL_E_SCHEMA; }
    } else if (op == CTL_OP_MOUNT) {
        int x = ctlj_find(d, args, "expect");
        int g = one_of(d, x, "mode", geo_names, 2);
        if (g < 0) { *field = "mode"; return CTL_E_SCHEMA; }
        e = check_members(d, g ? args_expect_lba : args_expect_chs, x, field);
        if (e != CTL_E_NONE) return e;
        int s = one_of(d, x, "source", ctl_src_name, CTL_SRC_COUNT);
        if (s <= CTL_SRC_NONE) { *field = "source"; return CTL_E_SCHEMA; }
        if (one_of(d, x, "device", dev_names_up, 2) < 0) { *field = "device"; return CTL_E_SCHEMA; }
    }
    *field = 0;
    return CTL_E_NONE;
}

void ctl_mreq_read(const ctlj_doc_t *d, int op, int args, ctl_mreq_t *m) {
    memset(m, 0, sizeof *m);
    int n = ctlj_find(d, args, "expect_epoch");
    m->has_epoch = n > 0;
    m->expect_epoch = n > 0 ? (uint32_t)d->node[n].num : 0;
    int x = args;
    if (op == CTL_OP_SETUP_CTRLG) {
        m->native = one_of(d, args, "mode", mode_names, 2) == 1;
        m->dev_base = one_of(d, args, "device", dev_names, 2) == 1 ? 0xB0 : 0xA0;
        n = ctlj_find(d, args, "then_mount");
        m->then_mount = n > 0 && d->node[n].num;
    } else {
        x = ctlj_find(d, args, "expect");
        m->lba = one_of(d, x, "mode", geo_names, 2) == 1;
        m->dev_base = one_of(d, x, "device", dev_names_up, 2) == 1 ? 0xB0 : 0xA0;
        m->src = (uint8_t)one_of(d, x, "source", ctl_src_name, CTL_SRC_COUNT);
        if (m->lba) { m->lba_sectors = d->node[ctlj_find(d, x, "lba_sectors")].num; return; }
    }
    m->cyls = (uint16_t)d->node[ctlj_find(d, x, "cyls")].num;
    m->heads = (uint8_t)d->node[ctlj_find(d, x, "heads")].num;
    m->spt = (uint8_t)d->node[ctlj_find(d, x, "spt")].num;
}

int ctl_check_request(const ctlj_doc_t *d, ctl_req_t *r) {
    r->id_ok = false; r->id = 0; r->op = -1; r->args = -1; r->token = -1; r->field = 0;
    int v = ctlj_find(d, 0, "v"), id = ctlj_find(d, 0, "id"), op = ctlj_find(d, 0, "op");
    int args = ctlj_find(d, 0, "args"), token = ctlj_find(d, 0, "token");
    // The id first, so every refusal from here on can echo it (3.3).
    if (id > 0 && d->node[id].type == CTLJ_INT && d->node[id].num >= 1 && d->node[id].num <= CTL_MAX_ID) {
        r->id_ok = true;
        r->id = (uint32_t)d->node[id].num;
    }
    if (v < 0 || d->node[v].type != CTLJ_INT || d->node[v].num != CTL_PROTO_VERSION) {
        r->field = "v";
        return CTL_E_PROTO_VERSION;
    }
    for (int i = 1; i < d->n; i++) {
        if (d->node[i].parent != 0) continue;
        if (i != v && i != id && i != op && i != args && i != token) { r->field = "request"; return CTL_E_SCHEMA; }
    }
    if (!r->id_ok) { r->field = "id"; return CTL_E_SCHEMA; }
    if (op < 0 || d->node[op].type != CTLJ_STR) { r->field = "op"; return CTL_E_SCHEMA; }
    if (args < 0 || d->node[args].type != CTLJ_OBJ) { r->field = "args"; return CTL_E_SCHEMA; }
    int o = ctl_op_find(d, op);
    r->op = o;                  // even when not built: the host tests' stand-ins (ctl.c)
    r->args = args;
    if (o < 0 || !ctl_ops[o].built) { r->field = "op"; return CTL_E_UNKNOWN_OP; }
    return ctl_check_args(d, r);
}

int ctl_check_args(const ctlj_doc_t *d, ctl_req_t *r) {
    int token = ctlj_find(d, 0, "token");
    // A token only where one is needed, so it is never sent anywhere else (3.3).
    // The one op that may carry one without needing one is detach (owner
    // ruling 2026-09-29, review A MED-2): the token of the lease it is under.
    // Its presence is not the schema's to decide; ctl.c (detach_lease_ok)
    // asks for it exactly while it matters.
    bool optional = r->op == CTL_OP_DETACH;
    if (!optional && ctl_op_needs_token(r->op) != (token > 0)) { r->field = "token"; return CTL_E_SCHEMA; }
    if (token > 0) {
        if (d->node[token].type != CTLJ_STR) { r->field = "token"; return CTL_E_SCHEMA; }
        r->token = token;
    }
    return check_args(d, r->op, r->args, &r->field);
}

// ---------------------------------------------------------------------------
//  The replay table
// ---------------------------------------------------------------------------

void ctl_replay_init(ctl_replay_t *t) { memset(t, 0, sizeof *t); }

int ctl_replay_check(const ctl_replay_t *t, uint32_t id, const uint8_t digest[32], int *slot) {
    *slot = -1;
    for (int i = 0; i < CTL_REPLAY_SLOTS; i++) {
        const ctl_rp_slot_t *s = &t->slot[i];
        if (s->state == CTL_RP_EMPTY || s->id != id) continue;
        *slot = i;
        // Another line under this id is never answered with this one's
        // response (R-API-1 finding 4).
        if (memcmp(s->digest, digest, 32) != 0) return CTL_RPV_REUSED;
        if (s->state == CTL_RP_PENDING) return CTL_RPV_INFLIGHT;
        if (s->too_long) return CTL_RPV_LOST;
        return CTL_RPV_REPLAY;
    }
    if (id <= t->max_id) return CTL_RPV_STALE;
    return CTL_RPV_NEW;
}

int ctl_replay_begin(ctl_replay_t *t, uint32_t id, const uint8_t digest[32]) {
    int pick = -1;
    for (int i = 0; i < CTL_REPLAY_SLOTS && pick < 0; i++)
        if (t->slot[i].state == CTL_RP_EMPTY) pick = i;
    for (int i = 0; i < CTL_REPLAY_SLOTS && pick < 0; i++)
        if (t->slot[i].state == CTL_RP_DONE) pick = i;
    if (pick < 0) return -1;
    for (int i = 0; i < CTL_REPLAY_SLOTS; i++)      // the oldest ended entry; never a pending one
        if (t->slot[pick].state != CTL_RP_EMPTY && t->slot[i].state == CTL_RP_DONE &&
            t->slot[i].age < t->slot[pick].age) pick = i;
    ctl_rp_slot_t *s = &t->slot[pick];
    s->id = id;
    memcpy(s->digest, digest, 32);
    s->state = CTL_RP_PENDING;
    s->age = t->next_age++;
    s->len = 0;
    s->too_long = false;
    if (id > t->max_id) t->max_id = id;
    return pick;
}

void ctl_replay_finish(ctl_replay_t *t, int slot, const char *resp, uint32_t len) {
    if (slot < 0 || slot >= CTL_REPLAY_SLOTS) return;
    ctl_rp_slot_t *s = &t->slot[slot];
    if (len > 0 && len <= CTL_REPLAY_BYTES) { memcpy(s->resp, resp, len); s->len = (uint16_t)len; }
    else s->too_long = true;
    s->state = CTL_RP_DONE;
}

// ---------------------------------------------------------------------------
//  The rate window
// ---------------------------------------------------------------------------

void ctl_rate_init(ctl_rate_t *r) { memset(r, 0, sizeof *r); }

bool ctl_rate_admit(ctl_rate_t *r, uint32_t now_ms) {
    if (r->n < CTL_RATE_PER_S) {
        r->at[(r->head + r->n) % CTL_RATE_PER_S] = now_ms;
        r->n++;
        return true;
    }
    // The oldest of the last 20 left the window 1,000 ms after it came.
    if (now_ms - r->at[r->head] >= 1000) {
        r->at[r->head] = now_ms;
        r->head = (r->head + 1) % CTL_RATE_PER_S;
        return true;
    }
    return false;
}
