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
    // Stage 4 (session, Tier M): in v1, not built yet.
    { "session.open",      CTL_TIER_SESSION, false,     5, false },
    { "session.close",     CTL_TIER_SESSION, false,     5, false },
    { "setup.ctrlg",       CTL_TIER_M,       true,  75000, false },
    { "mount",             CTL_TIER_M,       true,   1000, false },
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
void ctl_op_cap_jp(int op, bool built, uint8_t jp1, bool lease, uint8_t *state, const char **why) {
    *why = 0;
    if (op < 0 || op >= CTL_OP_COUNT || !built) {
        *state = CTL_CAP_ABSENT; *why = "NOT_BUILT";
    } else if (ctl_ops[op].tier != CTL_TIER_M) {
        *state = CTL_CAP_OPERATIONAL;
    } else if (jp1 == CTL_JP1_FITTED) {
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
    case CTL_OP_SETUP_CTRLG: case CTL_OP_MOUNT:
                                return 0;       // stage 4: nested schemas not written yet
    default:                    return args_none;
    }
}

static int check_args(const ctlj_doc_t *d, int op, int args, const char **field) {
    const ctl_arg_t *a = op_args(op);
    if (!a) return CTL_E_UNKNOWN_OP;                // never built without its schema
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
    if (ctl_op_needs_token(r->op) != (token > 0)) { r->field = "token"; return CTL_E_SCHEMA; }
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
