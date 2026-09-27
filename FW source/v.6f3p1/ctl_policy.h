// ATAboy Control, protocol v1 (palimpsest api/SPEC-v1.md): the boundary.
// Pure C, compiled unchanged into the firmware and the host tests.
//
//  - The op table (SPEC-v1 4.0). Anything not in it does not exist, and an
//    op in it that this build does not carry yet (`built` false) is refused
//    as if it were not there (E_UNKNOWN_OP) and listed ABSENT by hello.
//  - The ops that are never in the table, in any build: the human-only ones
//    (Tier H, 3.4) and the two left out of v1 (setup.detect, owner Q2;
//    setup.geometry, R-API-1 finding 20). hello lists them ABSENT so a
//    client can tell "never" from "not here".
//  - The request envelope and each op's arguments (3.3): unknown or missing
//    fields and wrong types are refused.
//  - The replay table (3.5): mutating requests run at most once per id per
//    boot, keyed by id AND the SHA-256 of the request line.
//  - The rate window (3.5): 20 requests in any 1 s.
//
// Stage 2 of the build (SPEC-v1 12.1) carries Tier R and Tier S (the
// ratchet), whose five ops are the first mutating ones: from this stage on
// the replay table is live on the unit.
#ifndef CTL_POLICY_H
#define CTL_POLICY_H

#include <stdint.h>
#include <stdbool.h>
#include "ctl_json.h"

#define CTL_PROTO_VERSION   1
#define CTL_MAX_ID          2147483647u         // 2^31 - 1
#define CTL_MAX_REQUEST     1024                // bytes, the \n included
#define CTL_MAX_RESPONSE    16384               // bytes, the \n included
#define CTL_RATE_PER_S      20
#define CTL_REPLAY_SLOTS    16
#define CTL_REPLAY_BYTES    512                 // a stored response, at most
#define CTL_PRESENCE_S      120
#define CTL_MOUNT_CREDIT_S  60
#define CTL_LEASE_IDLE_MAX  600
#define CTL_LOG_PER_READ    128
#define CTL_HASH_PER_READ   96

enum { CTL_TIER_R, CTL_TIER_S, CTL_TIER_SESSION, CTL_TIER_M };

// In the order of SPEC-v1 4.0's table.
enum {
    CTL_OP_HELLO, CTL_OP_STATUS, CTL_OP_IDENTIFY_CACHED, CTL_OP_CONFIG_GET,
    CTL_OP_TASKFILE_SNAPSHOT, CTL_OP_LOG_READ, CTL_OP_HASHES_READ, CTL_OP_DEBUG_E,
    CTL_OP_SELFTEST_STATUS, CTL_OP_CHIP_INFO, CTL_OP_JOB_LAST,
    CTL_OP_WP_ON, CTL_OP_PIPELINED_OFF, CTL_OP_IDENTIFY_FORBID, CTL_OP_UNMOUNT, CTL_OP_DETACH,
    CTL_OP_SESSION_OPEN, CTL_OP_SESSION_CLOSE, CTL_OP_SETUP_CTRLG, CTL_OP_MOUNT,
    CTL_OP_COUNT
};

typedef struct {
    const char *name;
    uint8_t     tier;       // CTL_TIER_*
    bool        job;        // runs on core 1 at the TUI's idle point (5.3)
    uint32_t    max_ms;     // SPEC-v1 4.0 "max time"
    bool        built;      // carried by this build (stage 2: Tier R and Tier S)
} ctl_op_t;

extern const ctl_op_t ctl_ops[CTL_OP_COUNT];

// Never in the op table (SPEC-v1 3.4, 4.0): listed ABSENT by hello.
#define CTL_HUMAN_ONLY_COUNT 15
extern const char *const ctl_human_only[CTL_HUMAN_ONLY_COUNT];
#define CTL_NOT_IN_V1_COUNT 2
extern const char *const ctl_not_in_v1[CTL_NOT_IN_V1_COUNT];

int  ctl_op_find(const ctlj_doc_t *d, int str_node);   // CTL_OP_* or -1 (any op, built or not)
bool ctl_op_mutating(int op);                           // Tier S, Tier M and the session ops
bool ctl_op_needs_token(int op);                        // Tier M and session.close

// Capability states (3.4). Stage 2 has no lease and no jumpers (stage 3's
// jumpers: ctl_op_cap_jp below), so a built
// op is OPERATIONAL (Tier S needs neither) and an op not built yet is ABSENT with why NOT_BUILT (a
// token of this stage: v1's list in 3.4 has no state for "in v1, not yet
// built", and ABSENT is "not in this build").
enum { CTL_CAP_OPERATIONAL, CTL_CAP_LOCKED, CTL_CAP_ABSENT };
void ctl_op_cap(int op, uint8_t *state, const char **why);
#if ATABOY_BOARD_JUMPERS
// Stage 3, with the jumpers built (SPEC-v1 3.4, 8.1, 8.2): a Tier M op is
// LOCKED with JP1_FITTED while JP1 is fitted or not yet released, with
// JUMPERS_UNREADABLE while the pads read back wrong or JP1 has not been read
// yet, and otherwise LOCKED with NO_LEASE, or OPERATIONAL while a lease is
// held (stage 4). Tier R, Tier S and the session ops are not the jumpers'.
// An op not built is ABSENT, NOT_BUILT, as in ctl_op_cap. jp1 is JP1's state
// as board_jumpers.h reports it (the same values, checked in ctl.c).
enum { CTL_JP1_UNKNOWN, CTL_JP1_OPEN, CTL_JP1_FITTED, CTL_JP1_UNREADABLE };
void ctl_op_cap_jp(int op, bool built, uint8_t jp1, bool lease, uint8_t *state, const char **why);
#endif

// ---------------------------------------------------------------------------
//  Errors (the closed list of SPEC-v1 3.3)
// ---------------------------------------------------------------------------
enum {
    CTL_E_NONE,
    CTL_E_PARSE, CTL_E_TOO_LARGE, CTL_E_PROTO_VERSION, CTL_E_UNKNOWN_OP, CTL_E_SCHEMA,
    CTL_E_RATE, CTL_E_BUSY_STATE, CTL_E_STALE, CTL_E_STATE, CTL_E_BUS_BUSY, CTL_E_TUI_BUSY,
    CTL_E_INFLIGHT, CTL_E_LEASE_HELD, CTL_E_NO_LEASE, CTL_E_LEASE_INVALID, CTL_E_LEASE_REVOKED,
    CTL_E_LOCKED, CTL_E_PRESENCE_REQUIRED, CTL_E_PRECONDITION, CTL_E_RANGE, CTL_E_DRIVE,
    CTL_E_RULE_NO_IDENTIFY, CTL_E_ID_REUSED, CTL_E_STALE_ID,
    CTL_E_COUNT
};
extern const char *const ctl_err_name[CTL_E_COUNT];

enum { CTL_OUT_DONE, CTL_OUT_NOT_EXECUTED, CTL_OUT_FAILED, CTL_OUT_UNKNOWN };
extern const char *const ctl_outcome_name[4];
enum { CTL_RETRY_SAFE, CTL_RETRY_UNSAFE, CTL_RETRY_AFTER_STATUS };
extern const char *const ctl_retry_name[3];

// ---------------------------------------------------------------------------
//  The envelope and the arguments
// ---------------------------------------------------------------------------
typedef struct {
    bool        id_ok;      // an id 1..2^31-1 could be read (echoed; else id null)
    uint32_t    id;
    int         op;         // CTL_OP_*, or -1
    int         args;       // node of the args object
    int         token;      // node of the token, or -1
    const char *field;      // the field at fault, when there is one
} ctl_req_t;

// Checks in SPEC-v1 3.5's order after the line and the rate: v
// (E_PROTO_VERSION), the envelope (E_SCHEMA), the op (E_UNKNOWN_OP), then
// the token rule and the op's arguments (E_SCHEMA, E_RANGE). CTL_E_NONE when
// the request is well formed for an op this build carries.
int ctl_check_request(const ctlj_doc_t *d, ctl_req_t *r);
// The second half alone (the token rule and the arguments) for r->op.
int ctl_check_args(const ctlj_doc_t *d, ctl_req_t *r);

// ---------------------------------------------------------------------------
//  The replay table (SPEC-v1 3.5, R-API-1 finding 4)
// ---------------------------------------------------------------------------
enum { CTL_RP_EMPTY, CTL_RP_PENDING, CTL_RP_DONE };
typedef struct {
    uint32_t id;
    uint8_t  digest[32];
    uint8_t  state;         // CTL_RP_*
    uint32_t age;           // insertion order; the oldest done entry goes first
    uint16_t len;           // stored response, without its \n; 0 with too_long
    bool     too_long;      // did not fit: never replayed (see ctl_replay_check)
    char     resp[CTL_REPLAY_BYTES];
} ctl_rp_slot_t;

typedef struct {
    ctl_rp_slot_t slot[CTL_REPLAY_SLOTS];
    uint32_t      max_id;   // the highest mutating id seen this boot
    uint32_t      next_age;
} ctl_replay_t;

enum {
    CTL_RPV_NEW,            // not seen: run it
    CTL_RPV_REPLAY,         // same id, same line, ended: send the stored response
    CTL_RPV_INFLIGHT,       // same id, same line, still running or waiting
    CTL_RPV_REUSED,         // same id, another line: E_ID_REUSED
    CTL_RPV_STALE,          // not in the table, and not above max_id: E_STALE_ID
    CTL_RPV_LOST,           // same id and line, ended, but its response did not fit
};
void ctl_replay_init(ctl_replay_t *t);
int  ctl_replay_check(const ctl_replay_t *t, uint32_t id, const uint8_t digest[32], int *slot);
// A new mutating request: its slot, marked pending (never evicted while
// pending), and max_id raised. -1 if every slot is pending.
int  ctl_replay_begin(ctl_replay_t *t, uint32_t id, const uint8_t digest[32]);
// Its response, as sent (without the \n).
void ctl_replay_finish(ctl_replay_t *t, int slot, const char *resp, uint32_t len);

// ---------------------------------------------------------------------------
//  The rate window: at most CTL_RATE_PER_S requests in any 1,000 ms
// ---------------------------------------------------------------------------
typedef struct {
    uint32_t at[CTL_RATE_PER_S];
    uint32_t n;             // admitted so far (saturates at CTL_RATE_PER_S)
    uint32_t head;          // the oldest of the last CTL_RATE_PER_S
} ctl_rate_t;
void ctl_rate_init(ctl_rate_t *r);
bool ctl_rate_admit(ctl_rate_t *r, uint32_t now_ms);   // true: counted and admitted

#endif
