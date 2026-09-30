// ATAboy Control, protocol v1: core 0's side (ctl.h). The Control port's
// bytes, the lines, the envelope, the core-0 ops, and the answers to the
// jobs core 1 runs (ctl_job.c).
//
// Everything here runs from ctl_task(), called from core 0's main loop
// between tud_task() passes, never inside a USB callback: the only two
// callbacks (the Control port's line state, and SET_CONFIGURATION through
// usb.c) set a flag and return. ctl_task() never waits: it reads what the
// port has, writes what the port will take (SPEC-v1 3.1, I-17), and
// returns. A host that does not read cannot stall core 0: a response not
// fully written within 5 s is abandoned and counted.
//
// One line in, one response out (SPEC-v1 3.1). Requests are read and
// answered only while the host holds DTR on the Control port; while DTR is
// low every byte is discarded unanswered, and nothing is ever written into
// TinyUSB's TX FIFO, which is overwritable before DTR (R-API-1 finding 10).
// While a response is going out no new line is read: the bytes wait in
// TinyUSB's RX FIFO, which NAKs the host when full and drops nothing.
//
// The order of the checks (SPEC-v1 3.5, client ambiguity A8): the line
// (E_PARSE, E_TOO_LARGE), the rate (E_RATE), v (E_PROTO_VERSION), the
// envelope and the op's arguments (E_SCHEMA, E_UNKNOWN_OP, E_RANGE), then,
// for a mutating request, the replay table, and only then the op itself.

#include "ctl.h"
#include "ctl_job.h"
#include "ctl_json.h"
#include "ctl_policy.h"
#include "ctl_sha256.h"
#include "ctl_build.h"
#include "ctl_state.h"
#include "tusb.h"
#include "pico/stdlib.h"
#include "pico/rand.h"
#include "pico/unique_id.h"
#include "ide.h"
#include "obs.h"
#include "config.h"
#include "fault.h"
#include <string.h>
#if ATABOY_BOARD_JUMPERS
#include "board_jumpers.h"
_Static_assert((int)CTL_JP1_UNKNOWN == (int)BJ_UNKNOWN && (int)CTL_JP1_OPEN == (int)BJ_OPEN &&
               (int)CTL_JP1_FITTED == (int)BJ_FITTED && (int)CTL_JP1_UNREADABLE == (int)BJ_UNREADABLE,
               "ctl_policy.h's JP1 states are board_jumpers.h's");
#endif

extern volatile bool is_mounted;
extern volatile bool cdc_connected;         // the TUI's port, interface 0 (ATAboy.c)
extern bool usb_msc_ide_busy(void);         // usb.c

#define CTL_ITF 1                           // CDC instance 1: interfaces 3 and 4 (usb_descriptors.c)
#define CTL_TX_ABANDON_MS   5000
#define CTL_LINE_QUIET_MS   2000            // a partial line with no \n (SPEC-v1 6.3)
#define CTL_GAP_MS          50              // longer between passes: a long USB callback, not silence
#define CTL_TUI_WAIT_MS     30000           // a job not picked up by then is withdrawn (5.3)
#define CTL_JOB_LAST_BYTES  1024

// What status and the epoch are computed from (SPEC-v1 4.3). The epoch is
// counted where each of these changes (ctl_state.h); this snapshot is the
// cross-check (epoch_poll) and status's consistency test.
typedef struct {
    uint8_t  mounted, lba, wp, pipelined, dev_base, no_identify, native_active, native_declared;
    uint8_t  forbidden, manual_chs, idp, setup_src;
    uint32_t lease_gen;             // stage 4: the lease (0: none held)
#if ATABOY_BOARD_JUMPERS
    uint8_t  jp1;                   // JP1's lock state (stage 3); JP2's level is not in the epoch (4.3)
#endif
    uint16_t cyls;
    uint8_t  heads, spt;
    uint64_t lba_sectors;
} ctl_fp_t;

static struct {
    // The port
    bool     dtr;
    uint32_t port_gen;              // counts port closes and USB resets
    uint32_t prev_pass_ms;
    // The line being read
    char     line[CTL_MAX_REQUEST];
    uint32_t len;
    bool     overflow;              // over 1,023 bytes: answered, discarding to the next \n
    uint32_t quiet_ms;
    // The response going out
    char     resp[CTL_MAX_RESPONSE];
    uint32_t resp_len, resp_sent, resp_at_ms;
    bool     resp_busy;
    // Identity and bookkeeping
    char     serial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    char     boot[17];
    uint32_t seq;
    ctl_fp_t fp;
    uint32_t fp_epoch;              // ctl_epoch when fp was taken
    uint32_t epoch_missed;          // changes no hook counted (epoch_poll); 0, or a hook is missing
    uint32_t tx_abandoned, parse_errors;
    uint32_t cur_id;                // the request a core-0 op is running for (its API entry)
    const char *err_field;          // err.field of a core-0 op's refusal
    ctl_replay_t replay;
    ctl_rate_t   rate;
    ctlj_doc_t   doc;
    // The job in the mailbox, as core 0 knows it
    uint32_t job_port_gen;
    int      job_slot;              // its replay slot when it is mutating (unmount, detach)
    // job.last (SPEC-v1 4.11)
    bool     jl_present;
    uint8_t  jl_op;
    uint32_t jl_id, jl_started, jl_ended, jl_begin, jl_end;
    uint8_t  jl_digest[32];
    uint32_t jl_len;
    char     jl_resp[CTL_JOB_LAST_BYTES];
    const char *err_why;            // err.why of a core-0 op's refusal
    // Stage 4. The lease (SPEC-v1 6.1). The token is 32 lower-case hex
    // characters from the TRNG; it is compared, never logged or shown.
    struct {
        bool     held;
        char     token[33];
        char     client[17];
        uint32_t idle_ms, last_ms, gen, next_gen;
        bool     old_valid;         // the last lease's token and how it ended
        char     old_token[33];
        uint8_t  old_by;
    } lease;
    // The presence slot (4.14): the one Tier M request waiting for its act.
    struct {
        bool     waiting;
        uint8_t  op;
        uint32_t id, arrival_ms, arrival_epoch, lease_gen;
        uint8_t  digest[32];
        int      slot;              // its replay slot
        ctl_mreq_t m;
    } pres;
    // The declared pair's mount credit (owner Q3): one mount, same lease,
    // same geometry, within 60 s of the set-up's end, no epoch change since.
    struct {
        bool     live;
        uint32_t lease_gen, end_ms, epoch;
    } credit;
    // Answers owed, in order, before any new line is read: requests
    // withdrawn from the slot or the mailbox, and session.close's own.
    struct {
        uint8_t  kind, err, by, retry;
        uint32_t id, port_gen;
        int      slot;
        const char *msg, *why;
    } outq[3];
    uint8_t  outq_n;
    // err.by, err.client and err.idle_ms_left of the next refusal (send_error).
    uint8_t  ex_by;
    const char *ex_client;
    bool     ex_idle;
    uint32_t ex_idle_ms;
} c;

static volatile bool dtr_dropped, usb_mounted;      // set by the two callbacks

static uint8_t page_buf[OBS_PAGE_HEADER + CTL_HASH_PER_READ * sizeof(obs_hash_entry_t)];
_Static_assert(OBS_PAGE_HEADER + CTL_LOG_PER_READ * sizeof(obs_log_entry_t) <= sizeof page_buf,
               "a log.read page fits");

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

#if ATABOY_HOST_TEST
// Host tests: stand-in handlers that make an op of a later stage (the
// session ops) count as built, so the dispatcher can be driven for them
// before stage 4 exists. Never compiled into the firmware.
typedef int (*ctl_test_op_fn)(ctlw_t *w, const ctlj_doc_t *d, int args);
inline ctl_test_op_fn ctl_test_op[CTL_OP_COUNT];
#endif

// ---------------------------------------------------------------------------
//  The state snapshot and the epoch
// ---------------------------------------------------------------------------

static void fingerprint(ctl_fp_t *f) {
    memset(f, 0, sizeof *f);        // padding too: compared with memcmp
    f->mounted = is_mounted;
    f->lba = config.use_lba_mode;
    f->wp = config.drive_write_protected;
    f->pipelined = config.pipelined_reads;
    f->dev_base = config.dev_base;
    f->no_identify = ide_mchs_no_identify();
    f->native_active = ide_manual_chs_native_active();
    f->native_declared = ide_native_drive();
    f->forbidden = ide_identify_forbidden();
    f->manual_chs = ide_manual_chs_active();
    f->idp = ide_idp_sent_since_boot();
    f->setup_src = ctl_setup_src;
    f->lease_gen = c.lease.held ? c.lease.gen : 0;
#if ATABOY_BOARD_JUMPERS
    f->jp1 = bj_jp1();
#endif
    f->cyls = config.cyls;
    f->heads = config.heads;
    f->spt = config.spt;
    f->lba_sectors = config.lba_sectors;
}

// The cross-check, once a pass: a snapshot that moved while the epoch did
// not is a change no hook counted. It is counted (epoch_missed, which the
// host tests require to stay 0) and the epoch is moved here, late, so a
// missing hook is never a missed change. This is stage 1's epoch, kept
// behind the hooks: on its own it missed a change and its reversal inside
// one pass (one long USB callback). On the unit the other core can be
// between its store and its bump when this looks, which counts one here
// that was not missed; only the host tests, where nothing runs between,
// read the count.
static void epoch_poll(void) {
    ctl_fp_t f;
    fingerprint(&f);
    uint32_t e = ctl_epoch;
    if (memcmp(&f, &c.fp, sizeof f) != 0) {
        if (e == c.fp_epoch) {
            c.epoch_missed++;
            // Not CTL_EPOCH_BUMP: this late count must stand even where the
            // hooks do not (the mutation run takes them out through it).
            __atomic_fetch_add(&ctl_epoch, 1u, __ATOMIC_SEQ_CST);
            e = ctl_epoch;
        }
        c.fp = f;
    }
    c.fp_epoch = e;
}

// ---------------------------------------------------------------------------
//  Responses
// ---------------------------------------------------------------------------

static void begin(ctlw_t *w, bool id_ok, uint32_t id) {
    ctlw_init(w, c.resp, CTL_MAX_RESPONSE - 1);         // room for the \n
    ctlw_obj(w, 0);
    ctlw_uint(w, "v", CTL_PROTO_VERSION);
    if (id_ok) ctlw_uint(w, "id", id); else ctlw_null(w, "id");
}

static void dev_block(ctlw_t *w) {
    ctlw_obj(w, "dev");
    ctlw_str(w, "serial", c.serial);
    ctlw_str(w, "boot", c.boot);
    ctlw_uint(w, "up_ms", now_ms());
    ctlw_uint(w, "seq", ++c.seq);
    ctlw_uint(w, "epoch", ctl_epoch);
    ctlw_end_obj(w);
}

// Ends the object, adds the \n and hands it to the writer (tx_pump), or,
// with send false, only leaves it in c.resp (a job's answer for a port
// that has since closed: kept as job.last, not sent).
static uint32_t finish(ctlw_t *w, bool send) {
    dev_block(w);
    ctlw_end_obj(w);
    if (w->overflow) return 0;
    c.resp[w->len] = '\n';
    c.resp_len = w->len + 1;
    c.resp_sent = 0;
    c.resp_at_ms = now_ms();
    c.resp_busy = send;
    return w->len;
}

// err.by (SPEC-v1 6.1), by CTL_BY_*.
static const char *const by_name[] = { "", "CLOSED", "TUI", "USB_RESET", "PORT_CLOSED", "IDLE" };

// detach's failure result (SPEC-v1 3.3, 4.12): the state it left, known.
static bool fail_detach;
static uint32_t send_error(bool id_ok, uint32_t id, uint8_t outcome, uint8_t retry, uint8_t err,
                           const char *msg, const char *why, const char *field, bool send) {
    ctlw_t w;
    begin(&w, id_ok, id);
    ctlw_bool(&w, "ok", false);
    ctlw_str(&w, "outcome", ctl_outcome_name[outcome]);
    if (fail_detach) {
        fail_detach = false;
        ctlw_obj(&w, "result");
        ctlw_bool(&w, "detached", false);
        ctlw_bool(&w, "mounted", is_mounted);
        ctlw_end_obj(&w);
    }
    ctlw_str(&w, "retry", ctl_retry_name[retry]);
    ctlw_obj(&w, "err");
    ctlw_str(&w, "code", ctl_err_name[err]);
    ctlw_str(&w, "msg", msg);
    if (why) ctlw_str(&w, "why", why);
    if (field) ctlw_str(&w, "field", field);
    if (c.ex_by) ctlw_str(&w, "by", by_name[c.ex_by]);
    if (c.ex_client) ctlw_str(&w, "client", c.ex_client);
    if (c.ex_idle) ctlw_uint(&w, "idle_ms_left", c.ex_idle_ms);
    c.ex_by = CTL_BY_NONE; c.ex_client = 0; c.ex_idle = false;
    ctlw_end_obj(&w);
    if (err == CTL_E_PARSE) c.parse_errors++;
    return finish(&w, send);
}

static void refuse(bool id_ok, uint32_t id, uint8_t err, const char *msg, const char *field) {
    send_error(id_ok, id, CTL_OUT_NOT_EXECUTED, CTL_RETRY_SAFE, err, msg, 0, field, true);
}

static const char *const tui_name[] = {
    "BOOT", "MAIN", "FEATURES", "DEBUG", "MOUNTED", "CONFIRM", "ERROR_BOX", "MODAL",
};

// ---------------------------------------------------------------------------
//  The core-0 ops (Tier R). Each writes its result's members into the open
//  result object and returns CTL_E_NONE, or an error and writes nothing that
//  is kept.
// ---------------------------------------------------------------------------

static int op_hello(ctlw_t *w, const ctlj_doc_t *d, int args) {
    (void)d; (void)args;
    ctlw_arr(w, "proto"); ctlw_uint(w, 0, CTL_PROTO_VERSION); ctlw_end_arr(w);
    ctlw_str(w, "serial", c.serial);
    ctlw_obj(w, "fw");
    ctlw_str(w, "version", CTL_FW_VERSION);
    ctlw_str(w, "banner", CTL_FW_BANNER);
    ctlw_str(w, "git", CTL_GIT);
    ctlw_bool(w, "dirty", CTL_DIRTY);
    ctlw_str(w, "sat_policy_sha256", CTL_SAT_POLICY_SHA256);
    ctlw_str(w, "ctl_policy_sha256", CTL_CTL_POLICY_SHA256);
    ctlw_obj(w, "build_flags");
#if ATABOY_SAT
    ctlw_bool(w, "sat", true);
#else
    ctlw_bool(w, "sat", false);
#endif
#if ATABOY_SAT && ATABOY_SAT_SMART_SAVES
    ctlw_bool(w, "sat_smart_saves", true);
#else
    ctlw_bool(w, "sat_smart_saves", false);
#endif
#if ATABOY_STRICT_MEDIA_BOUNDS
    ctlw_bool(w, "strict_media_bounds", true);
#else
    ctlw_bool(w, "strict_media_bounds", false);
#endif
    ctlw_bool(w, "ctl", true);
#if ATABOY_BOARD_JUMPERS
    ctlw_bool(w, "board_jumpers", true);        // stage 3: JP1 and JP2 (SPEC-v1 8)
#else
    ctlw_bool(w, "board_jumpers", false);
#endif
#ifdef IDE_FAULT_TRIPWIRE
    ctlw_bool(w, "tripwire", true);
#else
    ctlw_bool(w, "tripwire", false);
#endif
    ctlw_end_obj(w);
    ctlw_end_obj(w);
    ctlw_obj(w, "board");
    ctlw_str(w, "die", "RP2350");
    ctlw_uint(w, "stepping", rp2350_chip_version());
    ctlw_str(w, "package", "UNKNOWN");          // SPEC-v1 4.9: not read until verified
    ctlw_end_obj(w);
    ctlw_obj(w, "caps");
    for (int op = 0; op < CTL_OP_COUNT; op++) {
        uint8_t st; const char *why;
        ctl_op_cap(op, &st, &why);
#if ATABOY_BOARD_JUMPERS
        if (st == CTL_CAP_LOCKED && ctl_ops[op].tier == CTL_TIER_M)
            ctl_tier_m_cap(bj_jp1(), c.lease.held, &st, &why);
#endif
        ctlw_obj(w, ctl_ops[op].name);
        ctlw_str(w, "state", st == CTL_CAP_OPERATIONAL ? "OPERATIONAL" : st == CTL_CAP_LOCKED ? "LOCKED" : "ABSENT");
        if (why) ctlw_str(w, "why", why);
        ctlw_end_obj(w);
    }
    for (int i = 0; i < CTL_HUMAN_ONLY_COUNT; i++) {
        ctlw_obj(w, ctl_human_only[i]);
        ctlw_str(w, "state", "ABSENT"); ctlw_str(w, "why", "HUMAN_ONLY");
        ctlw_end_obj(w);
    }
    for (int i = 0; i < CTL_NOT_IN_V1_COUNT; i++) {
        ctlw_obj(w, ctl_not_in_v1[i]);
        ctlw_str(w, "state", "ABSENT"); ctlw_str(w, "why", "NOT_IN_V1");
        ctlw_end_obj(w);
    }
    ctlw_end_obj(w);
    ctlw_obj(w, "limits");
    ctlw_uint(w, "max_request_line", CTL_MAX_REQUEST);
    ctlw_uint(w, "max_response_line", CTL_MAX_RESPONSE);
    ctlw_uint(w, "max_log_entries_per_read", CTL_LOG_PER_READ);
    ctlw_uint(w, "max_hash_entries_per_read", CTL_HASH_PER_READ);
    ctlw_uint(w, "rate_per_s", CTL_RATE_PER_S);
    ctlw_uint(w, "presence_window_s", CTL_PRESENCE_S);
    ctlw_uint(w, "mount_credit_s", CTL_MOUNT_CREDIT_S);
    ctlw_uint(w, "lease_idle_s_max", CTL_LEASE_IDLE_MAX);
    ctlw_uint(w, "replay_slots", CTL_REPLAY_SLOTS);
    ctlw_uint(w, "max_mutating_id", c.replay.max_id);
    ctlw_end_obj(w);
    return CTL_E_NONE;
}

static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static const char *engine_name(uint8_t flags) {
    return (flags & 8) ? "SOFTWARE" : (flags & 2) ? "PASS" : (flags & 4) ? "FAIL" : "NOT_RUN";
}

static const char *const setup_name[CTL_SRC_COUNT] = {
    "NONE", "TUI_DETECT_NORMAL", "TUI_DETECT_LARGE", "TUI_DETECT_LBA", "TUI_MANUAL", "TUI_FORCE",
    "TUI_CTRLG_IDP", "TUI_CTRLG_NATIVE", "API_CTRLG_IDP", "API_CTRLG_NATIVE", "AUTO_MOUNT",
};
static const char *const wp_last_name[] = { "BOOT", "TUI", "API" };
#if ATABOY_BOARD_JUMPERS
static const char *const jumper_name[] = { "UNKNOWN", "OPEN", "FITTED", "UNREADABLE" };    // BJ_*
#endif

// The declared pair's credit (owner Q3), live: earned and not used, the
// lease it was earned under still held, within its 60 s, and no epoch change
// since the set-up ended (a TUI action, a ratchet op, JP1, the lease ending,
// another set-up; JP2's level is not one). That a mount it takes expects
// exactly what the set-up set follows: with no epoch change since, the
// state is the set-up's, and a mount's expect is compared with the state
// before this is asked (ctl_mreq_check, E_STALE).
static bool credit_live(uint32_t now) {
    return c.credit.live && c.lease.held && c.credit.lease_gen == c.lease.gen &&
           now - c.credit.end_ms <= CTL_MOUNT_CREDIT_S * 1000u && ctl_epoch == c.credit.epoch;
}

// The credit and the lease's idle time, as status shows them.
static uint32_t credit_ms_left(uint32_t now) {
    if (!credit_live(now)) return 0;
    uint32_t used = now - c.credit.end_ms;
    return used < CTL_MOUNT_CREDIT_S * 1000u ? CTL_MOUNT_CREDIT_S * 1000u - used : 0;
}
static uint32_t lease_idle_left(uint32_t now) {
    if (!c.lease.held) return 0;
    uint32_t idle = now - c.lease.last_ms;
    return idle < c.lease.idle_ms ? c.lease.idle_ms - idle : 0;
}

// The wp object of status and of wp.on's result (SPEC-v1 4.3, 4.12).
static void write_wp(ctlw_t *w) {
#if ATABOY_BOARD_JUMPERS
    bool jp1 = jp1_locked();                    // read live (8.2), and so forced before config is shown
#endif
    ctlw_obj(w, "wp");
    ctlw_bool(w, "effective", ide_write_protected());
    ctlw_bool(w, "config", config.drive_write_protected);
    ctlw_arr(w, "forced_by");
    if (ide_mchs_no_identify()) ctlw_str(w, 0, "CTRLG");     // review C, C-4 (ide.c)
#if ATABOY_BOARD_JUMPERS
    if (jp1) ctlw_str(w, 0, "JP1");
#endif
    ctlw_end_arr(w);
    uint8_t wl = ctl_wp_last;
    ctlw_str(w, "last_change", wl <= CTL_WP_API ? wp_last_name[wl] : "UNKNOWN");
    ctlw_end_obj(w);
}

static void write_status(ctlw_t *w) {
    ctlw_bool(w, "mounted", is_mounted);
    uint32_t ms = ctl_mount_seq;
    ctlw_uint(w, "mount_seq", ms);
    if (ms) ctlw_uint(w, "mount_log_seq", ctl_mount_log_seq); else ctlw_null(w, "mount_log_seq");
    ctlw_obj(w, "geometry");
    ctlw_str(w, "mode", config.use_lba_mode ? "LBA" : "CHS");
    ctlw_uint(w, "cyls", config.cyls);
    ctlw_uint(w, "heads", config.heads);
    ctlw_uint(w, "spt", config.spt);
    ctlw_uint(w, "lba_sectors", config.lba_sectors);
    ctlw_uint(w, "total_sectors", config.use_lba_mode ? config.lba_sectors
                                  : (uint64_t)config.cyls * config.heads * config.spt);
    ctlw_end_obj(w);
    // Which path set the geometry up: every set-up path writes it (4.3,
    // ctl_state.h), and every path that clears the geometry sets NONE.
    uint8_t src = ctl_setup_src;
    ctlw_obj(w, "setup");
    ctlw_str(w, "source", src < CTL_SRC_COUNT ? setup_name[src] : "UNKNOWN");
    ctlw_uint(w, "at_ms", ctl_setup_at_ms);
    ctlw_end_obj(w);
    ctlw_str(w, "device", config.dev_base == 0xB0 ? "SLAVE" : "MASTER");
    write_wp(w);
    ctlw_obj(w, "rules");
    ctlw_bool(w, "no_identify_ctrlg", ide_mchs_no_identify());
    ctlw_bool(w, "identify_forbidden", ide_identify_forbidden());
    ctlw_bool(w, "identify_barred", ide_identify_barred());
    ctlw_bool(w, "manual_chs_active", ide_manual_chs_active());
    ctlw_bool(w, "native_active", ide_manual_chs_native_active());
    ctlw_bool(w, "native_declared", ide_native_drive());
    ctlw_bool(w, "idp_sent_since_boot", ide_idp_sent_since_boot());
    ctlw_end_obj(w);
    ctlw_obj(w, "features");
    ctlw_bool(w, "auto_mount", config.auto_mount);
    ctlw_bool(w, "iordy", config.iordy_enabled);
    ctlw_bool(w, "intrq", config.intrq_enabled);
    ctlw_bool(w, "pipelined_reads", config.pipelined_reads);
    ctlw_end_obj(w);
    ctlw_obj(w, "bus");
    ctlw_bool(w, "msc_busy", usb_msc_ide_busy());
    ctlw_bool(w, "recovery_pending", ide_recovery_pending());
    ctlw_end_obj(w);
    uint8_t ts = ctl_tui_state;
    ctlw_obj(w, "tui");
    ctlw_bool(w, "connected", cdc_connected);
    ctlw_str(w, "state", ts <= CTL_TUI_MODAL ? tui_name[ts] : "UNKNOWN");
    ctlw_bool(w, "locked", false);              // stage 4 (the TUI lock)
    ctlw_end_obj(w);
#if ATABOY_BOARD_JUMPERS
    uint8_t j1 = bj_jp1(), j2 = bj_jp2();
    ctlw_obj(w, "jumpers");                     // stage 3 (SPEC-v1 8): JP1's lock state, JP2's level
    ctlw_str(w, "jp1", j1 <= BJ_UNREADABLE ? jumper_name[j1] : "UNKNOWN");
    ctlw_str(w, "jp2", j2 <= BJ_UNREADABLE ? jumper_name[j2] : "UNKNOWN");
#else
    ctlw_obj(w, "jumpers");                     // no jumper support in this build
    ctlw_str(w, "jp1", "NOT_BUILT");
    ctlw_str(w, "jp2", "NOT_BUILT");
#endif
    ctlw_uint(w, "mount_credit_ms_left", credit_ms_left(now_ms()));
    ctlw_end_obj(w);
    ctlw_obj(w, "presence");                    // 4.14: the waiting Tier M request
    ctlw_bool(w, "waiting", c.pres.waiting);
    if (c.pres.waiting) {
        uint32_t waited = now_ms() - c.pres.arrival_ms;
        ctlw_str(w, "op", ctl_ops[c.pres.op].name);
        ctlw_uint(w, "id", c.pres.id);
        ctlw_hex(w, "request_sha256", c.pres.digest, 32);
        ctlw_uint(w, "ms_left", waited < CTL_PRESENCE_S * 1000u ? CTL_PRESENCE_S * 1000u - waited : 0);
    } else {
        ctlw_str(w, "op", "NONE");
        ctlw_null(w, "id");
        ctlw_null(w, "request_sha256");
        ctlw_uint(w, "ms_left", 0);
    }
    ctlw_end_obj(w);
    ctlw_obj(w, "lease");                       // 6.1: never the token
    ctlw_bool(w, "held", c.lease.held);
    if (c.lease.held) ctlw_str(w, "client", c.lease.client); else ctlw_null(w, "client");
    ctlw_uint(w, "idle_ms_left", lease_idle_left(now_ms()));
    ctlw_bool(w, "tui_lock", false);            // not built (stage 4 deviation: tui_lock is refused)
    ctlw_end_obj(w);
    uint32_t js = ctl_job.state;
    bool live = js == CTL_JOB_QUEUED || js == CTL_JOB_RUNNING;
    ctlw_obj(w, "job");
    ctlw_str(w, "running", js == CTL_JOB_RUNNING ? ctl_ops[ctl_job.op].name : "NONE");
    if (live) ctlw_uint(w, "id", ctl_job.id); else ctlw_null(w, "id");
    ctlw_str(w, "queued", js == CTL_JOB_QUEUED ? ctl_ops[ctl_job.op].name : "NONE");
    ctlw_end_obj(w);
    ctlw_obj(w, "ids");
    ctlw_uint(w, "max_mutating", c.replay.max_id);
    ctlw_end_obj(w);
    uint8_t h[OBS_PAGE_HEADER];
    obs_log_page(0, h, OBS_PAGE_HEADER);        // the header alone: no entries fit
    ctlw_obj(w, "log");
    ctlw_uint(w, "next_seq", le32(h + 16));
    ctlw_uint(w, "oldest_seq", le32(h + 20));
    ctlw_end_obj(w);
    obs_hash_page(0, h, OBS_PAGE_HEADER);
    ctlw_obj(w, "hashes");
    ctlw_uint(w, "next_seq", le32(h + 16));
    ctlw_uint(w, "oldest_seq", le32(h + 20));
    ctlw_str(w, "engine", engine_name(h[28]));
    ctlw_end_obj(w);
    ide_self_resets_t sr;
    ide_self_resets(&sr);
    ctlw_obj(w, "counters");
    ctlw_uint(w, "unwitnessed_blocks", ide_unwitnessed_blocks());
    ctlw_uint(w, "self_resets", sr.detected);
    ctlw_uint(w, "tx_abandoned", c.tx_abandoned);
    ctlw_uint(w, "parse_errors", c.parse_errors);
    ctlw_end_obj(w);
    ctlw_uint(w, "epoch", ctl_epoch);
    ctlw_uint(w, "up_ms", now_ms());
}

// A consistent snapshot (4.3): the state is read before and after the copy;
// if it moved (core 1 changed something meanwhile) the copy is made again,
// up to 3 times, then E_BUSY_STATE.
static int op_status(ctlw_t *w, const ctlj_doc_t *d, int args) {
    (void)d; (void)args;
    ctlw_t start = *w;
    for (int tries = 0; tries < 3; tries++) {
        epoch_poll();
        ctl_fp_t before = c.fp;
        uint32_t e_before = ctl_epoch, ms_before = ctl_mount_seq;
        *w = start;
        write_status(w);
        ctl_fp_t after;
        fingerprint(&after);
        if (memcmp(&before, &after, sizeof after) == 0 && ms_before == ctl_mount_seq && e_before == ctl_epoch)
            return CTL_E_NONE;
    }
    *w = start;
    return CTL_E_BUSY_STATE;
}

static const char *const idc_origin[] = { "NONE", "AUTO_DETECT", "AUTO_MOUNT", "DEBUG_I", "DEBUG_S", "SAT_CHECK" };
static const char *const idc_why[] = { "NEVER", "CTRLG", "PROBE", "DETACH" };

static int op_identify_cached(ctlw_t *w, const ctlj_doc_t *d, int args) {
    (void)d; (void)args;
    static ide_idcache_t ic;        // static: core 0's stack is 4 KB (5.2)
    static uint8_t b[512];
    uint8_t dg[32];
    if (!ide_idcache_get(&ic)) return CTL_E_BUSY_STATE;
    if (!ic.present) {
        ctlw_bool(w, "present", false);
        ctlw_str(w, "why", ic.why <= IDE_IDC_WHY_DETACH ? idc_why[ic.why] : "NEVER");
        return CTL_E_NONE;
    }
    for (int i = 0; i < 256; i++) { b[2 * i] = (uint8_t)ic.words[i]; b[2 * i + 1] = (uint8_t)(ic.words[i] >> 8); }
    ctl_sha256(b, sizeof b, dg);
    ctlw_bool(w, "present", true);
    ctlw_bool(w, "stale", ic.stale);
    ctlw_str(w, "origin", ic.origin <= IDE_IDC_SAT_CHECK ? idc_origin[ic.origin] : "NONE");
    ctlw_uint(w, "at_ms", ic.at_ms);
    ctlw_str(w, "device", ic.dev_base == 0xB0 ? "SLAVE" : "MASTER");
    ctlw_hex(w, "words_hex", b, sizeof b);      // the drive's bytes, only as hex (4.1)
    ctlw_hex(w, "sha256", dg, sizeof dg);
    static const uint8_t used[] = { 49, 82, 83, 84, 85, 87 };   // the words the firmware itself reads
    static const char *const used_name[] = { "w49", "w82", "w83", "w84", "w85", "w87" };
    for (unsigned i = 0; i < sizeof used; i++) ctlw_uint(w, used_name[i], ic.words[used[i]]);
    return CTL_E_NONE;
}

static void config_fields(ctlw_t *w, const config_t *k) {
    ctlw_uint(w, "main_selected", k->main_selected);
    ctlw_uint(w, "feat_selected", k->feat_selected);
    ctlw_bool(w, "drive_write_protected", k->drive_write_protected);
    ctlw_bool(w, "auto_mount", k->auto_mount);
    ctlw_bool(w, "iordy_enabled", k->iordy_enabled);
    ctlw_bool(w, "intrq_enabled", k->intrq_enabled);
    ctlw_bool(w, "use_lba_mode", k->use_lba_mode);
    ctlw_uint(w, "cyls", k->cyls);
    ctlw_uint(w, "heads", k->heads);
    ctlw_uint(w, "spt", k->spt);
    ctlw_uint(w, "lba_sectors", k->lba_sectors);
    ctlw_uint(w, "dev_base", k->dev_base);
    ctlw_bool(w, "pipelined_reads", k->pipelined_reads);
}

// Every saved field, not magic or crc (config.h), compared field by field.
static bool config_same(const config_t *a, const config_t *b) {
    return a->main_selected == b->main_selected && a->feat_selected == b->feat_selected &&
           a->drive_write_protected == b->drive_write_protected && a->auto_mount == b->auto_mount &&
           a->iordy_enabled == b->iordy_enabled && a->intrq_enabled == b->intrq_enabled &&
           a->use_lba_mode == b->use_lba_mode && a->cyls == b->cyls && a->heads == b->heads &&
           a->spt == b->spt && a->lba_sectors == b->lba_sectors && a->dev_base == b->dev_base &&
           a->pipelined_reads == b->pipelined_reads;
}

static int op_config_get(ctlw_t *w, const ctlj_doc_t *d, int args) {
    (void)d; (void)args;
    config_t ram = config, fl;
    config_flash_copy(&fl);
    bool valid = config_valid(&fl);
    ctlw_obj(w, "ram"); config_fields(w, &ram); ctlw_end_obj(w);
    // A sector that fails its check is still shown as read, with valid false.
    ctlw_obj(w, "flash"); config_fields(w, &fl); ctlw_bool(w, "valid", valid); ctlw_end_obj(w);
    ctlw_bool(w, "dirty", !valid || !config_same(&ram, &fl));
    ctlw_str(w, "loaded_from", config_loaded_from_flash() ? "FLASH" : "DEFAULTS");
    return CTL_E_NONE;
}

// log.read and hashes.read (4.7): the READ BUFFER 4Ch and 48h pages, byte
// for byte, from the same function, with the header's fields beside them.
static int op_page(ctlw_t *w, const ctlj_doc_t *d, int args, bool hashes) {
    uint32_t cursor = (uint32_t)d->node[ctlj_find(d, args, "cursor")].num;
    uint32_t max = (uint32_t)d->node[ctlj_find(d, args, "max_entries")].num;
    uint32_t esize = hashes ? sizeof(obs_hash_entry_t) : sizeof(obs_log_entry_t);
    uint32_t cap = OBS_PAGE_HEADER + max * esize;
    uint32_t n = hashes ? obs_hash_page(cursor, page_buf, cap) : obs_log_page(cursor, page_buf, cap);
    ctlw_hex(w, "page_hex", page_buf, n);
    ctlw_uint(w, "first_seq", le32(page_buf + 8));
    ctlw_uint(w, "entries", (uint32_t)page_buf[12] | ((uint32_t)page_buf[13] << 8));
    ctlw_uint(w, "next_seq", le32(page_buf + 16));
    ctlw_uint(w, "oldest_seq", le32(page_buf + 20));
    ctlw_bool(w, "lost", page_buf[28] & 1);
    if (hashes) ctlw_str(w, "engine", engine_name(page_buf[28]));
    return CTL_E_NONE;
}
static int op_log_read(ctlw_t *w, const ctlj_doc_t *d, int args) { return op_page(w, d, args, false); }
static int op_hashes_read(ctlw_t *w, const ctlj_doc_t *d, int args) { return op_page(w, d, args, true); }

static const char *const fail_name[] = {
    "NONE", "NOT_READY", "ERR", "TIMEOUT", "STALE_DRQ", "NO_GEOMETRY", "NO_TIME", "BAD_END",
    "MANUAL_CHS", "MCHS_NOT_READY", "NO_DRIVE", "SELF_RESET",
};

static void fault_fields(ctlw_t *w, const char *key, const ide_fault_t *f) {
    ctlw_obj(w, key);
    ctlw_uint(w, "core", f->core);
    ctlw_uint(w, "pc", f->pc);
    ctlw_uint(w, "lr", f->lr);
    ctlw_uint(w, "cfsr", f->cfsr);
    ctlw_end_obj(w);
}

// debug.e (4.8): Debug E's records, from RAM only (its first line's
// register reads are taskfile.snapshot's). Each record is copied twice and
// compared, since core 0's own USB callbacks write them between passes of
// this loop, never during one, but core 1's Ctrl+G and Auto Detect may.
static int op_debug_e(ctlw_t *w, const ctlj_doc_t *d, int args) {
    int inc = ctlj_find(d, args, "include_data");
    bool data = inc > 0 && d->node[inc].num;
    static ide_fail_t f1, f2;
    static ide_salvage_t s1, s2;
    static ide_self_resets_t r1, r2;
    bool same = false;
    for (int tries = 0; tries < 3 && !same; tries++) {
        ide_last_failure(&f1); ide_salvage_get(&s1); ide_self_resets(&r1);
        ide_last_failure(&f2); ide_salvage_get(&s2); ide_self_resets(&r2);
        same = memcmp(&f1, &f2, sizeof f1) == 0 && memcmp(&s1, &s2, sizeof s1) == 0 &&
               memcmp(&r1, &r2, sizeof r1) == 0;
    }
    if (!same) return CTL_E_BUSY_STATE;
    ctlw_obj(w, "last_failure");
    ctlw_str(w, "kind", f1.kind <= IDE_FAIL_SELF_RESET ? fail_name[f1.kind] : "UNKNOWN");
    ctlw_uint(w, "command", f1.command);
    ctlw_uint(w, "status", f1.status);
    ctlw_uint(w, "error", f1.error);
    ctlw_arr(w, "tf");
    for (int i = 0; i < 5; i++) ctlw_uint(w, 0, f1.tf[i]);
    ctlw_end_arr(w);
    ctlw_bool(w, "drained", f1.drained);
    ctlw_bool(w, "salvaged", f1.salvaged);
    ctlw_bool(w, "reset", f1.reset);
    ctlw_bool(w, "hw_reset", f1.hw_reset);
    ctlw_bool(w, "reset_failed", f1.reset_failed);
    ctlw_bool(w, "pending", f1.pending);
    ctlw_uint(w, "lba", f1.lba);
    ctlw_uint(w, "done", f1.done);
    ctlw_uint(w, "count", f1.count);
    ctlw_end_obj(w);
    uint8_t dg[32];
    ctl_sha256(s1.data, sizeof s1.data, dg);
    ctlw_obj(w, "salvage");
    ctlw_bool(w, "valid", s1.valid);
    ctlw_uint(w, "seq", s1.seq);
    ctlw_uint(w, "lba", s1.lba);
    ctlw_uint(w, "status", s1.status);
    ctlw_uint(w, "error", s1.error);
    ctlw_uint(w, "command", s1.command);
    ctlw_uint(w, "ms", s1.ms);
    ctlw_hex(w, "data_sha256", dg, sizeof dg);
    if (data) ctlw_hex(w, "data_hex", s1.data, sizeof s1.data);     // drive bytes: hex only
    ctlw_end_obj(w);
    ctlw_obj(w, "self_resets");
    ctlw_uint(w, "detected", r1.detected);
    ctlw_uint(w, "idp_resent", r1.idp_resent);
    ctlw_uint(w, "native_failed", r1.native_failed);
    ctlw_uint(w, "last_lba", r1.last_lba);
    ctlw_bool(w, "unavailable", r1.unavailable);
    ctlw_uint(w, "unheld", r1.unheld);
    ctlw_end_obj(w);
    ctlw_uint(w, "unwitnessed_blocks", ide_unwitnessed_blocks());
    ide_fault_t ft;
    if (ide_fault_last(&ft)) fault_fields(w, "fault_before_boot", &ft);
    uint32_t rf[4];
    ide_refused_counts(rf);
    ctlw_obj(w, "refused");
    ctlw_uint(w, "identify_call", rf[0]);
    ctlw_uint(w, "identify_run", rf[1]);
    ctlw_uint(w, "idp_native", rf[2]);
    ctlw_uint(w, "idp_zero", rf[3]);
    ctlw_end_obj(w);
    return CTL_E_NONE;
}

static int op_selftest_status(ctlw_t *w, const ctlj_doc_t *d, int args) {
    (void)d; (void)args;
    uint8_t h[OBS_PAGE_HEADER];
    obs_hash_page(0, h, OBS_PAGE_HEADER);
    ctlw_str(w, "hash_engine", engine_name(h[28]));
    // Core 1 checks its canary every pass and stops the firmware on a hit,
    // so a damaged one is seen here only in the moment before that.
    ctlw_str(w, "core1_canary", core1_stack_intact() ? "INTACT" : "DAMAGED");
    ide_fault_t ft;
    if (ide_fault_last(&ft)) fault_fields(w, "fault_before_boot", &ft);
    else ctlw_null(w, "fault_before_boot");
    ctlw_str(w, "config", config_loaded_from_flash() ? "FLASH" : "DEFAULTS");
    ctlw_str(w, "image_sha256", "NOT_COMPUTED");     // ATABOY_CTL_SELFHASH is not built (4.10)
#if ATABOY_BOARD_JUMPERS
    ctlw_str(w, "jumpers", bj_pads_ok() ? "OK" : "PAD_CONFIG_MISMATCH");     // 4.10: read back now
#else
    ctlw_str(w, "jumpers", "NOT_BUILT");
#endif
    return CTL_E_NONE;
}

static int op_chip_info(ctlw_t *w, const ctlj_doc_t *d, int args) {
    (void)d; (void)args;
    ctlw_str(w, "die", "RP2350");
    ctlw_uint(w, "stepping", rp2350_chip_version());
    ctlw_str(w, "unique_id", c.serial);
    ctlw_str(w, "package", "UNKNOWN");
    return CTL_E_NONE;
}

// ---------------------------------------------------------------------------
//  The ratchet on core 0 (Tier S; SPEC-v1 4.12): wp.on, pipelined.off,
//  identify.forbid. No lease, no jumper; allowed while a job runs. Each is
//  one RAM store toward the safe value, never back, taking effect at once
//  (between USB callbacks, never inside one: both are core 0's). What each
//  races is stated in SPEC-v1 4.12 and 6.2. Each DONE writes one API entry
//  into the command log (5.4) after its store; a refusal writes nothing.
// ---------------------------------------------------------------------------

// expect_epoch, when given (4.3): the state is still the one the client saw.
static int epoch_named(const ctlj_doc_t *d, int args) {
    int n = ctlj_find(d, args, "expect_epoch");
    if (n > 0 && (uint32_t)d->node[n].num != ctl_epoch) { c.err_field = "expect_epoch"; return CTL_E_STALE; }
    return CTL_E_NONE;
}

static int op_wp_on(ctlw_t *w, const ctlj_doc_t *d, int args) {
    int e = epoch_named(d, args);
    if (e) return e;
    bool was = config.drive_write_protected;
    config.drive_write_protected = true;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);    // seen by the next WRITE(10) check (usb.c, TinyUSB)
    if (!was) { ctl_wp_last = CTL_WP_API; CTL_EPOCH_BUMP(); }
    uint32_t seq = obs_api(CTL_OP_WP_ON, c.cur_id, OBS_API_DONE);
    write_wp(w);
    ctlw_bool(w, "changed", !was);
    ctlw_uint(w, "log_seq", seq);
    return CTL_E_NONE;
}

// Taken at the next READ(10) (usb.c); a READ(10) in progress ends as it began.
static int op_pipelined_off(ctlw_t *w, const ctlj_doc_t *d, int args) {
    int e = epoch_named(d, args);
    if (e) return e;
    bool was = config.pipelined_reads;
    config.pipelined_reads = false;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    if (was) CTL_EPOCH_BUMP();
    (void)obs_api(CTL_OP_PIPELINED_OFF, c.cur_id, OBS_API_DONE);
    ctlw_bool(w, "pipelined_reads", config.pipelined_reads);
    ctlw_bool(w, "changed", was);
    return CTL_E_NONE;
}

// Sticky until power-up (ide.c). The API entry comes after the store: an
// ECh logged after it is a finding, never "forbidden in time" (4.12).
static int op_identify_forbid(ctlw_t *w, const ctlj_doc_t *d, int args) {
    int e = epoch_named(d, args);
    if (e) return e;
    bool changed = ide_identify_forbid();
    uint32_t seq = obs_api(CTL_OP_IDENTIFY_FORBID, c.cur_id, OBS_API_DONE);
    ctlw_bool(w, "identify_forbidden", ide_identify_forbidden());
    ctlw_bool(w, "changed", changed);
    ctlw_uint(w, "log_seq", seq);
    return CTL_E_NONE;
}

// job.last (4.11): the last job that ran (its API_BEGIN was written). A job
// refused at pickup, or withdrawn before it, never ran and is not kept.
static int op_job_last(ctlw_t *w, const ctlj_doc_t *d, int args) {
    (void)d; (void)args;
    ctlw_bool(w, "present", c.jl_present);
    if (!c.jl_present) return CTL_E_NONE;
    ctlw_str(w, "op", ctl_ops[c.jl_op].name);
    ctlw_uint(w, "id", c.jl_id);
    ctlw_hex(w, "request_sha256", c.jl_digest, 32);
    ctlw_uint(w, "started_ms", c.jl_started);
    ctlw_uint(w, "ended_ms", c.jl_ended);
    ctlw_uint(w, "log_seq_begin", c.jl_begin);
    ctlw_uint(w, "log_seq_end", c.jl_end);
    ctlw_raw(w, "response", c.jl_resp, c.jl_len);
    return CTL_E_NONE;
}

// ---------------------------------------------------------------------------
//  Stage 4: the lease (SPEC-v1 4.13, 6.1)
// ---------------------------------------------------------------------------

// session.open. tui_lock (6.4) is not built in this stage: asked for, it is
// refused and no lease opens (the host client never asks it by default).
static int op_session_open(ctlw_t *w, const ctlj_doc_t *d, int args) {
    if (d->node[ctlj_find(d, args, "tui_lock")].num) { c.err_field = "tui_lock"; c.err_why = "TUI_LOCK_NOT_BUILT"; return CTL_E_PRECONDITION; }
    uint32_t now = now_ms();
    if (c.lease.held) {
        c.ex_client = c.lease.client;
        c.ex_idle = true;
        c.ex_idle_ms = lease_idle_left(now);
        return CTL_E_LEASE_HELD;
    }
    static const char hx[] = "0123456789abcdef";
    uint64_t r0 = get_rand_64(), r1 = get_rand_64();
    for (int i = 0; i < 16; i++) {
        c.lease.token[i] = hx[(r0 >> (60 - 4 * i)) & 15];
        c.lease.token[16 + i] = hx[(r1 >> (60 - 4 * i)) & 15];
    }
    c.lease.token[32] = 0;
    int cl = ctlj_find(d, args, "client");
    uint32_t n = d->node[cl].str_len;
    memcpy(c.lease.client, d->line + d->node[cl].str_off, n);
    c.lease.client[n] = 0;
    c.lease.idle_ms = (uint32_t)d->node[ctlj_find(d, args, "idle_s")].num * 1000u;
    c.lease.last_ms = now;
    c.lease.gen = ++c.lease.next_gen;
    c.lease.held = true;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    ctl_lease_gen = c.lease.gen;
    CTL_EPOCH_BUMP();                                       // the lease is state (4.3)
    (void)obs_api(CTL_OP_SESSION_OPEN, c.cur_id, OBS_API_DONE);
    ctlw_str(w, "token", c.lease.token);
    ctlw_uint(w, "idle_s", c.lease.idle_ms / 1000u);
    ctlw_bool(w, "tui_lock", false);
    return CTL_E_NONE;
}

// The token of a session.close or Tier M request (4.14 step 4). The right
// one renews the lease's idle timer; the last lease's gets E_LEASE_REVOKED
// with how it ended.
static int lease_check(const ctlj_doc_t *d, int token, uint8_t *by) {
    *by = CTL_BY_NONE;
    if (c.lease.held && ctlj_str_is(d, token, c.lease.token)) { c.lease.last_ms = now_ms(); return CTL_E_NONE; }
    if (c.lease.old_valid && ctlj_str_is(d, token, c.lease.old_token)) { *by = c.lease.old_by; return CTL_E_LEASE_REVOKED; }
    return c.lease.held ? CTL_E_LEASE_INVALID : CTL_E_NO_LEASE;
}

// An answer owed, sent in order before any new line is read (outq_poll).
enum { CTL_OUTQ_REFUSE, CTL_OUTQ_CLOSED };
static void outq_push(uint8_t kind, uint32_t id, int slot, uint32_t port_gen, uint8_t err, uint8_t retry, uint8_t by,
                      const char *msg, const char *why) {
    if (c.outq_n >= sizeof c.outq / sizeof c.outq[0]) return;      // never: at most two withdrawals and a close
    c.outq[c.outq_n].kind = kind; c.outq[c.outq_n].id = id; c.outq[c.outq_n].slot = slot;
    c.outq[c.outq_n].port_gen = port_gen; c.outq[c.outq_n].err = err; c.outq[c.outq_n].retry = retry;
    c.outq[c.outq_n].by = by; c.outq[c.outq_n].msg = msg; c.outq[c.outq_n].why = why;
    c.outq_n++;
}

static void presence_publish(void);

// The waiting request leaves the slot without running (4.14): NOT_EXECUTED,
// answered in order. Its replay slot keeps that answer.
static void pres_withdraw(uint8_t err, uint8_t by, const char *msg, const char *why) {
    if (!c.pres.waiting) return;
    c.pres.waiting = false;
    presence_publish();
    outq_push(CTL_OUTQ_REFUSE, c.pres.id, c.pres.slot, c.port_gen, err,
              err == CTL_E_STALE ? CTL_RETRY_AFTER_STATUS : CTL_RETRY_SAFE, by, msg, why);
}

// The lease ends (6.1): its waiting request and a job of it still queued
// are withdrawn (E_LEASE_REVOKED, by), its credit ends, its token is kept
// only to say how it ended. A job of it already running is not stopped.
static void lease_end(uint8_t by) {
    if (!c.lease.held) return;
    pres_withdraw(CTL_E_LEASE_REVOKED, by, "the lease ended", 0);
    if (ctl_job.state == CTL_JOB_QUEUED && ctl_job.lease_gen == c.lease.gen && ctl_job_cas(CTL_JOB_QUEUED, CTL_JOB_EMPTY)) {
        outq_push(CTL_OUTQ_REFUSE, ctl_job.id, c.job_slot, c.job_port_gen, CTL_E_LEASE_REVOKED, CTL_RETRY_SAFE, by,
                  "the lease ended before pickup", 0);
        c.job_slot = -1;
    }
    c.credit.live = false;
    memcpy(c.lease.old_token, c.lease.token, sizeof c.lease.token);
    c.lease.old_by = by;
    c.lease.old_valid = true;
    c.lease.held = false;
    ctl_lease_by = by;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    ctl_lease_gen = 0;
    CTL_EPOCH_BUMP();
}

// session.close (4.13): its withdrawals answered first, then its own DONE.
static void session_close(uint32_t id, int slot, const ctlj_doc_t *d, int token) {
    uint8_t by;
    int e = lease_check(d, token, &by);
    if (e != CTL_E_NONE) {
        c.ex_by = by;
        uint32_t n = send_error(true, id, CTL_OUT_NOT_EXECUTED, CTL_RETRY_SAFE, (uint8_t)e,
                                e == CTL_E_LEASE_REVOKED ? "that lease has ended" :
                                e == CTL_E_NO_LEASE ? "no lease is held" : "not the lease's token", 0, "token", true);
        if (slot >= 0) ctl_replay_finish(&c.replay, slot, c.resp, n);
        return;
    }
    (void)obs_api(CTL_OP_SESSION_CLOSE, id, OBS_API_DONE);
    lease_end(CTL_BY_CLOSED);
    outq_push(CTL_OUTQ_CLOSED, id, slot, c.port_gen, 0, 0, 0, 0, 0);
}

// The lease's idle timer, paused while a request of it waits for presence
// or a job of it is queued, running or not yet answered (6.1, finding 14).
static void lease_poll(uint32_t now) {
    if (!c.lease.held) return;
    uint32_t js = ctl_job.state;
    bool paused = c.pres.waiting || (js != CTL_JOB_EMPTY && ctl_job.lease_gen == c.lease.gen);
    if (paused) c.lease.last_ms = now;
    else if (now - c.lease.last_ms >= c.lease.idle_ms) lease_end(CTL_BY_IDLE);
}

// ---------------------------------------------------------------------------
//  Stage 4: Tier M, request first (SPEC-v1 4.14; owner, R-API-1 finding 3)
// ---------------------------------------------------------------------------

// The presence row's text for the TUI (ctl.h), behind a sequence count:
// core 0 writes, core 1 copies, and a copy that saw the count move is taken
// again.
static volatile uint32_t pres_seq;
static char pres_text[CTL_PRESENCE_TEXT];
static void presence_publish(void) {
    __atomic_fetch_add(&pres_seq, 1u, __ATOMIC_SEQ_CST);        // odd: being written
    if (c.pres.waiting) {
        static const char hx[] = "0123456789abcdef";
        char h8[9];
        for (int i = 0; i < 4; i++) { h8[2 * i] = hx[c.pres.digest[i] >> 4]; h8[2 * i + 1] = hx[c.pres.digest[i] & 15]; }
        h8[8] = 0;
        // A setup.ctrlg sent with then_mount approves a mount as well (one act,
        // 4.14): the row says so, in the same 11 characters, so the longest
        // text still fits its 37 cells (review A MED-3).
        const char *label = c.pres.op == CTL_OP_SETUP_CTRLG && c.pres.m.then_mount ? "ctrlg+mount" : ctl_ops[c.pres.op].name;
        snprintf(pres_text, sizeof pres_text, "JP2? %s #%lu %s", label, (unsigned long)c.pres.id, h8);
    } else {
        pres_text[0] = 0;
    }
    __atomic_fetch_add(&pres_seq, 1u, __ATOMIC_SEQ_CST);        // even: done
}
bool ctl_presence_text(char *out, unsigned n) {
    for (int tries = 0; tries < 8; tries++) {
        uint32_t s0 = __atomic_load_n(&pres_seq, __ATOMIC_SEQ_CST);
        if (s0 & 1) continue;
        unsigned k = 0;
        for (; k + 1 < n && k + 1 < sizeof pres_text && pres_text[k]; k++) out[k] = pres_text[k];
        out[k] = 0;
        if (__atomic_load_n(&pres_seq, __ATOMIC_SEQ_CST) == s0) return out[0] != 0;
    }
    out[0] = 0;
    return false;
}

// Tier M's jumper precondition (4.14 step 5): JP1 open and readable, the
// jumpers built. why: the capability's word for it.
static bool jumpers_forbid(const char **why) {
#if ATABOY_BOARD_JUMPERS
    if (bj_tier_m_open()) return false;
    *why = bj_jp1() == BJ_FITTED ? "JP1_FITTED" : "JUMPERS_UNREADABLE";
#else
    *why = "JUMPERS_UNSUPPORTED";
#endif
    return true;
}

// A request into the mailbox (5.3): the approved one, or a mount on a credit.
static void mailbox_put(uint8_t op, uint32_t id, const uint8_t digest[32], const ctl_mreq_t *m, uint32_t lease_gen,
                        bool via_credit, int slot) {
    ctl_job.op = op;
    ctl_job.id = id;
    memcpy(ctl_job.digest, digest, 32);
    ctl_job.m = *m;
    ctl_job.expect_mount_seq = 0;
    ctl_job.has_epoch = false;
    ctl_job.lease_gen = lease_gen;
    ctl_job.via_credit = via_credit;
    ctl_job.credit_end_ms = c.credit.end_ms;
    ctl_job.credit_epoch = c.credit.epoch;
    ctl_job.queued_ms = now_ms();
    c.job_port_gen = c.port_gen;
    c.job_slot = slot;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);      // the request before the state
    ctl_job.state = CTL_JOB_QUEUED;
}

// A Tier M request's arrival (4.14), after the replay table: the token, the
// jumpers, expect_epoch, nothing else in flight, every precondition; then
// the mailbox at once (a mount on its pair's credit) or the presence slot.
// A refusal is answered now and changes nothing.
static void tier_m_arrival(uint32_t id, const uint8_t digest[32], const ctlj_doc_t *d, const ctl_req_t *r, int slot) {
    uint8_t by;
    const char *why = 0, *field = 0, *msg = "";
    ctl_mreq_t m = { 0 };
    uint8_t retry = CTL_RETRY_SAFE;
    int e = lease_check(d, r->token, &by);
    if (e != CTL_E_NONE) {
        msg = e == CTL_E_LEASE_REVOKED ? "that lease has ended" : e == CTL_E_NO_LEASE ? "no lease is held" : "not the lease's token";
        field = "token";
    } else if (jumpers_forbid(&why)) {
        e = CTL_E_LOCKED; msg = "JP1 fitted, the jumpers unreadable, or none built";
    } else {
        ctl_mreq_read(d, r->op, r->args, &m);
        if (m.has_epoch && m.expect_epoch != ctl_epoch) {
            e = CTL_E_STALE; msg = "the state changed since expect_epoch"; field = "expect_epoch"; retry = CTL_RETRY_AFTER_STATUS;
        } else if (ctl_job.state != CTL_JOB_EMPTY || c.pres.waiting) {
            e = CTL_E_INFLIGHT; msg = c.pres.waiting ? "a guarded request waits for its act" : "another job is in flight";
        } else {
            e = ctl_mreq_check((uint8_t)r->op, &m, &field, &why);
            if (e != CTL_E_NONE) {
                msg = "a precondition is not met";
                if (e == CTL_E_STALE) retry = CTL_RETRY_AFTER_STATUS;
            } else if (!(r->op == CTL_OP_MOUNT && credit_live(now_ms())) && !(cdc_connected && ctl_tui_state == CTL_TUI_MAIN)) {
                // A request that would wait for its act is refused unless the
                // person can see the presence row: the TUI on the main menu
                // with a terminal connected (owner ruling 2026-09-29, review A
                // MED-3 (2)). Last, so any other refusal keeps its own answer;
                // a mount on the pair's credit waits for nothing and is not
                // asked. Checked at arrival: a terminal that closes during the
                // wait does not end it (FORK-README item 36).
                e = CTL_E_PRECONDITION; why = "NO_TERMINAL"; msg = "no terminal shows the presence row: the TUI must be on the main menu with a terminal connected";
            }
        }
    }
    if (e != CTL_E_NONE) {
        c.ex_by = by;
        uint32_t n = send_error(true, id, CTL_OUT_NOT_EXECUTED, retry, (uint8_t)e, msg, why, field, true);
        if (slot >= 0) ctl_replay_finish(&c.replay, slot, c.resp, n);
        return;
    }
    uint32_t now = now_ms();
    if (r->op == CTL_OP_MOUNT && credit_live(now)) {
        c.credit.live = false;                              // single use: taken now, checked again at pickup
        mailbox_put(CTL_OP_MOUNT, id, digest, &m, c.lease.gen, true, slot);
        return;
    }
    c.pres.waiting = true;
    c.pres.op = (uint8_t)r->op;
    c.pres.id = id;
    memcpy(c.pres.digest, digest, 32);
    c.pres.arrival_ms = now;
    c.pres.arrival_epoch = ctl_epoch;
    c.pres.lease_gen = c.lease.gen;
    c.pres.slot = slot;
    c.pres.m = m;
    presence_publish();
}

// The waiting request, each pass (4.14): withdrawn at JP1, at any epoch
// change since it came, at 120 s; approved by a fit event whose FITTED edge
// came after it did, which moves it to the mailbox in this same step.
static void pres_poll(uint32_t now, bool fit, uint32_t edge_ms) {
    if (!c.pres.waiting) return;
    const char *why;
    if (jumpers_forbid(&why)) { pres_withdraw(CTL_E_LOCKED, CTL_BY_NONE, "JP1 fitted, or the jumpers unreadable", why); return; }
    if (ctl_epoch != c.pres.arrival_epoch) { pres_withdraw(CTL_E_STALE, CTL_BY_NONE, "the state changed while it waited", 0); return; }
    if (now - c.pres.arrival_ms >= CTL_PRESENCE_S * 1000u) {
        pres_withdraw(CTL_E_PRESENCE_REQUIRED, CTL_BY_NONE, "no JP2 act within 120 s", 0);
        return;
    }
    // After, by the difference: the millisecond clock wraps every 49.7 days,
    // and an edge just before a wrap is not later than an arrival after it.
    if (fit && (int32_t)(edge_ms - c.pres.arrival_ms) > 0 && ctl_job.state == CTL_JOB_EMPTY) {
        c.pres.waiting = false;
        presence_publish();
        mailbox_put(c.pres.op, c.pres.id, c.pres.digest, &c.pres.m, c.pres.lease_gen, false, c.pres.slot);
    }
}

// The answers owed, in order, while the writer is free.
static void outq_poll(void) {
    while (c.outq_n && !c.resp_busy) {
        bool send = c.outq[0].port_gen == c.port_gen && c.dtr;
        uint32_t n;
        if (c.outq[0].kind == CTL_OUTQ_CLOSED) {
            ctlw_t w;
            begin(&w, true, c.outq[0].id);
            ctlw_bool(&w, "ok", true);
            ctlw_str(&w, "outcome", "DONE");
            ctlw_obj(&w, "result");
            ctlw_bool(&w, "closed", true);
            ctlw_end_obj(&w);
            n = finish(&w, send);
        } else {
            c.ex_by = c.outq[0].by;
            n = send_error(true, c.outq[0].id, CTL_OUT_NOT_EXECUTED, c.outq[0].retry, c.outq[0].err, c.outq[0].msg,
                           c.outq[0].why, 0, send);
        }
        if (c.outq[0].slot >= 0) ctl_replay_finish(&c.replay, c.outq[0].slot, c.resp, n);
        for (unsigned i = 1; i < c.outq_n; i++) c.outq[i - 1] = c.outq[i];
        c.outq_n--;
    }
}

typedef int (*ctl_op_fn)(ctlw_t *w, const ctlj_doc_t *d, int args);
static ctl_op_fn core0_fn(int op) {
    switch (op) {
    case CTL_OP_HELLO:           return op_hello;
    case CTL_OP_STATUS:          return op_status;
    case CTL_OP_IDENTIFY_CACHED: return op_identify_cached;
    case CTL_OP_CONFIG_GET:      return op_config_get;
    case CTL_OP_LOG_READ:        return op_log_read;
    case CTL_OP_HASHES_READ:     return op_hashes_read;
    case CTL_OP_DEBUG_E:         return op_debug_e;
    case CTL_OP_SELFTEST_STATUS: return op_selftest_status;
    case CTL_OP_CHIP_INFO:       return op_chip_info;
    case CTL_OP_JOB_LAST:        return op_job_last;
    case CTL_OP_WP_ON:           return op_wp_on;
    case CTL_OP_PIPELINED_OFF:   return op_pipelined_off;
    case CTL_OP_IDENTIFY_FORBID: return op_identify_forbid;
    case CTL_OP_SESSION_OPEN:    return op_session_open;
    default:
#if ATABOY_HOST_TEST
        return ctl_test_op[op];
#else
        return 0;
#endif
    }
}

// DONE with the op's result, or its refusal. The length of what was written
// (without the \n), for the replay table.
static uint32_t run_core0(int op, uint32_t id, const ctlj_doc_t *d, int args) {
    ctl_op_fn fn = core0_fn(op);
    ctlw_t w;
    begin(&w, true, id);
    ctlw_bool(&w, "ok", true);
    ctlw_str(&w, "outcome", "DONE");
    ctlw_obj(&w, "result");
    c.cur_id = id;
    c.err_field = 0;
    c.err_why = 0;
    int e = fn ? fn(&w, d, args) : CTL_E_UNKNOWN_OP;
    if (e != CTL_E_NONE)
        return send_error(true, id, CTL_OUT_NOT_EXECUTED, e == CTL_E_STALE ? CTL_RETRY_AFTER_STATUS : CTL_RETRY_SAFE,
                          (uint8_t)e,
                          e == CTL_E_BUSY_STATE ? "the state kept changing while it was read" :
                          e == CTL_E_STALE ? "the state changed since expect_epoch" :
                          e == CTL_E_LEASE_HELD ? "another client holds the lease" :
                          e == CTL_E_PRECONDITION ? "a precondition is not met" : "not available",
                          c.err_why, c.err_field, true);
    ctlw_end_obj(&w);
    uint32_t n = finish(&w, true);
    if (w.overflow)     // never with this build's ops and limits; tested at their largest
        n = send_error(true, id, CTL_OUT_NOT_EXECUTED, CTL_RETRY_SAFE, CTL_E_BUSY_STATE,
                       "response over 16,384 bytes", 0, 0, true);
    return n;
}

// ---------------------------------------------------------------------------
//  Jobs: queued here, run by core 1 (ctl_job.c), answered here
// ---------------------------------------------------------------------------

// Arrival checks for taskfile.snapshot (4.6); every refusal NOT_EXECUTED.
static bool taskfile_arrival_ok(uint32_t id) {
    if (is_mounted) { refuse(true, id, CTL_E_STATE, "mounted: the bus is the host's", 0); return false; }
    if (usb_msc_ide_busy() || ide_recovery_pending()) {
        refuse(true, id, CTL_E_BUS_BUSY, "a USB command or its reset is still running", 0);
        return false;
    }
    return true;
}

// Arrival checks for unmount and detach (4.12; owner decision on R-API-1
// finding 11): the mount they name is the current one, and the epoch the
// one they give. Compared again at pickup (ctl_job.c). A mismatch changes
// nothing (I-27): only the replay table keeps the refusal.
static bool names_current_now(uint32_t id, const ctlj_doc_t *d, int args, uint32_t *ms, bool *has_e, uint32_t *ep) {
    int m = ctlj_find(d, args, "expect_mount_seq");     // required: the schema refused it missing (ctl_policy.c)
    *ms = m > 0 ? (uint32_t)d->node[m].num : 0xFFFFFFFFu;
    int n = ctlj_find(d, args, "expect_epoch");
    *has_e = n > 0;
    *ep = *has_e ? (uint32_t)d->node[n].num : 0;
    if (*ms != ctl_mount_seq) {
        send_error(true, id, CTL_OUT_NOT_EXECUTED, CTL_RETRY_AFTER_STATUS, CTL_E_STALE,
                   "expect_mount_seq is not the current mount", 0, "expect_mount_seq", true);
        return false;
    }
    if (*has_e && *ep != ctl_epoch) {
        send_error(true, id, CTL_OUT_NOT_EXECUTED, CTL_RETRY_AFTER_STATUS, CTL_E_STALE,
                   "the state changed since expect_epoch", 0, "expect_epoch", true);
        return false;
    }
    return true;
}

// detach's lease rule (owner ruling 2026-09-29 on review A MED-2, the
// minimum): a token is asked for only while a lease is held and a set-up is
// in place, the state a stranger's detach would destroy. A token that is
// sent is checked as everywhere else (the last lease's is E_LEASE_REVOKED,
// none held is E_NO_LEASE). unmount takes none: it stays the emergency
// stop. Checked at arrival only, like the mount it names is compared again
// at pickup but the lease is not: a detach queued before a lease opened
// still runs. Refusals change nothing; the replay table keeps them.
static bool detach_lease_ok(uint32_t id, const ctlj_doc_t *d) {
    int t = ctlj_find(d, 0, "token");
    if (t > 0) {
        uint8_t by;
        int e = lease_check(d, t, &by);
        if (e == CTL_E_NONE) return true;
        c.ex_by = by;
        send_error(true, id, CTL_OUT_NOT_EXECUTED, CTL_RETRY_SAFE, (uint8_t)e,
                   e == CTL_E_LEASE_REVOKED ? "that lease has ended" :
                   e == CTL_E_NO_LEASE ? "no lease is held" : "not the lease's token", 0, "token", true);
        return false;
    }
    if (!c.lease.held || ctl_setup_src == CTL_SRC_NONE) return true;
    c.ex_client = c.lease.client;
    c.ex_idle = true;
    c.ex_idle_ms = lease_idle_left(now_ms());
    send_error(true, id, CTL_OUT_NOT_EXECUTED, CTL_RETRY_SAFE, CTL_E_LEASE_HELD,
               "another client holds the lease and a set-up is in place: detach needs its token", 0, "token", true);
    return false;
}

static bool queue_job(int op, uint32_t id, const uint8_t digest[32], const ctlj_doc_t *d, int args) {
    uint32_t ms = 0, ep = 0;
    bool has_e = false;
    if (op == CTL_OP_DETACH && !detach_lease_ok(id, d)) return false;
    if (op == CTL_OP_TASKFILE_SNAPSHOT && !taskfile_arrival_ok(id)) return false;
    if ((op == CTL_OP_UNMOUNT || op == CTL_OP_DETACH) && !names_current_now(id, d, args, &ms, &has_e, &ep)) return false;
    if (ctl_job.state != CTL_JOB_EMPTY) { refuse(true, id, CTL_E_INFLIGHT, "another job is in flight", 0); return false; }
    // A guarded request waiting for its act is in flight for
    // taskfile.snapshot (SPEC-v1 3.5): the mailbox stays free for it.
    if (op == CTL_OP_TASKFILE_SNAPSHOT && c.pres.waiting) {
        refuse(true, id, CTL_E_INFLIGHT, "a guarded request waits for its act", 0);
        return false;
    }
    ctl_job.op = (uint8_t)op;
    ctl_job.id = id;
    ctl_job.lease_gen = 0;
    ctl_job.via_credit = false;
    ctl_job.expect_mount_seq = ms;
    ctl_job.has_epoch = has_e;
    ctl_job.expect_epoch = ep;
    memcpy(ctl_job.digest, digest, 32);
    ctl_job.queued_ms = now_ms();
    c.job_port_gen = c.port_gen;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);      // the request before the state
    ctl_job.state = CTL_JOB_QUEUED;
    // The ratchet wins (SPEC-v1 3.5, 4.14): an unmount or detach that names
    // the current mount withdraws a waiting guarded request, whether or not
    // it will change anything (with nothing mounted, unmount changes no
    // state and so no epoch). One naming another mount was refused above
    // and left it alone (I-27).
    if (op == CTL_OP_UNMOUNT || op == CTL_OP_DETACH)
        pres_withdraw(CTL_E_STALE, CTL_BY_NONE, "an unmount or detach came while it waited", 0);
    // No answer now: it comes when the job ends or is withdrawn.
    return true;
}

static void answer_job(void) {
    ctl_job_t *j = &ctl_job;
    bool send = c.job_port_gen == c.port_gen && c.dtr;
    ctlw_t w;
    uint32_t n;
    if (j->outcome == CTL_OUT_DONE && j->op == CTL_OP_UNMOUNT) {
        begin(&w, true, j->id);
        ctlw_bool(&w, "ok", true);
        ctlw_str(&w, "outcome", "DONE");
        ctlw_obj(&w, "result");
        ctlw_bool(&w, "mounted", is_mounted);
        ctlw_bool(&w, "changed", j->changed);
        ctlw_bool(&w, "native_mode_ended", j->native_ended);
        ctlw_end_obj(&w);
        n = finish(&w, send);
    } else if (j->outcome == CTL_OUT_DONE && j->op == CTL_OP_SETUP_CTRLG) {
        // The declared pair's credit (Q3): earned by a DONE set-up that asked
        // for it, of a lease still held, from its end.
        bool credit = j->m.then_mount && c.lease.held && c.lease.gen == j->lease_gen;
        if (credit) {
            c.credit.live = true;
            c.credit.lease_gen = j->lease_gen;
            c.credit.end_ms = j->ended_ms;
            c.credit.epoch = j->end_epoch;
        }
        begin(&w, true, j->id);
        ctlw_bool(&w, "ok", true);
        ctlw_str(&w, "outcome", "DONE");
        ctlw_obj(&w, "result");
        ctlw_uint(&w, "status", j->status);
        ctlw_str(&w, "mode", j->m.native ? "native" : "idp");
        ctlw_str(&w, "source", j->m.native ? "API_CTRLG_NATIVE" : "API_CTRLG_IDP");
        ctlw_str(&w, "device", j->m.dev_base == 0xB0 ? "SLAVE" : "MASTER");
        ctlw_obj(&w, "geometry");
        ctlw_uint(&w, "cyls", j->m.cyls);
        ctlw_uint(&w, "heads", j->m.heads);
        ctlw_uint(&w, "spt", j->m.spt);
        ctlw_uint(&w, "total_sectors", (uint64_t)j->m.cyls * j->m.heads * j->m.spt);
        ctlw_end_obj(&w);
        ctlw_bool(&w, "mount_credit", credit);
        ctlw_end_obj(&w);
        n = finish(&w, send);
    } else if (j->outcome == CTL_OUT_DONE && j->op == CTL_OP_MOUNT) {
        begin(&w, true, j->id);
        ctlw_bool(&w, "ok", true);
        ctlw_str(&w, "outcome", "DONE");
        ctlw_obj(&w, "result");
        ctlw_bool(&w, "mounted", true);
        ctlw_uint(&w, "mount_seq", j->mount_seq);
        ctlw_uint(&w, "mount_log_seq", j->mount_log_seq);
        ctlw_uint(&w, "total_sectors", j->total_sectors);
        ctlw_bool(&w, "used_credit", j->via_credit);
        ctlw_end_obj(&w);
        n = finish(&w, send);
    } else if (j->outcome == CTL_OUT_DONE && j->op == CTL_OP_DETACH) {
        begin(&w, true, j->id);
        ctlw_bool(&w, "ok", true);
        ctlw_str(&w, "outcome", "DONE");
        ctlw_obj(&w, "result");
        ctlw_bool(&w, "detached", true);
        ctlw_end_obj(&w);
        n = finish(&w, send);
    } else if (j->outcome == CTL_OUT_DONE) {
        begin(&w, true, j->id);
        ctlw_bool(&w, "ok", true);
        ctlw_str(&w, "outcome", "DONE");
        ctlw_obj(&w, "result");
        ctlw_uint(&w, "error", j->tf[1]);
        ctlw_uint(&w, "count", j->tf[2]);
        ctlw_uint(&w, "sector", j->tf[3]);
        ctlw_uint(&w, "cyl_lo", j->tf[4]);
        ctlw_uint(&w, "cyl_hi", j->tf[5]);
        ctlw_uint(&w, "device", j->tf[6]);
        ctlw_uint(&w, "alt_status", j->alt);
        static const char *const bit[8] = { "AMNF", "TK0NF", "ABRT", "MCR", "IDNF", "MC", "UNC", "BBK" };
        ctlw_arr(&w, "err_bits");
        for (int b = 7; b >= 0; b--) if (j->tf[1] & (1u << b)) ctlw_str(&w, 0, bit[b]);
        ctlw_end_arr(&w);
        ctlw_uint(&w, "at_us", j->at_us);
        ctlw_uint(&w, "log_seq_before", j->log_begin);
        ctlw_end_obj(&w);
        n = finish(&w, send);
    } else {
        uint8_t retry = j->outcome == CTL_OUT_NOT_EXECUTED && j->err != CTL_E_STALE ? CTL_RETRY_SAFE : CTL_RETRY_AFTER_STATUS;
        fail_detach = j->op == CTL_OP_DETACH && j->outcome == CTL_OUT_FAILED;
        c.ex_by = j->by;
        n = send_error(true, j->id, j->outcome, retry, j->err, j->msg, j->why, j->field, send);
    }
    if (j->ran) {
        c.jl_present = true;
        c.jl_op = j->op;
        c.jl_id = j->id;
        memcpy(c.jl_digest, j->digest, 32);
        c.jl_started = j->started_ms;
        c.jl_ended = j->ended_ms;
        c.jl_begin = j->log_begin;
        c.jl_end = j->log_end;
        c.jl_len = n <= CTL_JOB_LAST_BYTES ? n : 0;
        memcpy(c.jl_resp, c.resp, c.jl_len);
    }
    if (c.job_slot >= 0) { ctl_replay_finish(&c.replay, c.job_slot, c.resp, n); c.job_slot = -1; }
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    j->state = CTL_JOB_EMPTY;
}

static void job_poll(uint32_t now) {
    if (c.resp_busy) return;                      // one answer at a time
    uint32_t st = ctl_job.state;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);      // the state before core 1's result
    if (st == CTL_JOB_ENDED) { answer_job(); return; }
    if (st == CTL_JOB_QUEUED && now - ctl_job.queued_ms >= CTL_TUI_WAIT_MS && ctl_job_cas(CTL_JOB_QUEUED, CTL_JOB_EMPTY)) {
        // Withdrawn: nothing of it ran (the CAS decided it against a pickup).
        uint8_t ts = ctl_tui_state;
        uint32_t n = send_error(true, ctl_job.id, CTL_OUT_NOT_EXECUTED, CTL_RETRY_SAFE, CTL_E_TUI_BUSY,
                                "the TUI was not at its idle point within 30 s",
                                ts <= CTL_TUI_MODAL ? tui_name[ts] : 0, 0,
                                c.job_port_gen == c.port_gen && c.dtr);
        if (c.job_slot >= 0) { ctl_replay_finish(&c.replay, c.job_slot, c.resp, n); c.job_slot = -1; }
    }
}

// ---------------------------------------------------------------------------
//  One line
// ---------------------------------------------------------------------------

static bool read_id(const ctlj_doc_t *d, uint32_t *id) {
    int n = ctlj_find(d, 0, "id");
    if (n < 0 || d->node[n].type != CTLJ_INT || d->node[n].num < 1 || d->node[n].num > CTL_MAX_ID) return false;
    *id = (uint32_t)d->node[n].num;
    return true;
}

// Resend a stored response as it was sent, dev block and all, with
// "replayed":true added; it takes no new seq (3.5, client ambiguity A7).
static void send_replay(int slot) {
    const ctl_rp_slot_t *s = &c.replay.slot[slot];
    static const char tail[] = ",\"replayed\":true}\n";
    uint32_t n = s->len;                          // ends in its closing brace
    memcpy(c.resp, s->resp, n - 1);
    memcpy(c.resp + n - 1, tail, sizeof tail - 1);
    c.resp_len = n - 1 + sizeof tail - 1;
    c.resp_sent = 0;
    c.resp_at_ms = now_ms();
    c.resp_busy = true;
}

static void handle_line(uint32_t len) {
    char *line = c.line;
    if (len && line[len - 1] == '\r') len--;      // one optional \r before the \n
    if (len == 0) return;                         // an empty line: no answer (a host resyncs with \n)
    for (uint32_t i = 0; i < len; i++) {
        if (line[i] < 0x20 || line[i] > 0x7E) {   // 0x00-0x1F, 0x7F, anything >= 0x80
            send_error(false, 0, CTL_OUT_NOT_EXECUTED, CTL_RETRY_SAFE, CTL_E_PARSE,
                       "a byte outside 0x20-0x7E", 0, 0, true);
            return;
        }
    }
    ctlj_doc_t *d = &c.doc;
    int pr = ctlj_parse(line, len, d);
    if (pr == CTLJ_E_PARSE) {
        send_error(false, 0, CTL_OUT_NOT_EXECUTED, CTL_RETRY_SAFE, CTL_E_PARSE,
                   "not a request in the v1 grammar", 0, 0, true);
        return;
    }
    uint32_t id = 0;
    bool id_ok = read_id(d, &id);
    if (!ctl_rate_admit(&c.rate, now_ms())) {
        refuse(id_ok, id, CTL_E_RATE, "more than 20 requests in 1 s", 0);
        return;
    }
    ctl_req_t r;
    int e = ctl_check_request(d, &r);
    if (e != CTL_E_PROTO_VERSION && pr == CTLJ_E_DUP) { e = CTL_E_SCHEMA; r.field = "request"; }
#if ATABOY_HOST_TEST
    if (e == CTL_E_UNKNOWN_OP && r.op >= 0 && ctl_test_op[r.op]) e = ctl_check_args(d, &r);
#endif
    if (e != CTL_E_NONE) {
        const char *msg = e == CTL_E_PROTO_VERSION ? "this build speaks v1" :
                          e == CTL_E_UNKNOWN_OP ? "no such op in this build" :
                          e == CTL_E_RANGE ? "an argument is out of range" : "not the op's schema";
        refuse(r.id_ok, r.id, (uint8_t)e, msg, r.field);
        return;
    }
    uint8_t digest[32];
    ctl_sha256(line, len, digest);                // the line without its \r\n (3.5)
    int slot = -1;
    if (ctl_op_mutating(r.op)) {
        switch (ctl_replay_check(&c.replay, r.id, digest, &slot)) {
        case CTL_RPV_REPLAY:
            send_replay(slot);
            return;
        case CTL_RPV_INFLIGHT:
            send_error(true, r.id, CTL_OUT_NOT_EXECUTED, CTL_RETRY_AFTER_STATUS, CTL_E_INFLIGHT,
                       "this request is still in flight", 0, 0, true);
            return;
        case CTL_RPV_REUSED:
            send_error(true, r.id, CTL_OUT_NOT_EXECUTED, CTL_RETRY_UNSAFE, CTL_E_ID_REUSED,
                       "this id was used for another request", 0, "id", true);
            return;
        case CTL_RPV_STALE:
            refuse(true, r.id, CTL_E_STALE_ID, "id not above max_mutating_id; an earlier send of it may have run: read status, do not resend under a new id", "id");
            return;
        case CTL_RPV_LOST:
            send_error(true, r.id, CTL_OUT_NOT_EXECUTED, CTL_RETRY_AFTER_STATUS, CTL_E_BUSY_STATE,
                       "its response was not kept: read status", 0, 0, true);
            return;
        default:
            slot = ctl_replay_begin(&c.replay, r.id, digest);
            if (slot < 0) {
                send_error(true, r.id, CTL_OUT_NOT_EXECUTED, CTL_RETRY_SAFE, CTL_E_INFLIGHT,
                           "no replay slot free", 0, 0, true);
                return;
            }
        }
    }
    if (r.op == CTL_OP_SESSION_CLOSE) { session_close(r.id, slot, d, r.token); return; }
    if (ctl_ops[r.op].tier == CTL_TIER_M) { tier_m_arrival(r.id, digest, d, &r, slot); return; }
    if (ctl_ops[r.op].job) {
        if (queue_job(r.op, r.id, digest, d, r.args)) { c.job_slot = slot; return; }
        if (slot >= 0) ctl_replay_finish(&c.replay, slot, c.resp, c.resp_len - 1);
        return;
    }
    uint32_t n = run_core0(r.op, r.id, d, r.args);
    if (slot >= 0) ctl_replay_finish(&c.replay, slot, c.resp, n);
}

// ---------------------------------------------------------------------------
//  The port
// ---------------------------------------------------------------------------

// The line and anything waiting to go out are dropped, and a job queued but
// not yet picked up is withdrawn (SPEC-v1 2.4): nobody is there for its
// answer. A job already running runs to its end and is kept as job.last.
static void port_reset(void) {
    c.len = 0;
    c.overflow = false;
    c.quiet_ms = 0;
    c.resp_busy = false;
    tud_cdc_n_write_clear(CTL_ITF);
    c.port_gen++;
    if (ctl_job.state == CTL_JOB_QUEUED && ctl_job_cas(CTL_JOB_QUEUED, CTL_JOB_EMPTY) && c.job_slot >= 0) {
        uint32_t n = send_error(true, ctl_job.id, CTL_OUT_NOT_EXECUTED, CTL_RETRY_SAFE, CTL_E_TUI_BUSY,
                                "withdrawn: the port closed before pickup", 0, 0, false);
        ctl_replay_finish(&c.replay, c.job_slot, c.resp, n);
        c.job_slot = -1;
    }
}

static void tx_pump(uint32_t now) {
    if (!c.resp_busy) return;
    uint32_t room = tud_cdc_n_write_available(CTL_ITF);
    if (room) {
        uint32_t k = c.resp_len - c.resp_sent;
        if (k > room) k = room;
        c.resp_sent += tud_cdc_n_write(CTL_ITF, c.resp + c.resp_sent, k);
        tud_cdc_n_write_flush(CTL_ITF);
    }
    if (c.resp_sent >= c.resp_len) c.resp_busy = false;
    else if (now - c.resp_at_ms >= CTL_TX_ABANDON_MS) { c.resp_busy = false; c.tx_abandoned++; }
}

static void rx_pump(uint32_t now, uint32_t since_last) {
    bool got = false, empty = true;
    while (!c.resp_busy && !c.outq_n) {
        if (!tud_cdc_n_available(CTL_ITF)) break;
        empty = false;
        uint8_t b;
        if (tud_cdc_n_read(CTL_ITF, &b, 1) != 1) break;
        got = true;
        if (b == '\n') {
            bool over = c.overflow;
            uint32_t n = c.len;
            c.len = 0;
            c.overflow = false;
            c.quiet_ms = 0;
            if (!over) handle_line(n);          // an overlong line was answered when it overflowed
            continue;
        }
        if (c.overflow) continue;
        if (c.len >= CTL_MAX_REQUEST - 1) {     // this byte would be the 1,024th, and it is not \n
            c.overflow = true;
            send_error(false, 0, CTL_OUT_NOT_EXECUTED, CTL_RETRY_SAFE, CTL_E_TOO_LARGE,
                       "request line over 1,024 bytes", 0, 0, true);
            continue;
        }
        c.line[c.len++] = (char)b;
    }
    if (c.len == 0 && !c.overflow) { c.quiet_ms = 0; return; }
    // A partial line: 2 s of the host's silence ends it (6.3, R-API-1
    // finding 22). Only passes that found the FIFO empty count, and a gap of
    // over 50 ms between passes (core 0 in a long USB callback) restarts it.
    if (got || c.resp_busy || !empty) { c.quiet_ms = 0; return; }
    if (since_last > CTL_GAP_MS) { c.quiet_ms = 0; return; }
    c.quiet_ms += since_last;
    (void)now;
    if (c.quiet_ms >= CTL_LINE_QUIET_MS) {
        bool over = c.overflow;
        c.len = 0;
        c.overflow = false;
        c.quiet_ms = 0;
        if (!over)
            send_error(false, 0, CTL_OUT_NOT_EXECUTED, CTL_RETRY_SAFE, CTL_E_PARSE,
                       "a partial line: no \\n within 2 s", 0, 0, true);
    }
}

// ---------------------------------------------------------------------------
//  Entry points
// ---------------------------------------------------------------------------

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)rts;
    if (itf == CTL_ITF && !dtr) dtr_dropped = true;
}

void ctl_on_usb_mount(void) { usb_mounted = true; }

void ctl_init(void) {
    pico_get_unique_board_id_string(c.serial, sizeof c.serial);
    uint64_t b = get_rand_64();
    static const char hx[] = "0123456789abcdef";
    for (int i = 0; i < 16; i++) c.boot[i] = hx[(b >> (60 - 4 * i)) & 15];
    c.boot[16] = 0;
    ctl_replay_init(&c.replay);
    ctl_rate_init(&c.rate);
    c.job_slot = -1;
    fingerprint(&c.fp);
    c.fp_epoch = ctl_epoch;
    c.prev_pass_ms = now_ms();
}

void ctl_task(void) {
    uint32_t now = now_ms();
#if ATABOY_BOARD_JUMPERS
    // The jumpers, every pass, whatever the port is doing (SPEC-v1 8.1,
    // 8.3), and before the epoch's cross-check below. A fit event is
    // reported by the one pass that ends it and is never kept (8.3): this
    // pass's pres_poll gives it to the waiting request (4.14), or nothing
    // uses it, then or later.
    bj_event_t ev;
    bool fit = bj_task(now, &ev);
    uint32_t edge_ms = fit ? ev.edge_ms : 0;
#else
    bool fit = false;
    uint32_t edge_ms = 0;
#endif
    uint32_t since_last = now - c.prev_pass_ms;
    c.prev_pass_ms = now;
    bool dtr = tud_cdc_n_connected(CTL_ITF);
    if (dtr_dropped || usb_mounted || (c.dtr && !dtr)) {
        // The lease ends with the port (6.1), before port_reset withdraws
        // what else was queued.
        lease_end(usb_mounted ? CTL_BY_USB_RESET : CTL_BY_PORT_CLOSED);
        dtr_dropped = false;
        usb_mounted = false;
        port_reset();
    }
    c.dtr = dtr;
    lease_poll(now);
    pres_poll(now, fit, edge_ms);
    epoch_poll();
    if (!dtr) {
        // Nothing is read or answered with DTR low (3.1): bytes are
        // discarded, and a DTR rise starts with an empty line.
        tud_cdc_n_read_flush(CTL_ITF);
        c.len = 0; c.overflow = false; c.quiet_ms = 0; c.resp_busy = false;
        outq_poll();                            // owed answers kept (replay table), not sent
        job_poll(now);                          // a job still ends, and is kept as job.last
        return;
    }
    tx_pump(now);
    outq_poll();
    tx_pump(now);
    job_poll(now);
    tx_pump(now);
    rx_pump(now, since_last);
    tx_pump(now);
}
