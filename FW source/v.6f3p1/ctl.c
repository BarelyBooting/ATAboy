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
#include "tusb.h"
#include "pico/stdlib.h"
#include "pico/rand.h"
#include "pico/unique_id.h"
#include "ide.h"
#include "obs.h"
#include "config.h"
#include "fault.h"
#include <string.h>

extern volatile bool is_mounted;
extern volatile bool cdc_connected;         // the TUI's port, interface 0 (ATAboy.c)
extern bool usb_msc_ide_busy(void);         // usb.c

#define CTL_ITF 1                           // CDC instance 1: interfaces 3 and 4 (usb_descriptors.c)
#define CTL_TX_ABANDON_MS   5000
#define CTL_LINE_QUIET_MS   2000            // a partial line with no \n (SPEC-v1 6.3)
#define CTL_GAP_MS          50              // longer between passes: a long USB callback, not silence
#define CTL_TUI_WAIT_MS     30000           // a job not picked up by then is withdrawn (5.3)
#define CTL_JOB_LAST_BYTES  1024

// What status and the epoch are computed from (SPEC-v1 4.3). epoch counts
// changes of it, seen by comparing this snapshot at every pass of ctl_task().
typedef struct {
    uint8_t  mounted, lba, wp, pipelined, dev_base, no_identify, native_active, native_declared;
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
    uint32_t seq, epoch;
    ctl_fp_t fp;
    uint8_t  wp_last;               // 0 BOOT, 1 TUI (stage 1: nothing else can change it)
    uint32_t tx_abandoned, parse_errors;
    ctl_replay_t replay;
    ctl_rate_t   rate;
    ctlj_doc_t   doc;
    // The job in the mailbox, as core 0 knows it
    uint32_t job_port_gen;
    int      job_slot;              // its replay slot when it is mutating (stage 2 on)
    // job.last (SPEC-v1 4.11)
    bool     jl_present;
    uint8_t  jl_op;
    uint32_t jl_id, jl_started, jl_ended, jl_begin, jl_end;
    uint8_t  jl_digest[32];
    uint32_t jl_len;
    char     jl_resp[CTL_JOB_LAST_BYTES];
    // status.mount_seq, from core 1 (ctl_mount_noted)
    volatile uint32_t mount_seq, mount_log_seq;
} c;

static volatile bool dtr_dropped, usb_mounted;      // set by the two callbacks

static uint8_t page_buf[OBS_PAGE_HEADER + CTL_HASH_PER_READ * sizeof(obs_hash_entry_t)];
_Static_assert(OBS_PAGE_HEADER + CTL_LOG_PER_READ * sizeof(obs_log_entry_t) <= sizeof page_buf,
               "a log.read page fits");

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

#if ATABOY_HOST_TEST
// Host tests: stand-in handlers that make an op of a later stage count as
// built, so the replay path (for mutating requests only) can be driven from
// the envelope before stage 2 exists. Never compiled into the firmware.
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
    f->cyls = config.cyls;
    f->heads = config.heads;
    f->spt = config.spt;
    f->lba_sectors = config.lba_sectors;
}

// Stage 1's epoch: the snapshot above compared at every pass. A change and
// its reversal inside one pass (one long USB callback) is not counted.
static void epoch_poll(void) {
    ctl_fp_t f;
    fingerprint(&f);
    if (memcmp(&f, &c.fp, sizeof f) != 0) {
        if (f.wp != c.fp.wp) c.wp_last = 1;     // only the TUI (or its Load Defaults) writes it now
        c.fp = f;
        c.epoch++;
    }
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
    ctlw_uint(w, "epoch", c.epoch);
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

static uint32_t send_error(bool id_ok, uint32_t id, uint8_t outcome, uint8_t retry, uint8_t err,
                           const char *msg, const char *why, const char *field, bool send) {
    ctlw_t w;
    begin(&w, id_ok, id);
    ctlw_bool(&w, "ok", false);
    ctlw_str(&w, "outcome", ctl_outcome_name[outcome]);
    ctlw_str(&w, "retry", ctl_retry_name[retry]);
    ctlw_obj(&w, "err");
    ctlw_str(&w, "code", ctl_err_name[err]);
    ctlw_str(&w, "msg", msg);
    if (why) ctlw_str(&w, "why", why);
    if (field) ctlw_str(&w, "field", field);
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
    ctlw_bool(w, "board_jumpers", false);       // stage 3
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

static void write_status(ctlw_t *w) {
    ctlw_bool(w, "mounted", is_mounted);
    uint32_t ms = c.mount_seq;
    ctlw_uint(w, "mount_seq", ms);
    if (ms) ctlw_uint(w, "mount_log_seq", c.mount_log_seq); else ctlw_null(w, "mount_log_seq");
    ctlw_obj(w, "geometry");
    ctlw_str(w, "mode", config.use_lba_mode ? "LBA" : "CHS");
    ctlw_uint(w, "cyls", config.cyls);
    ctlw_uint(w, "heads", config.heads);
    ctlw_uint(w, "spt", config.spt);
    ctlw_uint(w, "lba_sectors", config.lba_sectors);
    ctlw_uint(w, "total_sectors", config.use_lba_mode ? config.lba_sectors
                                  : (uint64_t)config.cyls * config.heads * config.spt);
    ctlw_end_obj(w);
    // Which path set the geometry up is a new RAM field every set-up path
    // must write (4.3). Stage 1 does not track it yet, and says so.
    ctlw_obj(w, "setup");
    ctlw_str(w, "source", "UNKNOWN");
    ctlw_str(w, "at_ms", "UNKNOWN");
    ctlw_end_obj(w);
    ctlw_str(w, "device", config.dev_base == 0xB0 ? "SLAVE" : "MASTER");
    bool noid = ide_mchs_no_identify();
    ctlw_obj(w, "wp");
    ctlw_bool(w, "effective", ide_write_protected());
    ctlw_bool(w, "config", config.drive_write_protected);
    ctlw_arr(w, "forced_by");
    if (noid) ctlw_str(w, 0, "CTRLG");          // review C, C-4 (ide.c)
    ctlw_end_arr(w);
    ctlw_str(w, "last_change", c.wp_last ? "TUI" : "BOOT");
    ctlw_end_obj(w);
    ctlw_obj(w, "rules");
    ctlw_bool(w, "no_identify_ctrlg", noid);
    ctlw_bool(w, "identify_forbidden", false);  // stage 2 (identify.forbid)
    ctlw_bool(w, "identify_barred", noid);
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
    ctlw_obj(w, "jumpers");                     // stage 3: no jumper support in this build
    ctlw_str(w, "jp1", "NOT_BUILT");
    ctlw_str(w, "jp2", "NOT_BUILT");
    ctlw_uint(w, "mount_credit_ms_left", 0);
    ctlw_end_obj(w);
    ctlw_obj(w, "presence");                    // stage 4: no Tier M, nothing ever waits
    ctlw_bool(w, "waiting", false);
    ctlw_str(w, "op", "NONE");
    ctlw_null(w, "id");
    ctlw_null(w, "request_sha256");
    ctlw_uint(w, "ms_left", 0);
    ctlw_end_obj(w);
    ctlw_obj(w, "lease");                       // stage 4
    ctlw_bool(w, "held", false);
    ctlw_null(w, "client");
    ctlw_uint(w, "idle_ms_left", 0);
    ctlw_bool(w, "tui_lock", false);
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
    ctlw_uint(w, "epoch", c.epoch);
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
        uint32_t ms_before = c.mount_seq;
        *w = start;
        write_status(w);
        ctl_fp_t after;
        fingerprint(&after);
        if (memcmp(&before, &after, sizeof after) == 0 && ms_before == c.mount_seq) return CTL_E_NONE;
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
    ctlw_str(w, "jumpers", "NOT_BUILT");
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
    int e = fn ? fn(&w, d, args) : CTL_E_UNKNOWN_OP;
    if (e != CTL_E_NONE)
        return send_error(true, id, CTL_OUT_NOT_EXECUTED, CTL_RETRY_SAFE, (uint8_t)e,
                          e == CTL_E_BUSY_STATE ? "the state kept changing while it was read" : "not available",
                          0, 0, true);
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

static bool queue_job(int op, uint32_t id, const uint8_t digest[32]) {
    // Arrival checks for taskfile.snapshot (4.6); every refusal NOT_EXECUTED.
    if (is_mounted) { refuse(true, id, CTL_E_STATE, "mounted: the bus is the host's", 0); return false; }
    if (usb_msc_ide_busy() || ide_recovery_pending()) {
        refuse(true, id, CTL_E_BUS_BUSY, "a USB command or its reset is still running", 0);
        return false;
    }
    if (ctl_job.state != CTL_JOB_EMPTY) { refuse(true, id, CTL_E_INFLIGHT, "another job is in flight", 0); return false; }
    ctl_job.op = (uint8_t)op;
    ctl_job.id = id;
    memcpy(ctl_job.digest, digest, 32);
    ctl_job.queued_ms = now_ms();
    c.job_port_gen = c.port_gen;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);      // the request before the state
    ctl_job.state = CTL_JOB_QUEUED;
    // No answer now: it comes when the job ends or is withdrawn.
    return true;
}

static void answer_job(void) {
    ctl_job_t *j = &ctl_job;
    bool send = c.job_port_gen == c.port_gen && c.dtr;
    ctlw_t w;
    uint32_t n;
    if (j->outcome == CTL_OUT_DONE) {
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
        uint8_t retry = j->outcome == CTL_OUT_NOT_EXECUTED ? CTL_RETRY_SAFE : CTL_RETRY_AFTER_STATUS;
        n = send_error(true, j->id, j->outcome, retry, j->err, j->msg, 0, 0, send);
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
            refuse(true, r.id, CTL_E_STALE_ID, "id not above max_mutating_id", "id");
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
    if (ctl_ops[r.op].job) {
        if (queue_job(r.op, r.id, digest)) { c.job_slot = slot; return; }
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
    while (!c.resp_busy) {
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

void ctl_mount_noted(uint32_t marker_log_seq) {
    c.mount_log_seq = marker_log_seq;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    c.mount_seq = c.mount_seq + 1;              // only core 1 writes it
}

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
    c.prev_pass_ms = now_ms();
}

void ctl_task(void) {
    uint32_t now = now_ms();
    uint32_t since_last = now - c.prev_pass_ms;
    c.prev_pass_ms = now;
    bool dtr = tud_cdc_n_connected(CTL_ITF);
    if (dtr_dropped || usb_mounted || (c.dtr && !dtr)) {
        dtr_dropped = false;
        usb_mounted = false;
        port_reset();
    }
    c.dtr = dtr;
    epoch_poll();
    if (!dtr) {
        // Nothing is read or answered with DTR low (3.1): bytes are
        // discarded, and a DTR rise starts with an empty line.
        tud_cdc_n_read_flush(CTL_ITF);
        c.len = 0; c.overflow = false; c.quiet_ms = 0; c.resp_busy = false;
        job_poll(now);                          // a job still ends, and is kept as job.last
        return;
    }
    tx_pump(now);
    job_poll(now);
    tx_pump(now);
    rx_pump(now, since_last);
    tx_pump(now);
}
