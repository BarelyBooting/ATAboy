// ATAboy Control: core 1's side (SPEC-v1 5.3). The TUI's idle point and the
// job bodies, and the state the rest of the firmware reports as it changes
// (ctl_state.h: the epoch, write protect's last change, the setup source,
// mount_seq).
//
// menus.c calls ctl_core1_idle() at the top of its loop, after the stack
// canary and tripwire checks and before the "no terminal" skip, so a job
// runs whether or not anyone has the TUI open (procedure P-3 keeps it
// closed). A job is taken only when the TUI is on one of its plain screens
// with no box open (MAIN, FEATURES, DEBUG, MOUNTED): nested loops (the
// picker, manual entry, the Ctrl+G box, the seek test, confirm waits) never
// come back to the top while they run, so no job can start inside one.
//
// A job body never touches the TUI: no cdc_putchar/cdc_puts/cdc_printf, no
// draw_*, no get_input(), no bus_free_wait() or mchs_pending_refused() (they
// draw and wait for a key). With no terminal on interface 0 the TUI's output
// queue is not drained, so a job that drew would stop there and resume, maybe
// hours later, when someone opened it (R-API-1 finding 1). Before each bus
// step a job checks its deadline, pickup plus its max time, and sends nothing
// after it. unmount and detach send nothing at all: they wait only for a USB
// read to end, as the TUI's own unmount does.

#include "ctl.h"
#include "ctl_job.h"
#include "ctl_policy.h"
#include "ctl_state.h"
#include "ide.h"
#include "obs.h"
#include "config.h"
#include "manualchs.h"
#include "pico/stdlib.h"
#if ATABOY_BOARD_JUMPERS
#include "board_jumpers.h"
#endif

extern volatile bool is_mounted;
extern bool usb_msc_ide_busy(void);     // usb.c

ctl_job_t ctl_job;
volatile uint8_t ctl_tui_state = CTL_TUI_BOOT;

// ctl_state.h
volatile uint32_t ctl_epoch;
volatile uint8_t  ctl_wp_last = CTL_WP_BOOT;
volatile uint8_t  ctl_setup_src = CTL_SRC_NONE;
volatile uint32_t ctl_setup_at_ms;
volatile uint32_t ctl_mount_seq, ctl_mount_log_seq;
volatile uint32_t ctl_lease_gen;
volatile uint8_t  ctl_lease_by;

// detach's wait for a USB read started before it (SPEC-v1 4.12, R-API-1
// finding 8): the TUI's own unmount waits IDE_BUS_WAIT_MS (menus.c), the
// longest a host command may run plus 5 s, counted here from pickup so the
// whole job stays inside its 30 s.
#define CTL_DETACH_WAIT_MS (IDE_HOST_BUDGET_MS + 5000u)

bool ctl_job_cas(uint32_t from, uint32_t to) {
    uint32_t f = from;
    return __atomic_compare_exchange_n(&ctl_job.state, &f, to, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
}

static uint32_t job_now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

void ctl_setup_noted(uint8_t source) {
    if (source == CTL_SRC_NONE && ctl_setup_src == CTL_SRC_NONE) return;
    ctl_setup_at_ms = job_now_ms();
    ctl_setup_src = source;
    CTL_EPOCH_BUMP();
}

// Every mount path's MOUNT marker (menus.c log_mount): status.mount_seq
// counts mounts since boot (SPEC-v1 4.3, I-28). Only core 1 writes it.
void ctl_mount_noted(uint32_t marker_log_seq) {
    ctl_mount_log_seq = marker_log_seq;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    ctl_mount_seq = ctl_mount_seq + 1;
}

static void job_refuse(uint8_t err, const char *msg) {
    ctl_job.outcome = CTL_OUT_NOT_EXECUTED;
    ctl_job.err = err;
    ctl_job.msg = msg;
}

// taskfile.snapshot (SPEC-v1 4.6): registers 1 to 6 and Alternate Status,
// never register 7, whose read would clear a pending INTRQ. No command.
static void run_taskfile_snapshot(ctl_job_t *j) {
    // At pickup, as at arrival (6.2): the bus is the host's while mounted, and
    // a USB command still running or a reset it left pending is refused, not
    // waited for. is_mounted first, then the busy flag (usb.c's fence order).
    if (is_mounted) { job_refuse(CTL_E_STATE, "mounted: the bus is the host's"); return; }
    if (usb_msc_ide_busy() || ide_recovery_pending()) { job_refuse(CTL_E_BUS_BUSY, "a USB command or its reset is still running"); return; }
    uint32_t deadline = j->started_ms + ctl_ops[j->op].max_ms;
    j->ran = true;
    j->log_begin = obs_api(j->op, j->id, OBS_API_BEGIN);
    j->at_us = to_us_since_boot(get_absolute_time());
    for (uint8_t reg = 1; reg <= 6; reg++) {
        if (job_now_ms() > deadline) break;
        j->tf[reg] = ide_read_reg(reg);
        j->regs_read++;
    }
    if (j->regs_read == 6 && job_now_ms() <= deadline) {
        j->alt = ide_read_alt_status();
        j->regs_read++;
    }
    if (j->regs_read == 7) {
        j->outcome = CTL_OUT_DONE;
    } else {
        // The bus was held past the deadline (IORDY, say): nothing more read.
        j->outcome = CTL_OUT_UNKNOWN;
        j->err = CTL_E_DRIVE;
        j->msg = "deadline passed before every register was read";
    }
    j->log_end = obs_api(j->op, j->id, j->outcome == CTL_OUT_DONE ? OBS_API_DONE : OBS_API_UNKNOWN);
}

// unmount and detach name the mount they end (owner decision on R-API-1
// finding 11): compared on core 0 when the request came (ctl.c) and again
// here, at pickup, where a mount made meanwhile (the human's confirm-3,
// auto-mount; both on this core, so none can happen during this check) is
// caught. The epoch too, when the request gave one. A mismatch changes
// nothing: no API entry, no epoch, no state (I-27).
static bool names_current(ctl_job_t *j) {
    if (j->expect_mount_seq != ctl_mount_seq) {
        job_refuse(CTL_E_STALE, "expect_mount_seq is not the current mount");
        j->field = "expect_mount_seq";
        return false;
    }
    if (j->has_epoch && j->expect_epoch != ctl_epoch) {
        job_refuse(CTL_E_STALE, "the state changed since expect_epoch");
        j->field = "expect_epoch";
        return false;
    }
    return true;
}

// Past its pickup checks: it runs, between its API_BEGIN entry and its
// API_END (SPEC-v1 5.4), and is kept as job.last.
static void job_began(ctl_job_t *j) { j->ran = true; j->log_begin = obs_api(j->op, j->id, OBS_API_BEGIN); }

// unmount (SPEC-v1 4.12): the TUI's confirm-4 path, exactly, then the main
// menu. Not mounted: DONE, nothing changed.
static void __attribute__((noinline)) run_unmount(ctl_job_t *j) {
    if (!names_current(j)) return;
    job_began(j);
    j->changed = is_mounted;
    if (j->changed) j->native_ended = menus_api_unmount();
    j->outcome = CTL_OUT_DONE;
    j->log_end = obs_api(j->op, j->id, OBS_API_DONE);
}

// detach (SPEC-v1 4.12, R-API-1 finding 8): unmount if mounted, then wait,
// in every mode, for a USB read started before it; only once none is
// running clear the geometry and what else would let the drive be mounted
// again (menus_api_detach_clear). A read still running keeps working from
// config (re-review N-1), so then nothing is cleared: FAILED.
static void __attribute__((noinline)) run_detach(ctl_job_t *j) {
    if (!names_current(j)) return;
    job_began(j);
    if (is_mounted) (void)menus_api_unmount();
    __atomic_thread_fence(__ATOMIC_SEQ_CST);      // is_mounted false before the busy flag (usb.c)
    uint32_t until = j->started_ms + CTL_DETACH_WAIT_MS;
    while (usb_msc_ide_busy() && job_now_ms() < until) sleep_ms(10);
    if (usb_msc_ide_busy()) {
        j->outcome = CTL_OUT_FAILED;
        j->err = CTL_E_BUS_BUSY;
        j->msg = "a USB read is still running: unmounted, nothing cleared";
        j->log_end = obs_api(j->op, j->id, OBS_API_FAILED);
        return;
    }
    menus_api_detach_clear();
    j->outcome = CTL_OUT_DONE;
    j->log_end = obs_api(j->op, j->id, OBS_API_DONE);
}

// ---------------------------------------------------------------------------
//  Tier M (stage 4; SPEC-v1 4.14)
// ---------------------------------------------------------------------------

int ctl_mreq_check(uint8_t op, const ctl_mreq_t *m, const char **field, const char **why) {
    *field = 0;
    *why = 0;
    if (is_mounted) return CTL_E_STATE;                     // the bus is the host's
    if (op == CTL_OP_SETUP_CTRLG) {
        // What Ctrl+G's dialog waits for or refuses (menus.c bus_free_wait,
        // mchs_pending_refused), refused here, never waited for.
        if (usb_msc_ide_busy() || ide_recovery_pending()) return CTL_E_BUS_BUSY;
        if (m->dev_base != config.dev_base) { *field = "device"; *why = "NOT_THE_DEVICE"; return CTL_E_PRECONDITION; }
        int bad = manual_chs_bad_field(m->cyls, m->heads, m->spt);
        if (bad >= 0) { *field = bad == 0 ? "cyls" : bad == 1 ? "heads" : "spt"; return CTL_E_RANGE; }
        // The power-up default mode, never after a 91h this boot (the
        // dialog's rule, menus.c; ide.c's too).
        if (m->native && ide_idp_sent_since_boot()) { *field = "mode"; *why = "IDP_SENT_SINCE_BOOT"; return CTL_E_PRECONDITION; }
        return CTL_E_NONE;
    }
    // mount
    uint8_t src = ctl_setup_src;
    if (src == CTL_SRC_NONE) { *why = "NO_SETUP"; return CTL_E_PRECONDITION; }     // settings-only geometry (X21)
    bool geo = config.use_lba_mode ? config.lba_sectors > 0 : (config.cyls > 0 && config.heads > 0 && config.spt > 0);
    if (!geo) { *why = "NO_GEOMETRY"; return CTL_E_PRECONDITION; }
    if ((src == CTL_SRC_TUI_CTRLG_NATIVE || src == CTL_SRC_API_CTRLG_NATIVE) && !ide_manual_chs_native_active()) {
        *why = "MCHS_NEEDED"; return CTL_E_PRECONDITION;                            // the TUI's MCHS_NEEDED
    }
    // The expect, compared and set (4.14): any difference is stale.
    *field = "expect";
    if (m->src != src || m->lba != config.use_lba_mode || m->dev_base != config.dev_base) return CTL_E_STALE;
    if (m->lba ? m->lba_sectors != config.lba_sectors
               : (m->cyls != config.cyls || m->heads != config.heads || m->spt != config.spt)) return CTL_E_STALE;
    *field = 0;
    return CTL_E_NONE;
}

// At pickup (5.3, 4.14): the lease it came under still held, JP1 open and
// readable, the credit a mount was queued on still live, its expect_epoch,
// and every precondition again. A refusal here spends the act: the request
// ends NOT_EXECUTED, nothing of it ran.
static bool tier_m_pickup(ctl_job_t *j) {
    if (j->lease_gen == 0 || j->lease_gen != ctl_lease_gen) {
        job_refuse(CTL_E_LEASE_REVOKED, "the lease ended before pickup");
        j->by = ctl_lease_by;
        return false;
    }
#if ATABOY_BOARD_JUMPERS
    if (!bj_tier_m_open()) {
        job_refuse(CTL_E_LOCKED, "JP1 fitted, or the jumpers unreadable");
        j->why = bj_jp1() == BJ_FITTED ? "JP1_FITTED" : "JUMPERS_UNREADABLE";
        return false;
    }
#else
    job_refuse(CTL_E_LOCKED, "no jumpers in this build");
    j->why = "JUMPERS_UNSUPPORTED";
    return false;
#endif
    if (j->m.has_epoch && j->m.expect_epoch != ctl_epoch) {
        job_refuse(CTL_E_STALE, "the state changed since expect_epoch");
        j->field = "expect_epoch";
        return false;
    }
    if (j->via_credit && (job_now_ms() - j->credit_end_ms > CTL_MOUNT_CREDIT_S * 1000u || ctl_epoch != j->credit_epoch)) {
        job_refuse(CTL_E_PRESENCE_REQUIRED, "the mount credit lapsed before pickup");
        return false;
    }
    const char *field, *why;
    int e = ctl_mreq_check(j->op, &j->m, &field, &why);
    if (e != CTL_E_NONE) {
        job_refuse((uint8_t)e, "a precondition failed at pickup");
        j->field = field;
        j->why = why;
        return false;
    }
    return true;
}

static const char *const mchs_why[] = {
    "", "NO_DEVICE", "BUSY", "RECALIBRATE", "REFUSED", "BAD_ARGS", "NOT_READY", "IDP_SENT_SINCE_BOOT",
};

// setup.ctrlg: Ctrl+G after its Y (menus_api_ctrlg), within its 75 s: every
// wait inside it is ide.c's own and bounded (45 s ready, 10 s RECALIBRATE,
// 1 s 91h). Past the 75 s it reports UNKNOWN (6.3: a firmware defect).
static void __attribute__((noinline)) run_setup_ctrlg(ctl_job_t *j) {
    if (!tier_m_pickup(j)) return;
    job_began(j);
    uint32_t deadline = j->started_ms + ctl_ops[j->op].max_ms;
    uint8_t st = 0;
    int r = menus_api_ctrlg(j->m.cyls, j->m.heads, j->m.spt, j->m.native, &st);
    j->mchs_rc = (int8_t)r;
    j->status = st;
    uint8_t mark;
    if (job_now_ms() > deadline) {
        j->outcome = CTL_OUT_UNKNOWN; j->err = CTL_E_DRIVE; j->msg = "past its max time"; mark = OBS_API_UNKNOWN;
    } else if (r == IDE_MCHS_OK) {
        j->outcome = CTL_OUT_DONE; mark = OBS_API_DONE;
    } else if (r == IDE_MCHS_BAD_ARGS || r == IDE_MCHS_IDP_SENT) {
        // Nothing was sent (4.14): NOT_EXECUTED.
        j->outcome = CTL_OUT_NOT_EXECUTED; j->err = CTL_E_PRECONDITION; j->msg = "refused by ide.c, nothing sent";
        j->why = mchs_why[r]; mark = OBS_API_FAILED;
    } else {
        j->outcome = CTL_OUT_FAILED; j->err = CTL_E_DRIVE; j->msg = "the drive did not take the geometry";
        j->why = r > 0 && r < (int)(sizeof mchs_why / sizeof mchs_why[0]) ? mchs_why[r] : "UNKNOWN";
        mark = OBS_API_FAILED;
    }
    j->log_end = obs_api(j->op, j->id, mark);
    j->end_epoch = ctl_epoch;
}

// mount: TUI confirm-3 (menus_api_mount), no bus.
static void __attribute__((noinline)) run_mount(ctl_job_t *j) {
    if (!tier_m_pickup(j)) return;
    job_began(j);
    menus_api_mount();
    j->mount_seq = ctl_mount_seq;
    j->mount_log_seq = ctl_mount_log_seq;
    j->total_sectors = config.use_lba_mode ? config.lba_sectors
                                           : (uint64_t)config.cyls * config.heads * config.spt;
    j->outcome = CTL_OUT_DONE;
    j->log_end = obs_api(j->op, j->id, OBS_API_DONE);
    j->end_epoch = ctl_epoch;
}

static bool idle_screen(uint8_t s) {
    return s == CTL_TUI_MAIN || s == CTL_TUI_FEATURES || s == CTL_TUI_DEBUG || s == CTL_TUI_MOUNTED;
}

void ctl_core1_idle(uint8_t tui_state) {
    ctl_tui_state = tui_state;
    if (!idle_screen(tui_state) || ctl_job.state != CTL_JOB_QUEUED) return;
    if (!ctl_job_cas(CTL_JOB_QUEUED, CTL_JOB_RUNNING)) return;     // core 0 withdrew it first
    ctl_job_t *j = &ctl_job;
    j->ran = false;
    j->regs_read = 0;
    j->err = CTL_E_NONE;
    j->msg = "";
    j->field = 0;
    j->why = 0;
    j->by = CTL_BY_NONE;
    j->mchs_rc = -1;
    j->status = 0;
    j->changed = false;
    j->native_ended = false;
    j->started_ms = job_now_ms();
    j->log_begin = j->log_end = 0;
    if (j->op == CTL_OP_TASKFILE_SNAPSHOT) run_taskfile_snapshot(j);
    else if (j->op == CTL_OP_UNMOUNT) run_unmount(j);
    else if (j->op == CTL_OP_DETACH) run_detach(j);
    else if (j->op == CTL_OP_SETUP_CTRLG) run_setup_ctrlg(j);
    else if (j->op == CTL_OP_MOUNT) run_mount(j);
    else job_refuse(CTL_E_UNKNOWN_OP, "no such job");
    j->ended_ms = job_now_ms();
    __atomic_thread_fence(__ATOMIC_SEQ_CST);      // the result before the state
    j->state = CTL_JOB_ENDED;
}

void ctl_tui_busy(void) { ctl_tui_state = CTL_TUI_MODAL; }
