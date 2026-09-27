// ATAboy Control: core 1's side (SPEC-v1 5.3). The TUI's idle point and the
// job bodies.
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
// after it.

#include "ctl.h"
#include "ctl_job.h"
#include "ctl_policy.h"
#include "ide.h"
#include "obs.h"
#include "pico/stdlib.h"

extern volatile bool is_mounted;
extern bool usb_msc_ide_busy(void);     // usb.c

ctl_job_t ctl_job;
volatile uint8_t ctl_tui_state = CTL_TUI_BOOT;

bool ctl_job_cas(uint32_t from, uint32_t to) {
    uint32_t f = from;
    return __atomic_compare_exchange_n(&ctl_job.state, &f, to, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
}

static uint32_t job_now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

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
    j->started_ms = job_now_ms();
    j->log_begin = j->log_end = 0;
    if (j->op == CTL_OP_TASKFILE_SNAPSHOT) run_taskfile_snapshot(j);
    else job_refuse(CTL_E_UNKNOWN_OP, "no such job");
    j->ended_ms = job_now_ms();
    __atomic_thread_fence(__ATOMIC_SEQ_CST);      // the result before the state
    j->state = CTL_JOB_ENDED;
}

void ctl_tui_busy(void) { ctl_tui_state = CTL_TUI_MODAL; }
