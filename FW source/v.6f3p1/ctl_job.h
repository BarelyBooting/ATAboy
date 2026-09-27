// ATAboy Control: the job mailbox between core 0 (ctl.c) and core 1
// (ctl_job.c). Internal to the two.
//
// One slot of plain shared RAM with fences, the usb.c busy-flag pattern. It
// does not use the SIO inter-core FIFO or its interrupt: multicore_lockout
// owns them for config_save() (SPEC-v1 5.1, R-API-1 finding 18).
//
//   EMPTY --core 0 queues--> QUEUED --core 1 picks up--> RUNNING --> ENDED
//                              |                                       |
//                              +--core 0 withdraws--> EMPTY   core 0 answers, EMPTY
//
// Pickup and withdrawal are one compare-and-swap each on `state`, so a job
// is either withdrawn or run, never both (5.3).
#ifndef CTL_JOB_H
#define CTL_JOB_H

#include <stdint.h>
#include <stdbool.h>
#include "ctl_policy.h"

enum { CTL_JOB_EMPTY, CTL_JOB_QUEUED, CTL_JOB_RUNNING, CTL_JOB_ENDED };

// How a lease ended (SPEC-v1 6.1): err.by of E_LEASE_REVOKED.
enum { CTL_BY_NONE, CTL_BY_CLOSED, CTL_BY_TUI, CTL_BY_USB_RESET, CTL_BY_PORT_CLOSED, CTL_BY_IDLE };

typedef struct {
    volatile uint32_t state;        // CTL_JOB_*
    // Core 0 fills these before QUEUED.
    uint8_t  op;                    // CTL_OP_*
    uint32_t id;
    uint8_t  digest[32];            // of its request line (job.last)
    uint32_t queued_ms;
    // unmount and detach (SPEC-v1 4.12): the mount they name, compared again
    // at pickup, and the epoch if the request gave one.
    uint32_t expect_mount_seq;
    bool     has_epoch;
    uint32_t expect_epoch;
    // Core 1 fills these before ENDED.
    uint8_t  outcome;               // CTL_OUT_*
    uint8_t  err;                   // CTL_E_* when not DONE
    const char *msg;
    const char *field;              // err.field, when a refusal names one
    const char *why;                // err.why, when a refusal gives one
    uint8_t  by;                    // err.by (CTL_BY_*), E_LEASE_REVOKED at pickup
    bool     ran;                   // it got past its pickup checks (API markers written)
    uint32_t started_ms, ended_ms;
    uint32_t log_begin, log_end;    // its API_BEGIN and API_END entries (SPEC-v1 5.4)
    // taskfile.snapshot's result
    uint8_t  tf[7];                 // registers 1 to 6 (tf[0] unused)
    uint8_t  alt;                   // Alternate Status
    uint8_t  regs_read;             // how many of the 7 reads were made
    uint64_t at_us;
    // unmount's and detach's results
    bool     changed;               // unmount: it was mounted
    bool     native_ended;          // unmount: Ctrl+G's power-up default mode ended
    // Tier M (stage 4; SPEC-v1 4.14). Core 0 fills these before QUEUED.
    uint32_t lease_gen;             // the lease it came under; 0 for every other job
    ctl_mreq_t m;                   // its arguments (ctl_policy.h)
    bool     via_credit;            // a mount queued on a declared pair's credit (Q3)
    uint32_t credit_end_ms;         // ...the set-up's end, from which the credit's 60 s run
    uint32_t credit_epoch;          // ...and the epoch it holds for
    // Core 1 fills these before ENDED.
    int8_t   mchs_rc;               // setup.ctrlg: IDE_MCHS_*, -1 when nothing was called
    uint8_t  status;                // setup.ctrlg: the drive's last status byte
    uint32_t end_epoch;             // ctl_epoch right after API_END (a credit's epoch)
    uint32_t mount_seq, mount_log_seq;
    uint64_t total_sectors;
} ctl_job_t;

extern ctl_job_t ctl_job;
extern volatile uint8_t ctl_tui_state;      // CTL_TUI_*, published by core 1

bool ctl_job_cas(uint32_t from, uint32_t to);

// status.mount_seq and mount_log_seq (SPEC-v1 4.3): written by core 1 only
// (ctl_mount_noted, from every mount path), read by core 0.
extern volatile uint32_t ctl_mount_seq, ctl_mount_log_seq;
// The live lease (SPEC-v1 6.1), for core 1's pickup check: its number, 0
// when none is held, and how the last one ended (CTL_BY_*). Core 0 writes.
extern volatile uint32_t ctl_lease_gen;
extern volatile uint8_t  ctl_lease_by;

// Tier M's preconditions (SPEC-v1 4.14) that the firmware's state decides,
// read at arrival (core 0) and again at pickup (core 1): CTL_E_NONE, or the
// refusal with its err.field and err.why.
int ctl_mreq_check(uint8_t op, const ctl_mreq_t *m, const char **field, const char **why);

// status.setup (ctl_state.h): written by core 1 (and core 0's ratchet never).
extern volatile uint8_t  ctl_setup_src;
extern volatile uint32_t ctl_setup_at_ms;

#endif
