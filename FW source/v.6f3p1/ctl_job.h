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

enum { CTL_JOB_EMPTY, CTL_JOB_QUEUED, CTL_JOB_RUNNING, CTL_JOB_ENDED };

typedef struct {
    volatile uint32_t state;        // CTL_JOB_*
    // Core 0 fills these before QUEUED.
    uint8_t  op;                    // CTL_OP_*
    uint32_t id;
    uint8_t  digest[32];            // of its request line (job.last)
    uint32_t queued_ms;
    // Core 1 fills these before ENDED.
    uint8_t  outcome;               // CTL_OUT_*
    uint8_t  err;                   // CTL_E_* when not DONE
    const char *msg;
    bool     ran;                   // it got past its pickup checks (API markers written)
    uint32_t started_ms, ended_ms;
    uint32_t log_begin, log_end;    // its API_BEGIN and API_END entries (SPEC-v1 5.4)
    // taskfile.snapshot's result
    uint8_t  tf[7];                 // registers 1 to 6 (tf[0] unused)
    uint8_t  alt;                   // Alternate Status
    uint8_t  regs_read;             // how many of the 7 reads were made
    uint64_t at_us;
} ctl_job_t;

extern ctl_job_t ctl_job;
extern volatile uint8_t ctl_tui_state;      // CTL_TUI_*, published by core 1

bool ctl_job_cas(uint32_t from, uint32_t to);

#endif
