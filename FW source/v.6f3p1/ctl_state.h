// ATAboy Control, protocol v1 (palimpsest api/SPEC-v1.md): the state the
// rest of the firmware tells the API about, at the moment it changes.
//
// status.epoch (SPEC-v1 4.3) counts changes of: is_mounted, the geometry and
// its setup source, write protect, Pipelined Reads, the device, the
// no-IDENTIFY rules (Ctrl+G's and identify.forbid), the native mode, and
// from stage 3 JP1's state (board_jumpers.c; JP2's level is not counted,
// SPEC-v1 4.3), and (stage 4) the lease and the TUI lock. Stage 2 counts
// them where they happen: every place that stores one of them bumps the
// counter right after the store (CTL_EPOCH_BUMP), on whichever core it runs.
// Stage 1 compared a snapshot at each pass of ctl_task() instead, which
// missed a change and its reversal inside one long USB callback; that
// comparison is kept only as a cross-check (ctl.c, epoch_poll).
//
// Why after the store, not before: a request's expect_epoch is compared with
// the counter. Bumped after, a status read in between shows the new state
// with the old epoch and the next bump makes that epoch stale (a spurious
// E_STALE at worst). Bumped before, a status read in between could show the
// old state with the new epoch, and a request naming it would pass.
//
// With ATABOY_CTL off every macro here is empty, so the image is byte for
// byte the one built without the API (SPEC-v1 2.1).
#ifndef CTL_STATE_H
#define CTL_STATE_H

#include <stdint.h>
#include <stdbool.h>

#if ATABOY_CTL
extern volatile uint32_t ctl_epoch;                 // ctl_job.c
#define CTL_EPOCH_BUMP() ((void)__atomic_fetch_add(&ctl_epoch, 1u, __ATOMIC_SEQ_CST))

// status.wp.last_change: what changed write protect last.
enum { CTL_WP_BOOT, CTL_WP_TUI, CTL_WP_API };
extern volatile uint8_t ctl_wp_last;                // ctl_job.c

// status.setup.source (SPEC-v1 4.3): which path set the geometry up. NONE
// at power-up (a geometry loaded from the settings is not a set-up: SPEC-v1
// X21) and whenever a path clears it.
enum {
    CTL_SRC_NONE, CTL_SRC_TUI_DETECT_NORMAL, CTL_SRC_TUI_DETECT_LARGE, CTL_SRC_TUI_DETECT_LBA,
    CTL_SRC_TUI_MANUAL, CTL_SRC_TUI_FORCE, CTL_SRC_TUI_CTRLG_IDP, CTL_SRC_TUI_CTRLG_NATIVE,
    CTL_SRC_API_CTRLG_IDP, CTL_SRC_API_CTRLG_NATIVE, CTL_SRC_AUTO_MOUNT,
    CTL_SRC_COUNT
};
// A set-up path ran (or cleared the geometry): the source, the time, and
// an epoch bump. Clearing what is already clear changes nothing.
void ctl_setup_noted(uint8_t source);                // ctl_job.c
#else
#define CTL_EPOCH_BUMP() ((void)0)
#endif

#endif
