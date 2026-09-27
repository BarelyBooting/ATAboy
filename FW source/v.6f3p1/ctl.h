// ATAboy Control, protocol v1 (palimpsest api/SPEC-v1.md): the second CDC
// function, interfaces 3 and 4, speaking one JSON object per line. Built
// only with ATABOY_CTL=1 (CMakeLists.txt).
//
// Core 0 runs ctl_task() between tud_task() passes: it reads the Control
// port (CDC instance 1), answers the core-0 ops at once, and hands a job to
// core 1 through a one-slot mailbox (ctl_job.c). Core 1 takes a job only at
// the top of its menu loop, the TUI's idle point (ctl_core1_idle), never in
// a dialog. The TUI on interface 0 is untouched: no byte of one port is ever
// read as the other's, and the two share no queue.
//
// Stage 1 of the build (SPEC-v1 12.1): the descriptor, the transport and
// framing, the parser, the envelope with ids and the replay table, and the
// Tier R ops. Tier S (the ratchet), the jumpers, the lease and Tier M are
// not in this stage; hello lists them ABSENT (ctl_policy.h).
#ifndef CTL_H
#define CTL_H

#include <stdint.h>
#include <stdbool.h>

// Core 0 (ATAboy.c).
void ctl_init(void);
void ctl_task(void);
// usb.c's tud_mount_cb: SET_CONFIGURATION, after a USB reset. Only sets a
// flag; ctl_task() does the rest (SPEC-v1 2.4, R-API-1 finding 16).
void ctl_on_usb_mount(void);

// Core 1 (menus.c). The TUI's state as status.tui.state reports it.
enum {
    CTL_TUI_BOOT, CTL_TUI_MAIN, CTL_TUI_FEATURES, CTL_TUI_DEBUG, CTL_TUI_MOUNTED,
    CTL_TUI_CONFIRM, CTL_TUI_ERROR_BOX, CTL_TUI_MODAL,
};
// The top of core 1's loop: publishes the state, and runs a queued job
// there when the state is an idle one (MAIN, FEATURES, DEBUG, MOUNTED).
void ctl_core1_idle(uint8_t tui_state);
// A key taken from the TUI queue: the TUI is busy with it (MODAL) until the
// loop comes round to its top again.
void ctl_tui_busy(void);
// A MOUNT marker was written (menus.c log_mount): status.mount_seq counts
// mounts since boot (SPEC-v1 4.3, R-API-1 finding 11).
void ctl_mount_noted(uint32_t marker_log_seq);

#endif
