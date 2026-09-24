// Firmware update mode from the console: the decisions, kept pure (no
// hardware, no Pico SDK) so tests/host can check them. menus.c does the
// drawing and the reboot.
#ifndef FWUPDATE_H
#define FWUPDATE_H

#include <stdbool.h>
#include <stdint.h>

// The key that opens "Enter firmware update mode (Y/N)?": Ctrl+F (06h).
// It used to be B, and a terminal's Down arrow is ESC [ B: if the ESC [ part
// comes late or is lost, get_input() hands back a bare 'B' (review finding
// L1). No escape sequence can produce 06h. CSI and SS3 parameter and
// intermediate bytes are 20h..3Fh and final bytes 40h..7Eh, so a split or
// damaged arrow, F-key or PgUp/PgDn sequence only ever yields ESC (1Bh) or
// printable bytes.
#define FWUPDATE_KEY        0x06

// How long Y waits for a USB mass storage command that is still running on
// core 0 before giving up. Since review M-1 (0.6f3p7) a host command ends
// within its budget, IDE_HOST_BUDGET_MS (20 s, ide.h), plus a small margin,
// so this is three times that; should one still be running, Y refuses,
// which is the safe answer, and can be tried again.
#define FWUPDATE_WAIT_MS    60000u

// Does this key, on this screen, open the prompt? Only on the main menu, and
// only with nothing mounted: a mounted drive belongs to the USB host, and
// the reboot would pull it away in the middle of a transfer.
static inline bool fwupdate_key_opens_prompt(bool on_main_menu, bool mounted, int key) {
    return on_main_menu && !mounted && key == FWUPDATE_KEY;
}

// What to do after Y. `usb_busy` is true while an MSC callback that may use
// the IDE bus is running on core 0 (usb.c): unmounting stops new commands
// from reaching the drive, but not one that had already started (review
// finding L3). `waited_ms` is how long Y has been waiting for it.
typedef enum {
    FWUPDATE_GO,                // nothing mounted, nothing running: reboot
    FWUPDATE_WAIT,              // a USB command is still running: ask again shortly
    FWUPDATE_REFUSE_MOUNTED,    // a drive is mounted: do not reboot
    FWUPDATE_REFUSE_BUSY,       // still running after FWUPDATE_WAIT_MS: do not reboot
} fwupdate_step_t;

static inline fwupdate_step_t fwupdate_step(bool mounted, bool usb_busy, uint32_t waited_ms) {
    if (mounted) return FWUPDATE_REFUSE_MOUNTED;
    if (!usb_busy) return FWUPDATE_GO;
    return waited_ms < FWUPDATE_WAIT_MS ? FWUPDATE_WAIT : FWUPDATE_REFUSE_BUSY;
}

#endif
