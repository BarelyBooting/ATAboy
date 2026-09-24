// Manual CHS with no IDENTIFY: the decisions, kept pure (no hardware, no
// Pico SDK) so tests/host can check them. menus.c does the drawing and the
// entry; ide.c (ide_manual_chs) talks to the drive.
//
// Why it exists: some drives must be given an explicit geometry without being
// asked who they are. The first one is a 1990 Conner CP3044, whose manual
// never says it is ATA ("emulates IBM Task File") and has no IDENTIFY chapter,
// and whose model is on record showing thousands of false bad sectors under
// a BIOS's auto detection. Auto Detect always sends IDENTIFY (0xEC) first,
// and so does the forced route out of its error box. This entrance does not.
#ifndef MANUALCHS_H
#define MANUALCHS_H

#include <stdbool.h>
#include <stdint.h>

// The key: Ctrl+G (07h), on the main menu. A control key for the same reason
// as Ctrl+F (fwupdate.h): no split or damaged escape sequence from a terminal
// can produce one, so it is never pressed by accident that way. Nothing is
// sent to the drive before a final Y anyway.
#define MANUAL_CHS_KEY      0x07

// Only on the main menu, and only with nothing mounted: a mounted drive
// belongs to the USB host. (A reset still pending from a USB command is
// refused too, by menus.c, since ide.c keeps that state.)
static inline bool manual_chs_key_opens(bool on_main_menu, bool mounted, int key) {
    return on_main_menu && !mounted && key == MANUAL_CHS_KEY;
}

// The limits, from INITIALIZE DEVICE PARAMETERS (0x91) and the CHS address:
// the cylinder is 16 bits, the head is the low nibble of the device register
// (heads - 1 is sent, so 16 at most), and the sector number is 8 bits,
// counted from 1. 0 in a field means it was left empty. A product of zero
// cannot get through: each factor is at least 1.
#define MANUAL_CHS_MAX_CYLS  65535u
#define MANUAL_CHS_MAX_HEADS 16u
#define MANUAL_CHS_MAX_SPT   255u

// The first field that is out of range (0 cylinders, 1 heads, 2 sectors),
// or -1 when all three are good. Nothing is sent for a refused entry.
static inline int manual_chs_bad_field(uint32_t c, uint32_t h, uint32_t s) {
    if (c < 1 || c > MANUAL_CHS_MAX_CYLS) return 0;
    if (h < 1 || h > MANUAL_CHS_MAX_HEADS) return 1;
    if (s < 1 || s > MANUAL_CHS_MAX_SPT) return 2;
    return -1;
}

// What the screen says for each refused field.
static inline const char *manual_chs_refusal(int field) {
    static const char *why[] = {
        "Refused: cylinders must be 1 to 65535. Nothing sent.",
        "Refused: heads must be 1 to 16. Nothing sent.",
        "Refused: sectors per track must be 1 to 255. Nothing sent.",
    };
    return field >= 0 && field < 3 ? why[field] : "";
}

#endif
