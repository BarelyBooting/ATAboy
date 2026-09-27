// ATAboy Control, stage 3 (palimpsest api/SPEC-v1.md section 8): the
// physical-presence jumpers on the Rev .6 board. Built only with
// ATABOY_BOARD_JUMPERS=1, which needs ATABOY_CTL=1 (CMakeLists.txt).
//
//   JP1, net /OPT1_JMPR, GPIO30 (U5.38): LOCK, a standing state (8.2)
//   JP2, net /OPT2_JMPR, GPIO31 (U5.39): PERMIT, an act (8.3)
//
// The other pin of each header is GND and neither net has a resistor (1.3),
// so the pad's own pull-up is the whole bias: fitted reads low, open high.
// On the owner's unit the headers are not installed (owner, Q1): both read
// OPEN, so there is no LOCK and no fit event can ever happen.
//
// Fail closed (8.1, R-API-1 finding 13): before every decision the pad and
// IO registers of both pins are read back. Pull-up on, pull-down off, input
// enabled, isolation clear, function SIO, SIO output enable clear, and no
// output-enable, output or input override; anything else makes BOTH jumpers
// UNREADABLE, which is LOCK engaged and no PERMIT. The pads' reset state
// (pull-down, isolated, input off) reads low, which is "fitted": without the
// read-back it would look like a jumper, and so would a GPIO31 driven low.
//
// JP1 is read live at every jp1_locked() call (finding 12): any low sample
// (or an unreadable pad) locks at once, from either core, and forces
// config.drive_write_protected true in RAM, so taking the shunt off does not
// bring back an older "Disabled" (8.2). Only the release waits: 20 equal OPEN
// samples over at least 10 ms, taken by bj_task() on core 0.
//
// JP2 is sampled by bj_task() only, on core 0 (ctl_task). Its machine starts
// UNKNOWN at power-up and goes back to UNKNOWN after any gap over 50 ms
// between two passes (a long USB callback) and whenever the pads read back
// wrong, so it never makes an event from a level it did not see (I-25,
// finding 5). A fit event is an observed OPEN of 200 ms, then an observed
// FITTED of 200 ms. The OPEN is the debounced level (so the bounce of a shunt
// going on does not end it); the FITTED 200 ms must be unbroken, sample for
// sample, from its edge. No edge latch is read. An event is reported once,
// by the pass that completes it, and is never stored (8.3): stage 4 approves
// the one Tier M request that arrived before the event's edge, there and
// then, or nothing. Stage 3 has no Tier M, so an event is counted and does
// nothing else.
#ifndef BOARD_JUMPERS_H
#define BOARD_JUMPERS_H

#include <stdint.h>
#include <stdbool.h>

#if !ATABOY_CTL
#error "the jumpers are the Control API's (SPEC-v1 8): ATABOY_BOARD_JUMPERS needs ATABOY_CTL"
#endif

#define BJ_JP1_PIN          30u     // /OPT1_JMPR, U5.38
#define BJ_JP2_PIN          31u     // /OPT2_JMPR, U5.39

#define BJ_DEBOUNCE_SAMPLES 20u     // SPEC-v1 8.1: 20 equal samples...
#define BJ_DEBOUNCE_MS      10u     // ...over at least 10 ms
#define BJ_GAP_MS           50u     // a longer gap between passes: back to UNKNOWN (8.3)
#define BJ_OPEN_MS          200u    // an event: OPEN observed this long...
#define BJ_FITTED_MS        200u    // ...then FITTED observed this long

// A jumper as status.jumpers reports it (SPEC-v1 4.3). NOT_BUILT is what a
// build without this file reports (ctl.c).
enum { BJ_UNKNOWN, BJ_OPEN, BJ_FITTED, BJ_UNREADABLE };

// One fit event: the first FITTED sample of its 200 ms (its edge, which stage
// 4 compares with the waiting request's arrival) and the pass that ended it.
typedef struct {
    uint32_t edge_ms;
    uint32_t at_ms;
} bj_event_t;

// Core 0, once, before tusb_init() and before core 1 is launched (ATAboy.c).
void board_jumpers_init(void);

// Any core. True while JP1 is fitted, unreadable, or not yet released after
// either (8.2); true before board_jumpers_init() too, without forcing
// anything. ide_write_protected() calls it, and through it WRITE(10)'s check
// (usb.c) and ide_write_sectors()'s; the TUI's Write Protect toggle calls it.
bool jp1_locked(void);

// Core 0, every pass of ctl_task(), whatever the Control port is doing.
// Samples both jumpers, releases JP1 after a debounced OPEN, runs JP2's
// machine. True, with *ev filled, on the pass that completes a fit event.
bool bj_task(uint32_t now_ms, bj_event_t *ev);

// What status.jumpers shows (BJ_*): JP1's lock state, JP2's debounced level.
uint8_t bj_jp1(void);
uint8_t bj_jp2(void);

// selftest.status.jumpers (4.10): the read-back of both pads, now. Reads only.
bool bj_pads_ok(void);

// Tier M's jumper precondition (I-4, I-7), for stage 4: JP1 released (a
// debounced OPEN seen) and, read live now, open and readable.
bool bj_tier_m_open(void);

// Fit events since power-up (stage 3 counts them and does nothing else).
uint32_t bj_fit_events(void);

#endif
