// ATAboy Control, stage 3: JP1 LOCK and JP2 PERMIT (board_jumpers.h;
// palimpsest api/SPEC-v1.md section 8). Built only with ATABOY_BOARD_JUMPERS=1.
//
// Nothing here touches the IDE bus, the settings sector or the TUI: the only
// registers read are the pad, IO and SIO registers of GPIO30 and GPIO31, and
// the only ones written are theirs, once, by board_jumpers_init()
// (tests/host/check_ctl.py checks the call graph on the built ELF). The one
// store outside this file is config.drive_write_protected = true, in RAM,
// never false.

#include "board_jumpers.h"
#include "config.h"
#include "ctl_state.h"
#include "hardware/gpio.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/structs/pads_bank0.h"
#include "hardware/structs/sio.h"

#if !ATABOY_HOST_TEST
// GPIO30 and GPIO31 exist in the B package only (RP2354B, QFN-80); the board
// header declares it (PICO_RP2350A 0) in this build and only in this build
// (boards/ataboy_rev6.h). Both are below 32: SIO's low bank, one word.
_Static_assert(NUM_BANK0_GPIOS == 48, "ATABOY_BOARD_JUMPERS needs the RP2350B pin count (boards/ataboy_rev6.h)");
#endif
_Static_assert(BJ_JP1_PIN < 32 && BJ_JP2_PIN < 32, "the jumpers are read through SIO's low bank");

#define BJ_PINS      ((1u << BJ_JP1_PIN) | (1u << BJ_JP2_PIN))
// The pad: pull-up on, pull-down off, input enabled, isolation clear.
#define BJ_PAD_MASK  (PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS | PADS_BANK0_GPIO0_IE_BITS | \
                      PADS_BANK0_GPIO0_ISO_BITS)
#define BJ_PAD_WANT  (PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_IE_BITS)
// The IO bank: function SIO, no output, output-enable or input override.
#define BJ_CTRL_MASK (IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS | IO_BANK0_GPIO0_CTRL_OUTOVER_BITS | \
                      IO_BANK0_GPIO0_CTRL_OEOVER_BITS | IO_BANK0_GPIO0_CTRL_INOVER_BITS)
#define BJ_CTRL_WANT ((uint32_t)GPIO_FUNC_SIO << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB)

// A run of equal samples: its level, how many, its first and last times.
typedef struct {
    uint8_t  level;
    uint16_t n;
    uint32_t first, last;
} bj_run_t;

static struct {
    bool     inited;
    bool     started;           // bj_task has run once (the gap rule needs a last pass)
    // JP1's lock state (BJ_*, bits 0-7) and the count of low or unreadable
    // samples seen by jp1_locked() on either core (bits 8-31), in one word:
    // either core locks it, core 0 releases it with one compare-and-swap,
    // which fails if any lock was seen since it looked.
    volatile uint32_t jp1w;
    uint32_t r1_locks;          // the count when JP1's current run of samples began
    uint8_t  jp2;               // BJ_*: JP2's debounced level (core 0 only)
    uint32_t last_ms;           // the last pass
    bj_run_t r1, r2;            // the current run of each jumper's samples
    uint32_t open_since;        // JP2: the first sample of its debounced OPEN
    bool     cand;              // JP2: an OPEN of 200 ms then this FITTED edge: an event in the making
    uint32_t edge;              // ...the edge, the first FITTED sample
    uint32_t events;            // fit events since power-up
    uint32_t gaps;              // passes that found a gap over BJ_GAP_MS
} bj;

// ---------------------------------------------------------------------------
//  The read-back (SPEC-v1 8.1, R-API-1 finding 13)
// ---------------------------------------------------------------------------

static bool pad_ok(unsigned pin) {
    return (pads_bank0_hw->io[pin] & BJ_PAD_MASK) == BJ_PAD_WANT &&
           (io_bank0_hw->io[pin].ctrl & BJ_CTRL_MASK) == BJ_CTRL_WANT;
}

bool bj_pads_ok(void) {
    return pad_ok(BJ_JP1_PIN) && pad_ok(BJ_JP2_PIN) && !(sio_hw->gpio_oe & BJ_PINS);
}

// Both levels in one read of SIO's input register, trusted only when the
// read-back passes before it and after it: a pad changed in between is not
// believed either.
static bool sample(uint32_t *in) {
    if (!bj_pads_ok()) return false;
    *in = sio_hw->gpio_in;
    return bj_pads_ok();
}

// ---------------------------------------------------------------------------
//  JP1: LOCK (8.2)
// ---------------------------------------------------------------------------

// Write protect on in RAM, and the epoch counted after the store (ctl_state.h).
static void force_wp(void) {
    if (config.drive_write_protected) return;
    config.drive_write_protected = true;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    CTL_EPOCH_BUMP();
}

// JP1 seen fitted or unreadable, now: write protect forced on in RAM, then
// the state published with one more sighting counted, then one count of the
// epoch for both (never before the stores).
static void lock_seen(uint8_t st) {
    bool forced = !config.drive_write_protected;
    if (forced) config.drive_write_protected = true;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    uint32_t o = bj.jp1w, n;
    do {
        n = ((o & ~0xFFu) + 0x100u) | st;
    } while (!__atomic_compare_exchange_n(&bj.jp1w, &o, n, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST));
    if (forced || (o & 0xFFu) != st) CTL_EPOCH_BUMP();
}

bool jp1_locked(void) {
    if (!bj.inited) return true;            // nothing read yet: locked, and nothing forced
    uint32_t in;
    if (!sample(&in)) { lock_seen(BJ_UNREADABLE); return true; }
    if (!(in & (1u << BJ_JP1_PIN))) { lock_seen(BJ_FITTED); return true; }
    uint8_t p = (uint8_t)(bj.jp1w & 0xFFu);
    if (p == BJ_FITTED || p == BJ_UNREADABLE) {
        // Open now, but not released yet (bj_task: a debounced OPEN). Still
        // locked, and still forcing: a store of false that raced the lock
        // (the TUI's toggle, on core 1) is undone here.
        force_wp();
        return true;
    }
    return false;
}

uint8_t bj_jp1(void) { return (uint8_t)(bj.jp1w & 0xFFu); }
uint8_t bj_jp2(void) { return bj.jp2; }
uint32_t bj_fit_events(void) { return bj.events; }

bool bj_tier_m_open(void) {
    bool locked = jp1_locked();
    return !locked && bj_jp1() == BJ_OPEN;
}

// ---------------------------------------------------------------------------
//  Sampling, on core 0 (ctl_task)
// ---------------------------------------------------------------------------

static void run_step(bj_run_t *r, uint8_t s, uint32_t now) {
    if (r->n && r->level == s) {
        if (r->n < 0xFFFFu) r->n++;
        r->last = now;
        return;
    }
    r->level = s;
    r->n = 1;
    r->first = now;
    r->last = now;
}

static bool settled(const bj_run_t *r) {
    return r->n >= BJ_DEBOUNCE_SAMPLES && r->last - r->first >= BJ_DEBOUNCE_MS;
}

// JP2's machine back to a level it has not observed: nothing seen before
// counts toward an event (I-25).
static void jp2_forget(uint8_t level) {
    bj.jp2 = level;
    bj.cand = false;
    bj.r2.n = 0;
}

bool bj_task(uint32_t now, bj_event_t *ev) {
    if (!bj.inited) return false;
    if (bj.started && now - bj.last_ms > BJ_GAP_MS) {
        // Core 0 was away (a USB callback can hold it 20 s, ide.h): what the
        // jumpers did meanwhile was not seen, so it does not count. The
        // human refits JP2 (fail closed).
        bj.gaps++;
        jp2_forget(BJ_UNKNOWN);
        bj.r1.n = 0;
    }
    bj.started = true;
    bj.last_ms = now;
    uint32_t in;
    if (!sample(&in)) {
        lock_seen(BJ_UNREADABLE);
        jp2_forget(BJ_UNREADABLE);
        bj.r1.n = 0;
        return false;
    }
    // JP1: a low sample locks at once; the release needs the debounced OPEN,
    // all of it after the last low sample anyone saw (a shunt fitted and
    // taken off between two passes, seen only by a live read on either
    // core, starts the OPEN again).
    uint8_t s1 = (in & (1u << BJ_JP1_PIN)) ? BJ_OPEN : BJ_FITTED;
    uint32_t w = bj.jp1w;
    if ((w >> 8) != bj.r1_locks) { bj.r1_locks = w >> 8; bj.r1.n = 0; }
    run_step(&bj.r1, s1, now);
    if (s1 == BJ_FITTED) {
        lock_seen(BJ_FITTED);
    } else if (settled(&bj.r1) && (w & 0xFFu) != BJ_OPEN) {
        if (__atomic_compare_exchange_n(&bj.jp1w, &w, (w & ~0xFFu) | BJ_OPEN, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST))
            CTL_EPOCH_BUMP();
    }
    // JP2: the debounced level, and the event (8.3). Its level is not part
    // of the epoch (SPEC-v1 4.3): refitting JP2 must not make a waiting
    // request's expect_epoch stale, or cancel it (I-26).
    uint8_t s2 = (in & (1u << BJ_JP2_PIN)) ? BJ_OPEN : BJ_FITTED;
    run_step(&bj.r2, s2, now);
    if (settled(&bj.r2) && bj.r2.level != bj.jp2) {
        if (s2 == BJ_OPEN) {
            bj.open_since = bj.r2.first;
            bj.cand = false;
        } else {
            bj.cand = bj.jp2 == BJ_OPEN && bj.r2.first - bj.open_since >= BJ_OPEN_MS;
            bj.edge = bj.r2.first;
        }
        bj.jp2 = s2;
    }
    if (bj.cand) {
        if (bj.r2.level != BJ_FITTED || bj.r2.first != bj.edge) {
            bj.cand = false;                // an OPEN sample inside the 200 ms: no event, refit
        } else if (now - bj.edge >= BJ_FITTED_MS) {
            bj.cand = false;                // one event per fit
            bj.events++;
            ev->edge_ms = bj.edge;
            ev->at_ms = now;
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
//  The pads (8.1), before tusb_init() and core 1 (ATAboy.c)
// ---------------------------------------------------------------------------

void board_jumpers_init(void) {
    static const unsigned pins[2] = { BJ_JP1_PIN, BJ_JP2_PIN };
    for (int i = 0; i < 2; i++) {
        // The pull-up first, while the pad is still isolated at its reset
        // state (pull-down), so it is never let go low; then gpio_init: SIO,
        // an input, input enabled, isolation cleared (SDK gpio.c).
        gpio_pull_up(pins[i]);
        gpio_init(pins[i]);
    }
    bj.jp1w = BJ_UNKNOWN;
    bj.jp2 = BJ_UNKNOWN;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    bj.inited = true;
}
