// PIO bus timing, kept pure so tests/host can check it (holistic review
// 2026-09-26, L-6 and M-2). ataboy.pio counts in state machine cycles; every
// ATA PIO mode 0 number it meets depends on one cycle being at least
// IDE_PIO_CYCLE_NS long.
#ifndef IDE_PIO_TIMING_H
#define IDE_PIO_TIMING_H

#include <stdint.h>

#define IDE_PIO_CYCLE_NS 100u

// The integer clock divider that makes one state machine cycle at least
// IDE_PIO_CYCLE_NS at clk_sys = clk_hz. Rounded up: a clock that does not
// divide evenly gives slightly longer cycles, never shorter. 150 MHz (the
// SDK 2.2.0 default this firmware was measured at) gives 15, as before.
static inline uint32_t ide_pio_clkdiv_for(uint32_t clk_hz) {
    uint32_t per = 1000000000u / IDE_PIO_CYCLE_NS;          // cycles per second wanted
    uint32_t div = (clk_hz + per - 1) / per;
    return div < 1 ? 1 : (div > 65535 ? 65535 : div);
}

#endif
