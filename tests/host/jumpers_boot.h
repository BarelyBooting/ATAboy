// The ATAboy Control API's stage 3 (palimpsest api/SPEC-v1.md 8) in a host
// test of ide.c or menus.c: with ATABOY_BOARD_JUMPERS the real
// board_jumpers.c, its pads set up before the test starts, as ATAboy.c's
// main() sets them up before tusb_init(), and both shunts off (the owner's
// unit has no headers). Nothing without ATABOY_BOARD_JUMPERS, so older source
// trees (tui_compare.py) build as before.
#pragma once
#if ATABOY_BOARD_JUMPERS
#include "board_jumpers.c"
static const bool jumpers_booted = (board_jumpers_init(), true);
#endif
