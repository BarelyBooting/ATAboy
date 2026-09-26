// Faults and stack checks (holistic review 2026-09-26, L-4 and L-5; the
// owner's RP2354B guide, MEDIUM F1). fault.c has the HardFault handler.
//
// A fault on either core: the record is kept for Debug E (ide.h,
// ide_fault_record; it survives a warm reset, not a power cycle), the IDE
// bus is released (ide_fault_release_bus: nothing driven, nothing asserted,
// RESET- high), the other core is held in reset so nothing else reaches the
// bus, and the core stops. No reboot: a reboot would re-enumerate USB in the
// middle of a session, clear the no-IDENTIFY rule and could run Auto Mount.
// The ATAboy is then recovered by unplugging it.
#ifndef FAULT_H
#define FAULT_H

#include <stdint.h>
#include <stdbool.h>

// Core 1's stack canary (ATAboy.c) is intact. Core 1 checks it every pass
// of its menu loop.
bool core1_stack_intact(void);

// Stop as a fault would: record code as the CFSR field, release the bus,
// hold the other core, halt.
void fault_stop(uint32_t code) __attribute__((noreturn));

#endif
