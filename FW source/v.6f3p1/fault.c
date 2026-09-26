// HardFault handling (holistic review 2026-09-26, L-5; the owner's RP2354B
// guide, MEDIUM F1). See fault.h.
//
// The SDK's default HardFault handler is a breakpoint, which with no
// debugger locks the core up and leaves the bus exactly as it was: CS0- low
// and DIOR- held by a stalled read, or the data lines still driven after a
// write. This one runs on a stack of its own (the faulting stack may be the
// problem), records what it can, releases the bus, holds the other core in
// reset, and waits for the ATAboy to be unplugged.
#include "fault.h"
#include "ide.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "hardware/structs/psm.h"
#include "hardware/regs/psm.h"
#include "hardware/regs/addressmap.h"

#define FAULT_STACK_WORDS 256
static uint32_t fault_stack[2][FAULT_STACK_WORDS] __attribute__((aligned(8)));
// Read by the handler's assembly: the top of each core's fault stack.
uint32_t *const fault_stack_tops[2] = { &fault_stack[0][FAULT_STACK_WORDS], &fault_stack[1][FAULT_STACK_WORDS] };

static void __not_in_flash_func(fault_park)(uint32_t pc, uint32_t lr, uint32_t cfsr) __attribute__((noreturn));
static void __not_in_flash_func(fault_park)(uint32_t pc, uint32_t lr, uint32_t cfsr) {
    uint32_t core = get_core_num();
    ide_fault_record(core, pc, lr, cfsr);
    ide_fault_release_bus();
    // Nothing else reaches the bus: the other core is held in reset.
    hw_set_bits(&psm_hw->frce_off, core ? PSM_FRCE_OFF_PROC0_BITS : PSM_FRCE_OFF_PROC1_BITS);
    for (;;) __wfi();
}

// Called from isr_hardfault on the fault stack, with the stacked frame of
// the code that faulted (r0-r3, r12, lr, pc, xpsr), if it looks like RAM.
void __not_in_flash_func(fault_from_handler)(uint32_t *frame) __attribute__((noreturn, used));
void __not_in_flash_func(fault_from_handler)(uint32_t *frame) {
    uint32_t cfsr = *(volatile uint32_t *)0xE000ED28u;     // SCB CFSR
    uint32_t pc = 0, lr = 0;
    uintptr_t f = (uintptr_t)frame;
    if (!(f & 3u) && f >= SRAM_BASE && f + 32u <= SRAM_END) { lr = frame[5]; pc = frame[6]; }
    fault_park(pc, lr, cfsr);
}

void fault_stop(uint32_t code) {
    (void)save_and_disable_interrupts();
    fault_park(0, 0, code);
}

// Replaces the SDK's weak default (crt0.S: a breakpoint).
void __attribute__((naked)) isr_hardfault(void) {
    __asm volatile(
        "movs r0, #4            \n"
        "mov  r1, lr            \n"
        "tst  r0, r1            \n"
        "beq  1f                \n"
        "mrs  r0, psp           \n"
        "b    2f                \n"
        "1:                     \n"
        "mrs  r0, msp           \n"
        "2:                     \n"
        "ldr  r1, =0xd0000000   \n"     // SIO CPUID: 0 or 1
        "ldr  r1, [r1]          \n"
        "lsls r1, r1, #2        \n"
        "ldr  r2, =fault_stack_tops \n"
        "ldr  r2, [r2, r1]      \n"
        "movs r3, #0            \n"     // MSPLIM off: the fault stack is below the limit
        "msr  msplim, r3        \n"
        "mov  sp, r2            \n"
        "b    fault_from_handler \n"
        ".ltorg                 \n");
}
