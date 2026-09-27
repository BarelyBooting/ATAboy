// Stand-ins for the parts of the Pico SDK and TinyUSB that ide.c and usb.c
// use, so the real firmware sources can be compiled and run on a PC against
// a simulated drive (sim.hpp). Built as C++ so that writes to the SIO
// set/clear registers can be seen by the simulator.
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ---- virtual clock (nanoseconds) ------------------------------------------
extern uint64_t mock_now_ns;
typedef uint64_t absolute_time_t;
static inline absolute_time_t get_absolute_time(void) { return mock_now_ns / 1000; }
static inline uint32_t to_ms_since_boot(absolute_time_t t) { return (uint32_t)(t / 1000); }
static inline uint64_t to_us_since_boot(absolute_time_t t) { return t; }
static inline void busy_wait_us_32(uint32_t us) { mock_now_ns += (uint64_t)us * 1000; }
static inline void busy_wait_at_least_cycles(uint32_t c) { mock_now_ns += c * 7ull; }  // ~150 MHz
static inline void sleep_ms(uint32_t ms) { mock_now_ns += (uint64_t)ms * 1000000; }
static inline void tight_loop_contents(void) {}

// ---- GPIO / SIO -----------------------------------------------------------
extern uint32_t mock_gpio_out;
struct MockSioReg {
    bool is_set;
    MockSioReg &operator=(uint32_t v) {
        if (is_set) mock_gpio_out |= v; else mock_gpio_out &= ~v;
        return *this;
    }
};
// Output enables, as the firmware sets them (holistic review, B1 and the
// fault handler's bus release).
inline uint32_t mock_gpio_oe = 0;
struct MockSioOe {
    bool is_set;
    MockSioOe &operator=(uint32_t v) {
        if (is_set) mock_gpio_oe |= v; else mock_gpio_oe &= ~v;
        return *this;
    }
};
#define GPIO_IN 0
#define GPIO_OUT 1
enum { GPIO_OVERRIDE_NORMAL = 0, GPIO_OVERRIDE_INVERT, GPIO_OVERRIDE_LOW, GPIO_OVERRIDE_HIGH };
bool mock_intrq(void);
// The IDE RESET- line (GPIO 23) is driven with gpio_put; the simulated drive
// sees every change of it through this (test_read.cpp; a no-op elsewhere).
void mock_reset_line(bool high);
// Review C, C-5: gpio_init_mask leaves the output latches at 0, as the SDK
// does, and the latch as it was when the pins became outputs is kept.
inline uint32_t mock_gpio_out_at_dir = 0;
// Holistic review, B1: a pin of RESET- (23), CS0- (24), CS1- (25) or the
// transceiver enables (18, 19) that is let go to its pad pull-down: made an
// input by gpio_init or gpio_init_mask, or switched to SIO while not an
// output driven high. On the board that is a dip on the line (warm boot).
#define MOCK_ACTIVE_LOW ((1u << 18) | (1u << 19) | (1u << 23) | (1u << 24) | (1u << 25))
inline int mock_active_low_dips = 0;
inline uint32_t mock_sio_pins = 0;          // pins switched to SIO (gpio_set_function)
enum { GPIO_FUNC_SIO = 5, GPIO_FUNC_PIO0 = 6 };

// ---- the pad and IO bank registers, as a register file ---------------------
// (Stage 3 of the ATAboy Control API: board_jumpers.c reads them back.) Each
// starts at the RP2350's reset value: pads 0x116 (pull-down on, pull-up off,
// input off, isolated), IO ctrl 0x1f (no function). The SDK calls below set
// them as the SDK does (gpio.c); a test may write any value into them.
#define PADS_BANK0_GPIO0_ISO_BITS          0x00000100u
#define PADS_BANK0_GPIO0_OD_BITS           0x00000080u
#define PADS_BANK0_GPIO0_IE_BITS           0x00000040u
#define PADS_BANK0_GPIO0_PUE_BITS          0x00000008u
#define PADS_BANK0_GPIO0_PDE_BITS          0x00000004u
#define IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS   0x0000001fu
#define IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB    0
#define IO_BANK0_GPIO0_CTRL_OUTOVER_BITS   0x00003000u
#define IO_BANK0_GPIO0_CTRL_OEOVER_BITS    0x0000c000u
#define IO_BANK0_GPIO0_CTRL_INOVER_BITS    0x00030000u
#define MOCK_PAD_RESET  0x116u
#define MOCK_CTRL_RESET 0x1fu
struct MockPads { uint32_t io[48]; };
struct MockIoCtrl { uint32_t status, ctrl; };
struct MockIoBank { MockIoCtrl io[48]; };
inline MockPads mock_pads;
inline MockIoBank mock_io;
static inline void mock_pads_reset(void) {
    for (int i = 0; i < 48; i++) { mock_pads.io[i] = MOCK_PAD_RESET; mock_io.io[i].ctrl = MOCK_CTRL_RESET; }
}
inline bool mock_pads_at_reset = (mock_pads_reset(), true);
#define pads_bank0_hw (&mock_pads)
#define io_bank0_hw (&mock_io)
// JP1 (GPIO 30) and JP2 (GPIO 31): a fitted shunt ties the pin to GND.
inline bool mock_jumper_fitted[2] = { false, false };
inline bool mock_floating_high = false;     // a pin with no pull and nothing driving it
// A jumper pad's isolation cleared while it was pulled down and not up: the
// pin let go low for a moment (board_jumpers_init sets the pull-up first).
inline int mock_jumper_let_low = 0;
// What SIO's input register reads on GPIO 30 and 31, from the registers and
// the shunts: input off or isolated reads 0; a shunt wins; else what drives
// it (SIO's output when enabled, or an override), else the pulls.
static inline bool mock_pin_level(unsigned pin) {
    uint32_t pad = mock_pads.io[pin], ctrl = mock_io.io[pin].ctrl;
    if (!(pad & PADS_BANK0_GPIO0_IE_BITS) || (pad & PADS_BANK0_GPIO0_ISO_BITS)) return false;
    unsigned oeover = (ctrl >> 14) & 3, outover = (ctrl >> 12) & 3, inover = (ctrl >> 16) & 3;
    bool sio = (ctrl & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS) == GPIO_FUNC_SIO;
    bool oe = sio && ((mock_gpio_oe >> pin) & 1);
    oe = oeover == 3 ? true : oeover == 2 ? false : (oe != (oeover == 1));
    bool out = sio && ((mock_gpio_out >> pin) & 1);
    out = outover == 3 ? true : outover == 2 ? false : (out != (outover == 1));
    if (pad & PADS_BANK0_GPIO0_OD_BITS) oe = false;
    bool level;
    if (pin >= 30 && pin < 32 && mock_jumper_fitted[pin - 30]) level = false;
    else if (oe) level = out;
    else if (pad & PADS_BANK0_GPIO0_PUE_BITS) level = true;
    else if (pad & PADS_BANK0_GPIO0_PDE_BITS) level = false;
    else level = mock_floating_high;
    return inover == 3 ? true : inover == 2 ? false : (level != (inover == 1));
}
// Called as SIO's input register is read (a test changes a pad there, as
// another core or a fault could between the read-back and the read).
inline void (*mock_on_gpio_in)(void) = nullptr;
struct MockSioIn { operator uint32_t() const {
    if (mock_on_gpio_in) mock_on_gpio_in();
    uint32_t v = 0;
    for (unsigned p = 30; p < 32; p++) if (mock_pin_level(p)) v |= 1u << p;
    return v;
} };
struct MockSioOeRead { operator uint32_t() const { return mock_gpio_oe; } };
struct MockSio {
    MockSioReg gpio_set{true}; MockSioReg gpio_clr{false}; MockSioOe gpio_oe_set{true}; MockSioOe gpio_oe_clr{false};
    MockSioIn gpio_in; MockSioOeRead gpio_oe;
};
extern MockSio mock_sio;
#define sio_hw (&mock_sio)
// gpio_set_function's pad and IO bank writes (SDK gpio.c): input on, output
// disable off, the function with no overrides, then isolation cleared.
static inline void mock_pad_to_function(unsigned pin, unsigned fn) {
    if (pin >= 48) return;
    uint32_t pad = mock_pads.io[pin];
    if (pin >= 30 && pin < 32 && (pad & PADS_BANK0_GPIO0_ISO_BITS) && (pad & PADS_BANK0_GPIO0_PDE_BITS) &&
        !(pad & PADS_BANK0_GPIO0_PUE_BITS))
        mock_jumper_let_low++;
    pad = (pad | PADS_BANK0_GPIO0_IE_BITS) & ~PADS_BANK0_GPIO0_OD_BITS;
    mock_io.io[pin].ctrl = fn << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    mock_pads.io[pin] = pad & ~PADS_BANK0_GPIO0_ISO_BITS;
}
static inline void mock_pulls(unsigned pin, bool up, bool down) {
    if (pin >= 48) return;
    uint32_t pad = mock_pads.io[pin] & ~(PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS);
    mock_pads.io[pin] = pad | (up ? PADS_BANK0_GPIO0_PUE_BITS : 0) | (down ? PADS_BANK0_GPIO0_PDE_BITS : 0);
}
static inline void gpio_init_mask(uint32_t m) {
    if (m & MOCK_ACTIVE_LOW) mock_active_low_dips++;
    mock_gpio_out &= ~m; mock_gpio_oe &= ~m;
    for (unsigned p = 0; p < 32; p++) if (m & (1u << p)) mock_pad_to_function(p, GPIO_FUNC_SIO);
}
static inline void gpio_set_dir_out_masked(uint32_t m) { mock_gpio_out_at_dir = mock_gpio_out; mock_gpio_oe |= m; }
static inline void gpio_init(unsigned pin) {
    if (MOCK_ACTIVE_LOW & (1u << pin)) mock_active_low_dips++;
    mock_gpio_out &= ~(1u << pin); mock_gpio_oe &= ~(1u << pin);
    mock_pad_to_function(pin, GPIO_FUNC_SIO);
}
static inline void gpio_set_function(unsigned pin, unsigned fn) {
    if (fn == GPIO_FUNC_SIO) {
        if ((MOCK_ACTIVE_LOW & (1u << pin)) && !(mock_gpio_out & mock_gpio_oe & (1u << pin))) mock_active_low_dips++;
        mock_sio_pins |= 1u << pin;
    }
    mock_pad_to_function(pin, fn);
}
#define __uninitialized_ram(name) name
#define __not_in_flash_func(name) name
#define __scratch_x(group)
#define ATABOY_HOST_TEST 1
static inline uint8_t rp2350_chip_version(void) { return 4; }
// F1 stage 1: which core a log entry came from (obs.c); tests may set it.
inline unsigned mock_core_num = 0;
static inline unsigned get_core_num(void) { return mock_core_num; }
static inline void gpio_set_dir(unsigned, bool) {}
static inline void gpio_pull_down(unsigned pin) { mock_pulls(pin, false, true); }
static inline void gpio_pull_up(unsigned pin) { mock_pulls(pin, true, false); }
static inline void gpio_disable_pulls(unsigned pin) { mock_pulls(pin, false, false); }
// IORDY (GPIO 29) input override, as the firmware last set it: NORMAL means
// IORDY is believed, HIGH means it is ignored. The simulator uses it to see
// a PIO cycle that would wait on a drive holding IORDY low (review R1, R2).
inline unsigned mock_iordy_inover = 0;
static inline void gpio_set_inover(unsigned pin, unsigned v) {
    if (pin == 29) mock_iordy_inover = v;
    if (pin < 48) mock_io.io[pin].ctrl = (mock_io.io[pin].ctrl & ~IO_BANK0_GPIO0_CTRL_INOVER_BITS) | ((v & 3u) << 16);
}
static inline void gpio_put(unsigned pin, bool v) {
    if (v) mock_gpio_out |= (1u << pin); else mock_gpio_out &= ~(1u << pin);
    if (pin == 23) mock_reset_line(v);
}
static inline bool gpio_get(unsigned pin) { return pin == 28 ? mock_intrq() : false; }

// ---- TinyUSB MSC bits used by usb.c --------------------------------------
#include "tusb_config.h"
enum {
    SCSI_SENSE_NONE = 0x00, SCSI_SENSE_NOT_READY = 0x02, SCSI_SENSE_MEDIUM_ERROR = 0x03,
    SCSI_SENSE_ILLEGAL_REQUEST = 0x05, SCSI_SENSE_UNIT_ATTENTION = 0x06,
    SCSI_SENSE_HARDWARE_ERROR = 0x04, SCSI_SENSE_ABORTED_COMMAND = 0x0B,   // used by sat.c
    SCSI_SENSE_RECOVERED_ERROR = 0x01,
};
// TinyUSB's fixed-format sense (msc.h), 18 bytes; sat.c only takes its size.
typedef struct { uint8_t bytes[18]; } scsi_sense_fixed_resp_t;
// TinyUSB calls this after building the fixed sense for REQUEST SENSE.
int32_t tud_msc_request_sense_cb(uint8_t lun, void *buffer, uint16_t bufsize);
// sat.c is C11; the host tests build it as C++.
#ifdef __cplusplus
#define _Static_assert static_assert
#endif
bool tud_msc_set_sense(uint8_t lun, uint8_t key, uint8_t asc, uint8_t ascq);
