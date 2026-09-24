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
struct MockSio { MockSioReg gpio_set{true}; MockSioReg gpio_clr{false}; };
extern MockSio mock_sio;
#define sio_hw (&mock_sio)

#define GPIO_IN 0
#define GPIO_OUT 1
enum { GPIO_OVERRIDE_NORMAL = 0, GPIO_OVERRIDE_INVERT, GPIO_OVERRIDE_LOW, GPIO_OVERRIDE_HIGH };
bool mock_intrq(void);
static inline void gpio_init_mask(uint32_t) {}
static inline void gpio_set_dir_out_masked(uint32_t) {}
static inline void gpio_init(unsigned) {}
static inline void gpio_set_dir(unsigned, bool) {}
static inline void gpio_pull_down(unsigned) {}
static inline void gpio_pull_up(unsigned) {}
static inline void gpio_disable_pulls(unsigned) {}
static inline void gpio_set_inover(unsigned, unsigned) {}
static inline void gpio_put(unsigned pin, bool v) {
    if (v) mock_gpio_out |= (1u << pin); else mock_gpio_out &= ~(1u << pin);
}
static inline bool gpio_get(unsigned pin) { return pin == 28 ? mock_intrq() : false; }

// ---- TinyUSB MSC bits used by usb.c --------------------------------------
#include "tusb_config.h"
enum {
    SCSI_SENSE_NONE = 0x00, SCSI_SENSE_NOT_READY = 0x02, SCSI_SENSE_MEDIUM_ERROR = 0x03,
    SCSI_SENSE_ILLEGAL_REQUEST = 0x05, SCSI_SENSE_UNIT_ATTENTION = 0x06,
    SCSI_SENSE_HARDWARE_ERROR = 0x04, SCSI_SENSE_ABORTED_COMMAND = 0x0B,   // used by sat.c
};
// sat.c is C11; the host tests build it as C++.
#ifdef __cplusplus
#define _Static_assert static_assert
#endif
bool tud_msc_set_sense(uint8_t lun, uint8_t key, uint8_t asc, uint8_t ascq);
