// A whole TUI session through core 1's real loop (core1_entry), with keys on
// the mock clock and the simulated drive, captured byte for byte, so two
// builds can be compared beyond the single screens test_tui.cpp draws
// (tui_compare.py --session). With ATABOY_CTL=1 the ATAboy Control API is
// compiled in, with its real ctl*.c, and idle: nothing on its port. Its hook
// at the loop's top then runs every pass and must change no byte the TUI
// sends; only the banner may differ (the version).
//
//   tui_session <out file>
#include "mock_pico.h"
#include "sim.hpp"
#include "obs.c"
#include "ide.c"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

typedef struct { int unused; } queue_t;
#define PICO_ERROR_TIMEOUT (-1)
struct Arrival { uint64_t at_us; char c; };
static std::vector<Arrival> rx;
static size_t rxi;
static std::string tty;
static const char *out_path;
static void finish() {
    FILE *f = fopen(out_path, "wb");
    fwrite(tty.data(), 1, tty.size(), f);
    fclose(f);
    printf("session: %zu bytes, %zu keys\n", tty.size(), rx.size());
    exit(0);
}
static inline void queue_add_blocking(queue_t *, const void *c) { tty.push_back(*(const char *)c); }
static inline bool queue_try_remove(queue_t *, void *out) {
    if (rxi < rx.size() && get_absolute_time() >= rx[rxi].at_us) { *(char *)out = rx[rxi++].c; return true; }
    if (rxi == rx.size() && get_absolute_time() > rx.back().at_us + 20000000ull) finish();
    return false;
}
static inline absolute_time_t make_timeout_time_us(uint64_t us) { return get_absolute_time() + us; }
static inline bool time_reached(absolute_time_t t) { return get_absolute_time() >= t; }

#include "menus.c"

#if ATABOY_CTL
#include "ctl_json.c"
#include "ctl_sha256.c"
#include "ctl_policy.c"
#include "ctl.c"
#include "ctl_job.c"
// The Control port, closed: no host (the API idle).
bool tud_cdc_n_connected(uint8_t) { return false; }
uint32_t tud_cdc_n_available(uint8_t) { return 0; }
uint32_t tud_cdc_n_read(uint8_t, void *, uint32_t) { return 0; }
void tud_cdc_n_read_flush(uint8_t) {}
uint32_t tud_cdc_n_write(uint8_t, void const *, uint32_t) { return 0; }
uint32_t tud_cdc_n_write_flush(uint8_t) { return 0; }
uint32_t tud_cdc_n_write_available(uint8_t) { return 0; }
bool tud_cdc_n_write_clear(uint8_t) { return true; }
uint64_t get_rand_64(void) { return 1; }
void pico_get_unique_board_id_string(char *o, unsigned n) { snprintf(o, n, "0123456789ABCDEF"); }
void config_flash_copy(config_t *out) { *out = config; }
bool config_loaded_from_flash(void) { return false; }
#endif

bool core1_stack_intact(void) { return true; }
void fault_stop(uint32_t) { printf("FAIL: fault_stop\n"); exit(250); }
#ifdef IDE_FAULT_TRIPWIRE
void ide_trip_stop(void) { printf("FAIL: tripwire\n"); exit(251); }
#endif
uint64_t mock_now_ns = 0;
uint32_t mock_gpio_out = 0;
MockSio mock_sio;
SimDrive sim;
config_t config;
int mock_usb_boot_requests = 0;
queue_t cdc_tx_queue, cdc_rx_queue;
volatile bool cdc_connected = true, is_mounted = false, media_changed_waiting = false;
bool tud_msc_set_sense(uint8_t, uint8_t, uint8_t, uint8_t) { return true; }
void config_save(void) {}
void config_defaults(void) { memset(&config, 0, sizeof config); config.dev_base = 0xA0; config.drive_write_protected = true; }
bool usb_msc_ide_busy(void) { return false; }

int main(int argc, char **argv) {
    out_path = argc > 1 ? argv[1] : "session.bin";
    ide_hw_init();
    sim = SimDrive();
    config_defaults();
    ide_select_device(0xA0);
#if ATABOY_CTL
    ctl_init();
#endif
    // Keys, far enough apart for every screen to settle. Auto Detect (Y at
    // its question, Enter on the picker's first row), Mount and Unmount,
    // Features with its toggles, Debug T, E and I, the Ctrl+G box typed and
    // left with Esc, F10 answered N, Ctrl+L, and the arrows everywhere.
    const char *script[] = {
        "\x1b[B", "\x1b[C", "\x1b[D", "\x1b[A", "\r", "y", "@", "\r", "@",
        "\x1b[B", "\r", "y", "@", "u", "y", "@",
        "\x1b[A", "\x1b[C", "\r", "\x1b[B", "\x1b[B", "\r", "\x1b[B", "\r", "\x1b[B", "\r",
        "t", "@", "e", "@", "i", "@", "\x1b", "\x1b[A", "\x1b[A", "\x1b[A", "\x1b",
        "\x07", "1", "0", "4", "5", "\r", "2", "\r", "\x1b",
        "\x1b[21~", "n", "\x0c", "\x1b[B", "\x1b[B", "\x1b[C", "\x1b[D",
    };
    uint64_t t = 1000000;                           // 1 s after start
    for (const char *k : script) {
        if (!strcmp(k, "@")) { t += 15000000; continue; }      // a long pause (the drive's work)
        for (const char *p = k; *p; p++) { rx.push_back({ t, *p }); t += 1000; }
        t += 400000;
    }
    core1_entry();                                  // ends in finish() once the keys are used
    return 1;
}
