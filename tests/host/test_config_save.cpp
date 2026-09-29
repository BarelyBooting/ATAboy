// config_save() with core 0 storing into config while it runs (palimpsest
// api/SPEC-v1.md 5.5, R-API-1 finding 15). With the ATAboy Control API core
// 0 writes config bytes too (wp.on, pipelined.off), and it keeps running
// until config_save() takes the multicore lockout. The save used to compute
// the CRC over the live config and copy it afterwards, so a store between
// the two saved a sector whose CRC fails (the defaults at the next boot).
//
// The real config.c, with every call it makes before the lockout counted as
// a point at which core 0 may run: memset and memcpy (macros here), and the
// lockout itself. For each point and each store the API makes, the store is
// made there, and the sector written must pass config_valid() and hold
// either the value before the store or the value after it, and every other
// field as it was. Built with ATABOY_CTL=1 only (the old order stays in a
// build without the API, where core 0 never writes config).
//
//   test_config_save        exit status: the number of failed checks
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

static int hook_at = -1, hook_n = 0;
static void (*racer)() = nullptr;
static void hook() { if (hook_n++ == hook_at && racer) racer(); }
static void *hook_memset(void *d, int c, size_t n) { hook(); return memset(d, c, n); }
static void *hook_memcpy(void *d, const void *s, size_t n) { hook(); return memcpy(d, s, n); }
#define memset hook_memset
#define memcpy hook_memcpy
#include "config.c"
#undef memset
#undef memcpy
#if ATABOY_BOARD_JUMPERS
#include "mock_pico.h"
#include "board_jumpers.c"
uint64_t mock_now_ns = 0;
uint32_t mock_gpio_out = 0;
MockSio mock_sio;
volatile uint32_t ctl_epoch;
#endif

#if !ATABOY_CTL
#error "test_config_save is for the build with the Control API (ATABOY_CTL=1)"
#endif

static uint8_t programmed[FLASH_PAGE_SIZE];
static int programs = 0;
void flash_range_erase(uint32_t, size_t) {}
void flash_range_program(uint32_t, const uint8_t *d, size_t n) { memcpy(programmed, d, n); programs++; }
uint32_t save_and_disable_interrupts(void) { return 0; }
void restore_interrupts(uint32_t) {}
void multicore_lockout_start_blocking(void) { hook(); }    // core 0 stops here, no sooner
void multicore_lockout_end_blocking(void) {}

static int failures = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; \
    printf("FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void baseline() {
    memset(&config, 0, sizeof config);
    config.main_selected = 2; config.feat_selected = 5;
    config.drive_write_protected = false; config.auto_mount = false; config.iordy_enabled = true;
    config.intrq_enabled = false; config.use_lba_mode = false;
    config.cyls = 980; config.heads = 10; config.spt = 17; config.lba_sectors = 166600;
    config.dev_base = 0xA0; config.pipelined_reads = true;
}
// Every field but magic and crc, one by one. This used to be a memcmp of the
// whole struct, which also compared its padding bytes (after use_lba_mode,
// before lba_sectors, at the end): copies of a struct need not carry them, so
// with -O1 on g++ 16.2 the same test failed 6 checks at unchanged firmware
// (-O0 passed). The static_assert is here so that a field added to config_t
// is added to this list too.
static_assert(sizeof(config_t) == 32, "config_t changed: add the new field to same_settings()");
static bool same_settings(const config_t &x, const config_t &y) {
    return x.main_selected == y.main_selected && x.feat_selected == y.feat_selected &&
           x.drive_write_protected == y.drive_write_protected && x.auto_mount == y.auto_mount &&
           x.iordy_enabled == y.iordy_enabled && x.intrq_enabled == y.intrq_enabled &&
           x.use_lba_mode == y.use_lba_mode && x.cyls == y.cyls && x.heads == y.heads && x.spt == y.spt &&
           x.lba_sectors == y.lba_sectors && x.dev_base == y.dev_base && x.pipelined_reads == y.pipelined_reads;
}
static void wp_on_to(config_t *k) { k->drive_write_protected = true; }        // ctl.c op_wp_on's store
static void pipelined_off_to(config_t *k) { k->pipelined_reads = false; }     // ctl.c op_pipelined_off's store
static void wp_on() { wp_on_to(&config); }
static void pipelined_off() { pipelined_off_to(&config); }
#if ATABOY_BOARD_JUMPERS
// Stage 3 (palimpsest api/SPEC-v1.md 8.2, mutant X39): JP1 seen fitted forces
// write protect on in RAM, from core 0 (bj_task) or core 1 (the TUI's
// check). The real board_jumpers.c makes that store here, at each point:
// its pads set up (mock_pico.h's register file), the shunt on, one read.
static void jp1_fitted() {
    memset(&bj, 0, sizeof bj);
    mock_pads_reset();
    board_jumpers_init();
    mock_jumper_fitted[0] = true;
    (void)jp1_locked();
    mock_jumper_fitted[0] = false;
}
#endif

int main() {
    // How many points config_save() has, with no store at all.
    baseline();
    hook_at = -1; hook_n = 0; racer = nullptr;
    config_save();
    int points = hook_n;
    config_t clean;
    memcpy(&clean, programmed, sizeof clean);
    CHECK(programs == 1 && config_valid(&clean) && points >= 2, "a save with no store: %d programs, %d points", programs, points);
    struct { const char *name; void (*store)(); void (*to)(config_t *); } stores[] = {
        { "wp.on", wp_on, wp_on_to }, { "pipelined.off", pipelined_off, pipelined_off_to },
#if ATABOY_BOARD_JUMPERS
        { "JP1 fitted", jp1_fitted, wp_on_to },
#endif
    };
    for (auto &st : stores) {
        for (int k = 0; k < points; k++) {
            baseline();
            config_t before = config;
            hook_at = k; hook_n = 0; racer = st.store;
            config_save();
            racer = nullptr; hook_at = -1;
            config_t saved;
            memcpy(&saved, programmed, sizeof saved);
            CHECK(config_valid(&saved), "%s stored at point %d of %d: the saved sector fails its CRC (defaults at the next boot)",
                  st.name, k, points);
            // The value before or after the store, and nothing else changed.
            config_t after = before;
            st.to(&after);
            CHECK(same_settings(saved, before) || same_settings(saved, after),
                  "%s stored at point %d: the sector is neither the settings before nor after it", st.name, k);
        }
    }
#if ATABOY_BOARD_JUMPERS
    printf("config_save with a core-0 store at each of %d points (wp.on, pipelined.off, JP1 fitted)\n", points);
#else
    printf("config_save with a core-0 store at each of %d points (wp.on, pipelined.off)\n", points);
#endif
    printf("%d checks, %d failed\n", checks, failures);
    return failures > 255 ? 255 : failures;
}
