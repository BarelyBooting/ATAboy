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
static void wp_on_to(config_t *k) { k->drive_write_protected = true; }        // ctl.c op_wp_on's store
static void pipelined_off_to(config_t *k) { k->pipelined_reads = false; }     // ctl.c op_pipelined_off's store
static void wp_on() { wp_on_to(&config); }
static void pipelined_off() { pipelined_off_to(&config); }

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
        { "wp.on", wp_on, wp_on_to }, { "pipelined.off", pipelined_off, pipelined_off_to } };
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
            config_t sv = saved; sv.magic = 0; sv.crc = 0;
            config_t b = before, a = after;
            b.magic = a.magic = 0; b.crc = a.crc = 0;
            CHECK(memcmp(&sv, &b, sizeof sv) == 0 || memcmp(&sv, &a, sizeof sv) == 0,
                  "%s stored at point %d: the sector is neither the settings before nor after it", st.name, k);
        }
    }
    printf("config_save with a core-0 store at each of %d points (wp.on, pipelined.off)\n", points);
    printf("%d checks, %d failed\n", checks, failures);
    return failures > 255 ? 255 : failures;
}
