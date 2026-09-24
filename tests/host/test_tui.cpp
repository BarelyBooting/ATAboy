// Captures what menus.c sends to the terminal for a set of screens, so two
// builds of the TUI can be rendered and compared (tui_compare.py).
//   test_tui <out_dir>   writes one <screen>.bin per screen
#include "mock_pico.h"
#include <stdio.h>
#include <string>
#include <vector>

// ---- stand-ins menus.c needs ----------------------------------------------
typedef struct { int unused; } queue_t;
#define PICO_ERROR_TIMEOUT (-1)
static std::string tty;
static inline void queue_add_blocking(queue_t *, const void *c) { tty.push_back(*(const char *)c); }
static inline bool queue_try_remove(queue_t *, void *) { return false; }
static inline absolute_time_t make_timeout_time_us(uint64_t us) { return get_absolute_time() + us; }
static inline bool time_reached(absolute_time_t t) { return get_absolute_time() >= t; }

#include "menus.c"

uint64_t mock_now_ns = 0;
int mock_usb_boot_requests = 0;
uint32_t mock_gpio_out = 0;
MockSio mock_sio;
bool mock_intrq(void) { return false; }
bool tud_msc_set_sense(uint8_t, uint8_t, uint8_t, uint8_t) { return true; }
queue_t cdc_tx_queue, cdc_rx_queue;
volatile bool cdc_connected = true, is_mounted = false, media_changed_waiting = false;
config_t config;
void config_defaults(void) {}
void config_save(void) {}
void ide_select_device(uint8_t) {}
uint8_t ide_probe_devices(void) { return 0; }
void ide_reset_drive(void) {}
bool ide_wait_until_ready(uint32_t) { return true; }
uint8_t ide_read_reg(uint8_t) { return 0x50; }
void ide_set_iordy(bool) {}
bool ide_identify(uint16_t *) { return false; }
bool ide_set_geometry(uint8_t, uint8_t) { return true; }
void ide_read_taskfile(uint8_t tf[8]) { for (int i = 0; i < 8; i++) tf[i] = 0; tf[7] = 0x50; }
uint8_t ide_seek_read_one(uint32_t, bool) { return 0x50; }
#ifdef IDE_FAIL_NONE
static ide_fail_t stub_fail;
void ide_last_failure(ide_fail_t *out) { *out = stub_fail; }
#define SET_STUB_FAILURE() do { stub_fail.kind = IDE_FAIL_ERR; stub_fail.command = 0x20;     stub_fail.status = 0x51; stub_fail.error = 0x40; stub_fail.lba = 33648; stub_fail.done = 0;     stub_fail.count = 8; stub_fail.tf[0] = 8; stub_fail.tf[1] = 5; stub_fail.tf[2] = 0xC5;     stub_fail.tf[3] = 0; stub_fail.tf[4] = 0xA9; stub_fail.drained = true; } while (0)
#else
#define SET_STUB_FAILURE() do {} while (0)
#endif

static void save(const char *dir, const char *name) {
    std::string p = std::string(dir) + "/" + name + ".bin";
    FILE *f = fopen(p.c_str(), "wb");
    fwrite(tty.data(), 1, tty.size(), f);
    fclose(f);
    tty.clear();
}

static void reset_state() {
    memset(&config, 0, sizeof(config));
    config.dev_base = 0xA0;
    hdd_model_raw[0] = 0; hdd_status_text[0] = 0;
    cur_cyls = cur_heads = cur_spt = 0; total_lba_sectors = 0; use_lba_mode = false;
    show_detect_result = false; current_screen = SCREEN_MAIN;
}

int main(int argc, char **argv) {
    const char *dir = argc > 1 ? argv[1] : ".";

    reset_state();
    draw_bios_frame(); update_main_menu();
    save(dir, "main-empty");

    reset_state();
    strcpy(hdd_model_raw, "WDC AC280"); cur_cyls = 980; cur_heads = 10; cur_spt = 17;
    config.main_selected = 1;
    draw_bios_frame(); update_main_menu();
    save(dir, "main-chs");

    reset_state();
    strcpy(hdd_model_raw, "Maxtor 2F040L0"); use_lba_mode = true; total_lba_sectors = 80293248;
    config.dev_base = 0xB0; config.main_selected = 4;
    draw_bios_frame(); update_main_menu();
    save(dir, "main-lba-slave");

    // incremental: the loop redraws the menu without a new frame
    reset_state();
    strcpy(hdd_model_raw, "A MUCH LONGER DRIVE MODEL NAME 12345");
    draw_bios_frame(); update_main_menu();
    strcpy(hdd_model_raw, "SHORT");
    update_main_menu();
    save(dir, "main-incremental");

    reset_state();
    current_screen = SCREEN_FEATURES; config.feat_selected = 2;
    draw_bios_frame(); update_features_menu();
    save(dir, "features");

    reset_state();
    show_detect_result = true; strcpy(hdd_status_text, "\033[91;1mNo drive detected");
    draw_bios_frame(); update_main_menu(); draw_error_box(hdd_status_text, false);
    save(dir, "error-box");

    reset_state();
    strcpy(hdd_model_raw, "WDC AC280"); cur_cyls = 980; cur_heads = 10; cur_spt = 17;
    current_screen = SCREEN_MOUNTED;
    is_mounted = true;              // as the firmware has it on this screen
    draw_bios_frame(); update_main_menu(); draw_confirm_box("Drive mounted!  Press 'U' to Unmount");
    save(dir, "mounted");
    is_mounted = false;

    reset_state();
    current_screen = SCREEN_DEBUG;
    draw_bios_frame(); draw_debug_overlay(); run_debug_taskfile();
    save(dir, "debug-taskfile");

    reset_state();
    current_screen = SCREEN_DEBUG;
    draw_bios_frame(); draw_debug_overlay(); run_debug_errors();
    save(dir, "debug-errors");

    reset_state();
    current_screen = SCREEN_DEBUG;
    SET_STUB_FAILURE();
    draw_bios_frame(); draw_debug_overlay(); run_debug_errors();
    save(dir, "debug-errors-failure");
    return 0;
}
