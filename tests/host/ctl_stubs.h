// For the host tests that build menus.c (or usb.c) without the ATAboy
// Control API itself: the hooks those files call when ATABOY_CTL=1, as
// counters. test_ctl.cpp builds the real ones. Nothing here with ATABOY_CTL
// off, where the firmware sources call none of them.
#pragma once
#if ATABOY_CTL
inline int stub_ctl_idle = 0, stub_ctl_busy = 0, stub_ctl_mounts = 0, stub_ctl_usb_mounts = 0;
inline uint8_t stub_ctl_state = 0xFF;
void ctl_core1_idle(uint8_t s) { stub_ctl_idle++; stub_ctl_state = s; }
void ctl_tui_busy(void) { stub_ctl_busy++; }
void ctl_mount_noted(uint32_t) { stub_ctl_mounts++; }
void ctl_on_usb_mount(void) { stub_ctl_usb_mounts++; }
// Stage 2 (ctl_state.h): the epoch and what the menus report as it changes.
volatile uint32_t ctl_epoch;
volatile uint8_t ctl_wp_last;
inline int stub_ctl_setups = 0;
inline uint8_t stub_ctl_setup_src = 0;
void ctl_setup_noted(uint8_t s) { stub_ctl_setups++; stub_ctl_setup_src = s; }
// Stage 4: the presence row (menus.c update_main_menu). A test sets the text
// to draw the row; nullptr: nothing waits.
inline const char *stub_presence = nullptr;
bool ctl_presence_text(char *o, unsigned n) {
    if (!stub_presence) { o[0] = 0; return false; }
    snprintf(o, n, "%s", stub_presence);
    return true;
}
#ifdef CTL_STUB_IDCACHE             // menus.c built against stand-ins for ide.c
inline int stub_idc_puts = 0, stub_idc_stale = 0;
void ide_idcache_put(const uint16_t *, uint8_t) { stub_idc_puts++; }
void ide_idcache_stale(void) { stub_idc_stale++; }
inline int stub_idc_clears = 0;
void ide_idcache_clear(uint8_t) { stub_idc_clears++; }
// identify.forbid is never set in these tests (test_ctl drives it).
bool ide_identify_forbidden(void) { return false; }
bool ide_identify_barred(void) { return ide_mchs_no_identify(); }
#endif
#endif
