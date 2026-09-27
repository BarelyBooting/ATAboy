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
#ifdef CTL_STUB_IDCACHE             // menus.c built against stand-ins for ide.c
inline int stub_idc_puts = 0, stub_idc_stale = 0;
void ide_idcache_put(const uint16_t *, uint8_t) { stub_idc_puts++; }
void ide_idcache_stale(void) { stub_idc_stale++; }
#endif
#endif
