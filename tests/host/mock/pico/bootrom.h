// Host test stand-in for the Pico SDK header of the same name.
#pragma once
#include "mock_pico.h"
// The real call reboots into the ROM bootloader and does not return. The
// host tests record the request instead.
extern int mock_usb_boot_requests;
static inline void reset_usb_boot(uint32_t, uint32_t) { mock_usb_boot_requests++; }
