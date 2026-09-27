#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#define CFG_TUSB_RHPORT0_MODE    OPT_MODE_DEVICE
#if ATABOY_CTL
// The ATAboy Control API (palimpsest api/SPEC-v1.md 2.1): a second CDC
// function, instance 1, after MSC. Instance 0 stays the TUI. The buffer
// sizes below are per instance.
#define CFG_TUD_CDC              2
#else
#define CFG_TUD_CDC              1
#endif
#define CFG_TUD_MSC              1
// F1 stage 2 (2026-09-26): 16 KB, from 4 KB. TinyUSB 0.18 casts a
// transfer's length to 16 bits (usbd_edpt_xfer in msc_device.c), so 64 KB
// would send nothing; 16 KB is a quarter of palimpsest imagelba's 128-sector
// READ(10), so each of its commands is four callbacks and three of them can
// be read ahead while the one before goes out over USB (32 KB would leave
// one). Pipelined Reads Off still takes 4 KB a callback (usb.c).
#define CFG_TUD_MSC_EP_BUFSIZE   16384
#define CFG_TUD_CDC_RX_BUFSIZE   256
#define CFG_TUD_CDC_TX_BUFSIZE   512

#endif
