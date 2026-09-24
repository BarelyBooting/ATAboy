// MSC callbacks — called on core 0 from within tud_task().
// NEVER call tud_task() from here.  IDE wait loops use busy_wait only.

#include "tusb.h"
#include "class/msc/msc_device.h"
#include "ide.h"
#include "config.h"
#include <string.h>
#if ATABOY_SAT
#include "sat.h"
#endif

extern volatile bool is_mounted;
extern volatile bool media_changed_waiting;

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------

static uint64_t total_sectors(void) {
    if (!is_mounted) return 0;
    if (config.use_lba_mode) return config.lba_sectors;
    return (uint64_t)config.cyls * config.heads * config.spt;
}

// ---------------------------------------------------------------------------
//  Busy flag for core 1 (review finding L3)
// ---------------------------------------------------------------------------
// True while an MSC callback that may use the IDE bus is running: READ(10),
// WRITE(10), and ATA PASS-THROUGH through the SCSI callback. They run on
// core 0 inside tud_task(). Unmounting (is_mounted = false, on core 1) stops
// new commands from reaching the drive, but not one already running, which
// can take ~41 s on a failing sector. Core 1 checks this before it reboots
// into the ROM bootloader (menus.c, firmware update).
//
// Core 0 sets the flag and then reads is_mounted; core 1 clears is_mounted
// and then reads the flag. A full barrier sits between the write and the
// read on each side, so at least one core sees the other's write: either
// core 1 sees the flag and waits, or the callback sees "not mounted" and
// never touches the drive.
static volatile bool msc_ide_busy = false;

bool usb_msc_ide_busy(void) {
    __atomic_thread_fence(__ATOMIC_SEQ_CST);    // after the caller's is_mounted = false
    return msc_ide_busy;
}

static void msc_busy_begin(void) {
    msc_ide_busy = true;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);    // before the callback reads is_mounted
}

static void msc_busy_end(void) {
    __atomic_thread_fence(__ATOMIC_SEQ_CST);    // after the callback's last bus access
    msc_ide_busy = false;
}

// ---------------------------------------------------------------------------
//  MSC Required Callbacks
// ---------------------------------------------------------------------------

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8],
                        uint8_t product_id[16], uint8_t product_rev[4]) {
    (void)lun;
    const char vid[] = "ATAboy";
    const char pid[] = "Hard Drive";
    const char rev[] = "V0.6";

    memset(vendor_id, ' ', 8);
    memcpy(vendor_id, vid, strlen(vid) < 8 ? strlen(vid) : 8);
    memset(product_id, ' ', 16);
    memcpy(product_id, pid, strlen(pid) < 16 ? strlen(pid) : 16);
    memset(product_rev, ' ', 4);
    memcpy(product_rev, rev, strlen(rev) < 4 ? strlen(rev) : 4);
}

bool tud_msc_is_writable_cb(uint8_t lun) {
    (void)lun;
    return !config.drive_write_protected;
}

bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void)lun;
    return is_mounted;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count,
                         uint16_t *block_size) {
    (void)lun;
    *block_size = 512;
    uint64_t ts = total_sectors();
    *block_count = (ts > 0xFFFFFFFF) ? 0xFFFFFFFF : (uint32_t)ts;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition,
                           bool start, bool load_eject) {
    (void)lun; (void)power_condition; (void)start; (void)load_eject;
    return true;
}

// ---------------------------------------------------------------------------
//  READ10 — block transfer with partial first/last sector handling
// ---------------------------------------------------------------------------

static int32_t read10(uint8_t lun, uint32_t lba, uint32_t offset,
                      void *buffer, uint32_t bufsize) {
    (void)lun;
#if ATABOY_SAT
    sat_sense_forget();     // this command may set sense of its own
#endif
    if (!is_mounted) return -1;

    // TinyUSB derives the block size from the host's CBW (length / count)
    // and never checks it against the 512 we report. With 512-byte blocks
    // every call has offset 0 and a whole number of sectors. Anything else
    // comes from a malformed CBW: the partial-sector code below would then
    // index past its 512-byte buffer, or address the wrong sectors. Refuse.
    if (offset != 0 || (bufsize % 512) != 0) {
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00);
        return -1;
    }

    uint64_t max = total_sectors();
    if (max == 0) return -1;

    uint32_t remaining = bufsize;
    uint8_t *ptr = (uint8_t *)buffer;
    uint32_t cur_lba = lba;

    // Partial first sector (non-zero offset)
    if (offset && remaining > 0 && cur_lba < max) {
        uint8_t temp[512];
        if (ide_read_sectors(cur_lba, 1, temp) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
            return -1;
        }
        uint32_t n = 512 - offset;
        if (n > remaining) n = remaining;
        memcpy(ptr, temp + offset, n);
        ptr += n; remaining -= n; cur_lba++;
    }

    // Aligned middle sectors — single ATA command
    uint32_t aligned = remaining / 512;
    if (aligned > 0 && cur_lba < max) {
        if (cur_lba + aligned > max) aligned = (uint32_t)(max - cur_lba);
        uint32_t got = 0;
        if (ide_read_sectors_partial(cur_lba, aligned, ptr, &got) < 0) {
            // Issue #13: sectors before the failing one are real data, so hand
            // those back and nothing else. TinyUSB then calls again starting at
            // the failing sector, which is read again on its own and fails
            // there if it is still bad. A failed sector is never filled in.
            uint32_t done = (bufsize - remaining) + (got < aligned ? got : 0) * 512;
            if (done > 0) return (int32_t)done;
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
            return -1;
        }
        ptr += aligned * 512; remaining -= aligned * 512; cur_lba += aligned;
    }

    // Partial last sector
    if (remaining > 0 && cur_lba < max) {
        uint8_t temp[512];
        if (ide_read_sectors(cur_lba, 1, temp) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
            return -1;
        }
        memcpy(ptr, temp, remaining);
        ptr += remaining; remaining = 0;
    }

#if ATABOY_STRICT_MEDIA_BOUNDS
    // Never report data for a sector the drive did not supply. Upstream
    // zero-fills whatever lies at or beyond `max` and returns the full length
    // as SUCCESS - which is invisible to any host check that trusts status +
    // byte count, and silently fabricates an image's tail when the configured
    // geometry is smaller than the drive (measured: success + zeros for LBAs up
    // to 2^32-16 on an 80,293,248-sector drive). Instead: hand back only the
    // sectors actually read; TinyUSB calls back for the rest, which then starts
    // at or beyond `max` and fails here.
    if (remaining > 0) {
        uint32_t done = bufsize - remaining;
        if (done == 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x21, 0x00); // LBA out of range
            return -1;
        }
        return (int32_t)done;
    }
#else
    if (remaining > 0) memset(ptr, 0, remaining);
#endif
    return (int32_t)bufsize;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                          void *buffer, uint32_t bufsize) {
    msc_busy_begin();
    int32_t r = read10(lun, lba, offset, buffer, bufsize);
    msc_busy_end();
    return r;
}

// ---------------------------------------------------------------------------
//  WRITE10 — block transfer with partial first/last read-modify-write
// ---------------------------------------------------------------------------

static int32_t write10(uint8_t lun, uint32_t lba, uint32_t offset,
                       uint8_t *buffer, uint32_t bufsize) {
    (void)lun;
#if ATABOY_SAT
    sat_sense_forget();     // this command may set sense of its own
#endif
    if (!is_mounted || config.drive_write_protected) return -1;

    // TinyUSB derives the block size from the host's CBW (length / count)
    // and never checks it against the 512 we report. With 512-byte blocks
    // every call has offset 0 and a whole number of sectors. Anything else
    // comes from a malformed CBW: the partial-sector code below would then
    // index past its 512-byte buffer, or address the wrong sectors. Refuse.
    if (offset != 0 || (bufsize % 512) != 0) {
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00);
        return -1;
    }

    uint64_t max = total_sectors();
    if (max == 0) return -1;

    uint32_t remaining = bufsize;
    uint8_t *ptr = buffer;
    uint32_t cur_lba = lba;

    // Partial first sector — read-modify-write
    if (offset && remaining > 0 && cur_lba < max) {
        uint8_t temp[512];
        if (ide_read_sectors(cur_lba, 1, temp) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
            return -1;
        }
        uint32_t n = 512 - offset;
        if (n > remaining) n = remaining;
        memcpy(temp + offset, ptr, n);
        if (ide_write_sectors(cur_lba, 1, temp) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x03, 0x00);
            return -1;
        }
        ptr += n; remaining -= n; cur_lba++;
    }

    // Aligned middle sectors — single ATA command
    uint32_t aligned = remaining / 512;
    if (aligned > 0 && cur_lba < max) {
        if (cur_lba + aligned > max) aligned = (uint32_t)(max - cur_lba);
        if (ide_write_sectors(cur_lba, aligned, ptr) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x03, 0x00);
            return -1;
        }
        ptr += aligned * 512; remaining -= aligned * 512; cur_lba += aligned;
    }

    // Partial last sector — read-modify-write
    if (remaining > 0 && cur_lba < max) {
        uint8_t temp[512];
        if (ide_read_sectors(cur_lba, 1, temp) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
            return -1;
        }
        memcpy(temp, ptr, remaining);
        if (ide_write_sectors(cur_lba, 1, temp) < 0) {
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x03, 0x00);
            return -1;
        }
        remaining = 0;
    }

    return (int32_t)bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                           uint8_t *buffer, uint32_t bufsize) {
    msc_busy_begin();
    int32_t r = write10(lun, lba, offset, buffer, bufsize);
    msc_busy_end();
    return r;
}

// ---------------------------------------------------------------------------
//  SCSI — Mode Sense + misc
// ---------------------------------------------------------------------------

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16],
                        void *buffer, uint16_t bufsize) {
    // TinyUSB as shipped in pico-sdk 2.2.0 passes the HOST's requested
    // transfer length here, not the size of `buffer` (CFG_TUD_MSC_EP_BUFSIZE).
    // MODE SENSE(10) below memsets `bufsize` bytes, so a request longer than
    // the buffer overwrote RAM past it - TinyUSB's own USB state, is_mounted,
    // the current geometry. Newer TinyUSB clamps this; clamp here as well so
    // the firmware is safe whichever TinyUSB it is built against.
#if ATABOY_SAT
    uint16_t host_bufsize = bufsize;    // sat.c checks it against the CBW
#endif
    if (bufsize > CFG_TUD_MSC_EP_BUFSIZE) bufsize = CFG_TUD_MSC_EP_BUFSIZE;
    uint8_t opcode = scsi_cmd[0];
    uint8_t *buf = (uint8_t *)buffer;

    // Unit Attention on first access after mount/unmount
    if (is_mounted && media_changed_waiting) {
        tud_msc_set_sense(lun, SCSI_SENSE_UNIT_ATTENTION, 0x28, 0);
        media_changed_waiting = false;
        return -1;
    }

    switch (opcode) {
    case 0x1A:  // MODE SENSE (6)
    case 0x5A:  // MODE SENSE (10)
    {
        uint8_t page = scsi_cmd[2] & 0x3F;
        bool is10 = (opcode == 0x5A);
        uint8_t hdr = is10 ? 8 : 4;
        if (bufsize < hdr) return -1;
        memset(buffer, 0, bufsize);
        uint16_t pos = hdr;

        // Page 0x03: Format Device (SPT)
        if ((page == 0x03 || page == 0x3F) && pos + 24 <= bufsize) {
            buf[pos] = 0x03; buf[pos + 1] = 0x16;
            buf[pos + 10] = (uint8_t)(config.spt >> 8);
            buf[pos + 11] = (uint8_t)(config.spt & 0xFF);
            pos += 24;
        }
        // Page 0x04: Rigid Disk Geometry (Cyl/Heads)
        if ((page == 0x04 || page == 0x3F) && pos + 24 <= bufsize) {
            buf[pos] = 0x04; buf[pos + 1] = 0x16;
            buf[pos + 2] = 0;
            buf[pos + 3] = (uint8_t)(config.cyls >> 8);
            buf[pos + 4] = (uint8_t)(config.cyls & 0xFF);
            buf[pos + 5] = config.heads;
            pos += 24;
        }

        if (!is10) {
            buf[0] = (uint8_t)(pos - 1);
            buf[2] = config.drive_write_protected ? 0x80 : 0x00;
        } else {
            uint16_t full = pos - 2;
            buf[0] = (uint8_t)(full >> 8);
            buf[1] = (uint8_t)(full & 0xFF);
            buf[3] = config.drive_write_protected ? 0x80 : 0x00;
        }
        return (int32_t)pos;
    }

    case 0x00: return 0;  // TEST UNIT READY
    case 0x1B: return 0;  // START STOP UNIT
    case 0x35: return 0;  // SYNCHRONIZE CACHE
    case 0x1E: return 0;  // PREVENT ALLOW MEDIUM REMOVAL

#if ATABOY_SAT
    case 0xA1:  // ATA PASS-THROUGH (12)
    case 0x85:  // ATA PASS-THROUGH (16)
    {
        msc_busy_begin();
        int32_t r = sat_scsi(lun, scsi_cmd, buffer, host_bufsize);
        msc_busy_end();
        return r;
    }
#endif

    default:
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0);
        return -1;
    }
}
