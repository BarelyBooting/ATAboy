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
// can take up to the host command's whole budget (IDE_HOST_BUDGET_MS, 20 s,
// ide.h) plus a small fixed margin. Core 1 checks this before it reboots
// into the ROM bootloader (menus.c, firmware update), and before Auto Detect
// and the debug commands use the bus (review L-5).
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
//  Where one host command ends and the next begins (review M-1)
// ---------------------------------------------------------------------------
// ide.c gives each host SCSI command one time budget, IDE_HOST_BUDGET_MS, so
// it has to be told which callbacks belong to one command. What TinyUSB 0.18
// (pico-sdk 2.2.0, src/class/msc/msc_device.c) does:
//  - A READ(10) or WRITE(10) is handed over in chunks of at most
//    CFG_TUD_MSC_EP_BUFSIZE bytes, one tud_msc_read10_cb or
//    tud_msc_write10_cb call per chunk, each with lba = the CDB's LBA plus
//    the bytes done so far / 512 (proc_read10_cmd, proc_write10_new_data).
//    A negative return fails the command there (fail_scsi_op); nothing more
//    is asked for it.
//  - Any other command is one tud_msc_scsi_cb call (or none, for the ones
//    TinyUSB answers itself).
//  - When a command's CSW has gone out, passed or failed, TinyUSB calls
//    tud_msc_read10_complete_cb, tud_msc_write10_complete_cb or
//    tud_msc_scsi_complete_cb (mscd_xfer_cb, MSC_STAGE_STATUS_SENT). There is
//    no callback when a command starts.
// So a READ(10) or WRITE(10) callback carries on the command in progress
// only if one is in progress (no complete callback since, and no negative
// return), in the same direction, and it starts exactly where the bytes the
// last callback returned ended. A real continuation always meets all three,
// so it can never be taken for a new command and given a fresh budget.
// The other way: a command that ends without its CSW, because the host reset
// the device, has no complete callback. After a USB reset the host sends
// SET_CONFIGURATION again, which TinyUSB reports as tud_mount_cb, and that
// ends it here too. A bulk-only reset has no callback of any kind; the next
// READ(10) or WRITE(10) is then still new unless it starts exactly where the
// abandoned one stopped, in the same direction, in which case it inherits
// what is left of that budget (perhaps nothing, so it fails at once and the
// host's retry starts clean). A host only resets the device when it gave up
// on a command first, which is what the budget is there to prevent.
static struct {
    bool     open;          // a READ(10) or WRITE(10) in progress
    bool     write;         // ...which of the two
    uint32_t next_lba;      // where its next callback starts
} rw_cmd;

// Nothing is timed while no drive is mounted (review of 0.6f3p7, LOW): the
// callback then refuses without touching the bus, and core 1 may be using
// the bus meanwhile (Auto Detect, auto-mount) with waits that ide.c would
// cut to this command's time while it is entered. Called after
// msc_busy_begin(), so this read of is_mounted is the one the busy flag's
// protocol pairs with (above).
static void host_cmd_enter(bool rw, bool write, uint32_t lba) {
    if (!is_mounted) return;
    if (!rw || !rw_cmd.open || rw_cmd.write != write || lba != rw_cmd.next_lba)
        ide_host_cmd_begin();
    rw_cmd.open = rw;
    rw_cmd.write = write;
    ide_host_cmd_enter();
}

static void host_cmd_leave(uint32_t lba, int32_t r) {
    ide_host_cmd_leave();
    if (r < 0) rw_cmd.open = false;             // TinyUSB fails the command here
    else rw_cmd.next_lba = lba + (uint32_t)r / 512;
}

// ---------------------------------------------------------------------------
//  Fixed-format sense with ILI and INFORMATION (0.6f3p9, READ LONG)
// ---------------------------------------------------------------------------
// tud_msc_set_sense() only keeps key, ASC and ASCQ. TinyUSB (pico-sdk 2.2.0,
// msc_device.c) answers REQUEST SENSE itself with 18 bytes of fixed sense,
// VALID already set, then calls tud_msc_request_sense_cb() with them. READ
// LONG's length refusal needs ILI and the INFORMATION field as well (SBC), so
// they wait here and the callback adds them, only if the sense TinyUSB is
// about to send is still the one set with them (as sat.c does for its
// descriptor). Forgotten when the next command starts.
static struct {
    bool    valid;
    uint8_t lun, key, asc, ascq;
    uint32_t info;
} sense_ili;

static void usb_sense_forget(void) { sense_ili.valid = false; }

static void sense_ili_set(uint8_t lun, uint8_t key, uint8_t asc, uint8_t ascq, uint32_t info) {
    sense_ili.lun = lun; sense_ili.key = key; sense_ili.asc = asc; sense_ili.ascq = ascq;
    sense_ili.info = info;
    sense_ili.valid = true;
    tud_msc_set_sense(lun, key, asc, ascq);
}

// Called by the REQUEST SENSE callback (sat.c in a SAT build, below in one
// without) over TinyUSB's 18 bytes of fixed sense.
void usb_sense_ili_apply(uint8_t lun, uint8_t *b, uint16_t bufsize) {
    if (!sense_ili.valid) return;
    bool ours = sense_ili.lun == lun && (b[2] & 0x0F) == sense_ili.key &&
                b[12] == sense_ili.asc && b[13] == sense_ili.ascq;
    sense_ili.valid = false;        // one delivery: TinyUSB clears its sense next
    if (!ours || bufsize < 18) return;
    b[0] |= 0x80;                   // VALID: the INFORMATION field means something
    b[2] |= 0x20;                   // ILI
    b[3] = (uint8_t)(sense_ili.info >> 24);
    b[4] = (uint8_t)(sense_ili.info >> 16);
    b[5] = (uint8_t)(sense_ili.info >> 8);
    b[6] = (uint8_t)sense_ili.info;
}

#if !ATABOY_SAT
int32_t tud_msc_request_sense_cb(uint8_t lun, void *buffer, uint16_t bufsize) {
    usb_sense_ili_apply(lun, (uint8_t *)buffer, bufsize);
    return 18;                      // TinyUSB's fixed sense, as built
}
#endif

void tud_msc_read10_complete_cb(uint8_t lun) { (void)lun; rw_cmd.open = false; }
void tud_msc_write10_complete_cb(uint8_t lun) { (void)lun; rw_cmd.open = false; }
void tud_msc_scsi_complete_cb(uint8_t lun, uint8_t const scsi_cmd[16]) {
    (void)lun; (void)scsi_cmd;
    rw_cmd.open = false;
}
void tud_mount_cb(void) { rw_cmd.open = false; }

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
    usb_sense_forget();
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
            // That call is part of the same host command and shares its time
            // (review M-1): with too little left, it fails without sending
            // anything, and the host still has the good sectors.
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
    host_cmd_enter(true, false, lba);
    int32_t r = read10(lun, lba, offset, buffer, bufsize);
    host_cmd_leave(lba, r);
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
    usb_sense_forget();
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
    host_cmd_enter(true, true, lba);
    int32_t r = write10(lun, lba, offset, buffer, bufsize);
    host_cmd_leave(lba, r);
    msc_busy_end();
    return r;
}

// ---------------------------------------------------------------------------
//  READ BUFFER(10): the salvage capture, and nothing else (0.6f3p9)
// ---------------------------------------------------------------------------
// The only way the bytes a drive offered with a failed sector (ide.h, salvage
// capture) leave the firmware. Never through READ(10), ATA PASS-THROUGH or an
// image. Accepted exactly as:
//   CDB 3Ch, byte 1 bits 4:0 (MODE) 02h (data), byte 2 (BUFFER ID) 5Ah,
//   bytes 3-5 (BUFFER OFFSET) 0, bytes 6-8 ALLOCATION LENGTH (big-endian).
// Anything else: CHECK CONDITION, ILLEGAL REQUEST, 24h/00h (invalid field in
// CDB), no data. The answer is SALV_LEN bytes, cut to the allocation length:
//   0   "ATBSALV1"
//   8   valid: 1 a capture exists, 0 none (then everything after is 0)
//   9   ATA status at the failure     10  ATA error     11  ATA command
//   12  u32 LE: the failed sector, as the host numbers it
//   16  u32 LE: sequence, 1 for the first capture since power-up, +1 each
//   20  u32 LE: ms since boot at the capture
//   24..31  0
//   32..543 the 512 bytes as the drive offered them
// It does not touch the drive, and works mounted or not.
#define SALV_BUFFER_ID  0x5A
#define SALV_LEN        (32 + IDE_SALVAGE_BYTES)
_Static_assert(SALV_LEN <= CFG_TUD_MSC_EP_BUFSIZE, "the salvage answer must fit the MSC buffer");

static void put_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static int32_t read_buffer(uint8_t lun, uint8_t const cdb[16], uint8_t *buf, uint16_t bufsize) {
    if ((cdb[1] & 0x1F) != 0x02 || cdb[2] != SALV_BUFFER_ID || cdb[3] || cdb[4] || cdb[5]) {
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00);
        return -1;
    }
    uint32_t alloc = ((uint32_t)cdb[6] << 16) | ((uint32_t)cdb[7] << 8) | cdb[8];
    ide_salvage_t sv;
    ide_salvage_get(&sv);
    uint8_t r[SALV_LEN];
    memset(r, 0, sizeof r);
    memcpy(r, "ATBSALV1", 8);
    if (sv.valid) {
        r[8]  = 1;
        r[9]  = sv.status;
        r[10] = sv.error;
        r[11] = sv.command;
        put_le32(r + 12, sv.lba);
        put_le32(r + 16, sv.seq);
        put_le32(r + 20, sv.ms);
        memcpy(r + 32, sv.data, IDE_SALVAGE_BYTES);
    }
    uint32_t n = alloc < SALV_LEN ? alloc : SALV_LEN;
    if (n > bufsize) n = bufsize;           // never past the host's transfer length
    memcpy(buf, r, n);
    return (int32_t)n;
}

// ---------------------------------------------------------------------------
//  READ LONG(10) -> ATA READ LONG WITHOUT RETRIES (0.6f3p9)
// ---------------------------------------------------------------------------
// CDB 3Eh: byte 1 bit 1 CORRCT, bit 2 PBLOCK; bytes 2-5 LBA; bytes 7-8 BYTE
// TRANSFER LENGTH (both big-endian). One sector: its 512 bytes, then the ECC
// bytes the drive's IDENTIFY word 22 counts (ide.c, ide_read_long). Checked in
// this order, and nothing is sent to the drive for any refusal:
//   not mounted                              NOT READY 2/04/00
//   CORRCT or PBLOCK set                     ILLEGAL REQUEST 5/24/00 (only
//                                            uncorrected data is ever asked for)
//   geometry from Ctrl+G (no IDENTIFY)       ILLEGAL REQUEST 5/24/00
//   no IDENTIFY word 22, or it is not 1..64  ILLEGAL REQUEST 5/20/00 (the drive
//                                            does not say it has READ LONG)
//   48-bit mount (no 48-bit READ LONG)       ILLEGAL REQUEST 5/20/00
//   BYTE TRANSFER LENGTH 0                   GOOD, no data (SBC: not an error)
//   BYTE TRANSFER LENGTH not 512 + word 22   ILLEGAL REQUEST 5/24/00 with ILI,
//                                            INFORMATION = requested - actual
//   host transfer length under 512 + word 22 ILLEGAL REQUEST 5/24/00
//   LBA at or past the end of the mount      ILLEGAL REQUEST 5/21/00 (as READ(10))
// Then the drive:
//   the data and ECC bytes                   GOOD, 512 + word 22 bytes
//   ERR from the drive                       MEDIUM ERROR 3/11/00 (as READ(10))
//   not sent (a reset still pending, no      NOT READY 2/04/00 (as SAT)
//     time, not ready, no geometry)
//   no data in time, or not exactly 512 +    ABORTED COMMAND 0B/00/00 (as SAT)
//     word 22 bytes on offer
static int32_t read_long10(uint8_t lun, uint8_t const cdb[16], uint8_t *buf, uint16_t host_bufsize) {
#if ATABOY_SAT
    sat_sense_forget();
#endif
    if (!is_mounted) { tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x00); return -1; }
    if (cdb[1] & 0x06) { tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00); return -1; }
    if (ide_manual_chs_active()) { tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00); return -1; }
    uint16_t w22 = 0;
    if (!ide_read_long_word22(&w22) || w22 < 1 || w22 > IDE_LONG_ECC_MAX ||
        (config.use_lba_mode && config.lba_sectors > 0x0FFFFFFF)) {
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
        return -1;
    }
    uint32_t lba = ((uint32_t)cdb[2] << 24) | ((uint32_t)cdb[3] << 16) | ((uint32_t)cdb[4] << 8) | cdb[5];
    uint32_t len = ((uint32_t)cdb[7] << 8) | cdb[8];
    uint32_t want = 512u + w22;
    if (len == 0) return 0;
    if (len != want) {
        sense_ili_set(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00, len - want);   // two's complement if short
        return -1;
    }
    if (host_bufsize < want) { tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00); return -1; }
    if ((uint64_t)lba >= total_sectors()) { tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x21, 0x00); return -1; }
    switch (ide_read_long(lba, (uint8_t)w22, buf)) {
    case IDE_LONG_OK:       return (int32_t)want;
    case IDE_LONG_ERR:      tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00); return -1;
    case IDE_LONG_NOT_SENT: tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x00); return -1;
    default:                tud_msc_set_sense(lun, SCSI_SENSE_ABORTED_COMMAND, 0x00, 0x00); return -1;
    }
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
    uint16_t host_bufsize = bufsize;    // sat.c checks it against the CBW; READ LONG too
    if (bufsize > CFG_TUD_MSC_EP_BUFSIZE) bufsize = CFG_TUD_MSC_EP_BUFSIZE;
    usb_sense_forget();                 // this command may set sense of its own
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

    case 0x3C:  // READ BUFFER(10): the salvage capture only; no drive access
#if ATABOY_SAT
        sat_sense_forget();
#endif
        return read_buffer(lun, scsi_cmd, buf, bufsize);

    case 0x3E:  // READ LONG(10) -> ATA READ LONG WITHOUT RETRIES
    {
        msc_busy_begin();
        host_cmd_enter(false, false, 0);        // one callback, one command
        int32_t r = read_long10(lun, scsi_cmd, buf, host_bufsize);
        host_cmd_leave(0, r);
        msc_busy_end();
        return r;
    }

#if ATABOY_SAT
    case 0xA1:  // ATA PASS-THROUGH (12)
    case 0x85:  // ATA PASS-THROUGH (16)
    {
        msc_busy_begin();
        host_cmd_enter(false, false, 0);        // one callback, one command
        int32_t r = sat_scsi(lun, scsi_cmd, buffer, host_bufsize);
        host_cmd_leave(0, r);
        msc_busy_end();
        return r;
    }
#endif

    default:
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0);
        return -1;
    }
}
