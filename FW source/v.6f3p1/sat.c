// ATA PASS-THROUGH (SAT) for ATAboy, read-only. Runs on core 0 inside
// tud_task(), like the rest of usb.c.
//
// What is allowed is decided by sat_policy.c and nothing else. This file
// gathers the inputs the policy needs, runs the one command it returns, and
// reports the result. It has no data-out path: nothing here can send data to
// the drive, whatever the host asks for.
//
// Returning the drive's registers (SAT ATA Status Return descriptor)
// ------------------------------------------------------------------
// A command with CK_COND=1, and any command the drive ends with an error,
// gets CHECK CONDITION with descriptor-format sense (response code 72h)
// carrying one ATA Status Return descriptor (code 09h, SAT-2 12.2.2.7):
//
//   byte  0      72h (current, descriptor format)
//   byte  1      sense key            2  ASC            3  ASCQ
//   bytes 4..6   0                    7  additional length, 0Eh
//   byte  8      09h (descriptor code) 9 0Ch (additional length)
//   byte 10      bit 0 EXTEND: 1 when bytes 12, 14, 16, 18 hold HOB values
//   byte 11      ERROR
//   byte 12/13   COUNT 15:8 / 7:0
//   byte 14/15   LBA 31:24 / 7:0      (HOB LBA low / LBA low)
//   byte 16/17   LBA 39:32 / 15:8     (HOB LBA mid / LBA mid)
//   byte 18/19   LBA 47:40 / 23:16    (HOB LBA high / LBA high)
//   byte 20      DEVICE               21  STATUS
//
// CK_COND=1 and no error: RECOVERED ERROR, 00/1D (ATA pass through
// information available). An ATA error: the sense key and ASC say what the
// drive said (UNC, IDNF, ABRT, DF), as before, and the descriptor is added.
// A timeout or a bad ending has no descriptor: the command was aborted with a
// soft reset, which replaces the registers, so there is nothing true to say.
//
// Delivery. TinyUSB (pico-sdk 2.2.0, msc_device.c) answers REQUEST SENSE
// itself with 18 bytes of fixed-format sense built from tud_msc_set_sense(),
// then calls tud_msc_request_sense_cb() with that buffer, and sends however
// many bytes the callback returns, cut to the host's allocation length. So
// the callback below replaces the fixed sense with the 22-byte descriptor
// form, but only when the sense TinyUSB is about to send is still the one
// this file set. A host that asks for fewer than 22 bytes gets the first
// part: at 18, LBA 23:16, DEVICE and STATUS are lost. The callback is not told
// the allocation length or the DESC bit, so it cannot fall back to the fixed
// format that SAT-3 allows instead; that is the host's side to size.

#include "sat.h"
#include "sat_policy.h"
#include "ide.h"
#include "config.h"
#include "tusb.h"
#include "class/msc/msc_device.h"
#include <string.h>

extern volatile bool is_mounted;

_Static_assert(SAT_MAX_SECTORS * SAT_SECTOR_SIZE <= CFG_TUD_MSC_EP_BUFSIZE,
               "a SAT read must fit the MSC buffer");
_Static_assert(CFG_TUD_MSC_EP_BUFSIZE >= 31, "the MSC buffer must hold a CBW");
_Static_assert(sizeof(scsi_sense_fixed_resp_t) == 18, "TinyUSB's fixed sense is 18 bytes");

#define SAT_SENSE_LEN  22

// The descriptor sense waiting for the host's REQUEST SENSE, and the sense
// key / ASC / ASCQ it was set with, to recognise that it is still current.
static struct {
    bool    valid;
    uint8_t lun, key, asc, ascq;
    uint8_t sense[SAT_SENSE_LEN];
} pending;

void sat_sense_forget(void) {
    pending.valid = false;
}

static void sense_plain(uint8_t lun, uint8_t key, uint8_t asc, uint8_t ascq) {
    pending.valid = false;
    tud_msc_set_sense(lun, key, asc, ascq);
}

static void sense_with_regs(uint8_t lun, uint8_t key, uint8_t asc, uint8_t ascq,
                            const ide_sat_regs_t *r) {
    uint8_t *d = pending.sense;
    memset(d, 0, SAT_SENSE_LEN);
    d[0]  = 0x72;
    d[1]  = key & 0x0F;
    d[2]  = asc;
    d[3]  = ascq;
    d[7]  = SAT_SENSE_LEN - 8;
    d[8]  = 0x09;                   // ATA Status Return
    d[9]  = 0x0C;
    d[10] = r->hob ? 0x01 : 0x00;   // EXTEND
    d[11] = r->error;
    d[12] = r->hob_count;
    d[13] = r->count;
    d[14] = r->hob_lba_low;
    d[15] = r->lba_low;
    d[16] = r->hob_lba_mid;
    d[17] = r->lba_mid;
    d[18] = r->hob_lba_high;
    d[19] = r->lba_high;
    d[20] = r->device;
    d[21] = r->status;
    pending.lun = lun; pending.key = key; pending.asc = asc; pending.ascq = ascq;
    pending.valid = true;
    tud_msc_set_sense(lun, key, asc, ascq);
}

int32_t tud_msc_request_sense_cb(uint8_t lun, void *buffer, uint16_t bufsize) {
    uint8_t *b = (uint8_t *)buffer;
    const int32_t fixed = (int32_t)sizeof(scsi_sense_fixed_resp_t);   // what TinyUSB just built
    if (!pending.valid) return fixed;
    // Only if the sense being reported is still ours. Anything that set sense
    // since (a failed READ(10), say) wins, and the descriptor is dropped.
    bool ours = pending.lun == lun && (b[2] & 0x0F) == pending.key &&
                b[12] == pending.asc && b[13] == pending.ascq;
    pending.valid = false;          // one delivery: TinyUSB clears its sense next
    if (!ours || bufsize < SAT_SENSE_LEN) return fixed;
    memcpy(b, pending.sense, SAT_SENSE_LEN);
    return SAT_SENSE_LEN;
}

int32_t sat_scsi(uint8_t lun, uint8_t const cdb[16], void *buffer, uint16_t host_bufsize) {
    sat_sense_forget();

    // tud_msc_scsi_cb() is not told the CBW's direction or its full 32-bit
    // length. On the data-in path, TinyUSB (pico-sdk 2.2.0, msc_device.c)
    // received the CBW into this same buffer and has not written to it since,
    // so the CBW is still here. sat_cbw_parse() accepts it only if it matches
    // this command exactly: signature, LUN, all 16 CB bytes, and length.
    //
    // Limit, found in review (M1): on the DATA-OUT path the buffer holds the
    // host's payload, and a host can put a copy of a valid CBW at the start of
    // it. The parse then passes and an allowed READ runs; the host gets no
    // data back. Nothing outside the read-only allowlist can run this way, and
    // no data is ever sent to the drive, but the direction check is not a real
    // guarantee. Fixing it properly needs TinyUSB to pass the CBW direction to
    // the callback. The non-data rows cannot be reached this way at all: they
    // need a CBW length of 0, and on the data-out path TinyUSB passes the real
    // length (1 to 4096) as bufsize, which the copy's length must match.
    // If a later TinyUSB stops leaving the CBW here, every SAT command is
    // refused; it fails closed, not open.
    sat_input_t in = {0};
    sat_cbw_t cbw;
    in.cdb = cdb;
    if (sat_cbw_parse((const uint8_t *)buffer, lun, cdb, host_bufsize, &cbw)) {
        in.cdb_len  = cbw.cb_len;
        in.dir_in   = cbw.dir_in;
        in.xfer_len = cbw.xfer_len;
    }
    in.mounted  = is_mounted;
    in.lba_mode = config.use_lba_mode;

    sat_taskfile_t tf;
    sat_verdict_t v = sat_policy_check(&in, &tf);
    if (!v.allow) {
        sense_plain(lun, v.sense_key, v.asc, v.ascq);
        return -1;
    }

    ide_sat_regs_t regs;
    bool nondata = tf.protocol == SAT_PROTO_NON_DATA;
    int r = nondata ? ide_sat_nondata(&tf, &regs)
                    : ide_sat_pio_in(&tf, (uint8_t *)buffer, &regs);
    switch (r) {
    case IDE_SAT_OK:
        if (tf.ck_cond) {           // non-data only; the policy refuses it with data
            sense_with_regs(lun, SCSI_SENSE_RECOVERED_ERROR, 0x00, 0x1D, &regs);
            return -1;
        }
        return nondata ? 0 : (int32_t)(tf.sectors * SAT_SECTOR_SIZE);
    case IDE_SAT_NOT_ISSUED:
        sense_plain(lun, SCSI_SENSE_NOT_READY, 0x04, 0x00);
        return -1;
    case IDE_SAT_ATA_ERROR: {
        // Say what the drive said, so "the drive refused the command" is not
        // reported as a media fault. IDNF is also what a drive returns for a
        // sector past its end; the firmware's own range refusal is 5/21 instead.
        // The descriptor carries the drive's own registers, including, for a
        // read or verify, the LBA of the sector that failed.
        uint8_t err = regs.error;
        if (err & 0x40)      sense_with_regs(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00, &regs);   // UNC: unrecovered read error
        else if (err & 0x10) sense_with_regs(lun, SCSI_SENSE_MEDIUM_ERROR, 0x14, 0x01, &regs);   // IDNF: record not found
        else if (err & 0x04) sense_with_regs(lun, SCSI_SENSE_ABORTED_COMMAND, 0x00, 0x00, &regs); // ABRT: command aborted
        else if (err == 0)   sense_with_regs(lun, SCSI_SENSE_HARDWARE_ERROR, 0x44, 0x00, &regs); // DF with no error bits: device fault
        else                 sense_with_regs(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00, &regs);   // anything else
        return -1;
    }
    default:    // IDE_SAT_TIMEOUT, IDE_SAT_BAD_END: aborted with SRST, registers gone
        sense_plain(lun, SCSI_SENSE_ABORTED_COMMAND, 0x00, 0x00);
        return -1;
    }
}
