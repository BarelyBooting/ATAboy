// ATA PASS-THROUGH (SAT) for ATAboy, read-only. Runs on core 0 inside
// tud_task(), like the rest of usb.c.
//
// What is allowed is decided by sat_policy.c and nothing else. This file
// gathers the inputs the policy needs, runs the one command it returns, and
// reports the result. It has no data-out path: nothing here can send data to
// the drive, whatever the host asks for.

#include "sat.h"
#include "sat_policy.h"
#include "ide.h"
#include "config.h"
#include "tusb.h"
#include "class/msc/msc_device.h"

extern volatile bool is_mounted;

_Static_assert(SAT_MAX_SECTORS * SAT_SECTOR_SIZE <= CFG_TUD_MSC_EP_BUFSIZE,
               "a SAT read must fit the MSC buffer");
_Static_assert(CFG_TUD_MSC_EP_BUFSIZE >= 31, "the MSC buffer must hold a CBW");

int32_t sat_scsi(uint8_t lun, uint8_t const cdb[16], void *buffer, uint16_t host_bufsize) {
    // tud_msc_scsi_cb() is not told the CBW's direction or its full 32-bit
    // length. On the data-in path, TinyUSB (pico-sdk 2.2.0, msc_device.c)
    // received the CBW into this same buffer and has not written to it since,
    // so the CBW is still here. sat_cbw_parse() accepts it only if it matches
    // this command exactly: signature, LUN, all 16 CB bytes, and length.
    //
    // Limit, found in review: on the DATA-OUT path the buffer holds the host's
    // payload, and a host can put a copy of a valid CBW at the start of it. The
    // parse then passes and an allowed READ runs; the host gets no data back.
    // Nothing outside the read-only allowlist can run this way, and no data is
    // ever sent to the drive, but the direction check is not a real guarantee.
    // Fixing it properly needs TinyUSB to pass the CBW direction to the callback.
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
        tud_msc_set_sense(lun, v.sense_key, v.asc, v.ascq);
        return -1;
    }

    uint8_t err = 0;
    switch (ide_sat_pio_in(&tf, (uint8_t *)buffer, &err)) {
    case IDE_SAT_OK:
        return (int32_t)(tf.sectors * SAT_SECTOR_SIZE);
    case IDE_SAT_NOT_ISSUED:
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x00);
        return -1;
    case IDE_SAT_ATA_ERROR:
        // Say what the drive said, so "the drive refused the command" is not
        // reported as a media fault. IDNF is also what a drive returns for a
        // sector past its end; the firmware's own range refusal is 5/21 instead.
        if (err & 0x40)      tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);   // UNC: unrecovered read error
        else if (err & 0x10) tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x14, 0x01);   // IDNF: record not found
        else if (err & 0x04) tud_msc_set_sense(lun, SCSI_SENSE_ABORTED_COMMAND, 0x00, 0x00); // ABRT: command aborted
        else if (err == 0)   tud_msc_set_sense(lun, SCSI_SENSE_HARDWARE_ERROR, 0x44, 0x00); // DF with no error bits: device fault
        else                 tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);   // anything else
        return -1;
    default:    // IDE_SAT_TIMEOUT, IDE_SAT_BAD_END
        tud_msc_set_sense(lun, SCSI_SENSE_ABORTED_COMMAND, 0x00, 0x00);
        return -1;
    }
}
