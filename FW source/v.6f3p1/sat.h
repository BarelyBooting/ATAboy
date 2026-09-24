#ifndef SAT_H
#define SAT_H

#include <stdint.h>

// Handle ATA PASS-THROUGH(12) 0xA1 and (16) 0x85 from tud_msc_scsi_cb().
// `host_bufsize` is the bufsize TinyUSB passed in, before any clamping.
// Returns the data length on success, or -1 with sense set.
int32_t sat_scsi(uint8_t lun, uint8_t const cdb[16], void *buffer, uint16_t host_bufsize);

// Drop any ATA Status Return descriptor still waiting for REQUEST SENSE.
// Called when another command runs that may set its own sense, so a later
// REQUEST SENSE can never pair that command's sense with SAT registers.
void sat_sense_forget(void);

#endif
