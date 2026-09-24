// Restricted SAT (ATA PASS-THROUGH) policy for ATAboy: the allowlist.
//
// Pure code. No hardware, TinyUSB or Pico SDK dependencies, so the same file
// builds into the firmware and into the host test in test/.
//
// The policy is a closed list. A CDB is allowed only if it is one of the
// shapes below, byte for byte; everything else is refused before any ATA
// register is written.
//
//   ATA command              CDB form            PROTOCOL  count  addressing
//   0xEC IDENTIFY DEVICE     A1 (12) or 85 (16)  4         1      none
//   0x20 READ SECTORS        A1 (12) or 85 (16)  4         1..8   LBA28
//   0x21 READ SECTORS (NR)   A1 (12) or 85 (16)  4         1..8   LBA28
//   0x24 READ SECTORS EXT    85 (16), EXTEND=1   4         1..8   LBA48
//
// Common to every allowed CDB: CB length exactly 12 (A1) or 16 (85),
// MULTIPLE_COUNT=0, OFF_LINE=0, CK_COND=0, T_TYPE=0, T_DIR=1 (from device),
// BYTE_BLOCK=1, T_LENGTH=2 (count field), FEATURES=0, CONTROL=0, reserved
// bytes 0, HOB bytes 0 unless EXTEND=1, and the CBW must be data-in with a
// transfer length of exactly count * 512.
//
// DEVICE byte: the DEV bit (4) must be 0; the bridge always talks to the drive
// it selected. Bits 7 and 5 are obsolete and ignored. IDENTIFY needs bits 3..0
// clear. READ SECTORS needs LBA=1 (bit 6) and carries LBA 27:24 in bits 3..0.
// READ SECTORS EXT needs LBA=1 and bits 3..0 clear.
//
// LBA + count must not pass 2^28 (0x20/0x21) or 2^48 (0x24).
//
// State: the drive must be mounted (else NOT READY), and reads are allowed only
// when it was set up in LBA mode. IDENTIFY works in either mode.
//
// SAT reads are NOT clipped to the configured geometry (unlike READ(10)).
// They address the drive, and the drive's own IDNF is what stops a read past
// its end; that failure is passed on, and nothing on the SAT path zero-fills
// or returns data the drive did not send. A geometry smaller than the drive
// therefore cannot hide sectors from a tool that sized itself by IDENTIFY.

#ifndef SAT_POLICY_H
#define SAT_POLICY_H

#include <stdint.h>
#include <stdbool.h>

// One command's data must fit the MSC buffer (4096 bytes, tusb_config.h).
#define SAT_SECTOR_SIZE   512u
#define SAT_MAX_SECTORS   8u

// SCSI sense values used for refusals.
#define SAT_SK_NOT_READY        0x02
#define SAT_SK_ILLEGAL_REQUEST  0x05
#define SAT_ASC_NOT_READY       0x04    // 04/00 logical unit not ready
#define SAT_ASC_INVALID_OPCODE  0x20    // 20/00 invalid command operation code
#define SAT_ASC_LBA_RANGE       0x21    // 21/00 logical block address out of range
#define SAT_ASC_INVALID_FIELD   0x24    // 24/00 invalid field in CDB

// What the caller knows about the command, apart from the CDB itself.
typedef struct {
    const uint8_t *cdb;       // 16 bytes (the CBW's CB field)
    uint8_t  cdb_len;         // bCBWCBLength
    bool     dir_in;          // CBW direction is device-to-host
    uint32_t xfer_len;        // dCBWDataTransferLength
    bool     mounted;         // core 0 owns the IDE bus (is_mounted)
    bool     lba_mode;        // the drive was set up in LBA mode
} sat_input_t;

// The ATA task file to issue for an allowed CDB. `device` never carries the
// DEV bit or the obsolete bits 7 and 5; the executor adds the selected device.
typedef struct {
    uint8_t  command;
    uint8_t  feature, count, lba_low, lba_mid, lba_high;
    uint8_t  hob_feature, hob_count, hob_lba_low, hob_lba_mid, hob_lba_high;
    uint8_t  device;          // 0x00 (IDENTIFY) or 0x40 | LBA 27:24
    bool     ext;             // write the HOB registers (48-bit command)
    uint8_t  sectors;         // 512-byte PIO data-in blocks to transfer
} sat_taskfile_t;

typedef struct {
    bool     allow;
    uint8_t  sense_key, asc, ascq;   // valid when !allow
} sat_verdict_t;

// Decide one command. On allow, *tf holds the task file to send; on refuse
// *tf is zeroed. Never touches anything but its arguments.
sat_verdict_t sat_policy_check(const sat_input_t *in, sat_taskfile_t *tf);

// The fields of a USB mass storage Command Block Wrapper that the policy needs.
typedef struct {
    bool     dir_in;
    uint32_t xfer_len;
    uint8_t  cb_len;
} sat_cbw_t;

// Parse a 31-byte CBW image and confirm it describes the command in hand: the
// signature, LUN and all 16 CB bytes must match, and the transfer length cast
// to 16 bits must equal `bufsize16` (what TinyUSB passed to the callback).
// Returns false if any of that fails.
bool sat_cbw_parse(const uint8_t *img, uint8_t lun, const uint8_t cdb[16],
                   uint16_t bufsize16, sat_cbw_t *out);

#endif
