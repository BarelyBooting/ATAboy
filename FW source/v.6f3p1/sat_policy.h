// Restricted SAT (ATA PASS-THROUGH) policy for ATAboy: the allowlist.
//
// Pure code. No hardware, TinyUSB or Pico SDK dependencies, so the same file
// builds into the firmware and into the host test in test/.
//
// The policy is a closed list. A CDB is allowed only if it is one of the
// shapes below, byte for byte; everything else is refused before any ATA
// register is written.
//
//   ATA command                  CDB form             PROTO  count   other registers               IDENTIFY must show
//   0xEC IDENTIFY DEVICE         A1 or 85             4      1       all 0                         -
//   0x20 READ SECTORS            A1 or 85             4      1..8    LBA28                         -
//   0x21 READ SECTORS (NR)       A1 or 85             4      1..8    LBA28                         -
//   0x24 READ SECTORS EXT        85, EXTEND=1         4      1..8    LBA48                         83.10
//   0x40 READ VERIFY SECTORS     A1 or 85             3      0..255  LBA28 (count 0 = 256)         -
//   0xB0 SMART READ DATA *       A1 or 85, FEAT D0    4      1       LBA low 0, mid/high 4F/C2     82.0, 85.0
//   0xB0 SMART READ THRESHOLDS   A1 or 85, FEAT D1    4      1       LBA low 0, mid/high 4F/C2     82.0, 85.0
//   0xB0 SMART READ LOG          A1 or 85, FEAT D5    4      1..8    LBA low = log address, 4F/C2  82.0, 85.0, 84.0
//   0xB0 SMART RETURN STATUS *   A1 or 85, FEAT DA    3 CK   0       LBA low 0, mid/high 4F/C2     82.0, 85.0
//   0xF8 READ NATIVE MAX ADDRESS A1 or 85             3 CK   0       all 0                         82.10
//   0x27 READ NATIVE MAX EXT     85, EXTEND=1         3 CK   0       all 0                         82.10 and 83.10
//
// * Only in a build with ATABOY_SAT_SMART_SAVES=1 (below). The shipping build
//   refuses both rows, whatever the drive says: on ATA-3 drives they write.
//
// "A1 or 85" means the 16-byte form with EXTEND=0. "CK" means CK_COND must be
// 1: those commands exist to return registers, and CK_COND is how SAT hands
// registers back (sat.c, ATA Status Return descriptor). The last two are an
// extension to the design's table (API-DESIGN.md 2.3, rows N1 and N2): they
// report the drive's native size, so a Host Protected Area can be seen. SET MAX
// ADDRESS (0xF9, 0x37), DEVICE CONFIGURATION (0xB1), SECURITY and every write
// stay refused.
//
// Common to every allowed CDB: CB length exactly 12 (A1) or 16 (85),
// MULTIPLE_COUNT=0, CONTROL=0, reserved bytes 0, HOB FEATURES and HOB COUNT 0,
// HOB LBA bytes 0 unless the row is 48-bit, and FEATURES 0 except the SMART
// subcommand.
//
// Byte 2 depends on the protocol:
//   PIO data-in (4): exactly 0x0E. OFF_LINE=0, CK_COND=0, T_TYPE=0, T_DIR=1,
//     BYTE_BLOCK=1, T_LENGTH=2 (count field). The CBW must be data-in with a
//     transfer length of exactly count * 512.
//   Non-data (3): OFF_LINE=0, T_TYPE=0, T_LENGTH=0. T_DIR and BYTE_BLOCK only
//     describe a data phase, and there is none, so either value is accepted
//     (smartctl sends 0x2C, hdparm 0x20). CK_COND as in the table: required
//     where marked, 0 or 1 for READ VERIFY. The CBW transfer length must be 0;
//     its direction bit is ignored, as BOT says it is for a zero length.
//
// DEVICE byte: the DEV bit (4) must be 0; the bridge always talks to the drive
// it selected. Bits 7 and 5 are obsolete and ignored. IDENTIFY and SMART need
// bits 3..0 clear and ignore bit 6. READ SECTORS and READ VERIFY need LBA=1
// (bit 6) and carry LBA 27:24 in bits 3..0. READ SECTORS EXT and both READ
// NATIVE MAX commands need LBA=1 and bits 3..0 clear.
//
// LBA + count must not pass 2^28 (0x20, 0x21, 0x40) or 2^48 (0x24).
//
// State: the drive must be mounted (else NOT READY). Commands that carry a
// user-data address (0x20, 0x21, 0x24, 0x40) are allowed only when the drive
// was set up in LBA mode: sent to a drive that was set up in CHS mode, an LBA
// address may not be understood as one, and an old CHS-only drive would act on
// some other sector and report success. IDENTIFY, SMART and READ NATIVE MAX
// carry no user-data address, so they are allowed in CHS mode too.
//
// Drive capability (added 2026-09-23, review finding H1). SMART, READ NATIVE
// MAX and the 48-bit commands are optional in ATA, and on drives older than
// ATA-4 their opcodes may mean something else or nothing at all. Worse, a
// drive can implement SMART in a way that writes: the Conner CFS1275A saves
// its attribute values to non-volatile memory on every SMART READ DATA (D0).
// So those rows run only when the drive's own IDENTIFY DEVICE data, captured
// by the firmware during detection (ide.c), says the drive supports them:
//   - words 82..84 count only when bits 15:14 of word 83 AND of word 84 are
//     01b (their validity signature), and word 82 is neither 0000h nor FFFFh
//     (both mean "not reported"; an 83 or 84 of 0000h or FFFFh already fails
//     the signature);
//   - SMART rows: word 82 bit 0 (SMART feature set supported) AND word 85
//     bit 0 (SMART feature set enabled). A drive with SMART switched off
//     aborts every SMART command but ENABLE OPERATIONS, which is refused
//     here, so there is no point sending one. Seen on the ST380011A donor
//     (2026-09-24): w82 346Bh, w85 3468h, SMART supported but disabled, and
//     D1 and D5 came back ABRT (error 04h). Word 85 is valid only when bits
//     15:14 of word 87 are 01b, as words 82..84 are with 83 and 84: Linux
//     checks word 87 the same way before it reads word 85 (include/linux/
//     ata.h, ata_id_hpa_enabled(): "87 covers 85-87"; checked 2026-09-24),
//     and ATA/ATAPI-6 says so for words 85..87 (recalled, not re-read).
//     Word 85 is not tested for 0000h or FFFFh: 0000h is a real answer
//     (nothing enabled, so SMART refused), and a floating bus fails word
//     87's signature;
//   - SMART READ LOG additionally: word 84 bit 0 (SMART error logging
//     supported, ATA/ATAPI-5 and -6). The error log is read with SMART READ
//     LOG, so a drive that sets this bit implements the command. Word 84 bit 5
//     (General Purpose Logging) is not accepted instead: it covers READ LOG
//     EXT (2Fh), a different command;
//   - READ NATIVE MAX ADDRESS: word 82 bit 10 (Host Protected Area feature set);
//   - READ NATIVE MAX EXT: word 82 bit 10 and word 83 bit 10 (48-bit Address
//     feature set);
//   - READ SECTORS EXT: word 83 bit 10. Not a safety matter like SMART, but a
//     drive without 48-bit support would get an opcode it does not know.
// IDENTIFY, READ SECTORS and READ VERIFY are mandatory ATA commands and are
// not gated. When no IDENTIFY has been captured for the selected device since
// the last detection (id_captured false: nothing detected yet, a forced
// manual geometry after IDENTIFY failed, or a device change), every gated row
// is refused, 5/24/00 like every other refusal: the bridge will not send that
// CDB to this drive. The check comes after the mounted and LBA-mode checks,
// so an unmounted drive still reports NOT READY.
//
// SAT reads are NOT clipped to the configured geometry (unlike READ(10)).
// They address the drive, and the drive's own IDNF is what stops a read past
// its end; that failure is passed on, and nothing on the SAT path zero-fills
// or returns data the drive did not send. A geometry smaller than the drive
// therefore cannot hide sectors from a tool that sized itself by IDENTIFY.

#ifndef SAT_POLICY_H
#define SAT_POLICY_H

// SMART READ DATA (B0h/D0h) and SMART RETURN STATUS (B0h/DAh) write to the
// drive on the drives this bridge exists for. ATA-3 (X3T13/2008D rev 7b,
// 7.31.5 and 7.31.6) says the drive "saves any updated attribute values to
// non-volatile memory" before it answers either one; ATA-5 dropped that
// wording, but a drive built to ATA-3 still does it (Conner's CFS1275A manual
// says so too). The words 82..84 gate cannot tell such a drive from a newer
// one. So both are refused unless the firmware is built with
// ATABOY_SAT_SMART_SAVES=1, which a build used on archaeology specimens must
// never be. READ THRESHOLDS and READ LOG have no such wording and stay.
#ifndef ATABOY_SAT_SMART_SAVES
#define ATABOY_SAT_SMART_SAVES 0
#endif

#include <stdint.h>
#include <stdbool.h>

// One command's data must fit the MSC buffer (4096 bytes, tusb_config.h).
#define SAT_SECTOR_SIZE   512u
#define SAT_MAX_SECTORS   8u

// SAT PROTOCOL field values this policy can hand to the executor.
#define SAT_PROTO_NON_DATA  3
#define SAT_PROTO_PIO_IN    4

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
    // The drive's own IDENTIFY DEVICE words 82, 83 and 84 (command sets and
    // features supported), as the firmware captured them. Meaningless unless
    // id_captured.
    bool     id_captured;
    uint16_t id_w82, id_w83, id_w84;
    // Words 85 (command sets and features enabled) and 87 (bits 15:14 = 01b
    // when words 85..87 are valid), from the same IDENTIFY.
    uint16_t id_w85, id_w87;
} sat_input_t;

// The ATA task file to issue for an allowed CDB. `device` never carries the
// DEV bit or the obsolete bits 7 and 5; the executor adds the selected device.
typedef struct {
    uint8_t  command;
    uint8_t  feature, count, lba_low, lba_mid, lba_high;
    uint8_t  hob_feature, hob_count, hob_lba_low, hob_lba_mid, hob_lba_high;
    uint8_t  device;          // 0x00 (IDENTIFY, SMART) or 0x40 | LBA 27:24
    bool     ext;             // write the HOB registers (48-bit command)
    uint8_t  protocol;        // SAT_PROTO_PIO_IN or SAT_PROTO_NON_DATA
    bool     ck_cond;         // return the ATA registers even on success
    uint8_t  sectors;         // 512-byte PIO data-in blocks; 0 for non-data
} sat_taskfile_t;

typedef struct {
    bool     allow;
    uint8_t  sense_key, asc, ascq;   // valid when !allow
    // Allowed because of the drive's IDENTIFY words: the caller must check the
    // drive is still the one they came from before sending (sat.c). Set
    // exactly when the row needed a capability; false on every refusal.
    bool     needs_identity;
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
