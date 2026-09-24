// Restricted SAT policy. See sat_policy.h for the allowed set.
//
// Every check below is a refusal point. The function reads the CDB once into
// a local copy, validates the copy, and builds the task file from the same
// copy, so the bytes checked are the bytes that get sent to the drive.

#include "sat_policy.h"
#include <string.h>

#define ATA_IDENTIFY          0xEC
#define ATA_READ_SECTORS      0x20
#define ATA_READ_SECTORS_NR   0x21
#define ATA_READ_SECTORS_EXT  0x24

#define SAT_PROTO_PIO_IN      4

// Byte 2 of both CDB forms. OFF_LINE=0, CK_COND=0, T_TYPE=0, T_DIR=1,
// BYTE_BLOCK=1, T_LENGTH=2. Nothing else is accepted.
#define SAT_BYTE2_PIO_IN      0x0E

static sat_verdict_t refuse(uint8_t sk, uint8_t asc) {
    sat_verdict_t v = { false, sk, asc, 0x00 };
    return v;
}

sat_verdict_t sat_policy_check(const sat_input_t *in, sat_taskfile_t *tf) {
    if (tf) memset(tf, 0, sizeof(*tf));
    if (!in || !in->cdb || !tf)
        return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_OPCODE);

    uint8_t c[16];
    memcpy(c, in->cdb, sizeof(c));

    // ---- form and length ------------------------------------------------
    bool is16;
    if (c[0] == 0xA1 && in->cdb_len == 12) is16 = false;
    else if (c[0] == 0x85 && in->cdb_len == 16) is16 = true;
    else return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_OPCODE);

    // ---- map the register fields out of the form ------------------------
    uint8_t b1 = c[1], b2 = c[2];
    uint8_t feat, count, lba0, lba1, lba2, dev, cmd, ctrl;
    uint8_t hfeat = 0, hcount = 0, lba3 = 0, lba4 = 0, lba5 = 0;
    bool ext = false;
    if (!is16) {
        feat = c[3]; count = c[4];
        lba0 = c[5]; lba1 = c[6]; lba2 = c[7];
        dev  = c[8]; cmd  = c[9]; ctrl = c[11];
        if (b1 & 0x01) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);   // reserved
        if (c[10] != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);  // reserved
    } else {
        ext  = (b1 & 0x01) != 0;
        hfeat = c[3];  feat  = c[4];
        hcount = c[5]; count = c[6];
        lba3 = c[7];  lba0 = c[8];
        lba4 = c[9];  lba1 = c[10];
        lba5 = c[11]; lba2 = c[12];
        dev  = c[13]; cmd  = c[14]; ctrl = c[15];
    }

    // ---- fields that are the same for every allowed command -------------
    if ((b1 >> 5) != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);                  // MULTIPLE_COUNT
    if (((b1 >> 1) & 0x0F) != SAT_PROTO_PIO_IN) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD); // PROTOCOL
    if (b2 != SAT_BYTE2_PIO_IN) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
    if (ctrl != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
    if (feat != 0 || hfeat != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
    if (dev & 0x10) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);   // DEV: the bridge picks the device

    // ---- the closed command list ----------------------------------------
    uint64_t lba = 0;
    uint8_t tdev = 0;
    switch (cmd) {
    case ATA_IDENTIFY:
        if (ext) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (count != 1 || hcount != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (lba0 | lba1 | lba2 | lba3 | lba4 | lba5) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (dev & 0x0F) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        tdev = 0x00;
        break;

    case ATA_READ_SECTORS:
    case ATA_READ_SECTORS_NR:
        if (ext) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (hcount | lba3 | lba4 | lba5) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (!(dev & 0x40)) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);   // LBA addressing only
        if (count < 1 || count > SAT_MAX_SECTORS) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        lba = (uint64_t)lba0 | ((uint64_t)lba1 << 8) | ((uint64_t)lba2 << 16) | ((uint64_t)(dev & 0x0F) << 24);
        if (lba + count > (1ull << 28)) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_LBA_RANGE);
        tdev = (uint8_t)(0x40 | (dev & 0x0F));
        break;

    case ATA_READ_SECTORS_EXT:
        if (!is16 || !ext) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (hcount != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if ((dev & 0x4F) != 0x40) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);  // LBA=1, low nibble reserved
        if (count < 1 || count > SAT_MAX_SECTORS) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        lba = (uint64_t)lba0 | ((uint64_t)lba1 << 8) | ((uint64_t)lba2 << 16) |
              ((uint64_t)lba3 << 24) | ((uint64_t)lba4 << 32) | ((uint64_t)lba5 << 40);
        if (lba + count > (1ull << 48)) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_LBA_RANGE);
        tdev = 0x40;
        break;

    default:
        return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
    }

    // ---- the CBW must ask for exactly the data the CDB describes --------
    if (!in->dir_in) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
    if (in->xfer_len != (uint32_t)count * SAT_SECTOR_SIZE) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);

    // ---- state ----------------------------------------------------------
    if (!in->mounted) return refuse(SAT_SK_NOT_READY, SAT_ASC_NOT_READY);
    // An LBA read sent to a drive that was set up in CHS mode may not be
    // understood as LBA at all; an old CHS-only drive would read some other
    // sector and report success. Only IDENTIFY is allowed outside LBA mode.
    if (cmd != ATA_IDENTIFY && !in->lba_mode) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);

    // Allowed. The task file is written only now, so a refusal never leaves
    // anything in it but zeros.
    tf->command = cmd;
    tf->device = tdev;
    tf->count = count;
    tf->lba_low = lba0; tf->lba_mid = lba1; tf->lba_high = lba2;
    tf->ext = ext;
    if (ext) { tf->hob_lba_low = lba3; tf->hob_lba_mid = lba4; tf->hob_lba_high = lba5; }
    tf->sectors = count;

    sat_verdict_t ok = { true, 0, 0, 0 };
    return ok;
}

bool sat_cbw_parse(const uint8_t *img, uint8_t lun, const uint8_t cdb[16],
                   uint16_t bufsize16, sat_cbw_t *out) {
    if (!img || !cdb || !out) return false;
    memset(out, 0, sizeof(*out));
    // dCBWSignature "USBC", little-endian
    if (img[0] != 0x55 || img[1] != 0x53 || img[2] != 0x42 || img[3] != 0x43) return false;
    uint32_t len = (uint32_t)img[8] | ((uint32_t)img[9] << 8) |
                   ((uint32_t)img[10] << 16) | ((uint32_t)img[11] << 24);
    if (img[13] != lun) return false;                 // reserved bits 7..4 must be 0
    if (memcmp(img + 15, cdb, 16) != 0) return false;
    if ((uint16_t)len != bufsize16) return false;
    // bmCBWFlags: bit 7 is the direction, bits 6..0 are reserved. Anything
    // but exactly 0x80 counts as "not data-in", which the policy refuses.
    out->dir_in = (img[12] == 0x80);
    out->xfer_len = len;
    out->cb_len = img[14];                            // reserved bits kept, so they fail the length check
    return true;
}
